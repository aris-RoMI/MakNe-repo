import json
import requests
import time
import threading
import subprocess
import os, re
from io import BytesIO
from PIL import Image
from flask import Flask, request, jsonify
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
import logging
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from makne_msgs.srv import SetPointList, GetStatus, BoolSignal
from library.Constants import SlackConstants, CommandConstants, DBConstants, RobotStatus
from makne_db.db_manager import DBManager
from makne_service.map_editor import MapEditor


class NgrokManager:
    def __init__(self, port, ngrok_url, static_domain):
        self.port = port
        self.ngrok_url = ngrok_url
        self.static_domain = static_domain

    def start_ngrok(self):
        """ngrok을 실행하고, 공개 URL을 반환합니다."""
        ngrok_process = subprocess.Popen(['ngrok', 'http', self.static_domain, \
            str(self.port)], stdout=subprocess.PIPE)
        time.sleep(5)  # ngrok이 시작될 때까지 잠시 대기
        url = f"{self.ngrok_url}/api/tunnels"
        
        try:
            response = requests.get(url)
            data = response.json()
            
            # 첫 번째 터널의 public_url을 반환합니다.
            if data['tunnels']:
                public_url = data['tunnels'][0]['public_url']
                return public_url
            else:
                print("No tunnels found")
                return None
        except requests.exceptions.RequestException as e:
            print(f"Error fetching ngrok URL: {e}")
            return None

class SlackMessageHandler(Node):
    def __init__(self, token, port, ngrok_url, static_domain):
        super().__init__("message_handler")
        self.token = token
        self.port = port
        self.ngrok_manager = NgrokManager(port, ngrok_url, static_domain)
        self.app = Flask(__name__)
        self.db_manager = DBManager(DBConstants.DB_NAME)
        self.map_editor = MapEditor()
        self.client = WebClient(token=token)
        self.logger = logging.getLogger(__name__)

        self.point_client = self.create_client(SetPointList, '/set_pointlist')
        self.status_client = self.create_client(GetStatus, '/get_status')
        self.auto_timer_client = self.create_client(BoolSignal, "/auto_timer_signal")
        self.complete_task_server = self.create_service(BoolSignal, "/complete_task_signal", self.complete_task_callback)
        self.auto_return_server = self.create_service(BoolSignal, "/auto_return_signal", self.auto_return_callback)
        
        # 서비스 서버와 연결되기 전까지 대기
        while not self.point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.last_response_url = ""

        self.setup_routes()

    def fetch_channel_name(self, channel_id):
        """채널 ID를 사용하여 채널 이름을 가져옵니다."""
        url = 'https://slack.com/api/conversations.info'
        headers = {
            'Authorization': f'Bearer {self.token}',
        }
        params = {
            'channel': channel_id,
        }
        response = requests.get(url, headers=headers, params=params)
        data = response.json()
        if data.get('ok'):
            return data['channel'].get('name', 'Unknown channel')
        else:
            return 'Unknown channel'

    def fetch_user_name(self, user_id):
        """사용자 ID를 사용하여 사용자 이름을 가져옵니다."""
        url = 'https://slack.com/api/users.info'
        headers = {
            'Authorization': f'Bearer {self.token}',
        }
        params = {
            'user': user_id,
        }
        response = requests.get(url, headers=headers, params=params)
        data = response.json()
        if data.get('ok'):
            return data['user'].get('name', 'Unknown user')
        else:
            return 'Unknown user'
        
    def send_slack_message(self, channel, text):
        """Slack 채널에 메시지를 보냅니다."""
        url = 'https://slack.com/api/chat.postMessage'
        headers = {
            'Authorization': f'Bearer {self.token}',
            'Content-Type': 'application/json',
        }
        data = {
            'channel': channel,
            'text': text,
        }
        response = requests.post(url, headers=headers, json=data)
        if not response.ok:
            self.get_logger().error(f"Failed to send message to Slack: {response.text}")
            
    def find_interactive_message_ts(self, channel_id, search_text):
        """특정 텍스트를 포함한 인터랙티브 메시지의 타임스탬프를 검색"""
        url = "https://slack.com/api/conversations.history"
        headers = {
            "Authorization": f"Bearer {self.token}"
        }
        params = {
            "channel": channel_id,
            "limit": 100  # 한 번에 최대 100개의 메시지 가져오기
        }

        response = requests.get(url, headers=headers, params=params)
        response_data = response.json()
        

        if response_data['ok']:
            for message in response_data['messages']:
                # 메시지 본문에 검색 텍스트가 포함되어 있는지 확인
                if search_text in message.get('text', ''):
                    # 메시지에 첨부 파일(attachments)이 있는지 확인
                    attachments = message.get('attachments', [])
                    for attachment in attachments:
                        # 첨부 파일의 텍스트가 특정 문자열을 포함하는지 확인
                        if search_text in attachment.get('text', ''):
                            return message['ts']  # 조건에 맞는 메시지의 타임스탬프 반환

        print("Interactive message not found or failed to fetch history.")
        return None

    def upload_image_to_slack(self, channel_id, image_bytes, message=""):
        """Slack에 이미지를 업로드하고 메시지를 전송합니다."""
        try:
            # 이미지 파일을 메모리에 저장하고, 파일처럼 사용
            image_bytes.seek(0)  # 파일의 시작으로 이동
            result = self.client.files_upload_v2(
                channel = channel_id,
                initial_comment = message,
                file = image_bytes,
                filename = "current_Makne_location_image.png",
                title = "Makne Location Image",
            )
            # 결과를 로그에 기록
            self.logger.info(result)
            return jsonify({
                "response_type": "ephemeral",
                "text": "Image uploaded successfully!",
                "attachments": [
                    {
                        "title": "Uploaded Image",
                        "image_url": result["file"]["permalink"]
                    }
                ]
            })

        except SlackApiError as e:
            self.logger.error(f"Error uploading file: {e}")
            return jsonify({
                "response_type": "ephemeral",
                "text": f"Failed to upload image: {e.response['error']}"
            })


        
    def send_point(self, command_type, channel_id, user_name, id_list):
        """RobotManager로 이동해야할 목표정보를 전송합니다."""
        request = SetPointList.Request()
        request.command_type = command_type
        request.user_name = user_name
        request.point_list = id_list  # 주문자 정보 리스트를 넘김

        future = self.point_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_set_point_response(f, channel_id))
        
        
    def handle_set_point_response(self, future, channel_id):
        """목표정보 전송 결과를 표시합니다."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
                slack_message = f":white_check_mark: Service succeeded: {response.message}"
            else:
                self.get_logger().warn(f"Service call failed: {response.message}")
                slack_message = f":x: Service failed: {response.message}"
            
            # Slack 채널에 메시지를 보냅니다.
            # self.send_slack_message(channel_id, slack_message)
            
        except Exception as e:
            error_message = f"Service call failed: {str(e)}"
            self.get_logger().error(error_message)
            # self.send_slack_message(channel_id, f":x: {error_message}")

    def send_status_request(self, channel_id):
        """현재 로봇을 사용하고 있는 user 정보와 담당업무, 남은 시간 등을 요청합니다."""
        request = GetStatus.Request()
        request.command = True

        # 동기식 호출
        response = self.status_client.call(request)

        if response is not None:
            return {
                "current_user": response.current_user,
                "current_task": RobotStatus.STATUS_LIST[response.current_task],
                "remain_time": response.remain_time,
                "current_pose": response.current_pose,
                "current_path": response.current_path,
                "remain_battery": response.remain_battery
            }
        else:
            return {
                "current_user": "Error",
                "current_task": "Error",
                "remain_time": "N/A",
                "current_pose": "Error",
                "current_path": "Error",
                "remain_battery": "Error"
            }
            
    def process_being_used(self, current_user, current_task, remain_time):
        """이미 다른 유저가 로봇을 사용중일 때 알림창을 띄웁니다."""
        return jsonify({
                "response_type" : "ephemeral",
                "text": f"Robot is currently on {current_task} task by {current_user}. Please try again {remain_time}(s) later."
            })
        
    def process_get_state(self, current_user, current_task, remain_time, robot_pose, robot_path, remain_battery):
        """로봇 상태를 이미지로 표현하고, 그 이미지를 Slack에 업로드하면서 메시지도 함께 전송합니다."""
        if current_user == "":
                current_user = "None"
                
        # 로봇 상태 이미지를 생성
        buffer = self.map_editor.overlay_robot_state_on_image(robot_pose, robot_path)

        # 이미지를 메모리에 저장
        image = Image.open(BytesIO(buffer))

        # 이미지를 메모리에 저장 (BytesIO 객체로 변환)
        image_bytes = BytesIO()
        image.save(image_bytes, format='PNG')
        image_bytes.seek(0)
        # 이미지와 함께 보낼 메시지 설정
        message = f"current_user : {current_user}, current_task : {current_task}, remain_time : {remain_time}\ncurrent_battery : {remain_battery}%"

        # 이미지를 Slack에 업로드하면서 메시지를 포함
        return self.upload_image_to_slack(self.current_channel_id, image_bytes, message)
        
    
    def process_send_robot(self):
        """send_robot 명령에 대한 결과 표시 목적지 정보 및 호출, 취소 버튼 포함"""
        user_name = self.fetch_user_name(self.current_user_id)
        channel_name = self.fetch_channel_name(self.current_channel_id)

        mentioned_user_names = re.findall(r"@(\w+)", self.current_text)

        print(f"Waypoint List: {mentioned_user_names}")
        
        return jsonify({
            "response_type": "ephemeral",
            "text": f"{user_name} is sending the robot in the {channel_name} channel.",
            "blocks": [
                {
                    "type": "section",
                    "block_id": "select_location_block",
                    "text": {
                        "type": "mrkdwn",
                        "text": f"Current selected location list : {mentioned_user_names}\nAre you sure you want to proceed with this list?"
                    }
                },
                {
                    "type": "actions",
                    "elements": [
                        {
                            "type": "button",
                            "text": {
                                "type": "plain_text",
                                "text": "Confirm"
                            },
                            "style": "primary",
                            "value": json.dumps(mentioned_user_names),
                            "action_id": "confirm_send_action"
                        },
                        {
                            "type": "button",
                            "text": {
                                "type": "plain_text",
                                "text": "Cancel"
                            },
                            "style": "danger",
                            "value": "cancel_send",
                            "action_id": "cancel_send_action"
                        }
                    ]
                }
            ],
            "callback_id": "send_robot_callback",
            "selected_list": mentioned_user_names
        })

    def process_call_robot(self):
        """call_robot 명령에 대한 결과 표시 목적지 정보 및 호출, 취소 버튼 포함"""
        user_name = self.fetch_user_name(self.current_user_id)
        channel_name = self.fetch_channel_name(self.current_channel_id)

        location_names = self.db_manager.get_column_data("LocationInfo", "location_name")
        locations = [
            {"text": {"type": "plain_text", "text": name}, "value": name}
            for name in location_names
        ]
        return jsonify({
    "response_type": "ephemeral",
    "text": f"{user_name} is calling the robot in the {channel_name} channel.",
    "blocks": [
        {
            "type": "section",
            "block_id": "select_location_block",
            "text": {
                "type": "mrkdwn",
                "text": "Select Location and Confirm:"
            },
            "accessory": {
                "type": "static_select",
                "placeholder": {
                    "type": "plain_text",
                    "text": location_names[0]
                },
                "options": locations,
                "action_id": "select_location"
            }
        },
        {
            "type": "actions",
            "block_id": "actions_block",
            "elements": [
                {
                    "type": "button",
                    "text": {
                        "type": "plain_text",
                        "text": "Confirm"
                    },
                    "style": "primary",
                    "value": "confirm_location",
                    "action_id": "confirm_call_action"
                },
                {
                    "type": "button",
                    "text": {
                        "type": "plain_text",
                        "text": "Cancel"
                    },
                    "style": "danger",
                    "value": "cancel_call",
                    "action_id": "cancel_call_action"
                }
            ]
        }
    ],
    "callback_id": "call_robot_callback"
})
        
    def process_follow(self):
        """/follow 명령에 대한 결과 표시 취소 버튼 포함"""
        user_name = self.fetch_user_name(self.current_user_id)
        channel_name = self.fetch_channel_name(self.current_channel_id)
        response_message = f"Request successfully received by {user_name} in channel {channel_name}."

        self.send_point(CommandConstants.FOLLOW, self.current_channel_id, user_name, [])
        return jsonify({
            "response_type": "ephemeral",
            "text": response_message,
            "attachments": [
                {
                    "text": "You can cancel the command if necessary:",
                    "fallback": "You are unable to cancel the command",
                    "callback_id": "cancel",
                    "color": "#FF0000",
                    "attachment_type": "default",
                    "actions": [
                        {
                            "name": "action",
                            "text": "Cancel",
                            "type": "button",
                            "value": "cancel_call",
                            "style": "danger"
                        }
                    ]
                }
            ]
        })        
        
    def confirm_additional_request(self, channel_name, user_name, response_url):
        """추가적인 명령에 대한 안내창 띄움"""
        request= BoolSignal.Request()
        request.complete_signal = True
        response = self.auto_timer_client.call(request)
        
        if response:
            # Slack에 추가적인 명령을 입력하라는 안내문과 30초 뒤에 자동으로 복귀한다는 내용
            updated_message = {
                "response_type": "ephemeral",
                "text": f":white_check_mark: Additional command confirm. Please retype command you want.\n:warning: If you don't type any command the robot will automatically return to base.\nCurrent User : {user_name}, Channel_name : {channel_name}"
            }
            
            # 기존 메시지를 업데이트하여 취소됨을 알리는 메시지로 교체
            requests.post(response_url, json=updated_message)
            
        
            
    def setup_routes(self):
        """Flask 서버의 라우트를 설정합니다."""
        @self.app.route('/slack/events', methods=['POST'])
        @self.app.route('/slack/interactive-endpoint', methods=['POST'])
        
        # interactive 버튼 입력 등의 이벤트 발생했을 경우 처리
        def slack_events():    
            if request.content_type == 'application/x-www-form-urlencoded':
                payload = json.loads(request.form['payload'])
                callback_id = payload.get('callback_id')
                actions = payload.get('actions', [])
                
                # 구동중인 로봇에 취소 명령을 내리는 부분
                if callback_id == 'cancel':
                    actions = payload.get('actions')
                    if actions and actions[0]['value'] == 'cancel_call':
                        user_name = payload['user']['name']
                        channel_id = payload['channel']['id']
                        channel_name = self.fetch_channel_name(channel_id)
                        self.last_response_url = payload['response_url']

                        self.cancel_service(channel_id, user_name, self.last_response_url, service_callback = True)
                        
                    elif actions and actions[0]['value'] == 'cancel_send':
                        user_name = payload['user']['name']
                        channel_id = payload['channel']['id']
                        channel_name = self.fetch_channel_name(channel_id)
                        self.last_response_url = payload['response_url']

                        self.cancel_service(channel_id, user_name, self.last_response_url, service_callback = True)
                        
                if actions:
                    selected_value = actions[0].get('value')
                    action_id = actions[0].get('action_id')
                    user_name = payload['user']['name']
                    channel_id = payload['channel']['id']
                    channel_name = self.fetch_channel_name(channel_id)
                    self.last_response_url = payload['response_url']
                    
                    # 기존 작업이 끝나고 추가적인 명령을 내리겠다고 했을 경우
                    if selected_value == "yes":
                        self.confirm_additional_request(channel_name, user_name, self.last_response_url)
                    
                    # 기존 작업이 끝나고 추가적인 명령을 내리지 않겠다고 했을 경우
                    elif selected_value == "no":
                        self.return_to_base(channel_id, user_name, self.last_response_url)
                    
                    # 단순히 창을 닫고 명령을 끝내는 부분(call)
                    elif action_id == "cancel_call_action":
                        self.cancel_service(channel_id, user_name, self.last_response_url, service_callback = False)
                        
                    # 단순히 창을 닫고 명령을 끝내는 부분(send)
                    elif action_id == "cancel_send_action":
                        self.cancel_service(channel_id, user_name, self.last_response_url, service_callback = False)
                        
                    # robot_call 명령창에서 보낼 곳을 선택한 뒤 로봇에 보내는 부분
                    elif action_id == 'confirm_call_action':
                        selected_option = payload['state']['values']['select_location_block']['select_location']['selected_option']
                        if selected_option is None:
                            selected_location = "worker_1"
                        else:
                            selected_location = selected_option["text"]["text"]

                        self.send_point(CommandConstants.CALL_ROBOT, channel_id, user_name, [selected_location])
                        response_message = f"Request successfully received by {user_name} in channel {channel_name} command : Call.\nGoal :{selected_location}"

                        response = {
                            "response_type": "ephemeral",
                            "replace_original": "true",  
                            "text": response_message,
                            "attachments": [
                                {
                                    "text": "You can cancel the command if necessary:",
                                    "fallback": "You are unable to cancel the command",
                                    "callback_id": "cancel",
                                    "color": "#FF0000",  
                                    "attachment_type": "default",
                                    "actions": [
                                        {
                                            "name": "action",
                                            "text": "Cancel",
                                            "type": "button",
                                            "value": "cancel_call",
                                            "style": "danger"
                                        }
                                    ]
                                }
                            ]
                        }

                        # Slack에 메시지 업데이트 요청
                        requests.post(self.last_response_url, json=response)
                        
                    elif action_id == "confirm_send_action":
                        """send_robot 실행 결과 표시 취소 버튼 포함"""
                        slack_name_list = json.loads(payload['actions'][0]['value'])
                        self.send_point(CommandConstants.SEND_ROBOT, self.current_channel_id, user_name, slack_name_list)
                        
                        response_message = f"Request successfully received by {user_name} in channel {channel_name} command : Call.\nGoal :{slack_name_list}"
                        response = {
                            "response_type": "ephemeral",
                            "replace_original": "true",  
                            "text": response_message,
                            "attachments": [
                                {
                                    "text": "You can cancel the command if necessary:",
                                    "fallback": "You are unable to cancel the command",
                                    "callback_id": "cancel",
                                    "color": "#FF0000",  
                                    "attachment_type": "default",
                                    "actions": [
                                        {
                                            "name": "action",
                                            "text": "Cancel",
                                            "type": "button",
                                            "value": "cancel_send",
                                            "style": "danger"
                                        }
                                    ]
                                }
                            ]
                        }
                        requests.post(self.last_response_url, json=response)

            return '', 200


        @self.app.route('/slack/commands', methods=['POST'])
        # slash command에 대한 처리
        def slack_commands():
            data = request.form
            command = data.get('command')
            text = data.get('text')
            user_id = data.get('user_id')
            channel_id = data.get('channel_id')
            user_name = self.fetch_user_name(user_id)
            
            # 명령 정보 저장
            self.current_command = command
            self.current_text = text
            self.current_user_id = user_id
            self.current_channel_id = channel_id
            self.last_response_url = data.get('response_url')
            
            # 현재 로봇 상태를 확인하기 위해 상태 요청을 보냄
            status_response = self.send_status_request(channel_id)
            # 상태에 따른 명령어 처리 
            if command == SlackConstants.GET_STATUS:
                # current_pose를 (x, y) 형태의 튜플로 변환
                current_pose_tuple = (
                    status_response["current_pose"].position.x,
                    status_response["current_pose"].position.y
                )

                # current_path를 (x, y) 형태의 튜플 리스트로 변환
                current_path_list = [
                    (pose.pose.position.x, pose.pose.position.y)
                    for pose in status_response["current_path"].poses
                ]

                response = self.process_get_state(
                    status_response['current_user'], 
                    status_response['current_task'], 
                    status_response['remain_time'],
                    current_pose_tuple,  # (x, y) 형태의 현재 위치
                    current_path_list,    # (x, y) 형태의 경로 리스트
                    status_response["remain_battery"]
                )
                            
            # 로봇을 이용하는 사람이 없을 경우 명령 하달
            elif status_response.get('current_user') == "" or status_response.get('current_user') == user_name:
                if command == SlackConstants.SEND_ROBOT:
                    response = self.process_send_robot()
                elif command == SlackConstants.CALL_ROBOT:
                    response = self.process_call_robot()
                elif command == SlackConstants.FOLLOW:
                    response = self.process_follow()
                else:
                # 잘못된 명령어 처리 (Unknown Command)
                    response = jsonify({
                        "response_type": "ephemeral",
                        "text": "Unknown Command"
                    })
            
            # 로봇을 이용하는 사람이 있는 경우
            else:
                response = self.process_being_used(status_response['current_user'], 
                                                status_response['current_task'], 
                                                status_response['remain_time'])

            return response
    # service_callback은 로봇 구동 중에 명령을 취소한 것과 단순히 창을 닫는 동작을 구분하기 위한 변수(bool)
    def cancel_service(self, channel_id, user_name, response_url, service_callback):
        """로봇 호출 취소 작업을 처리하고 메시지를 업데이트하는 메소드."""
        # Slack에 취소 메시지 전송 및 기존 메시지 업데이트
        channel_name = self.fetch_channel_name(channel_id)
        updated_message = {
            "response_type": "ephemeral",
            "text": f":x: Request was canceled by {user_name} in channel {channel_name}."
        }
        if service_callback:
            self.send_point(CommandConstants.CANCEL, channel_id, user_name, [])
        
        # 기존 메시지를 업데이트하여 취소됨을 알리는 메시지로 교체
        requests.post(response_url, json=updated_message)
        
    def return_to_base(self, channel_id, user_name, response_url):
        """로봇 호출 취소 작업을 처리하고 메시지를 업데이트하는 메소드."""
        # Slack에 취소 메시지 전송 및 기존 메시지 업데이트
        channel_name = self.fetch_channel_name(channel_id)
        updated_message = {
            "response_type": "ephemeral",
            "text": f":white_check_mark: Request is over.\nHave a nice day!! {user_name}."
        }
        self.send_point(CommandConstants.RETURN, channel_id, user_name, [])
        
        # 기존 메시지를 업데이트하여 취소됨을 알리는 메시지로 교체
        requests.post(response_url, json=updated_message)

    def start_flask(self):
        """Flask 서버를 실행합니다."""
        self.app.run(port=self.port)

    def start(self):
        """Flask 서버를 먼저 시작하고 ngrok을 실행합니다."""
        # Flask 서버를 별도의 쓰레드에서 시작
        print("Flask activated")
        flask_thread = threading.Thread(target=self.start_flask)
        flask_thread.start()

        # Flask 서버가 시작될 때까지 잠시 대기
        time.sleep(3)

        ngrok_url = self.ngrok_manager.start_ngrok()
        if ngrok_url:
            print(f'ngrok URL: {ngrok_url}/slack/events')
            print(f'ngrok URL: {ngrok_url}/slack/commands')  # 명령어 엔드포인트 URL 출력
        else:
            print('Failed to start ngrok.')
    
    def auto_return_callback(self, request, response):
        """일정 시간 동안 추가적인 명령을 내리지 않았을 경우"""
        auto_return_message = {
            "response_type": "ephemeral",
            "replace_original": "true", 
            "text": "The allotted time has elapsed, the robot is returning to the base."  
        }
        requests.post(self.last_response_url, json=auto_return_message)
        response.success = True
        
        return response
            
    def complete_task_callback(self, request, response):
        """작업이 완료되었다는 메시지를 Slack에 보내기 위한 콜백 함수"""
        
        # 완료된 작업에 대한 Slack 메시지 구성
        completion_message = {
            "response_type": "ephemeral",
            "replace_original": "true",  # 기존 메시지를 덮어씁니다.
            "text": "Task completed. Would you like to issue another command?",
            "attachments": [
                {
                    "text": "Please select an option:",
                    "fallback": "You are unable to choose an option",
                    "callback_id": "task_complete_callback",
                    "color": "#3AA3E3",
                    "attachment_type": "default",
                    "actions": [
                        {
                            "name": "yes",
                            "text": "Yes",
                            "type": "button",
                            "value": "yes",
                            "style": "primary"
                        },
                        {
                            "name": "no",
                            "text": "No",
                            "type": "button",
                            "value": "no",
                            "style": "danger"
                        }
                    ]
                }
            ]
        }
        
        # 슬랙에서 봇이 마지막으로 띄운 창의 url 정보를 이용
        response_post = requests.post(self.last_response_url, json=completion_message)

        response.success = True
        
        return response
            
            
def main():
    config_path = os.path.join(get_package_share_directory("makne_service"), "config", SlackConstants.SLACK_CONFIG_FILE)
    with open(config_path) as config_file:
        config = json.load(config_file)

    slack_token = config['slack_token']
    ngrok_url = config['ngrok_url']
    static_domain = config["static_domain"]
    port = 5000

    # ROS 2 노드 초기화
    rclpy.init()

    # SlackMessageHandler 생성
    slack_message_handler = SlackMessageHandler(slack_token, port, ngrok_url, static_domain)

    try:
        # Flask 서버 시작
        slack_message_handler.start()

        # ROS 2 이벤트 루프 실행
        rclpy.spin(slack_message_handler)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()