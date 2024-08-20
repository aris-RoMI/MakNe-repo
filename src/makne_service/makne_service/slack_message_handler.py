import json
import requests
import time
import threading
import subprocess
import os, re
from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from makne_msgs.srv import SetPointList
from library.Constants import SlackConstants, CommandConstants, DBConstants
from makne_db.db_manager import DBManager


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

        self.point_client = self.create_client(SetPointList, '/set_pointlist')        
        while not self.point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

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
        
    def send_point_to_service(self, command_type, channel_id, id_list):
        request = SetPointList.Request()
        request.command_type = command_type  
        request.point_list = id_list  # 주문자 정보 리스트를 넘김

        future = self.point_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_service_response(f, channel_id))
        
    def handle_service_response(self, future, channel_id):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Service call succeeded: {response.message}")
                slack_message = f":white_check_mark: Service succeeded: {response.message}"
            else:
                self.get_logger().warn(f"Service call failed: {response.message}")
                slack_message = f":x: Service failed: {response.message}"
            
            # Slack 채널에 메시지를 보냅니다.
            self.send_slack_message(channel_id, slack_message) 
            
        except Exception as e:
            error_message = f"Service call failed: {str(e)}"
            self.get_logger().error(error_message)
            self.send_slack_message(channel_id, f":x: {error_message}") 

        
        
    def setup_routes(self):
        """Flask 서버의 라우트를 설정합니다."""
        @self.app.route('/slack/events', methods=['POST'])
        @self.app.route('/slack/interactive-endpoint', methods=['POST'])
        def slack_events():
            if request.content_type == 'application/x-www-form-urlencoded':
                payload = json.loads(request.form['payload'])
                # callback_id = payload.get('callback_id')
                actions = payload.get('actions', [])
                # # 호출 취소 버튼이 눌렸을 때의 동작
                # if callback_id == 'cancel':
                #     actions = payload.get('actions')
                #     if actions and actions[0]['value'] == 'cancel_call':
                #         user_name = payload['user']['name']
                #         channel_id = payload['channel']['id']
                #         response_url = payload['response_url']

                #         # 호출 취소 처리
                #         self.cancel_call(channel_id, user_name, response_url)
                if actions:
                    action_id = actions[0].get('action_id')
                    # block_id = actions[0].get('block_id')
                    # selected_value = payload['state']['values']['select_location_block']['select_location']['selected_option']['value']
                    user_name = payload['user']['name']
                    channel_id = payload['channel']['id']
                    channel_name = self.fetch_channel_name(channel_id)
                    response_url = payload['response_url']
                    
                    if action_id == "cancel_call_action" or actions[0]['value'] == "cancel_call":
                        user_name = payload['user']['name']
                        channel_id = payload['channel']['id']
                        response_url = payload['response_url']

                        # 호출 취소 처리
                        self.cancel_call(channel_id, user_name, response_url)

                    elif action_id == 'confirm_call_action':
                        # 로봇 호출 서비스 요청
                        # self.send_point_to_service(CommandConstants.CALL_ROBOT, channel_id, [selected_value])
                        response_message = f"Order has been successfully received by {user_name} in channel {channel_name}."

                        # 호출 완료 메시지와 호출 취소 버튼 전송
                        response = {
                            "response_type": "in_channel",
                            "replace_original": "true",  # 기존 메시지를 업데이트
                            "text": response_message,
                            "attachments": [
                                {
                                    "text": "You can cancel the command if necessary:",
                                    "fallback": "You are unable to cancel the command",
                                    "callback_id": "cancel",
                                    "color": "#FF0000",  # Red color for the cancel button
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
                        requests.post(response_url, json=response)

            return '', 200


        @self.app.route('/slack/commands', methods=['POST'])
        def slack_commands():
            data = request.form
            command = data.get('command')
            text = data.get('text')
            user_id = data.get('user_id')
            channel_id = data.get('channel_id')
            
            # /order 커맨드 처리
            if command == SlackConstants.SEND_ROBOT:
                user_name = self.fetch_user_name(user_id)
                channel_name = self.fetch_channel_name(channel_id)

                # 멘션된 사용자들의 slack_name 추출
                mentioned_user_names = re.findall(r"@(\w+)", text)
                
                # 작성자의 slack_name도 리스트에 추가
                slack_name_list = mentioned_user_names + [user_name]

                response_message = f"Order has been successfully received by {user_name} in channel {channel_name}."

                print(f"Order List: {slack_name_list}")
                
                # self.send_point_to_service(CommandConstants.ORDER, channel_id, slack_name_list)

                return jsonify({
                    "response_type": "in_channel",
                    "text": response_message,
                    "attachments": [
                        {
                            "text": "You can cancel the command if necessary:",
                            "fallback": "You are unable to cancel the command",
                            "callback_id": "cancel",
                            "color": "#FF0000",  # Red color for the cancel button
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
                
            # /call_robot 커맨드 처리
            elif command == SlackConstants.CALL_ROBOT:
                user_name = self.fetch_user_name(user_id)
                channel_name = self.fetch_channel_name(channel_id)

                # 데이터베이스에서 위치 이름을 가져옴
                location_names = self.db_manager.get_column_data("LocationInfo", "location_name")
                locations = [
                    {"text": {"type": "plain_text", "text": name}, "value": name}
                    for name in location_names
                ]

                return jsonify({
                    "response_type": "in_channel",
                    "text": f"{user_name}님이 {channel_name} 채널에서 로봇을 호출합니다.",
                    "blocks": [
                        {
                            "type": "section",
                            "block_id": "select_location_block",
                            "text": {
                                "type": "mrkdwn",
                                "text": "위치를 선택하세요:"
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
                            "elements": [
                                {
                                    "type": "button",
                                    "text": {
                                        "type": "plain_text",
                                        "text": "확인"
                                    },
                                    "style": "primary",
                                    "value": "confirm_location",
                                    "action_id": "confirm_call_action"
                                },
                                {
                                    "type": "button",
                                    "text": {
                                        "type": "plain_text",
                                        "text": "취소"
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
                
            elif command == SlackConstants.FOLLOW:
                user_name = self.fetch_user_name(user_id)
                channel_name = self.fetch_channel_name(channel_id)
                response_message = f"Follow Command has been successfully received by {user_name} in channel {channel_name}."
                # self.send_point_to_service(CommandConstants.FOLLOW, channel_id, [])
                
                return jsonify({
                    "response_type": "in_channel",
                    "text": response_message,
                    "attachments": [
                        {
                            "text": "You can cancel the command if necessary:",
                            "fallback": "You are unable to cancel the command",
                            "callback_id": "cancel",
                            "color": "#FF0000",  # Red color for the cancel button
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
        
    def cancel_order(self, channel_id, user_name):
        """주문 취소 작업을 처리하는 메소드."""
        print(f"Order canceled by {user_name} in channel {channel_id}")
        # Slack에 취소 메시지 전송
        self.send_slack_message(channel_id, f":x: The order has been canceled by {user_name}.")
        # self.send_point_to_service(CommandConstants.CANCEL, channel_id, [])
        
    def cancel_call(self, channel_id, user_name, response_url):
        """로봇 호출 취소 작업을 처리하고 메시지를 업데이트하는 메소드."""
        print(f"로봇 호출이 {user_name}에 의해 채널 {channel_id}에서 취소되었습니다.")
        
        # Slack에 취소 메시지 전송 및 기존 메시지 업데이트
        updated_message = {
            "response_type": "in_channel",
            "text": f":x: {user_name}님이 호출을 취소했습니다."
        }
        
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