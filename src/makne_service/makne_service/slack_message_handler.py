import json
import requests
import time
import threading
import subprocess
import os, re, sys
from flask import Flask, request, jsonify
from threading import Thread
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from makne_service.waypoint_calculator import WayPointCalculator
from makne_service.robot_manager import RobotManager
from makne_msgs.msg import Pose2D
from makne_msgs.srv import SetWaypoint
from library.Constants import DBConstants, SlackConstants, CommandConstants


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
        self.waypoint_calculator = WayPointCalculator(DBConstants.DB_NAME)

        self.waypoint_client = self.create_client(SetWaypoint, '/set_waypoint')        
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
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
        
    def send_waypoint_to_service(self, command_type, channel_id, waypoints):
        request = SetWaypoint.Request()
        request.command_type = command_type  # 예시: 1은 특정 command_type을 나타냄

        # Pose2D 객체 리스트 생성
        pose_list = []
        for wp in waypoints:
            pose = Pose2D()
            pose.x = wp[0]
            pose.y = wp[1]
            pose_list.append(pose)

        request.waypoints = pose_list  # Pose2D 리스트를 request.waypoints에 할당

        future = self.waypoint_client.call_async(request)
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
            self.send_slack_message(channel_id, slack_message)  # 실제 채널 이름으로 변경하세요
            
        except Exception as e:
            error_message = f"Service call failed: {str(e)}"
            self.get_logger().error(error_message)
            self.send_slack_message(channel_id, f":x: {error_message}")  # 실제 채널 이름으로 변경하세요

        
        
    def setup_routes(self):
        """Flask 서버의 라우트를 설정합니다."""
        @self.app.route('/slack/events', methods=['POST'])
        def slack_events():
            # 콘텐츠 타입을 확인
            if request.content_type != 'application/json':
                return jsonify({"error": "Unsupported Media Type"}), 415

            data = request.get_json(silent=True)
            if not data:
                return jsonify({"error": "Invalid JSON"}), 400

            if 'challenge' in data:
                return jsonify({'challenge': data['challenge']})

            return '', 200

        @self.app.route('/slack/commands', methods=['POST'])
        def slack_commands():
            data = request.form
            command = data.get('command')
            text = data.get('text')
            user_id = data.get('user_id')
            channel_id = data.get('channel_id')
            
            # /order 커맨드 처리
            if command == SlackConstants.ORDER:
                user_name = self.fetch_user_name(user_id)
                channel_name = self.fetch_channel_name(channel_id)

                # 멘션된 사용자들의 slack_name 추출
                mentioned_user_names = re.findall(r"@(\w+)", text)
                
                # 작성자의 slack_name도 리스트에 추가
                slack_name_list = mentioned_user_names + [user_name]

                response_message = f"User {user_name} in channel {channel_name} issued an /order command with text: {text}"
                way_point = self.waypoint_calculator.calculate_optimal_route(slack_name_list)
                print(f"Waypoint: {way_point}")
                
                self.send_waypoint_to_service(CommandConstants.ORDER, channel_id, way_point)
                

                return jsonify({
                    "response_type": "in_channel",
                    "text": response_message
                })
                
            # /call command 처리
            elif command == SlackConstants.CALL:
                user_name = self.fetch_user_name(user_id)
                channel_name = self.fetch_channel_name(channel_id)

                # 멘션된 사용자들의 slack_name 추출
                mentioned_user_ids = re.findall(r"<@([A-Z0-9]+)>", text)
                
                # 작성자의 slack_name도 리스트에 추가
                slack_id_list = mentioned_user_ids + [user_id]

                # 예시로 호출된 사용자들에게 알림 메시지를 생성
                response_message = f"User {user_name} in channel {channel_name} issued a /call command to: {', '.join(slack_id_list)}"
                print(response_message)

                return jsonify({
                    "response_type": "in_channel",
                    "text": response_message
                })

            return '', 200

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