import os
import yaml
from PIL import Image, ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory
from library.Constants import MapEditorConstants

class MapEditor():
    def __init__(self):
        with open(os.path.join(get_package_share_directory("makne_ui"), "data", MapEditorConstants.YAML_FILE_NAME)) as f:
            map_data = yaml.full_load(f)
        self.origin_map = Image.open(os.path.join(get_package_share_directory("makne_ui"), "data", map_data["image"]))
        
    def overlay_robot_state_on_image(self, robot_pose, robot_path):
        copy_map = self.origin_map.copy()
        draw = ImageDraw.Draw(copy_map)

        # 폰트 설정 (시스템에 설치된 폰트 경로 필요)
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 30)  # 폰트 크기를 키워 제목 느낌을 줍니다.

        # 이미지 상단에 제목 추가
        title_text = "Where are you Makne?"
        text_width, text_height = draw.textsize(title_text, font=font)
        image_width, image_height = copy_map.size
        title_position = ((image_width - text_width) // 2, 10)  # 중앙 정렬, 상단 10 픽셀 아래에 위치
        draw.text(title_position, title_text, font=font, fill="black")

        # 로봇의 위치에 사각형 그리기
        if robot_pose:
            x = robot_pose[MapEditorConstants.ROBOT_POSE_X_INDEX]
            y = robot_pose[MapEditorConstants.ROBOT_POSE_Y_INDEX]
            rect_size = 10  # 사각형 크기
            draw.rectangle([x - rect_size, y - rect_size, x + rect_size, y + rect_size], outline="red", fill="red")

        # 로봇 경로를 파란색 선으로 그리기
        if robot_path and len(robot_path) > 1:
            path_points = [(pose[MapEditorConstants.ROBOT_POSE_X_INDEX], pose[MapEditorConstants.ROBOT_POSE_Y_INDEX]) for pose in robot_path]
            draw.line(path_points, fill="blue", width=3)

        return copy_map
