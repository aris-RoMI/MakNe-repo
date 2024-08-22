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
        
        
    def overlay_robot_state_on_image(self, robot_state):
        copy_map = self.origin_map.copy()
        draw = ImageDraw.Draw(copy_map)
        

        # 폰트 설정 (시스템에 설치된 폰트 경로 필요)
        font = ImageFont.truetype("arial.ttf", 20)

        # 로봇의 위치와 경로 정보 오버레이
        if robot_state.current_pose:
            position_text = f"Position: ({robot_state.current_pose.position.x}, {robot_state.current_pose.position.y})"
            draw.text((10, 10), position_text, font=font, fill="red")

        if robot_state.path:
            path_text = f"Path points: {len(robot_state.path.poses)}"
            draw.text((10, 40), path_text, font=font, fill="blue")

        # 오버레이된 이미지를 저장
        output_image_path = "/path/to/output_image.png"  # 저장할 경로 지정
        self.image.save(output_image_path)

        return copy_map