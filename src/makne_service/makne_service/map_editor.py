import os
import yaml
from PIL import Image, ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory
from library.Constants import MapEditorConstants

class MapEditor():
    def __init__(self):
        with open(os.path.join(get_package_share_directory("makne_ui"), "data", MapEditorConstants.MAP_YAML_FILE)) as f:
            map_data = yaml.full_load(f)
        self.origin_map = Image.open(os.path.join(get_package_share_directory("makne_ui"), "data", map_data["image"]))
        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data['origin'][:2]
        
    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return pos_x, pos_y
        
    def overlay_robot_state_on_image(self, robot_pose, robot_path):
        # 맵을 복사하고 RGB 형식으로 변환
        copy_map = self.origin_map.convert("RGB").copy()
        draw = ImageDraw.Draw(copy_map)

        # 폰트 설정 (시스템에 설치된 폰트 경로 필요)
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)

        # 이미지 상단에 제목 추가
        title_text = "Where are you Makne?"
        text_width, text_height = draw.textsize(title_text, font=font)
        image_width, image_height = copy_map.size
        title_position = ((image_width - text_width) // 2, 10)
        draw.text(title_position, title_text, font=font, fill="black")

        # 로봇의 위치에 사각형 그리기
        if robot_pose:
            x = robot_pose[MapEditorConstants.X_INDEX]
            y = robot_pose[MapEditorConstants.Y_INDEX]
            x_pixel, y_pixel = self.calc_grid_position(x, y)

            rect_size = 10
            draw.rectangle([x_pixel - rect_size, y_pixel - rect_size, x_pixel + rect_size, y_pixel + rect_size], outline="red", fill="red")

        # 로봇 경로를 파란색 선으로 그리기
        if robot_path and len(robot_path) > 0:
            path_points = []
            for pose in robot_path:
                path_x_pixel, path_y_pixel = self.calc_grid_position(pose[MapEditorConstants.X_INDEX], pose[MapEditorConstants.Y_INDEX])
                path_points.append((path_x_pixel, path_y_pixel))
            path_points = [(x_pixel, y_pixel)] + path_points
            draw.line(path_points, fill="blue", width=3)

        return copy_map
