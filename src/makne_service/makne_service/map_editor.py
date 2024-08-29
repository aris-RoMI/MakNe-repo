import os
import yaml
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory
from library.Constants import MapEditorConstants

class MapEditor():
    def __init__(self):
        # YAML 파일 로드 및 맵 이미지 로드
        with open(os.path.join(get_package_share_directory("makne_ui"), "data", MapEditorConstants.MAP_YAML_FILE)) as f:
            map_data = yaml.full_load(f)
        self.map_image = cv2.imread(os.path.join(get_package_share_directory("makne_ui"), "data", map_data["image"]))
        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data['origin'][:2]
        self.image_scale = 1  # 이미지 스케일 설정

    def real_to_pixel_coords(self, real_coords):
        """실제 좌표를 픽셀 좌표로 변환"""
        x_pixel = int((real_coords[MapEditorConstants.X_INDEX] - self.map_origin[MapEditorConstants.X_INDEX]) / self.map_resolution * self.image_scale)
        y_pixel = int((self.map_image.shape[MapEditorConstants.X_INDEX] - (real_coords[MapEditorConstants.Y_INDEX] - self.map_origin[MapEditorConstants.Y_INDEX]) / self.map_resolution) * self.image_scale)
        return x_pixel, y_pixel

    def draw_robot_and_path(self, image, position_real, path_real):
        """지도 위에 로봇의 현재 위치와 경로를 그리기"""
        # 경로 그리기
        if path_real:
            for i in range(1, len(path_real)):
                start_point = self.real_to_pixel_coords(path_real[i-1])
                end_point = self.real_to_pixel_coords(path_real[i])
                print(start_point, end_point)
                cv2.line(image, start_point, end_point, (255, 0, 0), 1)

        # 현재 로봇 위치 그리기
        robot_position_pixel = self.real_to_pixel_coords(position_real)
        cv2.circle(image, robot_position_pixel, 3, (0, 0, 255), -1)

        return image

    def overlay_robot_state_on_image(self, robot_pose, robot_path):
        """로봇 상태를 이미지에 오버레이"""
        # 로봇과 경로를 지도 위에 그리기
        map_image_copy = self.map_image.copy()
        map_image_drawing = self.draw_robot_and_path(map_image_copy, robot_pose, robot_path)

        # 관심 영역 설정
        x_min = max(MapEditorConstants.ROI_CENTER_X - MapEditorConstants.ROI_SIZE // 2, 0)
        x_max = min(MapEditorConstants.ROI_CENTER_X + MapEditorConstants.ROI_SIZE // 2, map_image_drawing.shape[MapEditorConstants.Y_INDEX])
        y_min = max(MapEditorConstants.ROI_CENTER_Y - MapEditorConstants.ROI_SIZE // 2, 0)
        y_max = min(MapEditorConstants.ROI_CENTER_Y + MapEditorConstants.ROI_SIZE // 2, map_image_drawing.shape[MapEditorConstants.X_INDEX])

        # 관심 영역 자르기
        roi = map_image_drawing[y_min:y_max, x_min:x_max]

        # 관심 영역 회전 및 확대
        rotated_roi = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
        resized_roi = cv2.resize(rotated_roi, (rotated_roi.shape[MapEditorConstants.Y_INDEX] * 3, rotated_roi.shape[MapEditorConstants.X_INDEX] * 3), interpolation=cv2.INTER_LINEAR)

        # 텍스트 추가
        text = "Where are you Mak-ne?"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.9
        font_thickness = 3
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        text_x = (resized_roi.shape[MapEditorConstants.Y_INDEX] - text_size[0]) // 2
        text_y = text_size[MapEditorConstants.Y_INDEX] + 10

        cv2.putText(resized_roi, text, (text_x, text_y), font, font_scale, (0, 0, 0), font_thickness, cv2.LINE_AA)

        is_success, buffer = cv2.imencode(".png", resized_roi)
        if is_success:
            return buffer
        else:
            raise ValueError("Image encoding failed")
        # self.resized_roi = resized_roi
    def show_image(self):
        """결과 이미지를 화면에 보여주기"""
        cv2.imshow('Resized ROI with Text', self.resized_roi)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main():
    # 임의의 로봇 위치와 경로 설정
    robot_pose = [2.7, 2]
    robot_path = [[1, -1], [0.015, -0.035]]

    # MapEditor 인스턴스 생성 및 이미지 표시
    map_editor = MapEditor()
    map_editor.overlay_robot_state_on_image(robot_pose, robot_path)
    map_editor.show_image()

if __name__ == "__main__":
    main()
