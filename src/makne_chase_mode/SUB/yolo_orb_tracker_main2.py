import numpy as np
import cv2
import torch
from ultralytics import YOLO
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class YoloOrbTracker(Node):
    # Constants
    NODE_NAME = 'yolo_orb_tracker_main'
    SUB_COLOR_IMAGE_TOPIC = '/converted_color_image/compressed'
    PUB_CENTER_POINT_TOPIC = '/id1_center_point'
    PUB_BBOX_IMAGE_TOPIC = '/bbox_image'
    PUB_ORB_IMAGE_TOPIC = '/orb_image'
    QOS_PROFILE = 10
    YOLO_MODEL = 'yolov8n.pt'
    PERSON_CLASS_ID = 0
    YOLO_IMAGE_SIZE = 320
    MAX_FRAMES = 100
    CONFIDENCE_THRESHOLD = 0.5
    NMS_THRESHOLD = 0.15
    DISTANCE_THRESHOLD = 50
    ORB_SCORE_THRESHOLD = 30

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.image_sub = self.create_subscription(CompressedImage, self.SUB_COLOR_IMAGE_TOPIC, self.image_callback, self.QOS_PROFILE)

        self.image_pub = self.create_publisher(Image, self.PUB_BBOX_IMAGE_TOPIC, self.QOS_PROFILE)

        self.orb_image_pub = self.create_publisher(Image, self.PUB_ORB_IMAGE_TOPIC, self.QOS_PROFILE)

        self.pub_center_point = self.create_publisher(Point, self.PUB_CENTER_POINT_TOPIC, self.QOS_PROFILE)

        self.br = CvBridge()
        self.model = YOLO(self.YOLO_MODEL, task='detect', verbose=True)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        self.get_logger().info(message=f"설정된 디바이스: {self.device}")

        self.frame_objects = {}
        self.objects_list = []
        self.id1_objects = {}
        self.kalman_filters = {}
        self.next_object_id = 1

        self.orb = cv2.ORB_create()

        self.screen_center_x = None
        self.screen_center_y = None
        self.closest_id_assigned = False

        self.cv_image = None

        self.pub_flag = 0

        # 추가
        self.first_frame_processed = False
        self.id_1_center = None
        self.id_1_descriptors = None
        self.id1_orb = None
        self.id1_hist = None
        self.find_id1()
        self.detection_timer = self.create_timer(0.3, self.detection)

    def initialize_kalman_filter(self, x, y):
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]], np.float32)
        kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0]], np.float32)
        kf.processNoiseCov = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 10, 0],
                                    [0, 0, 0, 10]], np.float32) * 0.1

        kf.measurementNoiseCov = np.array([[0.1, 0],
                                            [0, 0.1]], np.float32) * 0.5

        kf.errorCovPost = np.eye(4, dtype=np.float32)
        kf.statePost = np.array([x, y, 0, 0], dtype=np.float32)
        return kf

    def predict_kalman(self, kf):
        prediction = kf.predict()
        return int(prediction[0]), int(prediction[1])

    def correct_kalman(self, kf, x, y):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        corrected = kf.correct(measurement)
        return int(corrected[0]), int(corrected[1])

    def publish_id1_center(self):
        point_msg = Point()

        if self.pub_flag == 1:
            center_x, center_y = self.frame_objects[1][0]
            point_msg.x = float(center_x)
            point_msg.y = float(center_y)
            point_msg.z = 0.0
        else:
            point_msg.x = float('inf')
            point_msg.y = float('inf')
            point_msg.z = float('inf')
            self.get_logger().info(message=f'X: {point_msg.x} / Y: {point_msg.y} / Z: {point_msg.z}')
        self.pub_center_point.publish(point_msg)

    def image_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def find_id1(self):
        if self.cv_image is None:
            return

        if self.screen_center_x is None or self.screen_center_y is None:
            height, width, _ = self.cv_image.shape
            self.screen_center_x = width // 2
            self.screen_center_y = height // 2

        results = self.model(self.cv_image, imgsz=self.YOLO_IMAGE_SIZE)

        confidences = []
        boxes = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)
                confidence = box.conf
                if class_id == self.PERSON_CLASS_ID and confidence > self.CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    boxes.append([x1, y1, w, h])
                    confidences.append(float(confidence))

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)

        orb_list = []
        center_list = []
        hist_list = []

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                roi = self.cv_image[y:y+h, x:x+w]
                roi_stretched = cv2.normalize(roi, None, 0, 255, cv2.NORM_MINMAX)

                # ROI에서 히스토그램 계산 (B, G, R 채널별)
                hist_b = cv2.calcHist([roi], [0], None, [256], [0, 256])
                hist_g = cv2.calcHist([roi], [1], None, [256], [0, 256])
                hist_r = cv2.calcHist([roi], [2], None, [256], [0, 256])
                hist_vector = np.concatenate((hist_b, hist_g, hist_r)).flatten()
                hist_list.append(hist_vector)

                keypoints, descriptors = self.orb.detectAndCompute(roi_stretched, None)

                orb_list.append(descriptors)
                
                center_x = x + w // 2
                center_y = y + h // 2
                current_position = np.array([center_x, center_y])
                center_list.append(current_position)

                distance_to_center = np.linalg.norm(current_position - np.array([self.screen_center_x, self.screen_center_y]))
                if not self.closest_id_assigned and distance_to_center < closest_distance:
                    closest_distance = distance_to_center
                    closest_object_id = i

        self.id1_orb = orb_list[closest_object_id]
        self.id_1_center = center_list[closest_object_id]
        self.id1_hist = hist_list[closest_object_id]
        self.get_logger().info("=============ID 1 object detected!!=============")

    def detection(self):
        if self.cv_image is None:
            return

        # if self.screen_center_x is None or self.screen_center_y is None:
        #     height, width, _ = self.cv_image.shape
        #     self.screen_center_x = width // 2
        #     self.screen_center_y = height // 2

        results = self.model(self.cv_image, imgsz=self.YOLO_IMAGE_SIZE)

        confidences = []
        boxes = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)
                confidence = box.conf
                if class_id == self.PERSON_CLASS_ID and confidence > self.CONFIDENCE_THRESHOLD:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    w, h = x2 - x1, y2 - y1
                    boxes.append([x1, y1, w, h])
                    confidences.append(float(confidence))

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)

        self.frame_objects.clear()

        current_objects = {}
        closest_distance = float('inf')
        closest_object_id = None

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                roi = self.cv_image[y:y+h, x:x+w]
                roi_stretched = cv2.normalize(roi, None, 0, 255, cv2.NORM_MINMAX)

                # ROI에서 히스토그램 계산 (B, G, R 채널별)
                hist_b = cv2.calcHist([roi], [0], None, [256], [0, 256])
                hist_g = cv2.calcHist([roi], [1], None, [256], [0, 256])
                hist_r = cv2.calcHist([roi], [2], None, [256], [0, 256])
                hist_vector = np.concatenate((hist_b, hist_g, hist_r)).flatten()

                keypoints, descriptors = self.orb.detectAndCompute(roi_stretched, None)

                center_x = x + w // 2
                center_y = y + h // 2
                current_position = np.array([center_x, center_y])

                distance_to_center = np.linalg.norm(current_position - np.array([self.screen_center_x, self.screen_center_y]))
                if not self.closest_id_assigned and distance_to_center < closest_distance:
                    closest_distance = distance_to_center
                    closest_object_id = self.next_object_id

                # 추가
                # if not self.first_frame_processed:
                #     self.get_logger().info(f'Processing first frame: object at {center_x},{center_y}')
                #     if distance_to_center < closest_distance:
                #         self.id_1_center = current_position
                #         self.id_1_descriptors = descriptors
                #         closest_distance = distance_to_center
                #         closest_object_id = self.next_object_id
                #         self.closest_id_assigned = True

                matched_id = None

                # ID 1과의 매칭 우선 확인
                orb_match_score = 0
                if self.id1_orb is not None:
                    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                    matches = bf.match(self.id1_orb, descriptors)
                    matches = sorted(matches, key=lambda x: x.distance)
                    if len(matches) > 0:
                        orb_match_score = matches[0].distance

                # distance = np.linalg.norm(self.id_1_center - current_position)
                similarity = cv2.compareHist(hist_vector, self.id1_hist, cv2.HISTCMP_CORREL)
                        
                # 거리와 유사도 기준을 충족하는 경우, 동일 객체로 간주
                if orb_match_score < self.ORB_SCORE_THRESHOLD and similarity > 0.8:
                    matched_id = 1
                    self.get_logger().info(f'ID 1 object matched (similarity: {similarity}, ORB score: {orb_match_score})')


                if len(self.objects_list) > 0:
                    previous_frame_objects = self.objects_list[-1]

                    if matched_id is None:
                        for object_id, (predicted_position, old_descriptors, _) in previous_frame_objects.items():
                            distance = np.linalg.norm(predicted_position - current_position)
                            orb_match_score = 0
                            if old_descriptors is not None and descriptors is not None:
                                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                                matches = bf.match(old_descriptors, descriptors)
                                matches = sorted(matches, key=lambda x: x.distance)
                                if len(matches) > 0:
                                    orb_match_score = matches[0].distance

                            if distance < self.DISTANCE_THRESHOLD and orb_match_score < self.ORB_SCORE_THRESHOLD:
                                matched_id = object_id
                                self.get_logger().info(f'Object {object_id} matched (distance: {distance}, ORB score: {orb_match_score})')
                                break

                if matched_id is None:
                    for frame_objects in reversed(self.objects_list[:-1]):
                        for object_id, (predicted_position, old_descriptors, _) in frame_objects.items():
                            distance = np.linalg.norm(predicted_position - current_position)
                            orb_match_score = 0
                            if old_descriptors is not None and descriptors is not None:
                                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                                matches = bf.match(old_descriptors, descriptors)
                                matches = sorted(matches, key=lambda x: x.distance)
                                if len(matches) > 0:
                                    orb_match_score = matches[0].distance

                            if distance < self.DISTANCE_THRESHOLD and orb_match_score < self.ORB_SCORE_THRESHOLD:
                                matched_id = object_id
                                self.get_logger().info(f'Restored object {object_id} (distance: {distance}, ORB score: {orb_match_score})')
                                break
                        if matched_id is not None:
                            break

                if matched_id is None:
                    matched_id = self.next_object_id
                    self.next_object_id += 1
                    self.get_logger().info(f'New object {matched_id} assigned')

                if matched_id not in self.kalman_filters:
                    self.kalman_filters[matched_id] = self.initialize_kalman_filter(center_x, center_y)

                predicted_x, predicted_y = self.predict_kalman(self.kalman_filters[matched_id])
                center_x, center_y = self.correct_kalman(self.kalman_filters[matched_id], center_x, center_y)

                height, width, _ = self.cv_image.shape
                if 0 <= predicted_x < width and 0 <= predicted_y < height:
                    self.cv_image = cv2.circle(self.cv_image, (predicted_x, predicted_y), 3, (255, 0, 0), -1)
                self.cv_image = cv2.circle(self.cv_image, (center_x, center_y), 3, (0, 255, 0), -1)

                # if closest_object_id == matched_id or (hasattr(self, 'id_1_assigned_object') and matched_id == self.id_1_assigned_object):
                #     matched_id = 1
                #     self.closest_id_assigned = True
                #     self.id_1_assigned_object = matched_id

                if matched_id == 1:
                    self.id1_objects[matched_id] = (np.array([center_x, center_y]), descriptors, boxes[i])

                if matched_id not in current_objects:
                    self.frame_objects[matched_id] = (current_position, descriptors, boxes[i])

                self.cv_image = cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 1)
                if matched_id == 1:
                    self.cv_image = cv2.putText(self.cv_image, 'Target', (x1, y2-5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)
                else:
                    self.cv_image = cv2.putText(self.cv_image, f'ID {matched_id}', (x1, y2-5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)

        if 1 in self.frame_objects:
            self.objects_list.append(self.frame_objects.copy())
            self.pub_flag = 1
        else:
            self.pub_flag = 0

        if len(self.objects_list) > self.MAX_FRAMES:
            self.objects_list.pop(0)

        self.publish_id1_center()

        # # 추가
        # if not self.first_frame_processed and closest_object_id is not None:
        #     self.first_frame_processed = True
        #     self.get_logger().info(f'First frame processed, ID 1 assigned to object {closest_object_id}')

        bbox_image_msg = self.br.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
        self.image_pub.publish(bbox_image_msg)



def main(args=None):
    rp.init(args=args)
    tracker = YoloOrbTracker()
    rp.spin(tracker)
    tracker.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
