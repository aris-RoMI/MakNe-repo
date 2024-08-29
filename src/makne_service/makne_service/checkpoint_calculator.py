import numpy as np
from makne_db.db_manager import DBManager
from library.Constants import DBConstants

class CheckPointCalculator():
    def __init__(self, db):
        self.db_manager = DBManager(db)
    
    def calculate_distance(self, point1, point2):
        """두 좌표 간의 유클리드 거리를 계산합니다."""
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def reorder_waypoints(self, starting_point, waypoints):
        """웨이포인트 배열을 현재 위치에서 최적의 순서로 재배치합니다."""
        current_point = starting_point
        ordered_waypoints = []
        
        while waypoints:
            # 가장 가까운 웨이포인트를 찾음
            closest_point = min(waypoints, key=lambda point: self.calculate_distance(current_point, point))
            ordered_waypoints.append(closest_point)
            waypoints.remove(closest_point)
            current_point = closest_point

        return ordered_waypoints

    def get_waypoints_from_slack_ids(self, slack_ids):
        """slack_id 리스트를 기반으로 위치 정보를 가져오고 최적의 순서로 재배치"""
        waypoints = []

        for slack_id in slack_ids:
            # WorkerInfo 테이블에서 worker_id를 가져옴
            worker_data = self.db_manager.get_data_with_condition(DBConstants.WORKER_INFO, DBConstants.SLACK_ID, slack_id)
            if worker_data:
                
                worker_id = worker_data[0][DBConstants.WORKER_ID_COLUMN]  # worker_data[0][1]은 worker_id를 의미
                # LocationInfo 테이블에서 해당 worker_id에 대한 위치 정보를 가져옴
                location_data = self.db_manager.get_data_with_condition(DBConstants.LOCATION_INFO, DBConstants.WORKER_ID, worker_id)
                if location_data:
                    
                    location_x = location_data[0][DBConstants.LOCATION_X_COLUMN]  # location_x
                    location_y = location_data[0][DBConstants.LOCATION_Y_COLUMN]  # location_y
                    print(f"location_by_slack_ids : {location_x}, {location_y}")
                    waypoints.append((location_x, location_y))

        return waypoints

    def calculate_optimal_route(self, slack_ids, starting_point):
        """slack_id 리스트와 시작점을 사용하여 최적의 경로를 계산"""
        waypoints = self.get_waypoints_from_slack_ids(slack_ids)
        if waypoints and starting_point:
            optimal_route = self.reorder_waypoints(starting_point, waypoints)
            return optimal_route
        else:
            print("No waypoints found for the provided slack_ids.")
            return []
        
    def get_point_from_location_name(self, location_name):
        location_data = self.db_manager.get_data_with_condition(DBConstants.LOCATION_INFO, DBConstants.LOCATION_NAME, location_name)
        location_x = location_data[0][DBConstants.LOCATION_X_COLUMN]
        location_y = location_data[0][DBConstants.LOCATION_Y_COLUMN]
        location_point = (location_x, location_y)
        return location_point

def main():
    waypoint_calculator = WayPointCalculator(DBConstants.DB_NAME)

    # 예시 시작점과 slack_id 리스트
    slack_ids = ["dlwlgh0106", "jigu0825"]

    optimal_route = waypoint_calculator.calculate_optimal_route(slack_ids)
    print("Optimal route:", optimal_route)

if __name__ == "__main__":
    main()