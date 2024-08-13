import numpy as np
from makne_db.db_manager import DBManager
from library.Constants import DBConstants

class WayPointCalculator():
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
            worker_data = self.db_manager.get_data_with_condition("WorkerInfo", "slack_id", slack_id)
            if worker_data:
                
                worker_id = worker_data[0][DBConstants.WORKER_ID]  # worker_data[0][1]은 worker_id를 의미
                # LocationInfo 테이블에서 해당 worker_id에 대한 위치 정보를 가져옴
                location_data = self.db_manager.get_data_with_condition("LocationInfo", "worker_id", worker_id)
                print(location_data)
                if location_data:
                    
                    location_x = location_data[0][DBConstants.LOCATION_X]  # location_x
                    location_y = location_data[0][DBConstants.LOCATION_Y]  # location_y
                    waypoints.append((location_x, location_y))

        return waypoints

    def calculate_optimal_route(self, starting_point, slack_ids):
        """slack_id 리스트와 시작점을 사용하여 최적의 경로를 계산"""
        waypoints = self.get_waypoints_from_slack_ids(slack_ids)
        if waypoints:
            optimal_route = self.reorder_waypoints(starting_point, waypoints)
            return optimal_route
        else:
            print("No waypoints found for the provided slack_ids.")
            return []

def main():
    waypoint_calculator = WayPointCalculator("makne_db")

    # 예시 시작점과 slack_id 리스트
    starting_point = (0, 0)
    slack_ids = ["dlwlgh0106", "syiner96", "jigu0825"]

    optimal_route = waypoint_calculator.calculate_optimal_route(starting_point, slack_ids)
    print("Optimal route:", optimal_route)

if __name__ == "__main__":
    main()