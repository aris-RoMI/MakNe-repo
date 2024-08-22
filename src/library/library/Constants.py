class SlackConstants:
    SEND_ROBOT = "/send_robot"
    CALL_ROBOT = "/call_robot"
    FOLLOW = "/follow"
    GET_STATUS = "/get_status"
    
    SLACK_CONFIG_FILE = "slack_config.json"
    
class DBConstants:
    DB_NAME = "makne_db"
    WORKER_INFO = "WorkerInfo"
    LOCATION_INFO = "LocationInfo"
    LOCATION_NAME = "location_name"
    SLACK_ID = "slack_id"
    WORKER_ID = "worker_id"
    WORKER_ID_COLUMN = 0
    LOCATION_X_COLUMN = 2
    LOCATION_Y_COLUMN = 3
    
    CAFE = "cafe"
    STANDBY = "standby"
    PRINTER = "printer"
    
class CommandConstants:
    BACK_TO_HOME = 0
    SEND_ROBOT = 1
    CALL_ROBOT = 2
    FOLLOW = 3
    RETURN = 4
    CANCEL = 99
    
class RobotStatus:
    STATUS_STANDBY = 0
    STATUS_SEND = 1
    STATUS_CALL = 2
    STATUS_FOLLOW = 3
    STATUS_WAITING = 4
    STATUS_RETURN = 5
    STATUS_ERROR = 6
    
    STATUS_LIST = ["standby","send", "call", "follow", "waiting", "return", "error"]
    
class MapEditorConstants:
    YAML_FILE_NAME = "room_103.yaml"
    ROBOT_POSE_X_INDEX = 0
    ROBOT_POSE_Y_INDEX = 1
