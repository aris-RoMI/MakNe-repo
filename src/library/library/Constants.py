class SlackConstants:
    SEND_ROBOT = "/send_robot"
    CALL_ROBOT = "/call_robot"
    FOLLOW = "/follow"
    
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
    ORDER = 1
    CALL = 2
    FOLLOW = 3
    CANCEL = 99
    
class RobotStatus:
    STATUS_STANDBY = 0
    STATUS_ORDER = 1
    STATUS_CALL = 2
    STATUS_FOLLOW = 3
    STATUS_ERROR = 90
    STATUS_CANCEL = 99
    
    
