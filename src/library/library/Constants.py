class SlackConstants:
    ORDER = "/order"
    CALL = "/call"
    CANCEL = "/cancel"
    
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
    
class CommandConstants:
    ORDER = 1
    CALL = 2
    CANCEL = 3