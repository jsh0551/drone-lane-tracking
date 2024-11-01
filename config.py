import cv2
from easydict import EasyDict
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

cfg = EasyDict()
cfg.SETTING = EasyDict()
cfg.DETECT = EasyDict()
cfg.CONTROL = EasyDict()
cfg.CALIBRATION = EasyDict()

# settings
cfg.SETTING.VIZ_FRONT = True
cfg.SETTING.VIZ_SIDE = False
cfg.SETTING.AIRSIM = True
cfg.SETTING.RECORD_DURATION = 5

# detection variables
cfg.DETECT.POS_RATIO = 0.25
cfg.DETECT.SIMILARITY_LIMIT = [0.75, 0.75, 0.85, 0.85]

# control variables
cfg.CONTROL.PERIOD = 0.05
cfg.CONTROL.STEP = 40
cfg.CONTROL.VELOCITY_AUTO = 6.0
cfg.CONTROL.VELOCITY_RULE = 1.0
cfg.CONTROL.ALTITUDE = 2.1
cfg.CONTROL.ORIGIN = [0.0, 0.0, 0.0, 0.0]
cfg.CONTROL.ROT_OFFSET = 10
cfg.CONTROL.DRONE_NUM = 5
cfg.CONTROL.RUNNER_NUM = 0
cfg.CONTROL.EVENT = 3 # 0: 100m, 1:200m, 2:400m, 3: over 400m

# calibration variables
cfg.CALIBRATION.POSITION = [8, 8, 4]
cfg.CALIBRATION.ANGLE = 0 # degrees
cfg.CALIBRATION.DVEL_LIMIT = 0.4
cfg.CALIBRATION.THETA_LIMIT = 3.0 # degrees
cfg.CALIBRATION.stablizing_time = 5
cfg.CALIBRATION.termination_time = 0.1
cfg.CALIBRATION.mode_response_time = 10
cfg.CALIBRATION.alt_threshold = 1.6
cfg.CALIBRATION.land_threshold = 0.3

# track params
cfg.TRACK_R = [37.898, 39.118, 40.338, 41.5,58, 42.778, 43.998, 45.218, 46.438]
cfg.TRACK_ANGLE = [0.0, 10.29766794, 20.85159044, 30.77199376, 40.11425805, 48.92749134, 57.25539304, 65.13697863]
cfg.TRACK_DISTANCE = [80, 200, 400, 800]

# camera params
if cfg.SETTING.AIRSIM == True:
    cfg.HFOV = 65
    cfg.ASPECT_RATIO = 4 / 3
    cfg.CAM_TILT = -30.0
    cfg.WIDTH = 480
    cfg.HEIGHT = 360
else:
    cfg.HFOV = 110
    cfg.ASPECT_RATIO = 4 / 3
    cfg.CAM_TILT = -45.0
    cfg.WIDTH = 480
    cfg.HEIGHT = 360

# PID params
if cfg.SETTING.AIRSIM == True:
    cfg.dvel_k_matrix = [[0.9, 0.005, 0.005], # 100m
                         [0.9, 0.005, 0.005], # 200m
                         [0,0,0], # 400m
                         [1.6, 0.02, 0.1]] # over 400m
        
    cfg.dtheta_k_matrix = [[0.1, 0.0001, 0.0001], # 100m
                        [0.015, 0.0005, 0.0005], # 200m
                        [0,0,0], # 400m
                        [0.5, 0.01, 0.002]] # over 400m
else:
    cfg.dvel_k_matrix = [[0.9, 0.005, 0.005], # 100m
                         [0.9, 0.005, 0.005], # 200m
                         [0,0,0], # 400m
                         [1.2, 0.02, 0.1]] # over 400m
        
    cfg.dtheta_k_matrix = [[0.1, 0.0001, 0.0001], # 100m
                        [0.015, 0.0005, 0.0005], # 200m
                        [0,0,0], # 400m
                        [0.5, 0.01, 0.002]] # over 400m

# ETC.
cfg.qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
cfg.font = cv2.FONT_HERSHEY_SIMPLEX
cfg.font_scale = 0.4
cfg.thickness = 1