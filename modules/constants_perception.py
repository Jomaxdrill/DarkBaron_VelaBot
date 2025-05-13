import numpy as np
import cv2
from utilities_camera import CAMERA_MAIN_RESOLUTION


#TODO: UNDERSTAND HOW ILLUMINATION CAN AFFECT THE MEASUREMENTS
#TODO: SET THS HSV IN THE GRAND CHALLENGE USED
###*COLOR RANGES
#*GREEN+
# V 180 at home with light #75 with kitchen lights
LOWER_GREEN1 = np.array([35, 85, 140])
UPPER_GREEN1 = np.array([85, 255, 255])
#*GREEN+
# V 180 at home with light #75 with kitchen lights
LOWER_GREEN2 = np.array([35, 85, 170])
UPPER_GREEN2 = np.array([85, 255, 255])
#*RED (Two ranges)
# V 165 at home with light #60 with kitchen lights, 110 sometimes
# First range: Hue 0-5 (lower red)
LOWER_RED1 = np.array([0, 115, 90])# 
UPPER_RED1 = np.array([10, 255, 255])
# Second range: Hue 160-179 (upper red)
LOWER_RED2 = np.array([160, 115, 90])
UPPER_RED2 = np.array([179, 255, 255])

#*BLUE
# V 165 at home with light #60 with kitchen lights, 90 sometimes
LOWER_BLUE1 = np.array([90, 115, 106])
UPPER_BLUE1 = np.array([130, 255, 255])
# V 165 at home with light #60 with kitchen lights, 90 sometimes
LOWER_BLUE2 = np.array([90, 115, 106])
UPPER_BLUE2 = np.array([130, 255, 255])
#*BLACK
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 100, 80])
# Updated BLOCK_COLORS to handle red's dual ranges
BLOCK_COLORS = {
    'green': {
        'ranges': [
            {'lower': LOWER_GREEN1, 'upper': UPPER_GREEN1},
            {'lower': LOWER_GREEN2, 'upper': UPPER_GREEN2}
        ]
    },
    'red': {
        'ranges': [
            {'lower': LOWER_RED1, 'upper': UPPER_RED1},
            {'lower': LOWER_RED2, 'upper': UPPER_RED2},
        ]
    },
    'blue': {
        'ranges': [
            {'lower': LOWER_BLUE1, 'upper': UPPER_BLUE1},
            {'lower': LOWER_BLUE2, 'upper': UPPER_BLUE2}
        ]
    },
    # 'black': {
    #     'ranges': [{'lower': LOWER_BLACK, 'upper': UPPER_BLACK}]
    # }
}
#TODO: IMPROVE THE THRESHOLDS FOR ARROW
LOWER_GREEN_ARROW = np.array([75, 95, 60]) #50,100,100
UPPER_GREEN_ARROW = np.array([95, 255, 220]) #70,255,255

##*COLORS FOR CONTOURS
CIRCLE_COLOR = (255, 255, 0)
CIRCLE_BORDER = 5
RECT_COLOR= (128, 0, 128)
RECT_BORDER = 2
GENERAL = (255, 0, 0)
GENERAL_BORDER = 3
CROSS_COLOR = (0, 255, 255)
CROSS_BORDER = 2
CROSS_LENGTH = 50
COLORS_TEXTS = {
	'green': (0, 255, 0),
	'red': (0, 0, 255),
	'blue': (255, 0, 0),
	'black': (0, 0, 0),
	}
####*FILTERS
GAUSS_KERNEL =  (5,5)
MEDIAN_BLUR_KERNEL = 5
KERNEL_MORPH = np.ones((3, 3), np.uint8)

###*TEXT
FONT = cv2.FONT_HERSHEY_PLAIN

###*FOR DETECTION
GRIPPER_COLOR = 'black'
NOISY_CONTOUR_AREA = 120 #minimum area of the contour to be considered
PIXEL_ANGLE = 0.062 #angle in degrees that represents a pixel movement #0.0605
CENTER_X_IMAGE = int(CAMERA_MAIN_RESOLUTION[0]//2)


DISTANCE_RANGES = {
    'red': {
        'low_ar': [
            (216, 240), (240, 230), (273, 220), (308, 210), (360, 200),
            (442, 190), (504, 180), (589, 170), (672, 160), (828, 150),
            (1040, 140), (1176, 130), (1410, 120), (1600, 110), (1960, 100),
            (2146, 95), (2480, 90), (2814, 85), (3266, 80), (3773, 75),
            (4590, 70), (5580, 65), (6930, 60), (8968, 55), (9360, 54),
            (9717, 53), (10287, 52), (10707, 51), (11305, 50), (11919, 49),
            (12831, 48), (13578, 47), (14496, 46), (14841, 45), (15800, 44),
            (16728, 43), (17914, 42), (19250, 41), (20566, 40), (22302, 39),
            (24156, 38), (26752, 37), (28861, 36), (30800, 35), (32631, 34)
        ],
        'high_ar': [
            (0.749, 33), (0.815, 32), (0.883, 31), (0.968, 30), (1.085, 29),
            (1.223, 28), (1.256, 27), (1.464, 26), (1.5, 25), (1.54, 24),
            (2, 23), (2.4, 22), (3.266, 21), (3.937, 20), (4.107, 19),
            (6.12, 18)
        ]
    },
    'green': {
        'low_ar': [
            (266, 240), (300, 230), (345, 220), (408, 210), (459, 200),
            (513, 190), (589, 180), (693, 170), (782, 160), (912, 150),
            (1026, 140), (1160, 130), (1426, 120), (1716, 110), (2128, 100),
            (2440, 95), (2624, 90), (3036, 85), (3358, 80), (3975, 75),
            (4730, 70), (5760, 65), (7169, 60), (9044, 55), (9317, 54),
            (10000, 53), (10496, 52), (11004, 51), (11610, 50), (12371, 49),
            (13104, 48), (13818, 47), (14647, 46), (16160, 45), (16892, 44),
            (18083, 43), (19425, 42), (20748, 41), (22610, 40), (24278, 39),
            (26035, 38), (29212, 37), (31640, 36), (33142, 35), (33572, 34)
        ],
        'high_ar': [
            (0.759, 33), (0.829, 32), (0.8888, 31), (0.984, 30), (1.073, 29),
            (1.171, 28), (1.346, 27), (1.4, 26), (1.6, 25), (1.7, 24),
            (2.12, 23), (2.47, 22), (3.1, 21), (3.8, 20), (4.1, 19),
            (5, 18)
        ]
    },
    'blue': {
        'low_ar': [
            (135, 240), (216, 230), (220, 220), (228, 210), (273, 200),
            (384, 190), (468, 180), (600, 170), (704, 160), (805, 150),
            (840, 140), (950, 130), (1134, 120), (1457, 110), (1768, 100),
            (1980, 95), (2301, 90), (2583, 85), (2948, 80), (3504, 75),
            (4212, 70), (5220, 65), (6400, 60), (8584, 55), (8850, 54),
            (9317, 53), (10000, 52), (10332, 51), (10920, 50), (11745, 49),
            (12232, 48), (13050, 47), (14100, 46), (14938, 45), (15840, 44),
            (17056, 43), (18404, 42), (19869, 41), (21505, 40), (23086, 39),
            (25092, 38), (27950, 37), (30551, 36), (32376, 35), (32850, 34)
        ],
        'high_ar': [
            (0.75, 33), (0.816, 32), (0.87, 31), (0.979, 30), (1.083, 29),
            (1.168, 28), (1.34, 27), (1.497, 26), (1.525, 25), (1.553, 24),
            (2.4, 23), (2.487, 22), (2.943, 21), (3.774, 20), (4, 19),
            (5.906, 18)
        ]
    }
}

#HOMOGRAPHY

HOMOGRAPHY_MATRIX = np.array([[ 2.37628909e+01,  8.41065180e+01, -1.17410009e+04],
 [-1.13422271e+01,  4.32666455e+02, -2.74226013e+04],
 [-1.06213304e-02,  2.48212005e-01,  1.00000000e+00]])

HOMOGRAPHY_SIZE = (720,1600)