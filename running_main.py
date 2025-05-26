import traceback
# import os
# import sys
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'modules')))
from modules.basic_init import *
from modules.utilities_camera import *
from modules.utilities_motors import *
from modules.planner import *

try:
	block_sequence = ['red','green','red']*3 # there is a total of 9 blocks to catch
	# Initialize GPIO pins
	init_gpio()
	# # Initialize camera
	CAMERA_PI_4 = init_camera()
	# # Initialize IMU sensor
	IMU_SENSOR = init_serial_read()
	# Initialize servo
	SERVO_GRIPPER = init_servo()
	# # Record positions
	record_pos = []
	record_pos.append(INITIAL_POS_DEFAULT)#you can change the initial position format (pos_x, pos_y, angle)
	goal_coordinates = [(30.48,111.84)]*TOTAL_BLOCKS*3 # 9 coordinates to go
	print(f'First goal coordinate for {block_sequence[-1]}  block is {goal_coordinates[-1]}')
	first_planner(block_sequence, record_pos, goal_coordinates, CAMERA_PI_4, IMU_SENSOR, SERVO_GRIPPER)
except KeyboardInterrupt:
	print("Stopping motors")
except Exception as error:
	print(f"An error occurred: {error}")
	traceback.print_exc()
finally:
	turn_off_motors()
	turn_off_servo(SERVO_GRIPPER)
	CAMERA_PI_4.stop()
	CAMERA_PI_4.close()
	gameover()
