from control_actions import *
from utilities_motors import *
from utilities_camera import *
from utilities_record import *
from utilities_perception import *
from basic_init import *

DEFAULT_ADVANCE_DISTANCE = 20.42  # cm
def first_planner(color_obj,goal_coordinate):
	# Initialize GPIO pins
	init_gpio()
	
	# Initialize camera
	camera = init_camera()
	
	# Initialize IMU sensor
	imu_sensor = init_serial_read()
	
	# Initialize servo
	servo = init_servo()
	
	# Initialize PWM motors
	pwm_left, pwm_right = motor_pwm_setup()
	
	# Define the color of the block to track
	color_block = 'green'
	
	# Define the goal coordinate in cm
	goal_coordinate = (0, 65)
	
	# Record positions
	record_pos = []
	

def localize():
	pass
	# #turn to angle 0 
	# control_rotation_imu(motor_pwm_setup, 0, imu_sensor)
	# #turn to angle 90
	# control_rotation_imu(motor_pwm_setup, 90, imu_sensor)
	# #turn to angle 180
	# control_rotation_imu(motor_pwm_setup, 180, imu_sensor)
	# #turn to angle 270
	# control_rotation_imu(motor_pwm_setup, -90, imu_sensor)

def align_with_block(camera_pi, color_block, imu_sensor,record_pos):
		info_block = inform_block(camera_pi, color_block)
		#check if there are any blocks detected
		if info_block:
			_, _,center = area_aspect_ratio_center(info_block)
			#calculate the angle of the block with respect to the center of the image
			angle_block = angle_block_gripper_by(center)
			print(f'angle: {angle_block}')
			current_angle = read_imu_yaw_angle(imu_sensor)
			print(f'current angle is {current_angle}')
			reference_angle = round(current_angle + angle_block, 2) - 2
			if reference_angle > 360:
				reference_angle = reference_angle % 360
			print(f'reference_angle is {reference_angle}')
			#control the rotation of the robot to align with the block
			control_rotation_imu(motor_pwm_setup, reference_angle, imu_sensor, record_pos)

def inform_block(camera_pi, color_block):
	image = take_image(camera_pi)
	#process the image to find potential blocks
	_, info_contours = find_block(image, color_block)
	#check if there are any blocks detected
	if len(info_contours):
		#find the block with the greatest area that is also the closest one to the robot
		target_block = get_nearest_block(info_contours)
		return target_block
	return False
def move_to_block(camera_pi, color_block,record_pos):
	#take a photo with the camera
	image = take_image(camera_pi)
	#process the image to find the block i'm aligned with
	_, info_contours = find_block(image, color_block)
	#check if there are any blocks detected
	if len(info_contours):

