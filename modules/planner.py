from control_actions import *
from utilities_motors import *
from utilities_camera import *
from utilities_record import *
from utilities_perception import *
from basic_init import *

TOTAL_BLOCKS = 1
LOCALIZE_SEQUENCE = [0,90,180,-90,0]
SEARCH_SEQUENCE = [0,20,-20]
EDGE_MAP = 30.48 #cm
TOTAL_MAP = (115, 98)
#SEARCH_SEQUENCE = [0,20,-20,45,-45,90,-90]
# TOTAL_MAP = (365.76,365.76) #edge of total square map
# OFFSET_LOCATE = 2 #cm
# CORNER_GOAL = (30.48, 335.28) #cm
# block_sequence = ['red','blue','green']
def first_planner(block_sequence, goal_coordinates):
	# Initialize GPIO pins
	init_gpio()
	# Initialize camera
	camera = init_camera()
	# Initialize IMU sensor
	imu_sensor = init_serial_read()
	# Initialize servo
	servo = init_servo()
	# Record positions
	record_pos = []
	getting_block = False
	while len(block_sequence) > 0:
		info_block = mode_search(record_pos, camera, block_sequence[-1], imu_sensor, block_sequence)
		#check if there are any blocks detected
		if not info_block:
			continue
		aligned = align_with_block(info_block, imu_sensor,record_pos)
		if not aligned:
			continue
		reached_block = move_to_block(camera, servo, imu_sensor, block_sequence[-1], record_pos)
		if not reached_block:
			continue
		take_secure_block(camera, block_sequence[-1], record_pos)
		if not getting_block:
			continue
		move_to_goal(servo, imu_sensor, goal_coordinates[-1], record_pos, block_sequence)
	gameover()
	return 'Done all blocks wiiiiiii'

def create_goal_coordinate(X_o, Y_o):
	#TODO: Maybe one time check how to do this dynamically, it seems like a pascal tree related problem
	goals_blocks = [
		(X_o + 0.5*EDGE_MAP,Y_o - 0.5*EDGE_MAP), #first block
		(X_o + 0.5*EDGE_MAP,Y_o - 1.5*EDGE_MAP), #second block
		(X_o + 1.5*EDGE_MAP,Y_o - 0.5*EDGE_MAP), #third block
		(X_o + 0.5*EDGE_MAP,Y_o - 2.5*EDGE_MAP), #fourth block
		(X_o + 1.5*EDGE_MAP,Y_o - 1.5*EDGE_MAP), #fifth block
		(X_o + 2.5*EDGE_MAP,Y_o - 0.5*EDGE_MAP), #sixth block
		(X_o + 1.5*EDGE_MAP,Y_o - 2.5*EDGE_MAP), #seventh block
		(X_o + 2.5*EDGE_MAP,Y_o - 1.5*EDGE_MAP), #eighth block
		(X_o + 2.5*EDGE_MAP,Y_o - 2.5*EDGE_MAP), #ninth block
		]
	return goals_blocks[::-1] #return the last element of the list
def localize(imu_sensor, record_pos):
	distances_calibration = []
	for value in LOCALIZE_SEQUENCE:
		control_rotation_imu(motor_pwm_setup, value, imu_sensor,record_pos)
		distances_calibration.append(distance_sonar() +10) #10 cm offset in x direction to imu
		print(f'localize {value}')
	hor_distance = distances_calibration[0] + distances_calibration[2]
	ver_distance = distances_calibration[1] + distances_calibration[3]
	diff_hor = hor_distance - TOTAL_MAP[0]
	diff_ver = ver_distance - TOTAL_MAP[1]
	pos_x = record_pos[-1][0]
	pos_y = record_pos[-1][1]
	#an attempt to correct the position
	pos_x = pos_x - diff_hor if diff_hor > 0 else pos_x + diff_hor
	pos_y = pos_y - diff_ver if diff_ver > 0 else pos_y + diff_ver
	record_pos.append((pos_x, pos_y, LOCALIZE_SEQUENCE[-1][2]))
	return 'Done'

def align_with_block(info_block, imu_sensor, record_pos):
	_, _,center = area_aspect_ratio_center(info_block)
	#calculate the angle of the block with respect to the center of the image
	angle_block = angle_block_gripper_by(center)
	print(f'angle: {angle_block}')
	current_angle = read_imu_yaw_angle(imu_sensor)
	print(f'current angle is {current_angle}')
	#TODO: check if angle control can be changed as move until the block is in the center of the image
	reference_angle = round(current_angle + angle_block, 2)
	#normalize angle to the corresponding system
	if reference_angle > OFFSET_YAW:
		reference_angle = reference_angle % OFFSET_YAW
	if reference_angle > OFFSET_YAW//2 and reference_angle < OFFSET_YAW:
		reference_angle = reference_angle - OFFSET_YAW
	print(f'reference_angle is {reference_angle}')
	#control the rotation of the robot to align with the block
	control_rotation_imu(motor_pwm_setup, reference_angle, imu_sensor, record_pos)
	return True


def move_to_block(camera_pi, servo, imu_sensor, color_block, record_pos):
	image = take_image(camera_pi)
	info_block = inform_block(image, color_block)
	#check if there are any blocks detected
	if info_block:
		area, aspect_ratio, _ = area_aspect_ratio_center(info_block)
		dist_block = distance_to_block_by(area, aspect_ratio)
		#depending the distance to the block decide how to tackle
		if not dist_block:
			return False
		#open gripper
		action_gripper(servo,'OPEN')
		#advance one revolution, align with the block and advance again until the block is in reach
		steps_to_advance = 0
		while dist_block != 'In range to catch':
			action = forward
			if dist_block == 'Far Object':
				steps_to_advance = DEFAULT_ADVANCE_DISTANCE*0.8
			elif dist_block == 'Close Object':
				steps_to_advance = DEFAULT_ADVANCE_DISTANCE*0.8
			elif dist_block == 'Too Close':
				action = reverse
				steps_to_advance = 1
			control_translation(action, steps_to_advance, record_pos)
			#for every advancement align with the block as much as possible
			#take an image
			image = take_image(camera_pi)
			info_contours = inform_block(image, color_block)
			aligned = align_with_block(info_contours,imu_sensor,record_pos)
			if aligned:
				#TODO: Modify here to be able to implement path planning logic
				print(f'moving to block detecting {info_contours}')
				area, aspect_ratio, _ = area_aspect_ratio_center(info_contours)
				dist_block = distance_to_block_by(area, aspect_ratio)
		#catch the block but confirm the block is in the gripper after and is in the ideal range
		image = take_image(camera_pi)
		info_contours = inform_block(image, color_block)
		area, aspect_ratio, _ = area_aspect_ratio_center(info_contours)
		dist_block = distance_to_block_by(area, aspect_ratio)
		if dist_block == 'Too Close':
			#move back to the ideal range
			steps_to_advance = 1.5
			control_translation(reverse, steps_to_advance, record_pos)
		action_gripper(servo,'CLOSE')
	return False

def take_secure_block(camera_pi, color_block,record_pos):
	#take an image
	image = take_image(camera_pi)
	#verify the block is in the gripper
	block_caught = check_block_gripper(image, color_block)
	#move in reverse if block was caught
	if block_caught:
		Steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
		control_translation(reverse, Steps_to_advance , record_pos)

def move_to_goal(servo,camera_pi, imu_sensor, goal_coordinate, record_pos, block_sequence):
	#turn to next goal location
	vector_to_goal = get_vector(record_pos[-1], goal_coordinate)
	angle_to_goal = get_angle(vector_to_goal)
	#turn to the goal location
	control_rotation_imu(motor_pwm_setup, angle_to_goal, imu_sensor)
	#TODO: path planning to goal logic
	steps_to_advance = np.linalg.norm(vector_to_goal)
	control_translation(forward, steps_to_advance, record_pos)
	#open gripper
	action_gripper(servo,'OPEN')
	block_placed = block_sequence.pop()
	goal_coordinate.pop()
	#send email to notify the block was placed
	return send_email(camera_pi, block_placed)
def mode_search(record_pos, camera_pi, servo, color_block, imu_sensor, block_sequence):
	#when the robot at least has delivered one block going back to map
	if len(block_sequence) < TOTAL_BLOCKS:
		steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
		control_translation(reverse, steps_to_advance , record_pos)
		#close gripper
		action_gripper(servo,'CLOSE')
		#localize again to make sure the robot is in the right position
		localize()
	for value in SEARCH_SEQUENCE:
		#control the rotation of the robot to search for block
		control_rotation_imu(motor_pwm_setup, value, imu_sensor, record_pos)
		#take an image
		image = take_image(camera_pi)
		info_contours = inform_block(image, color_block)
		if info_contours:
			return info_contours
	return False

try:
	block_sequence = ['blue']
	goal_coordinate = [(83,15),(83,15),(83,15)]
	# Initialize GPIO pins
	init_gpio()
	# # Initialize camera
	camera = init_camera()
	# # Initialize IMU sensor
	imu_sensor = init_serial_read()
	# # Initialize servo
	servo = init_servo()
	# # Record positions
	record_pos = []
	for color_block in block_sequence:
		info_block = mode_search(record_pos, camera, servo, color_block, imu_sensor, block_sequence)
		#check if there are any blocks detected
		if not info_block:
			raise ValueError(f"No {color_block} block detected")
		print(f"Found {color_block} block")
		print(info_block)	
		aligned = align_with_block(info_block, imu_sensor,record_pos)
		if not aligned:
			raise ValueError(f"Failed to align with {color_block} block")
		print(f"Aligned with {color_block} block")
		reached_block = move_to_block(camera, servo, imu_sensor, color_block, record_pos)
		if not reached_block:
			continue
except KeyboardInterrupt:
	print("Stopping motors")
	#
except Exception as error:
    print(f"An error occurred: {error}")
finally:
	turn_off_motors()
	turn_off_servo(servo)
	camera.stop()
	camera.close()
	gameover()