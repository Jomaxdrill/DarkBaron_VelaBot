from control_actions import *
from utilities_motors import *
from utilities_camera import *
from utilities_record import *
from utilities_perception import *
from basic_init import *
import copy
TOTAL_BLOCKS = 3
LOCALIZE_SEQUENCE = [0,90,180,-90,0]
SEARCH_SEQUENCE = [0,20,-20,45,-45,0]
EDGE_ZONE_BLOCK = 30.48 #cm
TOTAL_MAP = (115, 98)
OFFSET_ANGLE_image = 0.1#0.25 #1.5 #when  angle image is negative add the value, positive subtract
OFFSET_TO_GRIPPER = 23 #distance from imu to gripper center of grasping
OFFEST_SONAR_IMU = 8.5 #cm
has_block = False
#TODO: Maybe give mean of sonar distance of 10 measurements
#SEARCH_SEQUENCE = [0,20,45,90,120,0,-20,-45,-90,-120,0]
# TOTAL_MAP = (304.8,304.8) #edge of total square map
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
		#TODO: for now check how it behaves the avoiding with sonar
		avoid_hitting_wall(record_pos)
		found_block = mode_search(record_pos, camera, block_sequence[-1], imu_sensor, block_sequence)
		if not found_block:
			continue
		#check if there are any blocks detected
		aligned = align_with_block(camera,imu_sensor,record_pos)
		if not aligned:
			continue
		reached_block = move_to_block(camera, servo, imu_sensor, block_sequence[-1], record_pos)
		if not reached_block:
			continue
		getting_block = take_secure_block(camera, block_sequence[-1], record_pos)
		if not getting_block:
			continue
		block_placed = move_to_goal(servo, imu_sensor, goal_coordinates[-1], record_pos, block_sequence)
		if not block_placed:
			continue
		back_to_map(record_pos, servo, color_block, imu_sensor)
	gameover()
	return 'Done all blocks wiiiiiii'

def create_goal_coordinate(X_o, Y_o):
	#TODO: Maybe one time check how to do this dynamically, it seems like a pascal tree related problem
	goals_blocks = [
		(X_o + 0.5*EDGE_ZONE_BLOCK,Y_o - 0.5*EDGE_ZONE_BLOCK), #first block
		(X_o + 0.5*EDGE_ZONE_BLOCK,Y_o - 1.5*EDGE_ZONE_BLOCK), #second block
		(X_o + 1.5*EDGE_ZONE_BLOCK,Y_o - 0.5*EDGE_ZONE_BLOCK), #third block
		(X_o + 0.5*EDGE_ZONE_BLOCK,Y_o - 2.5*EDGE_ZONE_BLOCK), #fourth block
		(X_o + 1.5*EDGE_ZONE_BLOCK,Y_o - 1.5*EDGE_ZONE_BLOCK), #fifth block
		(X_o + 2.5*EDGE_ZONE_BLOCK,Y_o - 0.5*EDGE_ZONE_BLOCK), #sixth block
		(X_o + 1.5*EDGE_ZONE_BLOCK,Y_o - 2.5*EDGE_ZONE_BLOCK), #seventh block
		(X_o + 2.5*EDGE_ZONE_BLOCK,Y_o - 1.5*EDGE_ZONE_BLOCK), #eighth block
		(X_o + 2.5*EDGE_ZONE_BLOCK,Y_o - 2.5*EDGE_ZONE_BLOCK), #ninth block
		]
	return goals_blocks[::-1] #return inverted
def localize(imu_sensor, record_pos):
	distances_calibration = []
	for value in LOCALIZE_SEQUENCE:
		control_rotation_imu(motor_pwm_setup, value, imu_sensor,record_pos)
		distances_calibration.append(distance_sonar() + OFFEST_SONAR_IMU) #10 cm offset in x direction to imu
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

def align_with_block(camera_pi, imu_sensor, record_pos):
	image = take_image(camera_pi)
	info_contours = inform_block(image, color_block)
	if not info_contours:
		print('while reaching the block we lost track of it')
		return False
	_, _,center = area_aspect_ratio_center(info_contours)
	#calculate the angle of the block with respect to the center of the image
	angle_block = angle_block_gripper_by(center)
	angle_block = angle_block - OFFSET_ANGLE_image if angle_block <= 0 else angle_block + OFFSET_ANGLE_image
	print(f'angle: {angle_block}')
	current_angle = read_imu_yaw_angle(imu_sensor)
	print(f'current angle is {current_angle}')
	#TODO: check if angle control can be changed as move until the block is in the center of the image
	reference_angle = round(current_angle + angle_block, 2)
	#normalize angle to the corresponding system
	reference_angle = normalize_angle(reference_angle)
	print(f'reference_angle is {reference_angle}')
	#control the rotation of the robot to align with the block
	control_rotation_imu(motor_pwm_setup, reference_angle, imu_sensor, record_pos)
	return True

def get_distance_from_camera(camera_pi, color_block):
	image = take_image(camera_pi)
	info_block = inform_block(image, color_block)
	#check if there are any blocks detected
	if info_block:
		area, aspect_ratio, _= area_aspect_ratio_center(info_block)
		dist_block = distance_to_block_by(area, aspect_ratio)
		return dist_block
	return False

def move_to_block(camera_pi, servo, imu_sensor, color_block, record_pos):
	dist_block = get_distance_from_camera(camera_pi, color_block)
	if not dist_block:
		return False
		#open gripper
	#advance one revolution, align with the block and advance again until the block is in reach
	action_gripper(servo,'OPEN')
	steps_to_advance = 0
	while dist_block != 'Catch':
		action = forward
		steps_to_advance = dist_block
		if dist_block in ['Away','Remote']:
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE*3
		elif dist_block == 'Far':
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
		elif dist_block == 'Near':
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE
		elif dist_block == 'Close':
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE/2
		elif dist_block == 'Stop':
			action = reverse
			steps_to_advance = 1
			control_translation(action, steps_to_advance, record_pos)
			action_gripper(servo,'CLOSE')
			return True
		control_translation(action, steps_to_advance, record_pos)
		#for every advancement align with the block as much as possible
		aligned = align_with_block(camera_pi,imu_sensor,record_pos)
		if aligned:
			dist_block = get_distance_from_camera(camera_pi, color_block)
		avoid_hitting_wall(record_pos)
	action_gripper(servo,'CLOSE')
	return True

def take_secure_block(camera_pi, color_block, record_pos):
	#take an image
	image = take_image(camera_pi)
	#verify the block is in the gripper
	block_caught = check_block_gripper(image, color_block)
	#if i can confirm at first try it was caught open gripper and close again 3 times at least
	attempts = 0
	while not block_caught and attempts < 3:
		action_gripper(servo,'OPEN')
		action_gripper(servo,'CLOSE')
		block_caught = check_block_gripper(image, color_block)
	if attempts >=3 and not block_caught:
		return False
	has_block = True
	#move in reverse if block was caught
	Steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
	control_translation(reverse, Steps_to_advance , record_pos)
	return True

def avoid_other_blocks():
    #hide the zone of the caught block always 
    #look for blocks of different colors - obstacles
    #check their angles
    #if they are less than 15 degrees with respect to robot angle turn 20 degrees and a margin distance to settle(Near)
    
    #if there are no more block with angle less than 20 degrees calculate angle to goal again and move two revolutions
    #do this until goal is reached
    pass
    

def move_to_goal(servo,camera_pi, imu_sensor, goal_coordinate, record_pos, block_sequence):
	#turn to next goal location
	vector_to_goal = get_vector(record_pos[-1], goal_coordinate[-1])
	unitary_vector_goal = np.array(vector_to_goal/np.linalg.norm(vector_to_goal))
	#consider the offset of the gripper
	vector_to_goal = tuple(unitary_vector_goal*(np.linalg.norm(vector_to_goal) - OFFSET_TO_GRIPPER))
	angle_to_goal = get_angle(vector_to_goal)
	print(f'angle to goal is {angle_to_goal}')
	print(f'vector to goal is {vector_to_goal}')
	#turn to the goal location
	control_rotation_imu(motor_pwm_setup, angle_to_goal, imu_sensor)
	#TODO: path planning to goal logic
	steps_to_advance = np.linalg.norm(vector_to_goal)
	print(f'steps to advance is {steps_to_advance}')
	# if steps_to_advance > 100:
	# 	number_steps = int(steps_to_advance/100)
	# 	steps_to_advance = 100
	# 	for idx in range(number_steps):
	# 		#check if the robot is going to hit a wall
	# 		avoid_hitting_wall(record_pos)
	# 		#move forward
	# 		control_translation(forward, steps_to_advance, record_pos)
	control_translation(forward, steps_to_advance, record_pos)
	#open gripper
	action_gripper(servo,'OPEN')
	#send email to notify the block was placed
	#send_email(camera_pi, block_placed)
	block_placed = block_sequence.pop()
	goal_coordinate.pop()
	print(f'block placed is {block_placed}')
	return True

def avoid_hitting_wall(record_pos):
	if distance_sonar() < EDGE_ZONE_BLOCK:
		# Move back a bit
		control_translation(reverse, EDGE_ZONE_BLOCK, record_pos)

def mode_search(record_pos, camera_pi,color_block, imu_sensor):
	for value in SEARCH_SEQUENCE:
		#control the rotation of the robot to search for block
		control_rotation_imu(motor_pwm_setup, value, imu_sensor, record_pos)
		#take an image
		image = take_image(camera_pi)
		info_contours = inform_block(image, color_block)
		if info_contours:
			return info_contours
	return False

def back_to_map(record_pos, servo,color_block, imu_sensor):
	#when the robot at least has delivered one block going back to map
	steps_to_advance = DEFAULT_ADVANCE_DISTANCE
	control_translation(reverse, steps_to_advance , record_pos)
	#close gripper
	action_gripper(servo,'CLOSE')
	#localize again to make sure the robot is in the right position
	print(f'TIME TO RELOCALIZE GO TO POSITION 0 , LOOKING NOW FOR {color_block}')
	# localize()
	#rotate inverse
	angle_inverse = record_pos[-1][2] - OFFSET_YAW//2
	angle_inverse = normalize_angle(angle_inverse)
	control_rotation_imu(motor_pwm_setup, angle_inverse, imu_sensor, record_pos)
	return True
try:
	block_sequence = ['blue','green','red']
	bl_seq_copy = copy.deepcopy(block_sequence)
	goal_coordinate =  [(60.96,304.8)]*TOTAL_BLOCKS#[(83,15),(83,15),(83,15)][0,90]
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
	record_pos.append((30.48,30.48,0))
	for color_block in block_sequence[::-1]:
		info_block = mode_search(record_pos, camera,color_block, imu_sensor)
		#check if there are any blocks detected
		if not info_block:
			continue
		print(f"Found {color_block} block")
		print(info_block)
		aligned = align_with_block(camera,imu_sensor,record_pos)
		if not aligned:
			raise ValueError(f"Failed to align with {color_block} block")
		print(f"Aligned with {color_block} block")
		reached_block = move_to_block(camera, servo, imu_sensor, color_block, record_pos)
		if not reached_block:
			continue
			#raise ValueError(f"Failed to reach {color_block} block")
		print(f"Reached {color_block} block")
		getting_block = take_secure_block(camera, block_sequence[-1], record_pos)
		if not getting_block:
			continue
			#raise ValueError(f"Failed to grab {color_block} block")
		placed_block = move_to_goal(servo, camera, imu_sensor, goal_coordinate , record_pos, bl_seq_copy)
		if not placed_block:
			raise ValueError(f"Failed to place {color_block} block")
		back_to_map(record_pos, servo,color_block, imu_sensor)
	save_file_info(record_pos)
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