from control_actions import *
from utilities_motors import *
from utilities_camera import *
from utilities_record import *
from utilities_perception import *
from utilities_sensors import *
from basic_init import *
import copy
import traceback
TOTAL_BLOCKS = 3
LOCALIZE_SEQUENCE = [0,90,180,270,0]
SEARCH_SEQUENCE = [0,20,340,45,315,0]
EDGE_ZONE_BLOCK = 30.48 #cm
TOTAL_MAP = np.array([115,98])#(365.76,365.76)#(115, 98)
CENTER_MAP = TOTAL_MAP / 2
OFFSET_ANGLE_image = 0.1#0.25 #1.5 #when  angle image is negative add the value, positive subtract
OFFSET_TO_GRIPPER = 22 #distance from imu to gripper center of grasping
OFFEST_SONAR_IMU = 8.5 #cm
AVOID_ANGLE_FACTOR = 2.75
RADIUS_GOAL = 15
AVOID_ANGLE_DEFAULT = 25
OFFSET_GOAL = OFFSET_TO_GRIPPER - RADIUS_GOAL
risk_of_collide = 0
has_block = False
#TODO: Maybe give mean of sonar distance of 10 measurements
#SEARCH_SEQUENCE = [0,20,45,90,120,0,-20,-45,-90,-120,0]
# TOTAL_MAP = (304.8,304.8) #edge of total square map
# OFFSET_LOCATE = 2 #cm
# CORNER_GOAL = (30.48, 335.28) #cm
# block_sequence = ['red','blue','green']
def first_planner(block_sequence, goal_coordinates, risk_of_collide):
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
		avoid_hitting_wall(record_pos, imu_sensor)
		if not has_block:
			found_block = mode_search(record_pos, camera, block_sequence[-1], imu_sensor, block_sequence)
			if not found_block:
				continue
			#check if there are any blocks detected
			aligned = align_with_block(camera,imu_sensor,record_pos,block_sequence[-1])
			if not aligned:
				continue
			reached_block = move_to_block(camera, servo, imu_sensor, block_sequence[-1], record_pos)
			if not reached_block:
				continue
			getting_block = take_secure_block(camera, block_sequence[-1], record_pos)
			if not getting_block:
				continue
		else:
			block_placed = move_to_goal(servo, imu_sensor, goal_coordinates[-1], record_pos, block_sequence)
			if not block_placed:
				continue
			back_to_map(record_pos, servo, block_sequence[-1], imu_sensor)
	turn_off_motors()
	turn_off_servo(servo)
	camera.stop()
	camera.close()
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
	for _, value in enumerate(LOCALIZE_SEQUENCE):
		control_rotation_imu(value, imu_sensor,record_pos)
		distance_saved = distance_sonar_average() + OFFEST_SONAR_IMU
		distances_calibration.append(distance_saved) #10 cm offset in x direction to imu
		print(f'localize for {value} is {distance_saved}cm')
	hor_distance = distances_calibration[0] + distances_calibration[2]
	ver_distance = distances_calibration[1] + distances_calibration[3]
	print(f'hor distance is {hor_distance}')
	print(f'ver distance is {ver_distance}')
	diff_hor = hor_distance - TOTAL_MAP[0]
	diff_ver = ver_distance - TOTAL_MAP[1]
	print(f'err hor distance is {diff_hor}')
	print(f'err ver distance is {diff_ver}')
	pos_x = record_pos[-1][0]
	pos_y = record_pos[-1][1]
	#an attempt to correct the position
	pos_x = pos_x - diff_hor if diff_hor > 0 else pos_x + diff_hor
	pos_y = pos_y - diff_ver if diff_ver > 0 else pos_y + diff_ver
	print(f'final x distance is {pos_x}')
	print(f'final y  distance is {pos_y}')
	record_pos.append((pos_x, pos_y, LOCALIZE_SEQUENCE[-1]))
	return 'Done'

def align_with_block(camera_pi, imu_sensor, record_pos, color_block):
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
	control_rotation_imu(reference_angle, imu_sensor, record_pos)
	return True

def get_distance_from_camera(camera_pi, color_block):
	image = take_image(camera_pi)
	info_block = inform_block(image, color_block)
	#check if there are any blocks detected
	if info_block:
		area, aspect_ratio, _= area_aspect_ratio_center(info_block)
		print(f'aspect ratio block  {color_block} is', aspect_ratio)
		range_block , dist_block = obtain_distance(color_block, area, aspect_ratio)
		print(f'distance block  {color_block} with range {range_block}', dist_block)
		return range_block ,dist_block
	return False, False


def move_to_block(camera_pi, servo, imu_sensor, color_block, record_pos):
	# range_block, dist_block = get_distance_from_camera(camera_pi, color_block)
	# if not dist_block:
	# 	return False
		#open gripper
	#advance one revolution, align with the block and advance again until the block is in reach
	steps_to_advance = 0
	range_block = 'Away'
	dist_block = 0
	while range_block != 'Catch':
		#for every advancement align with the block as much as possible
		aligned = align_with_block(camera_pi,imu_sensor,record_pos,color_block)
		if aligned:
			range_block , dist_block = get_distance_from_camera(camera_pi, color_block)
			if not dist_block:
				return False
		action = forward
		# if range_block in ['Away','Remote']:
		# 	steps_to_advance = DEFAULT_ADVANCE_DISTANCE*3
		# elif range_block == 'Far':
		# 	steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
		# elif range_block == 'Near':
		# 	steps_to_advance = DEFAULT_ADVANCE_DISTANCE
		# if range_block == 'Close':
		# 	steps_to_advance = DEFAULT_ADVANCE_DISTANCE/3
		if range_block == 'Close':
			action_gripper(servo,'OPEN')
		if range_block == 'Stop':
			action = reverse
			steps_to_advance = MIN_RESOLUTION_LIN
			control_translation(action, steps_to_advance, record_pos)
			action_gripper(servo,'CLOSE')
			return True
		#TODO: Check if consider the number of blocks left less than 3 to change this 
		if range_block in ['Away','Far']:
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE*4
		if range_block == 'Not sure':
			steps_to_advance = dist_block/4
		else:
			steps_to_advance = dist_block - OFFSET_TO_GRIPPER
			if steps_to_advance < 0:
				steps_to_advance = DEFAULT_ADVANCE_DISTANCE/4
		control_translation(action, steps_to_advance, record_pos)
	time.sleep(0.05)
	action_gripper(servo,'CLOSE')
	return True

def check_wall(risk_of_collide):
	almost_hit_wall = avoid_hitting_wall(record_pos, imu_sensor)
	if almost_hit_wall:
		risk_of_collide +=1
		return True


def check_if_lost(risk_of_collide):
	if risk_of_collide >=3:
		localize()
		risk_of_collide = 0

def take_secure_block(camera_pi, color_block, record_pos):
	#take an image
	image = take_image(camera_pi)
	#verify the block is in the gripper
	block_caught = check_block_gripper(image, color_block)
	#if i can confirm at first try it was caught open gripper and close again 3 times at least
	attempts = 0
	while not block_caught and attempts < 2:
		action_gripper(servo,'OPEN')
		control_translation(reverse, MIN_RESOLUTION_LIN , record_pos)
		action_gripper(servo,'CLOSE')
		block_caught = check_block_gripper(image, color_block)
		attempts += 1
	if attempts >=2 and not block_caught:
		#go back a little 
		control_translation(reverse, DEFAULT_ADVANCE_DISTANCE , record_pos)
		has_block = False
		return False
	#move in reverse if block was caught
	Steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
	control_translation(reverse, Steps_to_advance , record_pos)
	has_block = True
	return True

def risky_blocks(camera_pi,take_block=False):
	image = take_image(camera_pi)
	#look for blocks of different colors - obstacles
	if take_block:
		image = hide_caught_block(image)
	blocks_found = []
	for color_to_check in list(DISTANCE_RANGES.keys()):
		block_color_contour = inform_block(image, color_to_check)
		if block_color_contour:
			blocks_found.append((block_color_contour, color_to_check))
	if not len(blocks_found):
		return True
	#calculate the angle of the block with respect to the center of the image
	print('block founds are', blocks_found)
	distances_angles = []
	for info_contour in blocks_found:
		area, aspect_ratio, center = area_aspect_ratio_center(info_contour[0])
		angle_block = angle_block_gripper_by(center)
		_ , dist_block = obtain_distance(info_contour[1], area, aspect_ratio)
		distances_angles.append((angle_block, dist_block))
	print('location founds are', distances_angles)
	risk_collision = []
	#check their angles
	#if they are less than 15 degrees with respect to robot angle turn 20 degrees and a margin distance to settle(Near)
	for block_risky in distances_angles:
		angle, distance  = block_risky
		if distance <= 35 and np.abs(angle) <= 18:
			print('block risky', block_risky)
			risk_collision.append(angle)
	if not len(risk_collision):
		print('No risk to collide, keep going!')
		return True


def avoid_other_blocks(record_pos,camera_pi, take_block = False):
	image = take_image(camera_pi)
	#look for blocks of different colors - obstacles
	if take_block:
		image = hide_caught_block(image)
	blocks_found = []
	for color_to_check in list(DISTANCE_RANGES.keys()):
		block_color_contour = inform_block(image, color_to_check)
		if block_color_contour:
			blocks_found.append((block_color_contour, color_to_check))
	if not len(blocks_found):
		return True
	#calculate the angle of the block with respect to the center of the image
	print('block founds are', blocks_found)
	distances_angles = []
	for info_contour in blocks_found:
		contour_info, color_check = info_contour
		area, aspect_ratio, center = area_aspect_ratio_center(contour_info)
		angle_block = angle_block_gripper_by(center)
		_ , dist_block = obtain_distance(color_check, area, aspect_ratio)
		distances_angles.append((angle_block, dist_block, color_check))
	print('location founds are', distances_angles)
	risk_collision = []
	#check their angles
	#if they are less than 15 degrees with respect to robot angle turn 20 degrees and a margin distance to settle(Near)
	for block_risky in distances_angles:
		angle, distance, color  = block_risky
		if distance <= 35 and np.abs(angle) <= 18:
			print(f'block risky {color}', block_risky)
			risk_collision.append(angle)
	if not len(risk_collision):
		print('No risk to collide, keep going!')
		return True	
	print('risk founds are', len(risk_collision),risk_collision)
	#get the min distance to calculate the movement angle
	closest_dist = min(location[1] for location in distances_angles)
	#decide which direction to move according to where there is more obstacles
	#*left side positive angle
	#*right side negative angle
	left_side = []
	right_side = []
	for obstacle in risk_collision:
		left_side.append(obstacle) if obstacle >=0 else right_side.append(obstacle)
	print('left side are', len(left_side),left_side)
	print('right side are', len(right_side),right_side)
	if not len(left_side):
		left_side.append(20)
	if not len(right_side):
		right_side.append(-20)
	#if there are more obstacles on right turn the min double the angle detected on left
	if len(right_side) >= len(left_side):
		angle_to_turn = max(left_side)*AVOID_ANGLE_FACTOR
		direction = 'turn left'
	else:
		angle_to_turn = min(right_side)*AVOID_ANGLE_FACTOR
		direction = 'turn right'
	print('direction is', direction)
	#turn to the corresponding angle
	angle_to_turn = angle_to_turn + record_pos[-1][2]
	steps_to_advance = closest_dist/np.cos(np.radians(angle_to_turn)) + DEFAULT_ADVANCE_DISTANCE*2
	print('distance to advance is', steps_to_advance)
	angle_to_turn = normalize_angle(angle_to_turn)
	print('rotate bot by', angle_to_turn)
	control_rotation_imu(angle_to_turn, imu_sensor,record_pos)
	#advance a minimum safe distance 
	last_angle = record_pos[-1][2]
	print('last angle is', last_angle)
	control_translation(forward, steps_to_advance, record_pos)
	if take_block:
		angle_to_turn = 2*last_angle - 90 if direction == 'turn left' else 2*last_angle + 90
		angle_to_turn = normalize_angle(angle_to_turn)
		print('rotate to correct bot by', angle_to_turn)
		control_rotation_imu(angle_to_turn, imu_sensor, record_pos)
	else:
		rotate_to_goal(imu_sensor, record_pos, goal_coordinate)
	return True

def rotate_to_goal(imu_sensor, record_pos, goal_coordinate):
	vector_to_goal = get_vector(record_pos[-1], goal_coordinate[-1])
	unitary_vector_goal = np.array(vector_to_goal/np.linalg.norm(vector_to_goal))
	#consider the offset of the gripper plus some radius of goal 
	vector_to_goal = tuple(unitary_vector_goal*(np.linalg.norm(vector_to_goal) - OFFSET_GOAL ))
	angle_to_goal = get_angle(vector_to_goal)
	print(f'angle to goal is {angle_to_goal}')
	print(f'vector to goal is {vector_to_goal}')
	#turn to the goal location
	control_rotation_imu(angle_to_goal, imu_sensor, record_pos)
	return vector_to_goal

def advance_to_goal(record_pos, goal_coordinate,imu_sensor):
	vector_to_goal = rotate_to_goal(imu_sensor, record_pos, goal_coordinate)
	steps_to_advance = np.linalg.norm(vector_to_goal)
	print(f'steps to advance is {steps_to_advance}')
	if not steps_to_advance < 30.48: 
		control_translation(forward, DEFAULT_ADVANCE_DISTANCE*2, record_pos)
	return steps_to_advance

def move_to_goal(servo,camera_pi, imu_sensor, goal_coordinate, record_pos, block_sequence):
	#turn to next goal location
	block_delivered = False
	steps_to_advance = np.inf
	
	while not block_delivered:
		if steps_to_advance > 120.98:
			steps_to_advance = advance_to_goal(record_pos, goal_coordinate,imu_sensor)
			# safe_to_move = avoid_other_blocks(record_pos,camera_pi)
			# if safe_to_move:
			# 	steps_to_advance = advance_to_goal(record_pos, goal_coordinate,imu_sensor)
		elif 120.98 <= steps_to_advance <= 30.48:
			steps_to_advance = advance_to_goal(record_pos, goal_coordinate,imu_sensor)
		else:
			#open gripper and place object as the place zone has been reached 
			action_gripper(servo,'OPEN')
			#*send email to notify the block has been placed
			block_placed = block_sequence.pop()
			#send_email(camera_pi, block_placed)
			goal_coordinate.pop()
			print(f'block placed is {block_placed}')
			print(f'block sequence pending is', block_sequence)
			return True

def avoid_hitting_wall(record_pos, imu_sensor):
	if distance_sonar() < EDGE_ZONE_BLOCK:
		# Move back a bit
		control_translation(reverse, EDGE_ZONE_BLOCK*2, record_pos)
		#aim to center
		return turn_back_robot(record_pos,imu_sensor)
	return False

def mode_search(record_pos, camera_pi,color_block, imu_sensor):
	for value in SEARCH_SEQUENCE:
		#control the rotation of the robot to search for block
		control_rotation_imu(value, imu_sensor, record_pos)
		#take an image
		image = take_image(camera_pi)
		info_contours = inform_block(image, color_block)
		if info_contours:
			return info_contours
	return False

def turn_back_robot(record_pos,imu_sensor):
	#rotate inverse
	last_angle = record_pos[-1][2]
	angle_inverse = last_angle + OFFSET_YAW//2
	angle_inverse = normalize_angle(angle_inverse)
	control_rotation_imu(angle_inverse, imu_sensor, record_pos)
	return True

def back_to_map(record_pos, servo,color_block, imu_sensor):
	#when the robot at least has delivered one block going back to map
	steps_to_advance = DEFAULT_ADVANCE_DISTANCE
	control_translation(reverse, steps_to_advance , record_pos)
	#close gripper
	action_gripper(servo,'CLOSE')
	#localize again to make sure the robot is in the right position
	print(f'TIME TO RELOCALIZE GO BACK TO MAP , LOOKING NOW FOR {color_block}')
	# localize()
	turn_back_robot(record_pos, imu_sensor)
	has_block = False
	return True
try:
	block_sequence = ['blue','green','red']#*TOTAL_BLOCKS
	bl_seq_copy = copy.deepcopy(block_sequence)
	# Initialize GPIO pins
	init_gpio()
	# # Initialize camera
	camera_pi_4 = init_camera()
	# # Initialize IMU sensor
	imu_sensor = init_serial_read()
	# Initialize servo
	servo = init_servo()
	# # Record positions
	record_pos = []
	#record_pos.append((60.96,60.96,0))
	init_position = (0,0,0)
	record_pos.append(init_position)
	goal_coordinate =  [(190,90)]*TOTAL_BLOCKS#create_goal_coordinate(*init_position[0:1])#[(83,15),(83,15),(83,15)][0,90](60.96,304.8)
	# avoid_other_blocks(record_pos,camera, True)
	# for color_block in block_sequence[::-1]:
	# 	info_block = mode_search(record_pos, camera_pi_4,color_block, imu_sensor)
	# 	#check if there are any blocks detected
	# 	if not info_block:
	# 		continue
	# 	print(f"Found {color_block} block")
	# 	print(info_block)
	# 	# aligned = align_with_block(camera_pi_4,imu_sensor,record_pos,color_block)
	# 	# if not aligned:
	# 	# 	raise ValueError(f"Failed to align with {color_block} block")
	# 	# print(f"Aligned with {color_block} block")
	# 	reached_block = move_to_block(camera_pi_4, servo, imu_sensor, color_block, record_pos)
	# 	print('Reached block was', reached_block)
	# 	if not reached_block:
	# 		continue
	# 		#raise ValueError(f"Failed to reach {color_block} block")
	# 	print(f"Reached {color_block} block")
	# 	getting_block = take_secure_block(camera_pi_4, block_sequence[-1], record_pos)
	# 	if not getting_block:
	# 		continue
	# 		#raise ValueError(f"Failed to grab {color_block} block")
	# 	placed_block = move_to_goal(servo, camera_pi_4, imu_sensor, goal_coordinate , record_pos, bl_seq_copy)
	# 	if not placed_block:
	# 		raise ValueError(f"Failed to place {color_block} block")
	# 	back_to_map(record_pos, servo,color_block, imu_sensor)
	# save_file_info(record_pos)
except KeyboardInterrupt:
	print("Stopping motors")
except Exception as error:
	print(f"An error occurred: {error}")
	traceback.print_exc() 
finally:
	turn_off_motors()
	turn_off_servo(servo)
	camera_pi_4.stop()
	camera_pi_4.close()
	gameover()
