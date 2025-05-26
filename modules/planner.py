from control_actions import *
from utilities_motors import *
from utilities_camera import *
from utilities_record import *
from utilities_perception import *
from utilities_sensors import *
from basic_init import *

#*------------------------
###*PARAMS FOR THE MAP
#*------------------------
TOTAL_MAP = np.array([304.8, 304.8])
INITIAL_POS_DEFAULT = (30.48, 30.48,0)
EDGE_ZONE_BLOCK = 30.48 #cm border of map for collision
TOTAL_BLOCKS = 3 #types of color blocks
LOCALIZE_SEQUENCE = [0,90,180,270,0]
SEARCH_SEQUENCE_STEPS = [0,45,-45,80,-80,0]

#*------------------------
###*OFFSETS FOR THE ROBOT
#*------------------------
OFFSET_ANGLE_image = 0.1#0.25 #1.5 #when  angle image is negative add the value, positive subtract
OFFSET_TO_GRIPPER = 22 #distance from imu to gripper center of grasping
OFFEST_SONAR_IMU = 8.5 #cm
OFFSET_GOAL = 10 #while position error accumulates set an extra margin distance to deliver blocks

#*------------------------
###*FOR AVOIDING OBSTACLES
#*------------------------
AVOID_ANGLE_FACTOR = 2.75
AVOID_ANGLE_DEFAULT = 25

def first_planner(block_sequence, record_pos, goal_coordinates, camera, imu_sensor, servo):
	reached_block = False
	has_block = False
	print('Start grand challenge t-t')
	while len(block_sequence) > 0:
		if len(block_sequence) == 4:
			print('Need to localize....')
			localize(imu_sensor, record_pos)
		if not has_block:
			found_block = mode_search(record_pos, camera, block_sequence[-1], imu_sensor)
			if not found_block:
				#advance a little and search again
				control_translation(forward, DEFAULT_ADVANCE_DISTANCE/2, record_pos,move_fast=True)
				print(f'PLANNER: Block {block_sequence[-1]} search derived in zero results, search again')
				continue
			reached_block = move_to_block(camera, servo, imu_sensor, block_sequence[-1], record_pos, goal_coordinates[-1])
			if not reached_block:
				print(f'PLANNER: Block {block_sequence[-1]} objective not found, search again')
				continue
			# getting_block = check_secured_block(camera, block_sequence[-1], record_pos)
			# if not getting_block:
			# 	print(f'PLANNER: Block {block_sequence[-1]} objective not found, search again')
			# 	continue
			has_block = True
		else:
			print(f'PLANNER: Block {block_sequence[-1]} redirecting to goal {goal_coordinates[-1]}')
			pending_blocks = move_to_goal(servo, camera, imu_sensor, goal_coordinates, record_pos, block_sequence)
			if pending_blocks:
				back_to_map(record_pos,servo, block_sequence[-1], imu_sensor)
				print(f'PLANNER: There are {len(block_sequence)} left.')
				has_block = False
	save_file_info(record_pos)
	print('Done all blocks wiiiiiii')


#*Creates a set of goalpoints for 9 blocks
def create_goal_coordinates(X_o, Y_o):
	#TODO: How to do this dynamically, it seems like a pascal tree related problem
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
	return goals_blocks[::-1]


def localize(imu_sensor, record_pos, dim_map = TOTAL_MAP):
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
	diff_hor = hor_distance - dim_map[0]
	diff_ver = ver_distance - dim_map[1]
	print(f'err hor distance is {diff_hor}')
	print(f'err ver distance is {diff_ver}')
	pos_x = record_pos[-1][0]
	pos_y = record_pos[-1][1]
	#To correct the position compare the errors if they are below a threshold
	if abs(diff_hor) <= 10 and abs(diff_ver) <=10:
		pos_x = pos_x - diff_hor if diff_hor > 0 else pos_x + diff_hor
		pos_y = pos_y - diff_ver if diff_ver > 0 else pos_y + diff_ver
	print(f'final x distance is {pos_x}')
	print(f'final y  distance is {pos_y}')
	record_pos.append((pos_x, pos_y, LOCALIZE_SEQUENCE[-1]))

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
	print(f'angle by camera: {angle_block}')
	current_angle = read_imu_yaw_angle(imu_sensor)
	print(f'current angle is {current_angle}')
	reference_angle = round(current_angle + angle_block, 2)
	reference_angle = normalize_angle(reference_angle)
	print(f'reference_angle is {reference_angle}')
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


def move_to_block(camera_pi, servo, imu_sensor, color_block, record_pos, goal_coord):
	#align with the block, advance again until the block is in reach
	steps_to_advance = 0
	range_block = 'Away'
	dist_block = 0
	action_gripper(servo,'OPEN')
	while range_block != 'Catch':
		had_to_avoid = avoid_hitting_wall(record_pos, imu_sensor)
		if had_to_avoid:
			action_gripper(servo,'CLOSE')
			print('Had to avoid hitting wall. Search again')
			return False
		print('Checking blocks obstacles')
		#avoid_other_blocks(record_pos, goal_coord, camera_pi, look_block = color_block)
		print('Finished avoiding obstacles search again')
		#for every advancement align with the block as much as possible
		aligned = align_with_block(camera_pi,imu_sensor,record_pos,color_block)
		if aligned:
			print('aligned to search for block', color_block)
			range_block , dist_block = get_distance_from_camera(camera_pi, color_block)
			if not dist_block:
				print('Lost track of block', color_block)
				return False
		action = forward
		if range_block == 'Stop':
			action = reverse
			steps_to_advance = MIN_RESOLUTION_LIN
			control_translation(action, steps_to_advance, record_pos)
		if range_block in ['Away','Far']:
			steps_to_advance = DEFAULT_ADVANCE_DISTANCE*4
		if range_block in ['Not sure']:
			steps_to_advance = dist_block/3
		else:
			steps_to_advance = dist_block - OFFSET_TO_GRIPPER
			if steps_to_advance < 0:
				steps_to_advance = DEFAULT_ADVANCE_DISTANCE/3
			elif steps_to_advance < 2:
				steps_to_advance = MIN_RESOLUTION_LIN*2
		move_fast = True if steps_to_advance >= DEFAULT_ADVANCE_DISTANCE*4 else False
		control_translation(action, steps_to_advance, record_pos, move_fast)
	control_translation(reverse, MIN_RESOLUTION_LIN*2, record_pos)
	action_gripper(servo,'CLOSE')
	time.sleep(0.1)
	return True

#*Check by ungrasp and grap again if a block is still there
def check_secured_block(camera_pi, color_block, record_pos):
	image = take_image(camera_pi)
	block_caught = check_block_gripper(image, color_block)
	#if i can confirm at first try it was caught open gripper and close again 3 times at least
	attempts = 0
	while not block_caught and attempts < 2:
		action_gripper(servo,'OPEN',soft=True)
		control_translation(reverse, MIN_RESOLUTION_LIN , record_pos)
		action_gripper(servo,'CLOSE')
		block_caught = check_block_gripper(image, color_block)
		attempts += 1
	if attempts >=2 and not block_caught:
		#go back a little
		control_translation(reverse, DEFAULT_ADVANCE_DISTANCE , record_pos)
		action_gripper(servo,'OPEN',soft=True)
		action_gripper(servo,'CLOSE')
		return False
	#move in reverse if block was caught
	Steps_to_advance = DEFAULT_ADVANCE_DISTANCE*2
	control_translation(reverse, Steps_to_advance , record_pos,move_fast=True)
	return True

#*check the risk of collision with obstacles
def risky_blocks(camera_pi, look_block=False):
	image = take_image(camera_pi)
	#*it means is looking for the goal instead
	if not look_block:
		image = hide_caught_block(image)
	blocks_found = []
	colors_to_search = list(DISTANCE_RANGES.keys())
	#*When looking for a specific color block these are not obstacles
	colors_to_search = [color for color in colors_to_search if color != look_block]
	for color_to_check in colors_to_search:
		block_color_contours = get_info_all_blocks(image, color_to_check)
		if not len(block_color_contours):
			continue
		#collect the contours found
		for contour_found in block_color_contours:
			blocks_found.append((contour_found, color_to_check))
	if not len(blocks_found):
		print('No risk to collide, keep going!')
		return False
	#*Calculate the angle of the block with respect to the center of the image
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
	#*if they are less than 20 degrees with respect to robot and a ditance of less than 35
	#*there is a risk of collision with them
	for block_risky in distances_angles:
		angle, distance, color  = block_risky
		if distance <= 35 and np.abs(angle) <= 20:
			print(f'block risky {color}', block_risky)
			risk_collision.append(angle)
	if not len(risk_collision):
		print('No risk to collide, keep going!')
		return False
	print('risk founds are', len(risk_collision),risk_collision)
	closest_dist = min(location[1] for location in distances_angles)
	print(f'closest dist is {closest_dist} cm')
	return risk_collision, closest_dist

#*A logic for avoid potential blocks in the way by turning to advance in the direction with less obstacles
def avoid_other_blocks(record_pos, goal_coord, camera_pi, look_block = False):
	results_risky_blocks = risky_blocks(camera_pi, look_block)
	if not results_risky_blocks:
		print('Continue the path')
		return True
	risk_collision, closest_dist = results_risky_blocks
	#*get the min distance to calculate the movement angle
	#*decide which direction to move according to where there is more obstacles
	#*left side positive angle
	#*right side negative angle
	left_side = []
	right_side = []
	for obstacle in risk_collision:
		left_side.append(obstacle) if obstacle >=0 else right_side.append(obstacle)
	print('left side are', len(left_side),left_side)
	print('right side are', len(right_side),right_side)
	if not len(left_side):
		left_side.append(AVOID_ANGLE_DEFAULT)
	if not len(right_side):
		right_side.append(-AVOID_ANGLE_DEFAULT)
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
	steps_to_advance = abs(closest_dist/np.cos(np.radians(angle_to_turn))) + EDGE_ZONE_BLOCK
	print(f'distance to advance is{steps_to_advance} cm')
	angle_to_turn = normalize_angle(angle_to_turn)
	print(f'rotate bot by deg {angle_to_turn} deg')
	control_rotation_imu(angle_to_turn, imu_sensor,record_pos)
	#check if there is risk to collide to wall after turning
	dist_to_wall = distance_sonar() + OFFEST_SONAR_IMU
	has_to_avoid = steps_to_advance > dist_to_wall
	if has_to_avoid:
		avoid_hitting_wall(record_pos, imu_sensor)
		print('had to avoid wall so expect to check this again soon')
		return True
	#advance a minimum safe distance
	last_angle = record_pos[-1][2]
	print('last angle is after turning is', last_angle)
	control_translation(forward, steps_to_advance, record_pos)
	#when taking block approximate where you might get a turn to do
	if look_block:
		print('aligning to find again block', look_block)
		angle_to_turn = last_angle - 90 - last_angle/3 if direction == 'turn left' else last_angle + 90 + last_angle/3
		angle_to_turn = normalize_angle(angle_to_turn)
		print('rotate to correct bot by', angle_to_turn)
		control_rotation_imu(angle_to_turn, imu_sensor, record_pos)
	else:
		print('avoid obstacles, aligning to goal coordinate', goal_coord)
		rotate_to_goal(imu_sensor, record_pos, goal_coord)
	return True

def rotate_to_goal(imu_sensor, record_pos, goal_coord,  rotate_fast = False):
	vector_to_goal = get_vector(record_pos[-1], goal_coord)
	print(f'vector to goal init is {vector_to_goal}')
	unitary_vector_goal = np.array(vector_to_goal/np.linalg.norm(vector_to_goal))
	#considering the offset of the gripper plus some radius of goal
	vector_to_goal = tuple(unitary_vector_goal*(np.linalg.norm(vector_to_goal) - OFFSET_TO_GRIPPER ))
	angle_to_goal = get_angle(vector_to_goal)
	angle_to_goal = normalize_angle(angle_to_goal)
	print(f'angle to goal is {angle_to_goal}')
	#turn to the goal location
	control_rotation_imu(angle_to_goal, imu_sensor, record_pos, rotate_fast)
	print('aligned to goal')
	return vector_to_goal

def advance_to_goal(record_pos, goal_coord,imu_sensor):
	vector_to_goal = rotate_to_goal(imu_sensor, record_pos, goal_coord)
	steps_to_advance = np.linalg.norm(vector_to_goal)
	print(f'steps to advance to goal is {steps_to_advance} cm')
	#the offset is due to an position error that acumulates over time while re localize
	control_translation(forward, steps_to_advance + OFFSET_GOAL, record_pos)

def move_to_goal(servo,camera_pi, imu_sensor, goal_coordinates, record_pos, block_sequence):
	advance_to_goal(record_pos, goal_coordinates[-1],imu_sensor)
	#*place block
	action_gripper(servo,'OPEN')
	block_placed = block_sequence.pop()
	#*send email to notify the block has been placed
	send_email(camera_pi, block_placed)
	goal_coordinates.pop()
	print(f'block placed is {block_placed}')
	if not len(block_sequence):
		return False
	return True

def avoid_hitting_wall(record_pos, imu_sensor):
	print('checking colliding....')
	risk_collide = False
	dist_sonar = distance_sonar()
	dist_sonar = dist_sonar + OFFEST_SONAR_IMU
	by_sonar =  dist_sonar < EDGE_ZONE_BLOCK*1.25
	by_position = False
	#check by its odometry it hasn't exceed the limits of the map
	last_x, last_y, _ = record_pos[-1]
	by_position = not((EDGE_ZONE_BLOCK/2 <= last_x <= TOTAL_MAP[0] - EDGE_ZONE_BLOCK) and \
	(EDGE_ZONE_BLOCK/2<= last_y <= TOTAL_MAP[1] - EDGE_ZONE_BLOCK))
	risk_collide = by_sonar or by_position
	if risk_collide:
		# Move back a bit
		print('Performing anti avoid hitting wall actions')
		control_translation(reverse, EDGE_ZONE_BLOCK*1.5, record_pos,move_fast=True)
		#move reverse
		angle_correct = normalize_angle(record_pos[-1][2]+180)
		control_rotation_imu(angle_correct, imu_sensor, record_pos,rotate_fast=True)
		return True
	return False

def generate_search_sequence(current_angle):
	sequence_to_derive = []
	for value_offset in SEARCH_SEQUENCE_STEPS:
		angle_search = value_offset + current_angle
		angle_search = normalize_angle(angle_search)
		sequence_to_derive.append(angle_search)
	return sequence_to_derive

def mode_search(record_pos, camera_pi,color_block, imu_sensor):
	sequence_to_search = generate_search_sequence(record_pos[-1][2])
	for value in sequence_to_search:
		#control the rotation of the robot to search for block
		print(f'Searching {color_block}....')
		control_rotation_imu(value, imu_sensor, record_pos,rotate_fast=True)
		#check surroundings for block
		image = take_image(camera_pi)
		suspect_block_found = inform_block(image, color_block, exhaustive=True)
		if suspect_block_found:
			print('look something was found')
			return suspect_block_found
	return False

def turn_back_robot(record_pos,imu_sensor):
	#rotate inverse
	last_angle = record_pos[-1][2]
	angle_inverse = last_angle + OFFSET_YAW//2
	angle_inverse = normalize_angle(angle_inverse)
	control_rotation_imu(angle_inverse, imu_sensor, record_pos, rotate_fast=True)
	return True

#*when the robot at least has delivered one block going back to map
def back_to_map(record_pos, servo, color_block, imu_sensor):
	steps_to_advance = DEFAULT_ADVANCE_DISTANCE
	control_translation(reverse, steps_to_advance , record_pos)
	#close gripper
	action_gripper(servo,'CLOSE')
	#localize again to make sure the robot is in the right position
	print(f'TIME TO RELOCALIZE GO BACK TO MAP , LOOKING NOW FOR {color_block}')
	# localize(imu_sensor, record_pos)
	turn_back_robot(record_pos, imu_sensor)
	return True

