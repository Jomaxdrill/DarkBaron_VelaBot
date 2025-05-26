import RPi.GPIO as gpio
import numpy as np
import time
from .constants_pi import ENCODER_LEFT,ENCODER_RIGHT
from .utilities_sensors import OFFSET_YAW, read_imu_yaw_angle,normalize_angle,distance_sonar
from .utilities_motors import turn_off_motors, motor_pwm_setup

#*------------------------
###* DEFAULTS FOR ACTIONS AND USE OF ENCODER
#*------------------------
DEFAULT_ADVANCE_DISTANCE = 20.42  # cm
DEFAULT_CLOSE_DISTANCE = 6.0  # cm
STEPS = 20 #steps equivalent to one revolution of the wheel
MIN_RESOLUTION_LIN = DEFAULT_ADVANCE_DISTANCE / STEPS #cm aprox watch means advance 1 step of encoder
MIN_RESOLUTION_ROT = 4.5 #degrees aprox watch means rotate 1 step of encoder
UNITARY_VECTOR_X = [1,0] #the angle is always respect to the x axis

#*------------------------
###*CONTROL OF PD GAINS
#*------------------------
K_linear = 1.25
K_ROT_IMU = 1.15
K_D_ROT_IMU = 0.35
ERROR_STEPS = 1

#*------------------------
###*DUTY CYCLE FOR SATURATION
#*------------------------
PWM_LINEAR_SLOW = 40
PWM_LINEAR_FAST = 55
PWM_ROT_MIN = 55 #Below 55 it could potentially not move
PWM_ROT_MIN2 = 70


#*Crucial for recording position and orientation
def transformation_robot_to_world(angle, position):
	pos_x, pos_y = position
	angle_rad = np.radians(angle)
	rot = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
					[np.sin(angle_rad), np.cos(angle_rad)]])
	return np.array([
		[rot[0,0], rot[0,1], pos_x],
		[rot[1,0], rot[1,1], pos_y],
		[0, 0, 1]
	])

def get_angle(vector_a):
	angle_dot_product = np.arccos(np.dot(vector_a, UNITARY_VECTOR_X)/(np.linalg.norm(vector_a)))
	angle = normalize_angle(np.degrees(angle_dot_product))
	return angle

def get_vector(node_a, node_b):
	return (node_b[0] - node_a[0], node_b[1] - node_a[1])

def control_translation(action, reference, history, move_fast = False):
	if not reference:
		return history
	pwm_move = PWM_LINEAR_FAST if move_fast else PWM_LINEAR_SLOW
	last_position = history[-1] if history else (0.0, 0.0, 0.0)
	pos_x, pos_y, angle = last_position
	transf_matrix = transformation_robot_to_world(angle, (pos_x, pos_y))
	counter_R = np.uint64(0)
	counter_L = np.uint64(0)
	button_R = int(0)
	button_L = int(0)
	ticks = 0
	advancement = 0
	duty_cycle = 0
	pwm_left, pwm_right, flag_type = action()
	pwm_left.start(pwm_move) # USE % OF duty cycle
	pwm_right.start(pwm_move) # USE % OF duty cycle
	print(f'reference linear is {reference} cm')
	steps_reference = round(reference/MIN_RESOLUTION_LIN)
	print(f'cycles encoder reference: {steps_reference} ')
	while ticks < steps_reference:
		#print(f'sonar:{distance_sonar()} cm')
		if int(gpio.input(ENCODER_RIGHT)) != int(button_R):
			button_R = int(gpio.input(ENCODER_RIGHT))
			counter_R += 1
			#print(counter_R)
		if int(gpio.input(ENCODER_LEFT)) != int(button_L):
			button_L = int(gpio.input(ENCODER_LEFT))
			counter_L += 1
			#print(counter_L)
		diff_counter = counter_L - counter_R
		#print('encoder error',diff_counter)
		duty_cycle =  pwm_move - K_linear* abs(diff_counter)
		#print(f'duty cycle is: {duty_cycle}')
		#left motor is going faster than right motor
		if diff_counter >= ERROR_STEPS:
			pwm_left.ChangeDutyCycle(duty_cycle)
		#right motor is going faster than left motor
		if diff_counter <= -ERROR_STEPS:
			pwm_right.ChangeDutyCycle(duty_cycle)
		ticks = round((counter_R + counter_L)/2)
		advancement = ticks*MIN_RESOLUTION_LIN
		#print(f'\ntraveled: {advancement}cm\n')
		course_advancement = flag_type * advancement
		#transform to the world system the coordinates of the robot
		advance_world = transf_matrix @ np.array([course_advancement, 0, 1])
		#print('advance world:', advance_world)
		pos_to_record = (*advance_world[0:2], angle)
		if pos_to_record != history[-1]:
			history.append(pos_to_record)
	pwm_left.stop()
	pwm_right.stop()
	print(f'success performing {advancement} cm\n')
	turn_off_motors()
	time.sleep(0.25)
	return history

def control_rotation_imu(reference, sensor_imu, history, rotate_fast = False):
	last_position = history[-1] if history else (0.0, 0.0, 0)
	min_duty_cycle = PWM_ROT_MIN2 if rotate_fast else PWM_ROT_MIN
	error_tolerance = 4 if rotate_fast else 1.5
	error_reference = np.inf
	prev_time = None
	derivative = 0
	current_time = None
	prev_time = None
	prev_error = 0
	duty_cycle = 0
	pwm_left_1, pwm_left_2, pwm_right_1, pwm_right_2 = motor_pwm_setup()
	pwm_left_1.start(0)
	pwm_left_2.start(0)
	pwm_right_1.start(0)
	pwm_right_2.start(0)
	while abs(error_reference) >= error_tolerance:
		yaw = read_imu_yaw_angle(sensor_imu)
		if yaw is None:
			continue
		#print('yaw sensor:', yaw)
		error_reference = reference - yaw
		#print('rot_error:', error_reference)
		#normalize the error when it has to turn more than 180 degrees
		if abs(error_reference) > OFFSET_YAW//2:
			error_reference = error_reference + OFFSET_YAW if error_reference < 0 else error_reference - OFFSET_YAW
			#print('error normalized is',error_reference)
		current_time = time.time()
		# Calculate derivative
		if prev_time is None:  # First iteration
			derivative = 0
			diff_time = 0.001  # Default small time step
		else:
			diff_time = current_time - prev_time
			#print('diff time', diff_time)
			if diff_time > 0:
				derivative = (error_reference - prev_error) / diff_time
			else:
				derivative = 0
		#print('derivative is:', derivative)
		duty_cycle = abs(error_reference)*K_ROT_IMU - derivative*K_D_ROT_IMU
		#print('calculated duty cycle', duty_cycle)
		#print(f'control signal: {duty_cycle}')
		#Apply saturation as duty cycle can't be negative or lower certain threshold to work
		duty_cycle = max(min(duty_cycle, 100),min_duty_cycle)#70 #55 in full power
		#print('real duty cycle', duty_cycle)
		#turn to left direction
		if error_reference > 0:
				pwm_left_1.ChangeDutyCycle(0)
				pwm_left_2.ChangeDutyCycle(duty_cycle)
				pwm_right_1.ChangeDutyCycle(0)
				pwm_right_2.ChangeDutyCycle(duty_cycle)
		# turn to right direction
		else:
			pwm_left_1.ChangeDutyCycle(duty_cycle)
			pwm_left_2.ChangeDutyCycle(0)
			pwm_right_1.ChangeDutyCycle(duty_cycle)
			pwm_right_2.ChangeDutyCycle(0)
		#print(f'control signal : {duty_cycle}')
		prev_time = current_time
		prev_error = error_reference
	new_rotation = error_reference + reference
	new_rotation = normalize_angle(new_rotation)
	print(f'success performing {new_rotation}°\n')
	pwm_left_1.stop()
	pwm_left_2.stop()
	pwm_right_1.stop()
	pwm_right_2.stop()
	time.sleep(0.25)
	turn_off_motors()
	history.append((*last_position[0:2], new_rotation))
	return history



