import RPi.GPIO as gpio
import numpy as np
import time
from constants_pi import ENCODER_LEFT,ENCODER_RIGHT
from utilities_sensors import OFFSET_YAW, read_imu_yaw_angle
from utilities_motors import turn_off_motors
MIN_RESOLUTION_LIN = 1.021 #cm aprox watch means advance 1 step of encoder
MIN_RESOLUTION_ROT = 4.5 #degrees aprox watch means rotate 1 step of encoder

K_linear = 1.25
K_ROT_IMU = 1.1
K_D_ROT_IMU = 1.175#1.175
ERROR_STEPS = 1

STEPS = 20 #steps equivalent to one revolution of the wheel

PWM_LINEAR = 30 #75 turnning #30 advance
PWM_ROT = 75



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


def get_angle(vector_a, vector_b):
	dot_product = np.dot(vector_a, vector_b)
	angle =np.arccos( dot_product / (np.linalg.norm(vector_a)*np.linalg.norm(vector_b)))
	angle = np.degrees(angle)
	#NORMALIZED THE ANGLE
	if angle > 180 and angle < OFFSET_YAW:
		angle = angle - OFFSET_YAW
	return angle

def get_vector(node_a, node_b):
	"""
	This function returns the vector from node_a to node_b.

	Args:
		node_a (tuple): The first node.
	"""
	return (node_b[0] - node_a[0], node_b[1] - node_a[1])

def control_translation(action, reference, history, setup_motor):
	last_position = history[-1] if history else (0, 0, 0)
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
	pwm_left.start(PWM_LINEAR) # USE % OF duty cycle
	pwm_right.start(PWM_LINEAR) # USE % OF duty cycle
	steps_reference = round(reference/MIN_RESOLUTION_LIN)
	print(f'steps reference: {steps_reference}')
	while ticks <= steps_reference:
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
		duty_cycle =  PWM_LINEAR - K_linear* abs(diff_counter)
		#print(f'duty cycle is: {duty_cycle}')
		#left motor is going faster than right motor
		if diff_counter >= ERROR_STEPS:
			pwm_left.ChangeDutyCycle(duty_cycle)
		#right motor is going faster than left motor
		if diff_counter <= -ERROR_STEPS:
			pwm_right.ChangeDutyCycle(duty_cycle)
		# if counter_R >= steps_reference:
		# 	pwm_right.stop()
		# if counter_L >= steps_reference:
		# 	pwm_left.stop()
		ticks = round((counter_R + counter_L)/2)
		advancement = ticks*MIN_RESOLUTION_LIN
		#print(f'\ntraveled: {advancement}cm\n')
		course_advancement = flag_type * advancement
		#transform to the world system the coordinates of the robot
		advance_world = transf_matrix @ np.array([course_advancement, 0, 1])
		#print('advance world:', advance_world)
		#record position considered
		history.append((*advance_world[0:2],angle))
	pwm_left.stop() 
	pwm_right.stop() 
	print(f'success performing {reference} cm\n')
	turn_off_motors()
	time.sleep(1)
	return history

def keep_straight_pwm(action,reference):
	pwm_left, pwm_right = action()
	success = 0
	error_reference = 0
	current_time = None
	prev_time = None
	prev_error = 0
	duty_cycle = 0
	pwm_left.start(0)
	pwm_right.start(0)
	#achieve certain time maintaining straight
	while success < 5:
		yaw = read_imu_yaw_angle()
		error_reference = reference-yaw
		if abs(error_reference) <= 0.1:
			success += 1
			#print(f'error reference: {error_reference}')
		else:
			if current_time is None:
				current_time = 0.001
			else:
				current_time = time.time()
			delta_time = current_time - prev_time
			delta_error = error_reference - prev_error
			derivative = delta_error / delta_time
			duty_cycle = PWM_LINEAR + K_ROT_IMU*error_reference + K_D_ROT_IMU * derivative
			if error_reference > 0:
				pwm_left.ChangeDutyCycle(duty_cycle)
			if error_reference < 0:
				pwm_right.ChangeDutyCycle(duty_cycle)
			prev_time = current_time
			prev_error = error_reference
	turn_off_motors()
	print(f'success performing {reference}°\n')



def control_rotation_imu(action, reference,sensor_imu,history = []):
	last_position = history[-1] if history else (0, 0, 0)
	_, _, angle = last_position
	error_reference = np.inf
	prev_time = None
	derivative = 0
	current_time = None
	prev_time = None
	prev_error = 0
	duty_cycle = 0
	pwm_left_1, pwm_left_2, pwm_right_1, pwm_right_2 = action()
	pwm_left_1.start(0)
	pwm_left_2.start(0)
	pwm_right_1.start(0)
	pwm_right_2.start(0)
	while abs(error_reference) >= 0.45:
		yaw = read_imu_yaw_angle(sensor_imu)
		if yaw is None:
			continue
		#print('yaw sensor:', yaw)
		error_reference = reference - yaw if reference > 0 else yaw- reference
		#print('rot_error:', error_reference)
		if current_time is None:
			current_time = time.time()
		# Calculate derivative
		if prev_time is None:  # First iteration
			derivative = 0
			diff_time = 0.001  # Default small time step
		else:
			diff_time = current_time - prev_time
			if diff_time > 0:
				derivative = (error_reference - prev_error) / diff_time
			else:
				derivative = 0
		duty_cycle = abs(error_reference)*K_ROT_IMU + derivative*K_D_ROT_IMU
		#print(f'control signal: {duty_cycle}')
		#Apply saturation as duty cycle can't be negative or lower certain threshold to work
		duty_cycle = max(min(duty_cycle, 100),70)#35
		if reference > 0:
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
		else:
			#turn to right direction
			if error_reference > 0:
				pwm_left_1.ChangeDutyCycle(duty_cycle)
				pwm_left_2.ChangeDutyCycle(0)
				pwm_right_1.ChangeDutyCycle(duty_cycle)
				pwm_right_2.ChangeDutyCycle(0)
			# turn to left direction
			else:
				pwm_left_1.ChangeDutyCycle(0)
				pwm_left_2.ChangeDutyCycle(duty_cycle)
				pwm_right_1.ChangeDutyCycle(0)
				pwm_right_2.ChangeDutyCycle(duty_cycle)

		#print(f'control signal : {duty_cycle}')
		prev_time = current_time
		prev_error = error_reference
		# pwm_left.ChangeDutyCycle(duty_cycle)
		# pwm_right.ChangeDutyCycle(duty_cycle)
	new_rotation = error_reference + reference
	if new_rotation >= OFFSET_YAW:
		new_rotation = new_rotation % OFFSET_YAW
	print(f'success performing {reference}°\n')
	pwm_left_1.stop()
	pwm_left_2.stop()
	pwm_right_1.stop()
	pwm_right_2.stop()
	time.sleep(1)
	turn_off_motors()
	history.append((*last_position[0:2], new_rotation))
	return history

#TODO: Function that if no blocks are found turn every x degrees until blocks of the color assigned are found
#
def search_action():
    pass
