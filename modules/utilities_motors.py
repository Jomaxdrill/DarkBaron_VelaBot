from constants_pi import *
import RPi.GPIO as gpio
import time

#*WHEELS
TIME_MOTOR = 2 #seconds
PWM_FREQ_MOTOR = 50 #Hz
PWM_CYCLE_MOTOR = 75 #%

#*SERVO
PWM_FREQ_GRIPPER = 50
ACTION_GRIPPER = {"CLOSE": 8, "OPEN": 13} #Values are the duty cycle
TIME_ACTION_GRIPPER = 0.45

def turn_off_motors():
	gpio.output(MOTOR_L_1, False)
	gpio.output(MOTOR_L_2, False)
	gpio.output(MOTOR_R_1, False)
	gpio.output(MOTOR_R_2, False)


def forward():
	#*Left Wheels
	pwm_left = gpio.PWM(MOTOR_L_1, PWM_FREQ_MOTOR)
	gpio.output(MOTOR_L_2, False)
	#*Right Wheels
	gpio.output(MOTOR_R_1, False)
	pwm_right = gpio.PWM(MOTOR_R_2, PWM_FREQ_MOTOR)
	return pwm_left, pwm_right,1

def reverse():
	#*Left Wheels
	gpio.output(MOTOR_L_1, False)
	pwm_left = gpio.PWM(MOTOR_L_2, PWM_FREQ_MOTOR)
	#*Right Wheels
	pwm_right = gpio.PWM(MOTOR_R_1, PWM_FREQ_MOTOR)
	gpio.output(MOTOR_R_2, False)
	return pwm_left, pwm_right,-1

def turnleft():
	#*Right Wheels will go constant 100% speed
	gpio.output(MOTOR_R_1, False)
	gpio.output(MOTOR_R_2, True)
	#*Left use pwm to rotate
	gpio.output(MOTOR_L_2, False)
	pwm_left = gpio.PWM(MOTOR_L_1, PWM_FREQ_MOTOR)
	pwm_left.start(PWM_CYCLE_MOTOR)
	#wait
	time.sleep(TIME_MOTOR)
	pwm_left.stop()
	#Send all pins low and cleanup
	turn_off_motors()

def turnright():
	#*Left Wheels will go constant 100% speed
	gpio.output(MOTOR_L_1, True)
	gpio.output(MOTOR_L_2, False)
	#*Right use pwm to rotate
	gpio.output(MOTOR_R_1, False)
	pwm_right = gpio.PWM(MOTOR_R_2, PWM_FREQ_MOTOR)
	pwm_right.start(PWM_CYCLE_MOTOR) # USE % OF duty cycle
	#Wait
	time.sleep(TIME_MOTOR)
	pwm_right.stop()
	#Send all pins low and cleanup
	turn_off_motors()


def pivotleft():
	#*Left wheels will go reverse
	gpio.output(MOTOR_L_1, False)
	pwm_left = gpio.PWM(MOTOR_L_2, PWM_FREQ_MOTOR)
	#*Right Wheels will go forward
	gpio.output(MOTOR_R_1, False)
	pwm_right = gpio.PWM(MOTOR_R_2, PWM_FREQ_MOTOR)
	return pwm_left, pwm_right

def pivotright():
	#*Left Wheels will forward
	pwm_left = gpio.PWM(MOTOR_L_1, PWM_FREQ_MOTOR)
	gpio.output(MOTOR_L_2, False)
	#*Right Wheels will go reverse
	pwm_right = gpio.PWM(MOTOR_R_1, PWM_FREQ_MOTOR)
	gpio.output(MOTOR_R_2, False)
	return pwm_left, pwm_right

def motor_pwm_setup():
	#*Left Wheels will forward
	pwm_left_1 = gpio.PWM(MOTOR_L_1, PWM_FREQ_MOTOR)
	pwm_left_2 = gpio.PWM(MOTOR_L_2, PWM_FREQ_MOTOR)
	pwm_right_1 = gpio.PWM(MOTOR_R_1, PWM_FREQ_MOTOR)
	pwm_right_2 = gpio.PWM(MOTOR_R_2, PWM_FREQ_MOTOR)
	return pwm_left_1, pwm_left_2, pwm_right_1, pwm_right_2

def init_servo():
	pwm_pin = gpio.PWM(GRIPPER_PIN, PWM_FREQ_GRIPPER)
	return pwm_pin
def action_gripper(pwm_pin, action_type):
	pwm_pin.start(0)
	pwm_pin.ChangeDutyCycle(ACTION_GRIPPER[action_type])
	time.sleep(TIME_ACTION_GRIPPER)

def turn_off_servo(pwm_pin):
	pwm_pin.stop()
	gpio.cleanup(GRIPPER_PIN)


