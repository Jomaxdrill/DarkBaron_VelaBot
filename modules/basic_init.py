from constants_pi import *
import RPi.GPIO as gpio
import serial
import time

def init_gpio():
	print('init gpio raspberry pi 4')
	gpio.setmode(gpio.BOARD)
	#*setup motors
	gpio.setup(MOTOR_L_1, gpio.OUT)
	gpio.setup(MOTOR_L_2, gpio.OUT)
	gpio.setup(MOTOR_R_1, gpio.OUT)
	gpio.setup(MOTOR_R_2, gpio.OUT)
	#*setup sonar sensor
	gpio.setup(TRIG, gpio.OUT)
	gpio.setup(ECHO, gpio.IN)
	#*setup encoders
	gpio.setup(ENCODER_LEFT, gpio.IN, pull_up_down=gpio.PUD_UP)
	gpio.setup(ENCODER_RIGHT, gpio.IN, pull_up_down=gpio.PUD_UP)
	#*setup gripper
	gpio.setup(GRIPPER_PIN, gpio.OUT)
	print('Setup complete')
def init_serial_read():
	print('init serial reading')
	imu_sensor = serial.Serial(IMU_SERIAL, 9600)  # Adjust the port and baud rate as needed
	while True:
		if imu_sensor.in_waiting > 0:
			time.sleep(1)
			# Avoid first n-lines of serial information
			readed_line = imu_sensor.readline()
			print(f'readed line {readed_line}.Ready to read yaw')
			return imu_sensor
def gameover():
	gpio.cleanup()
	print("Game Over")