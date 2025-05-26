from .constants_pi import *
import RPi.GPIO as gpio
import time
import numpy as np

#*------------------------
###* PARAMS FOR SENSORS
#*------------------------
OFFSET_YAW = 360 #crucial to use the proper angle references
SAMPLES = 30 #samples to collect for averaging sonar distance
SPEED_SOUND = 34300
TIMEOUT_HOLD= 0.04

#*------------------------
###* ENCODER-counts steps of wheels
#*------------------------
def record_count(encoder, counter, button):
	if int(gpio.input(encoder)) != int(button):
		button = int(gpio.input(encoder))
		counter += 1
		print(counter)

#*------------------------
###* SONAR
#*------------------------
def init_sonar():
	#*setup sonar sensor
	gpio.setup(TRIG, gpio.OUT)
	gpio.setup(ECHO, gpio.IN)

def distance_sonar_average():
	dist_measurements = []
	counter_measure = 0
	while counter_measure < 30:
		dist = distance_sonar()
		dist_measurements.append(dist)
		counter_measure += 1
	#apply moving average filter
	dist_measurements_filtered = moving_average_filter(dist_measurements)
	average_distance = sum(dist_measurements_filtered) / len(dist_measurements)
	return average_distance

def distance_sonar():
	#Ensure output has no false value
	gpio.output(TRIG, False)
	time.sleep(0.1)

	#generate the trigger pulse
	gpio.output(TRIG, True)
	time.sleep(0.00001)
	gpio.output(TRIG, False)
	start_time = time.time()
	#*this timeout is needed because for certain surfaces it doesn't return the ultra sonar wave
	#*by doing this the measurement is not stuck although the exactitude is compromised a lot
	timeout = 0
	#wait for ECHO to go high and low
	pulse_start = time.time()
	while gpio.input(ECHO) == 0 and timeout <=TIMEOUT_HOLD:
		current_time = time.time()
		timeout = current_time - start_time
		pulse_start = time.time()
	timeout = 0
	pulse_end = time.time()
	while gpio.input(ECHO) == 1 and timeout <=TIMEOUT_HOLD:
		current_time = time.time()
		timeout = current_time - start_time
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	#*calculate the distance in cm
	distance = (pulse_duration * SPEED_SOUND) / 2
	distance = round(distance, 2) or 0
	return distance

def moving_average_filter(data_info, window_size =4):
	data_moving_average = np.zeros(len(data_info))
	for idx in range(len(data_info)):
		#for windows that will be out of the range of the data
		if idx + window_size > len(data_info):
			data_average = np.mean(data_info[idx:])
		else:
			data_average = np.mean(data_info[idx:idx + window_size-1])
		data_moving_average[idx] = data_average
	return data_moving_average

#*------------------------
###* IMU BNO055
#*------------------------
def read_imu_yaw_angle(imu_sensor):
	if imu_sensor.in_waiting > 0:
		line = imu_sensor.readline()
		line = line.rstrip().lstrip()
		yaw_raw = str(line)
		yaw = float(yaw_raw.strip("'").strip("'b"))
		yaw = OFFSET_YAW - yaw
		return yaw

#**Normalize angle between [0,360)
def normalize_angle(angle):
	if angle < 0:
		angle = angle + OFFSET_YAW
	if angle >= OFFSET_YAW:
		angle = angle % OFFSET_YAW
	return angle

