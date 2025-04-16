from constants_pi import *
import RPi.GPIO as gpio
import time

OFFSET_YAW = 360

def record_count(encoder,counter,button):
    if int(gpio.input(encoder)) != int(button):
        button = int(gpio.input(encoder))
        counter += 1
        print(counter)

#sonar sensor
def distance():
    #Ensure output has no false value
    gpio.output(TRIG, False)
    time.sleep(0.1)

    #generate the trigger pulse
    gpio.output(TRIG, True)
    time.sleep(0.00001)
    gpio.output(TRIG, False)

    #wait for ECHO to go high and low
    while gpio.input(ECHO) == 0:
        pulse_start = time.time()
    while gpio.input(ECHO) == 1:
        pulse_end = time.time()

    #calculate the time difference
    pulse_duration = pulse_end - pulse_start

    #calculate the distance in cm
    distance = (pulse_duration * 34300) / 2
    distance = round(distance, 2)

    return distance


#imu sensor
def read_imu_yaw_angle(imu_sensor):
	count = 0
	if imu_sensor.in_waiting > 0:
		while count < 10:
			count += 1
		line = imu_sensor.readline().strip()
		yaw_raw = str(line)
		yaw = float(yaw_raw.strip("'").strip("'b"))
		if yaw == 360:
			return 0
		yaw_normalized = OFFSET_YAW - yaw
		if yaw_normalized > 180 and yaw_normalized < 360:
			yaw_normalized = yaw_normalized - OFFSET_YAW
		return yaw_normalized
