
from constants_pi import *
import RPi.GPIO as gpio
import time
PWM_FREQ_GRIPPER = 50
ACTION_GRIPPER = {"CLOSE": 8, "OPEN": 13} #Values are the duty cycle
TIME_ACTION_GRIPPER = 0.45
def init_servo():
    pwm_pin = gpio.PWM(GRIPPER_PIN, PWM_FREQ_GRIPPER)
    return pwm_pin
def action_gripper(pwm_pin, action_type):
    pwm_pin.start(0)
    pwm_pin.ChangeDutyCycle(ACTION_GRIPPER[action_type])
    time.sleep(TIME_ACTION_GRIPPER)
    pwm_pin.stop()