from adafruit_servokit import ServoKit
from gpiozero import LED
import time

#subject to change for motor connection
motorChannel = 0
ServoChannel = 2

#kit.continuous_servo[1].throttle = 0.5 || -0.5

kit = ServoKit(channels=16)
kit.continuous_servo[motorChannel].throttle = 0.2  

#inverse speed if necessary
def move_forward(speed):
    kit.continuous_servo[channel].throttle = speed

def move_backward(speed):
    kit.continuous_servo[channel].throttle = speed

def stop():
    kit.continuous_servo[channel].throttle = 0.0

def turn_left(turn_radius):
    kit.continuous_servo[channel].throttle = speed
    
def turn_right(turn_radius):
    kit.continuous_servo[channel].throttle = speed
   
