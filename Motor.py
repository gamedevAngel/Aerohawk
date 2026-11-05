from adafruit_servokit import ServoKit

import time

currentTime = time.perf_counter()
DeadTime = .05
TransChannel = 15 #subject to change for transmission connection
ForwardChannel = 1 #subject to change for motor connection
ReverseChannel = 2 #subject to change for motor connection

#subject to change based off max angle of servo
FirstGearAngle = 0
SecondGearAngle = 150

#initialize 16-channels
kit = ServoKit(channels=16) #Transmistion


#inverse speed if necessary

def _deadTime():
    dead = time.perf_counter() - currentTime < DeadTime
    if dead:
        currentTime = time.perf_counter()
        return True
    return False

def _adjustSpeed(channel: str,speed):
    if channel == "forward":
        kit.servo[ReverseChannel].angle = 0
        currentTime = time.perf_counter()
        time.sleep(DeadTime - .005)
        kit.servo[ForwardChannel].angle = int(speed / 100 * 65535)
    elif channel == "reverse":
        currentTime = time.perf_counter()
        time.sleep(DeadTime - .005)
        kit.servo[ReverseChannel].angle = int(speed / 100 * 65535)

def move_forward(speed):
    if _deadTime(): return
    _adjustSpeed("forward", speed)

def move_backward(speed):
    if _deadTime(): return
    _adjustSpeed("reverse", speed)

def stop():
    kit.servo[TransChannel].angle = 0.0

def _adjustTransmission(range: int[1-10]):
    kit.servo[TransChannel].angle = ((range-1)*SecondGearAngle)/9

def turn_left(turn_radius):
    kit.servo[TransChannel].angle = speed
    
def turn_right(turn_radius):
    kit.servo[TransChannel].angle = speed
   
