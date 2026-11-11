from adafruit_servokit import ServoKit

import time

currentTime = time.perf_counter()
DeadTime = .05

CurrentChannel = "Stop"
ForwardChannel = 1 #subject to change for motor connection
ReverseChannel = 2 #subject to change for motor connection
DirectionChannel = 3 #subject to change for Turn Servo connection
TransChannel = 15 #subject to change for transmission connection

#subject to change based off max angle of servo
FirstGearAngle = 0
SecondGearAngle = 150
ServoRadiusLimit = {
    "Left": 45,
    "Right": 135,
}
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
    else:
        currentTime = time.perf_counter()
        kit.servo[ReverseChannel].angle = 0

def move_forward(speed):
    if _deadTime(): return
    _adjustSpeed("forward", speed)

def move_backward(speed):
    if _deadTime(): return
    _adjustSpeed("reverse", speed)

def Stop_Motor():
    kit.servo[TransChannel].angle = 0.0
    if _deadTime(): return
    _adjustSpeed("Stop")
    
def _adjustTransmission(range: int[1-10]):
    kit.servo[TransChannel].angle = ((range-1)*SecondGearAngle)/9

def turn_left(turn_radius):

    if turn_radius < ServoRadiusLimit["Left"] or turn_radius > ServoRadiusLimit["Right"]:
        return
    kit.servo[DirectionChannel].angle = turn_radius
    
def turn_right(turn_radius):
    if turn_radius < ServoRadiusLimit["Left"] or turn_radius > ServoRadiusLimit["Right"]:
        return
    kit.servo[DirectionChannel].angle = turn_radius
   
