from controller import Robot, Motor, DistanceSensor
import math
import cv2
import time
import numpy as np

robot = Robot()

RightMotor = robot.getDevice('right wheel motor')
LeftMotor = robot.getDevice('left wheel motor')

RightMotor.setPosition(float('inf'))
LeftMotor.setPosition(float('inf'))
RightMotor.setVelocity(0.0)
LeftMotor.setVelocity(0.0)

TimeGap = int(robot.getBasicTimeStep())

Distance_sensor = [robot.getDevice('ps0'), robot.getDevice('ps2'), robot.getDevice('ps5'), robot.getDevice('ps7')]

robot.getDevice('ps0').enable(TimeGap)
robot.getDevice('ps2').enable(TimeGap)
robot.getDevice('ps5').enable(TimeGap)
robot.getDevice('ps7').enable(TimeGap)

camera = robot.getDevice('camera')
camera.enable(TimeGap)


def front_distance_1():
    return robot.getDevice('ps0').getValue()


def front_distance_2():
    return robot.getDevice('ps7').getValue()


def right_distance():
    return robot.getDevice('ps2').getValue()


def left_distance():
    return robot.getDevice('ps5').getValue()


def green():
    lower = np.array([56, 175, 0])
    upper = np.array([70, 235, 255])
    thd = camera.getImageArray()
    thd = np.asarray(thd, dtype=np.uint8)
    thd = cv2.rotate(thd, cv2.ROTATE_90_CLOCKWISE)
    thd = cv2.flip(thd, 1)
    thd = cv2.cvtColor(thd, cv2.COLOR_RGB2HSV)
    thd = cv2.inRange(thd, lower, upper)
    return avenger(thd)


def blue():
    lower = np.array([110, 200, 95])
    upper = np.array([125, 230, 220])
    thd = camera.getImageArray()
    thd = np.asarray(thd, dtype=np.uint8)
    thd = cv2.rotate(thd, cv2.ROTATE_90_CLOCKWISE)
    thd = cv2.flip(thd, 1)
    thd = cv2.cvtColor(thd, cv2.COLOR_RGB2HSV)
    thd = cv2.inRange(thd, lower, upper)
    return avenger(thd)


def violet():
    lower = np.array([130, 115, 50])
    upper = np.array([150, 215, 155])
    thd = camera.getImageArray()
    thd = np.asarray(thd, dtype=np.uint8)
    thd = cv2.rotate(thd, cv2.ROTATE_90_CLOCKWISE)
    thd = cv2.flip(thd, 1)
    thd = cv2.cvtColor(thd, cv2.COLOR_RGB2HSV)
    thd = cv2.inRange(thd, lower, upper)
    return avenger(thd)


def red():
    lower = np.array([170, 150, 70])
    upper = np.array([185, 240, 215])
    thd = camera.getImageArray()
    thd = np.asarray(thd, dtype=np.uint8)
    thd = cv2.rotate(thd, cv2.ROTATE_90_CLOCKWISE)
    thd = cv2.flip(thd, 1)
    thd = cv2.cvtColor(thd, cv2.COLOR_RGB2HSV)
    thd = cv2.inRange(thd, lower, upper)
    return avenger(thd)


def avenger(thd):
    contours, _ = cv2.findContours(thd, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if len(contours) >= 1:
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if 500 < cv2.contourArea(contours[0]) < 230000:
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                if 220 < cx < 290 and 150 < cy < 350:
                    return ("enemies ahead")
                elif cx < 220 and 150 < cy < 350:
                    return ("enemies left")
                elif cx > 290 and 150 < cy < 350:
                    return ("enemies right")
        elif cv2.contourArea(contours[0]) > 230000:
            return ("stop")
        elif cv2.contourArea(contours[0]) < 20:
            return ("no enemies")
    else:
        return ("no enemies")


def turn_right():
    RightMotor.setVelocity(-4.71)
    LeftMotor.setVelocity(4.71)


def turn_left():
    RightMotor.setVelocity(4.71)
    LeftMotor.setVelocity(-4.71)


hit = 0
while hit == 0 and robot.step(TimeGap) != -1:
    if green() == ("enemies left"):
        turn_left()
    elif green() == ("enemies right") or green() == ("no enemies"):
        turn_right()
    elif green() == ("enemies ahead"):
        RightMotor.setVelocity(6.28)
        LeftMotor.setVelocity(6.28)
    elif green() == ("stop"):
        RightMotor.setVelocity(0.0)
        LeftMotor.setVelocity(0.0)
        hit = 1

x = robot.getTime()
while (x <= robot.getTime() <= (x + 0.5)) and robot.step(TimeGap) != -1:
    RightMotor.setVelocity(-6.28)
    LeftMotor.setVelocity(-6.28)

while hit == 1 and robot.step(TimeGap) != -1:
    if blue() == ("enemies left"):
        turn_left()
    elif blue() == ("enemies right") or blue() == ("no enemies"):
        turn_right()
    elif blue() == ("enemies ahead"):
        RightMotor.setVelocity(6.28)
        LeftMotor.setVelocity(6.28)
    elif blue() == ("stop"):
        RightMotor.setVelocity(0.0)
        LeftMotor.setVelocity(0.0)
        hit = 2

x = robot.getTime()
while (x <= robot.getTime() <= (x + 0.5)) and robot.step(TimeGap) != -1:
    RightMotor.setVelocity(-6.28)
    LeftMotor.setVelocity(-6.28)

while hit == 2 and robot.step(TimeGap) != -1:
    if violet() == ("enemies left"):
        turn_left()
    elif violet() == ("enemies right") or violet() == ("no enemies"):
        turn_right()
    elif violet() == ("enemies ahead"):
        RightMotor.setVelocity(6.28)
        LeftMotor.setVelocity(6.28)
    elif violet() == ("stop"):
        RightMotor.setVelocity(0.0)
        LeftMotor.setVelocity(0.0)
        hit = 3

x = robot.getTime()
while (x <= robot.getTime() <= (x + 0.5)) and robot.step(TimeGap) != -1:
    RightMotor.setVelocity(-6.28)
    LeftMotor.setVelocity(-6.28)

while hit == 3 and robot.step(TimeGap) != -1:
    if red() == ("enemies left"):
        turn_left()
    elif red() == ("enemies right") or red() == ("no enemies"):
        turn_right()
    elif red() == ("enemies ahead"):
        RightMotor.setVelocity(6.28)
        LeftMotor.setVelocity(6.28)
    elif red() == ("stop"):
        RightMotor.setVelocity(0.0)
        LeftMotor.setVelocity(0.0)
        hit = 4