from controller import Robot, Motor, DistanceSensor
import math

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


def front_distance_1():
    return robot.getDevice('ps0').getValue()


def front_distance_2():
    return robot.getDevice('ps7').getValue()


def right_distance():
    return robot.getDevice('ps2').getValue()


def left_distance():
    return robot.getDevice('ps5').getValue()


while robot.step(TimeGap) != -1:

    right_collision = right_distance() > 200 or front_distance_1() > 80
    left_collision = left_distance() > 200 or front_distance_2() > 80

    vl = 6.28
    vr = 6.28

    if left_collision and right_collision and robot.getTime() > 180:

        while robot.step(TimeGap) != -1:
            RightMotor.setVelocity(0.0)
            LeftMotor.setVelocity(0.0)


    elif left_collision:

        current_time = (robot.getTime())
        endtime = current_time + 1.635
        while (robot.getTime() < endtime) and (robot.step(TimeGap) != 1):
            RightMotor.setVelocity(-6.28)
            LeftMotor.setVelocity(6.28)

    else:
        RightMotor.setVelocity(vr)
        LeftMotor.setVelocity(vl)
