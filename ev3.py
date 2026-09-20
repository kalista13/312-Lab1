#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent, SpeedDPS
import time
import math

left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)

from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_2

gyro = GyroSensor(INPUT_2)
gyro.mode = 'GYRO-ANG'

gyro.reset()
time.sleep(0.1)


def straight_line_error():
    left_motor.position = 0
    right_motor.position = 0

    # Change these values to change speed
    left_motor.on(SpeedPercent(-10))
    right_motor.on(SpeedPercent(-10))

    time.sleep(2)

    left_motor.off()
    right_motor.off()

    print("Left motor:", left_motor.position, "degrees")
    print("Right motor:", right_motor.position, "degrees")
    print("Difference:", left_motor.position - right_motor.position, "degrees")


def rotational_error(robot_angle):
    left_motor.position = 0
    right_motor.position = 0

    wheel_diameter = 5.5
    wheel_distance = 9

    motor_degrees = robot_angle * wheel_distance / wheel_diameter

    left_motor.on_for_degrees(SpeedPercent(-5), motor_degrees, block=False)
    right_motor.on_for_degrees(SpeedPercent(5), motor_degrees, block=True)

    left_motor.off()
    right_motor.off()

    print("Commanded robot angle:", robot_angle, "degrees")
    print("Left motor:", left_motor.position, "degrees")
    print("Right motor:", right_motor.position, "degrees")
    print("speed is 250")


def circle(radius):
    left_motor.position = 0
    right_motor.position = 0

    wheel_diameter = 5.5
    wheel_distance = 9

    d = wheel_distance / 2

    # Choose robot angular speed
    omega = 0.35

    # Wheel linear speeds from lecture equations
    left_speed_cm = omega * (radius + d)
    right_speed_cm = omega * (radius - d)

    # Scale to motor percentages
    max_speed_cm = max(abs(left_speed_cm), abs(right_speed_cm))

    left_percent = 20 * left_speed_cm / max_speed_cm
    right_percent = 20 * right_speed_cm / max_speed_cm

    # Distance each wheel travels for one full circle
    left_distance = 2 * 3.14159 * (radius + d)
    right_distance = 2 * 3.14159 * (radius - d)

    wheel_circumference = 3.14159 * wheel_diameter

    left_degrees = left_distance / wheel_circumference * 360
    right_degrees = right_distance / wheel_circumference * 360

    left_motor.on_for_degrees(
        SpeedPercent(-left_percent),
        left_degrees,
        block=False
    )

    right_motor.on_for_degrees(
        SpeedPercent(-right_percent),
        right_degrees,
        block=True
    )

    left_motor.off()
    right_motor.off()

def straight(distance_cm):
    left_motor.position = 0
    right_motor.position = 0

    wheel_diameter = 5.5
    wheel_circumference = 3.14159 * wheel_diameter

    motor_degrees = (distance_cm / wheel_circumference) * 360

    left_motor.on_for_degrees(
        SpeedPercent(-30),
        motor_degrees,
        block=False
    )

    right_motor.on_for_degrees(
        SpeedPercent(-30),
        motor_degrees,
        block=True
    )

    left_motor.off()
    right_motor.off()

    print("Left motor:", left_motor.position, "degrees")
    print("Right motor:", right_motor.position, "degrees")

def rotate(angle):
    wheel_diameter = 5.5
    wheel_distance = 9
    gyro_offset = 13

    # Reset gyro for every individual turn
    gyro.reset()

    # After reset, each turn starts from 0 degrees
    current_angle = gyro.angle - gyro_offset

    # Amount needed to complete this turn
    corrected_angle = angle - current_angle

    motor_degrees = abs(corrected_angle) * wheel_distance / wheel_diameter

    if corrected_angle > 0:
        left_speed = 20
        right_speed = -20
    else:
        left_speed = -20
        right_speed = 20

    # Main turn
    left_motor.on_for_degrees(
        SpeedPercent(left_speed),
        motor_degrees,
        block=False
    )

    right_motor.on_for_degrees(
        SpeedPercent(right_speed),
        motor_degrees,
        block=True
    )

    left_motor.off()
    right_motor.off()

    time.sleep(0.2)

    # Measure error after the turn
    current_angle = gyro.angle - gyro_offset
    error = angle - current_angle

    # Correct remaining error
    if abs(error) > 1:

        correction_degrees = abs(error) * wheel_distance / wheel_diameter

        if error > 0:
            left_speed = 5
            right_speed = -5
        else:
            left_speed = -5
            right_speed = 5

        left_motor.on_for_degrees(
            SpeedPercent(left_speed),
            correction_degrees,
            block=False
        )

        right_motor.on_for_degrees(
            SpeedPercent(right_speed),
            correction_degrees,
            block=True
        )

        left_motor.off()
        right_motor.off()

    # Final result for THIS turn only
    current_angle = gyro.angle - gyro_offset
    error = angle - current_angle

    print("Target angle:", angle)
    print("Raw gyro angle:", gyro.angle)
    print("Corrected gyro angle:", current_angle)
    print("Final error:", error)

def rectangle(length_cm, width_cm):
    gyro.reset()
    time.sleep(1)
    straight(length_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    gyro.reset()
    time.sleep(1)
    straight(width_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    gyro.reset()
    time.sleep(1)
    straight(length_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    gyro.reset()
    time.sleep(1)
    straight(width_cm)
    time.sleep(0.5)
    rotate(90)

def lemniscate(a):
    wheel_diameter = 5.5
    wheel_distance = 9
    wheel_radius = wheel_diameter / 2

    t = 0
    dt = 0.05
    t_speed = 0.35

    while t < 2 * math.pi:

        dx = a * math.cos(t)
        dy = a * math.cos(2 * t)

        ddx = -a * math.sin(t)
        ddy = -2 * a * math.sin(2 * t)

        speed = math.sqrt(dx**2 + dy**2)

        curvature = (
            dx * ddy - dy * ddx
        ) / (speed**3)

        v = t_speed * speed
        omega = v * curvature

        # Make final quarter straighten earlier
        if t > 3 * math.pi / 2:
            omega *= 0.6

        left_speed = v - (wheel_distance / 2) * omega
        right_speed = v + (wheel_distance / 2) * omega

        left_dps = left_speed / wheel_radius * 180 / math.pi
        right_dps = right_speed / wheel_radius * 180 / math.pi

        left_motor.on(SpeedDPS(-left_dps))
        right_motor.on(SpeedDPS(-right_dps))

        time.sleep(dt)

        # Shorten the middle straight-ish section
        if math.pi / 2 < t < math.pi:
            t += 1.5 * t_speed * dt
        else:
            t += t_speed * dt

    left_motor.off()
    right_motor.off()

#straight_line_error()
#rotational_error(90)

#straight(100)
#circle(50)
#rectangle(15,15)
#lemniscate(50)
#print("testing quarter")

rotate(180)
