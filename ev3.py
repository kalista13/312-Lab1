#!/usr/bin/env python3

from ev3dev2.motor import (LargeMotor,OUTPUT_B,OUTPUT_C,SpeedPercent,SpeedDPS)
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4

import time
import math

left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)

right_sensor = ColorSensor(INPUT_1)
left_sensor = ColorSensor(INPUT_4)

gyro = GyroSensor(INPUT_2)
gyro.mode = 'GYRO-ANG'

gyro.reset()
time.sleep(0.1)


# --------------------------------------------------
# 2. ERROR DATA COLLECTION & ANALYSIS
#
# Measure straight-line and rotational error
# using motor encoders and an independent method.
#
# Test multiple motor speeds and compare how
# motion error changes with speed.
# --------------------------------------------------
# --------------------------------------------------
# 2.1 STRAIGHT-LINE ERROR
#
# Measure straight-line motion error.
#
# Encoder method:
# Compare left and right motor rotations.
# --------------------------------------------------
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
#straight_line_error()

# --------------------------------------------------
# 2.2 ROTATION ERROR
#
# Measure rotational motion error.
#
# Encoder method:
# Compare wheel rotation during a commanded turn.
# --------------------------------------------------
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
#rotational_error()

# ==================================================
# 3. SHAPE MOVEMENTS
#
# Draw the required trajectories using the pen.
# Repeat each shape from the same starting position.
# ==================================================
# --------------------------------------------------
# 3.1 STRAIGHT LINE
#
# Move the robot along a straight-line path.
# Target distance: 1 m.
# --------------------------------------------------
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
#straight(100)

# --------------------------------------------------
# 3.2 CIRCLE
#
# Move the robot in a circular path.
# Target radius: 0.5 m.
# --------------------------------------------------
def circle(radius):
    left_motor.position = 0
    right_motor.position = 0

    wheel_diameter = 5.5
    wheel_distance = 9.3

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
#circle(50)

# --------------------------------------------------
# 3.3 RECTANGLE
#
# Move the robot around a rectangular path.
# Target size: 0.5 m x 1 m.
# --------------------------------------------------
def rotate(angle):
    wheel_diameter = 5.5
    wheel_distance = 9
    gyro_offset = 0

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
#rotate(90)

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
#ectangle(100,50)

# --------------------------------------------------
# 3.4 LEMNISCATE
#
# Move the robot in a figure-eight path.
# Wheel speeds vary with the trajectory curvature.
# --------------------------------------------------
def lemniscate(a=50, total_time=35, dt=0.1, max_dps_per_step=150):
    wheel_diameter = 5.5
    wheel_circumference = math.pi * wheel_diameter
    L = 9.1

    N = int(total_time / dt)

    xs, ys = [], []
    for i in range(N + 1):
        t = 2 * math.pi * i / N
        denom = 1 + math.sin(t) ** 2
        xs.append(a * math.cos(t) / denom)
        ys.append(a * math.sin(t) * math.cos(t) / denom)

    headings = []
    for i in range(N):
        dx = xs[i + 1] - xs[i]
        dy = ys[i + 1] - ys[i]
        headings.append(math.atan2(dy, dx))

    dps_list = []
    for i in range(N):
        dx = xs[i + 1] - xs[i]
        dy = ys[i + 1] - ys[i]
        v = math.sqrt(dx ** 2 + dy ** 2) / dt

        if i == 0:
            dtheta = 0.0
        else:
            dtheta = headings[i] - headings[i - 1]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
        omega = dtheta / dt

        v_right = v + omega * (L / 2.0)
        v_left = v - omega * (L / 2.0)
        dps_list.append((
            -(v_left / wheel_circumference) * 360.0,
            -(v_right / wheel_circumference) * 360.0
        ))

    # Slew-rate limit: cap how much the commanded speed can change per step
    limited = [dps_list[0]]
    for i in range(1, N):
        prev_l, prev_r = limited[-1]
        tgt_l, tgt_r = dps_list[i]
        new_l = prev_l + max(-max_dps_per_step, min(max_dps_per_step, tgt_l - prev_l))
        new_r = prev_r + max(-max_dps_per_step, min(max_dps_per_step, tgt_r - prev_r))
        limited.append((new_l, new_r))

    print("step, t, dps_left, dps_right")
    for i in range(0, N, 10):
        print(i, "%.2f" % (2 * math.pi * i / N), "%.1f" % limited[i][0], "%.1f" % limited[i][1])

    left_motor.position = 0
    right_motor.position = 0

    for dps_l, dps_r in limited:
        left_motor.on(SpeedDPS(dps_l))
        right_motor.on(SpeedDPS(dps_r))
        time.sleep(dt)

    left_motor.off()
    right_motor.off()
    print("Left motor total degrees:", left_motor.position)
    print("Right motor total degrees:", right_motor.position)
lemniscate()

# --------------------------------------------------
# 4.1 & 4.2 COMMAND ARRAY / EXECUTION
#
# Read motor commands from a 3x3 array.
#
# Each row contains:
# [left power, right power, duration]
#
# Execute each row sequentially.
# --------------------------------------------------
def command_array(commands):
    for row in commands:
        left_power = row[0]
        right_power = row[1]
        duration = row[2]

        left_motor.on(SpeedPercent(left_power))
        right_motor.on(SpeedPercent(right_power))

        time.sleep(duration)

    left_motor.off()
    right_motor.off()
commands = [
    [80, 60, 2],
    [60, 60, 1],
    [-50, 80, 2]
]
#command_array(commands)

# --------------------------------------------------
# 4.3 DEAD RECKONING
#
# Estimate the robot pose (x, y, theta)
# using measured motor encoder velocities.
#
# Uses differential-drive kinematics and
# numerical integration over time.
# --------------------------------------------------
def dead_reckoning(commands):
    wheel_diameter = 5.5
    wheel_radius = wheel_diameter / 2

    # In the notes this distance is 2d
    wheel_distance = 9

    # Starting pose
    x = 0.0
    y = 0.0
    theta = 0.0

    # Integration timestep
    dt = 0.05

    # Reset encoders
    left_motor.position = 0
    right_motor.position = 0

    previous_left = left_motor.position
    previous_right = right_motor.position
    previous_time = time.monotonic()

    for row in commands:

        left_power = row[0]
        right_power = row[1]
        duration = row[2]

        # Execute command
        left_motor.on(SpeedPercent(left_power))
        right_motor.on(SpeedPercent(right_power))

        end_time = time.monotonic() + duration

        while time.monotonic() < end_time:

            time.sleep(dt)

            current_time = time.monotonic()

            current_left = left_motor.position
            current_right = right_motor.position

            actual_dt = current_time - previous_time

            # Encoder change
            left_change = current_left - previous_left
            right_change = current_right - previous_right

            # Measured motor velocity in degrees/second
            left_dps = left_change / actual_dt
            right_dps = right_change / actual_dt

            # Convert motor velocity to wheel velocity in cm/s
            left_velocity = -math.radians(left_dps) * wheel_radius
            right_velocity = -math.radians(right_dps) * wheel_radius

            # Lecture equations
            V = (right_velocity + left_velocity) / 2

            omega = (
                right_velocity - left_velocity
            ) / wheel_distance

            # Integrate position using lecture equations
            x = x + V * math.cos(theta) * actual_dt
            y = y + V * math.sin(theta) * actual_dt
            theta = theta + omega * actual_dt

            previous_left = current_left
            previous_right = current_right
            previous_time = current_time

    left_motor.off()
    right_motor.off()

    # Keep theta between -180 and 180 degrees
    theta = (theta + math.pi) % (2 * math.pi) - math.pi

    print("Estimated x:", x, "cm")
    print("Estimated y:", y, "cm")
    print("Estimated theta:", math.degrees(theta), "degrees")
da_commands = [
    [-30, -15, 2]
]
#dead_reckoning(commands)

# --------------------------------------------------
# Light sensors
#
# Right sensor -> Port 1
# Left sensor  -> Port 4
# --------------------------------------------------
# --------------------------------------------------
# 5.1 COWARDICE
#
# Each sensor controls the motor on the same side.
#
# Left sensor  -> Left motor
# Right sensor -> Right motor
#
# Brighter light = faster motor
# --------------------------------------------------

def cowardice():

    while True:

        left_light = left_sensor.ambient_light_intensity
        right_light = right_sensor.ambient_light_intensity

        # Crossed connections
        left_speed = right_light
        right_speed = left_light

        # Limit motor speed
        left_speed = min(left_speed, 50)
        right_speed = min(right_speed, 50)

        # Your robot moves forward with negative speed
        left_motor.on(
            SpeedPercent(-left_speed)
        )

        right_motor.on(
            SpeedPercent(-right_speed)
        )

        print(
            "Left light:", left_light,
            "Right light:", right_light,
            "Left motor:", left_speed,
            "Right motor:", right_speed
        )

        time.sleep(0.05)
#cowardice()

# --------------------------------------------------
# 5.2 AGGRESSION
#
# Each sensor controls the motor on the opposite side.
#
# Left sensor  -> Right motor
# Right sensor -> Left motor
#
# Brighter light = faster motor
# --------------------------------------------------

def aggression():

    while True:

        left_light = left_sensor.ambient_light_intensity
        right_light = right_sensor.ambient_light_intensity

        # Same-side connections
        left_speed = left_light
        right_speed = right_light

        # Limit motor speed
        left_speed = min(left_speed, 50)
        right_speed = min(right_speed, 50)

        # Your robot moves forward with negative speed
        left_motor.on(
            SpeedPercent(-left_speed)
        )

        right_motor.on(
            SpeedPercent(-right_speed)
        )

        print(
            "Left light:", left_light,
            "Right light:", right_light,
            "Left motor:", left_speed,
            "Right motor:", right_speed
        )

        time.sleep(0.05)
#aggression()
