#!/usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
import time

left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)


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
    wheel_distance = 9.1

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
    wheel_distance = 9.1

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
        SpeedPercent(-20),
        motor_degrees,
        block=False
    )

    right_motor.on_for_degrees(
        SpeedPercent(-20),
        motor_degrees,
        block=True
    )

    left_motor.off()
    right_motor.off()

    print("Left motor:", left_motor.position, "degrees")
    print("Right motor:", right_motor.position, "degrees")

def rotate(angle):
    wheel_diameter = 5.5
    wheel_distance = 9.1

    motor_degrees = angle * wheel_distance / wheel_diameter

    left_motor.on_for_degrees(
        SpeedPercent(-20),
        motor_degrees,
        block=False
    )

    right_motor.on_for_degrees(
        SpeedPercent(20),
        motor_degrees,
        block=True
    )

    left_motor.off()
    right_motor.off()


def rectangle(length_cm, width_cm):
    straight(length_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    straight(width_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    straight(length_cm)
    time.sleep(0.5)
    rotate(90)
    time.sleep(0.5)

    straight(width_cm)
    time.sleep(0.5)
    rotate(90)
#straight_line_error()
#rotational_error(90)

#straight(100)
#circle(50)
rectangle(100, 50)

def lemniscate(a_cm):
    import math
    import time

    wheel_diameter = 5.5
    wheel_distance = 9.1

    # --------------------------------------------------
    # Lemniscate parameters
    # x(t) = a sin(t)
    # y(t) = a sin(t) cos(t)
    # --------------------------------------------------

    # Maximum motor speed percentage
    max_motor_percent = 20

    # Number of small time steps
    steps = 200

    # Go through two arcs of the figure-eight
    t_start = 0
    t_end = 2 * math.pi

    dt = (t_end - t_start) / steps

    # --------------------------------------------------
    # Calculate the curve information
    # --------------------------------------------------

    for i in range(steps):

        t = t_start + i * dt

        # First derivatives
        dx = a_cm * math.cos(t)

        dy = a_cm * (math.cos(t) ** 2 - math.sin(t) ** 2)

        # Second derivatives
        ddx = -a_cm * math.sin(t)

        ddy = -4 * a_cm * math.sin(t) * math.cos(t)

        # --------------------------------------------------
        # Robot linear velocity
        # --------------------------------------------------

        speed = math.sqrt(dx ** 2 + dy ** 2)

        # Avoid division by zero
        if speed == 0:
            continue

        # --------------------------------------------------
        # Curvature
        #
        # k = (x'y'' - y'x'') /
        #     (x'^2 + y'^2)^(3/2)
        # --------------------------------------------------

        curvature = (
            dx * ddy - dy * ddx
        ) / (
            (dx ** 2 + dy ** 2) ** 1.5
        )

        # --------------------------------------------------
        # Angular velocity
        # omega = v * curvature
        # --------------------------------------------------

        omega = speed * curvature

        # --------------------------------------------------
        # Differential-drive wheel velocities
        # --------------------------------------------------

        left_velocity = speed - (wheel_distance / 2) * omega
        right_velocity = speed + (wheel_distance / 2) * omega

        # --------------------------------------------------
        # Scale wheel velocities to motor percentages
        # --------------------------------------------------

        max_velocity = max(
            abs(left_velocity),
            abs(right_velocity)
        )

        left_percent = (
            left_velocity / max_velocity
        ) * max_motor_percent

        right_percent = (
            right_velocity / max_velocity
        ) * max_motor_percent

        # Your robot currently moves forward with
        # negative motor percentages.
        left_percent = -left_percent
        right_percent = -right_percent

        # --------------------------------------------------
        # Run motors for this small time step
        # --------------------------------------------------

        left_motor.on(SpeedPercent(left_percent))
        right_motor.on(SpeedPercent(right_percent))

        time.sleep(0.05)

    # Stop both motors
    left_motor.off()
    right_motor.off()
