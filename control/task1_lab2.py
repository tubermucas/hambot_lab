from control.MyRobot import MyRobot
import time


# ============================================================
# LAB 2 - PID CONTROL
# ============================================================

# The lab requires a fixed PID update period of 0.032 seconds.
DT = 0.032

# ------------------------------------------------------------
# STEP 1 CONSTANTS - LIDAR
# ------------------------------------------------------------

# 2 ft converted to millimeters.
# The LiDAR returns distances in millimeters.
LIDAR_TARGET_DISTANCE = 609.6

# ------------------------------------------------------------
# STEP 2 CONSTANTS - ENCODERS
# ------------------------------------------------------------

# 1 ft converted to meters.
ENCODER_TARGET_DISTANCE = 0.3048

# ------------------------------------------------------------
# GENERAL CONTROL LIMITS
# ------------------------------------------------------------

# Maximum motor command allowed during the PID controllers.
MAX_SPEED = 60

# LiDAR tolerance:
# 20 mm = 2 cm.
LIDAR_TOLERANCE = 20.0

# Encoder tolerance:
# 0.01 m = 1 cm.
ENCODER_TOLERANCE = 0.01


# ============================================================
# STEP 1 - LIDAR PID
# ============================================================

def get_front_distance(robot):
    """
    Return the distance directly in front of the robot.

    get_range_image() returns 360 LiDAR measurements in mm.
    Index 180 corresponds to the front of the HamBot.
    """

    lidar_data = robot.get_range_image()

    front_distance = lidar_data[180]

    return front_distance


def lidar_pid(robot, target_distance, kp, ki, kd):
    """
    Move the robot until it reaches the requested distance
    from the wall using PID control.

    target_distance:
        Desired distance from the wall in millimeters.

    kp:
        Proportional gain.

    ki:
        Integral gain.

    kd:
        Derivative gain.
    """

    # Sum of errors over time.
    # Used for the Integral part of the PID controller.
    integral = 0.0

    # Stores the previous error.
    # Used for the Derivative part of the PID controller.
    previous_error = None


    while True:

        # ----------------------------------------------------
        # 1. READ LIDAR
        # ----------------------------------------------------

        current_distance = get_front_distance(robot)


        # ----------------------------------------------------
        # 2. CALCULATE ERROR
        # ----------------------------------------------------
        #
        # Lab definition:
        #
        # error = desired value - measured value
        #
        # Example:
        #
        # target  = 609.6 mm
        # current = 1500 mm
        #
        # error = 609.6 - 1500
        #       = -890.4 mm
        #
        # Negative error -> robot is too far from the wall.
        # Positive error -> robot is too close to the wall.

        error = target_distance - current_distance


        # ----------------------------------------------------
        # 3. PROPORTIONAL TERM
        # ----------------------------------------------------

        proportional = kp * error


        # ----------------------------------------------------
        # 4. INTEGRAL TERM
        # ----------------------------------------------------

        integral += error * DT

        integral_term = ki * integral


        # ----------------------------------------------------
        # 5. DERIVATIVE TERM
        # ----------------------------------------------------

        if previous_error is None:

            # No previous error exists during the first loop.
            derivative = 0.0

        else:

            derivative = (error - previous_error) / DT


        derivative_term = kd * derivative


        # ----------------------------------------------------
        # 6. PID OUTPUT
        # ----------------------------------------------------

        pid_output = (
            proportional
            + integral_term
            + derivative_term
        )


        # ----------------------------------------------------
        # 7. CONVERT PID OUTPUT TO MOTOR SPEED
        # ----------------------------------------------------
        #
        # Our error is:
        #
        #     target - current
        #
        # If the robot is too far from the wall:
        #
        #     current > target
        #     error < 0
        #
        # But positive motor speed means forward.
        #
        # Therefore, we reverse the PID output sign.

        motor_speed = -pid_output


        # ----------------------------------------------------
        # 8. SATURATION
        # ----------------------------------------------------

        motor_speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, motor_speed)
        )


        # ----------------------------------------------------
        # 9. DISPLAY INFORMATION
        # ----------------------------------------------------

        print(
            f"LIDAR | "
            f"Distance: {current_distance:.1f} mm | "
            f"Target: {target_distance:.1f} mm | "
            f"Error: {error:.1f} mm | "
            f"Speed: {motor_speed:.2f} RPM"
        )


        # ----------------------------------------------------
        # 10. CHECK IF TARGET WAS REACHED
        # ----------------------------------------------------

        if abs(error) <= LIDAR_TOLERANCE:

            robot.stop_motors()

            print("Step 1 complete: target wall distance reached.")

            break


        # ----------------------------------------------------
        # 11. SEND COMMAND TO MOTORS
        # ----------------------------------------------------

        robot.run_motors(
            motor_speed,
            motor_speed
        )


        # ----------------------------------------------------
        # 12. SAVE ERROR FOR NEXT LOOP
        # ----------------------------------------------------

        previous_error = error


        # ----------------------------------------------------
        # 13. WAIT FOR NEXT PID UPDATE
        # ----------------------------------------------------

        time.sleep(DT)


# ============================================================
# STEP 2 - ENCODER PID
# Goal: drive forward 1 ft
# ============================================================

def get_traveled_distance(robot):
    """
    Calculate how far the robot has traveled using the wheel encoders.

    The HamBot encoders return wheel rotation in radians.

    Distance traveled by one wheel is:

        distance = wheel_angle * wheel_radius

    The left and right wheel distances are averaged to estimate
    the forward distance traveled by the robot.

    The returned distance is in meters.
    """

    # Read accumulated wheel rotations in radians.
    encoder_readings = robot.get_encoder_readings()

    left_radians = encoder_readings[0]
    right_radians = encoder_readings[1]

    # Convert wheel rotation into linear distance.
    left_distance = left_radians * robot.WHEEL_RADIUS
    right_distance = right_radians * robot.WHEEL_RADIUS

    # Average both wheels to estimate the robot's forward distance.
    traveled_distance = (
        left_distance + right_distance
    ) / 2.0

    return traveled_distance


def encoder_pid(robot, target_distance, kp, ki, kd):
    """
    Drive the robot forward a requested distance using
    wheel encoder feedback and PID control.

    target_distance:
        Desired distance traveled in meters.

    kp:
        Proportional gain.

    ki:
        Integral gain.

    kd:
        Derivative gain.
    """

    # --------------------------------------------------------
    # RESET ENCODERS
    # --------------------------------------------------------
    #
    # The encoder values are accumulated over time.
    # Resetting them lets this movement start from zero.

    robot.reset_encoders()


    # --------------------------------------------------------
    # PID VARIABLES
    # --------------------------------------------------------

    integral = 0.0

    previous_error = None


    while True:

        # ----------------------------------------------------
        # 1. READ ENCODERS
        # ----------------------------------------------------

        current_distance = get_traveled_distance(robot)


        # ----------------------------------------------------
        # 2. CALCULATE ERROR
        # ----------------------------------------------------
        #
        # Example:
        #
        # target distance  = 0.3048 m
        # traveled so far = 0.1000 m
        #
        # error = 0.3048 - 0.1000
        #       = 0.2048 m
        #
        # While the robot has not reached the target,
        # the error remains positive.

        error = target_distance - current_distance


        # ----------------------------------------------------
        # 3. PROPORTIONAL TERM
        # ----------------------------------------------------

        proportional = kp * error


        # ----------------------------------------------------
        # 4. INTEGRAL TERM
        # ----------------------------------------------------

        integral += error * DT

        integral_term = ki * integral


        # ----------------------------------------------------
        # 5. DERIVATIVE TERM
        # ----------------------------------------------------

        if previous_error is None:

            derivative = 0.0

        else:

            derivative = (error - previous_error) / DT


        derivative_term = kd * derivative


        # ----------------------------------------------------
        # 6. PID OUTPUT
        # ----------------------------------------------------

        pid_output = (
            proportional
            + integral_term
            + derivative_term
        )


        # ----------------------------------------------------
        # 7. MOTOR SPEED
        # ----------------------------------------------------
        #
        # Unlike the LiDAR controller, we DO NOT reverse the
        # sign here.
        #
        # At the beginning:
        #
        # target  = 0.3048 m
        # current = 0 m
        #
        # error = positive
        #
        # A positive motor speed moves the robot forward.
        #
        # Therefore:
        #
        # motor_speed = pid_output

        motor_speed = pid_output


        # ----------------------------------------------------
        # 8. SATURATION
        # ----------------------------------------------------

        motor_speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, motor_speed)
        )


        # ----------------------------------------------------
        # 9. DISPLAY INFORMATION
        # ----------------------------------------------------

        print(
            f"ENCODER | "
            f"Distance: {current_distance:.3f} m | "
            f"Target: {target_distance:.3f} m | "
            f"Error: {error:.3f} m | "
            f"Speed: {motor_speed:.2f} RPM"
        )


        # ----------------------------------------------------
        # 10. CHECK IF TARGET WAS REACHED
        # ----------------------------------------------------

        if abs(error) <= ENCODER_TOLERANCE:

            robot.stop_motors()

            print("Step 2 complete: encoder distance reached.")

            break


        # ----------------------------------------------------
        # 11. SEND COMMAND TO MOTORS
        # ----------------------------------------------------
        #
        # Same speed on both sides keeps the robot moving
        # approximately straight.

        robot.run_motors(
            motor_speed,
            motor_speed
        )


        # ----------------------------------------------------
        # 12. SAVE CURRENT ERROR
        # ----------------------------------------------------

        previous_error = error


        # ----------------------------------------------------
        # 13. WAIT FOR NEXT PID UPDATE
        # ----------------------------------------------------

        time.sleep(DT)


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    robot = MyRobot()


    # ========================================================
    # PID GAINS
    # ========================================================
    #
    # These are STARTING values.
    #
    # They must be tuned experimentally on the physical robot.
    # The LiDAR and encoder controllers use different units,
    # so they need different PID gains.


    # --------------------------------------------------------
    # STEP 1 PID GAINS - LIDAR
    # Error is measured in millimeters.
    # --------------------------------------------------------

    lidar_kp = 0.05
    lidar_ki = 0.0
    lidar_kd = 0.0


    # --------------------------------------------------------
    # STEP 2 PID GAINS - ENCODERS
    # Error is measured in meters.
    # --------------------------------------------------------

    encoder_kp = 150.0
    encoder_ki = 0.0
    encoder_kd = 0.0


    try:

        # ====================================================
        # STEP 1
        # LiDAR PID - Stop 2 ft from wall
        # ====================================================

        print("\nStarting Step 1 - LiDAR PID")

        lidar_pid(
            robot=robot,
            target_distance=LIDAR_TARGET_DISTANCE,
            kp=lidar_kp,
            ki=lidar_ki,
            kd=lidar_kd
        )


        # Pause after the robot has fully stopped.
        time.sleep(1)


        # ====================================================
        # STEP 2
        # Encoder PID - Drive forward 1 ft
        # ====================================================

        print("\nStarting Step 2 - Encoder PID")

        encoder_pid(
            robot=robot,
            target_distance=ENCODER_TARGET_DISTANCE,
            kp=encoder_kp,
            ki=encoder_ki,
            kd=encoder_kd
        )


    finally:

        # Safety:
        # If the program ends normally OR an error occurs,
        # make sure the motors are stopped.

        robot.stop_motors()


if __name__ == "__main__":
    main()
