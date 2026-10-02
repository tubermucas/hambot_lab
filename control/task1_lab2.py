from control.MyRobot import MyRobot
import time


# ============================================================
# LAB 2 - PID CONTROL
# STEPS 1, 2, AND 3
# ============================================================

# The assignment requires the PID controller to update
# every 0.032 seconds.
DT = 0.032


# ============================================================
# TARGET VALUES
# ============================================================

# LiDAR measurements are returned in millimeters.
# 2 ft = 609.6 mm
LIDAR_TARGET_DISTANCE = 609.6

# Encoder distance is calculated in meters.
# 1 ft = 0.3048 m
ENCODER_TARGET_DISTANCE = 0.3048


# ============================================================
# CONTROL LIMITS
# ============================================================

# Maximum motor speed that we allow the PID to request.
MAX_SPEED = 60

# Acceptable error for the LiDAR controller.
# 20 mm = 2 cm.
LIDAR_TOLERANCE = 20.0

# Acceptable error for the encoder controller.
# 0.01 m = 1 cm.
ENCODER_TOLERANCE = 0.01


# ============================================================
# LIDAR HELPER FUNCTION
# ============================================================

def get_front_distance(robot):
    """
    Return the distance directly in front of the robot.

    get_range_image() returns 360 LiDAR measurements
    in millimeters.

    Index 180 corresponds to the front of the HamBot.
    """

    lidar_data = robot.get_range_image()

    front_distance = lidar_data[180]

    return front_distance


# ============================================================
# LIDAR PID CONTROLLER
# Used for STEP 1 and STEP 3
# ============================================================

def lidar_pid(robot, target_distance, kp, ki, kd):
    """
    Move the robot until it reaches the requested distance
    from the wall using LiDAR feedback and PID control.

    target_distance:
        Desired distance from the wall in millimeters.

    kp:
        Proportional gain.

    ki:
        Integral gain.

    kd:
        Derivative gain.
    """

    # Stores the accumulated error for the Integral term.
    integral = 0.0

    # Stores the previous error for the Derivative term.
    # None means that there is no previous error yet.
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
        # Negative error:
        # robot is too far from the wall.
        #
        # Positive error:
        # robot is too close to the wall.

        error = target_distance - current_distance


        # ----------------------------------------------------
        # 3. PROPORTIONAL TERM
        # ----------------------------------------------------
        #
        # Reacts to the current error.

        proportional = kp * error


        # ----------------------------------------------------
        # 4. INTEGRAL TERM
        # ----------------------------------------------------
        #
        # Accumulates the error over time.

        integral += error * DT

        integral_term = ki * integral


        # ----------------------------------------------------
        # 5. DERIVATIVE TERM
        # ----------------------------------------------------
        #
        # Measures how quickly the error changes.

        if previous_error is None:

            # There is no previous error during the first loop.
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
        # target - current
        #
        # If the robot is too far from the wall,
        # the error is negative.
        #
        # Positive motor speed means forward.
        #
        # Therefore, we reverse the sign of the PID output.

        motor_speed = -pid_output


        # ----------------------------------------------------
        # 8. SATURATION
        # ----------------------------------------------------
        #
        # Keep motor speed between:
        #
        # -MAX_SPEED and +MAX_SPEED

        motor_speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, motor_speed)
        )


        # ----------------------------------------------------
        # 9. DISPLAY VALUES
        # ----------------------------------------------------
        #
        # Useful during testing and PID tuning.

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

            print("LiDAR target distance reached.")

            break


        # ----------------------------------------------------
        # 11. MOVE ROBOT
        # ----------------------------------------------------
        #
        # Same speed on both wheels:
        #
        # positive -> forward
        # negative -> backward

        robot.run_motors(
            motor_speed,
            motor_speed
        )


        # ----------------------------------------------------
        # 12. SAVE ERROR
        # ----------------------------------------------------
        #
        # Current error becomes the previous error
        # during the next PID calculation.

        previous_error = error


        # ----------------------------------------------------
        # 13. WAIT FOR NEXT PID UPDATE
        # ----------------------------------------------------

        time.sleep(DT)


# ============================================================
# ENCODER HELPER FUNCTION
# ============================================================

def get_traveled_distance(robot):
    """
    Calculate the distance traveled using the wheel encoders.

    The encoders return wheel rotation in radians.

    Linear distance is calculated using:

        distance = angle_in_radians * wheel_radius

    The left and right wheel distances are averaged.

    The returned distance is in meters.
    """

    # Read accumulated encoder rotations.
    encoder_readings = robot.get_encoder_readings()

    left_radians = encoder_readings[0]
    right_radians = encoder_readings[1]


    # Convert wheel rotation into linear distance.
    left_distance = left_radians * robot.WHEEL_RADIUS
    right_distance = right_radians * robot.WHEEL_RADIUS


    # Average the distance traveled by both wheels.
    traveled_distance = (
        left_distance + right_distance
    ) / 2.0


    return traveled_distance


# ============================================================
# ENCODER PID CONTROLLER
# Used for STEP 2
# ============================================================

def encoder_pid(robot, target_distance, kp, ki, kd):
    """
    Drive the robot forward by a requested distance using
    encoder feedback and PID control.

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
    # Encoder values accumulate while the robot moves.
    # Reset them so this movement begins at zero.

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
        # error = desired distance - traveled distance
        #
        # Example:
        #
        # target  = 0.3048 m
        # current = 0.1000 m
        #
        # error = 0.3048 - 0.1000
        #       = 0.2048 m

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
        # At the beginning:
        #
        # target > current
        #
        # therefore:
        #
        # error > 0
        #
        # Positive motor speed moves the robot forward.

        motor_speed = pid_output


        # ----------------------------------------------------
        # 8. SATURATION
        # ----------------------------------------------------

        motor_speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, motor_speed)
        )


        # ----------------------------------------------------
        # 9. DISPLAY VALUES
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

            print("Encoder target distance reached.")

            break


        # ----------------------------------------------------
        # 11. MOVE ROBOT
        # ----------------------------------------------------

        robot.run_motors(
            motor_speed,
            motor_speed
        )


        # ----------------------------------------------------
        # 12. SAVE ERROR
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

    # Create the robot.
    robot = MyRobot()


    # ========================================================
    # LIDAR PID GAINS
    # ========================================================
    #
    # These are starting values.
    # They will need to be tuned experimentally.

    lidar_kp = 0.05
    lidar_ki = 0.0
    lidar_kd = 0.0


    # ========================================================
    # ENCODER PID GAINS
    # ========================================================
    #
    # These are also starting values.
    # They will need to be tuned experimentally.

    encoder_kp = 150.0
    encoder_ki = 0.0
    encoder_kd = 0.0


    try:

        # ====================================================
        # STEP 1
        # LiDAR PID - Stop 2 ft from the wall
        # ====================================================

        print("\nStarting Step 1 - LiDAR PID")

        lidar_pid(
            robot=robot,
            target_distance=LIDAR_TARGET_DISTANCE,
            kp=lidar_kp,
            ki=lidar_ki,
            kd=lidar_kd
        )

        # Pause after Step 1.
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

        # Pause after Step 2.
        time.sleep(1)


        # ====================================================
        # STEP 3
        # LiDAR PID - Back up until 2 ft from the wall
        # ====================================================

        print("\nStarting Step 3 - LiDAR PID Backup")

        lidar_pid(
            robot=robot,
            target_distance=LIDAR_TARGET_DISTANCE,
            kp=lidar_kp,
            ki=lidar_ki,
            kd=lidar_kd
        )

        # Pause after Step 3.
        time.sleep(1)


    finally:

        # ====================================================
        # SAFETY
        # ====================================================
        #
        # Whether the program finishes normally or an error
        # occurs, make sure the motors are stopped.

        robot.stop_motors()


if __name__ == "__main__":
    main()
