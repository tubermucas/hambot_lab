from control.MyRobot import MyRobot
import time


# ============================================================
# LAB 2 - PID CONTROL
# STEPS 1 TO 4
# ============================================================

# The assignment requires a PID update time of 0.032 seconds.
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

# Maximum motor speed allowed by our controllers.
MAX_SPEED = 60

# LiDAR tolerance:
# 20 mm = 2 cm
LIDAR_TOLERANCE = 20.0

# Encoder tolerance:
# 0.01 m = 1 cm
ENCODER_TOLERANCE = 0.01

# Rotation tolerance:
# Robot can stop when it is within 2 degrees of the target.
ROTATION_TOLERANCE = 2.0


# ============================================================
# LIDAR HELPER FUNCTION
# ============================================================

def get_front_distance(robot):
    """
    Return the LiDAR distance directly in front of the robot.

    get_range_image() returns 360 measurements in millimeters.
    Index 180 corresponds to the front of the robot.
    """

    lidar_data = robot.get_range_image()

    if lidar_data == -1:
        raise RuntimeError("LiDAR is not enabled.")

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

    # Accumulated error used by the Integral term.
    integral = 0.0

    # Previous error used by the Derivative term.
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
        # Negative error:
        # robot is too far from the wall.
        #
        # Positive error:
        # robot is too close to the wall.

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
        # 7. CONVERT PID OUTPUT TO MOTOR SPEED
        # ----------------------------------------------------
        #
        # Example:
        #
        # target  = 609.6 mm
        # current = 1500 mm
        #
        # error = negative
        #
        # But positive motor speed means forward.
        #
        # Therefore the PID output sign is reversed.

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

            print("LiDAR target reached.")

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

    Encoder values are wheel rotations in radians.

    Linear distance:

        distance = angle_in_radians * wheel_radius

    The distances from the left and right wheels are averaged.

    Returns:
        Distance traveled in meters.
    """

    encoder_readings = robot.get_encoder_readings()

    left_radians = encoder_readings[0]
    right_radians = encoder_readings[1]


    # Convert wheel rotation into linear distance.
    left_distance = left_radians * robot.WHEEL_RADIUS
    right_distance = right_radians * robot.WHEEL_RADIUS


    # Average both wheels.
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

    # Reset the encoders so this movement starts from zero.
    robot.reset_encoders()


    # PID variables.
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
        # error = 0.2048 m

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
        # Here the error starts positive.
        #
        # Positive motor speed = forward.
        #
        # Therefore the PID output can be used directly.

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

            print("Encoder target reached.")

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
# IMU PID CONTROLLER
# Used for STEP 4
# ============================================================

def rotation_pid(robot, target_rotation, kp, ki, kd):
    """
    Rotate the robot in place by a requested angle using
    IMU/compass feedback and PID control.

    target_rotation:
        Desired relative rotation in degrees.

        Positive = counterclockwise
        Negative = clockwise

    Example:
        -180 degrees = 180 degrees clockwise
    """

    # PID variables.
    integral = 0.0
    previous_error = None


    # Heading before the rotation begins.
    previous_heading = robot.get_heading()


    # Total amount the robot has rotated.
    total_rotation = 0.0


    while True:

        # ----------------------------------------------------
        # 1. READ IMU HEADING
        # ----------------------------------------------------

        current_heading = robot.get_heading()


        # ----------------------------------------------------
        # 2. CALCULATE CHANGE IN HEADING
        # ----------------------------------------------------
        #
        # Heading is between 0 and 360 degrees.
        #
        # This formula handles the transition between
        # 359 degrees and 0 degrees correctly.
        #
        # Example:
        #
        # previous = 359 degrees
        # current  = 1 degree
        #
        # actual change = +2 degrees

        delta_heading = (
            current_heading
            - previous_heading
            + 180
        ) % 360 - 180


        # Add this small movement to the total rotation.
        total_rotation += delta_heading


        # ----------------------------------------------------
        # 3. CALCULATE ERROR
        # ----------------------------------------------------
        #
        # error = desired rotation - completed rotation
        #
        # STEP 4:
        #
        # target = -180 degrees
        #
        # Initially:
        #
        # total_rotation = 0
        #
        # error = -180 degrees

        error = target_rotation - total_rotation


        # ----------------------------------------------------
        # 4. PROPORTIONAL TERM
        # ----------------------------------------------------

        proportional = kp * error


        # ----------------------------------------------------
        # 5. INTEGRAL TERM
        # ----------------------------------------------------

        integral += error * DT

        integral_term = ki * integral


        # ----------------------------------------------------
        # 6. DERIVATIVE TERM
        # ----------------------------------------------------

        if previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - previous_error) / DT

        derivative_term = kd * derivative


        # ----------------------------------------------------
        # 7. PID OUTPUT
        # ----------------------------------------------------

        rotation_speed = (
            proportional
            + integral_term
            + derivative_term
        )


        # ----------------------------------------------------
        # 8. SATURATION
        # ----------------------------------------------------

        rotation_speed = max(
            -MAX_SPEED,
            min(MAX_SPEED, rotation_speed)
        )


        # ----------------------------------------------------
        # 9. DISPLAY INFORMATION
        # ----------------------------------------------------

        print(
            f"IMU | "
            f"Heading: {current_heading:.1f} deg | "
            f"Rotation: {total_rotation:.1f} deg | "
            f"Target: {target_rotation:.1f} deg | "
            f"Error: {error:.1f} deg | "
            f"Speed: {rotation_speed:.2f} RPM"
        )


        # ----------------------------------------------------
        # 10. CHECK IF TARGET WAS REACHED
        # ----------------------------------------------------

        if abs(error) <= ROTATION_TOLERANCE:

            robot.stop_motors()

            print("Rotation target reached.")

            break


        # ----------------------------------------------------
        # 11. ROTATE ROBOT IN PLACE
        # ----------------------------------------------------
        #
        # The wheels move in opposite directions.
        #
        # Positive rotation_speed:
        #
        #     left  = negative
        #     right = positive
        #
        #     -> counterclockwise
        #
        # Negative rotation_speed:
        #
        #     left  = positive
        #     right = negative
        #
        #     -> clockwise

        robot.run_motors(
            -rotation_speed,
            rotation_speed
        )


        # ----------------------------------------------------
        # 12. SAVE VALUES FOR NEXT LOOP
        # ----------------------------------------------------

        previous_error = error
        previous_heading = current_heading


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
    # These are starting values only.
    # They must be tuned experimentally on the physical robot.


    # LiDAR PID gains.
    # LiDAR error is measured in millimeters.
    lidar_kp = 0.05
    lidar_ki = 0.0
    lidar_kd = 0.0


    # Encoder PID gains.
    # Encoder error is measured in meters.
    encoder_kp = 150.0
    encoder_ki = 0.0
    encoder_kd = 0.0


    # IMU PID gains.
    # Rotation error is measured in degrees.
    imu_kp = 0.30
    imu_ki = 0.0
    imu_kd = 0.0


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

        time.sleep(1)


        # ====================================================
        # STEP 4
        # IMU PID - Rotate 180 degrees clockwise
        # ====================================================

        print("\nStarting Step 4 - IMU PID Clockwise Rotation")

        rotation_pid(
            robot=robot,
            target_rotation=-180.0,
            kp=imu_kp,
            ki=imu_ki,
            kd=imu_kd
        )

        time.sleep(1)


    finally:

        # ====================================================
        # SAFETY
        # ====================================================
        #
        # Always stop the motors if the program finishes
        # or if an error occurs.

        robot.stop_motors()


if __name__ == "__main__":
    main()
