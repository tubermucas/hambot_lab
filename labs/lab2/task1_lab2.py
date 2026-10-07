from control.MyRobot import MyRobot
import time


# ============================================================
# LAB 2 - PID CONTROL
# STEPS 1 TO 9
# ============================================================

# The assignment requires a fixed PID update period
# of 0.032 seconds.
DT = 0.032


# ============================================================
# TARGET VALUES
# ============================================================

# LiDAR measurements are returned in millimeters.
# 2 ft = 609.6 mm
LIDAR_TARGET_DISTANCE = 609.6


# Encoder distances are calculated in meters.

# 1 ft = 0.3048 m
ENCODER_TARGET_DISTANCE = 0.3048

# Final Step:
# 2 ft = 0.6096 m
FINAL_ENCODER_TARGET_DISTANCE = 0.6096


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
# 2 degrees
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

    # get_range_image() returns -1 if LiDAR is not enabled.
    if lidar_data == -1:
        raise RuntimeError("LiDAR is not enabled.")

    front_distance = lidar_data[180]

    return front_distance


# ============================================================
# LIDAR PID CONTROLLER
# Used for STEP 1, STEP 3, STEP 5, and STEP 7
# ============================================================

def lidar_pid(robot, target_distance, kp, ki, kd):
    """
    Move the robot until it reaches the desired distance
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
        # error = target - current
        #
        # If the robot is too far from the wall:
        #
        # error < 0
        #
        # Positive motor speed means forward.
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

    The encoders return wheel rotation in radians.

    Linear distance is calculated with:

        distance = angle_in_radians * wheel_radius

    The left and right wheel distances are averaged.

    Returns:
        Distance traveled in meters.
    """

    encoder_readings = robot.get_encoder_readings()

    left_radians = encoder_readings[0]
    right_radians = encoder_readings[1]


    # Convert wheel rotation into linear distance.
    left_distance = left_radians * robot.WHEEL_RADIUS
    right_distance = right_radians * robot.WHEEL_RADIUS


    # Average both wheel distances.
    traveled_distance = (
        left_distance + right_distance
    ) / 2.0


    return traveled_distance


# ============================================================
# ENCODER PID CONTROLLER
# Used for STEP 2, STEP 6, and STEP 9
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

    # Reset encoder accumulators so every new movement
    # starts from zero.
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
        # Therefore:
        #
        # error > 0
        #
        # Positive motor speed means forward.

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
# Used for STEP 4 and STEP 8
# ============================================================

def rotation_pid(robot, target_rotation, kp, ki, kd):
    """
    Rotate the robot in place by a requested angle using
    IMU/compass feedback and PID control.

    target_rotation:
        Desired relative rotation in degrees.

        Positive = counterclockwise
        Negative = clockwise
    """

    # PID variables.
    integral = 0.0
    previous_error = None


    # Read heading before rotation begins.
    previous_heading = robot.get_heading()


    # Total rotation completed during this movement.
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
        # get_heading() returns values between
        # 0 and 360 degrees.
        #
        # This formula correctly handles crossing
        # between 359 degrees and 0 degrees.

        delta_heading = (
            current_heading
            - previous_heading
            + 180
        ) % 360 - 180


        # Add the small heading change to total rotation.
        total_rotation += delta_heading


        # ----------------------------------------------------
        # 3. CALCULATE ERROR
        # ----------------------------------------------------
        #
        # error = desired rotation - completed rotation

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
        # Wheels move in opposite directions.
        #
        # Negative target -> clockwise
        # Positive target -> counterclockwise

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
    # These are STARTING VALUES.
    #
    # The final gains must be tuned experimentally on
    # the physical robot.


    # --------------------------------------------------------
    # LiDAR PID gains
    # --------------------------------------------------------

    lidar_kp = 0.05
    lidar_ki = 0.0
    lidar_kd = 0.0


    # --------------------------------------------------------
    # Encoder PID gains
    # --------------------------------------------------------

    encoder_kp = 150.0
    encoder_ki = 0.0
    encoder_kd = 0.0


    # --------------------------------------------------------
    # IMU PID gains
    # --------------------------------------------------------

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


        # ====================================================
        # STEP 5
        # LiDAR PID - Stop 2 ft from the wall
        # ====================================================

        print("\nStarting Step 5 - LiDAR PID")

        lidar_pid(
            robot=robot,
            target_distance=LIDAR_TARGET_DISTANCE,
            kp=lidar_kp,
            ki=lidar_ki,
            kd=lidar_kd
        )

        time.sleep(1)


        # ====================================================
        # STEP 6
        # Encoder PID - Drive forward 1 ft
        # ====================================================

        print("\nStarting Step 6 - Encoder PID")

        encoder_pid(
            robot=robot,
            target_distance=ENCODER_TARGET_DISTANCE,
            kp=encoder_kp,
            ki=encoder_ki,
            kd=encoder_kd
        )

        time.sleep(1)


        # ====================================================
        # STEP 7
        # LiDAR PID - Back up until 2 ft from the wall
        # ====================================================

        print("\nStarting Step 7 - LiDAR PID Backup")

        lidar_pid(
            robot=robot,
            target_distance=LIDAR_TARGET_DISTANCE,
            kp=lidar_kp,
            ki=lidar_ki,
            kd=lidar_kd
        )

        time.sleep(1)


        # ====================================================
        # STEP 8
        # IMU PID - Rotate 180 degrees counterclockwise
        # ====================================================

        print("\nStarting Step 8 - IMU PID Counterclockwise Rotation")

        rotation_pid(
            robot=robot,
            target_rotation=180.0,
            kp=imu_kp,
            ki=imu_ki,
            kd=imu_kd
        )

        time.sleep(1)


        # ====================================================
        # STEP 9
        # Encoder PID - Final forward drive of 2 ft
        # ====================================================
        #
        # Step 9 uses the same encoder PID controller.
        #
        # The difference is the requested distance:
        #
        # Steps 2 and 6:
        #     1 ft = 0.3048 m
        #
        # Step 9:
        #     2 ft = 0.6096 m

        print("\nStarting Step 9 - Final Encoder PID")

        encoder_pid(
            robot=robot,
            target_distance=FINAL_ENCODER_TARGET_DISTANCE,
            kp=encoder_kp,
            ki=encoder_ki,
            kd=encoder_kd
        )

        # Keep the robot stopped briefly at the final position.
        time.sleep(1)


    finally:

        # ====================================================
        # SAFETY / CLEAN SHUTDOWN
        # ====================================================
        #
        # disconnect_robot() stops the motors and shuts down
        # the robot sensors/background threads safely.

        robot.disconnect_robot()


if __name__ == "__main__":
    main()
