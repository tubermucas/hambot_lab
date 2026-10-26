from robot_systems.robot import HamBot
from robot_systems.camera import Camera
import time


class MyRobot(HamBot):
    # Physical dimensions
    WHEEL_RADIUS=0.045
    AXLE_WIDTH = 0.205
    DT = 0.1
    MAX_SPEED = 60
    MIN_SPEED = 10
    ROTATE_MAX_SP = 30
    ROTATE_MIN_SP = 5
    LIDAR_TOLERANCE = 0.025
    ENCODER_TOLERANCE = 0.3
    ANGLE_TOLERANCE = 2.0
    IMAGE_TOLERANCE = 0.025 # Adjust the tolerance, since it is in pixels. 
    DEPTH_MATCH_TOLERANCE = 0.5 # Adjust the tolerance for depth matching.


    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD, lidar_enabled=True, camera_enabled=True, camera_type=Camera.CAM_OAKD)

        if self.camera is None:
            raise RuntimeError("Camera not initialized.")
        
        self.camera.set_target_colors(
            (30, 220, 220),       # OpenCV HSV: approximate yellow
            hue_tolerance=10,
            min_saturation=100,
            min_value=60,
        )

        # Keeping track the last turned wheel
        # True - left wheel, False - right wheel
        #self.last_wheel = True

    """
    ---- Helper functions ----
    """

    # Run motors with the given speeds for left and right wheels
    def run_motors(self, left_speed, right_speed):
        

        """
        # Run motors according to the last turned wheel
        if self.last_wheel:
            self.set_left_motor_speed(left_speed)
            self.set_right_motor_speed(right_speed)
        else:
            self.set_right_motor_speed(right_speed)
            self.set_left_motor_speed(left_speed)

        """

        self.set_right_motor_speed(right_speed)
        self.set_left_motor_speed(left_speed)

    # Return the distance to the front object in metres, using the LiDAR. If no object is detected, return 100 metres.
    def get_front_distance(self, timeout=0.1):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            front = self.get_range_image()[180]

            if front > 0:
                return front / 1000.0  # metres
            elif front < 0:
                return 100.0

            time.sleep(0.005)

        raise RuntimeError("LiDAR reading timed out.")
    
    # Saturate the speed to be within the allowed range
    def saturation(self, speed, rotate=False):
        if not rotate:
            if speed > self.MAX_SPEED:
                return self.MAX_SPEED
            elif speed < -self.MAX_SPEED:
                return -self.MAX_SPEED
            elif 0 < speed < self.MIN_SPEED:
                return self.MIN_SPEED
            elif -self.MIN_SPEED < speed < 0:
                return -self.MIN_SPEED
            else:
                return speed
        else:
            if speed > self.ROTATE_MAX_SP:
                return self.ROTATE_MAX_SP
            elif speed < -self.ROTATE_MAX_SP:
                return -self.ROTATE_MAX_SP
            elif 0 < speed < self.ROTATE_MIN_SP:
                return self.ROTATE_MIN_SP
            elif -self.ROTATE_MIN_SP < speed < 0:
                return -self.ROTATE_MIN_SP
            else:
                return speed

    # Return the average of the left and right wheel encoder readings in radians
    def get_aver_encoders(self):
        """Return the mean wheel-encoder position in radians."""
        return sum(self.get_encoder_readings()) / len(self.get_encoder_readings())

    # Compute the PID output based on the current error and previous error
    def compute_pid(self, kp, ki, kd, integral, error, prev_error):
        proportional = kp * error

        integral += error * self.DT
        integral_term = ki * integral

        if prev_error is not None:
            derivative = (error - prev_error) / self.DT
        else:
            derivative = 0.0

        derivative_term = kd * derivative

        output = proportional + integral_term + derivative_term

        return output, integral, error

    # Return the width of the camera frame in pixels
    def get_width(self):
        frame = self.camera.get_frame()

        if frame is not None:
            return frame.shape[1]  # Return the width of the frame
        return 0

    """
    ---- Bug Zero Algorithm functions ----
    """

    # Move towards the goal using PID control 
    def move_to_goal(self, kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):
        front_integral = 0
        steering_integral = 0
        prev_front_error = None 
        prev_steering_error = None
        target_distance = 0.25

        while True:

            landmarks = self.camera.find_landmarks()

            cur_dist = self.get_front_distance()

            if len(landmarks) > 0:

                # Compute the front movement PID output 
                front_error = target_distance - cur_dist

                forward_speed, front_integral, prev_front_error = self.compute_pid(kp_fw, ki_fw, kd_fw, front_integral, front_error, prev_front_error)
                forward_speed = -forward_speed  # Invert the speed to move forward

                # Compute the steering PID output
                goal = landmarks[0]
                goal_x = goal.x
                goal_depth = goal.depth / 1000.0 # meters

                midpoint = self.get_width() / 2

                steering_error = goal_x - midpoint

                steering_output, steering_integral, prev_steering_error = self.compute_pid(kp_st, ki_st, kd_st, steering_integral, steering_error, prev_steering_error)

                """
                Checking though AI says this part is not needed, but lets keep it for now.
                if steering_error < 0: 
                    left_speed = self.saturation(forward_speed - steering_output)
                    right_speed = self.saturation(forward_speed + steering_output)
                elif steering_error > 0:
                    left_speed = self.saturation(forward_speed + steering_output)
                    right_speed = self.saturation(forward_speed - steering_output)
                else: 
                    left_speed = self.saturation(forward_speed)
                    right_speed = self.saturation(forward_speed)
                """

                # Saturate the calcualated outputs
                left_speed = self.saturation(forward_speed + steering_output)
                right_speed = self.saturation(forward_speed - steering_output)

                # Adjust Hyperparameter
                goal_centered = abs(steering_error) <= self.IMAGE_TOLERANCE 

                front_return_matches_goal = (
                    goal_centered and abs(goal_depth - cur_dist) <= self.DEPTH_MATCH_TOLERANCE
                )

                # Check if the robot has reached the goal
                if front_return_matches_goal and abs(front_error) <= self.LIDAR_TOLERANCE:
                    self.stop_motors()
                    return True

                # If the object before the goal, stop the robot and return False
                if abs(front_error) <= self.LIDAR_TOLERANCE and not front_return_matches_goal:
                    self.stop_motors()
                    return False
                
            else:
                # If no landmarks and an object is nearby, return False
                if cur_dist < target_distance:
                    return False
                
                # If no landmarks and no object nearby, rotate in place to search for the goal
                else:
                    left_speed = 20
                    right_speed = -20

            self.run_motors(left_speed, right_speed)

            time.sleep(self.DT)

    # Follow the wall using PID control
    def follow_wall(self, safe_distance, wall_type, kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):
        front_integral = 0
        steering_integral = 0
        prev_front_error = None
        prev_steering_error = None
        target_distance = 0.25

        while True: 

            # Compute the front movement PID output
            cur_dist = self.get_front_distance()
            
            front_error = target_distance - cur_dist

            forward_speed, front_integral, prev_front_error = self.compute_pid(kp_fw, ki_fw, kd_fw, front_integral, front_error, prev_front_error)
            forward_speed = -forward_speed  # Invert the speed to move forward

            # Compute the steering PID output based on the wall type
            if wall_type == "left":

                # Compute the steering error based on the left wall distance
                left_distance = min(self.get_range_image()[80:160]) / 1000.0
                steering_error = left_distance - safe_distance

                steering_output, steering_integral, prev_steering_error = self.compute_pid(kp_st, ki_st, kd_st, steering_integral, steering_error, prev_steering_error)

                """
                Checking though AI says this part is not needed, but lets keep it for now.
                steering_output = abs(steering_output)

                if steering_error < 0: 
                    left_speed = self.saturation(forward_speed+steering_output)
                    right_speed = self.saturation(forward_speed-steering_output)
                elif steering_error > 0:
                    left_speed = self.saturation(forward_speed-steering_output)
                    right_speed = self.saturation(forward_speed+steering_output)
                else:
                    left_speed = self.saturation(forward_speed)
                    right_speed = self.saturation(forward_speed)

                """
                # Saturate the calcualated outputs
                left_speed = self.saturation(forward_speed - steering_output)
                right_speed = self.saturation(forward_speed + steering_output)

                # If the robot is too close to the wall, turn away from it
                if cur_dist < 0.55: # Hyper parameter
                    self.compass_pid(-90, 0.7, 0.02, 0.05) # Applied pid numbers from lab 2 

                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None

                    continue
                    
            elif wall_type == "right":

                # Compute the steering error based on the right wall distance
                right_distance = min(self.get_range_image()[200:280]) / 1000.0
                steering_error = right_distance - safe_distance

                steering_output, steering_integral, prev_steering_error = self.compute_pid(kp_st, ki_st, kd_st, steering_integral, steering_error, prev_steering_error)
               

                """
                Checking though AI says this part is not needed, but lets keep it for now.
                steering_output = abs(steering_output)
                if steering_error < 0: 
                    left_speed = self.saturation(forward_speed-steering_output)
                    right_speed = self.saturation(forward_speed+steering_output)
                elif steering_error > 0:
                    left_speed = self.saturation(forward_speed+steering_output)
                    right_speed = self.saturation(forward_speed-steering_output)
                else:
                    left_speed = self.saturation(forward_speed)
                    right_speed = self.saturation(forward_speed)

                """

                # Saturate the calcualated outputs
                left_speed = self.saturation(forward_speed + steering_output)
                right_speed = self.saturation(forward_speed - steering_output)

                # If the robot is too close to the wall, turn away from it
                if cur_dist < 0.55: # Hyper parameter
                    # Applied pid numbers from lab 2 
                    self.compass_pid(90, 0.7, 0.02, 0.05) # Adjust the turning angle

                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None

                    continue

            else:
                print("Invalid wall type. Use 'left' or 'right'.")
                return
            
            # Check if the goal is detected while following the wall
            if self.camera.find_landmarks():
                return True

            self.run_motors(left_speed, right_speed)

            time.sleep(self.DT)

    # Bug Zero algorithm that combines moving to the goal and following the wall
    def bug_zero(self, safe_distance, wall_type, pid_mtg_params, pid_wf_params):
        mtg_kp_fw, mtg_ki_fw, mtg_kd_fw, mtg_kp_st, mtg_ki_st, mtg_kd_st = pid_mtg_params
        wf_kp_fw, wf_ki_fw, wf_kd_fw, wf_kp_st, wf_ki_st, wf_kd_st = pid_wf_params

        while True:
            # Check if the goal is detected
            if self.move_to_goal(mtg_kp_fw, mtg_ki_fw, mtg_kd_fw, mtg_kp_st, mtg_ki_st, mtg_kd_st):
                print("Goal reached")
                return True

            # If the goal is not detected, follow the wall
            if self.follow_wall(safe_distance, wall_type, wf_kp_fw, wf_ki_fw, wf_kd_fw, wf_kp_st, wf_ki_st, wf_kd_st):
                continue

    
    """
    ---- PID control functions ----
    """


    def lidar_pid(self, target_distance, kp, ki, kd):
        integral = 0
        prev_error = None

        while True:
            cur_dist = self.get_front_distance()

            if cur_dist is None:
                self.stop_motors()
                print("Invalid LiDAR reading: stopping motors.")
                break   

            error = target_distance - cur_dist

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            motor_speed = self.saturation(-output)

            print(
                f"d={cur_dist:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, speed={motor_speed:.2f}"
            )

            if abs(error) <= self.LIDAR_TOLERANCE:

                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)
            prev_error = error
            time.sleep(self.DT)


    def encoder_pid(self, target_dist, kp, ki, kd):

        start_pos = self.get_aver_encoders()
        integral = 0.0
        prev_error = None
        
        # Convert the distance to radians
        dist_rad = target_dist / self.WHEEL_RADIUS

        while True:
            cur_theta = self.get_aver_encoders()

            if cur_theta is None:
                self.stop_motors()
                print("Invalid Encoder reading: stopping motors.")
                break 

            traveled = cur_theta - start_pos

            error = dist_rad - traveled

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            motor_speed = self.saturation(output)

            print(
                f"trav={traveled:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, speed={motor_speed:.2f}"
            )

            if abs(error) <= self.ENCODER_TOLERANCE:
                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)

            prev_error = error

            time.sleep(self.DT)

    def compass_pid(self, target_angle, kp, ki, kd):

        prev_angle = self.get_heading()

        integral = 0.0
        prev_error = None
        total_rotation = 0

        if target_angle >= 0:
            #Rotate counter-clockwise
            left = -1
            right = 1
            direction = 1
        else:
            #Rotate clockwise
            left = 1
            right = -1
            direction = -1

        while True:
            cur_angle = self.get_heading(blocking=True)

            if cur_angle is None:
                self.stop_motors()
                print("Invalid heading: stopping motors.")
                break

            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180
            total_rotation += direction * delta_angle
            prev_angle = cur_angle

            error = abs(target_angle) - total_rotation

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            left_speed = self.saturation(left * output, rotate=True)
            right_speed = self.saturation(right * output, rotate=True)

            print(
                f"rotated={total_rotation:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, l_speed={left_speed:.2f}, r_speed={right_speed:.2f}"
            )

            if abs(error) <= self.ANGLE_TOLERANCE:
                self.stop_motors()
                break

            self.run_motors(left_speed, right_speed)

            prev_error = error

            time.sleep(self.DT)

    """
    ---- Basic movement functions ----
    """

    def move_forward(self, speed, dist):
        """Drive forward for ``dist`` metres."""

        start_pos = self.get_aver_encoders()
        
        # Convert the distance in metres
        dist_rad = round(dist / self.WHEEL_RADIUS, 4)

        self.run_motors(speed, speed)

        while True:
            cur_tetha = self.get_aver_encoders()
            traveled = cur_tetha - start_pos

            # Stop when the robot has traveled the requested distance
            if traveled >= dist_rad - 1:
                
                self.stop_motors()

                break

            time.sleep(0.1)

    def rotate(self, speed, angle):
        """Turn in place by ``angle`` degrees; positive angles are CCW."""

        prev_angle = self.get_heading() 
        total_rotation = 0

        if angle > 0:
            # Empirical offset compensates for the robot's turning overshoot.
            angle -= 23
            left_speed = -speed
            right_speed = speed

            #self.last_wheel = True
        else:
            angle += 23
            left_speed = speed
            right_speed = -speed

            #self.last_wheel = False
        
        self.run_motors(left_speed, right_speed)

        while True:

            cur_angle = self.get_heading()

            # Normalize the heading change to the shortest signed angle across 0°/360°
            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180

            total_rotation += delta_angle
            prev_angle = cur_angle

            if abs(total_rotation) >= abs(angle) - 1:
                self.stop_motors()
                break

            time.sleep(0.01) 

    def drive_circle(self, radius, angular_speed):
        """Drive one full circle with the requested radius and angular speed."""

        half_axle = self.AXLE_WIDTH / 2

        omega = abs(angular_speed)

        inner_speed = omega * (radius - half_axle) / self.WHEEL_RADIUS
        outer_speed = omega * (radius + half_axle) / self.WHEEL_RADIUS
        
        # Positive angular speed means counter-clockwise
        if angular_speed > 0:
            left_speed = inner_speed
            right_speed = outer_speed

            #self.last_wheel = True

        # Negative angular speed means clockwise
        else:
            left_speed = outer_speed
            right_speed = inner_speed

            #self.last_wheel = False

        prev_heading = self.get_heading()
        total_rotation = 0

        self.run_motors(left_speed, right_speed)

        while True:
            current_heading = self.get_heading()

            # Normalize the heading change to the shortest signed angle across 0°/360°
            delta_heading = ((current_heading - prev_heading + 180) % 360) - 180

            total_rotation += delta_heading
            prev_heading = current_heading

            # Stop when the robot has completed a full circle
            if abs(total_rotation) >= 345:
                self.stop_motors()
                break

            time.sleep(0.1)


