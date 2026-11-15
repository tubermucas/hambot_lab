from robot_systems.robot import HamBot
from robot_systems.camera import Camera

import time
import math 
import statistics
import random

class MyRobot(HamBot):
    # Physical dimensions
    WHEEL_RADIUS=0.045
    AXLE_WIDTH = 0.205
    DT = 0.1
    MAX_SPEED = 30 # was 60
    MIN_SPEED = 10
    ROTATE_MAX_SP = 30
    ROTATE_MIN_SP = 5
    LIDAR_TOLERANCE = 0.025
    ENCODER_TOLERANCE = 0.3
    ANGLE_TOLERANCE = 2.0
    IMAGE_TOLERANCE = 0.05 # Adjust the tolerance, since it is in pixels. 
    DEPTH_MATCH_TOLERANCE = 0.5 # Adjust the tolerance for depth matching.
    WALL_LOST = 0.4 # was 0.35 and 0.3
    FRONT_CLEARANCE = 0.3
    CELL_LIMIT = 0.45

    # (N, E, S, W) — 1 means "wall on that side"
    MAP = {
        1: (1, 0, 0, 1),
        2: (1, 1, 0, 0),
        3: (1, 0, 1, 1),
        4: (1, 1, 0, 0),
        5: (0, 1, 0, 1),
        6: (0, 0, 1, 1),
        7: (1, 1, 0, 0),
        8: (0, 1, 0, 1),
        9: (0, 0, 0, 1),
        10: (1, 1, 0, 0),
        11: (0, 0, 1, 1),
        12: (0, 1, 0, 0),
        13: (0, 1, 1, 1),
        14: (0, 0, 1, 1),
        15: (1, 1, 1, 0),
        16: (0, 1, 1, 1)
    }



    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD, lidar_enabled=True, camera_enabled=True, camera_type=Camera.CAM_PICAM)

        if self.camera is None:
            self.disconnect_robot()
            raise RuntimeError("Camera not initialized.")
        
        self.camera.set_target_colors(
            (80, 220, 220),
            hue_tolerance=15,
            min_saturation=100,
            min_value=60,
        )

        self.forward_motion = True

    """
    ---- Helper functions ----
    """

    def run_motors(self, left_speed, right_speed):

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
    def saturation(self, speed, cap=MAX_SPEED, rotate=False):
        if not rotate:
            if speed > cap:
                return cap
            elif speed < -cap:
                return -cap
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

    def check_walls(self):
        scan = self.get_range_image()
        
        front_values = [d for d in scan[175:186] if math.isfinite(d) and d > 0]
        front = statistics.median(front_values) if front_values else None

        left_values = [d for d in scan[85:96] if math.isfinite(d) and d > 0]
        left = statistics.median(left_values) if left_values else None

        right_values = [d for d in scan[265:276] if math.isfinite(d) and d > 0]
        right = statistics.median(right_values) if right_values else None

        back_values = [d for d in scan[0:6] + scan[355:360] if math.isfinite(d) and d > 0]
        back = statistics.median(back_values) if back_values else None

        walls = [0, 0, 0, 0] # [forward, right, backward, left]

        if front is not None and front < self.CELL_LIMIT:
            walls[0] = 1
        if right is not None and right < self.CELL_LIMIT:
            walls[1] = 1
        if back is not None and back < self.CELL_LIMIT:
            walls[2] = 1
        if left is not None and left < self.CELL_LIMIT:
            walls[3] = 1

        return walls


    def choose_action(self, prev_action):
        available_actions = self.check_walls()

        if prev_action == "forward" and available_actions[0] == 1:
            return "forward"
        if prev_action == "right" and available_actions[1] == 1:
            return "right"
        if prev_action == "backward" and available_actions[2] == 1:
            return "backward"
        if prev_action == "left" and available_actions[3] == 1:
            return "left"
 
        ones_indices = [i for i, val in enumerate(available_actions) if val == 1]
        random_index = random.choice(ones_indices) if ones_indices else None

        if random_index is not None:
            if random_index == 0:
                return "forward"
            elif random_index == 1:
                return "right"
            elif random_index == 2:
                return "backward"
            elif random_index == 3:
                return "left"

        return None

    def execute_action(self, action):

        if action == "right":
            self.compass_pid(-80, 0.7, 0.02, 0.05)
        if action == "backward":
            self.compass_pid(170, 0.7, 0.02, 0.05)
        if action == "left":
            self.compass_pid(80, 0.7, 0.02, 0.05)

        self.encoder_pid(0.3, 6, 0.2, 0.4)

    def update_particle(self, particle, action):
        cell, orientation = particle

        walls = self.MAP[cell]

        if action == "forward":
            if orientation == "N" and walls[0] == 0:
                cell -= 4
            elif orientation == "E" and walls[1] == 0:
                cell += 1
            elif orientation == "S" and walls[2] == 0:
                cell += 4
            elif orientation == "W" and walls[3] == 0:
                cell -= 1

        elif action == "right":
            if orientation == "N" and walls[1] == 0:
                cell += 1
            elif orientation == "E" and walls[2] == 0:
                cell += 4
            elif orientation == "S" and walls[3] == 0:
                cell -= 1
            elif orientation == "W" and walls[0] == 0:
                cell -= 4

        elif action == "backward":
            if orientation == "N" and walls[2] == 0:
                cell += 4
            elif orientation == "E" and walls[3] == 0:
                cell -= 1
            elif orientation == "S" and walls[0] == 0:
                cell -= 4
            elif orientation == "W" and walls[1] == 0:
                cell += 1

        elif action == "left":
            if orientation == "N" and walls[3] == 0:
                cell -= 1
            elif orientation == "E" and walls[0] == 0:
                cell -= 4
            elif orientation == "S" and walls[1] == 0:
                cell += 1
            elif orientation == "W" and walls[2] == 0:
                cell += 4

        return (cell, orientation)

    def observe_walls(self):
        heading = self.get_heading()

        walls = self.check_walls()

        mesured_walls = [0, 0, 0, 0]  # [N, E, S, W]

        # North
        if 85 <= heading <= 95:
            mesured_walls[0] = walls[0]
            mesured_walls[1] = walls[1]
            mesured_walls[2] = walls[2]
            mesured_walls[3] = walls[3]
        # West
        elif 175 <= heading <= 185:
            mesured_walls[0] = walls[1]
            mesured_walls[1] = walls[0]
            mesured_walls[2] = walls[3]
            mesured_walls[3] = walls[2]

        # South
        elif 265 <= heading <= 275:
            mesured_walls[0] = walls[2]
            mesured_walls[1] = walls[3]
            mesured_walls[2] = walls[0]
            mesured_walls[3] = walls[1]

        # East
        elif heading <= 5 or heading >= 355:
            mesured_walls[0] = walls[3]
            mesured_walls[1] = walls[0]
            mesured_walls[2] = walls[1]
            mesured_walls[3] = walls[2]

        return mesured_walls

    def sensor_model(self, s, z):
        if s == 0 and z == 0:
            return 0.6
        elif s == 0 and z == 1:
            return 0.4
        elif s == 1 and z == 0:
            return 0.2
        elif s == 1 and z == 1:
            return 0.8

    def normilize(weights):
        total_weight = sum(weights)
        if total_weight == 0:
            raise ValueError("Total weight is zero, cannot normalize.")
        return [w / total_weight for w in weights]

    def get_histogram(self, cells):
        histogram = []
        for cell in range(1, 17):
            histogram.append(cells.count(cell))
        return histogram

    def print_grid(self, counts):
        pass

    """
    ---- Localizaion: Particle Filter functions ----
    """

    def particle_filter(self):

        num_particles = 160
        particles = [ (cell, random.choice(["N", "E", "S", "W"])) 
                     for cell in range(1, 17) for _ in range(1, 11) ]

        weights = [1.0 / num_particles] * num_particles

        prev_action = "forward"

        while True: 
            action = self.choose_action(prev_action)

            if action is None:
                print("No available actions: stopping robot.")
                self.stop_motors()
                break

            prev_action = action

            self.execute_action(action)
            for i in range(1, num_particles+1):
                particles[i] = self.update_particle(particles[i], action)

            mes_z = self.observe_walls()
            for i in range(1, num_particles+1):
                particle = particles[i]
                cell = particle[0]
                expected_walls = self.MAP[cell]

                likelihood = 1.0
                for j in range(4):
                    likelihood *= self.sensor_model(expected_walls[j], mes_z[j])
                
                weights[i] = likelihood

            weights = self.normilize(weights)

            drawn = random.choices(particles, weights=weights, k=num_particles)
            particles = [ (p[0], random.choice(['N','E','S','W'])) for p in drawn ]
            weights = [1.0 / num_particles] * num_particles



            counts = self.get_histogram(p[0] for p in particles)
            self.print_grid(counts)
            mode_cell = max(counts)
            if (counts[mode_cell] >= 0.80 * num_particles):
                print(f"Robot is localized in cell {mode_cell}")
                break

            



    """
    ---- Bug Zero Algorithm functions ----
    """
    def check_landmarks(self):
        landmarks = self.camera.find_landmarks()
        if not landmarks:
            return False
        return True

    # Move towards the goal using PID control 
    def move_to_goal(self, kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):
        front_integral = 0
        steering_integral = 0
        prev_front_error = None 
        prev_steering_error = None
        target_distance = 0.25
        image_width = self.get_width()
        
        while True:

            landmarks = self.camera.find_landmarks()

            scan = self.get_range_image()

            front_values = [d for d in scan[170:190] if math.isfinite(d) and d > 0]

            cur_dist = min(front_values) / 1000.0 if front_values else None

            if len(landmarks) > 0:

                # Compute the front movement PID output 
                front_error = cur_dist - target_distance

                forward_speed, front_integral, prev_front_error = self.compute_pid(kp_fw, ki_fw, kd_fw, front_integral, front_error, prev_front_error)

                # Compute the steering PID output
                goal = landmarks[0]
                goal_x = goal.x

                midpoint = image_width / 2

                steering_error = goal_x - midpoint

                steering_output, steering_integral, prev_steering_error = self.compute_pid(kp_st, ki_st, kd_st, steering_integral, steering_error, prev_steering_error)

                turn = -steering_output
                left_speed = forward_speed + turn
                right_speed = forward_speed - turn

                peak_speed = max(abs(left_speed), abs(right_speed))
                if peak_speed > self.MAX_SPEED:
                    scale = self.MAX_SPEED / peak_speed
                    left_speed *= scale
                    right_speed *= scale


                print(f"steering_error: {steering_error}, front_error: {front_error}, left_speed: {left_speed}, right_speed: {right_speed}")
               
                # Check if the robot has reached the goal
                if abs(steering_error) <= self.IMAGE_TOLERANCE * image_width and abs(front_error) <= self.LIDAR_TOLERANCE:
                    self.stop_motors()
                    return True
                

            else:
                # If no landmarks are detected, stop the robot
                self.stop_motors()
                print("No landmarks detected: stopping motors.")
                return False

            self.run_motors(left_speed, right_speed)

            time.sleep(self.DT)

    def follow_wall(self, safe_distance, wall_type,
                    kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):

        if wall_type not in ("left", "right"):
            raise ValueError("wall_type must be 'left' or 'right'")

        target_distance = 0.15

        front_integral = steering_integral = 0.0
        prev_front_error = prev_steering_error = None

        angle_gain = 15.0  # Starting value; motor-command units per radian.

        def sector_median(center):
            values = sorted(
                d / 1000.0
                for d in scan[center - 3:center + 4]
                if math.isfinite(d) and d > 0
            )
            return values[len(values) // 2] if len(values) >= 3 else None

        try:
            while True:
                scan = self.get_range_image()

                front_values = [d for d in scan[170:190] if math.isfinite(d) and d > 0]

                if not front_values:
                    self.stop_motors()
                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    time.sleep(self.DT)
                    continue

                front_distance = min(front_values) / 1000.0

                if self.forward_motion:
                    self.forward_motion = False
                    self.lidar_pid(target_distance * 2, 40, 4, 6)


                if front_distance < self.FRONT_CLEARANCE:
                    angle = -60 if wall_type == "left" else 60
                    self.compass_pid(angle, 0.7, 0.02, 0.05)
                    

                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    continue

                start, stop = (
                    (80, 140) if wall_type == "left" else (220, 280)
                )

                wall_values = [d for d in scan[start:stop] if math.isfinite(d) and d > 0]
                if not wall_values:
                    self.stop_motors()
                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    time.sleep(self.DT)
                    continue
                
                wall_distance = min(wall_values) / 1000.0


                # Forward PID, capped independently of steering.
                front_error = front_distance - target_distance
                forward, front_integral, prev_front_error = self.compute_pid(
                    kp_fw, ki_fw, kd_fw,
                    front_integral, front_error, prev_front_error
                )

                forward = self.saturation(forward, 20)

                # Side-distance steering PID.
                steering_error = wall_distance - safe_distance
                steering, steering_integral, prev_steering_error = (
                    self.compute_pid(
                        kp_st, ki_st, kd_st,
                        steering_integral, steering_error, prev_steering_error
                    )
                )


                # Estimate alignment using side and front-side measurements.
                side_center = 90 if wall_type == "left" else 270
                diagonal_center = 120 if wall_type == "left" else 240

                side = sector_median(side_center)
                diagonal = sector_median(diagonal_center)

                wall_angle = None

                if side is not None and diagonal is not None:
                    # Mirror the geometry for right-wall following:
                    # x = forward distance; y = distance toward the selected side.
                    x = diagonal * math.sin(math.radians(30))
                    y = diagonal * math.cos(math.radians(30))

                    candidate_angle = math.atan2(y - side, x)

                    # Basic plausibility gates. These do NOT prove both readings
                    # belong to the same wall.
                    if (
                        side < safe_distance + 0.20
                        and abs(candidate_angle) <= math.radians(35)
                    ):
                        wall_angle = candidate_angle
                        steering += angle_gain * wall_angle

                steering = self.saturation(steering)
                # Diagnostic constraint for normal wall following.
                forward = max(0.0, forward)
                steering_limit = 0.7 * forward
                steering = max(-steering_limit, min(steering, steering_limit))

                turn = steering if wall_type == "left" else -steering

                left_speed = forward - turn
                right_speed = forward + turn

                scale = max(1.0, abs(left_speed) / self.MAX_SPEED,
                                abs(right_speed) / self.MAX_SPEED)
                
                left_speed /= scale
                right_speed /= scale

                # True assumes the cylinder's full width is visible, without occlusion.
                if self.check_landmarks():
                    self.stop_motors()
                    return True

                print(
                    f"front={front_distance:.3f}, wall={wall_distance:.3f}, "
                    f"forward={forward:.2f}, steering={steering:.2f}, "
                    f"left={left_speed:.2f}, right={right_speed:.2f}"
                )

                self.run_motors(left_speed, right_speed)
                time.sleep(self.DT)

        finally:
            self.stop_motors()


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

            # True assumes the cylinder's full width is visible, without occlusion.
            if self.check_landmarks():
                self.stop_motors()
                return True

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


