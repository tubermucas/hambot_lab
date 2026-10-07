from robot_systems.robot import HamBot
import time


class MyRobot(HamBot):
    # Physical dimensions
    WHEEL_RADIUS=0.045
    AXLE_WIDTH = 0.205

    # The lab requires a fixed PID update period of 0.032 seconds.
    DT = 0.032

    # Maximum motor command allowed during the PID controllers.
    MAX_SPEED = 60

    # LiDAR tolerance:
    # 20 mm = 2 cm.
    LIDAR_TOLERANCE = 20.0

    # Encoder tolerance:
    # 0.01 m = 1 cm.
    ENCODER_TOLERANCE = 0.01


    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD, lidar_enabled=True)

        # Keeping track the last turned wheel
        # True - left wheel, False - right wheel
        self.last_wheel = True



    def get_front_distance(self):
        
        lidar_data = self.get_range_image()
        return lidar_data[180]


    def get_aver_encoders(self):
        """Return the mean wheel-encoder position in radians."""
        return sum(self.get_encoder_readings()) / len(self.get_encoder_readings())

    
    def run_motors(self, left_speed, right_speed):
        # Run motors accordiung to the last turned wheel
        if self.last_wheel:
            self.set_left_motor_speed(left_speed)
            self.set_right_motor_speed(right_speed)
        else:
            self.set_right_motor_speed(right_speed)
            self.set_left_motor_speed(left_speed)


    def lidar_pid(self, target_distance, kp, ki, kd):
        integral = 0
        prev_error = None

        while True:
            cur_dist = self.get_front_distance()
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

            motor_speed = -output
            motor_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, motor_speed))

            if abs(error) <= self.LIDAR_TOLERANCE:

                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)

            prev_error = error

            time.sleep(self.DT)


    def encoder_pid(self, target_dist, direction, kp, ki, kd):

        start_pos = self.get_aver_encoders()
        integral = 0.0
        prev_error = None
        
        # Convert the distance to radians
        dist_rad = round(target_dist / self.WHEEL_RADIUS, 4)

        while True:
            cur_tetha = self.get_aver_encoders()
            traveled = cur_tetha - start_pos

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

            motor_speed = output * direction
            motor_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, motor_speed))

            if abs(error) <= self.ENCODER_TOLERANCE:
                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)

            prev_error = error

            time.sleep(self.DT)


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

            self.last_wheel = True
        else:
            angle += 23
            left_speed = speed
            right_speed = -speed

            self.last_wheel = False
        
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

            self.last_wheel = True

        # Negative angular speed means clockwise
        else:
            left_speed = outer_speed
            right_speed = inner_speed

            self.last_wheel = False

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


