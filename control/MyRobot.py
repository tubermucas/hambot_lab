from robot_systems.robot import HamBot
import time


class MyRobot(HamBot):
    # Physical dimensions
    WHEEL_RADIUS=0.045
    AXLE_WIDTH = 0.205

    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD)

        # Keeping track the last turned wheel
        # True - left wheel, False - right wheel
        self.last_wheel = True


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


