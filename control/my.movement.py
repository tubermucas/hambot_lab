from robot_systems.robot import HamBot


class MyRobot(HamBot):

    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD)

    def wheel_orientation(self):
        readings = self.get_encoder_readings()
        return sum(readings) / len(readings)

    def move_forward(self, speed, dist):
        init_theta = self.wheel_orientation()

        # Convert the requested distance into wheel rotation in radians
        dist_rad = dist / self.WHEEL_RADIUS

        self.set_left_motor_speed(speed)
        self.set_right_motor_speed(speed)

        while True:
            cur_theta = self.wheel_orientation()

            if cur_theta - init_theta >= dist_rad:
                self.stop_motors()
                break

    def rotate(self, speed, angle):
        prev_angle = self.get_heading()
        total_rotation = 0

        if angle < 0:
            left_speed = speed
            right_speed = -speed
        else:
            left_speed = -speed
            right_speed = speed

        self.set_left_motor_speed(left_speed)
        self.set_right_motor_speed(right_speed)

        while True:
            cur_angle = self.get_heading()

            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180

            total_rotation += delta_angle
            prev_angle = cur_angle

            if abs(total_rotation) >= abs(angle):
                self.stop_motors()
                break
#Lokman part
      def drive_circle(self, radius, direction, outer_speed=40):
        AXLE_WIDTH = 0.205
        half_axle = AXLE_WIDTH / 2

        if radius <= half_axle:
            raise ValueError("Radius is too small for the robot.")

        direction = direction.upper()

        if direction not in ("CCW", "CW"):
            raise ValueError("Direction must be 'CCW' or 'CW'.")

        speed_ratio = (radius - half_axle) / (radius + half_axle)
        inner_speed = outer_speed * speed_ratio

        if direction == "CCW":
            left_speed = inner_speed
            right_speed = outer_speed
        else:
            left_speed = outer_speed
            right_speed = inner_speed

        prev_heading = self.get_heading()
        total_rotation = 0

        self.set_left_motor_speed(left_speed)
        self.set_right_motor_speed(right_speed)

        while True:
            current_heading = self.get_heading()

            delta_heading = (
                (current_heading - prev_heading + 180) % 360
            ) - 180

            total_rotation += delta_heading
            prev_heading = current_heading

            if abs(total_rotation) >= 360:
                self.stop_motors()
                break



