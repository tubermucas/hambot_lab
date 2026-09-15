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
      def drive_circle(self, radius, angular_speed):
    AXLE_WIDTH = 0.205
    half_axle = AXLE_WIDTH / 2

    omega = abs(angular_speed)

    inner_speed_mps = omega * (radius - half_axle)
    outer_speed_mps = omega * (radius + half_axle)

    # Convert linear wheel speed to RPM
    inner_rpm = (
        inner_speed_mps / self.WHEEL_RADIUS
    ) * 60 / (2 * 3.141592653589793)

    outer_rpm = (
        outer_speed_mps / self.WHEEL_RADIUS
    ) * 60 / (2 * 3.141592653589793)

    # Positive angular speed = CCW
    if angular_speed > 0:
        left_speed = inner_rpm
        right_speed = outer_rpm

    # Negative angular speed = CW
    else:
        left_speed = outer_rpm
        right_speed = inner_rpm

    prev_heading = self.get_compass_reading()
    total_rotation = 0

    self.set_left_motor_speed(left_speed)
    self.set_right_motor_speed(right_speed)

    while True:
        current_heading = self.get_compass_reading()

        delta_heading = (
            (current_heading - prev_heading + 180) % 360
        ) - 180

        total_rotation += delta_heading
        prev_heading = current_heading

        if abs(total_rotation) >= 360:
            self.stop_motors()
            break



