from robot_systems.robot import HamBot 

class Motion: 
    def __init__(self):
        self.bot = HamBot(drivetrain=HamBot.DRIVE_2WD)

    def wheel_orientation(self):
        return sum(self.bot.get_encoder_readings()) / len(self.bot.get_encoder_readings())

    def move_forward(self, speed, dist):
        init_tetha = self.wheel_orientation()
        dist_rad = dist / self.bot.WHEEL_RADIUS

        self.bot.set_left_motor_speed(speed)
        self.bot.set_right_motor_speed(speed)

        while True:
            cur_tetha = self.wheel_orientation()

            if cur_tetha - init_tetha >= dist_rad:
                self.bot.stop_motors()
                break

    def rotate(self, speed, angle):
        prev_angle = self.bot.get_heading() 
        total_rotation = 0

        if angle < 0:
            left_speed = speed
            right_speed = -speed
        else:
            left_speed = -speed
            right_speed = speed
        
        self.bot.set_left_motor_speed(left_speed)
        self.bot.set_right_motor_speed(right_speed)

        while True:
            cur_angle = self.bot.get_heading()

            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180

            total_rotation += delta_angle
            prev_angle = cur_angle

            if abs(total_rotation) >= abs(angle):
                self.bot.stop_motors()
                break