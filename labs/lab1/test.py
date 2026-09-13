from control.movement import Motion

def ractangle(robot, speed, length, width):
    robot.move_forward(speed, length/2)
    robot.rotate(speed, -90)
    robot.move_forward(speed, width)
    robot.rotate(speed, -90)
    robot.move_forward(speed, length)
    robot.rotate(speed, -90)
    robot.move_forward(speed, width)
    robot.rotate(speed, -90)
    robot.move_forward(speed, length/2)


def main():
    motion = Motion()
    motion.move_forward(13, 1.0)
    motion.rotate(5, 90)


if __name__ == "__main__":
    main()