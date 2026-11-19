from control.MyRobot import MyRobot


def main():
    robot = MyRobot()
    try:
        robot.localize()
    finally:
        robot.disconnect_robot()


if __name__ == "__main__":
    main()
