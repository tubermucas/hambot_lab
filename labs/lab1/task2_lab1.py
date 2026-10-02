from control.MyRobot import MyRobot
import time

TIME = 0.5

def rectangle(robot, speed_for, speed_rot, length, width):
    """Trace a rectangle and finish at the starting position."""
    # Start halfway along a long side so the final leg returns to the start.
    robot.move_forward(speed_for, length/2)
    print("reached the first corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)

    robot.move_forward(speed_for, width)
    print("reached the second corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)
    robot.move_forward(speed_for, length)
    print("reached the third corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)
    robot.move_forward(speed_for, width)
    print("reached the forth corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)
    robot.move_forward(speed_for, length/2)
    print("reached the initial position")
    time.sleep(TIME)

def triangle(robot, speed_for, speed_rot, side):
    """Trace an equilateral triangle with the supplied side length."""
    robot.move_forward(speed_for, side)
    print("reached the first corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, 120)
    time.sleep(TIME)
    robot.move_forward(speed_for, side)
    print("reached the second corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, 120)
    time.sleep(TIME)
    robot.move_forward(speed_for, side)
    time.sleep(TIME)
    robot.rotate(speed_rot, 140)
    print("reached the initial position")
    time.sleep(TIME)


def main():
    # Configure one robot instance and the shared speeds for every shape.
    robot = MyRobot()
    forward_sp = 45
    rotate_sp = 20
    first_ang_sp = 4
    second_ang_sp = 5


    # Distances are in metres; circular angular speeds follow MyRobot's
    # positive-CCW / negative-CW convention.
    rec_length = 1.0
    rec_width = 1.0
    first_rad = 0.3
    second_rad = 0.5
    side_tri = 0.6

    print("Start rectangle")
    print("-" * 20)
    rectangle(robot, speed_for=forward_sp, speed_rot=rotate_sp, length=rec_length, width=rec_width)
    print("-" * 20 + "\n")

    print("Start the first circle")
    print("-" * 20)
    robot.drive_circle(radius=first_rad, angular_speed=first_ang_sp)
    print("-" * 20 + "\n")

    # Pause between shapes so the robot is fully stationary before continuing.
    time.sleep(2)

    print("Start the second circle")
    print("-" * 20)
    robot.drive_circle(radius=second_rad, angular_speed=-second_ang_sp)
    print("-" * 20 + "\n")

    time.sleep(2)

    print("Start triangle")
    print("-" * 20)
    triangle(robot, speed_for=forward_sp, speed_rot=rotate_sp, side=side_tri)
    print("-" * 20 + "\n")


if __name__ == "__main__":
    main()
    
