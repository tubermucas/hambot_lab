from control.MyRobot import MyRobot
import time

TIME = 0.7

def rectangle(robot, speed_for, speed_rot, length, width):
    """Trace a rectangle and finish at the starting position."""
    # Start halfway along a long side so the final leg returns to the start.
    robot.move_forward(speed_for, length/2)

    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)

    robot.move_forward(speed_for, width)

    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)

    robot.move_forward(speed_for, length)
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)

    robot.move_forward(speed_for, width)
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)
    robot.move_forward(speed_for, length/2)

    time.sleep(TIME)

def triangle(robot, speed_for, speed_rot, side):
    """Trace an equilateral triangle with the supplied side length."""
    robot.move_forward(speed_for, side)

    time.sleep(TIME)

    robot.rotate(speed_rot, 117)
    time.sleep(TIME)

    robot.move_forward(speed_for, side)
    time.sleep(TIME)

    robot.rotate(speed_rot, 117)
    time.sleep(TIME)

    robot.move_forward(speed_for, side)
    time.sleep(TIME)

    robot.rotate(speed_rot, 137)

    time.sleep(TIME)


def main():
    # Configure one robot instance and the shared speeds for every shape.
    robot = MyRobot()
    forward_sp = 60
    rotate_sp = 30
    angular_sp = 6


    # Distances are in metres; circular angular speeds follow MyRobot's
    # positive-CCW / negative-CW convention.
    rec_length = 0.6
    rec_width = 1
    first_rad = 0.25
    second_rad = 0.5
    side_tri = 0.5
    
    rectangle(robot, speed_for=forward_sp, speed_rot=rotate_sp, length=rec_length, width=rec_width)
    robot.drive_circle(radius=first_rad, angular_speed=angular_sp)
    
    # Pause between shapes so the robot is fully stationary before continuing.
    time.sleep(1)
    robot.drive_circle(radius=second_rad, angular_speed=-angular_sp)
    
    time.sleep(1)
    triangle(robot, speed_for=forward_sp, speed_rot=rotate_sp, side=side_tri)


if __name__ == "__main__":
    main()
    
