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

    robot.rotate(speed_rot, 115)
    time.sleep(TIME)

    robot.move_forward(speed_for, side)
    time.sleep(TIME)

    robot.rotate(speed_rot, 115)
    time.sleep(TIME)

    robot.move_forward(speed_for, side)
    time.sleep(TIME)
    
    robot.rotate(speed_rot, 135)

    time.sleep(TIME)


def main():
    robot = MyRobot()
    """
    sp_f = 40.0
    sp_r = 20.0
    len = 1.0
    wid = 1.0

    rectangle(robot, sp_f, sp_r, len, wid)  

    """
    
    triangle(robot, 60, 30, 0.5)

    """
    testing of rotation
    optimal speed for the rotating in place with no movements before or after: 10 rpm

    while True:

        speed = float(input("enter speed: "))
        rotation = float(input("enter the degree: "))

        motion.rotate(speed, rotation)    
    
    """
    """
    
    # speed which is good rn is 21 rpm
    speed = float(input("enter spead: "))
    dis = float(input("enter distance: "))
    speed_rot = float(input("enter rotation speed: "))
    degree = float(input("enter degree: "))


    print(f"move to {dis} at {speed}")
    robot.move_forward(speed, dis)

    time.sleep(1)

    print("rotating")
    robot.rotate(speed_rot, degree)


    time.sleep(1)

    print(f"move to {dis} at {speed}")
    robot.move_forward(speed, dis)
    """
    
    """
    test for move_forward function
    the most optimal speed is 17-18

    while True: 
        speed = float(input("enter speed: "))
        distance = float(input("enter distance: "))

        motion.move_forward(speed, distance)

    """

    
    
    """
    forward_sp = int(input("Enter speed: "))
    #rotate_sp = int(input("Enter rot speed: "))
    second_rad = float(input("Enter radius ICC: "))
    angular_sp = int(input("Enter angular speed: "))

    #side_tri = 1

    print("Start the second circle")
    print("-" * 20)
    robot.drive_circle(radius=second_rad, angular_speed=-angular_sp)
    print("-" * 20 + "\n")

    time.sleep(2) 

    print("First forward")
    print("-" * 20)
    robot.move_forward(forward_sp, 0.5)
    print("-" * 20 + "\n") 

    time.sleep(1)

    print("Second forward")
    print("-" * 20)
    robot.move_forward(forward_sp, 0.5)
    print("-" * 20 + "\n") 

    """


if __name__ == "__main__":
    main()