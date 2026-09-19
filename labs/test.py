from hambot_lab.control.MyRobot import MyRobot
import time

TIME = 0.5

def rectangle(robot, speed_for, speed_rot, length, width):
    robot.move_forward(speed_for, round(length/2, 2))
    print("reached the first corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME)

    robot.move_forward(speed_for, width)
    print("reached the second corner")
    time.sleep(TIME)

    robot.rotate(speed_rot, -90)
    time.sleep(TIME/5)
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
    robot.move_forward(speed_for, side)
    print("reached the first corner")
    time.sleep(5e-9)

    robot.rotate(speed_rot, 120)
    robot.move_forward(speed_for, side)
    print("reached the second corner")

    robot.rotate(speed_rot, 120)
    robot.move_forward(speed_for, side)
    robot.rotate(speed_rot, 120)
    print("reached the initial position")


def main():
    robot = MyRobot()
    
    sp_f = 40.0
    sp_r = 20.0
    len = 1.0
    wid = 1.0

    rectangle(robot, sp_f, sp_r, len, wid)  



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