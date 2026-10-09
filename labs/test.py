from control.MyRobot import MyRobot
import time

TIME = 0.7

def main():
    robot = MyRobot()
    print("testing")

    """
    #so far: kp = 35, ki = 4.5, kd = 6 for forward 
    kp = float(input("Enter kp: "))
    ki = float(input("Enter ki: "))
    kd = float(input("Enter kd: "))

    dis = float(input("Enter distance: "))
    
    robot.lidar_pid(dis, kp, ki, kd)
    print(robot.get_front_distance())
    print("done")

    """
    

    """
    # so far: kp = 6, ki = 0.2, kd = 0.4 for forward
    kp = float(input("Enter kp: "))
    ki = float(input("Enter ki: "))
    kd = float(input("Enter kd: "))

    dis = float(input("Enter distance: "))
    direct = 1 

    robot.encoder_pid(dis, kp, ki, kd)
    print("done")
    """

    # so far: kp = 0.7, ki = 0.02, kd = 0.05 for rotation
    kp = float(input("Enter kp: "))
    ki = float(input("Enter ki: "))
    kd = float(input("Enter kd: "))
    angle = float(input("Enter angle: "))

    #robot.move_forward(45, 0.3)

    robot.compass_pid(angle, kp, ki, kd)

    #robot.move_forward(45, 0.3)
    print("done")

    
if __name__ == "__main__":
    main()