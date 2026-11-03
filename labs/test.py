from control.MyRobot import MyRobot
import time

TIME = 0.7

def main():
    robot = MyRobot()
    print("testing")

    """
    kp_fw = float(input("Enter forward speed gain (kp_fw): ")) # 25
    ki_fw = float(input("Enter forward speed integral gain (ki_fw): ")) #0
    kd_fw = float(input("Enter forward speed derivative gain (kd_fw): ")) # 0

    kp_st = float(input("Enter steering gain (kp_st): ")) # 0.05
    #ki_st = float(input("Enter steering integral gain (ki_st): "))
    #kd_st = float(input("Enter steering derivative gain (kd_st): "))

    robot.move_to_goal(kp_fw, ki_fw, kd_fw, kp_st, 0, 0)
    """
    #kp_fw = float(input("Enter forward speed gain (kp_fw): ")) # 25
    #kp_st = float(input("Enter steering gain (kp_st): ")) # 0.05
    #kd_st = float(input("Enter steering derivative gain (kd_st): ")) # 0.05
    #dist_from_wall = float(input("Enter distance from wall (in meters): ")) # 0.3
    robot.follow_wall(0.2, "left", 40, 0, 0, 70, 0, 5)


if __name__ == "__main__":
    main()