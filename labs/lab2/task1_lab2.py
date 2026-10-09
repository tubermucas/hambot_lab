from pickletools import dis
from control.MyRobot import MyRobot
import time

def main():
    robot = MyRobot()

    
    
    kp_lidar = 40
    ki_lidar = 4
    kd_lidar = 6

    kp_encoder = 6
    ki_encoder = 0.2
    kd_encoder = 0.4

    kp_compass = 0.7
    ki_compass = 0.02
    kd_compass = 0.05
    
    
    if robot.get_front_distance(timeout=3.0) is None:
        print("Lidar is not working")
        return
    

    print("Step 1: lidar PID control")

    dis = 0.6

    robot.lidar_pid(dis, kp_lidar, ki_lidar, kd_lidar)
    print(robot.get_front_distance())

    print("-" * 20 + "\n")
    time.sleep(1)

    
    
    print("Step 2: Encoder PID control")
    dis = 0.3

    robot.encoder_pid(dis, kp_encoder, ki_encoder, kd_encoder)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 3: Lidar Backward PID control")
    dis = 0.6

    robot.lidar_pid(dis, kp_lidar, ki_lidar, kd_lidar)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 4: Compass PID control")
    
    angle = -170

    robot.compass_pid(angle, kp_compass, ki_compass, kd_compass)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 5: Lidar PID control")
    dis = 0.6

    robot.lidar_pid(dis, kp_lidar, ki_lidar, kd_lidar)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 6: Encoder PID control")
    dis = 0.3

    robot.encoder_pid(dis, kp_encoder, ki_encoder, kd_encoder)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 7: Lidar Backward PID control")
    dis = 0.6

    robot.lidar_pid(dis, kp_lidar, ki_lidar, kd_lidar)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 8: Compass PID control")
    angle = 170

    robot.compass_pid(angle, kp_compass, ki_compass, kd_compass)

    print("-" * 20 + "\n")
    time.sleep(1)

    print("Step 8: Encoder PID control")
    dis = 0.6

    robot.encoder_pid(dis, kp_encoder, ki_encoder, kd_encoder)

    print("-" * 20 + "\n")

    

if __name__ == "__main__":
    main()