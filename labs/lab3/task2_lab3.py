from control.MyRobot import MyRobot

def main():
    robot = MyRobot()

    pid_mtg_params = (40, 0, 5, 0.02, 0, 0) 
    pid_wf_params = (45, 0, 5, 90, 0, 5)

    robot.bug_zero(0.25, "left", pid_mtg_params, pid_wf_params)

if __name__ == "__main__":
    main()