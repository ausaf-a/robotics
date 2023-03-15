import cozmo
import threading
import json
import math
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import pickle 

# Method to move the robot and apply disturbance
# Input
# v_l: (float) left wheel speed
# v_r: (float) right wheel speed
def move(robot: cozmo.robot.Robot, v_l, v_r):
    global start_time, disturbance
    # start_dist is the time when the disturbance is triggered
    start_dist = 0.3
    if start_dist < time.time()-start_time < start_dist + 0.3:
        v_l += disturbance
        v_r += disturbance
    robot.drive_wheel_motors(v_l, v_r)

def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent, start_time, disturbance
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]

    # Reads distrubance from command
    # Default is zero
    disturbance = 0
    if len(sys.argv) > 1:
        disturbance = int(sys.argv[1])
        
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    ###############################

    robot.set_head_angle(cozmo.util.Angle(degrees=0)).wait_for_completed()
    cube = robot.world.wait_for_observed_light_cube()
    
    # set start_time after Cozmo detects a cube
    target = None 
    while True: 
        target = cube.pose.position.x - 50 - 30 - robot.pose.position.x # 5 cm 
        print(f'initial displacement: {target} mm')
        print(f'cube position: {cube.pose.position}')
        print(f'cozmo position: {robot.pose.position}')
        if input('enter anything to start:'): 
            break 

    # while True: 
    #     print('cube is ', cube.pose.position, 'mm away')
    #     time.sleep(0.05)


    start_position = robot.pose.position.x
    start_time = time.time()
    ei = 0
    ed = 0
    last_error = target

    time_remaining = 5 
    end_time = start_time + 5
    e = target 

    x = []
    y = []


    while time.time() < start_time + 7: 
        e = target - (robot.pose.position.x - start_position)
        x.append(time.time() - start_time)
        y.append(robot.pose.position.x - start_position)
        ei += e 
        ed = e - last_error
        
        # robot.

        pid = kp*e + ki*ei + kd*ed
        
        print(f'error: {e} mm')
        print(f'last: {last_error} mm')
        print(f'deriv: {ed} mm/s')
        print(f'int: {ei} mm')
        print(f'pid: {pid} mm/s')
        move(robot, pid, pid)
        # error = target - 
        # if pid < 1: 
        #     break
        last_error = e
        time.sleep(0.05)
    

    with open('out.pickle', 'wb') as f: 
        pickle.dump({'x': x, 'y': y, 't': target}, f)

    # plt.plot(x, y)
    # plt.savefig('foo.png')
    

    # print(f'time elapsed: {time.time() - start_time}')
    
class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoPID)
        stopevent.set()

if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
