## If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
## try uncommenting the following snippet
#Ausaf Ahmed and Kelly Qiu
# try:
#     import matplotlib
#     matplotlib.use('TkAgg')
# except ImportError:
#     pass

from skimage import color
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, distance_inches
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image

from markers import annotator

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

import math
import cv2

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters_create()



def xyh_from_pose(R, t):
    ''' 
    Use the rotation matrix and translation vector to compute the location and heading relative from the camera 
    Not meant to be called directly
    '''
    x = t[2] # in the grid, x is forward-backward
    y = -t[0] # in the grid, y is left-right

    # heading = np.rad2deg(math.atan2(R[1,0], R[0,0]))
    # heading = np.rad2deg(math.atan2(y, x))

    R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
    R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
    R_2p_1p = np.matmul(np.matmul(np.linalg.inv(R_2_2p), np.linalg.inv(R)), R_1_1p)
    yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0]) + math.pi
    heading = np.rad2deg(yaw)

    return x, y, heading

def estimate_pose(marker_size, image_coords, camera_intrinsics, camera_distortion=np.zeros((4,1)), cv2_flag=None):
    """
    marker_size: a (width, height) tuple containing the dimensions of your marker in mm
        NOTE: for our markers, measuring from the black corners, this would be (96.5, 96.5)
    image_coords: a list of 4 (x, y) tuples containing the coordinates of your marker's corners in the image
        in the following order: top-left, top-right, bottom-right, bottom-left (clockwise from top left)
    camera_intrinsics: your camera intrinsic matrix (3x3 - see `camera_settings` in `run`)

    Returns: an (x, y, h) position tuple
    """
    image_coords = np.array(image_coords, dtype=np.float32)
    half_width, half_height = marker_size[0]/2, marker_size[1]/2
    object_coords = np.array([ # market coordinates in frame of marker center
        [-half_width, -half_height, 0], # top-left
        [ half_width, -half_height, 0], # top-right
        [ half_width,  half_height, 0], # bottom-right
        [-half_width,  half_height, 0], # bottom-left
    ], dtype=np.float)
    success, r, t = cv2.solvePnP(object_coords, image_coords, camera_intrinsics, camera_distortion, cv2_flag or cv2.SOLVEPNP_P3P)
    
    if not success:
        return None

    # Convert the rotation vector into a matrix
    R, _ = cv2.Rodrigues(r)
    t = np.squeeze(t)

    return xyh_from_pose(R, t)

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)
marker_annotator = None



def compute_odometry(curr_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees
    
    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


async def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera. 
    Since this is an async function, it must be called using await, for example:

        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
            (NOTE: you don't have to implement this, but it might help you debug!!)
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh) 
          (as expected by the particle filter's measurement update)
        - (optionally) a PIL Image of what Cozmo's camera sees with marker annotations
    '''

    global grid
    global marker_annotator
    # Wait for the latest image from Cozmo
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    image = np.array(image_event.image)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    corner_coords = []
    # need to adjust corner orderings
        # need to adjust corner orderings
    for c in corners: 
        c = c[0].tolist()
        c = sorted(c, key=lambda p: p[1]) 
        c[:2] = sorted(c[:2], key=lambda p: p[0])        
        c[2:] = sorted(c[2:], key=lambda p: -p[0]) 
        c.insert(0, c[-1])
        c.pop()
        # c.reverse()
        corner_coords.append(c)

    markers = [estimate_pose((45,45), c, camera_settings) for c in corner_coords]
    if show_diagnostic_image:
        cv2.aruco.drawDetectedMarkers(image, corners, ids) 
    #     marker_annotator.markers = [{'corner_coords': c, 'xyh': m} for m,c in zip(markers, corner_coords)]
    #     marker_annotator.apply(image, 1)
    markers = [(x/grid.scale,y/grid.scale,h-360) for x,y,h in markers]

    PIL_image = Image.fromarray(np.uint8(image)).convert('RGB')
    return markers, PIL_image

async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui, pf
    global marker_annotator


    
    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    
    marker_annotator = annotator.MarkerAnnotator(robot.world.image_annotator)
    robot.world.image_annotator.add_annotator('Marker', marker_annotator)
    


    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)
    
    m_confident = False
    while True: 
        if not m_confident: 
            if robot.is_picked_up:
                await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                pf = ParticleFilter(grid)
                time.sleep(4)
                continue
            else:
                await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
                markers, img = await marker_processing(robot, camera_settings, show_diagnostic_image=True)
                
                curr_pose = robot.pose
                odom = compute_odometry(curr_pose)
                last_pose = curr_pose
                m_x, m_y, m_h, m_confident = pf.update(odom, markers)
                # await robot.drive_wheels(20, 30, duration = 1)
                # theta = theta + 30
                markers.sort(key=lambda x: x[0])
                if (len(markers) > 0) and markers[0][0] > 3:
                    await robot.drive_straight(distance_mm(40), speed_mmps(40)).wait_for_completed()
                else: 
                    await robot.turn_in_place(degrees(20)).wait_for_completed()
                
                gui.show_particles(pf.particles)
                gui.show_mean(m_x, m_y, m_h, m_confident)
                gui.show_camera_image(img)
                gui.updated.set()

        else: 
            # go to goal
            print('going to goal now')
            completed = False
            pickedup = False 
            while completed == False and pickedup == False:
                curr_pose = robot.pose
                odom = compute_odometry(curr_pose)
                last_pose = curr_pose
                m_x, m_y, m_h, m_confident = pf.update(odom, markers)
                
                dx = goal[0] - m_x
                dy = goal[1] - m_y

                angle = math.degrees(math.atan2(dy, dx))
                turn = diff_heading_deg(angle, m_h)

                dr = (dx**2 + dy**2)**(0.5)

                print(f'diff: dr={dr}, dx={dx}, dy={dy}')
                if robot.is_picked_up:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                    pf = ParticleFilter(grid)
                    time.sleep(4)
                    m_confident= False
                    pickedup = True 
                    continue
                if turn > 5: 
                    await robot.turn_in_place(cozmo.util.degrees(turn)).wait_for_completed()
                if dr > 1: 
                    print('still need to move,')
                    if robot.is_picked_up:
                        await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                        pf = ParticleFilter(grid)
                        time.sleep(4)
                        m_confident= False
                        pickedup = True 
                        continue
                    await robot.drive_straight(distance_mm(40), speed_mmps(40)).wait_for_completed()
                else:
                    if robot.is_picked_up:
                        await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                        pf = ParticleFilter(grid)
                        time.sleep(4)
                        m_confident= False
                        pickedup = True 
                        continue
                    await robot.turn_in_place(cozmo.util.degrees(-turn)).wait_for_completed()
                    if robot.is_picked_up:
                        await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                        pf = ParticleFilter(grid)
                        time.sleep(4)
                        m_confident= False
                        pickedup = True 
                        continue
                    print('Finished!')
                    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabCat).wait_for_completed()
                    completed = True
                    if robot.is_picked_up:
                        await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoLookFaceGetOut, in_parallel=True).wait_for_completed()
                        pf = ParticleFilter(grid)
                        time.sleep(4)
                        m_confident= False
                        completed = False
                        continue
            

            if not pickedup: 
                break


                    
                



        

class CozmoThread(threading.Thread):
    
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()

