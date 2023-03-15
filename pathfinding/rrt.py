from threading import local
import cozmo
import math
import sys
import time
import random
from cozmo.util import degrees, distance_mm, speed_mmps, distance_inches

from cmap import *
from gui import *
from utils import *

#Kelly Qiu and Ausaf Ahmed
MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    
    
    #temporary cod below to be replaced
    if get_dist(node0, node1) < limit:
        return node1
    y = node1.y - node0.y
    x = node1.x - node0.x
    angle = np.arctan2(y, x)
    newNode = Node((node0.x + np.cos(angle)*limit, node0.y + np.sin(angle)*limit))
    return newNode
    ############################################################################

    
    
    


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    
    width, height = cmap.get_size()
    nodeFound = False
    while not nodeFound:
        prob = random.randint(1,100)
        if prob < 6 and len(cmap.get_goals()) > 0:
            goals = cmap.get_goals()
            goal = goals[random.randint(0, len(goals) - 1)]
            return Node((goal.x, goal.y))
        else:
            rand_node = Node((random.randint(0, width - 1), random.randint(0, height - 1)))
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            nodeFound = True
    return rand_node
    ############################################################################
    


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        inf_rand_node = cmap.get_random_valid_node()
        nodes = cmap.get_nodes()
        nearest_node = nodes[0]
        nearest_node_dist = get_dist(inf_rand_node, nearest_node)
        for i in range(1, len(nodes)):
            node_dist = get_dist(inf_rand_node, nodes[i])
            if node_dist < nearest_node_dist:
                nearest_node = nodes[i]
                nearest_node_dist = node_dist
        rand_node = step_from_to(nearest_node, inf_rand_node)
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break
        
    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()
    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")



async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions. Potential pseudcode is below
    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    #Add final position as goal point to cmap, with final position being defined as a point that is at the center of the arena 
    #you can get map width and map weight from cmap.get_size()

    
    #reset the current stored paths in cmap
    #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
    #get path from the cmap
    # print('start')
    # start_node = Node((50, 35))
    # cmap.set_start(start_node)
    width, height = cmap.get_size()
    final_node = Node((width/2, height/2))
    cmap.add_goal(final_node)
    cmap.reset_paths()
    
    RRT(cmap, cmap.get_start())
    path = cmap.get_smooth_path()
    
    print(path)
    #marked and update_cmap are both outputted from detect_cube_and_update_cmap(robot, marked, cozmo_pos).
    #and marked is an input to the function, indicating which cubes are already marked
    #So initialize "marked" to be an empty dictionary and "update_cmap" = False
    marked = {}
    update_cmap = False
    idx = 0
    curr_node = cmap.get_start()
    # print('START=',curr_node.x, curr_node.y)
    cozmo_angle = 0
    cozmo_pos = Node((curr_node.x, curr_node.y))
    while True:
        idx = idx + 1

        if not path or idx >= len(path):
            break

        next_node = path[idx]

        # calculate relative offset to next node in path 
        dy = next_node.y - curr_node.y
        dx = next_node.x - curr_node.x
        
        # angle offset to next node from cozmo angle 
        angle = math.degrees(np.arctan2(dy, dx)) - cozmo_angle
        print(f'(dx={dx}, dy={dy}, angle={angle}, cozmo_angle={cozmo_angle})')        
        cozmo_angle += angle

        # turn to face the next node
        await robot.turn_in_place(cozmo.util.degrees(angle)).wait_for_completed()
        
        dist = get_dist(curr_node, next_node)
        
        step_size = 25
        update_cmap = False
        rpos = Node((robot.pose.position.x, robot.pose.position.y))
        while dist > step_size and not update_cmap:
            dist -= step_size
            cozmo_pos= Node((
                step_size*np.cos(cozmo_angle)+cozmo_pos.x,
                step_size*np.sin(cozmo_angle)+cozmo_pos.y
                ))

            await robot.drive_straight(distance_mm(step_size), speed_mmps(120)).wait_for_completed()
            update_cmap, goal_center, marked = await detect_cube_and_update_cmap(robot, marked, rpos)    
        
        if dist > 0 and not update_cmap: 
            cozmo_pos= Node((
                cozmo_pos.x + dist*np.cos(cozmo_angle),
                cozmo_pos.y + dist*np.sin(cozmo_angle)
            ))
            await robot.drive_straight(distance_mm(dist), speed_mmps(120)).wait_for_completed()
            update_cmap, goal_center, marked = await detect_cube_and_update_cmap(robot, marked, rpos)    
            if not update_cmap:     
                curr_node = next_node
        
        
        if update_cmap:
            print(f'detected new obstacle, planning path...')
            await robot.say_text("Obstacle detected", duration_scalar=0.5).wait_for_completed()
            cmap.reset_paths()

            # start the new RRT search from cozmo's current location 
            # rather than the current node
            print('coz: ', cozmo_pos.x, cozmo_pos.y)
            print('robot: ', robot.pose.position.x, robot.pose.position.y)
            cmap.set_start(Node((robot.pose.position.x, robot.pose.position.y)))
            RRT(cmap, cmap.get_start())
            path = cmap.get_smooth_path()
            curr_node = cmap.get_start()
            idx = 0

    print('finished executing path')     
    await robot.turn_in_place(cozmo.util.degrees(450), cozmo.util.degrees(120)).wait_for_completed()
    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabWin).wait_for_completed()
            
            
        

    #while the current cosmo position is not at the goal:
    
        #break if path is none or empty, indicating no path was found

        
        # Get the next node from the path
        #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
        #you can calculate the angle to turn through a trigonometric function

            
        # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle 

    
        # Set new start position for replanning with RRT

        #detect any visible obstacle cubes and update cmap
        
        #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths 

    
    ########################################################################
    
    
    
    
def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object
        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    
    globalTransform = np.array([
        [np.cos(local_angle), -np.sin(local_angle), local_origin.x], 
        [np.sin(local_angle), np.cos(local_angle), local_origin.y],
        [0, 0, 1]
        ])
    coordsLocal = np.array([
        [node.x], 
        [node.y], 
        [1]])
    new_node = globalTransform@coordsLocal
    
    new_node = new_node.tolist()
    # print(f'NEW_NODe={new_node[0][0]},{new_node[1]}')
    new_node = Node((new_node[0][0], new_node[1][0]))
    # print(f'NEW_NODE={new_node.x},{new_node.y}')
    return new_node
    ########################################################################


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset_paths()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        #creates cmap based on empty grid json
        #"start": [50, 35],
        #"goals": [] This is empty
        cmap = CozMap("maps/emptygrid.json", node_generator) 
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
