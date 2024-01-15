#!/usr/bin/env python

import math
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
import numpy as np
import tf.transformations as tft 
import datetime 
from std_msgs.msg import String
import random
from threading import Thread
from ant_colony import AntColony


def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Rotations:
    # 45: 0.7853981634 (0.25 * PI)
    # 90: 1.5707963268 (0.5 * PI)
    # 135: 2.3561944902 (0.75 * PI)
    # 180: 3.1415926536 (PI)
    # 225: 3.926990817 (1,25 * PI)
    # 270: 4.7123889804 (1.5 * PI)
    # 315: 5.4977871438 (1.75 * PI)
    # 360: 6.2831853072 (2 * PI)

    waypoints = [Pose(Point(2.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)), 
                 Pose(Point(5.0, 4.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(1.0, 8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-4.0, -6.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-3.0, 6.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(9.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(12.0, 11.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(9.0, -8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(4.0, -8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-3.0, -1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))] # arguments are position and orientation --> quaternion shows orientation
    
    path = np.array([(2, 0),
                    (5, 4),
                    (1, 8),
                    (-4, -6),
                    (-3, 6),
                    (9, 0),
                    (12, 11),
                    (9, -8),
                    (4, -8),
                    (-3, -1)])
    
    #ant_colony = AntColony(path, n_ants=20, n_best=3, n_iterations=150, decay=0.95)
    #path = ant_colony.run(False)

    def waypoints_to_ros(waypoint):
        return Pose(Point(waypoint[0], waypoint[1], 0), Quaternion(0.0, 0.0, 0.0, 1.0))
    
    waypoints = map(waypoints_to_ros, path)

    currentWaypoint = None
    perform_scan = False
    current_pose = None

    rumexFound = []

    def mergesort_wp(waypoints):
        if len(waypoints) <=1:
            return waypoints
        
        #splitting list in half
        middle= len(waypoints)/2
        left_side = waypoints[:middle]
        right_side = waypoints[middle:]
        #sort recursively
        left_side = mergesort_wp(left_side)
        right_side = mergesort_wp(right_side)

        return merge(left_side,right_side)
    
    def merge(left_side, right_side):
        merge_wp = []
        i=j=0
        while i < len(left_side) and j < len(right_side):
            if left_side[i].position.x < right_side[j].position.x:
                merge_wp.append(left_side[i])
                i+=1
            elif left_side[i].position.x > right_side[j].position.x:
                merge_wp.append(right_side[j])
                j+=1
            else:
                if left_side[i].position.y <= right_side[j].position.y:
                    merge_wp.append(left_side[i])
                    i+=1
                else: 
                    merge_wp.append(right_side[j])
                    j+=1
        merge_wp += left_side[i:]
        merge_wp += right_side[j:]
        return merge_wp
                

    def magnitude(vect): #part of 2-opt
        return math.sqrt(math.pow(vect[0], 2) + math.pow(vect[1], 2))
    
    def subtract(pose1, pose0): #part of 2-opt
        return [pose1.position.x - pose0.position.x, pose1.position.y - pose0.position.y ]
    
    def distance(waypoints): #part of 2-opt
        return sum(magnitude(subtract(waypoints[i], waypoints[i-1])) for i in range(1, len(waypoints)))
    
    def sort_waypoints_two_opt(waypoints): #main function for 2-opt
        improved = True
        while improved:
            improved = False
            for i in range(1, len(waypoints) - 1): # check - 1
                for j in range(i + 1, len(waypoints)):
                    if j - 1 == 1: continue
                    new_waypoints = waypoints[:]
                    new_waypoints[i:j] = waypoints[j-1:i-1:-1]
                    if distance(new_waypoints) < distance(waypoints):
                        waypoints = new_waypoints
                        improved = True
        return waypoints    
    
    def index_to_array(index, array):
        return array[index]

    def nearest_neighbor(waypoints):
        n = len(waypoints)
        coords = np.zeros((n,2)) # n arrays of [x, y] in an array
        for i, waypoint in enumerate(waypoints):
            coords[i,0] = waypoint.position.x
            coords[i,1] = waypoint.position.y

        remaining_wp = set(range(1,n))
        nn_wp = [0]
        while remaining_wp:
            last_wp = nn_wp[-1]
            indices = list(remaining_wp)
            distances = np.linalg.norm(coords[indices] - coords[last_wp], axis=1) 
            nearest_wp= np.argmin(distances)
            nearest_indice = indices[nearest_wp]
            remaining_wp.discard(nearest_indice)
            nn_wp.append(nearest_indice)

        nn_wp.append(0)
        return list(map(lambda x: index_to_array(x, waypoints), nn_wp))

    
    def pose_to_position(pose):
        return "{ x: " + str(pose.position.x) + ", y: " + str(pose.position.y) + ", z: " + str(pose.position.z) + " }"

    def get_waypoints_output(waypoints):
        return str(map(pose_to_position, waypoints))

    # print("Waypoints: " + str(waypoints))
    print("Sorted 2-opt: " + get_waypoints_output(sort_waypoints_two_opt(waypoints)))
    print("Sorted merge:" + get_waypoints_output(mergesort_wp(waypoints)))  
    print("Sorted nearest neighbor:" + get_waypoints_output(nearest_neighbor(waypoints))) 

    def odom_callback(data):
        #rospy.loginfo("Robot Twist:", data.pose.pose.position)
        if  currentWaypoint != None and current_pose != None:
            distance_threshold = 1.0
            perform_scan = magnitude(subtract(currentWaypoint, current_pose)) < distance_threshold

    def odom_subscriber():
        rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, odom_callback)

    odom_subscriber()


    #def searching_poses(pose):
        #currentWaypoint = waypoint
        #search_poses = [Pose(Point(currentWaypoint.position.x + 0.5, currentWaypoint.position.y, currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0)),
                       # Pose(Point(currentWaypoint.position.x , currentWaypoint.position.y + 0.5, currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0)),
                       # Pose(Point(currentWaypoint.position.x - 0.5, currentWaypoint.position.y, currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0)),
                       # Pose(Point(currentWaypoint.position.x , currentWaypoint.position.y - 0.5, currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0))]

        #for i in range(len(search_poses)):
            #goal = MoveBaseGoal()
            #goal.target_pose.header.frame_id = "map"
            #goal.target_pose.header.stamp = rospy.Time.now()
            #goal.target_pose.pose = search_poses[i]
            #print("Pose at the moment: " + str(search_poses[i]))
           # client.send_goal(goal)
            #client.wait_for_result()


    def detect(msg):
        if(msg.data == "success" and not perform_scan):
            if not rumexFound.__contains__(currentWaypoint):
                rospy.loginfo("Found rumex!")
                rumexFound.append(currentWaypoint)
        """ else:
            rospy.loginfo("No rumex found!") """

    rospy.Subscriber("/test_detect", String, detect)

    sorted_version = sort_waypoints_two_opt(waypoints)
    
    for waypoint in sorted_version:
        perform_scan = False # no scanning for plants
        currentWaypoint = waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint
        client.send_goal(goal)
        client.wait_for_result()
        print('arrived at '+ str(waypoint))

        #searching_poses(waypoint)


        #desired_yaw_degrees = 90 
        #for i in range(4): #loop for 360 degree turn in 4 steps
            #moving = False # only scans for plants when moving false
            #turn_quat = [goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
            #current_roll, current_pitch, current_yaw = tft.euler_from_quaternion(turn_quat)
            #desired_yaw_radians = math.radians(desired_yaw_degrees) # calculate radians for turn
            #new_yaw = current_yaw + desired_yaw_radians # calsculate the final yaw out of original yaw and the just calculated radians
            #new_orientation = Quaternion(*tft.quaternion_from_euler(current_roll,current_pitch,new_yaw))
            #pose =Pose(waypoint.position,new_orientation) #new Pose with new Quaternion
            #goal = MoveBaseGoal()
            #goal.target_pose.pose = pose
            #goal.target_pose.header.frame_id = "map"
            #goal.target_pose.header.stamp = rospy.Time.now()
            #client.send_goal(goal)
            #client.wait_for_result()
            
            #desired_yaw_degrees = desired_yaw_degrees + 90
        #if not rumexFound.__contains__(currentWaypoint.position):
            #for i in range(3):
               # if not rumexFound.__contains__(currentWaypoint.position):
                    #random point calculating with slight difference to original waypoint
                    #new_wp = (Pose(Point(currentWaypoint.position.x + random.uniform(-0.5,0.5), currentWaypoint.position.y + random.uniform(-0.5,0.5), currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0)))
                    #r_yaw_degrees = random.uniform(0,360) #random degree
                    #print( "The random degreee is " + str(r_yaw_degrees)+ " and the new Waypoin is " + str(new_wp))
                    #moving = False # still scanning
                    #calculating again the new Quaternion for turning
                    #turn_quat = [goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    #current_roll, current_pitch, current_yaw = tft.euler_from_quaternion(turn_quat)
                    #r_yaw_radians = math.radians(r_yaw_degrees)
                    #new_r_yaw = current_yaw + r_yaw_radians
                    #new_orientation = Quaternion(*tft.quaternion_from_euler(current_roll,current_pitch,new_r_yaw))
                    #pose =Pose(new_wp.position,new_orientation)
                    #goal = MoveBaseGoal()
                    #goal.target_pose.pose = pose
                    #goal.target_pose.header.frame_id = "map"
                    #goal.target_pose.header.stamp = rospy.Time.now()
                    #client.send_goal(goal)
                    #client.wait_for_result()

                    


    
    
    rumexTupel = []
    rumexFinal = []

    for i in range(len(rumexFound)):
        rumexGroup = []
        for j in range(i+1, len(rumexFound)):
            if not rumexTupel.__contains__((i,j)) and not rumexTupel.__contains__((j,i)) and magnitude(subtract(rumexFound[i], rumexFound[j])) < 0.5:
                rumexTupel.append((i,j))
                rumexGroup.append(rumexFound[j])

        x = rumexFound[i].position.x
        y = rumexFound[i].position.y
        z = rumexFound[i].position.z
        for pose in rumexGroup:
            x += pose.position.x
            y += pose.position.y
            z += pose.position.z

        if len(rumexGroup) != 0:
            x /= len(rumexGroup)
            y /= len(rumexGroup)
            z /= len(rumexGroup)

        rumexFinal.append(Point(x, y, z))

    rospy.loginfo("Rumex's found at" + str(rumexFound))
    rospy.loginfo("Rumex's grouped" + str(rumexFinal))
                


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.logwarn("Goal execution failed!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
