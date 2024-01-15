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

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    #waypoints = [Pose(Point(1.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)), 
                 #Pose(Point(-6.0, 1.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 #Pose(Point(-0.5, -7.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 #Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))] # arguments are position and orientation --> quaternion shows orientation
    
    waypoints = [Pose(Point(2.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)), 
                 Pose(Point(5.0, 4.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(1.0, 8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-4.0, -6.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-3.0, 6.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(9.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(12.0, 11.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(9.0, -8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(4.0, -8.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
                 Pose(Point(-3.0, -1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))]

    currentWaypoint = None
    moving = False

    rumexFound = []

    

    def detect(msg):
        if(msg.data == "success" and not moving):
            if not rumexFound.__contains__(currentWaypoint.position):
                rospy.loginfo("Found rumex!")
                rumexFound.append(currentWaypoint.position)
        """ else:
            rospy.loginfo("No rumex found!") """

    rospy.Subscriber("/test_detect", String, detect)

    for waypoint in waypoints:
        moving = True
        moving = True # no scanning for plants
        currentWaypoint = waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint
        client.send_goal(goal)
        client.wait_for_result()

        desired_yaw_degrees = 90 
        for i in range(4): #loop for 360 degree turn in 4 steps
            moving = False # only scans for plants when moving false
            turn_quat = [goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
            current_roll, current_pitch, current_yaw = tft.euler_from_quaternion(turn_quat)
            desired_yaw_radians = math.radians(desired_yaw_degrees) # calculate radians for turn
            new_yaw = current_yaw + desired_yaw_radians # calsculate the final yaw out of original yaw and the just calculated radians
            new_orientation = Quaternion(*tft.quaternion_from_euler(current_roll,current_pitch,new_yaw))
            pose =Pose(waypoint.position,new_orientation) #new Pose with new Quaternion
            goal = MoveBaseGoal()
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            client.wait_for_result()
            #desired_yaw_degrees = desired_yaw_degrees + 90
        if not rumexFound.__contains__(currentWaypoint.position):
            for i in range(3):
                if not rumexFound.__contains__(currentWaypoint.position):
                    #random point calculating with slight difference to original waypoint
                    new_wp = (Pose(Point(currentWaypoint.position.x + random.uniform(-0.5,0.5), currentWaypoint.position.y + random.uniform(-0.5,0.5), currentWaypoint.position.z), Quaternion(0.0, 0.0, 0.0, 1.0)))
                    r_yaw_degrees = random.uniform(0,360) #random degree
                    print( "The random degreee is " + str(r_yaw_degrees)+ " and the new Waypoin is " + str(new_wp))
                    moving = False # still scanning
                    # calculating again the new Quaternion for turning
                    turn_quat = [goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    current_roll, current_pitch, current_yaw = tft.euler_from_quaternion(turn_quat)
                    r_yaw_radians = math.radians(r_yaw_degrees)
                    new_r_yaw = current_yaw + r_yaw_radians
                    new_orientation = Quaternion(*tft.quaternion_from_euler(current_roll,current_pitch,new_r_yaw))
                    pose =Pose(new_wp.position,new_orientation)
                    goal = MoveBaseGoal()
                    goal.target_pose.pose = pose
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    client.send_goal(goal)
                    client.wait_for_result()





        moving = False
                   # break


    rospy.loginfo("Rumex' found at" + str(rumexFound))
        
    
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