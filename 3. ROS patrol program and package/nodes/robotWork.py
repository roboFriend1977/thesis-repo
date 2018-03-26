

#!/usr/bin/env python

""" robotPatrol.py - Version 1.1 2017-1-15

	Developed by Sami Salama Hussen Hajjaj 	

	Motion algorithm for PatroBot; Agriculture Patrol Robot
	
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from math import radians, pi, pow, sqrt
from random import sample
import math
import time

class RobotPatrol():
    def __init__(self):	
		
		# rospy housekeeping
        rospy.init_node('robotRoam_ROS', anonymous=False)
        rospy.on_shutdown(self.shutdown)

		# Travel Options:
        self.resume = rospy.get_param("~res", False) 	# T: resume, F: from beginning 
        self.random = rospy.get_param("~ran", False)	# T: random, F: in given sequence
        self.intermit = rospy.get_param("~int", False)	# T: intermittent, F: continuous
                
        # init program   
        self.init_odom_tf()			# Initialize odom tf for localization
        self.localize()				# localizes robot in map based on GPS data
        self.init_path_data()		# Read raw travel coordinates from robot files
        self.init_waypoints()		# Process raw data to develop waypoints (Poses)
        self.init_visualization()	# visualize target Poses 
        self.init_robot_motion()	# call and prep move_base package 
        
        # if robot stopped mid-way, but resume was not called, start from the beginning
        if not self.resume: self.reset_completionStatus() 
               
        # Robot movement (Patrol) (Work or resume)
        i = 0
        while not rospy.is_shutdown():	# for each Pose in given path .. 
			
            # if path already completed, start over (for continuous patrol)
            if i == self.numbPoints: 
				i = 0		# reset counter  
				self.reset_completionStatus()	# reset poseResults
				if self.random: # if random path
					self.markers_pub.publish(self.eraser)	# erase
					self.init_waypoints()		# reset waypoints 
					self.init_visualization()	# reset visualization
					
			# visualize all require Poses
            self.markers_pub.publish(self.squares)					
            for arrow in self.arrows: self.markers_pub.publish(arrow)	
										
			# if Point not already crossed? skip
            if self.completedPoints[i] != 1:
			
				goal = MoveBaseGoal()	# Int and setup goal
				goal.target_pose.header.frame_id = 'map'		
				goal.target_pose.header.stamp = rospy.Time.now()
				goal.target_pose.pose = self.waypoints[i]       
				self.move_base.send_goal(goal)	# move robot to goal
            
				# Testing (optional) 
				in_time = self.move_base.wait_for_result(rospy.Duration(120))	
				# longer distances might require longer time
				if not in_time:		# If not reached in time
					self.move_base.cancel_goal()	# abort
					rospy.loginfo("Goal[%i]: Could not be reached !", i)
				else:											
					state = self.move_base.get_state()
					if state == GoalStatus.SUCCEEDED: 			
						rospy.loginfo("Goal[%i]: reached!", (i+1))
						self.localize()				
						# refresh robot Pose 
						self.completedPoints[i] = 1	
						# Update completion Status 
						self.updateCompletion()		
						# inform android 
						
            if self.intermit: time.sleep(3)	# if intermittent, pause for 3 seconds. 
            i += 1	# go to next point

    def init_odom_tf(self):

# -----------------------------------------------------------------------------------------
		
		# Set the odom frame
		self.odom_frame = '/odom'
		
		# Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")

    def localize(self):
		try:
			(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return
		return (Pose(Point(*trans), Quaternion(*rot)))	# returns a Pose

    def init_path_data(self):
		# Init raw Data lists 
        self.rawpoints = list()			# raw x, y coordinates of target patrol points
        self.completedPoints = list()   # completion status of each point
        
        # read raw data from files and to lists - rawPoints
        rawpoints_file = open('catkin_ws/src/ros_android_hri/initial/deliveryPoints.txt','r')	
        for line in rawpoints_file: self.rawpoints.append(float(line))
        rawpoints_file.close()

        # read raw data from files and to lists - status
        completionStatus_file = open('catkin_ws/src/ros_android_hri/initial/completionStatus.txt','r')	
        for line in completionStatus_file: self.completedPoints.append(int(line))        
        completionStatus_file.close()

        # init completionStatus publisher
        self.completionStatus_pub = rospy.Publisher('robotStatus', Header, queue_size=5)

    def reset_completionStatus(self):
		for i in range(len(self.completedPoints)): self.completedPoints[i] = 0        
        
    def init_waypoints(self):

        # Lists initiations 
        positions = list()
        angles = list()
        quaternions = list()
        self.waypoints = list()
        
        self.numbPoints = len(self.rawpoints)/2		# number of waypoints, used throughout
        #rospy.loginfo("Number of points is: %i",self.numbPoints)

        # Positions: Construct a 3 x n array (each point (x,y,z) coordinates
        # start from beginning of rawpoints array, increment by 2, and stop at second last
        for i in range(0,len(self.rawpoints)-1,2): positions.append(Point(self.rawpoints[i],self.rawpoints[i+1], 0.0))
        
        

		# if random required => randomize positions  
        if self.random: positions = sample(positions, self.numbPoints)
		
		# Orientations: Construct n array of eulor angles, between each 2 points
        for i in range(len(positions)):
			if i < (self.numbPoints -1): # direction is from point i to point i+1 
				deltaX = positions[i+1].x - positions[i].x
				deltaY = positions[i+1].y - positions[i].y
			else: # last point, direction from it to first point 
				deltaX = positions[0].x - positions[i].x
				deltaY = positions[0].y - positions[i].y
			angles.append(math.atan2(deltaY, deltaX))	# add to angles list

        for angle in angles:	# Convert euler angles to Quaternions
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

		# Construct the list of Waypoints
        for i in range(len(positions)): 
			self.waypoints.append(Pose(positions[i], quaternions[i]))

    def init_visualization(self):
        self.eraser = Marker()
        self.eraser.action = 3

        # Init squares list 
        self.squares = Marker()
        self.squares.type = Marker.CUBE_LIST
		# settings
        self.squares.id = 0        
        self.squares.ns = 'waypoints'
        self.squares.lifetime = rospy.Duration(0)	# 0 for permenant
        self.squares.action = Marker.ADD
        # visuals - size
        self.squares.scale.x = 0.15
        self.squares.scale.y = 0.15
        # visuals - colors        
        self.squares.color.r = 1.0 
        self.squares.color.g = 0.7
        self.squares.color.b = 1.0
        self.squares.color.a = 1.0
        # Header settings
        self.squares.header.frame_id = 'odom'
        self.squares.header.stamp = rospy.Time.now()
        self.squares.points = list()
        
        # Init arrows list         
        self.arrows = list()
        for j in range(len(self.waypoints)): 
			# create
			arrow = Marker()
			arrow.type = Marker.ARROW
			# settings 
			arrow.id = j
			arrow.ns = 'waypointArrows'
			arrow.lifetime = rospy.Duration(0)
			arrow.action = Marker.ADD
			# visuals - size
			arrow.scale.x = 0.4		# arrow length
			arrow.scale.y = 0.03	# arrow width
			arrow.scale.z = 0.03	# arrow height
			# visuals - colors
			arrow.color.r = 1.4 
			arrow.color.g = 0.9 
			arrow.color.b = 1.4 
			arrow.color.a = 1.4
			# Header settings
			arrow.header.frame_id = 'odom'
			arrow.header.stamp = rospy.Time.now()
			arrow.points = list()
			# add arrow to arrows list
			self.arrows.append(arrow)
       
        # Attach each square & arrow to robot waypoints
        for k in range(len(self.waypoints)): 
			# each arrow neads two points
			ps = Pose()
			ps = self.waypoints[k]
			self.squares.points.append(ps.position)
			self.arrows[k].pose = ps 

        # Start visualization publisher	(to publish squares & arrows) 
        self.markers_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
    def init_robot_motion(self):
        rospy.loginfo("Initiating Robot Delivery Operation ... ")
        #self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)			# simulation 
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)	# actual robot
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)	# start a client to the move_base server
        self.move_base.wait_for_server(rospy.Duration(60))							# allow time for action server to start
        rospy.loginfo("Moving robot ...")
        
    def updateCompletion(self):	
		# create header message containing update
		completionUpdate = Header()
		completionUpdate.seq = 1	# task update
		completionUpdate.frame_id = 'another Point Completed'
		completionUpdate.stamp = rospy.Time.now()
		
		# publish update message
		self.completionStatus_pub.publish(completionUpdate)

    def shutdown(self):
        # inform user
        rospy.loginfo("Stopping robot ...")

        # Cancel any active goals & stop the robot
        self.move_base.cancel_goal()
        self.cmd_vel_pub.publish(Twist())
        self.markers_pub.publish(self.eraser)	# erase all markers
        
        # Update data in completionStatus file (for Resume)
        completionStatus_file = open('catkin_ws/src/ros_android_hri/initial/completionStatus.txt','w')	
        for completedPoint in self.completedPoints: 
			s = str(completedPoint)
			s += '\n'
			completionStatus_file.write(s)
        completionStatus_file.close()   

        # reset Params
        if rospy.has_param("~res"): rospy.delete_param("~res")   # resume       
        if rospy.has_param("~ran"): rospy.delete_param("~ran")   # random     
        if rospy.has_param("~int"): rospy.delete_param("~int")   # intermittent             
			
if __name__ == '__main__':
	try:
		RobotPatrol()
	except rospy.ROSInterruptException:
		rospy.loginfo("Robot Delivery Operation Finished")
