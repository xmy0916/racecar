#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

class following_path:
    def __init__(self):
        self.current_pose = rospy.Subscriber('/pf/pose/odom', Odometry, self.callback_read_current_position, queue_size=1)
        self.Pose = []
        self.path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.callback_read_path, queue_size=1)
        self.path_info = []
        self.Goal = []
        self.navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.reach_goal = False
        self.MAX_VELOCITY = 0.5
        self.MIN_VELOCITY = 0
        self.max_angle = 1
        self.steering_velocity = 1
        self.jerk = 0.0
        self.acceleration = 0.0
        self.LOOKAHEAD_DISTANCE = 0.4
        self.Low_Speed_Mode = False
        
    def callback_read_path(self, data):
        # Organize the pose message and only ask for (x,y) and orientation
        # Read the Real time pose message and load them into path_info
        self.path_info = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_info[-1]) # Set the last pose of the global path as goal location

    def callback_read_current_position(self, data):
        if self.reach_goal: # Stop updating the information.
            self.path_info = []
            self.Pose = []
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0

        if not len(self.path_info) == 0:
            # Read the path information to path_point list
            path_points_x = [float(point[0]) for point in self.path_info]
            path_points_y = [float(point[1]) for point in self.path_info]
            path_points_w = [float(point[2]) for point in self.path_info]

            # Read the current pose of the car from particle filter
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # Convert the quaternion angle to eular angle
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = [float(x), float(y), float(yaw)]

            if self.dist(self.Goal, self.Pose) < 1.0:
                self.Low_Speed_Mode = True
                if self.dist(self.Goal, self.Pose) < 0.3:
                    self.reach_goal = True
                    print('Goal Reached!')
                else:
                    print('Low Speed Mode ON!')
            else:
                self.Low_Speed_Mode = False

            # 2. Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
            dist_array = np.zeros(len(path_points_x))

            for i in range(len(path_points_x)):
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), (x,y))
            
            goal = np.argmin(dist_array) # Assume the closet point as the goal point at first
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 0.3)) & (dist_array > (self.LOOKAHEAD_DISTANCE - 0.3)))[0]
            for id in goal_array:
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
                if abs(diff_angle) < np.pi/4: # Check if the one that is the cloest to the lookahead direction
                    goal = id
                    break

            L = dist_array[goal]
            # 3. Transform the goal point to vehicle coordinates. 
            glob_x = path_points_x[goal] - x 
            glob_y = path_points_y[goal] - y 
            goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
            goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            diff_angle = path_points_w[goal] - yaw # Find the turning angle
            r = L/(2*math.sin(diff_angle)) # Calculate the turning radius
            angle = 2 * math.atan(0.4/r) # Find the wheel turning radius

            angle = np.clip(angle, -self.max_angle, self.max_angle) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
            angle = (0 if abs(angle) < 0.1 else angle)
            VELOCITY = self.speed_control(angle, self.MIN_VELOCITY, self.MAX_VELOCITY)

            # Write the Velocity and angle data into the ackermann message
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = VELOCITY
            ackermann_control.drive.steering_angle = angle
            ackermann_control.drive.steering_angle_velocity = self.steering_velocity   
        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        self.navigation_input.publish(ackermann_control)
    
    # Computes the Euclidean distance between two 2D points p1 and p2
    def dist(self, p1, p2):
	try:
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
	except:
		return 0.5

    # Compute the angle between car direction and goal direction
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sin_ang, cos_ang) 

    # Control the speed of the car within the speed limit
    def speed_control(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        # Assume the speed change linearly with respect to yaw angle
        if self.Low_Speed_Mode:
            Velocity = 0.5
        else:
            k = (MIN_VELOCITY - MAX_VELOCITY)/self.max_angle + 0.5
            Velocity = k * abs(angle) + MAX_VELOCITY
        return Velocity

    
if __name__ == "__main__":

    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()
