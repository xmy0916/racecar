#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

class A_star_path:
    def __init__(self):
        self.current_pose = rospy.Subscriber('/vesc/odom', Odometry, self.callback_read_current_position, queue_size=1)
        self.Mapdata = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.callback_read_map_data, queue_size=1)
        self.local_path = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.callback_local_target, queue_size=1)
        self.control_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.Pose = []
        self.start_id = []
        self.Goal = []
        self.path = []
        self.GridCell = []
        self.origin = []
        self.width = []
        self.height = []
        self.resolution = []
        self.init_pose_set = False
        self.Steering = list(a*0.05 for a in range(-10, 11)) 
        self.Speed = [0.75] 
        self.car_length = 0.33

    def callback_read_map_data(self, data):
        self.GridCell = list(data.data)
        self.origin = data.info.origin
        self.width = data.info.width
        self.height = data.info.height
        self.resolution = data.info.resolution
        # Convert pose position to location of the map cell
        self.GridCell = np.reshape(np.array(self.GridCell), (self.height, self.width))
        
    def callback_read_current_position(self, data):
        # Read the current car pose from particle filter
        px = data.pose.pose.position.x
        py = data.pose.pose.position.y
        pori_x = data.pose.pose.orientation.x
        pori_y = data.pose.pose.orientation.y
        pori_z = data.pose.pose.orientation.z
        pori_w = data.pose.pose.orientation.w
        pose_quaternion = (pori_x, pori_y, pori_z, pori_w)
        pose_eular = euler_from_quaternion(pose_quaternion)
        pose_yaw = pose_eular[2]
        # Set the pose object with current car inferred pose
        self.Pose = [float(px), float(py), float(pose_yaw)]
        # Convert the current pose to index on grid cell
        if not len(self.Pose) == 0:
            pose_ind_x = int((self.Pose[0] - self.origin.position.x)/self.resolution)
            pose_ind_y = int((self.Pose[1] - self.origin.position.y)/self.resolution)
            if (not self.init_pose_set):
                if (self.GridCell[pose_ind_y][pose_ind_x]) == 0:
                    print('Initial Pose set!')
                    self.init_pose_set = True
                    self.start_id = [pose_ind_x, pose_ind_y]
                else:
                    print('Invalid initial pose, please choose another pose location.')
        
        # Sample the potential local goal for the car to navigate
        pose_estimate = self.ackermann_planner()
        cost_array = []
        for candi in pose_estimate:
            cost_value = self.cost_fn(candi)
            cost_array.append(cost_value)
        # Find the action correpsonding to the lowest cost
        min_cost_id = cost_array.index(min(cost_array))
        # print(pose_estimate[min_cost_id][0:2])
        # Load the control message to the controller input
        ackermann_control_input = AckermannDriveStamped()
        ackermann_control_input.drive.speed = pose_estimate[min_cost_id][2]
        ackermann_control_input.drive.steering_angle = pose_estimate[min_cost_id][3]
        self.control_input.publish(ackermann_control_input)
    
    def callback_local_target(self, data):
        self.path = []
        path_info = data.poses
        for path in path_info:
            px = path.pose.position.x
            py = path.pose.position.y
            pori_x = path.pose.orientation.x
            pori_y = path.pose.orientation.y
            pori_z = path.pose.orientation.z
            pori_w = path.pose.orientation.w
            path_quaternion = (pori_x, pori_y, pori_z, pori_w)
            path_eular = euler_from_quaternion(path_quaternion)
            path_yaw = path_eular[2]
            self.path.append([px, py, path_yaw])
        self.Goal = self.path[-1]

    def ackermann_planner(self, look_forward_t = 0.5):
        # define the possible action from current state
        goal_estimation = [] # Format: [goal_x, goal_y, speed_control, steering_control]
        for speed in self.Speed:
            for steering in self.Steering:
                arc_length = speed*look_forward_t
                if steering == 0:
                    goal_estimation.append([float(self.Pose[0] + math.cos(self.Pose[2])*arc_length), float(self.Pose[1] + math.sin(self.Pose[2])*arc_length), speed, steering])
                else:
                    turning_radius = self.car_length/math.tan(steering)
                    alpha = arc_length/turning_radius
                    x_dot = turning_radius * math.sin(alpha)
                    y_dot = turning_radius * (1 - math.cos(alpha))
                    goal_x = float(self.Pose[0] + x_dot * math.cos(self.Pose[2]) - y_dot * math.sin(self.Pose[2]))
                    goal_y = float(self.Pose[1] + x_dot * math.sin(self.Pose[2]) + y_dot * math.cos(self.Pose[2]))
                    goal_estimation.append([goal_x, goal_y, speed, steering])
        return goal_estimation
    
    def cost_fn(self, sample, collision_weight = 6, distance_weight = 6, path_weight = 4):
        # Check if the sample is in collision with obstacles, if so, set cost value to inf
        # Use path error and euclidean distance as cost function 
        distance_cost = self.euclidean_heuristic_map(sample)
        # Update the collsion cost of the sample
        collision_cost = -self.collision_distance(sample)
        path_error_cost = 0
        # Find the minimum distance between estimation point and point on the path
        min_distance = float('inf')
        counter = 0
        min_dist_idx = 0
        for point in self.path:
            distance = ((sample[0] - point[0])**2 + (sample[1] - point[1])**2)**0.5
            if distance < min_distance:
                min_distance = distance
                min_dist_idx = counter
            counter += 1
        path_error_cost = min_distance
        cost_value = collision_cost * collision_weight + distance_cost * distance_weight + path_error_cost * path_weight
        return cost_value

    def euclidean_heuristic_map(self, current_pose):
        # Start with goal location and then gradually increase to the rest of the map.
        if len(self.Goal) != 0:
            euclidean_distance = ((self.Goal[0] - current_pose[0])**2 + (self.Goal[1] - current_pose[1])**2)**0.5
        else:
            euclidean_distance = float('inf')
        return euclidean_distance
    
    def collision_distance(self, sample):
        # This function is intended to find the minimum distance between obstacle and estimated pose
        min_distance = 1.5
        for index_y in range(0, self.height):
            for index_x in range(0,self.width):
                if self.GridCell[index_y][index_x] != 0: # Indicating the obstacles
                    obstacle_pose_y = index_y * self.resolution + self.origin.position.y
                    obstacle_pose_x = index_x * self.resolution + self.origin.position.x
                    dist = ((sample[0] - obstacle_pose_x)**2 + (sample[1] - obstacle_pose_y)**2)**0.5
                    if dist < min_distance:
                        min_distance = dist
        return min_distance

if __name__ == "__main__":
    rospy.init_node("A_star_planning")
    A_star_path()
    rospy.spin()