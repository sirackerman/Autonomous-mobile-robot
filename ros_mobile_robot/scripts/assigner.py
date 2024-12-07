#!/usr/bin/env python

import rospy
import numpy as np
from copy import copy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from ros_mobile_robot.msg import PointArray
from numpy.linalg import norm
from functions import Robot, information_gain, discount

class FrontierAssigner:
    def __init__(self):
        rospy.init_node('frontier_assigner', anonymous=False)
        
        # Parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.info_radius = rospy.get_param('~info_radius', 1.0)
        self.info_multiplier = rospy.get_param('~info_multiplier', 3.0)
        self.hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)
        self.hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)
        self.delay_after_assignment = rospy.get_param('~delay_after_assignment', 0.5)
        self.rate = rospy.Rate(rospy.get_param('~rate', 100))
        
        # Initialize robot
        self.robot = Robot("")  # Empty namespace for single robot
        
        # Class variables
        self.map_data = None
        self.frontiers = []
        
        # Subscribers
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        rospy.Subscriber('filtered_points', PointArray, self.frontiers_callback)
        
        # Publishers for visualization
        self.assigned_point_pub = rospy.Publisher('assigned_point', Marker, queue_size=10)
        
        # Initialize visualization marker
        self.init_marker()
        
    def init_marker(self):
        """Initialize visualization marker for assigned point"""
        self.assigned_marker = Marker()
        self.assigned_marker.header.frame_id = "map"
        self.assigned_marker.type = Marker.SPHERE
        self.assigned_marker.action = Marker.ADD
        self.assigned_marker.scale.x = 0.5
        self.assigned_marker.scale.y = 0.5
        self.assigned_marker.scale.z = 0.5
        self.assigned_marker.color.r = 0.0
        self.assigned_marker.color.g = 0.0
        self.assigned_marker.color.b = 1.0
        self.assigned_marker.color.a = 1.0
        
    def map_callback(self, data):
        """Store map data when received"""
        self.map_data = data
        
    def frontiers_callback(self, data):
        """Process incoming frontier points"""
        self.frontiers = []
        for point in data.points:
            self.frontiers.append(np.array([point.x, point.y]))
            
    def compute_path_cost(self, frontier):
        """Compute cost to reach frontier"""
        robot_pos = self.robot.get_position()
        return norm(robot_pos - frontier)
        
    def compute_frontier_utilities(self):
        """Compute utility values for all frontiers"""
        utilities = []
        robot_pos = self.robot.get_position()
        
        for frontier in self.frontiers:
            # Calculate information gain
            info_gain = information_gain(self.map_data, frontier, self.info_radius)
            
            # Apply hysteresis if close to current assigned point
            if norm(frontier - self.robot.assigned_point) <= self.hysteresis_radius:
                info_gain *= self.hysteresis_gain
                
            # Calculate cost (distance)
            cost = self.compute_path_cost(frontier)
            
            # Calculate utility
            utility = info_gain * self.info_multiplier - cost
            utilities.append(utility)
            
        return utilities
        
    def publish_assigned_point(self, point):
        """Publish visualization of assigned point"""
        self.assigned_marker.header.stamp = rospy.Time.now()
        self.assigned_marker.pose.position.x = point[0]
        self.assigned_marker.pose.position.y = point[1]
        self.assigned_marker.pose.position.z = 0.0
        self.assigned_point_pub.publish(self.assigned_marker)
        
    def assign_frontier(self):
        """Assign best frontier to robot"""
        # If no frontiers or map, return
        if not self.frontiers or self.map_data is None:
            return
            
        # Get current robot state
        robot_state = self.robot.get_state()
        
        # If robot is still moving to goal, skip
        if robot_state in [0, 1]:  # PENDING or ACTIVE
            return
            
        # Compute utilities for all frontiers
        utilities = self.compute_frontier_utilities()
        
        if not utilities:
            return
            
        # Select frontier with highest utility
        best_idx = np.argmax(utilities)
        best_frontier = self.frontiers[best_idx]
        
        # Send robot to best frontier
        self.robot.send_goal(best_frontier)
        
        # Visualize assigned point
        self.publish_assigned_point(best_frontier)
        
        # Wait before next assignment
        rospy.sleep(self.delay_after_assignment)
        
    def run(self):
        """Main run loop"""
        while not rospy.is_shutdown():
            self.assign_frontier()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        assigner = FrontierAssigner()
        assigner.run()
    except rospy.ROSInterruptException:
        pass
