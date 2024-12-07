#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from ros_mobile_robot.msg import PointArray
from functions import is_valid_point
import random
import math

class GlobalRRTDetector:
    def __init__(self):
        rospy.init_node('global_rrt_detector', anonymous=False)
        
        # Parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.eta = rospy.get_param('~eta', 15.0)  # Maximum RRT branch length
        self.min_frontier_size = rospy.get_param('~min_frontier_size', 0.3)
        self.update_frequency = rospy.get_param('~update_frequency', 5.0)
        
        # Initialize map data
        self.map_data = None
        self.resolution = 0
        self.width = 0
        self.height = 0
        self.origin_x = 0
        self.origin_y = 0
        
        # RRT Graph
        self.vertices = []
        self.edges = []
        
        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        self.points_pub = rospy.Publisher('/detected_points', PointStamped, queue_size=10)
        self.frontiers_pub = rospy.Publisher('/global_frontiers', PointArray, queue_size=10)
        
        rospy.sleep(1)  # Wait for publishers to initialize
        
    def map_callback(self, data):
        """Process incoming map data"""
        self.map_data = data
        self.resolution = data.info.resolution
        self.width = data.info.width
        self.height = data.info.height
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        
    def world_to_map(self, wx, wy):
        """Convert world coordinates to map coordinates"""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my
        
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates"""
        wx = mx * self.resolution + self.origin_x
        wy = my * self.resolution + self.origin_y
        return wx, wy
        
    def is_frontier_point(self, mx, my):
        """Check if a point is a frontier (unknown space adjacent to free space)"""
        if not (0 <= mx < self.width and 0 <= my < self.height):
            return False
            
        # Point must be unknown
        if self.map_data.data[my * self.width + mx] != -1:
            return False
            
        # Check if adjacent to free space
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_x = mx + dx
            new_y = my + dy
            if (0 <= new_x < self.width and 0 <= new_y < self.height and 
                self.map_data.data[new_y * self.width + new_x] == 0):
                return True
        return False
        
    def get_random_point(self):
        """Get a random valid point in the map"""
        while True:
            mx = random.randint(0, self.width - 1)
            my = random.randint(0, self.height - 1)
            wx, wy = self.map_to_world(mx, my)
            if is_valid_point(self.map_data, [wx, wy]):
                return [wx, wy]
                
    def nearest_vertex(self, point):
        """Find nearest vertex in RRT graph"""
        if not self.vertices:
            return None
        distances = [math.sqrt((v[0] - point[0])**2 + (v[1] - point[1])**2) 
                    for v in self.vertices]
        return self.vertices[distances.index(min(distances))]
        
    def new_point(self, q_rand, q_near):
        """Get new point at most eta distance from q_near toward q_rand"""
        if q_near is None:
            return q_rand
            
        dx = q_rand[0] - q_near[0]
        dy = q_rand[1] - q_near[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist <= self.eta:
            return q_rand
            
        theta = math.atan2(dy, dx)
        return [q_near[0] + self.eta * math.cos(theta),
                q_near[1] + self.eta * math.sin(theta)]
                
    def detect_frontiers(self):
        """Detect frontiers using RRT"""
        if self.map_data is None:
            return
            
        # Get random point
        q_rand = self.get_random_point()
        
        # Find nearest vertex
        q_near = self.nearest_vertex(q_rand)
        
        # Get new point
        q_new = self.new_point(q_rand, q_near)
        
        # Convert to map coordinates
        mx, my = self.world_to_map(q_new[0], q_new[1])
        
        # Check if it's a frontier
        if self.is_frontier_point(mx, my):
            point = PointStamped()
            point.header.frame_id = "map"
            point.header.stamp = rospy.Time.now()
            point.point.x = q_new[0]
            point.point.y = q_new[1]
            point.point.z = 0.0
            self.points_pub.publish(point)
            
        # Add to RRT graph
        self.vertices.append(q_new)
        if q_near is not None:
            self.edges.append((q_near, q_new))
            
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(self.update_frequency)
        
        while not rospy.is_shutdown():
            if self.map_data is not None:
                self.detect_frontiers()
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = GlobalRRTDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
