#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from ros_mobile_robot.msg import PointArray
from functions import is_valid_point, Robot
import random
import math

class LocalRRTDetector:
    def __init__(self):
        rospy.init_node('local_rrt_detector', anonymous=False)
        
        # Parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.eta = rospy.get_param('~eta', 1.0)  # Local RRT branch length
        self.sensor_range = rospy.get_param('~sensor_range', 3.5)  # LDS-02 range
        self.update_frequency = rospy.get_param('~update_frequency', 10.0)
        
        # Initialize map data
        self.map_data = None
        self.tf_listener = tf.TransformListener()
        self.robot_pose = None
        
        # RRT Graph
        self.vertices = []
        self.edges = []
        
        # Publishers and Subscribers
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        self.points_pub = rospy.Publisher('/local_detected_points', PointStamped, queue_size=10)
        self.frontiers_pub = rospy.Publisher('/local_frontiers', PointArray, queue_size=10)
        
        rospy.sleep(1)  # Wait for publishers to initialize
        
    def map_callback(self, data):
        """Process incoming map data"""
        self.map_data = data
        
    def get_robot_pose(self):
        """Get current robot pose"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', self.robot_frame, rospy.Time(0))
            self.robot_pose = trans
            return True
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException):
            return False
            
    def is_in_sensor_range(self, point):
        """Check if point is within sensor range of robot"""
        if self.robot_pose is None:
            return False
        dx = point[0] - self.robot_pose[0]
        dy = point[1] - self.robot_pose[1]
        return math.sqrt(dx*dx + dy*dy) <= self.sensor_range
        
    def get_random_point_in_range(self):
        """Get random point within sensor range"""
        if self.robot_pose is None:
            return None
            
        while True:
            # Random angle and distance within sensor range
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, self.sensor_range)
            
            # Calculate point coordinates
            x = self.robot_pose[0] + distance * math.cos(angle)
            y = self.robot_pose[1] + distance * math.sin(angle)
            
            if is_valid_point(self.map_data, [x, y]):
                return [x, y]
                
    def nearest_vertex(self, point):
        """Find nearest vertex in local RRT graph"""
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
                
    def is_frontier_point(self, point):
        """Check if point is a frontier point in local area"""
        if not is_valid_point(self.map_data, point):
            return False
            
        # Convert to map coordinates
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        
        mx = int((point[0] - origin_x) / resolution)
        my = int((point[1] - origin_y) / resolution)
        
        # Check surrounding cells
        width = self.map_data.info.width
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_x = mx + dx
            new_y = my + dy
            if (0 <= new_x < width and 
                0 <= new_y < self.map_data.info.height):
                idx = new_y * width + new_x
                if self.map_data.data[idx] == 0:  # Free space nearby
                    return True
        return False
        
    def detect_local_frontiers(self):
        """Detect frontiers using local RRT"""
        if self.map_data is None or not self.get_robot_pose():
            return
            
        # Get random point within sensor range
        q_rand = self.get_random_point_in_range()
        if q_rand is None:
            return
            
        # Find nearest vertex
        q_near = self.nearest_vertex(q_rand)
        
        # Get new point
        q_new = self.new_point(q_rand, q_near)
        
        # Check if it's a frontier point
        if self.is_frontier_point(q_new) and self.is_in_sensor_range(q_new):
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
            
        # Periodically clear old vertices outside sensor range
        self.vertices = [v for v in self.vertices if self.is_in_sensor_range(v)]
            
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(self.update_frequency)
        
        while not rospy.is_shutdown():
            self.detect_local_frontiers()
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = LocalRRTDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
