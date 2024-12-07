#!/usr/bin/env python

import rospy
import tf
import numpy as np
from copy import copy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from ros_mobile_robot.msg import PointArray
from functions import information_gain, is_valid_point
from sklearn.cluster import MeanShift

class FrontierFilter:
    def __init__(self):
        rospy.init_node('frontier_filter', anonymous=False)
        
        # Parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.info_radius = rospy.get_param('~info_radius', 1.0)
        self.costmap_clearing_threshold = rospy.get_param('~costmap_clearing_threshold', 70)
        self.rate = rospy.Rate(rospy.get_param('~rate', 100))
        
        # Initialize tf listener
        self.tf_listener = tf.TransformListener()
        
        # Class variables
        self.map_data = None
        self.frontiers = []
        self.centroids = []
        
        # Publishers
        self.marker_pub = rospy.Publisher('frontiers', Marker, queue_size=10)
        self.centroid_pub = rospy.Publisher('centroids', Marker, queue_size=10)
        self.filtered_points_pub = rospy.Publisher('filtered_points', PointArray, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        rospy.Subscriber('/detected_points', PointStamped, self.points_callback)
        rospy.Subscriber('/local_detected_points', PointStamped, self.points_callback)
        
        # Setup visualization markers
        self.init_markers()
        
    def init_markers(self):
        """Initialize visualization markers"""
        # Frontiers marker
        self.points_marker = Marker()
        self.points_marker.header.frame_id = "map"
        self.points_marker.type = Marker.POINTS
        self.points_marker.action = Marker.ADD
        self.points_marker.scale.x = 0.2
        self.points_marker.scale.y = 0.2
        self.points_marker.color.r = 1.0
        self.points_marker.color.a = 1.0
        
        # Centroids marker
        self.centroids_marker = Marker()
        self.centroids_marker.header.frame_id = "map"
        self.centroids_marker.type = Marker.POINTS
        self.centroids_marker.action = Marker.ADD
        self.centroids_marker.scale.x = 0.2
        self.centroids_marker.scale.y = 0.2
        self.centroids_marker.color.g = 1.0
        self.centroids_marker.color.a = 1.0
        
    def map_callback(self, data):
        """Store map data when received"""
        self.map_data = data
        
    def points_callback(self, data):
        """Process incoming frontier points"""
        if self.map_data is None:
            return
            
        point = np.array([data.point.x, data.point.y])
        if is_valid_point(self.map_data, point):
            self.frontiers.append(point)
            
    def cluster_frontiers(self):
        """Cluster frontier points using MeanShift"""
        if len(self.frontiers) < 2:
            return self.frontiers
            
        # Convert to numpy array
        points = np.array(self.frontiers)
        
        # Perform clustering
        ms = MeanShift(bandwidth=0.3)
        ms.fit(points)
        
        return ms.cluster_centers_
        
    def filter_centroids(self, centroids):
        """Filter centroids based on information gain and validity"""
        filtered = []
        for point in centroids:
            # Skip if point is invalid or has low information gain
            if not is_valid_point(self.map_data, point):
                continue
                
            info_gain = information_gain(self.map_data, point, self.info_radius)
            if info_gain < 0.2:  # Minimum information gain threshold
                continue
                
            filtered.append(point)
        return filtered
        
    def publish_visualization(self):
        """Publish visualization markers"""
        # Update timestamps
        now = rospy.Time.now()
        self.points_marker.header.stamp = now
        self.centroids_marker.header.stamp = now
        
        # Update frontier points
        self.points_marker.points = []
        for point in self.frontiers:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            self.points_marker.points.append(p)
            
        # Update centroid points
        self.centroids_marker.points = []
        for point in self.centroids:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            self.centroids_marker.points.append(p)
            
        # Publish markers
        self.marker_pub.publish(self.points_marker)
        self.centroid_pub.publish(self.centroids_marker)
        
    def publish_filtered_points(self):
        """Publish filtered points for the assigner"""
        msg = PointArray()
        for point in self.centroids:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            msg.points.append(p)
        self.filtered_points_pub.publish(msg)
        
    def run(self):
        """Main run loop"""
        while not rospy.is_shutdown():
            if self.map_data is not None and len(self.frontiers) > 0:
                # Cluster frontiers
                self.centroids = self.cluster_frontiers()
                
                # Filter centroids
                self.centroids = self.filter_centroids(self.centroids)
                
                # Publish visualization and filtered points
                self.publish_visualization()
                self.publish_filtered_points()
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        filter_node = FrontierFilter()
        filter_node.run()
    except rospy.ROSInterruptException:
        pass
