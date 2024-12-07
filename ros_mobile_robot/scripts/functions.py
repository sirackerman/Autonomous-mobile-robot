#!/usr/bin/env python

import rospy
import tf
import actionlib
from numpy import array, linalg, inf
from numpy.linalg import norm
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from math import floor

class Robot:
    """Class to handle single robot exploration"""
    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.listener = tf.TransformListener()
        
        # Wait for robot transform
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(
                    self.global_frame, 
                    self.robot_frame,
                    rospy.Time(0),
                    rospy.Duration(5.0)
                )
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Waiting for robot transform...")
                rospy.sleep(1)
        
        # Get initial position
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.global_frame, self.robot_frame, rospy.Time(0))
            self.position = array([trans[0], trans[1]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get robot position!")
            self.position = array([0, 0])

        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_position(self):
        """Get current robot position"""
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.global_frame, self.robot_frame, rospy.Time(0))
            self.position = array([trans[0], trans[1]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get robot position!")
        return self.position

    def send_goal(self, point):
        """Send a goal point to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.assigned_point = array(point)

    def cancel_goal(self):
        """Cancel current goal"""
        self.client.cancel_goal()
        self.assigned_point = self.get_position()

    def get_state(self):
        """Get move_base state"""
        return self.client.get_state()

def index_of_point(mapData, point):
    """Get the index of a point in the map data array"""
    resolution = mapData.info.resolution
    origin_x = mapData.info.origin.position.x
    origin_y = mapData.info.origin.position.y
    width = mapData.info.width
    
    x = point[0]
    y = point[1]
    
    idx_x = int(floor((x - origin_x) / resolution))
    idx_y = int(floor((y - origin_y) / resolution))
    
    index = idx_y * width + idx_x
    
    if index >= len(mapData.data) or index < 0:
        return None
    return index

def point_of_index(mapData, index):
    """Get the coordinates of a point from its index"""
    resolution = mapData.info.resolution
    origin_x = mapData.info.origin.position.x
    origin_y = mapData.info.origin.position.y
    width = mapData.info.width
    
    y = origin_y + (index // width) * resolution
    x = origin_x + (index % width) * resolution
    
    return array([x, y])

def information_gain(mapData, point, radius):
    """Calculate information gain for exploring a point"""
    info_gain = 0
    index = index_of_point(mapData, point)
    if index is None:
        return 0
        
    r_region = int(radius / mapData.info.resolution)
    width = mapData.info.width

    # Check cells in a square region around the point
    for i in range(-r_region, r_region + 1):
        for j in range(-r_region, r_region + 1):
            current_index = index + i * width + j
            
            # Skip if outside map bounds
            if current_index >= len(mapData.data) or current_index < 0:
                continue
                
            # Calculate actual distance to cell
            cell_point = point_of_index(mapData, current_index)
            distance = norm(array(point) - cell_point)
            
            # Only count if within radius and unknown (-1)
            if distance <= radius and mapData.data[current_index] == -1:
                info_gain += 1

    return info_gain * (mapData.info.resolution ** 2)

def discount(mapData, assigned_pt, centroids, info_gains, radius):
    """Discount information gain for points near already assigned goals"""
    index = index_of_point(mapData, assigned_pt)
    if index is None:
        return info_gains
        
    r_region = int(radius / mapData.info.resolution)
    width = mapData.info.width

    # For each frontier point
    for i in range(len(centroids)):
        centroid = centroids[i]
        # If within radius of assigned point, discount its info gain
        if norm(centroid - assigned_pt) <= radius:
            info_gains[i] = max(0, info_gains[i] - r_region)

    return info_gains

def is_valid_point(mapData, point):
    """Check if a point is valid (inside map and not occupied)"""
    index = index_of_point(mapData, point)
    if index is None:
        return False
    return 0 <= index < len(mapData.data) and mapData.data[index] != 100
