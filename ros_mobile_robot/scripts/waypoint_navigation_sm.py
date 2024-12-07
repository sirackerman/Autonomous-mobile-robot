#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WaypointQueue:
    def __init__(self):
        self.waypoints = []
        self.current_goal_active = False
        # Subscribe to goals from RViz
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
    def goal_callback(self, msg):
        self.waypoints.append(msg.pose)
        rospy.loginfo("Received new goal from RViz. Queue size: %d", len(self.waypoints))

        
    def get_next_waypoint(self):
        if self.waypoints:
            return self.waypoints.pop(0)
        return None

    def has_waypoints(self):
        return len(self.waypoints) > 0

class GetNextGoal(smach.State):
    def __init__(self, waypoint_queue):
        smach.State.__init__(self, 
                            outcomes=['goal_available', 'no_more_goals'],
                            output_keys=['current_goal'])
        self.waypoint_queue = waypoint_queue

    def execute(self, userdata):
        rospy.sleep(1)  # Small delay to check for new goals
        next_goal = self.waypoint_queue.get_next_waypoint()
        if next_goal:
            userdata.current_goal = next_goal
            return 'goal_available'
        return 'no_more_goals'

class NavigateToGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['goal_reached', 'goal_failed'],
                            input_keys=['current_goal'])
        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Unsubscribe from the regular move_base_simple/goal topic to prevent direct goal setting
        self.move_base_simple_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.dummy_callback)

        # Wait for action server
        self.client.wait_for_server()

    def dummy_callback(self, msg):
        # This prevents RViz goals from directly going to move_base
        pass
        
    def execute(self, userdata):
        # Cancel any existing goals first
        self.client.cancel_all_goals()
        rospy.sleep(1)  # Give time for cancellation

       # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.current_goal
        
        # Send goal and wait for result
        rospy.loginfo("Navigating to goal...")
        self.client.send_goal(goal)
        # Wait for the result with monitoring
        while not rospy.is_shutdown():
            if self.client.wait_for_result(rospy.Duration(0.5)):  # Check every 0.5 seconds
                # Check if goal is complete
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal reached successfully!")
                    return 'goal_reached'
                else:
                    rospy.logwarn("Goal failed with state: %d", state)
                    return 'goal_failed'
            
            rospy.sleep(0.1)  # Small sleep to prevent CPU hogging

class GoalComplete(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue'])
        
    def execute(self, userdata):
        rospy.loginfo('Goal completed! Moving to next goal...')
        return 'continue'

def main():
    rospy.init_node('waypoint_navigation_sm')

    # Create waypoint queue
    waypoint_queue = WaypointQueue()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GET_NEXT_GOAL', 
                              GetNextGoal(waypoint_queue),
                              transitions={'goal_available': 'NAVIGATE_TO_GOAL',
                                         'no_more_goals': 'GET_NEXT_GOAL'})
        
        smach.StateMachine.add('NAVIGATE_TO_GOAL',
                              NavigateToGoal(),
                              transitions={'goal_reached': 'GOAL_COMPLETE',
                                         'goal_failed': 'GET_NEXT_GOAL'})
        
        smach.StateMachine.add('GOAL_COMPLETE',
                              GoalComplete(),
                              transitions={'continue': 'GET_NEXT_GOAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('waypoint_nav_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan in a separate thread so we don't block
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
