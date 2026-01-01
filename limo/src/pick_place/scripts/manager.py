#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from pick_place.srv import AddObjectToScene
from actionlib_msgs.msg import GoalID
import actionlib
import math
from typing import Literal, Dict
from find_object_2d.msg import ObjectsStamped
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse

class PickPlaceManager:
    """
        Manages the pick and place operations for the robot in a fully-autonomous mission.
        This node coordinates search, navigation, object detection, grasp planning and execution.
        To fully run the pick and place mission, launch this node along with the necessary
        supporting nodes such as object detection, navigation, and manipulation.
            >> roslaunch pick_place gazebo.launch # Unless in real-world mode
            >> roslaunch pick_place object_search.launch # For object detection and search
            >> roslaunch pick_place moveit.launch # For manipulation
            >> roslaunch pick_place object_grip.launch # For object gripping and placement
        All the above nodes can be launched together with this node using the mission launch file:
            >> roslaunch pick_place mission.launch # This node to manage the overall mission
    """
    
    def __init__(self):
        
        # States
        self.exploare_active = True
        self.object = None
        self.object_detected = False
        self.table = None
        self.table_detected = False
        self.cubes = {}
        self.labels = {}
        # self.robot_pose = None
        
        # Initialize the ROS node
        rospy.init_node('pick_place_manager')
        rospy.loginfo("Pick and Place Manager Node Initialized")
        
        # Movebase client
        # self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Publishers
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        
        # Services
        # self.move_base.wait_for_service()
        rospy.wait_for_service("/rtabmap/pause")
        rospy.wait_for_service("/rtabmap/resume")
        rospy.wait_for_service("/rtabmap/trigger_new_map")
        rospy.wait_for_service("/add_object_to_scene")
        rospy.wait_for_service("/pick_object")
        rospy.wait_for_service("/place_object")

        self.pause_rtabmap = rospy.ServiceProxy("/rtabmap/pause", Empty)
        self.resume_rtabmap = rospy.ServiceProxy("/rtabmap/resume", Empty)
        self.add_object = rospy.ServiceProxy("/add_object_to_scene", AddObjectToScene)
        self.pick_lift_object = rospy.ServiceProxy("/pick_object", Empty)
        self.place_release_object = rospy.ServiceProxy("/place_object", Empty)
        
        
        # Subscribers "This intiates the callback when an object is detected"
        self.obj_sub = rospy.Subscriber("/object/detection", MarkerArray, self.object_detected_callback)
        # self.odom_sub = rospy.Subscriber("/odom", )
        
        rospy.loginfo("Pick & Place Manager ready")
    
    # HARD STOP: stop exploration + pause RTAB-Map
    def hard_stop(self):
        rospy.loginfo("HARD STOP: Stopping exploration & pausing RTAB-Map")

        # 1) Cancel all active move_base goals
        self.cancel_pub.publish(GoalID())

        # 2) Pause RTAB-Map
        try:
            self.pause_rtabmap()
        except rospy.ServiceException as e:
            rospy.logwarn(f"RTAB-Map pause failed: {e}")

        self.explore_active = False
    
    # Object/Table detection callback
    def object_detected_callback(self, msg):
        # Process the message
        self.cubes = {}
        self.labels = {}
        # First pass: Extract cubes and labels
        for marker in msg.markers:
            if marker.type == 1: # CUBE
                self.cubes[marker.id] = {
                    'position': {'x': marker.pose.position.x, 'y': marker.pose.position.y, 'z': marker.pose.position.z},
                    'orientation': {
                        'x': marker.pose.orientation.x, 'y': marker.pose.orientation.y,
                        'z': marker.pose.orientation.z, 'w': marker.pose.orientation.w
                    },
                    'frame_id': marker.header.frame_id,
                    'scale': {
                        'x': marker.scale.x, 'y': marker.scale.y, 'z': marker.scale.z
                    }
                }
            elif marker.type == 9: # TEXT_VIEW_FACING (label)
                self.labels[marker.id] = {
                    'text': ''.join(marker.text.split('\n')[0].split()),
                    'position': {
                        'x': marker.pose.position.x,
                        'y': marker.pose.position.y,
                        'z': marker.pose.position.z
                    }
                }
        for id_, label in self.labels.items():
            label_type = label["text"]
            if (id_-1) in self.cubes:
                if label_type == "redcube":
                    self.object = self.cubes[id_-1]
                    self.object_detected = True
                elif label_type == "yellowtable":
                    self.table = self.cubes[id_-1]
                    self.table_detected = True
        rospy.loginfo(f"Detected object: {self.object_detected}, detected table: {self.table_detected}")
    
    # Add the detected object/table to the planning scene
    def add_to_scene(self, type: Literal["object", "table"]):
        rospy.loginfo(f"Adding {type} to planning scene...")
        
        message = AddObjectToScene()
        message.is_table = True if type=="table" else False
        self.add_object(message)
        
        rospy.loginfo(f"Added {type} to planning scene!")
    
    # Navigate to detected object with offset
    def navigate_to_object(self):
        rospy.loginfo("Navigating to detected object...")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()

        obj_x = self.object_pose.pose.position.x
        obj_y = self.object_pose.pose.position.y

        dx = obj_x
        dy = obj_y
        distance = math.sqrt(dx*dx + dy*dy)

        offset = 0.05
        if distance > offset:
            factor = (distance - offset) / distance
            goal.pose.position.x = dx * factor
            goal.pose.position.y = dy * factor
        else:
            goal.pose.position.x = dx
            goal.pose.position.y = dy

        goal.pose.position.z = 0.0
        goal.pose.orientation = self.object_pose.pose.orientation

        self.goal_pub.publish(goal)
        rospy.loginfo("Navigation goal published (offset 0.35 m)")
    
    # Resume RTAB-Map (optional, call after pick)
    def resume_mapping(self):
        rospy.loginfo("Resuming RTAB-Map...")
        try:
            self.resume_rtabmap()
        except rospy.ServiceException as e:
            rospy.logwarn(f"RTAB-Map resume failed: {e}")
    
    def pick_object(self):
        rospy.loginfo("Picking up the object...")
        # Implement pick logic here
        self.pick_lift_object()
    
    # Step 4: Search for the drop-off table
    def navigate_to_dropoff(self):
        rospy.loginfo("Searching for the drop-off zone/table...")
        # Implement navigation logic here
        self.exploare_active = True
        # TODO: run the explore_lite node again to search for the drop-off table, resume SLAM if it was paused and reset its map
        # 
        # TODO: once the table is detected, store its location, break the table search and proceed to navigation
        self.exploare_active = False
        pass
    
    # Step 5: Navigate to drop-off table
    def navigate_to_table(self, table_location):
        rospy.loginfo(f"Apporaching the drop-off table at {table_location}...")
        # Implement navigation logic here
        # TODO: the robot should be at an optimal distance for both manipulation and perception (roughly: 0.35 meters)
        #
        # TODO: use move_base, publish goals to /move_base_simple/goal
        pass
    
    # Step 6: Place the object
    def place_object(self):
        rospy.loginfo("Placing the object in its designated location...")
        # Implement place logic here
        self.place_release_object()
    
    # A callback when the full mission is triggered
    def execute_mission(self, msg):
        # High-level method to execute the full pick and place mission
        if not self.exploare_active:
            rospy.logwarn("Exploration is not active. A new exploration is triggered!")
            # TODO: run the explore_lite node again to search for objects
            self.exploare_active = True
        rospy.logwarn("Executing full pick and place mission...")
        return EmptyResponse()
        # self.search_for_objects()
        # if self.object_location:
        #     self.navigate_to_object(self.object_location)
        #     self.pick_object()
        #     self.navigate_to_dropoff()
        #     if self.object_location:  # Assuming we reuse object_location for table location
        #         self.navigate_to_table(self.object_location)
        #         self.place_object()
    
    def run(self):
        # TODO: Create a ros service that triggers the full pick and place mission
        rospy.Service('run/mission', Empty, self.execute_mission)
        rospy.loginfo("Pick and Place Manager is running...")
        rospy.spin()

if __name__ == '__main__':
    manager = PickPlaceManager()
    try:
        manager.run()
    except rospy.ROSInterruptException:
        pass