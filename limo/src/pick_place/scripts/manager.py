#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pick_place.srv import AddObjectToScene, AddObjectToSceneRequest
import actionlib
import math
import subprocess
import os
import time
import signal
from typing import Literal, Dict
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

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
        self.exploration = None
        self.object: Dict | None = None
        self.object_detected = False
        self.table: Dict | None = None
        self.table_detected = False
        self.cubes = {}
        self.labels = {}
        
        # Initialize the ROS node
        rospy.init_node('pick_place_manager')
        rospy.loginfo("Pick and Place Manager Node Initialized")
        
        # Movebase client
        self.move_base_actions = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_actions.wait_for_server()
        
        # Services
        rospy.wait_for_service("/rtabmap/reset")
        self.reset_rtabmap = rospy.ServiceProxy('/rtabmap/reset', Empty)
        rospy.wait_for_service("/rtabmap/pause")
        self.pause_rtabmap = rospy.ServiceProxy('/rtabmap/pause', Empty)
        rospy.wait_for_service("/add_object_to_scene")
        self.add_object = rospy.ServiceProxy("/add_object_to_scene", AddObjectToScene)
        rospy.wait_for_service("/pick_object")
        self.pick_lift_object = rospy.ServiceProxy("/pick_object", Trigger)
        rospy.wait_for_service("/place_object")
        self.place_release_object = rospy.ServiceProxy("/place_object", Trigger)
        
        # Subscribers "This intiates the callback when an object is detected"
        self.obj_sub = rospy.Subscriber("/object/detection", MarkerArray, self.object_detected_callback)
        # self.odom_sub = rospy.Subscriber("/odom", )
        
        rospy.loginfo("Pick & Place Manager ready")
    
    # start search
    def start_search(self):
        rospy.loginfo("Starting search!!!")
        self.object_detected = False
        self.table_detected = False
        if self.exploration:
            rospy.logwarn("Exploration is already running !!!")
            return
        cmd = ["roslaunch", "pick_place", "exploration.launch"]
        self.exploration = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(2)
        rospy.loginfo("Search has started!")
    
    # stop search
    def stop_search(self):
        # Cancel current move-base goal
        rospy.logwarn("Stopping search!!!!")
        rospy.loginfo("Cancelling current move_base_goal")
        self.move_base_actions.cancel_goal()
        rospy.logwarn("Goal cancelled!")
        # Stop and kill exploration launch
        rospy.loginfo("Killing exploration process")
        if self.exploration:
            os.killpg(os.getpgid(self.exploration.pid), signal.SIGINT)
            self.exploration.wait()
            self.exploration = None
        rospy.logwarn("Exploration process is killed!")
        # Pause rtabmapping
        self.pause_rtabmap()
        rospy.loginfo("SLAM is paused!!")
    
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
        obj_once_detected = self.object_detected
        tab_once_detected = self.table_detected
        for id_, label in self.labels.items():
            label_type = label["text"]
            if (id_-1) in self.cubes:
                if label_type == "redcube":
                    self.object = self.cubes[id_-1]
                    self.object_detected = True
                elif label_type == "yellowtable":
                    self.table = self.cubes[id_-1]
                    self.table_detected = True
        if self.object_detected != obj_once_detected or self.table_detected != tab_once_detected:
            rospy.loginfo(f"Detected object: {self.object_detected}, detected table: {self.table_detected}")
    
    # Add the detected object/table to the planning scene
    def add_to_scene(self, type: Literal["object", "table"]):
        rospy.loginfo(f"Adding {type} to planning scene...")
        message = AddObjectToSceneRequest()
        message.is_table = 1 if type=="table" else 0
        result = self.add_object(message)
        if result.success:
            return True
        i = 0
        while not result.success and i < 3:
            result = self.add_object(message)
            rospy.sleep(0.5)
            if result.success:
                rospy.loginfo(f"Added {type} to planning scene!")
                return True
        rospy.logwarn(f"Failed to add {type} to planning scene!")
        return False
    
    # Navigate to detected object with offset
    def navigate_to_object(self, object: Literal["object", "table"]):
        if object == "object":
            rospy.loginfo("Navigating to detected object...")
            obj = self.object.copy()
        else:
            rospy.loginfo("Searching for the drop-off zone/table...")
            obj = self.table.copy()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        obj_x = obj['position']["x"]
        obj_y = obj['position']["y"]
        distance = math.sqrt(obj_x**2 + obj_y**2)
        offset = 0.05
        if distance > offset:
            factor = (distance - offset) / distance
            goal.target_pose.pose.position.x = obj_x * factor
            goal.target_pose.pose.position.y = obj_y * factor
        else:
            goal.target_pose.pose.position.x = obj_x
            goal.target_pose.pose.position.y = obj_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = obj['orientation']["x"]
        goal.target_pose.pose.orientation.y = obj['orientation']["y"]
        goal.target_pose.pose.orientation.z = obj['orientation']["z"]
        goal.target_pose.pose.orientation.w = obj['orientation']["w"]
        self.move_base_actions.send_goal(goal)
        rospy.loginfo("Navigation goal published (offset 5 cm)")
        finished = self.move_base_actions.wait_for_result(rospy.Duration(60.0))
        if not finished:
            rospy.logwarn("Timed out achieving goal, canceling.")
            self.move_base_actions.cancel_goal()
            return False
        state = self.move_base_actions.get_state()
        rospy.loginfo("Goal finished with state: %d", state)
        return True
    
    def pick_object(self):
        rospy.loginfo("Picking up the object...")
        # Implement pick logic here
        result = self.pick_lift_object()
        if result.success:
            rospy.loginfo("Successfully picked up the object!")
            return True
        return False
    
    def reset_mapping(self):
        rospy.loginfo("Reset RTAB-Map...")
        try:
            self.reset_rtabmap()
            return True
        except rospy.ServiceException as e:
            rospy.logwarn(f"RTAB-Map reset failed: {e}")
            return False
    
    # Step 6: Place the object
    def place_object(self):
        rospy.loginfo("Placing the object in its designated location...")
        # Implement place logic here
        result = self.place_release_object()
        if result.success:
            rospy.loginfo("Successfully placed the object on table!")
            return True
        return False
    
    # A callback when the full mission is triggered
    def execute_mission(self, msg):
        # High-level method to execute the full pick and place mission
        response = TriggerResponse()
        response.success = False
        if self.exploration:
            rospy.logwarn("Mission is already active. Please wait till current mission finishes!")
            response.message = "Mission is already active. Please wait till current mission finishes!"
            return response
        # Step 1: initiate search
        self.start_search()
        # Wait till object is detected and tracked
        while not self.object_detected:
            rospy.loginfo("Looking for the object .....")
            rospy.sleep(0.5)
        # Step 2: stop searching
        self.stop_search()
        # Step 3: add object to scene
        if not self.add_to_scene("object"):
            response.message = "Failed to add object to its planning scene, mission is cancelled!"
            return response
        # Step 4: navigate to object
        if not self.navigate_to_object("object"):
            response.message = "Failed to navigate to object, mission is cancelled!"
            return response
        # Step 5: pick object
        if not self.pick_object():
            response.message = "Failed to pick object, mission is cancelled!"
            return response
        # Step 6: resume searching
        if not self.reset_mapping():
            response.message = "Failed to reset mapping, mission is cancelled!"
            return response
        self.start_search()
        # Wait till table is detected and tracked
        while not self.table_detected:
            rospy.loginfo("Looking for the placing table .....")
            rospy.sleep(0.5)
        # Step 8: stop searching
        self.stop_search()
        # Step 9: add table to scene
        if not self.add_to_scene("table"):
            response.message = "Failed to add table to the planning scene, mission is cancelled!"
            return response
        # Step 10: navigate to table
        if not self.navigate_to_object("table"):
            response.message = "Failed to approach table, mission is cancelled!"
            return response
        # Step 11: place object
        if not self.place_object():
            response.message = "Failed to place object on table, mission is cancelled!"
            return response
        response.success = True
        response.message = "Mission Accomplished!!"
        rospy.loginfo("Mission is completed Successfully!!!!")
        return response
    
    def run(self):
        # TODO: Create a ros service that triggers the full pick and place mission
        rospy.Service('run/mission', Trigger, self.execute_mission)
        rospy.loginfo("Pick and Place Manager is running...")
        rospy.spin()

if __name__ == '__main__':
    manager = PickPlaceManager()
    try:
        manager.run()
    except rospy.ROSInterruptException:
        pass