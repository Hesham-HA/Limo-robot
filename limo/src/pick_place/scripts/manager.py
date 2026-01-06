#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pick_place.srv import AddObjectToScene, AddObjectToSceneRequest
import actionlib
import dynamic_reconfigure.client
import math
import subprocess
import os
import time
import signal
from typing import Literal, Dict
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
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
        self.robot_pose = None
        self.object: Dict | None = None
        self.object_detection_counts = 0 # add counts to handle detection noise
        self.object_time_detected = 0.0
        self.table: Dict | None = None
        self.table_detection_counts = 0
        self.table_time_detected = 0.0
        self.cubes = {}
        self.labels = {}
        
        # Initialize the ROS node
        rospy.init_node('pick_place_manager')
        rospy.loginfo("Pick and Place Manager Node Initialized")
        
        # Movebase client
        self.move_base_actions = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_actions.wait_for_server()
        local_planner = rospy.get_param("~local_planner", 'move_base/DWAPlannerROS')
        self.move_base_configs = dynamic_reconfigure.client.Client(local_planner, timeout=5)
        self.local_planner_params = self.move_base_configs.get_configuration()
        
        # TF Buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Services
        rospy.wait_for_service("/rtabmap/reset")
        self.reset_rtabmap = rospy.ServiceProxy('/rtabmap/reset', Empty)
        rospy.wait_for_service("/rtabmap/pause")
        self.pause_rtabmap = rospy.ServiceProxy('/rtabmap/pause', Empty)
        rospy.wait_for_service("/rtabmap/resume")
        self.resume_rtabmap = rospy.ServiceProxy('/rtabmap/resume', Empty)
        rospy.wait_for_service("/clear_octomap")
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        rospy.wait_for_service("/add_object_to_scene")
        self.add_object = rospy.ServiceProxy("/add_object_to_scene", AddObjectToScene)
        rospy.wait_for_service("/pick_object")
        self.pick_lift_object = rospy.ServiceProxy("/pick_object", Trigger)
        rospy.wait_for_service("/place_object")
        self.place_release_object = rospy.ServiceProxy("/place_object", Trigger)
        
        # Subscribers "This intiates the callback when an object is detected"
        rospy.Subscriber("/object/detection", MarkerArray, self.object_detected_callback)
        
        # Publishers: velocity commands
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        rospy.loginfo("Pick & Place Manager ready")
    
    # start search
    def start_search(self):
        rospy.loginfo("Starting search!!!")
        if self.exploration:
            rospy.logwarn("Exploration is already running !!!")
            return
        cmd = ["roslaunch", "pick_place", "exploration.launch"]
        self.exploration = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
        time.sleep(2)
        rospy.loginfo("Search has started!")
    
    # stop search
    def stop_search(self):
        rospy.logwarn("Stopping search!!!!")
        # Stop and kill exploration launch
        rospy.loginfo("Killing exploration process")
        if self.exploration:
            os.killpg(os.getpgid(self.exploration.pid), signal.SIGINT)
            self.exploration.wait()
            self.exploration = None
        rospy.logwarn("Exploration process is killed!")
        # Cancel current move-base goal
        rospy.loginfo("Cancelling current move_base_goal")
        self.move_base_actions.cancel_all_goals()
        rospy.sleep(1.0)
        rospy.loginfo("Forcing robot stop...")
        for _ in range(100):
            self.stop_robot()
            rospy.sleep(0.1)
        self.move_base_actions.cancel_all_goals()
        rospy.logwarn("Goal cancelled and robot stopped!")
        # Pause rtabmapping
        self.pause_rtabmap()
        rospy.loginfo("SLAM is paused!!")
        rospy.sleep(1.0)
    
    # stop motion
    def stop_robot(self):
        self.cmd_vel.publish(Twist())
    
    # get current pose
    def get_robot_pose(self):
        try:
            # Wait for the transform to become available
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time.now(), rospy.Duration(2.0))
            self.robot_pose = PoseStamped()
            self.robot_pose.header.frame_id = "map"
            self.robot_pose.pose.position.x = trans.transform.translation.x
            self.robot_pose.pose.position.y = trans.transform.translation.y
            self.robot_pose.pose.position.z = trans.transform.translation.z
            self.robot_pose.pose.orientation = trans.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", e)
    
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
        obj_once_detected = self.object_detection_counts
        tab_once_detected = self.table_detection_counts
        for id_, label in self.labels.items():
            label_type = label["text"]
            if (id_-1) in self.cubes:
                if label_type == "redcube" or label_type.startswith("object"):
                    self.object = self.cubes[id_-1]
                    self.object_detection_counts += 1
                    if rospy.Time.now().to_sec() - self.object_time_detected > 3.0: # Two seconds span between consecutive detections
                        self.object_detection_counts = 1
                    self.object_time_detected = rospy.Time.now().to_sec()
                elif label_type == "yellowtable" or label_type.startswith("table"):
                    self.table = self.cubes[id_-1]
                    self.table_detection_counts += 1
                    if rospy.Time.now().to_sec() - self.table_time_detected > 3.0:  # Two seconds span between consecutive detections
                        self.table_detection_counts = 1
                    self.table_time_detected = rospy.Time.now().to_sec()
        if self.object_detection_counts and not obj_once_detected:
            rospy.loginfo(f"Object detected: {self.object}")
        if self.table_detection_counts and not tab_once_detected:
            rospy.loginfo(f"Table detected: {self.table}")
    
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
    def navigate_to_object(self, object: Literal["object", "table"], move_base: bool=True):
        if object == "object":
            rospy.loginfo("Navigating to detected object...")
            obj = self.object.copy()
        else:
            rospy.loginfo("Searching for the drop-off zone/table...")
            obj = self.table.copy()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        target_pose = PoseStamped()
        target_pose.header.frame_id = obj["frame_id"]
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = obj['position']["x"]
        target_pose.pose.position.y = obj['position']["y"]
        target_pose.pose.position.z = obj['position']["z"]
        target_pose.pose.orientation.x = obj['orientation']["x"]
        target_pose.pose.orientation.y = obj['orientation']["y"]
        target_pose.pose.orientation.z = obj['orientation']["z"]
        target_pose.pose.orientation.w = obj['orientation']["w"]
        self.get_robot_pose()
        if self.robot_pose is None:
            rospy.logerr("Failed to get current robot pose!")
            return False
        try:
            transform = self.tf_buffer.lookup_transform("map", obj["frame_id"], rospy.Time.now(), rospy.Duration(1.0))
            target_pose_map = tf2_geometry_msgs.do_transform_pose(target_pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform target pose: {e}")
            return False
        
        target_x = target_pose_map.pose.position.x
        target_y = target_pose_map.pose.position.y
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y
        dx = target_x - robot_x
        dy = target_y - robot_y
        d = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        q1, q2, q3, q4 = quaternion_from_euler(0, 0, heading)
        
        if move_base: # reach to an intermediate pose
            # Set very small goal tolerance
            params = {"xy_goal_tolerance": 0.04, "yaw_goal_tolerance": 0.01}
            try:
                self.move_base_configs.update_configuration(params)
                rospy.loginfo("Updated planner tolerances: xy=0.04 m, yaw=0.01 rad")
            except Exception as e:
                rospy.logwarn(f"Failed to update planner configuration: {e}")
            factor = (d - 0.6) / d
            target_x = robot_x + dx * factor
            target_y = robot_y + dy * factor
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = target_x
            goal.target_pose.pose.position.y = target_y
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = q1
            goal.target_pose.pose.orientation.y = q2
            goal.target_pose.pose.orientation.z = q3
            goal.target_pose.pose.orientation.w = q4
            self.move_base_actions.send_goal(goal)
            rospy.loginfo("Navigation goal published (offset 60 cm)")
            finished = self.move_base_actions.wait_for_result(rospy.Duration(60.0))
            if not finished:
                rospy.logwarn("Timed out achieving goal, canceling.")
                self.move_base_actions.cancel_goal()
                return False
            # Restore default goal tolerance
            params = {"xy_goal_tolerance": self.local_planner_params["xy_goal_tolerance"], "yaw_goal_tolerance": self.local_planner_params["yaw_goal_tolerance"]}
            try:
                self.move_base_configs.update_configuration(params)
                rospy.loginfo("Original planner tolerances are restored")
            except Exception as e:
                rospy.logwarn(f"Failed to restore planner configuration: {e}")
            state = self.move_base_actions.get_state()
            rospy.loginfo("Goal finished with state: %d", state)
            return True
        else: # approach to object in a straight line (approximately 32 cm distant)
            rospy.loginfo("Sending velocity commands directly towards target")
            d_traveled = 0
            cmd = Twist()
            cmd.linear.x = 0.02
            cmd.angular.z = 0.0
            self.cmd_vel.publish(cmd)
            while not rospy.is_shutdown():
                rospy.sleep(1)
                d_traveled += 1.1*0.03*1
                if d_traveled - d + 1.1*0.02*1 >= 0.06:
                    self.stop_robot()
                    rospy.loginfo("Reached target (cmd_vel).")
                    return True
            rospy.logwarn("Failed to reach target (cmd_vel).")
            return False
    
    def pick_object(self):
        rospy.loginfo("Picking up the object...")
        # Implement pick logic here
        result = self.pick_lift_object()
        if result.success:
            rospy.loginfo("Successfully picked up the object!")
            d_traveled = 0
            cmd = Twist()
            cmd.linear.x = -0.05
            cmd.angular.z = 0.0
            self.cmd_vel.publish(cmd)
            while not rospy.is_shutdown():
                rospy.sleep(1)
                d_traveled += 0.9*0.03*1
                if d_traveled > 0.6:
                    self.stop_robot()
                    rospy.loginfo("Stepped backwards (cmd_vel).")
                    return True
            rospy.logwarn("Failed to step backwards (cmd_vel)!")
            return True
        rospy.logerr("Failed to pick object!")
        return False
    
    def reset_mapping(self):
        rospy.loginfo("Reset RTAB-Map...")
        try:
            rospy.set_param("/rtabmap/rtabmap/Grid/Sensor", 0)
            rospy.set_param("/rtabmap/rtabmap/Grid/RangeMax", 3.0)
            rospy.wait_for_service("/rtabmap/update_parameters")
            update_params = rospy.ServiceProxy("/rtabmap/update_parameters", Empty)
            update_params()
            self.resume_rtabmap()
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
        self.object_detection_counts = 0
        self.table_detection_counts = 0
        rospy.loginfo("Starting full pick and place mission...")
        # Step 1: initiate search
        self.start_search()
        # Wait till object is detected and tracked
        while self.object_detection_counts < 3:
            rospy.loginfo("Looking for the object .....")
            rospy.sleep(0.5)
        rospy.loginfo("Object is found!")
        # Step 2: stop searching
        self.stop_search()
        # Step 3: add object to scene
        self.clear_octomap()
        self.navigate_to_object("object", move_base=True)
        if not self.add_to_scene("object"):
            response.message = "Failed to add object to its planning scene, mission is cancelled!"
            return response
        # Step 4: navigate to object
        if not self.navigate_to_object("object", move_base=False):
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
        while self.table_detection_counts < 3:
            rospy.loginfo("Looking for the placing table .....")
            rospy.sleep(0.5)
        rospy.loginfo("Table is found!")
        # Step 8: stop searching
        self.stop_search()
        # Step 9: add table to scene
        self.clear_octomap()
        self.navigate_to_object("table", move_base=True)
        if not self.add_to_scene("table"):
            response.message = "Failed to add table to the planning scene, mission is cancelled!"
            return response
        # Step 10: navigate to table
        if not self.navigate_to_object("table", move_base=False):
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