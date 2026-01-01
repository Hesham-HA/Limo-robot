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
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
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
        
        # TF Buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Services
        rospy.wait_for_service("/rtabmap/reset")
        self.reset_rtabmap = rospy.ServiceProxy('/rtabmap/reset', Empty)
        rospy.wait_for_service("/rtabmap/pause")
        self.pause_rtabmap = rospy.ServiceProxy('/rtabmap/pause', Empty)
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
        rospy.Subscriber("/odom", Odometry, self.pose_estimation)
        
        # Publishers: velocity commands
        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
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
        rospy.sleep(2.0)
        rospy.loginfo("Forcing robot stop...")
        for _ in range(10):
            self.stop_robot()
            rospy.sleep(0.1)
        rospy.logwarn("Goal cancelled and robot stopped!")
        # Pause rtabmapping
        self.pause_rtabmap()
        rospy.loginfo("SLAM is paused!!")
        rospy.sleep(1.0)
    
    # stop motion
    def stop_robot(self):
        self.cmd_vel.publish(Twist())
    
    # get current pose
    def pose_estimation(self, msg):
        self.robot_pose = msg.pose.pose
    
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
    
    def head_to_target(self, target_x, target_y, yaw_tolerance=0.05):
        """Rotate in place until robot faces (target_x, target_y).
        target_x/target_y must be expressed in the same frame as self.robot_pose.
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.logdebug("head_to_target loop")
            if self.robot_pose is None:
                rate.sleep()
                continue

            px = self.robot_pose.position.x
            py = self.robot_pose.position.y
            q = self.robot_pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            desired_yaw = math.atan2(target_y - py, target_x - px)
            err = self._angle_diff(desired_yaw, yaw)

            if abs(err) < yaw_tolerance:
                self.stop_robot()   # ensure robot is not drifting
                return True

            cmd = Twist()
            cmd.linear.x = 0.0  # rotate in place only
            # P controller on yaw error, saturate
            cmd.angular.z = max(-0.6, min(0.6, 1.5 * err))
            self.cmd_vel.publish(cmd)
            rate.sleep()

    def drive_to_target(self, target_x, target_y, fraction=1.0):
        """Drive toward the target (target expressed in same frame as self.robot_pose).
        fraction optionally moves toward a fractional point between current pose and target.
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.logdebug("drive_to_target loop")
            if self.robot_pose is None:
                rate.sleep()
                continue

            px = self.robot_pose.position.x
            py = self.robot_pose.position.y
            q = self.robot_pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # vector from robot to (possibly scaled) target
            dx = (target_x - px) * fraction
            dy = (target_y - py) * fraction
            dist = math.hypot(dx, dy)

            if dist < 0.01:
                self.stop_robot()
                return True

            # desired yaw in same frame as yaw
            desired_yaw = math.atan2(dy, dx)
            heading_err = self._angle_diff(desired_yaw, yaw)  # normalized to [-pi,pi]

            cmd = Twist()

            # If the heading error is large, rotate in place (no forward)
            heading_threshold = 0.25  # radians (~14 degrees). tune as needed
            if abs(heading_err) > heading_threshold:
                cmd.linear.x = 0.0
            else:
                # Move forward scaled by distance and reduced by heading error
                # Use cos(heading_err) to reduce forward when slightly off-angle
                linear_speed = 0.6 * dist
                linear_speed *= math.cos(heading_err)  # reduces when off-angle
                # clamp to [0, 0.15] (tuned smaller to be safer)
                cmd.linear.x = max(0.0, min(0.15, linear_speed))

            # angular controller
            cmd.angular.z = max(-0.6, min(0.6, 2.0 * heading_err))

            self.cmd_vel.publish(cmd)
            rate.sleep()

    
    @staticmethod
    def _angle_diff(a, b):
        """Normalize angle difference to [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d
    
    # Navigate to detected object with offset
    def navigate_to_object(self, object: Literal["object", "table"], move_base: bool=True, fraction = 1.0):
        if object == "object":
            rospy.loginfo("Navigating to detected object...")
            obj = self.object.copy()
        else:
            rospy.loginfo("Searching for the drop-off zone/table...")
            obj = self.table.copy()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        target_pose_map = PoseStamped()
        target_pose_map.header.frame_id = "map"
        target_pose_map.header.stamp = rospy.Time.now()
        target_pose_map.pose.position.x = obj['position']["x"]
        target_pose_map.pose.position.y = obj['position']["y"]
        target_pose_map.pose.orientation.w = 1.0  # Identity orientation for the point
        try:
            # transform map->odom to compute target in odom frame
            transform = self.tf_buffer.lookup_transform("odom", "map", rospy.Time(0), rospy.Duration(1.0))
            target_pose_odom = tf2_geometry_msgs.do_transform_pose(target_pose_map, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform target pose: {e}")
            return False

        raw_x = target_pose_odom.pose.position.x
        raw_y = target_pose_odom.pose.position.y
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        dx = raw_x - robot_x
        dy = raw_y - robot_y
        dist_to_target = math.hypot(dx, dy)
        offset = 0.04
        if dist_to_target > offset:
            ratio = (dist_to_target - offset) / dist_to_target
            target_x = robot_x + dx * ratio
            target_y = robot_y + dy * ratio
        else:
            target_x = raw_x
            target_y = raw_y

        if move_base:
            # IMPORTANT: target_x/target_y are in the odom frame, so set frame_id accordingly.
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"               # <-- was "map" (bug)
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = target_x
            goal.target_pose.pose.position.y = target_y
            goal.target_pose.pose.position.z = 0.0
            # if you want the orientation in map frame, you must transform it; for now we keep obj orientation (may be in map)
            goal.target_pose.pose.orientation.x = obj['orientation']["x"]
            goal.target_pose.pose.orientation.y = obj['orientation']["y"]
            goal.target_pose.pose.orientation.z = obj['orientation']["z"]
            goal.target_pose.pose.orientation.w = obj['orientation']["w"]

            self.move_base_actions.send_goal(goal)
            rospy.loginfo("Navigation goal published (offset 5 cm) in odom frame")
            finished = self.move_base_actions.wait_for_result(rospy.Duration(60.0))
            if not finished:
                rospy.logwarn("Timed out achieving goal, canceling.")
                self.move_base_actions.cancel_goal()
                return False
            state = self.move_base_actions.get_state()
            rospy.loginfo("Goal finished with state: %d", state)
            return True

        else:
            rospy.loginfo("Sending velocity commands towards target (odom frame)")
            if not self.head_to_target(target_x, target_y):
                rospy.logwarn("Failed to orient toward target")
                return False
            success = self.drive_to_target(target_x, target_y, fraction)
            if success:
                rospy.loginfo("Reached target (cmd_vel).")
            else:
                rospy.logwarn("Failed to reach target (cmd_vel).")
            return success
    
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
        self.clear_octomap()
        self.navigate_to_object("object", fraction=0.2)
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
        self.clear_octomap()
        self.navigate_to_object("table", fraction=0.2)
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