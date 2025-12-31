#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
import math
import tf

class PickPlaceManager:

    def __init__(self):
        rospy.init_node("pick_place_manager")

        # -------------------
        # State
        # -------------------
        self.explore_active = True
        self.object_detected = False
        self.object_pose = None

        # -------------------
        # Subscribers
        # -------------------
        self.obj_sub = rospy.Subscriber(
            "/detected_object_pose",
            PoseStamped,
            self.object_callback
        )

        # -------------------
        # Publishers
        # -------------------
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )

        self.cancel_pub = rospy.Publisher(
            "/move_base/cancel",
            GoalID,
            queue_size=1
        )

        # -------------------
        # Services (RTAB-Map)
        # -------------------
        rospy.wait_for_service("/rtabmap/pause")
        rospy.wait_for_service("/rtabmap/resume")

        self.pause_rtabmap = rospy.ServiceProxy("/rtabmap/pause", Empty)
        self.resume_rtabmap = rospy.ServiceProxy("/rtabmap/resume", Empty)

        # -------------------
        # TF Listener
        # -------------------
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Pick & Place Manager ready")

    # ==========================================
    # HARD STOP: stop exploration + pause RTAB-Map
    # ==========================================
    def hard_stop(self):
        rospy.loginfo("HARD STOP: Stopping exploration & pausing RTAB-Map")
        self.cancel_pub.publish(GoalID())
        try:
            self.pause_rtabmap()
        except rospy.ServiceException as e:
            rospy.logwarn(f"RTAB-Map pause failed: {e}")
        self.explore_active = False

    # ==========================================
    # Object detection callback
    # ==========================================
    def object_callback(self, msg):
        if self.object_detected:
            return

        rospy.loginfo("Object detected → executing HARD STOP")
        self.hard_stop()

        # Save object pose
        self.object_pose = msg
        self.object_detected = True

        # Navigate to object
        self.navigate_to_object()

        # Navigate to drop-off (5 cm before the object)
        self.navigate_to_dropoff(self.object_pose)

    # ==========================================
    # Navigate to detected object with offset
    # ==========================================
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

        offset = 0.35
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

    # ==========================================
    # Navigate to drop-off point (5 cm before object)
    # ==========================================
    def navigate_to_dropoff(self, object_pose):
        rospy.loginfo("Navigating to drop-off point near the object")

        try:
            # Transform object pose to map frame
            self.tf_listener.waitForTransform("map", object_pose.header.frame_id,
                                              rospy.Time(0), rospy.Duration(4.0))
            object_in_map = self.tf_listener.transformPose("map", object_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF transform failed: {e}")
            return

        # Compute vector from robot origin (assume robot at 0,0 in map frame) to object
        obj_x = object_in_map.pose.position.x
        obj_y = object_in_map.pose.position.y
        distance = math.sqrt(obj_x**2 + obj_y**2)

        offset = 0.05  # 5 cm before object
        if distance > offset:
            factor = (distance - offset) / distance
            drop_x = obj_x * factor
            drop_y = obj_y * factor
        else:
            drop_x = obj_x
            drop_y = obj_y

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = drop_x
        goal.pose.position.y = drop_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = object_in_map.pose.orientation

        self.goal_pub.publish(goal)
        rospy.loginfo("Drop-off navigation goal published (5 cm before object)")

    # ==========================================
    # Resume RTAB-Map (optional)
    # ==========================================
    def resume_mapping(self):
        rospy.loginfo("Resuming RTAB-Map...")
        try:
            self.resume_rtabmap()
        except rospy.ServiceException as e:
            rospy.logwarn(f"RTAB-Map resume failed: {e}")

    # ==========================================
    # Run ROS loop
    # ==========================================
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    PickPlaceManager().run()
