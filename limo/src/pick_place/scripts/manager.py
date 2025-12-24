#!/usr/bin/env python3
import rospy

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
        # TODO: Define any useful attributes here
        self.exploare_active = True
        self.object_location = None
        
        # TODO: Define subscribers, publishers, and service clients here
        # e.g., self.object_detection_sub = rospy.Subscriber(...)
        
        # Initialize the ROS node
        rospy.init_node('pick_place_manager')
        rospy.loginfo("Pick and Place Manager Node Initialized")
    
    # TODO: Step 1: Wait for search results
    def search_for_objects(self):
        rospy.loginfo("Searching for objects...")
        # Implement search logic here
        # TODO: subscribe to the object detection results from the object search (object_extractor.py)
        #
        # TODO: once an object is detected, store its location, break the object search and proceed to navigation
        # You should run a service or something to stop the explore_lite node
        # You should also somehow stop the SLAM of rtabmap (or pause it if possible)
        self.exploare_active = False
        pass
    
    # TODO: Step 2: Navigate to object
    def navigate_to_object(self, object_location):
        rospy.loginfo(f"Navigating to object at {object_location}...")
        # Implement navigation logic here
        # TODO: the robot should be at an optimal distance for both manipulation and perception (roughly: 0.35 meters)
        #
        # TODO: use move_base, publish goals to /move_base_simple/goal
        pass
    
    # TODO: Step 3: Pick the object
    def pick_object(self):
        rospy.loginfo("Picking up the object...")
        # Implement pick logic here
        # TODO: run the add object service to add the detected object to the planning scene (i.e rosservice call /add_object_to_scene "is_table: false")
        #
        # TODO: run the pick service to pick the object (i.e rosservice call /pick_object)
        pass
    
    # TODO: Step 4: Search for the drop-off table
    def navigate_to_dropoff(self):
        rospy.loginfo("Searching for the drop-off zone/table...")
        # Implement navigation logic here
        self.exploare_active = True
        # TODO: run the explore_lite node again to search for the drop-off table, resume SLAM if it was paused and reset its map
        # 
        # TODO: once the table is detected, store its location, break the table search and proceed to navigation
        self.exploare_active = False
        pass
    
    # TODO: Step 5: Navigate to drop-off table
    def navigate_to_table(self, table_location):
        rospy.loginfo(f"Apporaching the drop-off table at {table_location}...")
        # Implement navigation logic here
        # TODO: the robot should be at an optimal distance for both manipulation and perception (roughly: 0.35 meters)
        #
        # TODO: use move_base, publish goals to /move_base_simple/goal
        pass
    
    # TODO: Step 6: Place the object
    def place_object(self):
        rospy.loginfo("Placing the object in its designated location...")
        # Implement place logic here
        # TODO: run the place service to place the object (i.e rosservice call /place_object)
        pass

    # A callback when the full mission is triggered
    def execute_mission(self):
        # High-level method to execute the full pick and place mission
        if not self.exploare_active:
            rospy.logwarn("Exploration is not active. A new exploration is triggered!")
            # TODO: run the explore_lite node again to search for objects
            self.exploare_active = True
        self.search_for_objects()
        if self.object_location:
            self.navigate_to_object(self.object_location)
            self.pick_object()
            self.navigate_to_dropoff()
            if self.object_location:  # Assuming we reuse object_location for table location
                self.navigate_to_table(self.object_location)
                self.place_object()
    
    def run(self):
        # TODO: Create a ros service that triggers the full pick and place mission
        rospy.loginfo("Pick and Place Manager is running...")
        rospy.spin()

if __name__ == '__main__':
    manager = PickPlaceManager()
    try:
        manager.run()
    except rospy.ROSInterruptException:
        pass