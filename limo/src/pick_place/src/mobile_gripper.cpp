#include "ros/ros.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>

class MobileGripper
{
private:
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;
  
  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  
  // Services
  ros::ServiceServer pick_service_;
  ros::ServiceServer place_service_;
  
  // Subscribers (for manual object mode)
  ros::Subscriber bbox_sub_;
  
  // Parameters
  bool use_scene_object_;  // true = use object from scene, false = use from bbox
  std::string object_name_;
  std::string table_name_;
  std::string zero_state_name_;
  std::string top_grip_state_name_;
  std::string front_grip_state_name_;
  std::string arm_group_name;
  std::string gripper_group_name;
  std::string grasp_joint_name;
  double bbox_scale_factor_;  // Scale down the bounding box for better object isolation
  
  // Manual object data
  geometry_msgs::PoseStamped manual_object_pose_;
  bool has_manual_object_;
  double manual_object_width_;
  double manual_object_height_;
  double manual_object_depth_;
  
  // Initial pose
  geometry_msgs::Pose initial_pose_;
  bool has_initial_pose_;
  
public:
  MobileGripper()
    : spinner_(1), has_manual_object_(false), has_initial_pose_(false)
  {
    spinner_.start();
    
    // Parameters
    nh_.param<bool>("use_scene_object", use_scene_object_, true);
    nh_.param<std::string>("object_name", object_name_, "detected_object");
    nh_.param<std::string>("table_name", table_name_, "detected_table");
    nh_.param<std::string>("zero_state_name", zero_state_name_, "zero");
    nh_.param<std::string>("top_grip_state_name", top_grip_state_name_, "top_grip");
    nh_.param<std::string>("front_grip_state_name", front_grip_state_name_, "front_grip");
    nh_.param<std::string>("arm_group", arm_group_name, "arm");
    nh_.param<std::string>("gripper_group", gripper_group_name, "gripper");
    nh_.param<std::string>("grasp_joint", grasp_joint_name, "grasping_frame_joint");
    nh_.param<double>("bbox_scale_factor", bbox_scale_factor_, 0.85); // Scale down manual object size for safety margin
    
    ROS_INFO("MobileGripper initialized");
    ROS_INFO("  - use_scene_object: %s", use_scene_object_ ? "true" : "false");
    ROS_INFO("  - object_name: %s", object_name_.c_str());
    ROS_INFO("  - table_name: %s", table_name_.c_str());
    ROS_INFO("  - zero_state_name: %s", zero_state_name_.c_str());
    
    ros::WallDuration(1.0).sleep();
    
    // Initialize MoveIt
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_group_name);
    arm_group_->setPlanningTime(45.0);
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(gripper_group_name);
    gripper_group_->setPlanningTime(5.0);
    
    // Store initial pose
    initial_pose_ = arm_group_->getCurrentPose().pose;
    has_initial_pose_ = true;
    ROS_INFO("Stored initial arm pose");
    
    // Services
    pick_service_ = nh_.advertiseService("pick_object", &MobileGripper::pickService, this);
    place_service_ = nh_.advertiseService("place_object", &MobileGripper::placeService, this);
    
    // Subscriber for manual mode
    if (!use_scene_object_)
    {
      bbox_sub_ = nh_.subscribe("/object_bounding_boxes", 1, &MobileGripper::bboxCallback, this);
      ROS_INFO("Subscribed to /object_bounding_boxes for manual object mode");
    }
    
    ROS_INFO("Services ready:");
    ROS_INFO("  - /pick_object");
    ROS_INFO("  - /place_object");
  }
  
  void bboxCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    if (use_scene_object_) return;  // Ignore if using scene object
    
    // Find the cube marker (bounding box)
    for (const auto& marker : msg->markers)
    {
      if (marker.type == visualization_msgs::Marker::CUBE)
      {
        manual_object_pose_.header = marker.header;
        manual_object_pose_.pose = marker.pose;
        manual_object_width_ = marker.scale.x;
        manual_object_height_ = marker.scale.z;
        manual_object_depth_ = marker.scale.y;
        has_manual_object_ = true;
        
        ROS_INFO_THROTTLE(2.0, "Updated manual object: [%.3f, %.3f, %.3f], size [%.3f x %.3f x %.3f]",
                 marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                 manual_object_width_, manual_object_depth_, manual_object_height_);
        break;
      }
    }
  }
  
  bool openGripper()
  {
    ROS_INFO("Opening gripper...");
    double opening_position = 0.00;  // Open position
    gripper_group_->setJointValueTarget(grasp_joint_name, opening_position);
    bool success = bool(gripper_group_->move());
    if (success)
    {
      ROS_INFO("Gripper opened successfully");
    }else{
      ROS_WARN("Failed to open gripper");
    }  
    return success;
  }
  
  bool closeGripper()
  {
    ROS_INFO("Closing gripper...");
    double closing_position = -0.01;  // Closed position
    gripper_group_->setJointValueTarget(grasp_joint_name, closing_position);
    bool success = bool(gripper_group_->move());
    if (success)
    {
      ROS_INFO("Gripper closed successfully");
    }else{
      ROS_WARN("Failed to close gripper");
    }
    return success;
  }

  bool pickService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    ROS_INFO("======== PICK SERVICE CALLED ========");
    geometry_msgs::PoseStamped object_pose;
    std::string frame_id;
    std::string wide_or_tall;
    if (use_scene_object_)
    {
      // Get object from planning scene
      auto objects = planning_scene_interface_->getObjects({object_name_});
      if (objects.empty())
      {
        res.success = false;
        res.message = "Object '" + object_name_ + "' not found in planning scene";
        ROS_ERROR_STREAM(res.message);
        return true;
      }
      const auto& obj = objects.begin()->second;
      // CRITICAL DEBUG: Print what we actually got
      ROS_INFO("=== RETRIEVED OBJECT ===");
      ROS_INFO("  Frame: %s", obj.header.frame_id.c_str());
      ROS_INFO("  Timestamp: %.3f", obj.header.stamp.toSec());
      ROS_INFO("  Position: [%.3f, %.3f, %.3f]",
        obj.primitive_poses[0].position.x,
        obj.primitive_poses[0].position.y,
        obj.primitive_poses[0].position.z
      );
      object_pose.header = obj.header;
      object_pose.pose = obj.primitive_poses[0];
      frame_id = obj.header.frame_id;
      // find out if the object is wide or tall
      auto& prim = obj.primitives[0];
      if (prim.type == shape_msgs::SolidPrimitive::CYLINDER)
      {
        double height = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
        double radius = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
        double diameter = 2.0 * radius;
        
        wide_or_tall = (height > diameter * 1.5) ? "tall" : "wide";
        
        ROS_INFO("CYLINDER detected: h=%.3fm, r=%.3fm (d=%.3fm) → %s",
                height, radius, diameter, wide_or_tall.c_str());
      }
      else if (prim.type == shape_msgs::SolidPrimitive::BOX)
      {
        double length = prim.dimensions[shape_msgs::SolidPrimitive::BOX_X];
        double width = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
        double height = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
        double max_base = std::max(length, width);
        
        wide_or_tall = (height > max_base * 1.5) ? "tall" : "wide";
        
        ROS_INFO("BOX detected: [%.3f x %.3f x %.3f] → %s",
                length, width, height, wide_or_tall.c_str());
      }
      else
      {
        ROS_WARN("Unknown primitive type: %d", prim.type);
        wide_or_tall = "tall";
      }
      
      ROS_INFO("Using object from planning scene (%s)", wide_or_tall.c_str());
    }
    else
    {
      // Use manual object from bbox
      if (!has_manual_object_)
      {
        res.success = false;
        res.message = "No manual object data available from bounding box";
        ROS_ERROR_STREAM(res.message);
        return true;
      }
      
      object_pose = manual_object_pose_;
      frame_id = manual_object_pose_.header.frame_id;
      
      // Add manual object to planning scene for collision checking and find out if it's wide or tall
      wide_or_tall = addManualObjectToScene();
      ROS_INFO("Using manual object from bounding box");
    }
    
    ROS_INFO_STREAM("Object pose: " << object_pose);
    
    // Execute pick and lift
    bool success = pickAndLift(object_pose, frame_id, wide_or_tall);
    
    if (success)
    {
      res.success = true;
      res.message = "Successfully picked and lifted object";
      ROS_INFO_STREAM(res.message);
    }
    else
    {
      res.success = false;
      res.message = "Failed to pick object";
      ROS_ERROR_STREAM(res.message);
    }
    
    ROS_INFO("======== PICK SERVICE DONE ========");
    return true;
  }
  
  bool placeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    ROS_INFO("======== PLACE SERVICE CALLED ========");
    
    // Get table from planning scene
    auto tables = planning_scene_interface_->getObjects({table_name_});
    if (tables.empty())
    {
      res.success = false;
      res.message = "Table '" + table_name_ + "' not found in planning scene. Add it first!";
      ROS_ERROR_STREAM(res.message);
      return true;
    }
    
    const auto& table = tables.begin()->second;
    geometry_msgs::PoseStamped table_pose;
    table_pose.header = table.header;
    table_pose.pose = table.primitive_poses[0];
    
    ROS_INFO_STREAM("Table pose: " << table_pose);
    
    // Calculate place pose on top of table
    geometry_msgs::PoseStamped place_pose = table_pose;
    
    // Place on top of table (table.z + table_height/2 + object_height/2 + clearance)
    double table_height = table.primitives[0].dimensions[2];  // BOX_Z
    place_pose.pose.position.z = table_pose.pose.position.z + table_height/2.0 + 0.05 + 0.05;
    
    // Optionally offset in X or Y to not place directly on center
    place_pose.pose.position.x += 0.1;  // 10cm offset
    
    ROS_INFO("Placing on table at: [%.3f, %.3f, %.3f]",
             place_pose.pose.position.x,
             place_pose.pose.position.y,
             place_pose.pose.position.z);
    
    // Execute place and release
    bool success = placeAndRelease(place_pose);
    
    if (success)
    {
      res.success = true;
      res.message = "Successfully placed and released object on table";
      ROS_INFO_STREAM(res.message);
    }
    else
    {
      res.success = false;
      res.message = "Failed to place object";
      ROS_ERROR_STREAM(res.message);
    }
    
    ROS_INFO("======== PLACE SERVICE DONE ========");
    return true;
  }
  
  bool pickAndLift(const geometry_msgs::PoseStamped& object_pose, const std::string& frame_id, const std::string& wide_or_tall)
  {
    ROS_INFO(">>> pickAndLift START");
    ROS_INFO("Object at: [%.3f, %.3f, %.3f] in frame '%s'",
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z,
            frame_id.c_str());
    
    // 1. Select appropriate initial pose
    ROS_INFO(">>> STEP 1: Selecting opening position (%s object)...", wide_or_tall.c_str());
    std::string initial_state = (wide_or_tall == "wide") ? front_grip_state_name_ : top_grip_state_name_;
    ROS_INFO("Chosen state: %s", initial_state.c_str());
    
    if (!returnToZero(initial_state)){
      ROS_WARN("Failed to move to opening position! Trying to plan with current pose...");
    } else {
      ROS_INFO("Moved to opening position");
    }
    
    // 2. Open gripper
    ROS_INFO(">>> STEP 2: Opening gripper...");
    if (!openGripper())
    {
      ROS_ERROR("Failed to open gripper");
      return false;
    }
    ROS_INFO("Gripper opened");
    
    // 3. Calculate grasp pose
    ROS_INFO(">>> STEP 3: Calculating grasp pose...");
    geometry_msgs::PoseStamped grasp_pose = object_pose;
    
    // IMPORTANT: Check if object pose is valid
    if (object_pose.pose.position.x == 0 && 
        object_pose.pose.position.y == 0 && 
        object_pose.pose.position.z == 0)
    {
      ROS_ERROR("Object pose is at origin (0,0,0)! Cannot grasp.");
      return false;
    }
    
    grasp_pose.pose.position.x -= 0.08;  // 8cm back from object
    grasp_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, M_PI/2, 0));
    
    ROS_INFO("Grasp pose: [%.3f, %.3f, %.3f]",
            grasp_pose.pose.position.x,
            grasp_pose.pose.position.y,
            grasp_pose.pose.position.z);
    
    // 4. Move to grasp pose
    ROS_INFO(">>> STEP 4: Planning to grasp pose...");
    arm_group_->setPoseReferenceFrame(frame_id);
    arm_group_->setPoseTarget(grasp_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = arm_group_->plan(plan);
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to plan to grasp pose (error: %d)", result.val);
      return false;
    }
    ROS_INFO("Plan found, executing...");
    
    if (arm_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Failed to execute grasp motion");
      return false;
    }
    ROS_INFO("Reached grasp pose");
    
    // 5. Close gripper
    ROS_INFO(">>> STEP 5: Closing gripper...");
    ros::Duration(0.5).sleep();
    if (!closeGripper())
    {
      ROS_ERROR("Failed to close gripper");
      return false;
    }
    ROS_INFO("Gripper closed");
    
    // 6. Attach object
    ROS_INFO(">>> STEP 6: Attaching object...");
    auto objects = planning_scene_interface_->getObjects({object_name_});
    if (!objects.empty())
    {
      const auto& obj = objects.begin()->second;
      moveit_msgs::AttachedCollisionObject aco;
      aco.link_name = "joint6_flange";
      aco.object = obj;
      aco.object.operation = aco.object.ADD;
      planning_scene_interface_->applyAttachedCollisionObject(aco);
      ROS_INFO("Object attached");
    } else {
      ROS_WARN("Object not found in scene, skipping attachment");
    }
    
    // 7. Lift object
    ROS_INFO(">>> STEP 7: Lifting object...");
    geometry_msgs::PoseStamped lift_pose = grasp_pose;
    lift_pose.pose.position.z += 0.15;
    
    arm_group_->setPoseTarget(lift_pose);
    if (!arm_group_->move())
    {
      ROS_ERROR("Failed to lift object");
      return false;
    }
    ROS_INFO("Object lifted");
    
    // 8. Return to home
    ROS_INFO(">>> STEP 8: Returning to home...");
    if (!returnToZero(zero_state_name_))
    {
      ROS_WARN("Failed to return to home, but object is picked");
    } else {
      ROS_INFO("Returned to home");
    }
    
    ROS_INFO(">>> pickAndLift COMPLETE");
    return true;
  }
  
  bool placeAndRelease(const geometry_msgs::PoseStamped& place_pose)
  {
    // 1. Move to place pose
    ROS_INFO("Moving to place pose...");
    arm_group_->setPoseReferenceFrame(place_pose.header.frame_id);
    arm_group_->setPoseTarget(place_pose);
    if (!arm_group_->move())
    {
      ROS_ERROR("Failed to move to place pose");
      return false;
    }
    // 2. Detach object
    ROS_INFO("Detaching object...");
    std::vector<std::string> object_ids = {object_name_};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    // 3. Open gripper
    ROS_INFO("Opening gripper...");
    if (!openGripper())
    {
      ROS_ERROR("Failed to open gripper");
      return false;
    }
    // 4. Move back slightly
    ROS_INFO("Moving back...");
    geometry_msgs::PoseStamped retreat_pose = place_pose;
    retreat_pose.pose.position.x -= 0.05;
    arm_group_->setPoseTarget(retreat_pose);
    arm_group_->move();
    // 5. Return to initial/home pose
    ROS_INFO("Returning to home position...");
    if (!returnToZero(zero_state_name_))
    {
      ROS_WARN("Failed to return to home, but object is placed");
    }
    return true;
  }
  
  bool returnToZero(const std::string state_name)
  {
    std::vector<std::string> named_targets = arm_group_->getNamedTargets();
    
    if (std::find(named_targets.begin(), named_targets.end(), state_name) != named_targets.end())
    {
      ROS_INFO(">>> Using named state: %s", state_name.c_str());
      arm_group_->setNamedTarget(state_name);
      
      // CHANGED: Plan first, then execute with error checking
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto result = arm_group_->plan(plan);
      
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR(">>> Failed to plan to state '%s' (error: %d)", state_name.c_str(), result.val);
        return false;
      }
      
      ROS_INFO(">>> Plan found, executing...");
      result = arm_group_->execute(plan);
      
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR(">>> Failed to execute to state '%s' (error: %d)", state_name.c_str(), result.val);
        return false;
      }
      
      ROS_INFO(">>> ✓ Reached state '%s'", state_name.c_str());
      return true;
    }
    else if (has_initial_pose_)
    {
      ROS_INFO(">>> Using stored initial pose");
      arm_group_->setPoseTarget(initial_pose_);
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto result = arm_group_->plan(plan);
      
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_ERROR(">>> Failed to plan to initial pose");
        return false;
      }
      
      result = arm_group_->execute(plan);
      return (result == moveit::core::MoveItErrorCode::SUCCESS);
    }
    else
    {
      ROS_WARN(">>> No state '%s' or initial pose available", state_name.c_str());
      ROS_INFO(">>> Available states:");
      for (const auto& target : named_targets)
      {
        ROS_INFO(">>>   - %s", target.c_str());
      }
      return false;
    }
  }
  
  std::string addManualObjectToScene()
  {
    if (!has_manual_object_) return "none";
    
    // Remove existing first
    std::vector<std::string> object_ids = {object_name_};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    ros::Duration(0.3).sleep();
    
    // Add new object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header = manual_object_pose_.header;
    collision_object.id = object_name_;
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = manual_object_width_*bbox_scale_factor_;
    primitive.dimensions[1] = manual_object_depth_*bbox_scale_factor_;
    primitive.dimensions[2] = manual_object_height_*bbox_scale_factor_;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(manual_object_pose_.pose);
    collision_object.operation = collision_object.ADD;
    
    planning_scene_interface_->applyCollisionObject(collision_object);
    
    ROS_INFO("Added manual object to planning scene");
    
    return primitive.dimensions[0] > primitive.dimensions[2] ? "wide" : "tall";
  }
  
  void spin()
  {
    ros::waitForShutdown();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mobile_gripper");
  MobileGripper gripper;
  gripper.spin();
  return 0;
}