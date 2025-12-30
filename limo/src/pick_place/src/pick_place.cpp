#include "pick_place/pick_place.h"

ObjectPickAndPlace::ObjectPickAndPlace()
  : spinner_(2), has_cloud_(false), has_bbox_(false), has_initial_pose_(false)
{
  spinner_.start();

  // Get node name for namespaced parameters
  std::string node_name = ros::this_node::getName();
  
  // Initialize parameters with default values
  object_name_ = nh_.param(node_name + "/object_name", std::string("detected_object"));
  table_name_ = nh_.param(node_name + "/table_name", std::string("detected_table"));
  zero_state_name_ = nh_.param(node_name + "/zero_state_name", std::string("zero"));
  top_grip_state_name_ = nh_.param(node_name + "/top_grip_state_name", std::string("top_grip"));
  front_grip_state_name_ = nh_.param(node_name + "/front_grip_state_name", std::string("front_grip"));
  arm_group_name = nh_.param(node_name + "/arm_group", std::string("arm"));
  gripper_group_name = nh_.param(node_name + "/gripper_group", std::string("gripper"));
  grasp_joint_name = nh_.param(node_name + "/grasp_joint", std::string("grasping_frame_joint"));
  grasp_link_name = nh_.param(node_name + "/grasp_link", std::string("joint6_flange"));
  bbox_scale_factor_ = nh_.param(node_name + "/bbox_scale_factor", 0.9);
  plane_distance_threshold_ = nh_.param(node_name + "/plane_distance_threshold", 0.015);
  cylinder_score_threshold_ = nh_.param(node_name + "/cylinder_score_threshold", 0.4);
  plane_max_iterations_ = nh_.param(node_name + "/plane_max_iterations", 500);
  plane_probability_ = nh_.param(node_name + "/plane_probability", 0.95);
  use_perception_ = nh_.param(node_name + "/use_perception", true);
  use_multiple_plane_removal_ = nh_.param(node_name + "/use_multiple_plane_removal", false);
  
  ROS_INFO("Pick-And-Place Server initialized");
  ROS_INFO("  - object_name: %s", object_name_.c_str());
  ROS_INFO("  - table_name: %s", table_name_.c_str());
  ROS_INFO("  - zero_state_name: %s", zero_state_name_.c_str());
  ROS_INFO("  - top_grip_state_name: %s", top_grip_state_name_.c_str());
  ROS_INFO("  - front_grip_state_name: %s", front_grip_state_name_.c_str());
  ROS_INFO("  - arm_group: %s", arm_group_name.c_str());
  ROS_INFO("  - gripper_group: %s", gripper_group_name.c_str());
  ROS_INFO("  - grasp_joint: %s", grasp_joint_name.c_str());
  ROS_INFO("  - grasp_link: %s", grasp_link_name.c_str());
  ROS_INFO("  - bbox_scale_factor: %.2f (smaller = tighter fit around object)", bbox_scale_factor_);
  ROS_INFO("  - plane_distance_threshold: %.3f m", plane_distance_threshold_);
  ROS_INFO("  - cylinder_score_threshold: %.2f", cylinder_score_threshold_);
  ROS_INFO("  - plane_max_iterations: %d", plane_max_iterations_);
  ROS_INFO("  - plane_probability: %.2f", plane_probability_);
  ROS_INFO("  - use_perception: %s", use_perception_ ? "true" : "false");
  ROS_INFO("  - use_multiple_plane_removal: %s", use_multiple_plane_removal_ ? "true" : "false");

  ros::WallDuration(1.0).sleep();

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(30.0));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  // Subscribers
  cloud_subscriber_ = nh_.subscribe("/camera/depth/points", 1, &ObjectPickAndPlace::cloudCB, this);
  bbox_subscriber_ = nh_.subscribe("/object_bounding_boxes", 1, &ObjectPickAndPlace::bboxCB, this);
  
  // Services
  add_object_service_ = nh_.advertiseService("add_object_to_scene", &ObjectPickAndPlace::addObjectService, this);
  pick_service_ = nh_.advertiseService("pick_object", &ObjectPickAndPlace::pickService, this);
  place_service_ = nh_.advertiseService("place_object", &ObjectPickAndPlace::placeService, this);

  ROS_INFO("Services ready:");
  ROS_INFO("  - /add_object_to_scene");
  ROS_INFO("  - /pick_object");
  ROS_INFO("  - /place_object");
}

void ObjectPickAndPlace::cloudCB(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  latest_cloud_ = msg;
  has_cloud_ = true;
}

void ObjectPickAndPlace::bboxCB(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  if (!msg->markers.empty())
  {
    latest_bboxes_ = *msg;
    has_bbox_ = true;
  }
}

bool ObjectPickAndPlace::addObjectService(pick_place::AddObjectToScene::Request& req, pick_place::AddObjectToScene::Response& res)
{
  ROS_INFO("========================================");
  ROS_INFO("Add object service called");
  // Run the actual MoveIt operations in a separate thread
  std::string result_message;
  auto future = std::async(std::launch::async, &ObjectPickAndPlace::addObjectAsync, this, req.is_table, std::ref(result_message));
  // Wait for completion
  bool success = future.get();
  res.success = success;
  res.message = result_message;
  if (success)
  {
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    ROS_ERROR_STREAM(res.message);
  }
  return true;
}

bool ObjectPickAndPlace::addObjectAsync(bool is_table, std::string& result_message)
{
  std::string object_name = is_table ? table_name_ : object_name_;
  ROS_INFO("========================================");
  ROS_INFO("Add object service called for %s", object_name.c_str());
  
  int max_retries = 5;
  double retry_delay = 0.5;
  int min_points_required = 30;

  removeExistingObject(object_name);
  
  for (int attempt = 1; attempt <= max_retries; attempt++)
  {
    ROS_INFO("Attempt %d/%d", attempt, max_retries);
    
    if (use_perception_ && !has_cloud_)
    {
      if (attempt < max_retries)
      {
        ROS_WARN("No point cloud data, waiting %.1fs...", retry_delay);
        ros::Duration(retry_delay).sleep();
        ros::spinOnce();
        continue;
      }
      result_message = "No point cloud data available after " + std::to_string(max_retries) + " attempts";
      return false;
    }
    
    if (!has_bbox_)
    {
      if (attempt < max_retries)
      {
        ROS_WARN("No bounding box data, waiting %.1fs...", retry_delay);
        ros::Duration(retry_delay).sleep();
        ros::spinOnce();
        continue;
      }
      result_message = "No bounding box data available after " + std::to_string(max_retries) + " attempts";
      return false;
    }
  
    try
    {
      // Get bounding box
      visualization_msgs::Marker bbox_marker;
      for (const auto& marker : latest_bboxes_.markers)
      {
        if (marker.type == visualization_msgs::Marker::CUBE)
        {
          bbox_marker = marker;
          break;
        }
      }
      if (bbox_marker.type != visualization_msgs::Marker::CUBE)
      {
        result_message = "No valid bounding box marker found";
        return false;
      }
      ROS_INFO("Bounding box center: [%.3f, %.3f, %.3f]", 
              bbox_marker.pose.position.x, bbox_marker.pose.position.y, bbox_marker.pose.position.z);
      ROS_INFO("Bounding box scale: [%.3f, %.3f, %.3f]", bbox_marker.scale.x, bbox_marker.scale.y, bbox_marker.scale.z);

      // Extract object info
      ObjectParams object_params;
      // Filter by cloud or add object from detection only
      if (use_perception_)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*latest_cloud_, *cloud);
        ROS_INFO("Original cloud has %zu points", cloud->points.size());
        // Filter by bbox
        filterPointCloudByBBox(cloud, bbox_marker);
        ROS_INFO("After bbox filtering: %zu points", cloud->points.size());
        if (cloud->points.empty())
        {
          result_message = "No points found in bounding box region";
          return false;
        }
        // Remove plane under object
        double bbox_bottom = bbox_marker.pose.position.z - bbox_marker.scale.z / 2.0;
        ROS_INFO("Bounding box bottom at Z=%.3fm", bbox_bottom);
        bool skip_plane_removal = false;
        if (bbox_bottom > 0.05)
        {
          ROS_INFO("Object is elevated (bottom > 5cm) - will skip aggressive plane removal");
          skip_plane_removal = true;
        }
        removeOutliers(cloud);
        ROS_INFO("After outlier removal: %zu points", cloud->points.size());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        computeNormals(cloud, cloud_normals);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        if (!skip_plane_removal && cloud->points.size() > 1000)
        {
          removePlaneSurface(cloud, inliers_plane);
          extractNormals(cloud_normals, inliers_plane);
          ROS_INFO("After plane removal: %zu points", cloud->points.size());
        }
        else
        {
          ROS_INFO("Skipping plane removal (elevated object or few points)");
        }
        // Check if enough points were filtered
        if (cloud->points.size() < min_points_required)
        {
          if (attempt < max_retries)
          {
            ROS_WARN("Too few points (%zu < %d), retrying...", cloud->points.size(), min_points_required);
            ros::Duration(retry_delay).sleep();
            ros::spinOnce();
            continue;
          }
          result_message = "Too few points remaining after plane removal (< " + 
                        std::to_string(min_points_required) + " points) after " +  std::to_string(max_retries) + " attempts";
          return false;
        }
        // Fit either a box or a cylinder
        double cylinder_score = 0.0;
        double box_score = 0.0;
        ObjectParams cylinder_params = fitCylinder(cloud, cloud_normals, cylinder_score);
        ObjectParams box_params = fitBox(cloud, box_score);
        ROS_INFO("Cylinder fit score: %.3f, Box fit score: %.3f", cylinder_score, box_score);
        ROS_INFO("Cylinder score threshold: %.3f", cylinder_score_threshold_);
        double height_to_width_ratio = bbox_marker.scale.z / std::max(bbox_marker.scale.x, bbox_marker.scale.y);
        ROS_INFO("Height-to-width ratio: %.2f (>1.5 suggests cylinder)", height_to_width_ratio);
        // Force use a box if the object is a table/plane
        if (is_table)
        {
          object_params = box_params;
          object_params.shape_type = "box";
          ROS_INFO("Forced BOX shape for table");
        }
        else
        {
          if (cylinder_score > cylinder_score_threshold_ && (cylinder_score > box_score * 0.9 || height_to_width_ratio > 1.5))
          {
            object_params = cylinder_params;
            object_params.shape_type = "cylinder";
            ROS_INFO("Selected CYLINDER shape (tall object detected)");
          }
          else if (box_score > 0.0)
          {
            object_params = box_params;
            object_params.shape_type = "box";
            ROS_INFO("Selected BOX shape");
          }
        }
      }
      else
      {
        object_params.shape_type = "box";
        object_params.box_dimensions[0] = bbox_marker.scale.x * bbox_scale_factor_;
        object_params.box_dimensions[1] = bbox_marker.scale.y;
        object_params.box_dimensions[2] = bbox_marker.scale.z;
        object_params.center_pt[0] = bbox_marker.pose.position.x;
        object_params.center_pt[1] = bbox_marker.pose.position.y;
        object_params.center_pt[2] = bbox_marker.pose.position.z;
      }

      // Add object to planning scene
      bool added = addObjectToPlanningScene(object_params, latest_cloud_->header.frame_id, object_name);
      if (added)
      {
        result_message = "Object successfully added to planning scene as " + 
                    object_params.shape_type + " (attempt " + std::to_string(attempt) + "/" + std::to_string(max_retries) + ")";
        return true;
      }
    }
    catch (const std::exception& e)
    {
      if (attempt < max_retries)
      {
        ROS_WARN("Exception on attempt %d: %s. Retrying...", attempt, e.what());
        ros::Duration(retry_delay).sleep();
        ros::spinOnce();
        continue;
      }
      result_message = std::string("Exception occurred after ") + 
                    std::to_string(max_retries) + " attempts: " + e.what();
      return false;
    }
  }
  // Failed after many attempts
  result_message = "Failed to add object after " + std::to_string(max_retries) + " attempts";
  return false;
}

bool ObjectPickAndPlace::pickService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("======== PICK SERVICE CALLED ========");
  // Run the actual MoveIt operations in a separate thread
  std::string result_message;
  auto future = std::async(std::launch::async, &ObjectPickAndPlace::pickAsync, this, std::ref(result_message));
  // Wait for completion
  bool success = future.get();
  res.success = success;
  res.message = result_message;
  if (success)
  {
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    ROS_ERROR_STREAM(res.message);
  }
  ROS_INFO("======== PICK SERVICE DONE ========");
  return true;
}

bool ObjectPickAndPlace::pickAsync(std::string& result_message)
{
  std::string planning_frame = arm_group_->getPlanningFrame();
  auto objects = planning_scene_interface_->getObjects({object_name_});
  std::map<std::string, geometry_msgs::Pose> object_poses = planning_scene_interface_->getObjectPoses({object_name_});
  geometry_msgs::PoseStamped object_pose;
  std::string wide_or_tall;
  if (object_poses.find(object_name_) == object_poses.end() or objects.empty())
  {
    result_message = "Object '" + object_name_ + "' not found in planning scene";
    return false;
  }
  const auto& obj = objects.begin()->second;
  ROS_INFO("=== RETRIEVED OBJECT ===");
  ROS_INFO("  Frame: %s", obj.header.frame_id.c_str());
  ROS_INFO("  Timestamp: %.3f", obj.header.stamp.toSec());
  ROS_INFO("  Position: [%.3f, %.3f, %.3f]",
    object_poses[object_name_].position.x,
    object_poses[object_name_].position.y,
    object_poses[object_name_].position.z
  );
  object_pose.header = obj.header;
  object_pose.header.stamp = ros::Time::now();
  object_pose.pose = object_poses[object_name_];
  
  auto& prim = obj.primitives[0];
  if (prim.type == shape_msgs::SolidPrimitive::CYLINDER)
  {
    double height = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
    double radius = prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
    double diameter = 2.0 * radius;
    wide_or_tall = (height > diameter) ? "tall" : "wide";
    ROS_INFO("CYLINDER detected: h=%.3fm, r=%.3fm (d=%.3fm) -> %s", height, radius, diameter, wide_or_tall.c_str());
  }
  else if (prim.type == shape_msgs::SolidPrimitive::BOX)
  {
    double length = prim.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    double width = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    double height = prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
    double max_base = std::max(length, width);
    wide_or_tall = (height > 1.5*max_base) ? "tall" : "wide";
    ROS_INFO("BOX detected: [%.3f x %.3f x %.3f] -> %s", length, width, height, wide_or_tall.c_str());
  }
  else
  {
    ROS_WARN("Unknown primitive type: %d", prim.type);
    wide_or_tall = "tall";
  }
  
  ROS_INFO("Using object from planning scene (%s)", wide_or_tall.c_str());
  ROS_INFO_STREAM("Object pose: " << object_pose);
  
  // Run the pickAndLift operation in a separate thread
  bool success = pickAndLift(object_pose, planning_frame, wide_or_tall);
  if (success)
  {
    result_message = "Successfully picked and lifted object";
    return true;
  }
  else
  {
    result_message = "Failed to pick object";
    return false;
  }
}

bool ObjectPickAndPlace::placeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("======== PLACE SERVICE CALLED ========");
  // Run the actual MoveIt operations in a separate thread
  std::string result_message;
  auto future = std::async(std::launch::async, &ObjectPickAndPlace::placeAsync, this, std::ref(result_message));
  // Wait for completion
  bool success = future.get();
  res.success = success;
  res.message = result_message;
  if (success)
  {
    ROS_INFO_STREAM(res.message);
  }
  else
  {
    ROS_ERROR_STREAM(res.message);
  }
  ROS_INFO("======== PLACE SERVICE DONE ========");
  return true;
}

bool ObjectPickAndPlace::placeAsync(std::string& result_message)
{
  std::string planning_frame = arm_group_->getPlanningFrame();
  auto tables = planning_scene_interface_->getObjects({table_name_});
  std::map<std::string, geometry_msgs::Pose> table_poses = planning_scene_interface_->getObjectPoses({table_name_});
  geometry_msgs::PoseStamped table_pose;
  if (table_poses.find(table_name_) == table_poses.end() or tables.empty())
  {
    result_message = "Table '" + table_name_ + "' not found in planning scene. Add it first!";
    return false;
  }
  const auto& table = tables.begin()->second;
  table_pose.header = table.header;
  table_pose.pose = table_poses[table_name_];
  ROS_INFO_STREAM("Table pose: " << table_pose);
  
  geometry_msgs::PoseStamped place_pose = table_pose;
  double table_height = table.primitives[0].dimensions[2];
  place_pose.pose.position.z = table_pose.pose.position.z + table_height/2.0 + 0.05 + 0.05;
  place_pose.pose.position.x += 0.1;
  
  ROS_INFO("Placing on table at: [%.3f, %.3f, %.3f]", place_pose.pose.position.x, place_pose.pose.position.y, place_pose.pose.position.z);
  bool success = placeAndRelease(place_pose);
  if (success)
  {
    result_message = "Successfully placed and released object on table";
    return true;
  }
  else
  {
    result_message = "Failed to place object";
    return false;
  }
}

void ObjectPickAndPlace::removeExistingObject(const std::string& object_name)
{
  // Note: originally a new planning_scene_interface is retrieved for this job
  auto objects = planning_scene_interface_->getObjects({object_name});
  if (!objects.empty())
  {
    ROS_INFO("Removing existing %s from planning scene", object_name.c_str());
    std::vector<std::string> object_ids = {object_name};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    ros::Duration(0.5).sleep();
    ROS_INFO("Cleaned up existing '%s' from planning scene", object_name.c_str());
  }
}

void ObjectPickAndPlace::filterPointCloudByBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const visualization_msgs::Marker& bbox)
{
  double cx = bbox.pose.position.x;
  double cy = bbox.pose.position.y;
  double cz = bbox.pose.position.z;
  double half_x = bbox.scale.y / 2.0 * bbox_scale_factor_;
  double half_y = bbox.scale.x / 2.0;
  double half_z = bbox.scale.z / 2.0;
  ROS_INFO("Filter box: center=[%.3f,%.3f,%.3f], half_size=[%.3f,%.3f,%.3f]", cx, cy, cz, half_x, half_y, half_z);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(cx - half_x, cx + half_x);
  pass.filter(*cloud);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(cy - half_y, cy + half_y);
  pass.filter(*cloud);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(std::max(0.0, cz - half_z), cz + half_z);
  pass.filter(*cloud);
}

void ObjectPickAndPlace::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud);
}

void ObjectPickAndPlace::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

void ObjectPickAndPlace::extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals, const pcl::PointIndices::Ptr& inliers_plane)
{
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

void ObjectPickAndPlace::removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane)
{
  size_t original_size = cloud->points.size();
  pcl::SACSegmentation<pcl::PointXYZ> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  segmentor.setMaxIterations(500);
  segmentor.setDistanceThreshold(0.01);
  segmentor.setProbability(0.99);
  segmentor.setInputCloud(cloud);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);
  size_t num_inliers = inliers_plane->indices.size();
  double plane_ratio = static_cast<double>(num_inliers) / original_size;
  ROS_INFO("Plane detection: %zu/%zu inliers (%.1f%%)", num_inliers, original_size, plane_ratio * 100.0);
  if (plane_ratio > 0.7)
  {
    ROS_WARN("Plane is %.1f%% of cloud - likely detecting object surface, not table!", plane_ratio * 100.0);
    ROS_WARN("Skipping plane removal to preserve object");
    inliers_plane->indices.clear();
    return;
  }
  if (coefficients_plane->values.size() >= 4)
  {
    Eigen::Vector3d plane_normal(
      coefficients_plane->values[0],
      coefficients_plane->values[1],
      coefficients_plane->values[2]
    );
    plane_normal.normalize();
    Eigen::Vector3d z_axis(0, 0, 1);
    double dot_product = std::abs(plane_normal.dot(z_axis));
    double angle_from_horizontal = acos(dot_product) * 180.0 / M_PI;
    ROS_INFO("Plane orientation: %.1f° from horizontal (normal: [%.2f, %.2f, %.2f])",
            angle_from_horizontal, plane_normal.x(), plane_normal.y(), plane_normal.z());
    if (angle_from_horizontal > 20.0)
    {
      ROS_WARN("Plane is tilted %.1f° - not a table! Keeping points.", angle_from_horizontal);
      inliers_plane->indices.clear();
      return;
    }
    double plane_z_avg = 0.0;
    double non_plane_z_avg = 0.0;
    size_t non_plane_count = 0;
    std::set<size_t> inlier_set(inliers_plane->indices.begin(), inliers_plane->indices.end());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      if (inlier_set.count(i))
      {
        plane_z_avg += cloud->points[i].z;
      }
      else
      {
        non_plane_z_avg += cloud->points[i].z;
        non_plane_count++;
      }
    }
    plane_z_avg /= num_inliers;
    if (non_plane_count > 0)
      non_plane_z_avg /= non_plane_count;
    ROS_INFO("Plane avg Z: %.3fm, Non-plane avg Z: %.3fm", plane_z_avg, non_plane_z_avg);
    if (plane_z_avg >= non_plane_z_avg - 0.02)
    {
      ROS_WARN("Plane is not below object - not removing it");
      inliers_plane->indices.clear();
      return;
    }
  }
  if (plane_ratio < 0.2)
  {
    ROS_WARN("Plane too small (%.1f%%), not removing", plane_ratio * 100.0);
    inliers_plane->indices.clear();
    return;
  }
  ROS_INFO("Removing table plane (%.1f%% of cloud)", plane_ratio * 100.0);
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
  ROS_INFO("After plane removal: %zu points (removed %zu)", cloud->points.size(), num_inliers);
}

ObjectParams ObjectPickAndPlace::fitCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals, double& score)
{
  ObjectParams params;
  score = 0.0;
  if (cloud->points.size() < 100)
  {
    ROS_WARN("Too few points (%zu) for reliable cylinder fitting", cloud->points.size());
    return params;
  }
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  double cloud_width = std::max(max_pt.x - min_pt.x, max_pt.y - min_pt.y);
  double estimated_radius = cloud_width / 2.0;
  double min_radius = std::max(0.015, estimated_radius * 0.5);
  double max_radius = std::min(0.150, estimated_radius * 2.0);
  ROS_INFO("Cylinder fitting: cloud_width=%.3fm, estimated_r=%.3fm, range=[%.3f, %.3f]", cloud_width, estimated_radius, min_radius, max_radius);
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  segmentor.setNormalDistanceWeight(0.05);
  segmentor.setMaxIterations(10000);
  segmentor.setDistanceThreshold(0.02);
  segmentor.setRadiusLimits(min_radius, max_radius);
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);
  segmentor.segment(*inliers_cylinder, *coefficients_cylinder);
  if (coefficients_cylinder->values.size() == 7 && !inliers_cylinder->indices.empty())
  {
    score = static_cast<double>(inliers_cylinder->indices.size()) / cloud->points.size();
    params.radius = coefficients_cylinder->values[6];
    params.direction_vec[0] = coefficients_cylinder->values[3];
    params.direction_vec[1] = coefficients_cylinder->values[4];
    params.direction_vec[2] = coefficients_cylinder->values[5];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cylinder_cloud);
    Eigen::Vector3d axis_dir(params.direction_vec[0], params.direction_vec[1], params.direction_vec[2]);
    axis_dir.normalize();
    double min_proj = std::numeric_limits<double>::max();
    double max_proj = std::numeric_limits<double>::lowest();
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& pt : cylinder_cloud->points)
    {
      Eigen::Vector3d point(pt.x, pt.y, pt.z);
      centroid += point;
      double proj = point.dot(axis_dir);
      min_proj = std::min(min_proj, proj);
      max_proj = std::max(max_proj, proj);
    }
    centroid /= cylinder_cloud->points.size();
    params.height = max_proj - min_proj;
    params.center_pt[0] = centroid.x();
    params.center_pt[1] = centroid.y();
    params.center_pt[2] = centroid.z();
    ROS_INFO("Cylinder: r=%.3fm, h=%.3fm, inliers=%zu/%zu (%.1f%%)",
             params.radius, params.height, inliers_cylinder->indices.size(), cloud->points.size(), score * 100.0);
  }
  else
  {
    ROS_WARN("Cylinder fitting failed: coeff_size=%zu, inliers=%zu (need >50)", coefficients_cylinder->values.size(), inliers_cylinder->indices.size());
  }
  return params;
}

ObjectParams ObjectPickAndPlace::fitBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double& score)
{
  ObjectParams params;
  score = 0.0;
  if (cloud->points.empty())
    return params;
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  params.box_dimensions[0] = max_pt.x - min_pt.x;
  params.box_dimensions[1] = max_pt.y - min_pt.y;
  params.box_dimensions[2] = max_pt.z - min_pt.z;
  params.center_pt[0] = (min_pt.x + max_pt.x) / 2.0;
  params.center_pt[1] = (min_pt.y + max_pt.y) / 2.0;
  params.center_pt[2] = (min_pt.z + max_pt.z) / 2.0;
  double box_volume = params.box_dimensions[0] * params.box_dimensions[1] * params.box_dimensions[2];
  if (box_volume > 0)
  {
    double point_density = cloud->points.size() / box_volume;
    score = std::min(1.0, point_density / 50000.0);
  }
  ROS_INFO("Box: [%.3f x %.3f x %.3f]m, points=%zu, score=%.3f",
           params.box_dimensions[0], params.box_dimensions[1], params.box_dimensions[2], cloud->points.size(), score);
  return params;
}

bool ObjectPickAndPlace::addObjectToPlanningScene(const ObjectParams& params, const std::string& cloud_frame, const std::string& object_name)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = arm_group_->getPlanningFrame();
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = object_name;
  shape_msgs::SolidPrimitive primitive;
  if (params.shape_type == "cylinder")
  {
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = params.height;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = params.radius;
    ROS_INFO("Scaled cylinder: h=%.3fm, r=%.3fm", primitive.dimensions[primitive.CYLINDER_HEIGHT], primitive.dimensions[primitive.CYLINDER_RADIUS]);
  }
  else
  {
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = params.box_dimensions[0];
    primitive.dimensions[primitive.BOX_Y] = params.box_dimensions[1];
    primitive.dimensions[primitive.BOX_Z] = params.box_dimensions[2];
    ROS_INFO("Scaled box: [%.3f x %.3f x %.3f]m", primitive.dimensions[primitive.BOX_X], primitive.dimensions[primitive.BOX_Y], primitive.dimensions[primitive.BOX_Z]);
  }
  // position in camera frame
  geometry_msgs::PoseStamped pose_cam;
  pose_cam.header.frame_id = cloud_frame;
  pose_cam.header.stamp = ros::Time::now();
  pose_cam.pose.position.x = params.center_pt[0];
  pose_cam.pose.position.y = params.center_pt[1];
  pose_cam.pose.position.z = params.center_pt[2];
  ROS_INFO("Object in camera frame '%s': [%.3f, %.3f, %.3f]",
          cloud_frame.c_str(), pose_cam.pose.position.x, pose_cam.pose.position.y, pose_cam.pose.position.z);
  if (params.shape_type == "cylinder")
  {
    Eigen::Vector3d z_cyl(params.direction_vec[0], params.direction_vec[1], params.direction_vec[2]);
    z_cyl.normalize();
    Eigen::Vector3d z_world(0.0, 0.0, 1.0);
    Eigen::Vector3d axis = z_world.cross(z_cyl);
    if (axis.norm() < 1e-6)
    {
      pose_cam.pose.orientation.w = 1.0;
      pose_cam.pose.orientation.x = 0.0;
      pose_cam.pose.orientation.y = 0.0;
      pose_cam.pose.orientation.z = 0.0;
    }
    else
    {
      axis.normalize();
      double angle = acos(z_world.dot(z_cyl));
      pose_cam.pose.orientation.x = axis.x() * sin(angle / 2.0);
      pose_cam.pose.orientation.y = axis.y() * sin(angle / 2.0);
      pose_cam.pose.orientation.z = axis.z() * sin(angle / 2.0);
      pose_cam.pose.orientation.w = cos(angle / 2.0);
    }
  }
  else
  {
    pose_cam.pose.orientation.w = 1.0;
    pose_cam.pose.orientation.x = 0.0;
    pose_cam.pose.orientation.y = 0.0;
    pose_cam.pose.orientation.z = 0.0;
  }
  // position in target frame_id
  geometry_msgs::PoseStamped pose_target;
  if (!tf_buffer_->canTransform(collision_object.header.frame_id, pose_cam.header.frame_id, ros::Time::now(), ros::Duration(2.0)))
  {
    ROS_ERROR("TF not available between %s and %s. Aborting addObjectToPlanningScene.",
              collision_object.header.frame_id.c_str(), pose_cam.header.frame_id.c_str());
    return false;
  }
  try
  {
    tf_buffer_->transform(pose_cam, pose_target, collision_object.header.frame_id, ros::Duration(2.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("TF transform failed: " << ex.what());
    return false;
  }
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose_target.pose);
  collision_object.operation = collision_object.ADD;
  ROS_INFO("Adding %s [%s] to planning scene at [%.3f, %.3f, %.3f] in %s frame",
          object_name.c_str(),
          params.shape_type.c_str(),
          pose_target.pose.position.x,
          pose_target.pose.position.y,
          pose_target.pose.position.z,
          collision_object.header.frame_id.c_str());
  bool success = planning_scene_interface_->applyCollisionObject(collision_object);
  if (!success)
  {
    ROS_ERROR("Failed to apply collision object to planning scene");
    return false;
  }
  ros::Duration(3.0).sleep();
  // Verification of added object
  ROS_INFO("Verifying added object...");
  std::map<std::string, geometry_msgs::Pose> object_poses = planning_scene_interface_->getObjectPoses({object_name});
  if (object_poses.find(object_name) != object_poses.end())
  {
    ROS_INFO("Object pose in planning frame: [%.3f, %.3f, %.3f]",
            object_poses[object_name].position.x,
            object_poses[object_name].position.y,
            object_poses[object_name].position.z);
  }
  else
  {
    ROS_WARN("Object '%s' not found in planning scene after adding", object_name.c_str());
  }
  return true;
}

bool ObjectPickAndPlace::openGripper()
{
  ROS_INFO("Opening gripper...");
  double opening_position = 0.00;
  gripper_group_->setJointValueTarget(grasp_joint_name, opening_position);
  bool success = bool(gripper_group_->move());
  if (success)
  {
    ROS_INFO("Gripper opened successfully");
  }
  else
  {
    ROS_WARN("Failed to open gripper");
  }  
  return success;
}

bool ObjectPickAndPlace::closeGripper()
{
  ROS_INFO("Closing gripper...");
  double closing_position = -0.01;
  gripper_group_->setJointValueTarget(grasp_joint_name, closing_position);
  bool success = bool(gripper_group_->move());
  if (success)
  {
    ROS_INFO("Gripper closed successfully");
  }
  else
  {
    ROS_WARN("Failed to close gripper");
  }
  return success;
}

bool ObjectPickAndPlace::pickAndLift(const geometry_msgs::PoseStamped& object_pose, const std::string& frame_id, const std::string& wide_or_tall)
{
  ROS_INFO(">>> pickAndLift START");
  arm_group_->setPoseReferenceFrame(frame_id);
  arm_group_->setPlanningTime(20.0);
  arm_group_->setNumPlanningAttempts(10);
  arm_group_->setGoalJointTolerance(0.01);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.02);
  ROS_INFO("Object at: [%.3f, %.3f, %.3f] in frame '%s'",
          object_pose.pose.position.x,
          object_pose.pose.position.y,
          object_pose.pose.position.z,
          frame_id.c_str());
  
  ROS_INFO(">>> STEP 1: Selecting opening position (%s object)...", wide_or_tall.c_str());
  std::string initial_state = (wide_or_tall == "wide") ? top_grip_state_name_ : front_grip_state_name_;
  ROS_INFO("Chosen state: %s", initial_state.c_str());
  if (!returnToZero(initial_state))
  {
    ROS_WARN("Failed to move to opening position! Trying to plan with current pose...");
  }
  else
  {
    ROS_INFO("Moved to opening position");
  }

  ROS_INFO(">>> STEP 2: Opening gripper...");
  if (!openGripper())
  {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  ROS_INFO("Gripper opened");
  
  ROS_INFO(">>> STEP 3: Calculating pre-grasp pose...");
  geometry_msgs::PoseStamped pre_grasp_pose = object_pose;
  if (object_pose.pose.position.x == 0 && 
      object_pose.pose.position.y == 0 && 
      object_pose.pose.position.z == 0)
  {
    ROS_ERROR("Object pose is at origin (0,0,0)! Cannot grasp.");
    return false;
  }
  tf2::Quaternion q;
  if (wide_or_tall == "tall") {
    q.setRPY(0, -M_PI/2, M_PI); // Pointing forward
  } else {
    q.setRPY(M_PI, 0, 0); // Pointing downward
  }
  pre_grasp_pose.pose.orientation = tf2::toMsg(q);
  if (wide_or_tall == "tall") {
    pre_grasp_pose.pose.position.x -= 0.1; // 6cm back from object
  } else {
    pre_grasp_pose.pose.position.z += 0.06; // 6cm above object
  }
  ROS_INFO("Pre-grasp pose: [%.2f, %.2f, %.2f] (%.2f, %.2f, %.2f)",
    pre_grasp_pose.pose.position.x, pre_grasp_pose.pose.position.y, pre_grasp_pose.pose.position.z,
    pre_grasp_pose.pose.orientation.x, pre_grasp_pose.pose.orientation.y, pre_grasp_pose.pose.orientation.z);
  
  ROS_INFO(">>> STEP 4: Planning to pre-grasp pose...");
  arm_group_->setStartStateToCurrentState();
  arm_group_->clearPoseTargets();
  arm_group_->setPoseTarget(pre_grasp_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = arm_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Failed to plan to pre-grasp pose (error: %d)", result.val);
    return false;
  }
  ROS_INFO("Plan to pre-grasp found, executing...");
  if (arm_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Failed to execute pre-grasp motion");
    return false;
  }

  // Now, perform a straight-line (Cartesian) path to the final grasp
  ROS_INFO(">>> STEP 5: Planning to grasp pose...");
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::PoseStamped grasp_pose = pre_grasp_pose;
  if (wide_or_tall == "tall") {
    grasp_pose.pose.position.x += 0.08; // Move forward into the object
  } else {
    grasp_pose.pose.position.z -= 0.04; // Move down onto the object
  }
  waypoints.push_back(grasp_pose.pose);
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = arm_group_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, false);
  if (fraction < 0.8)
  {
    ROS_ERROR("Failed to compute Cartesian path to grasp (fraction: %.2f)", fraction);
    return false;
  }
  ROS_INFO("Cartesian path found, executing...");
  if (arm_group_->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Failed to execute Cartesian grasp motion");
    return false;
  }
  ROS_INFO("Reached grasp pose");
  
  ROS_INFO(">>> STEP 6: Closing gripper...");
  ros::Duration(0.5).sleep();
  if (!closeGripper())
  {
    ROS_ERROR("Failed to close gripper");
    return false;
  }
  ROS_INFO("Gripper closed");
  
  ROS_INFO(">>> STEP 7: Attaching object...");
  auto objects = planning_scene_interface_->getObjects({object_name_});
  if (!objects.empty())
  {
    moveit_msgs::AttachedCollisionObject aco;
    aco.link_name = grasp_link_name;
    aco.object.id = object_name_;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface_->applyAttachedCollisionObject(aco);
    ROS_INFO("Object attached");
  }
  else
  {
    ROS_WARN("Object not found in scene, skipping attachment");
  }
  
  ROS_INFO(">>> STEP 8: Lifting object...");
  geometry_msgs::PoseStamped lift_pose = grasp_pose;
  lift_pose.pose.position.z += 0.15;
  arm_group_->setPoseTarget(lift_pose);
  if (!arm_group_->move())
  {
    ROS_ERROR("Failed to lift object");
    return false;
  }
  ROS_INFO("Object lifted");
  
  ROS_INFO(">>> STEP 9: Returning to home...");
  if (!returnToZero(zero_state_name_))
  {
    ROS_WARN("Failed to return to home, but object is picked");
  }
  else
  {
    ROS_INFO("Returned to home");
  }
  ROS_INFO(">>> pickAndLift COMPLETE");
  return true;
}

bool ObjectPickAndPlace::placeAndRelease(const geometry_msgs::PoseStamped& place_pose)
{
  ROS_INFO("Moving to place pose...");
  arm_group_->setPoseReferenceFrame(place_pose.header.frame_id);
  arm_group_->setPoseTarget(place_pose);
  if (!arm_group_->move())
  {
    ROS_ERROR("Failed to move to place pose");
    return false;
  }
  ROS_INFO("Detaching object...");
  std::vector<std::string> object_ids = {object_name_};
  planning_scene_interface_->removeCollisionObjects(object_ids);
  ROS_INFO("Opening gripper...");
  if (!openGripper())
  {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  ROS_INFO("Moving back...");
  geometry_msgs::PoseStamped retreat_pose = place_pose;
  retreat_pose.pose.position.x -= 0.05;
  arm_group_->setPoseTarget(retreat_pose);
  arm_group_->move();
  ROS_INFO("Returning to home position...");
  if (!returnToZero(zero_state_name_))
  {
    ROS_WARN("Failed to return to home, but object is placed");
  }
  return true;
}

bool ObjectPickAndPlace::returnToZero(const std::string state_name)
{
  std::vector<std::string> named_targets = arm_group_->getNamedTargets();
  arm_group_->setPlanningTime(10.0);  // Max 10 seconds planning time
  arm_group_->setNumPlanningAttempts(5);  // Try up to 5 times
  arm_group_->setGoalJointTolerance(0.015);
  arm_group_->setGoalPositionTolerance(0.01);
  arm_group_->setGoalOrientationTolerance(0.01);

  if (std::find(named_targets.begin(), named_targets.end(), state_name) != named_targets.end())
  {
    ROS_INFO("Using named state: %s", state_name.c_str());
    arm_group_->setNamedTarget(state_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_ERROR(">>> Failed to plan to state '%s'", state_name.c_str());
      return false;
    }
    ROS_INFO(">>> Plan found, executing...");
    
    moveit::core::MoveItErrorCode result = arm_group_->execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Successfully reached state '%s'", state_name.c_str());
      return true;
    }
    else
    {
      ROS_ERROR("Execution failed with code: %d", result.val);
      return false;
    }
  }
  else if (has_initial_pose_)
  {
    ROS_INFO("Using stored initial pose");
    arm_group_->setPoseTarget(initial_pose_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    arm_group_->execute(plan);
    return success;
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

void ObjectPickAndPlace::spin()
{
  ros::waitForShutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ObjectPickAndPlace service;
  ROS_INFO("Object Pick/Place is ready.");
  ROS_INFO("Call /add_object_to_scene when object/table is detected and visible.");
  ROS_INFO("Call /pick_object when object is added.");
  ROS_INFO("Call /place_object when object is attached and table is added.");
  service.spin();
  return 0;
}