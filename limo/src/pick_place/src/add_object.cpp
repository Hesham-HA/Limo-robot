#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pick_place/AddObjectToScene.h>

class ObjectSegmentService
{
private:
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  
  // Subscribers
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber bbox_subscriber_;
  
  // Service
  ros::ServiceServer add_object_service_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Stored data
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  visualization_msgs::MarkerArray latest_bboxes_;
  bool has_cloud_;
  bool has_bbox_;
  
  // Parameters
  double bbox_scale_factor_;  // Scale down the bounding box for better object isolation
  double plane_distance_threshold_;
  double cylinder_score_threshold_;
  int plane_max_iterations_;
  double plane_probability_;
  bool use_multiple_plane_removal_;
  double object_scale_factor_;
  
  // Object shape parameters
  struct ObjectParams
  {
    std::string shape_type; // "box" or "cylinder"
    
    // Box parameters
    double box_dimensions[3]; // length, width, height
    
    // Cylinder parameters
    double radius;
    double height;
    double direction_vec[3];
    
    // Common parameters
    double center_pt[3];
  };
  
public:
  ObjectSegmentService()
    : has_cloud_(false), has_bbox_(false)
  {
    // Initialize parameters
    nh_.param("bbox_scale_factor", bbox_scale_factor_, 0.9);  // Use only 90% of bbox
    nh_.param("plane_distance_threshold", plane_distance_threshold_, 0.015);
    nh_.param("cylinder_score_threshold", cylinder_score_threshold_, 0.4);
    nh_.param("plane_max_iterations", plane_max_iterations_, 500);  // Max RANSAC iterations for plane removal
    nh_.param("plane_probability", plane_probability_, 0.95);  // Add probability threshold
    nh_.param("use_multiple_plane_removal", use_multiple_plane_removal_, false);  // Remove planes iteratively
    nh_.param("object_scale_factor", object_scale_factor_, 0.85); // Scale down object size for safety margin
    
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(30.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers
    cloud_subscriber_ = nh_.subscribe("/camera/depth/points", 1, &ObjectSegmentService::cloudCB, this);
    bbox_subscriber_ = nh_.subscribe("/object_bounding_boxes", 1, &ObjectSegmentService::bboxCB, this);
    
    // Service
    add_object_service_ = nh_.advertiseService("add_object_to_scene", &ObjectSegmentService::addObjectService, this);
    
    ROS_INFO("Object Segment Service initialized");
    ROS_INFO("  - bbox_scale_factor: %.2f (smaller = tighter fit around object)", bbox_scale_factor_);
    ROS_INFO("  - plane_distance_threshold: %.3f m", plane_distance_threshold_);
    ROS_INFO("  - cylinder_score_threshold: %.2f", cylinder_score_threshold_);
    ROS_INFO("Waiting for point cloud on /camera/depth/points");
    ROS_INFO("Waiting for bounding boxes on /object_bounding_boxes");
    ROS_INFO("Service available at /add_object_to_scene");
  }
  
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    latest_cloud_ = msg;
    has_cloud_ = true;
  }
  
  void bboxCB(const visualization_msgs::MarkerArray::ConstPtr& msg)
  {
    if (!msg->markers.empty())
    {
      latest_bboxes_ = *msg;
      has_bbox_ = true;
    }
  }

  bool addObjectService(pick_place::AddObjectToScene::Request& req, pick_place::AddObjectToScene::Response& res)
  {
    std::string object_name = req.is_table ? "detected_table" : "detected_object";
    ROS_INFO("========================================");
    ROS_INFO("Add object service called for %s", object_name.c_str());
    ROS_INFO("========================================");
    
    // NEW: Add retry parameters
    int max_retries = 5;  // Try up to 5 times
    double retry_delay = 0.5;  // Wait 0.5 seconds between retries
    int min_points_required = 30;  // Reduce from 50 to 30

    // Remove existing object with same name from planning scene
    removeExistingObject(object_name);
    
    // NEW: Retry loop
    for (int attempt = 1; attempt <= max_retries; attempt++)
    {
      ROS_INFO("Attempt %d/%d", attempt, max_retries);
      
      // Check if we have necessary data
      if (!has_cloud_)
      {
        if (attempt < max_retries)
        {
          ROS_WARN("No point cloud data, waiting %.1fs...", retry_delay);
          ros::Duration(retry_delay).sleep();
          ros::spinOnce();  // Process callbacks
          continue;
        }
        res.success = false;
        res.message = "No point cloud data available after " + std::to_string(max_retries) + " attempts";
        ROS_ERROR_STREAM(res.message);
        return true;
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
        res.success = false;
        res.message = "No bounding box data available after " + std::to_string(max_retries) + " attempts";
        ROS_ERROR_STREAM(res.message);
        return true;
      }
    
      // Process the object
      try
      {
        // Get the bounding box information (assuming only 1 object)
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
          res.success = false;
          res.message = "No valid bounding box marker found";
          ROS_ERROR_STREAM(res.message);
          return true;
        }
        
        ROS_INFO("Bounding box center: [%.3f, %.3f, %.3f]", 
                bbox_marker.pose.position.x,
                bbox_marker.pose.position.y,
                bbox_marker.pose.position.z);
        ROS_INFO("Bounding box scale: [%.3f, %.3f, %.3f]",
                bbox_marker.scale.x, bbox_marker.scale.y, bbox_marker.scale.z);
        
        // Convert point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*latest_cloud_, *cloud);
        ROS_INFO("Original cloud has %zu points", cloud->points.size());
        
        // Filter point cloud based on bounding box region (TIGHTER FIT)
        filterPointCloudByBBox(cloud, bbox_marker);
        ROS_INFO("After bbox filtering: %zu points", cloud->points.size());
        
        if (cloud->points.empty())
        {
          res.success = false;
          res.message = "No points found in bounding box region";
          ROS_ERROR_STREAM(res.message);
          return true;
        }
        
        // Remove statistical outliers
        removeOutliers(cloud);
        ROS_INFO("After outlier removal: %zu points", cloud->points.size());
        
        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        computeNormals(cloud, cloud_normals);
        
        // Remove plane surface (table)
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        removePlaneSurface(cloud, inliers_plane);
        extractNormals(cloud_normals, inliers_plane);
        ROS_INFO("After plane removal: %zu points", cloud->points.size());
        
        // Minimum points for reliable fitting
        if (cloud->points.size() < min_points_required)
        {
          if (attempt < max_retries)
          {
            ROS_WARN("Too few points (%zu < %d), retrying in %.1fs...", 
                    cloud->points.size(), min_points_required, retry_delay);
            ros::Duration(retry_delay).sleep();
            ros::spinOnce();
            continue;  // Try again
          }
          res.success = false;
          res.message = "Too few points remaining after plane removal (< " + 
                        std::to_string(min_points_required) + " points) after " + 
                        std::to_string(max_retries) + " attempts";
          ROS_ERROR_STREAM(res.message);
          return true;
        }
        
        // Try to fit both cylinder and box, choose the best fit
        ObjectParams object_params;
        double cylinder_score = 0.0;
        double box_score = 0.0;
        
        ObjectParams cylinder_params = fitCylinder(cloud, cloud_normals, cylinder_score);
        ObjectParams box_params = fitBox(cloud, box_score);
        
        ROS_INFO("Cylinder fit score: %.3f, Box fit score: %.3f", cylinder_score, box_score);
        ROS_INFO("Cylinder score threshold: %.3f", cylinder_score_threshold_);
        
        // PREFER CYLINDER for tall narrow objects (like cans)
        // Check aspect ratio to help decide
        double height_to_width_ratio = bbox_marker.scale.z / std::max(bbox_marker.scale.x, bbox_marker.scale.y);
        
        ROS_INFO("Height-to-width ratio: %.2f (>1.5 suggests cylinder)", height_to_width_ratio);
        
        // Decision logic: prefer cylinder if score is reasonable AND object is tall
        if (req.is_table)
        {
          // Force box fitting for tables
          object_params = box_params;
          object_params.shape_type = "box";
          ROS_INFO("✓ Forced BOX shape for table");
        }
        else
        {
          if (cylinder_score > cylinder_score_threshold_ && 
              (cylinder_score > box_score * 0.8 || height_to_width_ratio > 1.5))
          {
            object_params = cylinder_params;
            object_params.shape_type = "cylinder";
            ROS_INFO("✓ Selected CYLINDER shape (tall object detected)");
          }
          else if (box_score > 0.0)
          {
            object_params = box_params;
            object_params.shape_type = "box";
            ROS_INFO("✓ Selected BOX shape");
          }
        }
        
        // Add the object to the planning scene
        bool added = addObjectToPlanningScene(object_params, latest_cloud_->header.frame_id, object_name);
        
        if (added)
        {
          res.success = true;
          res.message = "Object successfully added to planning scene as " + 
                      object_params.shape_type + " (attempt " + std::to_string(attempt) + "/" + 
                      std::to_string(max_retries) + ")";
          // ... (add dimension info) ...
          ROS_INFO_STREAM(res.message);
          return true;  // Success!
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
        res.success = false;
        res.message = std::string("Exception occurred after ") + 
                      std::to_string(max_retries) + " attempts: " + e.what();
        ROS_ERROR_STREAM(res.message);
        return true;
      }
    }
    // If we exhausted all retries
    res.success = false;
    res.message = "Failed to add object after " + std::to_string(max_retries) + " attempts";
    ROS_ERROR_STREAM(res.message);
    
    ROS_INFO("========================================");
    return true;
  }
  
  void removeExistingObject(const std::string& object_name)
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // Check if object exists
    auto objects = planning_scene_interface.getObjects({object_name});
    if (!objects.empty())
    {
      ROS_INFO("Removing existing %s from planning scene", object_name.c_str());
      std::vector<std::string> object_ids = {object_name};
      planning_scene_interface.removeCollisionObjects(object_ids);
      ros::Duration(0.5).sleep();  // Give time for removal to process
    }
  }

  void filterPointCloudByBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const visualization_msgs::Marker& bbox)
  {
    // Extract bounding box center and dimensions
    double cx = bbox.pose.position.x;
    double cy = bbox.pose.position.y;
    double cz = bbox.pose.position.z;
    
    // TIGHTER fit - use bbox_scale_factor_ (default 0.9 = 90% of original bbox)
    double half_x = bbox.scale.x / 2.0; // * bbox_scale_factor_;
    double half_y = bbox.scale.y / 2.0; // * bbox_scale_factor_;
    double half_z = bbox.scale.z / 2.0; // * bbox_scale_factor_;
    
    ROS_INFO("Filter box: center=[%.3f,%.3f,%.3f], half_size=[%.3f,%.3f,%.3f]", cx, cy, cz, half_x, half_y, half_z);
    
    // Apply passthrough filters
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
  
  void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
  }
  
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }
  
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                     const pcl::PointIndices::Ptr& inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }
  
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const pcl::PointIndices::Ptr& inliers_plane)
  {
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(plane_max_iterations_);  // CHANGED: Use parameter
    segmentor.setDistanceThreshold(plane_distance_threshold_);
    segmentor.setProbability(plane_probability_);  // NEW: Add probability threshold
    segmentor.setInputCloud(cloud);
    
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    
    // NEW: Only remove plane if it's actually large (likely the table)
    double plane_ratio = static_cast<double>(inliers_plane->indices.size()) / cloud->points.size();
    ROS_INFO("Plane has %zu inliers (%.1f%% of cloud)", inliers_plane->indices.size(), plane_ratio * 100.0);
    
    // NEW: Only remove if plane is significant (> 30% of points)
    if (plane_ratio < 0.3)
    {
      ROS_WARN("Plane is too small (%.1f%%), keeping all points", plane_ratio * 100.0);
      inliers_plane->indices.clear();  // Don't remove anything
      return;
    }
    
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
  }
  
  ObjectParams fitCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                          double& score)
  {
    ObjectParams params;
    score = 0.0;
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setNormalDistanceWeight(0.1);
    segmentor.setMaxIterations(10000);  // More iterations for better fit
    segmentor.setDistanceThreshold(0.01);
    segmentor.setRadiusLimits(0.01, 0.08);  // Typical can: 2-8cm radius
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);
    
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);
    
    if (coefficients_cylinder->values.size() == 7 && !inliers_cylinder->indices.empty())
    {
      // Calculate score based on inlier ratio
      score = static_cast<double>(inliers_cylinder->indices.size()) / cloud->points.size();
      
      // Extract cylinder parameters
      params.radius = coefficients_cylinder->values[6];
      params.direction_vec[0] = coefficients_cylinder->values[3];
      params.direction_vec[1] = coefficients_cylinder->values[4];
      params.direction_vec[2] = coefficients_cylinder->values[5];
      
      // Estimate height and center from inlier points
      pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers_cylinder);
      extract.setNegative(false);
      extract.filter(*cylinder_cloud);
      
      // Get min/max along cylinder axis
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
      
      // Center point
      params.center_pt[0] = centroid.x();
      params.center_pt[1] = centroid.y();
      params.center_pt[2] = centroid.z();
      
      ROS_INFO("Cylinder: r=%.3fm, h=%.3fm, inliers=%zu/%zu (%.1f%%)",
               params.radius, params.height, 
               inliers_cylinder->indices.size(), cloud->points.size(),
               score * 100.0);
    }
    else
    {
      ROS_WARN("Cylinder fitting failed");
    }
    
    return params;
  }
  
  ObjectParams fitBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double& score)
  {
    ObjectParams params;
    score = 0.0;
    
    if (cloud->points.empty())
      return params;
    
    // Compute axis-aligned bounding box
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    // Calculate dimensions
    params.box_dimensions[0] = max_pt.x - min_pt.x; // length
    params.box_dimensions[1] = max_pt.y - min_pt.y; // width
    params.box_dimensions[2] = max_pt.z - min_pt.z; // height
    
    // Calculate center
    params.center_pt[0] = (min_pt.x + max_pt.x) / 2.0;
    params.center_pt[1] = (min_pt.y + max_pt.y) / 2.0;
    params.center_pt[2] = (min_pt.z + max_pt.z) / 2.0;
    
    // Calculate score based on volume occupancy
    double box_volume = params.box_dimensions[0] * params.box_dimensions[1] * params.box_dimensions[2];
    if (box_volume > 0)
    {
      double point_density = cloud->points.size() / box_volume;
      score = std::min(1.0, point_density / 50000.0); // Normalize
    }
    
    ROS_INFO("Box: [%.3f x %.3f x %.3f]m, points=%zu, score=%.3f",
             params.box_dimensions[0], params.box_dimensions[1], 
             params.box_dimensions[2], cloud->points.size(), score);
    
    return params;
  }
  
  bool addObjectToPlanningScene(const ObjectParams& params, const std::string& cloud_frame, const std::string& object_name)
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "odom";
    collision_object.id = object_name;
    
    // Create primitive shape
    shape_msgs::SolidPrimitive primitive;
    
    if (params.shape_type == "cylinder")
    {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      // NEW: Scale down the dimensions
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = params.height; // / object_scale_factor;
      primitive.dimensions[primitive.CYLINDER_RADIUS] = params.radius; // * object_scale_factor_;
      
      ROS_INFO("Scaled cylinder: h=%.3fm (%.3f%%), r=%.3fm (%.3f%%)",
              primitive.dimensions[primitive.CYLINDER_HEIGHT],
              1 * 100.0,
              primitive.dimensions[primitive.CYLINDER_RADIUS],
              object_scale_factor_ * 100.0);
    }
    else // box
    {
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      // NEW: Scale down the dimensions
      primitive.dimensions[primitive.BOX_X] = params.box_dimensions[0]; // * object_scale_factor_;
      primitive.dimensions[primitive.BOX_Y] = params.box_dimensions[1]; // * object_scale_factor_;
      primitive.dimensions[primitive.BOX_Z] = params.box_dimensions[2];
      
      ROS_INFO("Scaled box: [%.3f x %.3f x %.3f]m (%.3f%%)",
              primitive.dimensions[primitive.BOX_X],
              primitive.dimensions[primitive.BOX_Y],
              primitive.dimensions[primitive.BOX_Z],
              object_scale_factor_ * 100.0);
    }
    
    // Create pose in camera frame
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header.frame_id = cloud_frame;
    pose_cam.pose.position.x = params.center_pt[0];
    pose_cam.pose.position.y = params.center_pt[1];
    pose_cam.pose.position.z = params.center_pt[2];
    
    // Set orientation based on shape
    if (params.shape_type == "cylinder")
    {
      // Align cylinder with z-axis
      Eigen::Vector3d z_cyl(params.direction_vec[0], 
                           params.direction_vec[1], 
                           params.direction_vec[2]);
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
      // Identity orientation for box
      pose_cam.pose.orientation.w = 1.0;
      pose_cam.pose.orientation.x = 0.0;
      pose_cam.pose.orientation.y = 0.0;
      pose_cam.pose.orientation.z = 0.0;
    }
    
    // Transform to odom frame
    geometry_msgs::PoseStamped pose_odom;
    try
    {
      // Use ros::Time(0) to get the latest transform available
      pose_cam.header.stamp = ros::Time(0);
      tf_buffer_->transform(pose_cam, pose_odom, "odom", ros::Duration(2.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("TF transform failed: " << ex.what());
      return false;
    }
    
    // Add to planning scene
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose_odom.pose);
    collision_object.operation = collision_object.ADD;
    
    planning_scene_interface_.applyCollisionObject(collision_object);
    
    ROS_INFO("Added %s [%s] to planning scene at [%.3f, %.3f, %.3f] in odom frame",
            object_name.c_str(),
            params.shape_type.c_str(),
            pose_odom.pose.position.x,
            pose_odom.pose.position.y,
            pose_odom.pose.position.z);
    
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_object_service");
  
  ObjectSegmentService service;
  
  ROS_INFO("Add Object Service is ready.");
  ROS_INFO("Call /add_object_to_scene when object is detected and visible.");
  
  ros::spin();
  
  return 0;
}