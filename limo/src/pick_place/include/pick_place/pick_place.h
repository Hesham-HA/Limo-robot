#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include <ros/ros.h>
#include <future>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pick_place/AddObjectToScene.h>


// Object shape parameters struct
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

// Structure to hold accumulated pose measurements
struct PoseMeasurement {
  double x, y, z;
  ObjectParams params;
  std::string shape_type;
};

class ObjectPickAndPlace
{
private:
  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner_;
  
  // MoveIt interfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  
  // Subscribers
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber bbox_subscriber_;
  
  // Publisher
  ros::Publisher gripper_traj_pub_;

  // Service
  ros::ServiceServer add_object_service_;
  ros::ServiceServer pick_service_;
  ros::ServiceServer place_service_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Stored data
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  visualization_msgs::MarkerArray latest_bboxes_;
  geometry_msgs::Pose initial_pose_;
  bool has_cloud_;
  bool has_bbox_;
  bool has_initial_pose_;
  
  // Parameters
  std::string object_name_;
  std::string table_name_;
  std::string zero_state_name_;
  std::string top_grip_state_name_;
  std::string front_grip_state_name_;
  std::string arm_group_name;
  std::string gripper_group_name;
  std::string grasp_joint_name;
  std::string grasp_link_name;
  double bbox_scale_factor_;
  double plane_distance_threshold_;
  double cylinder_score_threshold_;
  double plane_probability_;
  int plane_max_iterations_;
  bool use_perception_;
  bool use_multiple_plane_removal_;
  
public:
  ObjectPickAndPlace();
  
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& msg);
  
  void bboxCB(const visualization_msgs::MarkerArray::ConstPtr& msg);

  PoseMeasurement averagePoseMeasurements(const std::vector<PoseMeasurement>& measurements);

  bool addObjectService(pick_place::AddObjectToScene::Request& req, pick_place::AddObjectToScene::Response& res);

  bool addObjectAsync(const bool is_table, std::string& result_message);

  bool pickService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool pickAsync(std::string& result_message);
  
  bool placeService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool placeAsync(std::string& result_message);
  
  void removeExistingObject(const std::string& object_name);

  void filterPointCloudByBBox(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const visualization_msgs::Marker& bbox);
  
  void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);
  
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals, const pcl::PointIndices::Ptr& inliers_plane);
  
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane);
  
  ObjectParams fitCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals, double& score);
  
  ObjectParams fitBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double& score);
  
  bool addObjectToPlanningScene(const ObjectParams& params, const std::string& cloud_frame, const std::string& object_name);

  bool openGripper();
  
  bool closeGripper();
  
  bool pickAndLift(const geometry_msgs::PoseStamped& object_pose, const std::string& frame_id, const std::string& wide_or_tall);
  
  bool placeAndRelease(const geometry_msgs::PoseStamped& place_pose);
  
  bool returnToZero(const std::string state_name);

  void spin();
};

#endif // PICK_PLACE_H