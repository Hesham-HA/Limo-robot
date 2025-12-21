#include <ros/ros.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PoseStamped.h>

using namespace moveit::task_constructor;

Task createTask(const geometry_msgs::PoseStamped& grasp_pose,
                const geometry_msgs::PoseStamped& lift_pose,
                const geometry_msgs::PoseStamped& place_pose)
{
  Task task;
  task.stages()->setName("limo_pick_place");
  task.loadRobotModel();

  // Global properties
  task.setProperty("group", "arm");
  task.setProperty("eef", "gripper");
  task.setProperty("ik_frame", "joint6_flange");

  /* ===============================
   * 1. Current state
   * =============================== */
  auto current = std::make_unique<stages::CurrentState>("current");
  task.add(std::move(current));

  /* ===============================
   * 2. Move to grasp pose
   * =============================== */
  auto move_to_grasp = std::make_unique<stages::MoveTo>("move_to_grasp");
  move_to_grasp->setGroup("arm");
  move_to_grasp->setGoal(grasp_pose);
  task.add(std::move(move_to_grasp));

  /* ===============================
   * 3. Attach object
   * =============================== */
  auto attach = std::make_unique<stages::ModifyPlanningScene>("attach_object");
  attach->attachObject("object", "joint6_flange");
  task.add(std::move(attach));

  /* ===============================
   * 4. Lift
   * =============================== */
  auto lift = std::make_unique<stages::MoveTo>("lift");
  lift->setGroup("arm");
  lift->setGoal(lift_pose);
  task.add(std::move(lift));

  /* ===============================
   * 5. Move to place
   * =============================== */
  auto move_to_place = std::make_unique<stages::MoveTo>("move_to_place");
  move_to_place->setGroup("arm");
  move_to_place->setGoal(place_pose);
  task.add(std::move(move_to_place));

  /* ===============================
   * 6. Detach object
   * =============================== */
  auto detach = std::make_unique<stages::ModifyPlanningScene>("detach_object");
  detach->detachObject("object", "joint6_flange");
  task.add(std::move(detach));

  return task;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "limo_mtc_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /* ===== Example poses (replace with perception output) ===== */
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = "base_footprint";
  grasp_pose.pose.position.x = 0.5;
  grasp_pose.pose.position.y = 0.0;
  grasp_pose.pose.position.z = 0.3;
  grasp_pose.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped lift_pose = grasp_pose;
  lift_pose.pose.position.z += 0.15;

  geometry_msgs::PoseStamped place_pose = lift_pose;
  place_pose.pose.position.y += 0.3;

  /* ===== Wait for object to appear in scene ===== */
  while (ros::ok()) {
    auto objs = planning_scene_interface_.getObjects({"object"});
    if (!objs.empty()) break;
    ros::Duration(0.1).sleep();
  }
  /* ===== Create and plan task ===== */
  Task task = createTask(grasp_pose, lift_pose, place_pose);

  if (!task.plan()) {
    ROS_ERROR("Task planning failed");
    return 1;
  }

  ROS_INFO("Task planning succeeded with %zu solution(s)", task.solutions().size());

  const auto& solution = task.solutions().front();

  auto result = task.execute(*solution);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Task execution failed");
    return 1;
  }
  ROS_INFO("Task executed successfully");
  return 0;
}
