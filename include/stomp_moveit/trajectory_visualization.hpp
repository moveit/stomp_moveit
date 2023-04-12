#pragma once

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace stomp_moveit
{
namespace visualization
{

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("stomp_moveit");
const auto GREEN = []() {
  std_msgs::msg::ColorRGBA color;
  color.r = 0.1;
  color.g = 0.8;
  color.b = 0.1;
  color.a = 1.0;
  return color;
}();

const auto TRANSLUCENT_LIGHT = []() {
  std_msgs::msg::ColorRGBA color;
  color.r = 0.1;
  color.g = 0.8;
  color.b = 0.1;
  color.a = 0.5;
  return color;
}();
}  // namespace

bool publishTrajectoryPoints(const robot_trajectory::RobotTrajectory& robot_trajectory,
                             const moveit::core::LinkModel* ee_parent_link,
                             rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher,
                             const std_msgs::msg::ColorRGBA& color = GREEN)
{
  // Create Sphere Marker
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = "panda_link0";
  sphere_marker.ns = "Path";
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::msg::Marker::ADD;
  sphere_marker.lifetime = rclcpp::Duration(0, 0);  // Infinite lifetime
  sphere_marker.scale.x = 0.01;
  sphere_marker.scale.y = 0.01;
  sphere_marker.scale.z = 0.01;
  sphere_marker.frame_locked = false;

  visualization_msgs::msg::MarkerArray markers_array;
  // Visualize end effector position of cartesian path
  for (std::size_t index = 0; index < robot_trajectory.getWayPointCount(); index++)
  {
    const Eigen::Isometry3d& tip_pose = robot_trajectory.getWayPoint(index).getGlobalLinkTransform(ee_parent_link);
    sphere_marker.pose = tf2::toMsg(tip_pose);

    // Normalize pose
    double norm = 0;
    norm += sphere_marker.pose.orientation.w * sphere_marker.pose.orientation.w;
    norm += sphere_marker.pose.orientation.x * sphere_marker.pose.orientation.x;
    norm += sphere_marker.pose.orientation.y * sphere_marker.pose.orientation.y;
    norm += sphere_marker.pose.orientation.z * sphere_marker.pose.orientation.z;
    norm = std::sqrt(norm);
    if (norm < std::numeric_limits<double>::epsilon())
    {
      sphere_marker.pose.orientation.w = 1;
      sphere_marker.pose.orientation.x = 0;
      sphere_marker.pose.orientation.y = 0;
      sphere_marker.pose.orientation.z = 0;
    }
    else
    {
      sphere_marker.pose.orientation.w = sphere_marker.pose.orientation.w / norm;
      sphere_marker.pose.orientation.x = sphere_marker.pose.orientation.x / norm;
      sphere_marker.pose.orientation.y = sphere_marker.pose.orientation.y / norm;
      sphere_marker.pose.orientation.z = sphere_marker.pose.orientation.z / norm;
    }

    // Create a sphere point
    // Add the point pair to the line message
    sphere_marker.color = color;

    sphere_marker.id = index;

    markers_array.markers.push_back(sphere_marker);
  }

  if (markers_array.markers.empty())
  {
    return false;
  }

  marker_publisher->publish(markers_array);
  return true;
}

PostIterationFn
get_iteration_path_publisher(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher,
                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                             const moveit::core::JointModelGroup* group)
{
  assert(group != nullptr);

  std::shared_ptr<const moveit::core::RobotState> reference_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    reference_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }

  PostIterationFn path_publisher = [=](int /*iteration_number*/, double /*cost*/, const Eigen::MatrixXd& values) {
    static thread_local robot_trajectory::RobotTrajectory trajectory(planning_scene_monitor->getRobotModel(), group);
    fill_robot_trajectory(values, *reference_state, trajectory);

    std::vector<const moveit::core::LinkModel*> tips;
    if (!group->getEndEffectorTips(tips))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
    }

    // For each end effector
    for (const moveit::core::LinkModel* ee_parent_link : tips)
    {
      publishTrajectoryPoints(trajectory, ee_parent_link, marker_publisher, TRANSLUCENT_LIGHT);
    }
  };

  return path_publisher;
}

DoneFn
get_success_trajectory_publisher(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher,
                                 planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                 const moveit::core::JointModelGroup* group)
{
  assert(group != nullptr);

  std::shared_ptr<const moveit::core::RobotState> reference_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
    reference_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }

  DoneFn path_publisher = [=](bool success, int /*total_iterations*/, double /*final_cost*/,
                              const Eigen::MatrixXd& values) {
    static thread_local robot_trajectory::RobotTrajectory trajectory(reference_state->getRobotModel(), group);
    if (success)
    {
      fill_robot_trajectory(values, *reference_state, trajectory);

      std::vector<const moveit::core::LinkModel*> tips;
      if (!group->getEndEffectorTips(tips))
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
      }

      // For each end effector
      for (const moveit::core::LinkModel* ee_parent_link : tips)
      {
        publishTrajectoryPoints(trajectory, ee_parent_link, marker_publisher);
      }
    }
  };

  return path_publisher;
}
}  // namespace visualization
}  // namespace stomp_moveit
