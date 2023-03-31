#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_model/joint_model_group.h>
#include <stomp_moveit/stomp_moveit_task.hpp>

#include <stomp/utils.h>

namespace stomp_moveit
{
namespace filters
{
const static FilterFn NoFilter = [](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) { return true; };

FilterFn simple_smoothing_matrix(size_t num_timesteps)
{
  Eigen::MatrixXd smoothing_matrix;
  stomp::generateSmoothingMatrix(num_timesteps, 1.0, smoothing_matrix);
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    for (int i = 0; i < filtered_values.rows(); ++i)
    {
      filtered_values.row(i).transpose() = smoothing_matrix * (filtered_values.row(i).transpose());
    }
    return true;
  };
}
FilterFn enforce_position_bounds(const moveit::core::JointModelGroup* group)
{
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    filtered_values = values;
    const auto& joints = group->getActiveJointModels();
    for (size_t i = 0; i < joints.size(); ++i)
    {
      for (int j = 0; j < filtered_values.cols(); ++j)
      {
        joints.at(i)->enforcePositionBounds(&filtered_values.coeffRef(i, j));
      }
    }
    return true;
  };
}
FilterFn chain(const std::vector<FilterFn>& filter_functions)
{
  return [=](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) {
    Eigen::MatrixXd values_in = values;
    for (const auto& filter_fn : filter_functions)
    {
      filter_fn(values_in, filtered_values);
      values_in = filtered_values;
    }
    return true;
  };
}
}  // namespace filters
}  // namespace stomp_moveit
