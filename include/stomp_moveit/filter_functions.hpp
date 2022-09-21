#pragma once

#include <Eigen/Geometry>
#include <stomp_moveit/stomp_moveit_task.hpp>

namespace stomp_moveit
{
namespace filters
{
const static FilterFn NoFilter = [](const Eigen::MatrixXd& values, Eigen::MatrixXd& filtered_values) { return true; };
}
}  // namespace stomp_moveit
