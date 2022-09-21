#pragma once

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

namespace stomp_moveit
{
namespace costs
{
CostFn get_collision_cost_function(const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                                   const moveit::core::JointModelGroup* group, double collision_penalty)
{
  const auto& joints = group ? group->getActiveJointModels() : planning_scene->getRobotModel()->getActiveJointModels();
  const auto& group_name = group ? group->getName() : "";

  CostFn cost_fn = [=](const Eigen::MatrixXd& values, Eigen::VectorXd& costs, bool& validity) {
    static thread_local moveit::core::RobotState sample_state(planning_scene->getCurrentState());

    costs.setZero(values.cols());

    validity = true;

    for (size_t timestep = 0; timestep < values.cols(); ++timestep)
    {
      set_joint_positions(values.col(timestep), joints, sample_state);
      sample_state.update();

      if (planning_scene->isStateColliding(sample_state, group_name))
      {
        costs(timestep) = collision_penalty;
        validity = false;
      }

      // TODO: Check intermediate collisions
    }

    // TODO: Apply kernel smoothing

    return true;
  };

  return cost_fn;
}
}  // namespace costs
}  // namespace stomp_moveit
