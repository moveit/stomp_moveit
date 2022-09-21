#include <rclcpp/rclcpp.hpp>

#include <stomp/stomp.h>

#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>

// MoveItCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

stomp::StompConfiguration getStompConfiguration()
{
  stomp::StompConfiguration config;
  // General settings
  config.num_iterations = 200;            /**< @brief Maximum number of iteration allowed */
  config.num_iterations_after_valid = 10; /**< @brief Stomp will stop optimizing this many iterations after finding a
                                     valid solution */
  config.num_timesteps = 40;              /**< @brief Number of timesteps */
  config.num_dimensions = 7;              /**< @brief Parameter dimensionality */
  config.delta_t = 0.1;                   /**< @brief Time change between consecutive points */
  config.initialization_method = stomp::TrajectoryInitializations::
      LINEAR_INTERPOLATION; /**< @brief TrajectoryInitializations::TrajectoryInitialization */

  // Probability Calculation
  config.exponentiated_cost_sensitivity = 0; /**< @brief Default exponetiated cost sensitivity coefficient */

  // Noisy trajectory generation
  config.num_rollouts = 20; /**< @brief Number of noisy trajectories*/
  config.max_rollouts = 20; /**< @brief The combined number of new and old rollouts during each iteration shouldn't
                       exceed this value */

  // Cost calculation
  config.control_cost_weight = 0.0; /**< @brief Percentage of the trajectory accelerations cost to be applied in the
                                 total cost calculation >*/
  return config;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stomp_moveit_example", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup MoveIt
  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitor()->waitForCurrentRobotState(node->now(), 1.0 /* seconds */);
  moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "stomp_moveit",
                                                      moveit_cpp->getPlanningSceneMonitor());

  geometry_msgs::msg::Pose block_pose;
  block_pose.position.z = 1.0;

  visual_tools.publishCollisionBlock(block_pose, "my_block", 0.3);

  const auto robot_model = moveit_cpp->getRobotModel();
  const auto group = robot_model->getJointModelGroup("panda_arm");
  const auto joints = group->getActiveJointModels();

  const auto planning_scene =
      planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp->getPlanningSceneMonitor())->diff();
  planning_scene->decoupleParent();

  const auto start_state = planning_scene->getCurrentState();
  auto goal_state = start_state;
  goal_state.setJointPositions(joints.at(3), { 0.5 });
  goal_state.setJointPositions(joints.at(2), { 0.5 });

  using namespace stomp_moveit;

  // Configure stomp
  stomp::StompConfiguration config = getStompConfiguration();

  auto noise_generator_fn = noise::get_normal_distribution_generator(config.num_timesteps, { 0.2, 0.2, 0.2, 0.2, 0.1,
                                                                                             0.1, 0.1 } /* stddev */);
  auto cost_fn = costs::get_collision_cost_function(planning_scene, group, 1.0 /* penalty */);
  auto iteration_callback_fn = visualization::get_iteration_path_publisher(visual_tools, group);
  auto done_callback_fn = visualization::get_success_trajectory_publisher(visual_tools, group);

  stomp::TaskPtr task = std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filters::NoFilter,
                                                         iteration_callback_fn, done_callback_fn);

  while (rclcpp::ok())
  {
    stomp::Stomp stomp(config, task);

    Eigen::MatrixXd trajectory;
    if (stomp.solve(get_positions(start_state, joints), get_positions(goal_state, joints), trajectory))
    {
      std::cout << "STOMP succeeded" << std::endl;
    }
    else
    {
      std::cout << "A valid solution was not found" << std::endl;
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  executor.cancel();

  return 0;
}
