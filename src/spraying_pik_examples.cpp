#include <iostream>
#include <memory>

#include "PikRos.hpp"

#include <Eigen/Dense>
#include "RvizVisualization.hpp"

int main(int argc, char** argv)
{
  //-----------------Initialization------------------
  ros::init(argc, argv, "spraying_pik_examples");
  ros::NodeHandle nh;

  RvizVisualization rviz_vis(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  PikRos::Settings settings;
  settings.use_constrained_opt = true;
  settings.gradient_threshold = 1e-3;
  settings.dq_limit = 10 * M_PI / 180.0;

  settings.polish_solution = true;
  settings.polish_dq_limit = 3 * M_PI / 180.0;
  settings.polish_gradient_threshold = 1e-2;

  settings.ang_err_clamp_magnitude = 30 * M_PI / 180.0;
  settings.lin_err_clamp_magnitude = 0.3;

  settings.debug_mode = false;

  PikRos::Pik pik_kinova(nh, "my_gen3", "arm", settings);

  Eigen::VectorXd initial_guess(7);
  initial_guess << -M_PI / 2.0, -1.2288, M_PI, -2.28, 0.0, -0.7211,
      M_PI / 2.0;  // Initial joint positions

  Eigen::VectorXd q;

  // Setting up vector of IkTasks
  Eigen::VectorXd desired_tool_position(3);
  Eigen::VectorXd desired_approach_vector(3);
  Eigen::VectorXd desired_elbow_position(3);

  //-----------------Example 1------------------
  desired_tool_position << 0.4, 1.0, 0.2;
  desired_approach_vector << 0.0, 1.0, 0.0;
  desired_elbow_position << 0, -0.5, 0.5;
  std::cout << "desired_approach_vector = \n" << desired_approach_vector.normalized() << "\n";

  std::vector<PikRos::IkTask> ik_tasks{ PikRos::IkTask(PikRos::FRAME_POSITION, "spraying_frame", desired_tool_position),
                                        PikRos::IkTask(PikRos::FRAME_APPROACH_AXIS, "spraying_frame",
                                                       desired_approach_vector),
                                        PikRos::IkTask(PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position) };

  std::cout << "\n\nPress Enter to solve task 1\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  rviz_vis.publishVisualizationPointsSpraying(desired_elbow_position, desired_tool_position, desired_approach_vector);
  std::cout << "Solving task 1...\n";

  q = pik_kinova.solve(ik_tasks, initial_guess);

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks);

  pik_kinova.goToJointPosition(q);

  //-----------------Example 2------------------

  desired_tool_position << 0.4, 1.0, 0.8;
  desired_approach_vector << 1.0, 1.0, 1.35;
  std::cout << "desired_approach_vector = \n" << desired_approach_vector.normalized() << "\n";

  std::vector<PikRos::IkTask> ik_tasks_2{
    PikRos::IkTask(PikRos::FRAME_POSITION, "spraying_frame", desired_tool_position),
    PikRos::IkTask(PikRos::FRAME_APPROACH_AXIS, "spraying_frame", desired_approach_vector),
    PikRos::IkTask(PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position)
  };

  std::cout << "\n\nPress Enter to solve task 2\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  rviz_vis.publishVisualizationPointsSpraying(desired_elbow_position, desired_tool_position, desired_approach_vector);
  std::cout << "Solving task 2...\n";

  q = pik_kinova.solve(ik_tasks_2, initial_guess);

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks_2);

  pik_kinova.goToJointPosition(q);

  //-----------------Example 3------------------

  desired_tool_position << 0.4, 1.0, 0.8;
  desired_approach_vector << 1.0, 1.0, -1.0;
  std::cout << "desired_approach_vector = \n" << desired_approach_vector.normalized() << "\n";

  std::vector<PikRos::IkTask> ik_tasks_3{
    PikRos::IkTask(PikRos::FRAME_POSITION, "spraying_frame", desired_tool_position),
    PikRos::IkTask(PikRos::FRAME_APPROACH_AXIS, "spraying_frame", desired_approach_vector),
    PikRos::IkTask(PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position)
  };

  std::cout << "\n\nPress Enter to solve task 3\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  rviz_vis.publishVisualizationPointsSpraying(desired_elbow_position, desired_tool_position, desired_approach_vector);
  std::cout << "Solving task 3...\n";

  q = pik_kinova.solve(ik_tasks_3, initial_guess);

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks_3);

  pik_kinova.goToJointPosition(q);

  //-----------------Shutdown------------------
  std::cout << "ready to shutdown!\n";
  ROS_WARN_STREAM("Finished!");
  return 0;
}