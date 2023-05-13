#include <iostream>
#include <memory>

#include "PikRos.hpp"

#include <Eigen/Dense>
#include "RvizVisualization.hpp"

int main(int argc, char **argv)
{
  //-----------------Initialization------------------
  ros::init(argc, argv, "pik_examples");
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

  Eigen::VectorXd start_q(7);
  start_q << 0.0, 0.26, 3.14, -2.28, 0.0, 0.96, 1.57;
  Eigen::VectorXd q;

  // Setting up vector of IkTasks
  Eigen::VectorXd desired_tool_position(3);
  Eigen::VectorXd desired_tool_orientation(4);
  Eigen::VectorXd desired_elbow_position(3);

  Eigen::VectorXd desired_tool_pose(7);

  //-----------------Example 1------------------
  desired_tool_position << -0.2, -0.3, 0.6;
  desired_tool_orientation << 0.5, 0.5, 0.5, 0.5;
  desired_elbow_position << -0.3, -0.3, 0.0;

  desired_tool_pose << desired_tool_position, desired_tool_orientation;

  std::vector<PikRos::IkTask> ik_tasks{ PikRos::IkTask( PikRos::FRAME_POSE, "tool_frame", desired_tool_pose ), 
                                        PikRos::IkTask( PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position )};

  std::cout << "\n\nPress Enter to solve task 1\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
  rviz_vis.publishVisualizationPoints(desired_elbow_position, desired_tool_position, desired_tool_orientation);
  std::cout << "Solving task 1...\n";

  q = pik_kinova.solve( ik_tasks,
                        start_q );

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks);

  pik_kinova.goToJointPosition(q);

  //-----------------Example 2------------------

  desired_tool_position << -0.2, -0.3, 0.6;
  desired_tool_orientation << 0.5, 0.5, 0.5, 0.5;
  desired_elbow_position << -1.0, -0.3, 1.0;

  std::vector<PikRos::IkTask> ik_tasks_2{ PikRos::IkTask( PikRos::FRAME_POSITION, "tool_frame", desired_tool_position ), 
                                          PikRos::IkTask( PikRos::FRAME_ORIENTATION, "tool_frame", desired_tool_orientation ), 
                                          PikRos::IkTask( PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position )};

  std::cout << "\n\nPress Enter to solve task 2\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
  rviz_vis.publishVisualizationPoints(desired_elbow_position, desired_tool_position, desired_tool_orientation);
  std::cout << "Solving task 2...\n";

  q = pik_kinova.solve( ik_tasks_2,
                        start_q );

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks_2);

  pik_kinova.goToJointPosition(q);

  //-----------------Example 3------------------

  desired_tool_position << -0.3, -0.5, 0.7;
  desired_tool_orientation << 0.5, 0.5, 0.5, 0.5;
  desired_elbow_position << 0.3, -0.3, 0.0;

  desired_tool_pose << desired_tool_position, desired_tool_orientation;

  std::vector<PikRos::IkTask> ik_tasks_3{ PikRos::IkTask( PikRos::FRAME_POSITION, "tool_frame", desired_tool_position ), 
                                          PikRos::IkTask( PikRos::FRAME_ORIENTATION, "tool_frame", desired_tool_orientation ),
                                          //PikRos::IkTask( PikRos::FRAME_POSE, "tool_frame", desired_tool_pose ),  
                                          PikRos::IkTask( PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position )};

  std::cout << "\n\nPress Enter to solve task 3\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
  rviz_vis.publishVisualizationPoints(desired_elbow_position, desired_tool_position, desired_tool_orientation);
  std::cout << "Solving task 3...\n";

  q = pik_kinova.solve( ik_tasks_3,
                        start_q );

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks_3);

  pik_kinova.goToJointPosition(q);

  //-----------------Example 4------------------

  desired_tool_position << -0.3, -0.5, 1.7;
  desired_tool_orientation << 0.5, 0.5, 0.5, 0.5;
  desired_elbow_position << 0.3, -0.3, 0.0;

  std::vector<PikRos::IkTask> ik_tasks_4{ PikRos::IkTask( PikRos::FRAME_POSITION, "tool_frame", desired_tool_position ), 
                                          PikRos::IkTask( PikRos::FRAME_ORIENTATION, "tool_frame", desired_tool_orientation ), 
                                          PikRos::IkTask( PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position )};

  std::cout << "\n\nPress Enter to solve task 3\n\n";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
  rviz_vis.publishVisualizationPoints(desired_elbow_position, desired_tool_position, desired_tool_orientation);
  std::cout << "Solving task 4...\n";

  q = pik_kinova.solve( ik_tasks_4,
                        start_q );

  std::cout << "Errors:\n";
  pik_kinova.printErrors(q, ik_tasks_4);

  pik_kinova.goToJointPosition(q);

  //-----------------Shutdown------------------
  std::cout << "ready to shutdown!\n";
  ROS_WARN_STREAM("Finished!");
  return 0;
}