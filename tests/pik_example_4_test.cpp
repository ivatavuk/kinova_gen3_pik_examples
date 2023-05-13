#include <iostream>
#include <memory>

#include "PikRos.hpp"

#include <Eigen/Dense>

int main(int argc, char **argv)
{
  //-----------------Expected solutions--------------

  const Eigen::Matrix<double, 7, 1> expected_q = (Eigen::Matrix<double, 7, 1>() << 2.02412, 0.388816, -1.14375, -0.0393241, 0.135001, -0.0172962, -2.1368).finished();

  const double tests_tolerance = 1e-2;
  //-----------------Initialization------------------
  ros::init(argc, argv, "pik_examples");
  ros::NodeHandle nh;

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
  desired_tool_position << -0.3, -0.5, 1.7;
  desired_tool_orientation << 0.5, 0.5, 0.5, 0.5;
  desired_elbow_position << 0.3, -0.3, 0.0;

  std::vector<PikRos::IkTask> ik_tasks{ PikRos::IkTask( PikRos::FRAME_POSITION, "tool_frame", desired_tool_position ), 
                                        PikRos::IkTask( PikRos::FRAME_ORIENTATION, "tool_frame", desired_tool_orientation ), 
                                        PikRos::IkTask( PikRos::FRAME_POSITION, "forearm_link", desired_elbow_position )};

  q = pik_kinova.solve( ik_tasks,
                        start_q );
  
  if(expected_q.isApprox(q, tests_tolerance)) 
  {
    std::cout << "Example 1 test succeeded\n";
    return 0;
  }
  else
  {
    std::cout << "Example 1 test failed\n";
    return -1;
  }
}