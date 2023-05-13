#ifndef RVIZ_VISUALIZATION_HPP_
#define RVIZ_VISUALIZATION_HPP_

#include <vector>
#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

class RvizVisualization
{

public:
  RvizVisualization(ros::NodeHandle &nh);

  void publishVisualizationPoints(const Eigen::Vector3d &desired_elbow_position, 
                                  const Eigen::Vector3d &desired_tool_position,
                                  const Eigen::VectorXd &desired_tool_orientation );

  void publishVisualizationPointsSpraying(const Eigen::Vector3d &desired_elbow_position, 
                                          const Eigen::Vector3d &desired_tool_position,
                                          const Eigen::Vector3d &desired_approach_vector );
private:
  ros::NodeHandle nh_;
  ros::Publisher desired_elbow_position_pub_, desired_tool_position_pub_;
  ros::Publisher approach_vector_pose_pub_, desired_tool_pose_pub_;
}; 

#endif //RVIZ_VISUALIZATION_HPP_