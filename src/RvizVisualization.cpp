#include "RvizVisualization.hpp"

RvizVisualization::RvizVisualization( ros::NodeHandle &nh )
  : nh_(nh)
{ 
  desired_elbow_position_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("rviz_visualization/desired_elbow_pos", 1);
  desired_tool_position_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("rviz_visualization/desired_tool_pos", 1);
  approach_vector_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rviz_visualization/desired_approach_vector", 1);
  desired_tool_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rviz_visualization/desired_tool_pose", 1);
}

void RvizVisualization::publishVisualizationPoints( const Eigen::Vector3d &desired_elbow_position, 
                                                    const Eigen::Vector3d &desired_tool_position,
                                                    const Eigen::VectorXd &desired_tool_orientation )
{
  geometry_msgs::PointStamped temp_msg;
  temp_msg.header.stamp = ros::Time::now();
  temp_msg.header.frame_id = "base_link";
  temp_msg.point.x = desired_elbow_position(0);
  temp_msg.point.y = desired_elbow_position(1);
  temp_msg.point.z = desired_elbow_position(2);
  desired_elbow_position_pub_.publish(temp_msg);
  temp_msg.point.x = desired_tool_position(0);
  temp_msg.point.y = desired_tool_position(1);
  temp_msg.point.z = desired_tool_position(2);
  desired_tool_position_pub_.publish(temp_msg);

  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header = temp_msg.header;
  pose_stamped_msg.pose.position = temp_msg.point;

  pose_stamped_msg.pose.orientation.w = desired_tool_orientation(0); 
  pose_stamped_msg.pose.orientation.x = desired_tool_orientation(1);
  pose_stamped_msg.pose.orientation.y = desired_tool_orientation(2);
  pose_stamped_msg.pose.orientation.z = desired_tool_orientation(3);
  desired_tool_pose_pub_.publish(pose_stamped_msg);
}


void RvizVisualization::publishVisualizationPointsSpraying( const Eigen::Vector3d &desired_elbow_position, 
                                                            const Eigen::Vector3d &desired_tool_position,
                                                            const Eigen::Vector3d &desired_approach_vector )
{
  geometry_msgs::PointStamped temp_msg;
  temp_msg.header.stamp = ros::Time::now();
  temp_msg.header.frame_id = "base_link";
  temp_msg.point.x = desired_elbow_position(0);
  temp_msg.point.y = desired_elbow_position(1);
  temp_msg.point.z = desired_elbow_position(2);
  desired_elbow_position_pub_.publish(temp_msg);
  temp_msg.point.x = desired_tool_position(0);
  temp_msg.point.y = desired_tool_position(1);
  temp_msg.point.z = desired_tool_position(2);
  desired_tool_position_pub_.publish(temp_msg);

  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header = temp_msg.header;
  pose_stamped_msg.pose.position = temp_msg.point;

  Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
    Eigen::Vector3d::UnitX(), desired_approach_vector.normalized()
    ); //Rviz displays arrow of pose in the x direction 
  pose_stamped_msg.pose.orientation.w = orientation.w();
  pose_stamped_msg.pose.orientation.x = orientation.x();
  pose_stamped_msg.pose.orientation.y = orientation.y();
  pose_stamped_msg.pose.orientation.z = orientation.z();

  approach_vector_pose_pub_.publish(pose_stamped_msg);
}