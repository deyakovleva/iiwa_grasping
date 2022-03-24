﻿#include <gpd_ros/grasp_messages.h>

gpd_ros::GraspConfigList GraspMessages::createGraspListMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  gpd_ros::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}

gpd_ros::GraspConfig GraspMessages::convertToGraspMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::GraspConfig msg;
  tf::pointEigenToMsg(hand.getPosition(), msg.position);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);

  return msg;
}

gpd_ros::GraspConfigListPose GraspMessages::createGraspListPoseMsg(const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std_msgs::Header& header)
{
  gpd_ros::GraspConfigListPose msg;

  for (int i = 0; i < hands.size(); i++) {
    msg.poses.push_back(convertToGraspGeomPoseMsg(*hands[i]));
  }

  msg.header = header;

  return msg;
}


gpd_ros::GraspConfigGeomPose GraspMessages::convertToGraspGeomPoseMsg(const gpd::candidate::Hand& hand)
{
  gpd_ros::GraspConfigGeomPose msg;
  tf::pointEigenToMsg(hand.getPosition(), msg.pose.position);

//  tf::quaternionEigenToMsg(hand.getQuaternion(), msg.pose.orientation);

  Eigen::Quaternion<double> quat (hand.getOrientation());
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();
  msg.pose.orientation.w = quat.w();

  msg.width.data = float(hand.getGraspWidth());
  msg.score.data = float(hand.getScore());

  return msg;
}
