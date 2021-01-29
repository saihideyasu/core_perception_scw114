/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "can_odometry_core_scw.h"

namespace autoware_connector
{

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

// Constructor
CanOdometryNode::CanOdometryNode() : private_nh_("~"), v_info_(), odom_(ros::Time::now())
{
  initForROS();
}

// Destructor
CanOdometryNode::~CanOdometryNode()
{
}

void CanOdometryNode::initForROS()
{
  // ros parameter settings
  if (!nh_.hasParam("/vehicle_info/wheel_base"))
  {
    v_info_.is_stored = false;
    ROS_INFO("vehicle_info is not set");
  }
  else
  {
    private_nh_.getParam("/vehicle_info/wheel_base", v_info_.wheel_base);
    // ROS_INFO_STREAM("wheel_base : " << wheel_base);

    v_info_.is_stored = true;
  }

  // setup subscriber
  sub1_ = nh_.subscribe("vehicle_status", 10, &CanOdometryNode::callbackFromVehicleStatus, this);
  sub_bus_ = nh_.subscribe("/microbus/can_receive502", 10, &CanOdometryNode::callbackFromVehicleStatus_microbus, this);
  sub_config_ = nh_.subscribe("/config/can2odom_scw", 10, &CanOdometryNode::callbackConfig, this);
  //sub_gnss_pose_ = nh_.subscribe("/gnss_pose", 10, &CanOdometryNode::callbackGnssPose, this);//車角度確認用

  // setup publisher
  pub1_ = nh_.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
  pub_can_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>("/can_velocity", 10);
  pub_tmp_ = nh_.advertise<std_msgs::String>("/can_odom_tmp", 10);

  config_.kmph_th = 0.1;
}

void CanOdometryNode::run()
{
  ros::spin();
}

void CanOdometryNode::publishOdometry(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  double vx = kmph2mps(msg->speed);
  double vth = v_info_.convertSteeringAngleToAngularVelocity(kmph2mps(msg->speed), msg->angle);
  odom_.updateOdometry(vx, vth, msg->header.stamp);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  // publish the message
  pub1_.publish(odom);
}

void CanOdometryNode::callbackGnssPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  gnss_pose_ = *msg;
}

void CanOdometryNode::callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  publishOdometry(msg);
}

void CanOdometryNode::callbackConfig(const autoware_config_msgs::ConfigCanOdometry &msg)
{
  config_ = msg;
}

void CanOdometryNode::callbackFromVehicleStatus_microbus(const autoware_can_msgs::MicroBusCan502ConstPtr &msg)
{
	double vx = msg->velocity_mps;//kmph2mps(msg->velocity_mps);
	if(vx < config_.kmph_th / 3.6) vx = 0;
	double vth = v_info_.convertSteeringAngleToAngularVelocity_microbus(vx, msg->angle_actual);
	odom_.updateOdometry(vx, vth, msg->header.stamp);

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);

	// next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = msg->header.stamp;
	odom.header.frame_id = "odom";

	// set the position
	odom.pose.pose.position.x = odom_.x;
	odom.pose.pose.position.y = odom_.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	// set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.angular.z = vth;

	// publish the message
	pub1_.publish(odom);

  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "can";
  twist.header.stamp = msg->header.stamp;
  twist.twist.linear.x = vx;
  twist.twist.linear.y = 0;
  twist.twist.linear.z = 0;
  twist.twist.angular.x = 0;
  twist.twist.angular.y = 0;
  twist.twist.angular.z = vth;
  pub_can_velocity_.publish(twist);

  /*double groll, gpitch, gyaw;
  geometry_quat_to_rpy(groll, gpitch, gyaw, gnss_pose_.pose.orientation);
  std::stringstream ss;
  ss << std::fixed;
  ss << std::setprecision(6) << vx << "," << vth << "," << gyaw << "," << msg->angle_actual;
  std_msgs::String str;
  str.data = ss.str();
  pub_tmp_.publish(str);*/
}

}  // autoware_connector
