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

#ifndef NMEA2TFPOSE_CORE_H
#define NMEA2TFPOSE_CORE_H

// C++ includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <gnss/geo_pos_conv.hpp>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>

namespace gnss_localizer
{
class Nmea2TFPoseNode
{
public:
  Nmea2TFPoseNode();
  ~Nmea2TFPoseNode();

  void run();

private:
  // constants
  const std::string MAP_FRAME_;
  //const std::string GPS_FRAME_;
  std::string GPS_FRAME_;
  int node_number_;
  bool use_node_number_;

  // variables
  int32_t plane_number_;
  geo_pos_conv geo_, geo_sub_;
  std::vector<geo_pos_conv> geo_vec_;
  ros::Time prev_stamp_;
  geo_pos_conv last_geo_;
  double roll_, pitch_, yaw_;
  double pva_roll_, pva_pitch_, pva_yaw_;
  double orientation_time_, position_time_, surface_speed_;
  double lat_std_, lon_std_, alt_std_;
  double lat_std_sub_, lon_std_sub_, alt_std_sub_;
  double north_velocity_std_dev_, east_velocity_std_dev_, up_velocity_std_dev_;
  double roll_std_dev_, pitch_std_dev_, yaw_std_dev_;
  double x_accel_, y_accel_, z_accel_, x_gyro_, y_gyro_, z_gyro_;
  unsigned char gnss_stat_;
  ros::Time current_time_, orientation_stamp_;
  tf::TransformBroadcaster br_;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_;
  ros::Publisher pub_surface_speed_, pub_std_dev_, pub_std_dev2_, pub_imu_, pub_stat_, pub_time_, pub_fix_, pub_tmp_;

  // subscriber
  ros::Subscriber sub1_, sub_gnss_select_;

  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);
  void callbackGnssSelect(const std_msgs::Int32 &msg);

  // initializer
  void initForROS();

  // functions
  void publishPoseStamped();
  void publishTF();
  void createOrientation();
  void convert(std::vector<std::string> nmea, ros::Time current_stamp);
};

std::vector<std::string> split(const std::string &string);

}  // gnss_localizer
#endif  // NMEA2TFPOSE_CORE_H
