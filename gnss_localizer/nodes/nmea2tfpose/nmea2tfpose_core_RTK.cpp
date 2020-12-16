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

#include "nmea2tfpose_core_RTK.h"

namespace gnss_localizer
{
uint8_t GpsFixCovTypeToNavSatFixCovType(uint8_t covariance_type)
{
  switch(covariance_type)
  {
    case gps_common::GPSFix::COVARIANCE_TYPE_APPROXIMATED:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    case gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    case gps_common::GPSFix::COVARIANCE_TYPE_KNOWN:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

    case gps_common::GPSFix::COVARIANCE_TYPE_UNKNOWN:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    default:
      ROS_ERROR_STREAM("Unknown GPSFix covariance type: " << covariance_type);
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
}

// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , orientation_stamp_(0)
  , surface_speed_(0)
  , lat_std_(0)
  , lon_std_(0)
  , alt_std_(0)
  , gnss_stat_(0)
  , use_node_number_(false)
{
  initForROS();
  geo_.set_plane(plane_number_);
  geo_sub_.set_plane(plane_number_);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

double degrees_minutes_seconds(double deg)
{
	int degrees = (int)deg;
	double d_minutes = (deg-degrees)*60.0;
	int minutes = (int)d_minutes;
	double seconds = (d_minutes-minutes)*60.0;
	//return degrees*100.0+minutes+seconds/100.0;
	return degrees*100.0+d_minutes;
}

double dig2rad(double dig)
{
	while(dig <0 || dig >=360){
		if(dig<0) dig+=360;
	  else if(dig>=360) dig-=360;
	}
	return dig*M_PI/180.0;
}

std::vector<std::string> split(const std::string &string, const char sep)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, sep))
	str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);
  private_nh_.getParam("node_number", node_number_);

  std::string nmea_topic;
  if(private_nh_.getParam("nmea_topic", nmea_topic) == false)
  {
    std::cerr << "error : not nmea_topic" << std::endl;
  }

  // setup subscriber
  sub1_ = nh_.subscribe(nmea_topic, 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);
  sub_gnss_select_ = nh_.subscribe("/gnss_select", 1, &Nmea2TFPoseNode::callbackGnssSelect, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  pub2_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose_sub", 10);
  pub_surface_speed_ = nh_.advertise<autoware_msgs::GnssSurfaceSpeed>("gnss_surface_speed", 10);
  pub_std_dev_ = nh_.advertise<autoware_msgs::GnssStandardDeviation>("gnss_standard_deviation", 10);
  pub_std_dev2_ = nh_.advertise<autoware_msgs::GnssStandardDeviation>("gnss_standard_deviation_sub", 10);
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("gnss_imu", 10);
  pub_stat_ = nh_.advertise<std_msgs::UInt8>("gnss_rtk_stat", 10);
  pub_time_ = nh_.advertise<autoware_system_msgs::Date>("gnss_time", 10);
  pub_fix_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);
  pub_tmp_ = nh_.advertise<std_msgs::Float64>("gnss_tmp", 10);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.y();
  pose.pose.position.y = geo_.x();
  pose.pose.position.z = geo_.z();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  pub1_.publish(pose);

  autoware_msgs::GnssSurfaceSpeed gss;
  gss.header.frame_id = MAP_FRAME_;
  gss.header.stamp = current_time_;
  gss.surface_speed = surface_speed_;
  pub_surface_speed_.publish(gss);

  autoware_msgs::GnssStandardDeviation gsd;
  gsd.header.frame_id = MAP_FRAME_;
  gsd.header.stamp = current_time_;
  gsd.lat_std_dev = lat_std_;
  gsd.lon_std_dev = lon_std_;
  gsd.alt_std_dev = alt_std_;
  pub_std_dev_.publish(gsd);

  sensor_msgs::Imu imu;
  imu.header.frame_id = MAP_FRAME_;
  imu.header.stamp = current_time_;
  imu.linear_acceleration.x = x_accel_;
  imu.linear_acceleration.y = y_accel_;
  imu.linear_acceleration.z = z_accel_;
  imu.angular_velocity.x = x_gyro_;
  imu.angular_velocity.y = y_gyro_;
  imu.angular_velocity.z = z_gyro_;
  pub_imu_.publish(imu);

  std_msgs::UInt8 stat;
  stat.data = gnss_stat_;
  pub_stat_.publish(stat);
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.y(), geo_.x(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  std::stringstream stream;
  if(use_node_number_ == true || node_number_ == -1) stream << GPS_FRAME_;
  else stream << GPS_FRAME_ << node_number_;
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, stream.str()));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  std::cout << "yaw2 : " << yaw_ << std::endl;
  roll_ = 0;
  pitch_ = 0;
}

void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if(nmea.at(0) == "#BESTPOSA")
    {
      if(nmea.size() == 30)
      {
        //std::cout << "aaa : " << nmea.at(16) << std::endl;
        double lat = degrees_minutes_seconds(stod(nmea.at(11))); std::cout << "lat : " << std::setprecision(16) << lat << std::endl;
        double lon = degrees_minutes_seconds(stod(nmea.at(12))); std::cout << "lon : " << std::setprecision(16) << lon << std::endl;
        double h = stod(nmea.at(13)); std::cout << "h : " << std::setprecision(16) << h << std::endl;
        geo_.set_llh_nmea_degrees(lat, lon, h);
        geo_vec_.insert(geo_vec_.begin(), geo_);
        if(geo_vec_.size() > 5) geo_vec_.resize(5);
        ROS_INFO("BESTPOS is subscribed.");
        std::cout << "lat : " << std::setprecision(16) << geo_.y() << std::endl;
        std::cout << "lat : " << std::setprecision(16) << geo_.x() << std::endl;
        std::cout << "lat : " << std::setprecision(16) << geo_.z() << std::endl;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = MAP_FRAME_;
        pose.header.stamp = current_time_;
        pose.pose.position.x = geo_.y();
        pose.pose.position.y = geo_.x();
        pose.pose.position.z = geo_.z();
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
        pub1_.publish(pose);

        /*fix_.latitude = lat;
        fix_.longitude = lon;
        fix_.altitude = h;*/

        double dev_lat = stod(nmea.at(16));
        double dev_lon = stod(nmea.at(17));
        double dev_alt = stod(nmea.at(18));
        sensor_msgs::NavSatFix fix;
        fix.latitude = lat;
        fix.longitude = lon;
        fix.altitude = h;
        fix.position_covariance[0] = std::pow(dev_lon, 2);
        fix.position_covariance[4] = std::pow(dev_lat, 2);
        fix.position_covariance[8] = std::pow(dev_alt, 2);
        fix.position_covariance_type = GpsFixCovTypeToNavSatFixCovType(gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
        pub_fix_.publish(fix);
      }
    }
    else if(nmea.at(0) == "#BESTGNSSPOSA")
    {
      if(nmea.size() == 30)
      {
        //std::cout << "aaa : " << nmea.at(16) << std::endl;
        double lat = degrees_minutes_seconds(stod(nmea.at(11))); std::cout << "lat2 : " << std::setprecision(16) << lat << std::endl;
        double lon = degrees_minutes_seconds(stod(nmea.at(12))); std::cout << "lon2 : " << std::setprecision(16) << lon << std::endl;
        double h = stod(nmea.at(13)); std::cout << "h2 : " << std::setprecision(16) << h << std::endl;
        geo_sub_.set_llh_nmea_degrees(lat, lon, h);
        ROS_INFO("BESTGNSSPOS is subscribed.");

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "gps2";
        pose.header.stamp = current_time_;
        pose.pose.position.x = geo_sub_.y();
        pose.pose.position.y = geo_sub_.x();
        pose.pose.position.z = geo_sub_.z();
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
        pub2_.publish(pose);

        lat_std_sub_ = stod(nmea.at(16));
        lon_std_sub_ = stod(nmea.at(17));
        alt_std_sub_ = stod(nmea.at(18));

        autoware_msgs::GnssStandardDeviation gsd;
        gsd.header.frame_id = "gps2";
        gsd.header.stamp = current_time_;
        gsd.lat_std_dev = lat_std_sub_;
        gsd.lon_std_dev = lon_std_sub_;
        gsd.alt_std_dev = alt_std_sub_;
        pub_std_dev2_.publish(gsd);
      }
    }
    else if(nmea.at(0) == "#INSPVAXA")
    {
      if(nmea.size() == 32)
      {
        double north_vel = stod(nmea.at(15));
        double east_vel = stod(nmea.at(16));
        double up_vel = stod(nmea.at(17));
        this->surface_speed_ = sqrt(north_vel*north_vel + east_vel*east_vel + up_vel*up_vel);

        autoware_msgs::GnssSurfaceSpeed gss;
        gss.header.frame_id = MAP_FRAME_;
        gss.header.stamp = current_time_;
        gss.surface_speed = surface_speed_;
        pub_surface_speed_.publish(gss);

        roll_= dig2rad(stod(nmea.at(18)));
        pitch_= -dig2rad(stod(nmea.at(19)));
        yaw_ = dig2rad(stod(nmea.at(20)));
        pva_roll_ = dig2rad(stod(nmea.at(18)));
        pva_pitch_ = dig2rad(stod(nmea.at(19)));
        pva_yaw_ = dig2rad(stod(nmea.at(20)));
        std::cout << "yaw  : " << yaw_ << std::endl;
        std::cout << "roll : " << roll_ << std::endl;
        std::cout << "pitch: " << pitch_ << std::endl;
        double x = cos(yaw_), y = sin(yaw_);
        yaw_ = atan2(x,y);

        std_msgs::Float64 flo;
        flo.data = pitch_;
        pub_tmp_.publish(flo);
        /*lat_std_ = stod(nmea.at(21));
        lon_std_ = stod(nmea.at(22));
        alt_std_ = stod(nmea.at(23));

        autoware_msgs::GnssStandardDeviation gsd;
        gsd.header.frame_id = MAP_FRAME_;
        gsd.header.stamp = current_time_;
        gsd.lat_std_dev = lat_std_;
        gsd.lon_std_dev = lon_std_;
        gsd.alt_std_dev = alt_std_;
        pub_std_dev_.publish(gsd);*/

        //gnss_stat_ = stoi(nmea.at(9));
        std::vector<std::string> vec = split(nmea.at(9), ';');
        if(vec.size() < 2) gnss_stat_ = 0;
        else if(vec[1] == "INS_SOLUTION_GOOD") gnss_stat_ = 3;
        else gnss_stat_ = 0;
        std_msgs::UInt8 stat;
        stat.data = gnss_stat_;
        pub_stat_.publish(stat);

        ROS_INFO("INSPVAXA is subscribed.");
      }
    }
    else if(nmea.at(0) == "#INSSTDEVA")
    {
      if(nmea.size() == 23)
      {
        //lat_std_ = stod(nmea.at(9));
        lon_std_ = stod(nmea.at(10));
        alt_std_ = stod(nmea.at(11));
        std::vector<std::string> lat_str = split(nmea.at(9), ';');
        lat_std_ = stod(lat_str.at(1));

        north_velocity_std_dev_ = stod(nmea.at(12));
        east_velocity_std_dev_ = stod(nmea.at(13));
        up_velocity_std_dev_ = stod(nmea.at(14));

        roll_std_dev_ = stod(nmea.at(15));
        pitch_std_dev_ = stod(nmea.at(16));
        yaw_std_dev_ = stod(nmea.at(17));

        autoware_msgs::GnssStandardDeviation gsd;
        gsd.header.frame_id = MAP_FRAME_;
        gsd.header.stamp = current_time_;
        gsd.lat_std_dev = lat_std_;
        gsd.lon_std_dev = lon_std_;
        gsd.alt_std_dev = alt_std_;
        gsd.north_velocity_std_dev = north_velocity_std_dev_;
        gsd.east_velocity_std_dev = east_velocity_std_dev_;
        gsd.up_velocity_std_dev = up_velocity_std_dev_;
        gsd.roll_std_dev = roll_std_dev_;
        gsd.pitch_std_dev = pitch_std_dev_;
        gsd.azimuth_std_dev = yaw_std_dev_;
        pub_std_dev_.publish(gsd);
      }
    }
    else if(nmea.at(0) == "#RAWIMUA")
    {
      if(nmea.size() == 18)
      {
        x_accel_ = stod(nmea.at(14)) * pow(2,-29) * 100;
        y_accel_ = stod(nmea.at(13)) * pow(2,-29) * 100;
        z_accel_ = stod(nmea.at(12)) * pow(2,-29) * 100;
        y_gyro_ = stod(nmea.at(16)) * pow(2,-33) * 100;
        z_gyro_ = stod(nmea.at(15)) * pow(2,-33) * 100;
        std::vector<std::string> ss = split(nmea[17], '*');
        x_gyro_ = stod(ss.at(0)) * pow(2,-33) * 100;
        ROS_INFO("RAWIMU is subscribed.");
        std::cout << "imu_acc : x," << stod(nmea.at(14)) << " y," << stod(nmea.at(13)) << " z," << stod(nmea.at(12)) << std::endl;
        std::cout << "imu_acc : x," << x_accel_ << " y," << y_accel_ << " z," << z_accel_ << std::endl;
        std::cout << "imu_gyro : x," << stod(nmea.at(17)) << " y," << stod(nmea.at(16)) << " z," << stod(nmea.at(15)) << std::endl;
        std::cout << "imu_gyro : x," << x_gyro_ << " y," << y_gyro_ << " z," << z_gyro_ << std::endl;

        double orie_x = stod(nmea.at(14)) * pow(2,-29) * 100;
        sensor_msgs::Imu imu;
        imu.header.frame_id = MAP_FRAME_;
        imu.header.stamp = current_time_;
        geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromRollPitchYaw(pva_roll_, -pva_pitch_, -pva_yaw_);
        imu.orientation = qua;
        imu.linear_acceleration.x = x_accel_;
        imu.linear_acceleration.y = y_accel_;
        imu.linear_acceleration.z = z_accel_;
        imu.angular_velocity.x = x_gyro_;
        imu.angular_velocity.y = y_gyro_;
        imu.angular_velocity.z = z_gyro_;
        pub_imu_.publish(imu);
      }
    }
    else if(nmea.at(0) == "#TIMEA")
    {
      if(nmea.size() == 20)
      {
        autoware_system_msgs::Date date;
        date.year = stoi(nmea.at(13));
        date.month = stoi(nmea.at(14));
        date.day = stoi(nmea.at(15));
        date.hour = stoi(nmea.at(16));
        date.min = stoi(nmea.at(17));
        date.sec = stoi(nmea.at(18)) / (float)1000.0;
        pub_time_.publish(date);
        //std_msgs::Float64 date;
        //date.data = hour*60.0*60.0 + min*60.0 + msec/1000.0;
        //pub_time_.publish(date);
      }
    }
    else if(nmea.at(0) == "$GNGGA")
    {
        std::cout << "GNGGA" << std::endl;
        double lat = stod(nmea.at(2)); std::cout << "lat : " << std::setprecision(16) << lat << std::endl;
        double lon = stod(nmea.at(4)); std::cout << "lon : " << std::setprecision(16) << lon << std::endl;
        double h = stod(nmea.at(9)); std::cout << "h : " << std::setprecision(16) << h << std::endl;
        geo_.set_llh_nmea_degrees(lat, lon, h);
        geo_vec_.insert(geo_vec_.begin(), geo_);
        if(geo_vec_.size() > 5) geo_vec_.resize(5);
        ROS_INFO("GNGGA is subscribed.");
        std::cout << "lat : " << std::setprecision(16) << geo_.y() << std::endl;
        std::cout << "lat : " << std::setprecision(16) << geo_.x() << std::endl;
        std::cout << "lat : " << std::setprecision(16) << geo_.z() << std::endl;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = MAP_FRAME_;
        pose.header.stamp = current_time_;
        pose.pose.position.x = geo_.y();
        pose.pose.position.y = geo_.x();
        pose.pose.position.z = geo_.z();
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
        pub1_.publish(pose);

        double sx = geo_.x() - last_geo_.x();
        double sy = geo_.y() - last_geo_.y();
        double sz = geo_.z() - last_geo_.z();
        double distance = sqrt(sx*sx + sy*sy + sz*sz);
        ros::Duration ros_time_diff = current_stamp - prev_stamp_;
        double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
        autoware_msgs::GnssSurfaceSpeed gss;
        gss.header.frame_id = MAP_FRAME_;
        gss.header.stamp = current_time_;
        gss.surface_speed = distance / time_diff;
        pub_surface_speed_.publish(gss);

        prev_stamp_ = current_stamp;
    }
  }catch (const std::exception &e)
  {
		ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence, ','), msg->header.stamp);

  double timeout = 10.0;
  if (fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
	double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
	double threshold = 0.2;
	//if (dt > threshold)
	{
	  ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
	  //createOrientation();
	  //publishPoseStamped();
	  publishTF();
	  last_geo_ = geo_;
	}
	return;
  }

  double e = 1e-2;
  if (fabs(orientation_time_ - position_time_) < e)
  {
	publishPoseStamped();
	publishTF();
	return;
  }
}

void Nmea2TFPoseNode::callbackGnssSelect(const std_msgs::Int32 &msg)
{
  if(msg.data == node_number_) use_node_number_ = true;
  else use_node_number_ = false;
}
}  // gnss_localizer
