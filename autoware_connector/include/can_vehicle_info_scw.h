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

#ifndef CAN_VEHICLE_INFO_H
#define CAN_VEHICLE_INFO_H

namespace autoware_connector
{

const double handle_angle_right_max = 730;
const double handle_angle_left_max = 765;
const double wheelrad_to_steering_can_value_left = 20935.4958411006;//20639.7444769791;//20935.4958411006;//20691.8161699557;//20952.8189547718;
const double wheelrad_to_steering_can_value_right = 20791.4464661611;//20066.8329952857;//20791.4464661611;//20802.5331916036;//20961.415734248;
const double wheelrad_to_steering_can_value_left_intercept = 0;//277.358467233321;
const double wheelrad_to_steering_can_value_right_intercept = 0;//111.715455085083;
const double angle_velocity_correction_slope_ = -0.0150859385;
const double angle_velocity_correction_intersept_ = 1.1773688546;
const double angle_magn_right = handle_angle_right_max / 15000;
const double angle_magn_left = handle_angle_left_max / 15000;
//const double angle_magn_right = wheelrad_to_steering_can_value_right / handle_angle_right_max;
//const double angle_magn_left = wheelrad_to_steering_can_value_left / handle_angle_left_max;

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}
struct VehicleInfo
{
  bool is_stored;
  double wheel_base;
  double minimum_turning_radius;
  double maximum_steering_wheel_angle_deg;

  VehicleInfo()
  {
    is_stored = false;
    wheel_base = 0.0;
    minimum_turning_radius = 0.0;
    maximum_steering_wheel_angle_deg = 0.0;
  }
  double convertSteeringAngleToAngularVelocity(const double cur_vel_mps, const double cur_angle_rad)  // rad/s
  {
    return is_stored ? tan(cur_angle_rad) * cur_vel_mps / wheel_base : 0;
  }
  double convertSteeringAngleToAngularVelocity_microbus(const double cur_vel_mps, const double angle_actual)
  {
      double wheel_val;
      if(angle_actual > 0)
      {
          wheel_val = (angle_actual - wheelrad_to_steering_can_value_left_intercept) / wheelrad_to_steering_can_value_left;
      }
      else
      {
          wheel_val = (angle_actual - wheelrad_to_steering_can_value_right_intercept) / wheelrad_to_steering_can_value_right;
      }
      //std::cout << "wheel : " << wheel_val << std::endl;
      double ret = tan(wheel_val) * cur_vel_mps / wheel_base;
      //ros::Time time =  ros::Time::now();
      //std::cout << "aaa," << cur_vel_mps << "," << ret << "," << time.sec<< "," << time.nsec << "," << wheel_val << std::endl;
      //double hosei = (angle_velocity_correction_slope_ * cur_vel_mps + angle_velocity_correction_intersept_);
      //if(hosei < 1) hosei = 1;
      //ret *= hosei;
      return is_stored ? ret : 0;
  }
  double getCurrentSteeringAngle(const double steering_wheel_angle_rad)  // steering wheel [rad] -> steering [rad]
  {
    return is_stored ?
               steering_wheel_angle_rad * getMaximumSteeringWheelAngle() / deg2rad(maximum_steering_wheel_angle_deg) :
               0;
  }
  double getMaximumSteeringWheelAngle()  // radian
  {
    return is_stored ? asin(wheel_base / minimum_turning_radius) : 0;
  }
};
}
#endif  // CAN_VEHICLE_INFO_H
