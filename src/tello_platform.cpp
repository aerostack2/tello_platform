/*!*******************************************************************************************
 *  \file       tello_platform.cpp
 *  \brief      Implements the functionality and communication with the tello drone.
 *  \authors    Daniel Fernández Sánchez
 *              Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "tello_platform.hpp"

TelloPlatform::TelloPlatform() : as2::AerialPlatform() {
  this->tello      = new Tello;
  this->connected_ = true;

  this->declare_parameter<double>("minSpeed", 0.02);
  this->declare_parameter<double>("maxSpeed", 15.0);
  this->get_parameter("minSpeed", min_speed_);
  this->get_parameter("maxSpeed", max_speed_);
  configureSensors();

  // TODO: set command freq

  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
    recvIMU();
    recvBattery();
    recvBarometer();
  });
}

TelloPlatform::~TelloPlatform() {
  this->tello->~Tello();
  delete (this->tello);
}

// *********************************************************
// ***************** Aerial Platform Methods ***************
// *********************************************************
void TelloPlatform::configureSensors() {
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_ptr_    = std::make_unique<as2::sensors::Battery>("battery", this);
  barometer_ptr_  = std::make_unique<as2::sensors::Barometer>("barometer", this);
  // TODO: De donde saco la odometria
  odometry_ptr_ = std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odometry", this);
  // image_ptr_ = std::make_unique<as2::sensors::Camera>("camera", this);
}

bool TelloPlatform::ownSendCommand() {
  const uint8_t pose_control_mode = as2::control_mode::convertToUint8t(
      as2_msgs::msg::ControlMode::POSITION, as2_msgs::msg::ControlMode::YAW_ANGLE,
      as2_msgs::msg::ControlMode::BODY_FLU_FRAME);

  const uint8_t speed_control_mode = as2::control_mode::convertToUint8t(
      as2_msgs::msg::ControlMode::SPEED, as2_msgs::msg::ControlMode::YAW_SPEED,
      as2_msgs::msg::ControlMode::BODY_FLU_FRAME);

  switch (as2::control_mode::convertToUint8t(this->getControlMode())) {
    case pose_control_mode: {
      double x_m, y_m, z_m, yaw_rad;
      x_m     = this->command_pose_msg_.pose.position.x;     // m
      y_m     = this->command_pose_msg_.pose.position.y;     // m
      z_m     = this->command_pose_msg_.pose.position.z;     // m
      yaw_rad = this->command_pose_msg_.pose.orientation.z;  // rad

      std::vector<double> new_ref = {x_m, y_m, z_m, yaw_rad};
      if (reference_point_ != new_ref) {
        double x   = std::clamp(x_m, min_linear_pose_, max_linear_pose_) * 100;  // cm
        double y   = std::clamp(y_m, min_linear_pose_, max_linear_pose_) * 100;  // cm
        double z   = std::clamp(z_m, min_linear_pose_, max_linear_pose_) * 100;  // cm
        double yaw = normalizeDegrees(yaw_rad * 180 / M_PI);                     // degrees

        bool xSend   = tello->x_motion(x);
        bool ySend   = tello->y_motion(y);
        bool zSend   = tello->z_motion(z);
        bool yawSend = tello->yaw_twist(yaw);

        if (!xSend) {
          RCLCPP_ERROR(this->get_logger(), "Sending X position command failed.");
          return false;
        } else if (!ySend) {
          RCLCPP_ERROR(this->get_logger(), "Sending Y position command failed.");
          return false;
        } else if (!zSend) {
          RCLCPP_ERROR(this->get_logger(), "Sending Z position failed.");
          return false;
        } else if (!yawSend) {
          RCLCPP_ERROR(this->get_logger(), "Sending Yaw orientation failed.");
          return false;
        }
        reference_point_ = new_ref;
      }
      return true;
    }
    case speed_control_mode: {
      double vx_   = this->command_twist_msg_.twist.linear.x;   // m/s
      double vy_   = this->command_twist_msg_.twist.linear.y;   // m/s
      double vz_   = this->command_twist_msg_.twist.linear.z;   // m/s
      double vyaw_ = this->command_twist_msg_.twist.angular.z;  // rad/s

      std::vector<double> new_ref = {vx_, vy_, vz_, vyaw_};
      if (reference_speed_ != new_ref) {
        double vx   = std::clamp(vx_, min_speed_, max_speed_);
        vx          = normalize(vx, min_speed_, max_speed_);  // %
        double vy   = std::clamp(vy_, min_speed_, max_speed_);
        vy          = normalize(vy, min_speed_, max_speed_);  // %
        double vz   = std::clamp(vz_, min_speed_, max_speed_);
        vz          = normalize(vz, min_speed_, max_speed_);  // %
        double vyaw = std::clamp(vyaw_, min_speed_, max_speed_);
        vyaw        = normalize(vyaw, min_speed_, max_speed_);  // %

        bool speed_send = tello->speedMotion(vx, vy, vz, vyaw);

        if (!speed_send) {
          RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending control speed command");
          return false;
        }

        reference_speed_ = new_ref;
      }
      return true;
    }
  };
  return false;
}

bool TelloPlatform::ownSetArmingState(bool state) {
  std::pair<bool, std::string> resp;
  bool ret = state;

  if (state && !this->connected_) {
    resp = tello->sendCommand("command");  // TODO: send command --> bool
    if (resp.first == true and resp.second != "Error") {
      ret = true;
    } else {
      ret = false;
    }
  }
  return ret;
}

bool TelloPlatform::ownSetOffboardControl(bool offboard) {
  RCLCPP_DEBUG(this->get_logger(), "Offboard status changed to %d", offboard);
  return this->connected_;
}

bool TelloPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) {
  RCLCPP_DEBUG(this->get_logger(), "New control mode: %s",
               as2::control_mode::controlModeToString(msg).c_str());
  this->control_mode_in_ = msg;
  return true;
}

bool TelloPlatform::ownTakeoff() {
  if (this->connected_) {
    tello->sendCommand("takeoff");  // TODO: return if takeoff ok

    reference_point_ = {0, 0, tello->getHeight(), 0};
    return true;
  }
  return false;
}

bool TelloPlatform::ownLand() {
  if (this->connected_) {
    tello->sendCommand("land");  // TODO: return if land ok

    return true;
  }
  return false;
}

// **********************************************************
// ******************** CALLBACK METHODS ********************
// **********************************************************
void TelloPlatform::recvIMU() {
  std::vector<coordinates> imu_info = tello->getIMU();

  tf2::Quaternion q;
  float roll_rad  = float(imu_info[0].x * M_PI) / 180;
  float pitch_rad = float(imu_info[0].y * M_PI) / 180;
  float yaw_rad   = float(imu_info[0].z * M_PI) / 180;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp          = this->get_clock()->now();
  imu_msg.header.frame_id       = "imu";
  imu_msg.orientation.x         = q.x();
  imu_msg.orientation.y         = q.y();
  imu_msg.orientation.z         = q.z();
  imu_msg.orientation.w         = q.w();
  imu_msg.angular_velocity.x    = imu_info[1].x;
  imu_msg.angular_velocity.y    = imu_info[1].y;
  imu_msg.angular_velocity.z    = imu_info[1].z;
  imu_msg.linear_acceleration.x = imu_info[2].x;
  imu_msg.linear_acceleration.y = imu_info[2].y;
  imu_msg.linear_acceleration.z = imu_info[2].z;

  imu_sensor_ptr_->updateData(imu_msg);
}

void TelloPlatform::recvBattery() {
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.percentage   = tello->getBattery();

  battery_ptr_->updateData(battery_msg);
}

void TelloPlatform::recvBarometer() {
  sensor_msgs::msg::FluidPressure barometer_msg;
  barometer_msg.header.stamp   = this->get_clock()->now();
  barometer_msg.fluid_pressure = tello->getBarometer();
  barometer_ptr_->updateData(barometer_msg);
}

// **********************************************************
// ******************** AUXILIAR METHODS ********************
// **********************************************************
/**
 * @brief -100 .. 100
 *
 * @param value
 * @param min_value
 * @param max_value
 * @return double
 */
double TelloPlatform::normalize(double value, double min_value, double max_value) {
  value = 2 * (value - min_value) / (max_value - min_value) - 1;
  return 100 * value;
}

/**
 * @brief 0 .. 359
 *
 * @param value
 * @return double
 */
double TelloPlatform::normalizeDegrees(double value) {
  while (abs(value) > 360) {
    value = abs(value) - 360;
  }
  return value;
}

/*void TelloPlatform::recvVideo(){
  Mat frame = tello->getFrame();
  if (!frame.empty())
  {
      imshow("CTello Stream", frame);
  }

}*/
