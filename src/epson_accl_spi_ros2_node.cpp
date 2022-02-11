//==============================================================================
//
// 	epson_accl_spi_ros2_node.cpp
//     - ROS2 node for Epson ACCL sensor evaluation
//     - This program initializes the Epson ACCL and publishes ROS messages in
//       ROS topic /epson_imu as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//
//  [This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Copyright (c) 2020, 2021 Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

extern "C" {
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_spi.h"
#include "accel_epsonCommon.h"
}

#include <chrono>
#include <memory>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <string>


//=========================================================================
// Epson ACCL ROS2 C++ Node
//
// 1. Retrieves node parameters from launch file otherwise uses defaults.
// 2. Creates a publisher for ACCL messages
// 3. Sets up communication and initializes ACCL
// 4. Starts an internal timer to periodically check for incoming ACCL burst data
// 5. Formats and publishes ACCL messages until <CTRL-C>
//=========================================================================

using namespace std;
using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
 public:
  explicit ImuNode(const rclcpp::NodeOptions &op) : Node("accl_node", op) {
    ParseParams();

    // publisher
    imu_data_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_.c_str(), 10);

    Init();
    // Poll to check for ACCL burst read data @ 4000Hz
    // (atleast 2x the highest output rate 2000Hz)
    std::chrono::milliseconds ms((int)(1000.0 / poll_rate_));
    timer_ = this->create_wall_timer(ms, std::bind(&ImuNode::Spin, this));
  }

  ~ImuNode() {
    acclStop();
    spiRelease();
    gpioRelease();
    seRelease();
    RCLCPP_INFO(this->get_logger(), "Cleanup and shutown completed.");
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;

  // ACCL configuration settings
  struct EpsonAcclOptions options_ = {};

  // Buffer for scaled values of ACCL read burst
  struct EpsonAcclData epson_data_ = {};

  std::string frame_id_;
  std::string imu_topic_;
  double poll_rate_;
  const double GRAVITY = 9.80665;


  std::string prod_id_;
  std::string serial_id_;


  // NOTE: It is recommended to change ACCL settings by launch files instead of
  //       modifying directly here in the source code
  //       Refer to the launch file and datasheets for more info 
  void ParseParams() {
    std::string key;
    frame_id_ = "imu_link";
    imu_topic_ = "/epson_accl/data_raw";
    poll_rate_ = 4000.0;

    // Initialize defaults of ACCL settings
    options_.mesmod_sel = 0;
    options_.temp_stabil = 0;
    options_.ext_sel =
        0;  // 1=External Trigger Enable
    options_.ext_pol = 0;
    options_.drdy_on = 1;
    options_.drdy_pol = 1;  // 1 = active HIGH 0=active LOW
    options_.dout_rate = CMD_RATE200;
    options_.filter_sel = CMD_FIRTAP512FC16;
    options_.flag_out = 1;
    options_.temp_out = 1;
    options_.accel_out = 1;
    options_.count_out = 1;
    options_.checksum_out = 1;

    // Read parameters
    key = "frame_id";
    if (this->get_parameter(key, frame_id_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(),
                  frame_id_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  frame_id_.c_str());
    }

    key = "burst_polling_rate";
    if (this->get_parameter(key, poll_rate_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%.1f", key.c_str(), poll_rate_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%.1f",
                  key.c_str(), poll_rate_);
    }

    key = "mesmod_sel";
    if (this->get_parameter(key, options_.mesmod_sel)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.mesmod_sel);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.mesmod_sel);
    }

    key = "temp_stabil";
    if (this->get_parameter(key, options_.temp_stabil)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.temp_stabil);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.temp_stabil);
    }

    key = "ext_sel";
    if (this->get_parameter(key, options_.ext_sel)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.ext_sel);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.ext_sel);
    }

    key = "ext_pol";
    if (this->get_parameter(key, options_.ext_pol)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.ext_pol);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.ext_pol);
    }

    key = "dout_rate";
    if (this->get_parameter(key, options_.dout_rate)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(),
                  options_.dout_rate);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.dout_rate);
    }

    key = "filter_sel";
    if (this->get_parameter(key, options_.filter_sel)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(),
                  options_.filter_sel);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.filter_sel);
    }

    key = "flag_out";
    if (this->get_parameter(key, options_.flag_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.flag_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.flag_out);
    }

    key = "temp_out";
    if (this->get_parameter(key, options_.temp_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.temp_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.temp_out);
    }

    key = "accel_out";
    if (this->get_parameter(key, options_.accel_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.accel_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.accel_out);
    }

    key = "count_out";
    if (this->get_parameter(key, options_.count_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.count_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.count_out);
    }

    key = "checksum_out";
    if (this->get_parameter(key, options_.checksum_out)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t%d", key.c_str(), options_.checksum_out);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%d", key.c_str(),
                  options_.checksum_out);
    }

    key = "imu_topic";
    if (this->get_parameter(key, imu_topic_)) {
      RCLCPP_INFO(this->get_logger(), "%s:\t\t%s", key.c_str(),
                  imu_topic_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Not specified param %s. Set default value:\t%s", key.c_str(),
                  imu_topic_.c_str());
    }

  }

  void Init() {
    std::chrono::seconds sec(1);
    rclcpp::Rate one_sec(sec);

    while (rclcpp::ok() && !seInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the Seiko Epson HCL "
                  "layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !gpioInit()) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the GPIO layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !spiInit(SPI_MODE3, 500000)) {
      RCLCPP_WARN(this->get_logger(),
                  "Retry to initialize the SPI layer in 1 second period...");
      one_sec.sleep();
    }

    while (rclcpp::ok() && !InitAccl(options_)) {
      RCLCPP_WARN(this->get_logger(), "Retry to initialize the ACCL...");
      one_sec.sleep();
    }

    IdentifyDevice();
    acclStart();
  }

  bool InitAccl(const struct EpsonAcclOptions &options_) {
    RCLCPP_INFO(this->get_logger(), "Checking sensor power on status...");
    if (!acclPowerOn()) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: failed to power on Sensor. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Sensor...");
    if (!acclInitOptions(options_)) {
      RCLCPP_WARN(this->get_logger(),
                  "Error: could not initialize Epson Sensor. Exiting...");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Epson ACCL initialized.");
    return true;
  }

  std::string get_prod_id() {
    unsigned short prod_id1 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID1, false);
    unsigned short prod_id2 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID2, false);
    unsigned short prod_id3 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID3, false);
    unsigned short prod_id4 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID4, false);

    char myarray[] = {
        static_cast<char>(prod_id1), static_cast<char>(prod_id1 >> 8),
        static_cast<char>(prod_id2), static_cast<char>(prod_id2 >> 8),
        static_cast<char>(prod_id3), static_cast<char>(prod_id3 >> 8),
        static_cast<char>(prod_id4), static_cast<char>(prod_id4 >> 8)};
    std::string prod_id(myarray);
    return prod_id;
  }

  std::string get_serial_id() {
    unsigned short ser_num1 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, false);
    unsigned short ser_num2 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, false);
    unsigned short ser_num3 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, false);
    unsigned short ser_num4 =
        registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, false);

    char myarray[] = {
        static_cast<char>(ser_num1), static_cast<char>(ser_num1 >> 8),
        static_cast<char>(ser_num2), static_cast<char>(ser_num2 >> 8),
        static_cast<char>(ser_num3), static_cast<char>(ser_num3 >> 8),
        static_cast<char>(ser_num4), static_cast<char>(ser_num4 >> 8)};
    std::string ser_num(myarray);
    return ser_num;
  }

  void IdentifyDevice() {
    RCLCPP_INFO(this->get_logger(), "PRODUCT ID:\t%s", get_prod_id().c_str());
    RCLCPP_INFO(this->get_logger(), "SERIAL ID:\t%s", get_serial_id().c_str());
  }

  void PubImuData() {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    for (int i = 0; i < 9; i++) {
      imu_msg->orientation_covariance[i] = 0;
      imu_msg->angular_velocity_covariance[i] = 0;
      imu_msg->linear_acceleration_covariance[i] = 0;
    }
    imu_msg->orientation_covariance[0] = -1;
    imu_msg->header.frame_id = frame_id_;

    while (rclcpp::ok()) {
      // Check DRDY is active before burst read
      if (acclDataReady()) {
        // Call to read and post process ACCL stream
        // Will return 0 if data incomplete or checksum error
        if (acclDataReadBurstNOptions(options_, &epson_data_)) {
          imu_msg->header.stamp = this->now();
        
          // Linear acceleration
          imu_msg->linear_acceleration.x = epson_data_.accel_x * GRAVITY;
          imu_msg->linear_acceleration.y = epson_data_.accel_y * GRAVITY;
          imu_msg->linear_acceleration.z = epson_data_.accel_z * GRAVITY;
        
          imu_data_pub_->publish(*imu_msg);
        }
      }
    }
  }

  void Spin() { PubImuData(); }
};  // end of class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer, which ensures a sync of all console
  // output even from a launch file.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto imu_node = std::make_shared<ImuNode>(options);

  rclcpp::spin(imu_node);
  rclcpp::shutdown();
  return 0;
}
