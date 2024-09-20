// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#ifndef TURTLEBOT3_NODE__TURTLEBOT3_HPP_
#define TURTLEBOT3_NODE__TURTLEBOT3_HPP_

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <turtlebot3_msgs/msg/sensor_state.hpp>

#include "turtlebot3_node/control_table.hpp"
#include "turtlebot3_node/devices/devices.hpp"
#include "turtlebot3_node/devices/motor_power.hpp"
#include "turtlebot3_node/devices/reset.hpp"
#include "turtlebot3_node/devices/sound.hpp"
#include "turtlebot3_node/dynamixel_sdk_wrapper.hpp"
#include "turtlebot3_node/odometry.hpp"
#include "turtlebot3_node/sensors/battery_state.hpp"
#include "turtlebot3_node/sensors/imu.hpp"
#include "turtlebot3_node/sensors/joint_state.hpp"
#include "turtlebot3_node/sensors/sensor_state.hpp"
#include "turtlebot3_node/sensors/sensors.hpp"

namespace robotis
{
namespace turtlebot3
{
extern const ControlTable extern_control_table;
class TurtleBot3 : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

    typedef struct
  {
    bool use_stamped_vel;
    float cmd_vel_timeout;
  } Control;

  explicit TurtleBot3(const std::string & usb_port);
  virtual ~TurtleBot3() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void init_control();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);
  void drive_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;
  Control ctrl_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors *> sensors_;
  std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr drive_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_ = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_echo_pub_ = nullptr;

  geometry_msgs::msg::TwistStamped::SharedPtr last_cmd_vel_msg_ = nullptr;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace turtlebot3
}  // namespace robotis
#endif  // TURTLEBOT3_NODE__TURTLEBOT3_HPP_
