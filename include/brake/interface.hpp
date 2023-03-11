/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_BRAKE_INTERFACE_H
#define OPENVMP_BRAKE_INTERFACE_H

#include <map>
#include <memory>
#include <string>

#include "brake/srv/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace brake {

class Interface {
 public:
  Interface(rclcpp::Node *node, bool engaged_by_default = true);
  virtual ~Interface() {}

  bool get_engaged() const { return engaged_; }
  void set_engaged(bool engaged) { engaged_ = engaged; }

  static const std::string SRV_COMMAND;
  static const std::string PUB_LAST_CHANGED;
  static const std::string PUB_LAST_ENGAGED;

 protected:
  rclcpp::Node *node_;
  rclcpp::Parameter interface_prefix_;
  bool engaged_;

  std::string get_prefix_();

  virtual void command_handler_real_(
      const std::shared_ptr<brake::srv::Command::Request> request,
      std::shared_ptr<brake::srv::Command::Response> response) = 0;

 private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<brake::srv::Command>::SharedPtr command_;

  std::mutex lock_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr last_changed_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr last_engaged_;

  void command_handler_(
      const std::shared_ptr<brake::srv::Command::Request> request,
      std::shared_ptr<brake::srv::Command::Response> response);
};

}  // namespace brake

#endif  // OPENVMP_BRAKE_INTERFACE_H
