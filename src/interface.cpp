/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "brake/interface.hpp"

#include <functional>

#include "brake/srv/command.hpp"

namespace brake {

const std::string Interface::SRV_COMMAND = "/command";
const std::string Interface::PUB_LAST_CHANGED = "/last_changed";
const std::string Interface::PUB_LAST_ENGAGED = "/last_engaged";

Interface::Interface(rclcpp::Node *node, const std::string &interface_prefix,
                     bool engaged_by_default)
    : node_{node},
      interface_prefix_{interface_prefix},
      engaged_{engaged_by_default} {
  RCLCPP_DEBUG(node_->get_logger(),
               "Initializing the ROS2 interface to the brake");

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  command_ = node_->create_service<brake::srv::Command>(
      interface_prefix_ + SRV_COMMAND,
      std::bind(&Interface::command_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  RCLCPP_DEBUG(node_->get_logger(), "Created the service...");

  last_changed_ = node->create_publisher<std_msgs::msg::UInt64>(
      interface_prefix_ + PUB_LAST_CHANGED, 10);
  last_engaged_ = node->create_publisher<std_msgs::msg::Bool>(
      interface_prefix_ + PUB_LAST_ENGAGED, 10);
  RCLCPP_DEBUG(node_->get_logger(), "Created the publisher...");

  std_msgs::msg::Bool msg_engaged;
  msg_engaged.data = engaged_;
  last_engaged_->publish(msg_engaged);
  RCLCPP_DEBUG(node_->get_logger(),
               "Finished initializing the ROS2 interface to the brake");
}

void Interface::command_handler_(
    const std::shared_ptr<brake::srv::Command::Request> request,
    std::shared_ptr<brake::srv::Command::Response> response) {
  command_handler_real_(request, response);

  auto now = rclcpp::Time().nanoseconds();
  std_msgs::msg::UInt64 msg_now;
  msg_now.data = now;

  std_msgs::msg::Bool msg_engaged;
  msg_engaged.data = request->engage;

  engaged_ = request->engage;
  last_changed_->publish(msg_now);
  last_engaged_->publish(msg_engaged);
  if (engaged_) {
    RCLCPP_INFO(node_->get_logger(), "Engaged the brake: %s",
                interface_prefix_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "Disengaged the brake: %s",
                interface_prefix_.c_str());
  }
}

}  // namespace brake