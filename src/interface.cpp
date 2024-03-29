/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_brake/interface.hpp"

#include <functional>

#include "remote_brake/srv/command.hpp"

namespace remote_brake {

const std::string Interface::SRV_COMMAND = "/command";
const std::string Interface::PUB_LAST_CHANGED = "/last_changed";
const std::string Interface::PUB_LAST_ENGAGED = "/last_engaged";

Interface::Interface(rclcpp::Node *node, bool engaged_by_default)
    : node_{node}, engaged_{engaged_by_default} {
  RCLCPP_DEBUG(node_->get_logger(),
               "Initializing the ROS2 interface to the brake");

  node->declare_parameter("brake_prefix",
                          "/brake/" + std::string(node_->get_name()));
  node->get_parameter("brake_prefix", interface_prefix_);

  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto prefix = get_prefix_();
  command_ = node_->create_service<srv::Command>(
      prefix + SRV_COMMAND,
      std::bind(&Interface::command_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
  RCLCPP_DEBUG(node_->get_logger(), "Created the service...");

  last_changed_ = node->create_publisher<std_msgs::msg::UInt64>(
      prefix + PUB_LAST_CHANGED, 10);
  last_engaged_ = node->create_publisher<std_msgs::msg::Bool>(
      prefix + PUB_LAST_ENGAGED, 10);
  RCLCPP_DEBUG(node_->get_logger(), "Created the publisher...");

  std_msgs::msg::Bool msg_engaged;
  msg_engaged.data = engaged_;
  last_engaged_->publish(msg_engaged);
  RCLCPP_DEBUG(node_->get_logger(),
               "Finished initializing the ROS2 interface to the brake");
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

void Interface::command_handler_(
    const std::shared_ptr<srv::Command::Request> request,
    std::shared_ptr<srv::Command::Response> response) {
  auto now = rclcpp::Time().nanoseconds();
  std_msgs::msg::UInt64 msg_now;
  msg_now.data = now;

  std_msgs::msg::Bool msg_engaged;
  msg_engaged.data = request->engage;

  engaged_ = request->engage;
  last_changed_->publish(msg_now);
  last_engaged_->publish(msg_engaged);

  auto prefix = get_prefix_();
  if (engaged_) {
    RCLCPP_INFO(node_->get_logger(), "Engaged the brake: %s", prefix.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "Disengaged the brake: %s",
                prefix.c_str());
  }

  command_handler_real_(request, response);
}

}  // namespace remote_brake
