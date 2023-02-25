/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-11-10
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_BRAKE_FAKE_INTERFACE_H
#define OPENVMP_BRAKE_FAKE_INTERFACE_H

#include <map>
#include <memory>
#include <string>

#include "brake/interface.hpp"
#include "brake/srv/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace brake {

class FakeInterface : public Interface {
 public:
  FakeInterface(rclcpp::Node *node, bool engaged_by_default = true);
  virtual ~FakeInterface() = default;

 protected:
  virtual void command_handler_real_(
      const std::shared_ptr<brake::srv::Command::Request> request,
      std::shared_ptr<brake::srv::Command::Response> response) override;
};

}  // namespace brake

#endif  // OPENVMP_BRAKE_FAKE_INTERFACE_H
