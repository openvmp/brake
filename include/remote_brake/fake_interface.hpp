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

#include "rclcpp/rclcpp.hpp"
#include "remote_brake/interface.hpp"
#include "remote_brake/srv/command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace remote_brake {

class FakeInterface : public Interface {
 public:
  FakeInterface(rclcpp::Node *node, bool engaged_by_default = true);
  virtual ~FakeInterface() = default;

 protected:
  virtual void command_handler_real_(
      const std::shared_ptr<srv::Command::Request> request,
      std::shared_ptr<srv::Command::Response> response) override;
};

}  // namespace remote_brake

#endif  // OPENVMP_BRAKE_FAKE_INTERFACE_H
