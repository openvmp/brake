/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "brake/fake_interface.hpp"

#include <functional>

#include "brake/srv/command.hpp"

namespace brake {

FakeInterface::FakeInterface(rclcpp::Node *node,
                             bool engaged_by_default)
    : Interface(node, engaged_by_default) {}

void FakeInterface::command_handler_real_(
    const std::shared_ptr<brake::srv::Command::Request> request,
    std::shared_ptr<brake::srv::Command::Response> response) {
  (void)request;
  (void)response;

  // Nothing needs to go into response
  return;
}

}  // namespace brake
