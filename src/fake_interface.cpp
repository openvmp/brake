/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_brake/fake_interface.hpp"

#include <functional>

#include "remote_brake/srv/command.hpp"

namespace remote_brake {

FakeInterface::FakeInterface(rclcpp::Node *node, bool engaged_by_default)
    : Interface(node, engaged_by_default) {}

void FakeInterface::command_handler_real_(
    const std::shared_ptr<srv::Command::Request> request,
    std::shared_ptr<srv::Command::Response> response) {
  (void)request;
  (void)response;

  // Nothing needs to go into response
  return;
}

}  // namespace remote_brake
