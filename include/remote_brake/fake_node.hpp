/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_BRAKE_FAKE_NODE_H
#define OPENVMP_BRAKE_FAKE_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_brake/fake_interface.hpp"

namespace remote_brake {

class FakeNode : public rclcpp::Node {
 public:
  FakeNode();

  std::shared_ptr<FakeInterface> intf_;
};

}  // namespace remote_brake

#endif  // OPENVMP_BRAKE_FAKE_NODE_H
