/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_brake/fake_node.hpp"

namespace remote_brake {

FakeNode::FakeNode() : rclcpp::Node("brake_fake") {
  intf_ = std::make_shared<FakeInterface>(this);
}

}  // namespace remote_brake
