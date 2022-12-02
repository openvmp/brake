/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-12-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "brake/fake_node.hpp"

namespace brake {

FakeNode::FakeNode() : rclcpp::Node("brake_fake") {
  intf_ = std::make_shared<FakeInterface>(this, this->get_namespace());
}

}  // namespace brake
