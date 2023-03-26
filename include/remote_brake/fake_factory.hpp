/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-16
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_BRAKE_FAKE_FACTORY_H
#define OPENVMP_BRAKE_FAKE_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_brake/interface.hpp"

namespace remote_brake {

class FakeFactory {
 public:
  static std::shared_ptr<Interface> New(rclcpp::Node *node);
};

}  // namespace remote_brake

#endif  // OPENVMP_BRAKE_FAKE_FACTORY_H