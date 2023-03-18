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

#include "brake/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace brake {

class FakeFactory {
 public:
  static std::shared_ptr<brake::Interface> New(rclcpp::Node *node);
};

}  // namespace brake

#endif  // OPENVMP_BRAKE_FAKE_FACTORY_H