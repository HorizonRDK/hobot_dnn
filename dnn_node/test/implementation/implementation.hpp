// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class TestImpl : public ::testing::Test {
 protected:
  void SetUp() {
    node_ = rclcpp::Node::make_shared("test_impl");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestImpl, Init) {
}
