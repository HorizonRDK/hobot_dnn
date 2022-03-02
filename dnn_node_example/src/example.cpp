// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "include/fasterrcnn_body_det_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "This is dnn node example!");

  std::string node_name = "body_det";
  FasterRcnnBodyDetNode body_det_node(node_name);
  if (body_det_node.Init() == 0) {
    if (body_det_node.Run() != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"),
        "Run FasterRcnnBodyDetNode failed!");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("example"),
        "Run FasterRcnnBodyDetNode done!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Init FasterRcnnBodyDetNode failed!");
  }

  rclcpp::shutdown();
  return 0;
}
