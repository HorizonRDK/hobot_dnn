// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "include/utils/null_output_parser.h"

int32_t nullOutputParser::Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor)
{
  RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
    "FasterRcnnKpsOutputParser parse start");
  return 0;
}
