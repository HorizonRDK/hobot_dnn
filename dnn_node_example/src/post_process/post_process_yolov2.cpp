// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_yolov2.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int Yolov2PostProcess::SetOutParser(Model* model_manage) {
  RCLCPP_INFO(rclcpp::get_logger("Yolov2PostProcess"), "Set out parser");
  output_index_ = model_output_count_ - 1;
  std::shared_ptr<OutputParser> parser =
      std::make_shared<hobot::dnn_node::Yolo2OutputParser>();
  model_manage->SetOutputParser(output_index_, parser);

  return 0;
}
