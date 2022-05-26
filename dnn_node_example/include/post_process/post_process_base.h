// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node_data.h"
#include "rclcpp/rclcpp.hpp"
#include "util/output_parser/perception_common.h"

#ifndef POST_PROCESS_BASE_H_
#define POST_PROCESS_BASE_H_

using hobot::dnn_node::DnnNodeOutput;

using hobot::dnn_node::DNNResult;
using hobot::dnn_node::Model;

using hobot::dnn_node::OutputDescription;
using hobot::dnn_node::OutputParser;

using ai_msgs::msg::PerceptionTargets;

class PostProcessBase {
 public:
  explicit PostProcessBase(int32_t model_output_count) {
    model_output_count_ = model_output_count;
  }
  virtual ~PostProcessBase() {}

  virtual ai_msgs::msg::PerceptionTargets::UniquePtr PostProcess(
      const std::shared_ptr<DnnNodeOutput>& outputs);

  virtual int SetOutParser(Model* model_manage) = 0;

  void SetModelInputWH(const int &width, const int &height)
  {
    model_input_height_ = height;
    model_input_width_ = width;
  }

 protected:
  int32_t model_output_count_ = 3;
  int32_t output_index_ = 2;
  int model_input_width_ = 0;
  int model_input_height_ = 0;
};

#endif  // POST_PROCESS_BASE_H_
