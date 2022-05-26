// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_ssd.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int SsdPostProcess::SetOutParser(Model* model_manage) {
  output_index_ = model_output_count_ - 1;
  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<hobot::dnn_node::SSDAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // set ssd paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "ssd_branch");
  for (int i = 0; i < output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("ssd");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> ssd_out_parser =
      std::make_shared<hobot::dnn_node::SSDOutputParser>();
  model_manage->SetOutputParser(output_index_, ssd_out_parser);
  return 0;
}
