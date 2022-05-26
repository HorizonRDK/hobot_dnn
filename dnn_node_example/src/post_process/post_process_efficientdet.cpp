// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_efficientdet.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int EfficientDetPostProcess::SetOutParser(Model* model_manage)
{
  RCLCPP_INFO(rclcpp::get_logger("EfficientDetPostProcess"),
                "Set out parser");
  output_index_ = model_output_count_ - 1;
  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<hobot::dnn_node::EfficientDetAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // set paser
  auto efficientDetParser =
    std::make_shared<hobot::dnn_node::EfficientDetOutputParser>();
  auto ret = efficientDetParser->Setdequanti_file(dequanti_file_);
  if (0 != ret)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EfficientDetPostProcess"),
                "Set dequanti file: %s failed!!", dequanti_file_.c_str());
  }

  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "efficientdet_branch");
  for (int i = 0; i < output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("efficientdet");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> parser =
        std::dynamic_pointer_cast<OutputParser>(efficientDetParser);
  model_manage->SetOutputParser(output_index_, parser);
  return 0;
}

void EfficientDetPostProcess::SetDequanti_file(const std::string &dequanti_file)
{
  this->dequanti_file_ = dequanti_file;
}
