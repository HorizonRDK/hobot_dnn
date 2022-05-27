// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_classification.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int ClassificationPostProcess::SetOutParser(Model* model_manage)
{
  RCLCPP_INFO(rclcpp::get_logger("ClassificationPostProcess"),
                "Set out parser");
  output_index_ = model_output_count_ - 1;

  auto ClsParser =
    std::make_shared<hobot::dnn_node::ClassficationOutputParser>();
  auto ret = ClsParser->InitClassNames(cls_name_file_);
  if (0 != ret)
  {
    return ret;
  }

  std::shared_ptr<OutputParser> parser =
        std::dynamic_pointer_cast<OutputParser>(ClsParser);
  model_manage->SetOutputParser(output_index_, parser);

  return 0;
}

ai_msgs::msg::PerceptionTargets::UniquePtr
ClassificationPostProcess::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output)
{
  const auto &outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("ClassificationPostProcess"),
              "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ClassificationPostProcess"),
                "Invalid outputs");
    return nullptr;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  auto *det_result =
      dynamic_cast<Dnn_Parser_Result *>(outputs[output_index_].get());
  if (!det_result) {
    RCLCPP_INFO(rclcpp::get_logger("ClassificationPostProcess"),
                "invalid cast");
    return 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("ClassificationPostProcess"),
              "out cls size: %d",
              det_result->perception.cls.size());
  for (auto &cls : det_result->perception.cls)
  {
    std::string clsname = cls.class_name;
    std::stringstream ss;
    ss << "class type:" << cls.class_name
       << ", score:" << cls.score;
    RCLCPP_INFO(rclcpp::get_logger("ClassificationPostProcess"),
                  "%s", ss.str().c_str());

    auto xmin = model_input_width_ / 2;
    auto ymin = model_input_height_ / 2;
    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(xmin);
    roi.rect.set__y_offset(ymin);
    roi.rect.set__width(0);
    roi.rect.set__height(0);

    ai_msgs::msg::Target target;
    target.set__type(cls.class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }
  return pub_data;
}