// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_fasterrcnn.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int FasterRcnnPostProcess::SetOutParser(Model* model_manage) {
  // set box paser
  // 使用easy dnn中定义的FaceHandDetectionOutputParser后处理进行更新
  std::shared_ptr<OutputParser> box_out_parser =
      std::make_shared<hobot::easy_dnn::FaceHandDetectionOutputParser>();
  model_manage->SetOutputParser(box_output_index_, box_out_parser);

  // set kps paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, kps_output_index_, "body_kps_branch");
  output_desc->GetDependencies().push_back(box_output_index_);
  output_desc->SetType("body_kps");
  model_manage->SetOutputDescription(output_desc);
  auto parser_para = std::make_shared<FasterRcnnKpsParserPara>();
  // get kps parser para from model
  hbDNNTensorProperties tensor_properties;
  model_manage->GetOutputTensorProperties(tensor_properties, kps_output_index_);
  parser_para->aligned_kps_dim.clear();
  parser_para->kps_shifts_.clear();
  for (int i = 0; i < tensor_properties.alignedShape.numDimensions; i++) {
    parser_para->aligned_kps_dim.push_back(
        tensor_properties.alignedShape.dimensionSize[i]);
  }
  for (int i = 0; i < tensor_properties.shift.shiftLen; i++) {
    parser_para->kps_shifts_.push_back(
        static_cast<uint8_t>(tensor_properties.shift.shiftData[i]));
  }

  std::stringstream ss;
  ss << "aligned_kps_dim:";
  for (const auto& val : parser_para->aligned_kps_dim) {
    ss << " " << val;
  }
  ss << "\nkps_shifts: ";
  for (const auto& val : parser_para->kps_shifts_) {
    ss << " " << val;
  }
  ss << "\n";
  RCLCPP_INFO(
      rclcpp::get_logger("FasterRcnnPostProcess"), "%s", ss.str().c_str());

  std::shared_ptr<OutputParser> kps_out_parser =
      std::make_shared<FasterRcnnKpsOutputParser>(parser_para);
  model_manage->SetOutputParser(kps_output_index_, kps_out_parser);

  return 0;
}

ai_msgs::msg::PerceptionTargets::UniquePtr FasterRcnnPostProcess::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  const auto& outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("FasterRcnnPostProcess"),
              "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("FasterRcnnPostProcess"),
                 "Invalid outputs");
    return nullptr;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());

  // key is model output index
  std::vector<ai_msgs::msg::Roi> rois;
  std::vector<ai_msgs::msg::Point> body_kps;

  auto* filter2d_result =
      dynamic_cast<Filter2DResult*>(outputs[box_output_index_].get());
  if (!filter2d_result) {
    RCLCPP_INFO(rclcpp::get_logger("FasterRcnnPostProcess"), "invalid cast");
    return nullptr;
  }

  RCLCPP_INFO(rclcpp::get_logger("FasterRcnnPostProcess"),
              "out box size: %d",
              filter2d_result->boxes.size());
  for (auto& rect : filter2d_result->boxes) {
    if (rect.left < 0) rect.left = 0;
    if (rect.top < 0) rect.top = 0;
    std::stringstream ss;
    ss << "rect: " << rect.left << " " << rect.top << " " << rect.right << " "
       << rect.bottom << ", " << rect.conf;
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.set__type("body");
    roi.rect.set__x_offset(rect.left);
    roi.rect.set__y_offset(rect.top);
    roi.rect.set__width(rect.right - rect.left);
    roi.rect.set__height(rect.bottom - rect.top);

    rois.emplace_back(roi);
  }

  auto lmk_result =
      dynamic_cast<LandmarksResult*>(outputs[kps_output_index_].get());
  if (!lmk_result) {
    return nullptr;
  }

  std::stringstream ss;
  for (const auto& value : lmk_result->values) {
    ai_msgs::msg::Point target_point;
    target_point.set__type("body_kps");
    ss << "kps point: ";
    for (const auto& lmk : value) {
      ss << "\n" << lmk.x << "," << lmk.y << "," << lmk.score;
      geometry_msgs::msg::Point32 pt;
      pt.set__x(lmk.x);
      pt.set__y(lmk.y);
      target_point.point.emplace_back(pt);
    }
    ss << "\n";
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                 "FasterRcnnKpsOutputParser parse kps: %s",
                 ss.str().c_str());
    body_kps.emplace_back(target_point);
  }

  if (body_kps.size() != rois.size()) {
    return nullptr;
  }

  for (size_t idx = 0; idx < rois.size(); idx++) {
    ai_msgs::msg::Target target;
    target.set__type("person");
    target.rois.push_back(rois.at(idx));
    target.points.push_back(body_kps.at(idx));
    pub_data->targets.emplace_back(std::move(target));
  }

  return pub_data;
}
