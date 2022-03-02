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
#include "include/fasterrcnn_kps_output_parser.h"

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
}

int32_t FasterRcnnKpsOutputParser::Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_descriptions,
      std::shared_ptr<DNNTensor>& output_tensor,
      std::vector<std::shared_ptr<OutputDescription>>& depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>>& depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>>& depend_outputs) {
  RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
    "FasterRcnnKpsOutputParser parse start");
  if (!parser_para_) {
    RCLCPP_ERROR(rclcpp::get_logger("fasterrcnn_parser"),
      "FasterRcnnKpsOutputParser para is not set");
    return -1;
  }
  if (output_descriptions) {
    RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
      "type: %s, GetDependencies size: %d",
      output_descriptions->GetType().c_str(),
      output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty()) {
      RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
        "Dependencies: %d", output_descriptions->GetDependencies().front());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
    "dep out size: %d %d %d",
    depend_output_descs.size(),
    depend_output_tensors.size(),
    depend_outputs.size());
  size_t body_box_num = 0;
  Filter2DResult *filter2d_result = nullptr;
  if (depend_outputs.size() >= 3) {
    filter2d_result = dynamic_cast<Filter2DResult *>(depend_outputs[1].get());
    if (!filter2d_result) {
      RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"), "invalid cast");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"), "out box size: %d",
        filter2d_result->boxes.size());
      body_box_num = filter2d_result->boxes.size();
    }
  }

  std::shared_ptr<LandmarksResult> result = nullptr;
  if (!output) {
    result = std::make_shared<LandmarksResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<LandmarksResult>(output);
    result->Reset();
  }

  if (body_box_num < 1 || !filter2d_result) {
    return 0;
  }

  int feature_size = parser_para_->aligned_kps_dim.at(1) *
                     parser_para_->aligned_kps_dim.at(2) *
                     parser_para_->aligned_kps_dim.at(3);
  int h_stride = parser_para_->aligned_kps_dim.at(2) *
    parser_para_->aligned_kps_dim.at(3);
  int w_stride = parser_para_->aligned_kps_dim.at(3);

  int32_t *kps_feature =
    reinterpret_cast<int32_t *>(output_tensor->sysMem[0].virAddr);
  for (size_t box_id = 0; box_id < body_box_num; ++box_id) {
    const auto &body_box = filter2d_result->boxes[box_id];
    float x1 = body_box.left;
    float y1 = body_box.top;
    float x2 = body_box.right;
    float y2 = body_box.bottom;
    float w = x2 - x1 + 1;
    float h = y2 - y1 + 1;

    float scale_x = parser_para_->kps_feat_width_ / w;
    float scale_y = parser_para_->kps_feat_height_ / h;

    float pos_distance = parser_para_->kps_pos_distance_ *
        parser_para_->kps_feat_width_;

    auto *mxnet_out_for_one_point_begin = kps_feature + feature_size * box_id;

    Landmarks skeleton;
    skeleton.resize(parser_para_->kps_points_number_);

    for (int kps_id = 0; kps_id < parser_para_->kps_points_number_; ++kps_id) {
      // find the best position
      int max_w = 0;
      int max_h = 0;
      int max_score_before_shift = mxnet_out_for_one_point_begin[kps_id];
      int32_t *mxnet_out_for_one_point = nullptr;
      for (int hh = 0; hh < parser_para_->kps_feat_height_; ++hh) {
        for (int ww = 0; ww < parser_para_->kps_feat_width_; ++ww) {
          mxnet_out_for_one_point =
              mxnet_out_for_one_point_begin + hh * h_stride + ww * w_stride;
          if (mxnet_out_for_one_point[kps_id] > max_score_before_shift) {
            max_w = ww;
            max_h = hh;
            max_score_before_shift = mxnet_out_for_one_point[kps_id];
          }
        }
      }

      float max_score =
          GetFloatByInt(max_score_before_shift,
          parser_para_->kps_shifts_[kps_id]);

      // get delta
      mxnet_out_for_one_point =
          mxnet_out_for_one_point_begin + max_h * h_stride + max_w * w_stride;
      const auto x_delta =
          mxnet_out_for_one_point[2 * kps_id +
          parser_para_->kps_points_number_];
      const auto x_shift = parser_para_->kps_shifts_[2 * kps_id +
          parser_para_->kps_points_number_];
      float fp_delta_x = GetFloatByInt(x_delta, x_shift) * pos_distance;

      const auto y_delta =
          mxnet_out_for_one_point[2 * kps_id +
          parser_para_->kps_points_number_ + 1];
      const auto y_shift =
          parser_para_->kps_shifts_[2 * kps_id +
          parser_para_->kps_points_number_ + 1];
      float fp_delta_y = GetFloatByInt(y_delta, y_shift) * pos_distance;

      Point point;
      point.x =
          (max_w + fp_delta_x + 0.46875 + parser_para_->kps_anchor_param_)
          / scale_x + x1;
      point.y =
          (max_h + fp_delta_y + 0.46875 + parser_para_->kps_anchor_param_)
          / scale_y + y1;
      point.score = SigMoid(max_score);
      skeleton[kps_id] = point;
    }
    result->values.emplace_back(skeleton);
  }

  RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
    "Kps size: %d", result->values.size());
  RCLCPP_INFO(rclcpp::get_logger("fasterrcnn_parser"),
    "FasterRcnnKpsOutputParser parse done");
  return 0;
}
