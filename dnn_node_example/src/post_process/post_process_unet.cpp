// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_unet.h"
#include "include/dnn_example_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/core/matx.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

int UnetPostProcess::SetOutParser(Model* model_manage)
{
  RCLCPP_INFO(rclcpp::get_logger("UnetPostProcess"), "Set out parser");
  output_index_ = model_output_count_ - 1;
  std::shared_ptr<OutputParser> parser =
      std::make_shared<hobot::dnn_node::UnetOutputParser>();
  model_manage->SetOutputParser(output_index_, parser);

  return 0;
}

ai_msgs::msg::PerceptionTargets::UniquePtr
UnetPostProcess::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output)
{
  const auto &outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("UnetPostProcess"),
              "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("UnetPostProcess"),
                "Invalid outputs");
    return nullptr;
  }
  auto *det_result =
    dynamic_cast<Dnn_Parser_Result *>(outputs[output_index_].get());
  if (!det_result) {
    RCLCPP_INFO(rclcpp::get_logger("UnetPostProcess"),
                "invalid cast");
    return 0;
  }

  uint8_t bgr_putpalette[] = {
      128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
      153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
      0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
      0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

  auto pyramid =
      std::dynamic_pointer_cast<DnnExampleOutput>(node_output)->pyramid;
  if (!pyramid)
  {
    return nullptr;
  }
  char *y_img = reinterpret_cast<char *>(pyramid->y_vir_addr);
  char *uv_img = reinterpret_cast<char *>(pyramid->uv_vir_addr);
  auto height = pyramid->height;
  auto width = pyramid->width;
  auto img_y_size = height * width;
  auto img_uv_size = img_y_size / 2;
  char *buf = new char[img_y_size + img_uv_size];
  memcpy(buf, y_img, img_y_size);
  memcpy(buf + img_y_size, uv_img, img_uv_size);
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
  cv::Mat mat;  //  get bgr mat from pyramid
  cv::cvtColor(nv12, mat, CV_YUV2BGR_NV12);  //  nv12 to bgr
  delete[] buf;

  auto seg = det_result->perception.seg;
  int parsing_width = seg.width;
  int parsing_height = seg.height;

  cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
  uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();
  // auto w_base = perception->w_base;
  // auto h_base = perception->h_base;
  for (int h = 0; h < parsing_height; ++h)
  {
    for (int w = 0; w < parsing_width; ++w)
    {
      auto id = seg.seg[h * parsing_width + w];
      *parsing_img_ptr++ = bgr_putpalette[id * 3];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
    }
  }

  // resize parsing image
  cv::resize(parsing_img, parsing_img, mat.size(), 0, 0);

  // alpha blending
  float alpha_f = 0.5;
  cv::Mat dst;
  addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
  mat = std::move(dst);

  std::string saving_path = "render_unet.jpeg";
  RCLCPP_WARN(rclcpp::get_logger("UnetPostProcess"),
              "Draw result to file: %s",
              saving_path.c_str());
  cv::imwrite(saving_path, mat);
  return nullptr;
}
