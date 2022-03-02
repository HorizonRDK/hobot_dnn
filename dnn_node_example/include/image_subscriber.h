// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifndef IMAGE_SUBSCRIBER_H_

using ImgCbType =
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &msg)>;

struct SubMsgInfo {
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> msg_queue;
  std::mutex msg_mtx;
  std::condition_variable cond;
  size_t msg_q_len_limit = 2;
};

class ImageSubscriber : public rclcpp::Node {
 public:
  ImageSubscriber(ImgCbType sub_cb_fn = nullptr,
  std::string node_name = "img_sub", std::string topic_name = "");
  ~ImageSubscriber();

  sensor_msgs::msg::Image::ConstSharedPtr GetImg();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
    subscription_ = nullptr;
  ImgCbType img_cb_ = nullptr;

  SubMsgInfo sub_msg_info_;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string topic_name_ = "/image_raw";

  std::chrono::high_resolution_clock::time_point sub_img_tp_;
  int sub_img_frameCount_ = 0;
  std::mutex frame_stat_mtx_;

  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

#define IMAGE_SUBSCRIBER_H_

#endif  // IMAGE_SUBSCRIBER_H_
