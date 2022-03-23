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

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef IMAGE_SUBSCRIBER_H_

using ImgCbType =
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &msg)>;

#ifdef SHARED_MEM_ENABLED
using SharedMemImgCbType =
  std::function<void(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr &msg)>;
#endif

struct SubMsgInfo {
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> msg_queue;
  std::mutex msg_mtx;
  std::condition_variable cond;
  size_t msg_q_len_limit = 2;
};

class ImageSubscriber : public rclcpp::Node {
 public:
#ifdef SHARED_MEM_ENABLED
  explicit ImageSubscriber(SharedMemImgCbType sub_cb_fn,
  std::string node_name = "img_sub", std::string topic_name = "");
#endif
  explicit ImageSubscriber(ImgCbType sub_cb_fn = nullptr,
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

#ifdef SHARED_MEM_ENABLED
  SharedMemImgCbType sharedmem_img_cb_ = nullptr;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      hbmem_subscription_;
  void sharedmem_topic_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  std::string sharedmem_topic_name_ = "/hbmem_img";
#endif

  std::chrono::high_resolution_clock::time_point sub_img_tp_;
  int sub_img_frameCount_ = 0;
  std::mutex frame_stat_mtx_;

  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};

#define IMAGE_SUBSCRIBER_H_

#endif  // IMAGE_SUBSCRIBER_H_
