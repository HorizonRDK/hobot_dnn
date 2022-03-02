// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "include/image_subscriber.h"

ImageSubscriber::ImageSubscriber(ImgCbType sub_cb_fn,
  std::string node_name, std::string topic_name)
    : Node(node_name), img_cb_(sub_cb_fn) {
  // this->declare_parameter("sub_img_topic", topic_name_);
  // if (this->get_parameter("sub_img_topic", topic_name_)) {
  //   RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
  //   "Update sub_img_topic with topic_name: %s", topic_name_.c_str());
  // }
  if (!topic_name.empty()) {
    topic_name_ = topic_name;
  }
  RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
  "Create subscription with topic_name: %s", topic_name_.c_str());
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name_, 10,
      std::bind(&ImageSubscriber::topic_callback, this,
                std::placeholders::_1));
}


#ifdef SHARED_MEM_ENABLED
ImageSubscriber::ImageSubscriber(SharedMemImgCbType sub_cb_fn,
  std::string node_name, std::string topic_name)
    : Node(node_name), sharedmem_img_cb_(sub_cb_fn) {
  if (!topic_name.empty()) {
    topic_name_ = topic_name;
  }

  hbmem_subscription_ =
      this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
          sharedmem_topic_name_, 10,
          std::bind(&ImageSubscriber::sharedmem_topic_callback, this,
                    std::placeholders::_1));
  RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
    "Create hbmem_subscription with topic_name: %s",
    sharedmem_topic_name_.c_str());
}
#endif

ImageSubscriber::~ImageSubscriber() {}

sensor_msgs::msg::Image::ConstSharedPtr ImageSubscriber::GetImg() {
  std::unique_lock<std::mutex> lg(sub_msg_info_.msg_mtx);
  sub_msg_info_.cond.wait(lg, [&](){
      return !sub_msg_info_.msg_queue.empty() || !rclcpp::ok();
  });
  sensor_msgs::msg::Image::ConstSharedPtr img_msg = nullptr;
  if (!rclcpp::ok() || sub_msg_info_.msg_queue.empty()) {
      return img_msg;
  }
  img_msg = sub_msg_info_.msg_queue.front();
  sub_msg_info_.msg_queue.pop();
  return img_msg;
}

void ImageSubscriber::topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv img");

  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  if (img_cb_) {
    img_cb_(msg);
  }
  std::unique_lock<std::mutex> lg(sub_msg_info_.msg_mtx);
  if (sub_msg_info_.msg_queue.size() >= sub_msg_info_.msg_q_len_limit) {
    RCLCPP_WARN(rclcpp::get_logger("img_sub"),
    "Drop img! Sub msg queue len: %d exceeds limit: %d",
    sub_msg_info_.msg_queue.size(), sub_msg_info_.msg_q_len_limit);
    sub_msg_info_.msg_queue.pop();
  }
  sub_msg_info_.msg_queue.push(msg);
  sub_msg_info_.cond.notify_one();
}

#ifdef SHARED_MEM_ENABLED
void ImageSubscriber::sharedmem_topic_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv img");
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  auto& img_msg = msg;
  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding.data()
  << ", h: " << img_msg->height
  << ", w: " << img_msg->width
  << ", step: " << img_msg->step
  << ", index: " << img_msg->index
  << ", stamp: " << img_msg->time_stamp
  << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "%s", ss.str().c_str());

  if (sharedmem_img_cb_) {
    sharedmem_img_cb_(msg);
  }
}
#endif
