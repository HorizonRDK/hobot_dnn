// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif
#include "include/image_utils.h"
#include "dnn_node/dnn_node.h"
#include "util/image_proc.h"
#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef FASTERRCNN_BODY_DET_NODE_H_
#define FASTERRCNN_BODY_DET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::TaskId;
using hobot::dnn_node::ModelTaskType;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

enum class DnnFeedType {
  // 用于预测的图片来源，0：本地图片；1：订阅到的image msg
  FROM_LOCAL = 0,
  FROM_SUB = 1
};

struct FasterRcnnOutput : public DnnNodeOutput {
  std::string image_name = "";
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
};

class FasterRcnnBodyDetNode : public DnnNode {
 public:
  FasterRcnnBodyDetNode(
  const std::string & node_name,
  const NodeOptions & options = NodeOptions());
  ~FasterRcnnBodyDetNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs)
    override;

 private:
  std::string model_file_name_ = "config/multitask_body_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  const int32_t model_output_count_ = 3;
  // box output index is 1
  const int32_t box_output_index_ = 1;
  // kps output index is 2
  const int32_t kps_output_index_ = 2;

  // 用于预测的图片来源，0：本地图片；1：订阅到的image msg
  int feed_type_ = 0;
  std::string image_ = "config/test.jpg";
  // 0: bgr, 1: nv12
  int image_type_ = 0;
  int dump_render_img_ = 0;
  int is_sync_mode_ = 1;
  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 0;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  std::mutex frame_stat_mtx_;

  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output);
  int FeedFromLocal();
  int FeedFromSubscriber();
  int Render(cv::Mat& mat, const Filter2DResult *, const LandmarksResult *,
  const float& model_input_h, const float& model_input_w);

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
    ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::chrono::high_resolution_clock::time_point sub_img_tp_;
  int sub_img_frameCount_ = 0;
  std::mutex sub_frame_stat_mtx_;
};

#endif  // FASTERRCNN_BODY_DET_NODE_H_
