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
#include "cv_bridge/cv_bridge.h"

#include "include/image_utils.h"
#include "dnn_node/dnn_node.h"
#include "include/image_subscriber.h"

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

  int Run();

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PreProcess(std::vector<std::shared_ptr<DNNInput>> &inputs,
                const TaskId& task_id,
                const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr)
                override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs)
    override;

 private:
  std::string model_file_name_ = "config/multitask_body_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  const int model_input_width_ = 960;
  const int model_input_height_ = 544;
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

  std::shared_ptr<ImageSubscriber> image_subscriber_ = nullptr;

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
};

#endif  // FASTERRCNN_BODY_DET_NODE_H_
