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
#include <queue>
#include <unordered_map>
#include <utility>

#include "dnn_node/dnn_node.h"
#include "dnn_node/dnn_node_impl.h"

namespace hobot {
namespace dnn_node {

DnnNode::DnnNode(
  const std::string & node_name,
  const NodeOptions & options) :
  rclcpp::Node(node_name, options) {
  dnn_node_para_ptr_ = std::make_shared<DnnNodePara>();
  dnn_node_impl_ = std::make_shared<DnnNodeImpl>(dnn_node_para_ptr_);
}

DnnNode::DnnNode(
  const std::string & node_name,
  const std::string & namespace_,
  const NodeOptions & options) :
  rclcpp::Node(node_name, namespace_, options) {
  dnn_node_para_ptr_ = std::make_shared<DnnNodePara>();
  dnn_node_impl_ = std::make_shared<DnnNodeImpl>(dnn_node_para_ptr_);
}

DnnNode::~DnnNode() {
}

int DnnNode::SetOutputParser() {
  if (!dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  if (dnn_node_para_ptr_->output_parsers_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"),
    "Output parsers are not set in node para!");
    return -1;
  }

  for (const auto& parser_pair : dnn_node_para_ptr_->output_parsers_) {
    std::shared_ptr<OutputParser> out_parser = parser_pair.second;
    if (GetModel()->SetOutputParser(parser_pair.first, out_parser) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
      "Set output parser index %d fail!", parser_pair.first);
      return -1;
    }
  }

  return 0;
}

int DnnNode::Init() {
  RCLCPP_INFO(rclcpp::get_logger("dnn"), "Node init.");

  int ret = 0;
  // 1. set model info in node para
  ret = SetNodePara();
  if (ret != 0) {
    return ret;
  }

  // check node para
  if (ModelTaskType::InvalidType == dnn_node_para_ptr_->model_task_type) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid model task type");
    return -1;
  }

  // 2. model init
  ret = dnn_node_impl_->ModelInit();
  if (ret != 0) {
    return ret;
  }

  // 3. set output parser
  ret = SetOutputParser();
  if (ret != 0) {
    return ret;
  }

  // 4. task init
  ret = dnn_node_impl_->TaskInit();
  if (ret != 0) {
    return ret;
  }

  return ret;
}

int DnnNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &output) {
  if (output) {
    RCLCPP_INFO(rclcpp::get_logger("dnn"),
      "Post process in dnn node");
  }
  return 0;
}

Model* DnnNode::GetModel() {
  return dnn_node_impl_->GetModel();
}

int DnnNode::GetModelInputSize(int32_t input_index, int& w, int& h) {
  return dnn_node_impl_->GetModelInputSize(input_index, w, h);
}

int DnnNode::Run(std::vector<std::shared_ptr<DNNInput>> &inputs,
          const std::shared_ptr<DnnNodeOutput> &output,
          const std::shared_ptr<std::vector<hbDNNRoi>> rois,
          const bool is_sync_mode,
          const int alloctask_timeout_ms,
          const int infer_timeout_ms) {
  // 1 申请推理task
  auto task_id = dnn_node_impl_->AllocTask(alloctask_timeout_ms);
  // 2 将准备好的模型输入数据inputs通过前处理接口输入给模型
  // 并通过推理任务的task_id指定推理任务
  if (dnn_node_impl_->PreProcess(inputs, task_id, rois) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Run PreProcess failed!");
    return -1;
  }

  // 3 创建模型输出数据
  // dnn_output用于存储模型推理输出
  std::shared_ptr<DnnNodeOutput> dnn_output = nullptr;
  if (output) {
    // 使用传入的DnnNodeOutput
    dnn_output = output;
  }
  if (!dnn_output) {
    // 没有传入，创建DnnNodeOutput
    dnn_output = std::make_shared<DnnNodeOutput>();
  }

  // 4 执行模型推理
  if (dnn_node_impl_->RunInferTask(dnn_output, task_id,
    std::bind(&DnnNode::PostProcess, this, std::placeholders::_1),
    is_sync_mode, infer_timeout_ms) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Run RunInferTask failed!");
    dnn_node_impl_->ReleaseTask(task_id);
    return -1;
  }

  // 5 如果是同步推理模式
  if (is_sync_mode) {
    // 释放推理task
    dnn_node_impl_->ReleaseTask(task_id);
    // 通过后处理接口处理模型输出
    if (PostProcess(dnn_output) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Run PostProcess failed!");
    }
  }

  return 0;
}

}  // namespace dnn_node
}  // namespace hobot
