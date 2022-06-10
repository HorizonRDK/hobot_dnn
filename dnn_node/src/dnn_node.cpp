// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dnn_node/dnn_node.h"

#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dnn_node/dnn_node_impl.h"

namespace hobot {
namespace dnn_node {

DnnNode::DnnNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  dnn_node_para_ptr_ = std::make_shared<DnnNodePara>();
  dnn_node_impl_ = std::make_shared<DnnNodeImpl>(dnn_node_para_ptr_);
}

DnnNode::DnnNode(const std::string &node_name,
                 const std::string &namespace_,
                 const NodeOptions &options)
    : rclcpp::Node(node_name, namespace_, options) {
  dnn_node_para_ptr_ = std::make_shared<DnnNodePara>();
  dnn_node_impl_ = std::make_shared<DnnNodeImpl>(dnn_node_para_ptr_);
}

DnnNode::~DnnNode() {}

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

  for (const auto &parser_pair : dnn_node_para_ptr_->output_parsers_) {
    std::shared_ptr<OutputParser> out_parser = parser_pair.second;
    if (GetModel()->SetOutputParser(parser_pair.first, out_parser) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
                   "Set output parser index %d fail!",
                   parser_pair.first);
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
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Set node para failed!");
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
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Model init failed!");
    return ret;
  }

  // 3. set output parser
  ret = SetOutputParser();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Set output parser failed!");
    return ret;
  }

  // 4. task init
  ret = dnn_node_impl_->TaskInit();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Task init failed!");
    return ret;
  }

  return ret;
}

int DnnNode::PostProcess(const std::shared_ptr<DnnNodeOutput> &output) {
  if (output) {
    RCLCPP_INFO(rclcpp::get_logger("dnn"), "Post process in dnn node");
  }
  return 0;
}

Model *DnnNode::GetModel() { return dnn_node_impl_->GetModel(); }

int DnnNode::GetModelInputSize(int32_t input_index, int &w, int &h) {
  return dnn_node_impl_->GetModelInputSize(input_index, w, h);
}

int DnnNode::Run(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
                 const std::shared_ptr<DnnNodeOutput> &output,
                 const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                 const bool is_sync_mode,
                 const int alloctask_timeout_ms,
                 const int infer_timeout_ms) {
  std::vector<std::shared_ptr<DNNTensor>> tensor_inputs;
  InputType input_type = InputType::DNN_INPUT;
  std::vector<std::shared_ptr<OutputDescription>> output_descs{};
  return dnn_node_impl_->Run(
      dnn_inputs,
      tensor_inputs,
      input_type,
      output_descs,
      output,
      std::bind(&DnnNode::PostProcess, this, std::placeholders::_1),
      rois,
      is_sync_mode,
      alloctask_timeout_ms,
      infer_timeout_ms);
}

int DnnNode::Run(std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
                 std::vector<std::shared_ptr<OutputDescription>> &output_descs,
                 const std::shared_ptr<DnnNodeOutput> &output,
                 const std::shared_ptr<std::vector<hbDNNRoi>> rois,
                 const bool is_sync_mode,
                 const int alloctask_timeout_ms,
                 const int infer_timeout_ms) {
  std::vector<std::shared_ptr<DNNTensor>> tensor_inputs;
  InputType input_type = InputType::DNN_INPUT;
  return dnn_node_impl_->Run(
      dnn_inputs,
      tensor_inputs,
      input_type,
      output_descs,
      output,
      std::bind(&DnnNode::PostProcess, this, std::placeholders::_1),
      rois,
      is_sync_mode,
      alloctask_timeout_ms,
      infer_timeout_ms);
}

int DnnNode::Run(std::vector<std::shared_ptr<DNNTensor>> &tensor_inputs,
                 std::vector<std::shared_ptr<OutputDescription>> &output_descs,
                 const std::shared_ptr<DnnNodeOutput> &output,
                 const bool is_sync_mode,
                 const int alloctask_timeout_ms,
                 const int infer_timeout_ms) {
  std::vector<std::shared_ptr<DNNInput>> dnn_inputs;
  InputType input_type = InputType::DNN_TENSOR;
  return dnn_node_impl_->Run(
      dnn_inputs,
      tensor_inputs,
      input_type,
      output_descs,
      output,
      std::bind(&DnnNode::PostProcess, this, std::placeholders::_1),
      nullptr,
      is_sync_mode,
      alloctask_timeout_ms,
      infer_timeout_ms);
}

}  // namespace dnn_node
}  // namespace hobot
