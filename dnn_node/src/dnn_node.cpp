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
  // 如果用户没有继承dnn_node中的SetOutputParser接口
  RCLCPP_WARN(rclcpp::get_logger("dnn"), "Run default SetOutputParser.");

  if (!dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid node para!");
    return -1;
  }

  if (!dnn_node_para_ptr_->output_parsers_.empty()) {
    // 使用用户在配置参数中设置的parser进行设置
    RCLCPP_WARN(rclcpp::get_logger("dnn"),
                "Set output parser with dnn node para.");

    for (const auto &parser_pair : dnn_node_para_ptr_->output_parsers_) {
      std::shared_ptr<OutputParser> out_parser = parser_pair.second;
      if (!out_parser) {
        RCLCPP_ERROR(rclcpp::get_logger("dnn"), "Invalid out_parser");
        return -1;
      }
      if (GetModel()->SetOutputParser(parser_pair.first, out_parser) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("dnn"),
                     "Set output parser index %d fail!",
                     parser_pair.first);
        return -1;
      }
      RCLCPP_INFO(rclcpp::get_logger("dnn"),
                  "SetOutputParser %d success",
                  parser_pair.first);
    }
  } else {
    // 没有通过DnnNodePara参数配置解析方法，使用dnn node内置的parser进行设置
    RCLCPP_WARN(
        rclcpp::get_logger("dnn"),
        "Set output parser with default dnn node parser, you will get all "
        "output tensors and should parse output_tensors in PostProcess.");
    return dnn_node_impl_->SetDefaultOutputParser();
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

  // 校验bpu_core_ids参数
  if (!dnn_node_para_ptr_->bpu_core_ids.empty()) {
    if (dnn_node_para_ptr_->bpu_core_ids.size() !=
        dnn_node_para_ptr_->task_num) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn"),
                   "DnnNodePara of bpu_core_ids size %d should be zero or "
                   "equal with task_num %d",
                   dnn_node_para_ptr_->bpu_core_ids.size(),
                   dnn_node_para_ptr_->task_num);
      return -1;
    }
    for (const auto &bpu_core_id : dnn_node_para_ptr_->bpu_core_ids) {
      if (bpu_core_id < BPUCoreIDType::BPU_CORE_ANY ||
          bpu_core_id > BPUCoreIDType::BPU_CORE_1) {
        RCLCPP_ERROR(rclcpp::get_logger("dnn"),
                     "Invalid bpu_core_id %d, which should be [%d, %d]",
                     static_cast<int>(bpu_core_id),
                     static_cast<int>(BPUCoreIDType::BPU_CORE_ANY),
                     static_cast<int>(BPUCoreIDType::BPU_CORE_1));
        return -1;
      }
    }
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
