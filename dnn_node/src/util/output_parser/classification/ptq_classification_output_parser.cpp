// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "util/output_parser/classification/ptq_classification_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <string>

namespace hobot {
namespace dnn_node {

int ClassficationOutputParser::InitClassNames(const std::string &cls_name_file)
{
  std::ifstream fi(cls_name_file);
  if (fi)
  {
    std::string line;
    while (std::getline(fi, line))
    {
      class_names_.push_back(line);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ClassficationOutputParser"),
        "can not open cls name file: %s", cls_name_file.c_str());
    return -1;
  }
  return 0;
}

int32_t ClassficationOutputParser::Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor)
{
  if (!output_tensor)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ClassficationOutputParser"),
                 "output_tensor invalid cast");
    return -1;
  }

  std::shared_ptr<Dnn_Parser_Result> result;
  if (!output)
  {
    result = std::make_shared<Dnn_Parser_Result>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<Dnn_Parser_Result>(output);
  }

  int ret = PostProcess(output_tensor, result->perception);
  if (ret != 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ClassficationOutputParser"),
                "postprocess return error, code = %d", ret);
  }
  std::stringstream ss;
  ss << "ClassficationOutputParser parse finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("ClassficationOutputParser"),
              "%s", ss.str().c_str());
  return ret;
}

int ClassficationOutputParser::PostProcess(
    std::shared_ptr<DNNTensor> &output_tensors,
                  Perception &perception)
{
  perception.type = Perception::CLS;
  GetTopkResult(output_tensors, perception.cls);
  return 0;
}

void ClassficationOutputParser::GetTopkResult(
    std::shared_ptr<DNNTensor> &tensor,
    std::vector<Classification> &top_k_cls)
{
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  std::priority_queue<Classification,
                      std::vector<Classification>,
                      std::greater<Classification>>
      queue;
  int *shape = tensor->properties.validShape.dimensionSize;
  RCLCPP_DEBUG(rclcpp::get_logger("ClassficationOutputParser"),
              "PostProcess shape[1]: %d shape[2]: %d shape[3]: %d",
              shape[1], shape[2], shape[3]);
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++)
  {
    float score = reinterpret_cast<float *>(tensor->sysMem[0].virAddr)[i];
    queue.push(Classification(i, score, GetClsName(i)));
    if (queue.size() > top_k_) {
      queue.pop();
    }
  }
  while (!queue.empty())
  {
    top_k_cls.emplace_back(queue.top());
    queue.pop();
  }
  std::reverse(top_k_cls.begin(), top_k_cls.end());
}

const char *ClassficationOutputParser::GetClsName(int id) {
  if (!class_names_.empty()) {
    return class_names_[id].c_str();
  }
  return nullptr;
}

}  // namespace dnn_node
}  // namespace hobot
