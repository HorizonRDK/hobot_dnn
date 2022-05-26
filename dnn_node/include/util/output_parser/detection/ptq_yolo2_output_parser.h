// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_
#define _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_

#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "util/output_parser/perception_common.h"

#include "dnn/hb_dnn_ext.h"

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {
/**
 * Config definition for Yolo2
 */
struct PTQYolo2Config {
  int stride;
  std::vector<std::pair<double, double>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

  std::string Str() {
    std::stringstream ss;
    ss << "stride: " << stride;
    ss << "; anchors_table: ";
    for (auto anchors : anchors_table) {
      ss << "[" << anchors.first << "," << anchors.second << "] ";
    }
    ss << "; class_num: " << class_num;
    return ss.str();
  }
};

extern PTQYolo2Config default_ptq_yolo2_config;

class Yolo2OutputParser : public SingleBranchOutputParser
{
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

 private:
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                  Perception &perception);

 private:
  PTQYolo2Config yolo2_config_ = default_ptq_yolo2_config;
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_
