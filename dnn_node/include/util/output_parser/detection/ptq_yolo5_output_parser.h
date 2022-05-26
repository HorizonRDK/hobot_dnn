// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _DETECTION_PTQ_YOLO5_OUTPUT_PARSER_H_
#define _DETECTION_PTQ_YOLO5_OUTPUT_PARSER_H_

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
 * Config definition for Yolo5
 */
struct PTQYolo5Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<float>> dequantize_scale;

  std::string Str() {
    std::stringstream ss;
    ss << "strides: ";
    for (const auto &stride : strides) {
      ss << stride << " ";
    }

    ss << "; anchors_table: ";
    for (const auto &anchors : anchors_table) {
      for (auto data : anchors) {
        ss << "[" << data.first << "," << data.second << "] ";
      }
    }
    ss << "; class_num: " << class_num;
    return ss.str();
  }
};

extern PTQYolo5Config default_ptq_yolo5_config;

class Yolo5AssistParser : public SingleBranchOutputParser {};

class Yolo5OutputParser : public MultiBranchOutputParser
{
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

 private:
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                  Perception &perception);

  void PostProcess(std::shared_ptr<DNNTensor> tensor,
                   int layer,
                   std::vector<Detection> &dets);

  double Dequanti(int32_t data,
                  int layer,
                  bool big_endian,
                  int offset,
                  hbDNNTensorProperties &properties);

 private:
  PTQYolo5Config yolo5_config_ = default_ptq_yolo5_config;
  float score_threshold_ = 0.4;
  float nms_threshold_ = 0.5;
  int nms_top_k_ = 5000;
  bool has_dequanti_node_ = true;
  std::string dequanti_file_ = "";
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_PTQ_YOLO5_OUTPUT_PARSER_H_
