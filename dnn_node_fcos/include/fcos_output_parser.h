// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_NODE_FCOS_OUTPUT_PARSER_H
#define DNN_NODE_FCOS_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/description.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

using hobot::easy_dnn::DNNResult;
using hobot::easy_dnn::DNNTensor;
using hobot::easy_dnn::InputDescription;
using hobot::easy_dnn::Model;
using hobot::easy_dnn::MultiBranchOutputParser;
using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;
using hobot::easy_dnn::SingleBranchOutputParser;

class FcosOutputDescription : public OutputDescription {
 public:
  FcosOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  int op_type;
};

struct Perception {
 public:
  float x1;
  float y1;
  float x2;
  float y2;
  float conf;
  int type;
  std::string category_name = "";
  friend bool operator>(const Perception &lhs, const Perception &rhs) {
    return (lhs.conf > rhs.conf);
  }
};

class FcosDetResult : public DNNResult {
 public:
  std::vector<Perception> boxes;
  void Reset() override { boxes.clear(); }
};

// FcosConfig
struct FcosConfig {
  std::vector<int> strides;
  int class_num;
  std::vector<std::string> class_names;
  std::string det_name_list;
};
extern FcosConfig default_fcos_config;

class FcosDetectionAssistParser : public SingleBranchOutputParser {};

class FcosDetectionOutputParser : public MultiBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs);

 private:
  void GetBboxAndScoresNHWC(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                            std::vector<Perception> &dets);

  void GetBboxAndScoresNCHW(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                            std::vector<Perception> &dets);

  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                  std::vector<Perception> &det_result);

  void nms(std::vector<Perception> &input, float iou_threshold, int top_k,
           std::vector<Perception> &result, bool suppress);

 private:
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 1000;
  FcosConfig fcos_config_ = default_fcos_config;
};

#endif  // DNN_NODE_FCOS_OUTPUT_PARSER_H
