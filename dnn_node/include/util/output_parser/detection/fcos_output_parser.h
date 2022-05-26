// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _DETECTION_FCOS_OUTPUT_PARSER_H
#define _DETECTION_FCOS_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "util/output_parser/perception_common.h"

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {

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
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

 private:
  void GetBboxAndScoresNHWC(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                            std::vector<Detection> &dets);

  void GetBboxAndScoresNCHW(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                            std::vector<Detection> &dets);

  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                  Perception &perception);

 private:
  float score_threshold_ = 0.5;
  float nms_threshold_ = 0.6;
  int nms_top_k_ = 500;
  FcosConfig fcos_config_ = default_fcos_config;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_FCOS_OUTPUT_PARSER_H
