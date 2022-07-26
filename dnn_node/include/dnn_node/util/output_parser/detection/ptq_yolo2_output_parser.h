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

#ifndef _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_
#define _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

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

class Yolo2AssistParser : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override {
    return 0;
  }
};

class Yolo2OutputParser : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

 private:
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                  Perception& perception);

 private:
  PTQYolo2Config yolo2_config_ = default_ptq_yolo2_config;
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_PTQ_YOLO2_OUTPUT_PARSER_H_
