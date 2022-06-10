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

#ifndef PTQ_YOLO3_DARKNET_OUTPUT_PARSER_H
#define PTQ_YOLO3_DARKNET_OUTPUT_PARSER_H

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
 * Config definition for Yolo3
 */
struct PTQYolo3DarknetConfig {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

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

extern PTQYolo3DarknetConfig default_ptq_yolo3_darknet_config;

class Yolo3_darknetAssistParser : public SingleBranchOutputParser {};

class Yolo3DarknetOutputParser : public MultiBranchOutputParser
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

  void PostProcessNHWC(std::shared_ptr<DNNTensor> tensor,
                       int layer,
                       std::vector<Detection> &dets);

  void PostProcessNCHW(std::shared_ptr<DNNTensor> tensor,
                       int layer,
                       std::vector<Detection> &dets);

 private:
  PTQYolo3DarknetConfig yolo3_config_ = default_ptq_yolo3_darknet_config;
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // PTQ_YOLO3_DARKNET_OUTPUT_PARSER_H_
