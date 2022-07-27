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

#ifndef _DETECTION_FCOS_OUTPUT_PARSER_H
#define _DETECTION_FCOS_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

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

class FcosDetectionAssistParser
    : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override {
    return 0;
  }
};

class FcosDetectionOutputParser
    : public MultiBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
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
