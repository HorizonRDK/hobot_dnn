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

using hobot::dnn_node::output_parser::Bbox;
using hobot::dnn_node::output_parser::Detection;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;

namespace hobot {
namespace dnn_node {
namespace parser_fcos {

float score_threshold_ = 0.5;
float nms_threshold_ = 0.6;
int nms_top_k_ = 500;

int InitClassNum(const int &class_num);

int InitClassNames(const std::string &cls_name_file);

int InitStrides(const std::vector<int> &strides, const int &model_output_count);

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::shared_ptr<DnnParserResult> &output);

}  // namespace parser_fcos
}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_FCOS_OUTPUT_PARSER_H
