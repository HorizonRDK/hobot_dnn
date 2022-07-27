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

#ifndef EASY_DNN_FILTER2D_OUTPUT_PARSER_H
#define EASY_DNN_FILTER2D_OUTPUT_PARSER_H

#include <algorithm>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dnn_node/util/output_parser/detection/detection_data_structure.h"
#include "dnn_node/util/output_parser/output_description_parser.h"

namespace hobot {
namespace easy_dnn {

class Filter2DOutputParser;

/* json data sample
{
  "task": "detection",
  "class_name": ["vehiclerear"],
  "output_name": "centerness",
  "stride": 4,
  "score_threshold": 0.5,
  "roi_input": {"fpx": 480, "fpy": 270, "width": 960, "height": 512},
  "vanishing_point": [480, 270]
}
 */

class Filter2DOutputDescription : public OutputDescription {
 public:
  Filter2DOutputDescription(Model *model, int index, std::string type = "")
      : OutputDescription(model, index, type) {}
  int input_width = 0;
  int input_height = 0;
  std::vector<std::string> classes_names;
  int stride = 0;
  int feature_width = 0;
  int feature_height = 0;
  // generated point by width,height and stride in advance
  std::vector<std::vector<int>> feature_points;
  double score_threshold = 0.0;
  float nms_threshold = 0.5;

  friend std::ostream &operator<<(std::ostream &os,
                                  Filter2DOutputDescription *desc) {
    os << "Filter2D input width:" << desc->input_width
       << ", input height:" << desc->input_height
       << ", stride:" << desc->stride;
    os << ", class_names:";
    std::copy(desc->classes_names.begin(),
              desc->classes_names.end(),
              std::ostream_iterator<std::string>(os, ", "));
    return os;
  }
};

typedef struct {
  int16_t max_id;
  int16_t max_score;
  int16_t h_index;
  int16_t w_index;
  int8_t data[8];  // score[num_classes], pred[4], centerness[1], padding
} filter_2d_post_process_type;

class Filter2DResult : public DNNResult {
 public:
  std::vector<PerceptionRect> boxes;

  void Reset() override { boxes.clear(); }
};

class Filter2DOutputParser : public SingleBranchOutputParser<Filter2DResult> {
 public:
  int32_t Parse(
      std::shared_ptr<Filter2DResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;

  static void NMS(std::vector<PerceptionRect> &candidates,
                  std::vector<PerceptionRect> &result,
                  float overlap_ratio,
                  size_t top_N,
                  bool add_score);

  int GetTypeID(const std::string &class_name);
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_FILTER2D_OUTPUT_PARSER_H
