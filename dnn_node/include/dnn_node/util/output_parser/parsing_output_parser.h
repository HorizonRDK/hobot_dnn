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

#ifndef EASY_DNN_PARSING_OUTPUT_PARSER_H
#define EASY_DNN_PARSING_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

// #include "dnn_node/dnn_node/util/output_parser/output_description_parser.h"

namespace hobot {
namespace easy_dnn {

/* json data sample
{
    "task":"parsing",
    "num_classes":16,
    "labels":[
        {
            "class_name":["road", "pothole", "parking_space"],
            "color_map":[128, 64, 128]
        },
        ...
    ],
    "roi_input":{
        "fp_x":480,
        "fp_y":270,
        "width":960,
        "height":512
    },
    "vanishing_point":[
        480,
        270
    ]
}
 */
struct ParsingLabel {
  std::vector<std::string> class_names;
  std::vector<int> color_map;
};

struct ParsingInputPadding {
  int padding_left = 0;
  int padding_top = 0;
  int padding_right = 0;
  int padding_bottom = 0;
};

class ParsingOutputDescription : public OutputDescription {
 public:
  ParsingOutputDescription(Model *model, int index, std::string type = "")
      : OutputDescription(model, index, type) {}
  int classes_num = 0;
  ParsingInputPadding input_padding;
  std::vector<ParsingLabel> labels;
};

class ParsingResult : public DNNResult {
 public:
  std::vector<uint8_t> data;
  int height;
  int width;
  int channel;

  void Reset() override { data.clear(); }
};

class ParsingOutputParser : public SingleBranchOutputParser<ParsingResult> {
 public:
  int32_t Parse(
      std::shared_ptr<ParsingResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;
};

}  // namespace easy_dnn
}  // namespace hobot
#endif  // EASY_DNN_PARSING_OUTPUT_PARSER_H
