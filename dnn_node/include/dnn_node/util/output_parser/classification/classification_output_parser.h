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

#ifndef EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H
#define EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H

#include <algorithm>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {
class ClassificationResult : public DNNResult {
 public:
  float max_conf;
  int class_id;
  std::string class_name;
};

class ClassificationOutputParser
    : public SingleBranchOutputParser<ClassificationResult> {
 public:
  int32_t Parse(
      std::shared_ptr<ClassificationResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H
