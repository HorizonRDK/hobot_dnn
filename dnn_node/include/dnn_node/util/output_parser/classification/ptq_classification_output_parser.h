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

#ifndef _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_
#define _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

namespace hobot {
namespace dnn_node {

class ClassficationOutputParser
    : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;

  int InitClassNames(const std::string &cls_name_file);

 private:
  int PostProcess(std::shared_ptr<DNNTensor> &tensors, Perception &perception);

  void GetTopkResult(std::shared_ptr<DNNTensor> &tensor,
                     std::vector<Classification> &top_k_cls);

  const char *GetClsName(int id);

 private:
  int top_k_ = 1;
  std::vector<std::string> class_names_;
};

}  // namespace dnn_node
}  // namespace hobot

#endif  // _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_
