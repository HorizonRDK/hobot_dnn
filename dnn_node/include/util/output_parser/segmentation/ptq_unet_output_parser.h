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

#ifndef _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_
#define _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_

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
 * Method for post processing
 */
class UnetOutputParser : public SingleBranchOutputParser
{
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

 private:
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                  Perception &perception);

 private:
  int num_classes_ = 20;
};

}  // namespace dnn_node
}  // namespace hobot

#endif  // _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_
