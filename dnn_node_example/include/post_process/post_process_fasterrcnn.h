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

#include <memory>
#include <string>
#include <vector>

#include "include/post_process/post_process_base.h"

#include "util/output_parser/detection/fasterrcnn_kps_output_parser.h"

#ifndef POST_PROCESS_FASTERRCNN_H_
#define POST_PROCESS_FASTERRCNN_H_

class FasterRcnnPostProcess : public PostProcessBase {
 public:
  explicit FasterRcnnPostProcess(int32_t model_output_count)
      : PostProcessBase(model_output_count) {}
  ~FasterRcnnPostProcess() {}

  ai_msgs::msg::PerceptionTargets::UniquePtr PostProcess(
      const std::shared_ptr<DnnNodeOutput>& outputs) override;

  int SetOutParser(Model* model_manage) override;

 private:
  // box output index is 1
  const int32_t box_output_index_ = 1;
  // kps output index is 2
  const int32_t kps_output_index_ = 2;
};

#endif  // POST_PROCESS_FASTERRCNN_H_
