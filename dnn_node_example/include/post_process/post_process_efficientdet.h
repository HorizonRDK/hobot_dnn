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

#include <string>

#include "include/post_process/post_process_base.h"
#include "dnn_node/util/output_parser/detection/ptq_efficientdet_output_parser.h"

#ifndef POST_PROCESS_EFFICIENTDET_H_
#define POST_PROCESS_EFFICIENTDET_H_

class EfficientDetPostProcess : public PostProcessBase {
 public:
  explicit EfficientDetPostProcess(int32_t model_output_count,
                                    std::string &dequanti_file)
      : PostProcessBase(model_output_count),
      dequanti_file_(dequanti_file) {}
  ~EfficientDetPostProcess() {}

  int SetOutParser(Model* model_manage) override;

 private:
  std::string dequanti_file_;
};

#endif  // POST_PROCESS_EFFICIENTDET_H_
