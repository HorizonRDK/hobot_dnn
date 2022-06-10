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

#include "include/post_process/post_process_base.h"
#include "util/output_parser/detection/ptq_ssd_output_parser.h"

#ifndef POST_PROCESS_SSD_H_
#define POST_PROCESS_SSD_H_

class SsdPostProcess : public PostProcessBase {
 public:
  explicit SsdPostProcess(int32_t model_output_count)
      : PostProcessBase(model_output_count) {}
  ~SsdPostProcess() {}

  int SetOutParser(Model* model_manage) override;
};

#endif  // POST_PROCESS_SSD_H_
