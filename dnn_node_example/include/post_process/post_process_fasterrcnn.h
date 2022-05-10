// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

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
