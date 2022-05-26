// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>

#include "include/post_process/post_process_base.h"
#include "util/output_parser/segmentation/ptq_unet_output_parser.h"

#ifndef POST_PROCESS_UNET_H
#define POST_PROCESS_UNET_H

class UnetPostProcess : public PostProcessBase {
 public:
  explicit UnetPostProcess(int32_t model_output_count)
      : PostProcessBase(model_output_count) {}

  ai_msgs::msg::PerceptionTargets::UniquePtr PostProcess(
      const std::shared_ptr<DnnNodeOutput>& outputs) override;

  int SetOutParser(Model* model_manage) override;
};

#endif  // POST_PROCESS_UNET_H
