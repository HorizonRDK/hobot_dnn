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

#include "util/output_parser/classification/ptq_classification_output_parser.h"

#ifndef POST_PROCESS_CLASSIFICATION_H_
#define POST_PROCESS_CLASSIFICATION_H_

class ClassificationPostProcess : public PostProcessBase {
 public:
  explicit ClassificationPostProcess(int32_t model_output_count)
      : PostProcessBase(model_output_count) {}
  ~ClassificationPostProcess() {}

  ai_msgs::msg::PerceptionTargets::UniquePtr PostProcess(
      const std::shared_ptr<DnnNodeOutput>& outputs) override;

  int SetOutParser(Model* model_manage) override;

  void SetClsNameFile(const std::string &cls_name_file_);

 private:
  std::string cls_name_file_;
};

#endif  // POST_PROCESS_CLASSIFICATION_H_
