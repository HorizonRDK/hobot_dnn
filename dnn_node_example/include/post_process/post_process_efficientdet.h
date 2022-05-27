// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>

#include "include/post_process/post_process_base.h"
#include "util/output_parser/detection/ptq_efficientdet_output_parser.h"

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
