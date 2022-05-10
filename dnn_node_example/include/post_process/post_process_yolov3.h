// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/post_process/post_process_base.h"
#include "util/output_parser/detection/ptq_yolo3_darknet_output_parser.h"

#ifndef POST_PROCESS_YOLOV3_H_
#define POST_PROCESS_YOLOV3_H_

class Yolov3PostProcess : public PostProcessBase {
 public:
  explicit Yolov3PostProcess(int32_t model_output_count)
      : PostProcessBase(model_output_count) {}
  ~Yolov3PostProcess() {}

  int SetOutParser(Model* model_manage) override;
};

#endif  // POST_PROCESS_YOLOV3_H_
