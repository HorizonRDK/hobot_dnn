// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>

#include "include/post_process/post_process_base.h"
#include "util/output_parser/segmentation/ptq_unet_output_parser.h"

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/core/matx.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#ifndef POST_PROCESS_UNET_H
#define POST_PROCESS_UNET_H

class UnetPostProcess : public PostProcessBase {
 public:
  explicit UnetPostProcess(int32_t model_output_count, int dump_render)
      : PostProcessBase(model_output_count), dump_render_img_(dump_render) { }

  ai_msgs::msg::PerceptionTargets::UniquePtr PostProcess(
      const std::shared_ptr<DnnNodeOutput>& outputs) override;

  int SetOutParser(Model* model_manage) override;

 private:
  int RenderUnet(const std::shared_ptr<DnnNodeOutput>& node_output);

 private:
  Parsing seg;
  int dump_render_img_ = 0;
};

#endif  // POST_PROCESS_UNET_H
