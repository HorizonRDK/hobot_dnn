// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_NODE_IMAGE_PROC_H
#define DNN_NODE_IMAGE_PROC_H

#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

class ImageProc {
 public:
  // 使用nv12编码格式图片数据生成NV12PyramidInput
  // 如果输入图片size小于scale size（模型输入size）：将输入图片padding到左上区域
  // 如果输入图片size大于scale size（模型输入size）：crop输入图片左上区域
  // - 参数
  //   - [in] in_img_data 图片数据
  //   - [in] in_img_height 图片的高度
  //   - [in] in_img_width 图片的宽度
  //   - [in] scaled_img_height 模型输入的高度
  //   - [in] scaled_img_width 模型输入的宽度
  static std::shared_ptr<NV12PyramidInput> GetNV12PyramidFromNV12Img(
      const char* in_img_data,
      const int& in_img_height,
      const int& in_img_width,
      const int& scaled_img_height,
      const int& scaled_img_width);
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // DNN_NODE_IMAGE_PROC_H
