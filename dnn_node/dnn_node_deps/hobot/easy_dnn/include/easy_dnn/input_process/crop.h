// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_INPUT_PROCESS_CROP_H_
#define _EASY_DNN_INPUT_PROCESS_CROP_H_

#include <memory>
#include <ostream>
#include <string>

#include "easy_dnn/description.h"
#include "easy_dnn/input_processor.h"

namespace hobot {
namespace easy_dnn {

/**
 *            image_width, image_width_stride
 *      ┌---------------------------------------┐
 *      |                               |       |
 *      |                               |       |
 *      |       (x,y)----------┐        |       |
 *      |       |              |        |       |
 *      |       |              |        |       |   image_height
 *      |       |              |height  |       |
 *      |       |              |        |       |
 *      |       |              |        |       |
 *      |       └--------------┘        |       |
 *      |             width             |       |
 *      |                               |       |
 *      |                               |       |
 *      └-------------------------------┘-------┘
 */

class CropDescription : public InputDescription {
 public:
  CropDescription(Model* mode,
                  int index,
                  std::string type = "",
                  int x = 0,
                  int y = 0,
                  int width = 0,
                  int height = 0)
      : InputDescription(mode, index, type),
        x(x),
        y(y),
        height(height),
        width(width) {}

  int x;
  int y;
  int width;   // 0 means crop to the right edge [x,...]
  int height;  // 0 means crop to the bottom edge [y, ...]

  friend std::ostream& operator<<(std::ostream& os, CropDescription* desc) {
    os << "CropDescription: x=" << desc->x << ", y=" << desc->y
       << ", width=" << desc->width << ", height=" << desc->height;
    return os;
  }
};

class CropProcessor : public InputProcessor {
 public:
  int32_t Process(std::shared_ptr<DNNTensor>& tensor,
                  std::shared_ptr<InputDescription>& input_desc,
                  std::shared_ptr<DNNInput>& input) override;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_INPUT_PROCESS_CROP_H_
