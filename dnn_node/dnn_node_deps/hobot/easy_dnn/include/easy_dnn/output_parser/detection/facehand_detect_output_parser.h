// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_FACE_HAND_DETECTION_OUTPUT_PARSER_H
#define EASY_DNN_FACE_HAND_DETECTION_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"
#include "filter2d_output_parser.h"

namespace hobot {
namespace easy_dnn {

class FaceHandDetectionOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_DETECTION_OUTPUT_PARSER_H
