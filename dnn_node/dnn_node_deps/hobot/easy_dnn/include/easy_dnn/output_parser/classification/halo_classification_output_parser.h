// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_HALO_CLASSIFICATION_OUTPUT_PARSER_H
#define EASY_DNN_HALO_CLASSIFICATION_OUTPUT_PARSER_H

#include <algorithm>
#include <complex>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {

template <typename T>
static void Softmax(T *arr, size_t l) {
  if (l == 0u) return;
  T max = arr[0u], sum = 0;
  for (size_t i = 1u; i < l; ++i)
    if (max < arr[i]) max = arr[i];
  for (size_t i = 0u; i < l; ++i) {
    arr[i] = std::exp(arr[i] - max);
    sum += arr[i];
  }
  for (size_t i = 0u; i < l; ++i) arr[i] /= sum;
}

class HaloClassificationResult : public DNNResult {
 public:
  std::vector<int8_t> features_;
};

class HaloClassificationOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H
