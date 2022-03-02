// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_TRACKING_OUTPUT_PARSER_H
#define EASY_DNN_TRACKING_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {

/*
{
    "task":"tracking_feature",
    "size":[1, 2, 1, 256]
}
 */
class TrackingOutputDescription : public OutputDescription {
 public:
  TrackingOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  std::vector<int> size;
  hbDNNTensorShape valid_dim;
  hbDNNTensorShape align_dim;
  int32_t tensor_layout;
};

class TrackingResult : public DNNResult {
 public:
  std::vector<uint8_t> data;
};

class TrackingOutputDescriptionParser : OutputDescriptionParser {
 public:
  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;
};

class TrackingOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(std::vector<InputDescription *> &input_descriptions,
                std::vector<OutputDescription *> &output_descriptions,
                std::vector<DNNTensor *> &output_tensors,
                std::vector<std::shared_ptr<class DNNResult>> &outputs,
                int output_index) override;

  int32_t GetReorderTrackingFmapNative(int8_t *feature_map,
                                       int32_t dim_h,
                                       int32_t dim_w,
                                       int32_t dim_c,
                                       int32_t h_total,
                                       int32_t w_total,
                                       int32_t c_total,
                                       int layout,
                                       std::vector<uint8_t> &output);
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_TRACKING_OUTPUT_PARSER_H
