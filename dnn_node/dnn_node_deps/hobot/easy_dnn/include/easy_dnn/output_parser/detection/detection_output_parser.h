// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_DETECTION_OUTPUT_PARSER_H
#define EASY_DNN_DETECTION_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"
#include "filter2d_output_parser.h"

namespace hobot {
namespace easy_dnn {

typedef std::pair<int32_t, int32_t> AnchorWHPair;
#define DETECTION_COORD_SHIFT_BITS 2
#define UP_SCALE 10.0f

/* json data sample
{
    "task":"person_head_detection",
    "anchor_wh_pair":[7, 8, 10, 12, 16, 19, 40, 50],
    "linear_a":4,
    "linear_b":2,
    "class_name":[
        "person_head"
    ],
    "reg_type":"frcnn",
    "legacy_bbox":1,
    "pixel_center_align":0,
    "score_threshold":[
        0.25
    ],
    "image_size":[
        128,
        64
    ],
    "norm_method":"height",
    "norm_len":116
}
 */
class DetectionOutputDescription : public OutputDescription {
 public:
  DetectionOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  std::vector<AnchorWHPair> anchor_wh_pair;
  std::vector<AnchorWHPair> scaled_anchor_wh_pair;
  std::vector<std::string> classes_names;
  int op_type;
  std::vector<float> score_threshold;
};

class DetectionOutputDescriptionParser : public OutputDescriptionParser {
 public:
  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;
};

class DetectionResult : public DNNResult {
 public:
  std::vector<PerceptionRect> boxes;
};

typedef struct {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
  int8_t score;
  uint8_t class_label;
  int16_t padding[3];
} easy_dnn_post_process_bbox_with_pad_type_t;

class DetectionOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;

  static int32_t ParseDetectionResultWithDPP(
      std::shared_ptr<DetectionResult> &result,
      std::shared_ptr<DNNTensor> &output_tensor,
      uint32_t branch_num,
      DetectionOutputDescription *output_desc);
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_DETECTION_OUTPUT_PARSER_H
