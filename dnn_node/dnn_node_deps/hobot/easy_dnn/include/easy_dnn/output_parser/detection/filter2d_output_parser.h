// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_FILTER2D_OUTPUT_PARSER_H
#define EASY_DNN_FILTER2D_OUTPUT_PARSER_H

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "detection_data_structure.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {

class Filter2DOutputParser;

/* json data sample
{
  "task": "detection",
  "class_name": ["vehiclerear"],
  "output_name": "centerness",
  "stride": 4,
  "score_threshold": 0.5,
  "roi_input": {"fpx": 480, "fpy": 270, "width": 960, "height": 512},
  "vanishing_point": [480, 270]
}
 */
class Filter2DOutputDescription : public OutputDescription {
 public:
  Filter2DOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  int input_width;
  int input_height;
  std::vector<std::string> classes_names;
  int stride;
  int feature_width;
  int feature_height;
  // generated point by width,height and stride in advance
  std::vector<std::vector<int>> feature_points;
  double score_threshold;
  float nms_threshold = 0.5;

  friend std::ostream &operator<<(std::ostream &os,
                                  Filter2DOutputDescription *desc) {
    os << "Filter2D input width:" << desc->input_width
       << ", input height:" << desc->input_height
       << ", stride:" << desc->stride;
    os << ", class_names:";
    std::copy(desc->classes_names.begin(),
              desc->classes_names.end(),
              std::ostream_iterator<std::string>(os, ", "));
    return os;
  }
};

typedef struct {
  int16_t max_id;
  int16_t max_score;
  int16_t h_index;
  int16_t w_index;
  int8_t data[8];  // score[num_classes], pred[4], centerness[1], padding
} filter_2d_post_process_type;

class Filter2DResult : public DNNResult {
 public:
  std::vector<PerceptionRect> boxes;

  void Reset() override { boxes.clear(); }
};

class Filter2DOutputDescriptionParser : public OutputDescriptionParser {
 public:
  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;
};

class Filter2DOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;

  static void NMS(std::vector<PerceptionRect> &candidates,
                  std::vector<PerceptionRect> &result,
                  float overlap_ratio,
                  size_t top_N,
                  bool add_score);

  int GetTypeID(const std::string &class_name);
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_FILTER2D_OUTPUT_PARSER_H
