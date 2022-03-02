// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H
#define EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {

/*
{
    "task":"person_age_classification",
    "properties":[
        {
            "channel_labels":[
                "Adult",
                "Child"
            ]
        },
        ...
    ]
}
 */
class ClassificationOutputDescription : public OutputDescription {
 public:
  ClassificationOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  // multi group labels, there is only one group in most cases
  std::vector<std::vector<std::string>> channel_labels;
  bool output_feature;
  int op_type;  // TODO(ruxin.song): output parser can't get

  friend std::ostream &operator<<(
      std::ostream &os, const ClassificationOutputDescription *output_desc) {
    os << "ClassificationOutputDescription:";
    for (auto &labels : output_desc->channel_labels) {
      os << "[";
      std::copy(labels.begin(),
                labels.end(),
                std::ostream_iterator<std::string>(os, ", "));
      os << "], ";
    }
    return os;
  }
};

class ClassificationResult : public DNNResult {
 public:
  float max_conf;
  int class_id;
  std::string class_name;
};

class ClassificationOutputDescriptionParser : public OutputDescriptionParser {
 public:
  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;
};

class ClassificationOutputParser : public SingleBranchOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override;

  int32_t GetMaxConfCh(int32_t &max_conf_ch_id,
                       float &max_conf,
                       std::shared_ptr<DNNTensor> &output_tensor,
                       ClassificationOutputDescription *output_desc);
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_CLASSIFICATION_OUTPUT_PARSER_H
