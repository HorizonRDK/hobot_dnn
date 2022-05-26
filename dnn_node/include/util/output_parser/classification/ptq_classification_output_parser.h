// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_
#define _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_

#include <string>
#include <vector>
#include <memory>

#include "util/output_parser/perception_common.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {

class ClassficationOutputParser : public SingleBranchOutputParser
{
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

  int InitClassNames(const std::string &cls_name_file);

 private:
  int PostProcess(std::shared_ptr<DNNTensor> &tensors,
                  Perception &perception);

  void GetTopkResult(std::shared_ptr<DNNTensor> &tensor,
                     std::vector<Classification> &top_k_cls);

  const char *GetClsName(int id);

 private:
  int top_k_ = 1;
  std::vector<std::string> class_names_;
};

}  // namespace dnn_node
}  // namespace hobot

#endif  // _CLASSIFICATION_PTQ_CLASSIFICATION_OUTPUT_PARSER_H_
