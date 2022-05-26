// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_
#define _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_

#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "util/output_parser/perception_common.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"

namespace hobot {
namespace dnn_node {
/**
 * Method for post processing
 */
class UnetOutputParser : public SingleBranchOutputParser
{
 public:
  int32_t Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

 private:
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                  Perception &perception);

 private:
  int num_classes_ = 20;
};

}  // namespace dnn_node
}  // namespace hobot

#endif  // _SEGMENTATION_PTQ_UNET_OUTPUT_PARSER_H_
