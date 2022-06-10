// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "util/output_parser/segmentation/ptq_unet_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include <queue>

#include "util/output_parser/algorithm.h"
#include "util/output_parser/utils.h"

namespace hobot {
namespace dnn_node {

int32_t UnetOutputParser::Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor)
{
  if (!output_tensor)
  {
    RCLCPP_ERROR(rclcpp::get_logger("UnetOutputParser"),
                 "output_tensor invalid cast");
    return -1;
  }

  std::shared_ptr<Dnn_Parser_Result> result;
  if (!output)
  {
    result = std::make_shared<Dnn_Parser_Result>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<Dnn_Parser_Result>(output);
  }

  auto depend_output_tensors =
          std::vector<std::shared_ptr<DNNTensor>>{output_tensor};

  int ret = PostProcess(depend_output_tensors, result->perception);
  if (ret != 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("UnetOutputParser"),
                "postprocess return error, code = %d", ret);
  }
  std::stringstream ss;
  ss << "UnetOutputParser parse finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("UnetOutputParser"),
              "%s", ss.str().c_str());
  return ret;
}

int UnetOutputParser::PostProcess(
  std::vector<std::shared_ptr<DNNTensor>> &tensors,
  Perception &perception)
{
  perception.type = Perception::SEG;
  hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // get shape
  int h_index, w_index, c_index;
  get_tensor_hwc_index(tensors[0], &h_index, &w_index, &c_index);
  int height = tensors[0]->properties.validShape.dimensionSize[h_index];
  int width = tensors[0]->properties.validShape.dimensionSize[w_index];
  int channel = tensors[0]->properties.validShape.dimensionSize[c_index];

  RCLCPP_DEBUG(rclcpp::get_logger("UnetOutputParser"),
              "PostProcess width: %d height: %d channel: %d",
              width, height, channel);

  float *data = reinterpret_cast<float *>(tensors[0]->sysMem[0].virAddr);
  perception.seg.seg.resize(height * width);
  perception.seg.width = width;
  perception.seg.height = height;
  perception.seg.num_classes = num_classes_;

  // argmax, operate in NHWC format
  for (int h = 0; h < height; ++h)
  {
    for (int w = 0; w < width; ++w)
    {
      float top_score = -1000000.0f;
      int top_index = 0;
      float *c_data = data + (width * h + w) * channel;
      for (int c = 0; c < channel; c++)
      {
        if (c_data[c] > top_score)
        {
          top_score = c_data[c];
          top_index = c;
        }
      }
      perception.seg.seg[h * width + w] = top_index;
    }
  }
  return 0;
}

}  // namespace dnn_node
}  // namespace hobot
