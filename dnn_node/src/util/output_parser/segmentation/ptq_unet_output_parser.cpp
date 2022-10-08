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

#include "dnn_node/util/output_parser/segmentation/ptq_unet_output_parser.h"

#include <queue>

#include "dnn_node/util/output_parser/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace dnn_node {
namespace parser_unet {

class UnetOutputDescription : public OutputDescription {
 public:
  UnetOutputDescription(Model* mode, int index, std::string type = "")
      : OutputDescription(mode, index, std::move(type)) {}
  ~UnetOutputDescription() override = default;

  uint32_t valid_w = 0;
  uint32_t valid_h = 0;
  int parse_render = 0;
};

int PostProcess(std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
                Perception& perception,
                int valid_w,
                int valid_h);

int ParseRenderPostProcess(
    std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
    Perception& perception);

int num_classes_ = 20;

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output,
    int img_w,
    int img_h,
    int model_w,
    int model_h,
    bool parser_render,
    std::shared_ptr<DnnParserResult>& result) {
  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }
  if (node_output->output_tensors.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ClassficationOutputParser"),
                 "output_tensors is empty");
    return -1;
  }

  int valid_w = img_w > static_cast<uint32_t>(model_w)
                    ? static_cast<uint32_t>(model_w)
                    : img_w;
  int valid_h = img_h > static_cast<uint32_t>(model_h)
                    ? static_cast<uint32_t>(model_h)
                    : img_h;

  int ret = PostProcess(
      node_output->output_tensors, result->perception, valid_w, valid_h);
  if (1 == parser_render) {
    ParseRenderPostProcess(node_output->output_tensors, result->perception);
  }
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("UnetOutputParser"),
                "postprocess return error, code = %d",
                ret);
  }
  std::stringstream ss;
  ss << "UnetOutputParser parse finished, predict result: "
     << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("UnetOutputParser"), "%s", ss.str().c_str());
  return ret;
}

int PostProcess(std::vector<std::shared_ptr<DNNTensor>>& tensors,
                Perception& perception,
                int valid_w,
                int valid_h) {
  perception.type = Perception::SEG;
  hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // get shape
  int h_index, w_index, c_index;
  hobot::dnn_node::output_parser::get_tensor_hwc_index(
      tensors[0], &h_index, &w_index, &c_index);
  int height = tensors[0]->properties.validShape.dimensionSize[h_index];
  int width = tensors[0]->properties.validShape.dimensionSize[w_index];
  int channel = tensors[0]->properties.validShape.dimensionSize[c_index];

  float* data = reinterpret_cast<float*>(tensors[0]->sysMem[0].virAddr);
  int parse_valid_h = valid_h / 4;
  int parse_valid_w = valid_w / 4;
  perception.seg.data.resize(parse_valid_h * parse_valid_w * channel);
  perception.seg.valid_w = parse_valid_w;
  perception.seg.valid_h = parse_valid_h;
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes_;

  for (int h = 0; h < parse_valid_h; h++) {
    auto index = h * parse_valid_w * channel;
    auto* dst = &perception.seg.data[index];
    auto* src = data + h * width * channel;
    int copy_len = parse_valid_w * channel * sizeof(float);
    memcpy(dst, src, copy_len);
  }

  return 0;
}

int ParseRenderPostProcess(std::vector<std::shared_ptr<DNNTensor>>& tensors,
                           Perception& perception) {
  perception.type = Perception::SEG;
  hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // get shape
  int h_index, w_index, c_index;
  hobot::dnn_node::output_parser::get_tensor_hwc_index(
      tensors[0], &h_index, &w_index, &c_index);
  int height = tensors[0]->properties.validShape.dimensionSize[h_index];
  int width = tensors[0]->properties.validShape.dimensionSize[w_index];
  int channel = tensors[0]->properties.validShape.dimensionSize[c_index];

  RCLCPP_DEBUG(rclcpp::get_logger("UnetOutputParser"),
               "PostProcess width: %d height: %d channel: %d",
               width,
               height,
               channel);

  float* data = reinterpret_cast<float*>(tensors[0]->sysMem[0].virAddr);
  perception.seg.seg.resize(height * width);
  perception.seg.width = width;
  perception.seg.height = height;
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes_;

  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      float top_score = -1000000.0f;
      int top_index = 0;
      float* c_data = data + (width * h + w) * channel;
      for (int c = 0; c < channel; c++) {
        if (c_data[c] > top_score) {
          top_score = c_data[c];
          top_index = c;
        }
      }
      perception.seg.seg[h * width + w] = top_index;
    }
  }
  return 0;
}

}  // namespace parser_unet
}  // namespace dnn_node
}  // namespace hobot
