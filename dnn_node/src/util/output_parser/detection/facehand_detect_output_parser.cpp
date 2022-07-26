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

#include "dnn_node/util/output_parser/detection/facehand_detect_output_parser.h"
#include "dnn_node/util/output_parser/utils.h"
namespace hobot {
namespace easy_dnn {

using cpu_op_rcnn_post_process_bbox_float_type_t = struct {
  float left;
  float top;
  float right;
  float bottom;
  float score;
  float class_label;
};

using detection_post_process_bbox_with_pad_type_t = struct {
  int16_t left;
  int16_t top;
  int16_t right;
  int16_t bottom;
  int8_t score;
  uint8_t class_label;
  int16_t padding[3];
};

int32_t FaceHandDetectionOutputParser::Parse(
    std::shared_ptr<Filter2DResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_description,
    std::shared_ptr<DNNTensor> &output_tensor) {
  auto *detection_result =
      reinterpret_cast<uint8_t *>(output_tensor->sysMem[0].virAddr);
  auto tensor_properties_type = output_tensor->properties.tensorType;

  if (HB_DNN_TENSOR_TYPE_F32 == tensor_properties_type) {
    size_t item_size = sizeof(cpu_op_rcnn_post_process_bbox_float_type_t);
    float output_byte_size = *reinterpret_cast<float *>(detection_result);
    uint32_t box_num = output_byte_size / item_size;
    auto *p_box =
        reinterpret_cast<cpu_op_rcnn_post_process_bbox_float_type_t *>(
            reinterpret_cast<uintptr_t>(detection_result) + item_size);
    for (uint32_t i = 0; i < box_num; i++) {
      PerceptionRect roi{};
      roi.left = p_box[i].left;
      roi.top = p_box[i].top;
      roi.right = p_box[i].right;
      roi.bottom = p_box[i].bottom;
      roi.conf = p_box[i].score;
      roi.type = p_box[i].class_label;
      roi.branch = output_description->GetIndex();
      output->boxes.push_back(roi);
    }
  } else {
    size_t item_size = sizeof(detection_post_process_bbox_with_pad_type_t);
    uint16_t output_byte_size = *reinterpret_cast<uint16_t *>(detection_result);
    uint16_t box_num = output_byte_size / item_size;
    auto *p_box =
        reinterpret_cast<detection_post_process_bbox_with_pad_type_t *>(
            reinterpret_cast<uintptr_t>(detection_result) + item_size);
    for (uint32_t i = 0; i < box_num; i++) {
      PerceptionRect roi{};
      roi.left = p_box[i].left;
      roi.top = p_box[i].top;
      roi.right = p_box[i].right;
      roi.bottom = p_box[i].bottom;
      roi.conf = p_box[i].score;
      roi.type = p_box[i].class_label;
      roi.branch = output_description->GetIndex();
      output->boxes.push_back(roi);
    }
  }
  return 0;
}

}  // namespace easy_dnn
}  // namespace hobot
