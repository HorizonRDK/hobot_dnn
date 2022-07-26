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

#include "dnn_node/util/output_parser/detection/filter2d_output_parser.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "dnn_node/util/output_parser/utils.h"

namespace hobot {
namespace easy_dnn {

#define NULLPTR_ERR(ptr) #ptr " is null pointer"

#define RETURN_IF_FAILED(statement) \
  do {                              \
    auto code_ = (statement);       \
    if ((code_) != 0) {             \
      return code_;                 \
    }                               \
  } while (0)

#define LOGE_AND_RETURN_IF(condition, err_msg, code) \
  do {                                               \
    if (condition) {                                 \
      std::cout << (err_msg) << std::endl;           \
      return code;                                   \
    }                                                \
  } while (0)

#define LOGE_AND_RETURN_IF_NULL(ptr, code) \
  LOGE_AND_RETURN_IF(ptr == nullptr, NULLPTR_ERR(ptr), code)

#define LOGE_AND_RETURN_IF_NO_MEMBER(key, desc_doc)           \
  if (!(desc_doc).HasMember(key)) {                           \
    std::cout << "key '" << (key) << "' not exits in desc\n"; \
    break;                                                    \
  }

static inline float Sigmoid(float x) { return 1.0 / (1 + exp(-x)); }

#define CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

PerceptionRect Distance2BBox(const std::vector<int> &point,
                             const filter_2d_post_process_type &box_info,
                             int num_classes,
                             const float *scale,
                             int stride,
                             int max_h,
                             int max_w) {
  PerceptionRect rect{};
  float x1 = point[0] - box_info.data[num_classes] * scale[0] * stride;
  float y1 = point[1] - box_info.data[num_classes + 1] * scale[1] * stride;
  float x2 = point[0] + box_info.data[num_classes + 2] * scale[2] * stride;
  float y2 = point[1] + box_info.data[num_classes + 3] * scale[3] * stride;
  rect.left = CLAMP(x1, 0, max_w - 1);
  rect.top = CLAMP(y1, 0, max_h - 1);
  rect.right = CLAMP(x2, 0, max_w - 1);
  rect.bottom = CLAMP(y2, 0, max_h - 1);
  return rect;
}

int32_t Filter2DOutputParser::Parse(
    std::shared_ptr<Filter2DResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_description,
    std::shared_ptr<DNNTensor> &output_tensor) {
  auto *output_desc =
      dynamic_cast<Filter2DOutputDescription *>(output_description.get());

  LOGE_AND_RETURN_IF_NULL(output_desc, -1);
  // LOGD << "Filter2d output parser start," << output_desc;

  int num_classes = output_desc->classes_names.size();

  int input_height = output_desc->input_height;
  int input_width = output_desc->input_width;
  int stride = output_desc->stride;
  int feature_width = output_desc->feature_width;

  // reduce floating-point type operations
  // sigmoid (x) > y is equivalent to x > -(log(1/y)-1) and
  // x * f > y is equivalent to x > y/f
  // so sigmoid (x * f) > y is equivalent to x > -(log(1/y)-1)/f
  auto score_inv =
      static_cast<float>(-log(1.0 / output_desc->score_threshold - 1));
  const float *scale = output_tensor->properties.scale.scaleData + 8;
  std::vector<int8_t> score_thrs;
  for (int class_id = 0; class_id < num_classes; ++class_id) {
    score_thrs.push_back(
        static_cast<int8_t>(floor(score_inv / scale[class_id])));
  }

  auto box_num =
      *reinterpret_cast<uint16_t *>(output_tensor->sysMem[0].virAddr);
  // LOGD << "Boxes number: " << box_num;
  auto *box_addr = reinterpret_cast<filter_2d_post_process_type *>(
      output_tensor->sysMem[0].virAddr);

  std::vector<std::vector<int>> &points = output_desc->feature_points;

  // skip first 16 bytes, the first 16 bytes are the number of boxes
  box_addr++;
  for (int box_id = 0; box_id < box_num; ++box_id) {
    for (int class_id = 0; class_id < num_classes; ++class_id) {
      int8_t score = box_addr[box_id].data[class_id];
      // check the threshold
      if (score > score_thrs[class_id]) {
        int point_idx =
            box_addr[box_id].h_index * feature_width + box_addr[box_id].w_index;
        auto rect = Distance2BBox(points[point_idx],
                                  box_addr[box_id],
                                  num_classes,
                                  scale + num_classes,
                                  stride,
                                  input_height,
                                  input_width);
        float centerness =
            static_cast<float>(box_addr[box_id].data[num_classes + 4]) *
            scale[num_classes + 4];
        centerness = 1.0 / (1 + std::exp(-centerness));
        rect.conf =
            std::sqrt(Sigmoid(static_cast<float>(score) * scale[class_id]) *
                      centerness) *
            10;
        rect.perception_type = GetTypeID(output_desc->classes_names[class_id]);
        output->boxes.emplace_back(rect);
      }
    }
  }
  // LOGD << "result box size: " << output->boxes.size();
  // LOGD << "Filter2d output parser finished";
  return 0;
}

int Filter2DOutputParser::GetTypeID(const std::string &class_name) {
  static std::unordered_map<std::string, int> perception_type_map = {
      {"person", PerceptionType_Person},
      {"vehicle_rear", PerceptionType_Veh_Rear},
      {"traffic_light", PerceptionType_TrafficLight},
      {"cyclist", PerceptionType_Cyclist},
      {"vehicle", PerceptionType_FullCar},
      {"traffic_sign", PerceptionType_TrafficSign},
      {"road_arrow", PerceptionType_TrafficArrow},
      {"traffic_cone", PerceptionType_TrafficCone}};
  if (perception_type_map.find(class_name) == perception_type_map.end()) {
    return PerceptionType_Other;
  }
  return perception_type_map[class_name];
}

void Filter2DOutputParser::NMS(std::vector<PerceptionRect> &candidates,
                               std::vector<PerceptionRect> &result,
                               const float overlap_ratio,
                               const size_t top_N,
                               const bool add_score) {
  if (candidates.empty()) {
    return;
  }
  std::vector<bool> skip(candidates.size(), false);
  std::stable_sort(
      candidates.begin(), candidates.end(), std::greater<PerceptionRect>());
  size_t skip_size = candidates.size();
  size_t count = 0;
  for (size_t i = 0; i < skip_size; ++i) {
    if (skip[i]) {
      continue;
    }
    if (count >= top_N) {
      break;
    }
    PerceptionRect rect_i = candidates[i];
    skip[i] = true;
    ++count;
    float area_i = (rect_i.right - rect_i.left) * (rect_i.bottom - rect_i.top);

    // suppress the significantly covered bbox
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (candidates[i].perception_type != candidates[j].perception_type) {
        continue;
      }
      // get intersections
      PerceptionRect rect_j = candidates[j];
      float xx1 = std::max(rect_i.left, rect_j.left);
      float yy1 = std::max(rect_i.top, rect_j.top);
      float xx2 = std::min(rect_i.right, rect_j.right);
      float yy2 = std::min(rect_i.bottom, rect_j.bottom);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);

      if (area_intersection_valid) {
        // compute overlap
        float area_j =
            (rect_j.right - rect_j.left) * (rect_j.bottom - rect_j.top);
        float o = area_intersection / (area_i + area_j - area_intersection);

        if (o > overlap_ratio) {
          skip[j] = true;
          if (add_score) {
            rect_i.conf += rect_j.conf;
          }
        }
      }
    }
    result.push_back(candidates[i]);
  }
}

}  // namespace easy_dnn
}  // namespace hobot
