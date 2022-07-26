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

#include "include/post_process/post_process_base.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn_node/util/output_parser/detection/fasterrcnn_kps_output_parser.h"
#include "include/image_utils.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"

ai_msgs::msg::PerceptionTargets::UniquePtr PostProcessBase::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  const auto &outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("PostProcessBase"),
              "outputs size: %d",
              outputs.size());
  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("PostProcessBase"), "Invalid outputs");
    return nullptr;
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  auto *det_result =
      dynamic_cast<Dnn_Parser_Result *>(outputs[output_index_].get());
  if (!det_result) {
    RCLCPP_INFO(rclcpp::get_logger("PostProcessBase"), "invalid cast");
    return 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("PostProcessBase"),
              "out box size: %d",
              det_result->perception.det.size());
  for (auto &rect : det_result->perception.det) {
    if (rect.bbox.xmin < 0) rect.bbox.xmin = 0;
    if (rect.bbox.ymin < 0) rect.bbox.ymin = 0;
    if (rect.bbox.xmax >= model_input_width_) {
      rect.bbox.xmax = model_input_width_ - 1;
    }
    if (rect.bbox.ymax >= model_input_height_) {
      rect.bbox.ymax = model_input_height_ - 1;
    }

    std::stringstream ss;
    ss << "det rect: " << rect.bbox.xmin << " " << rect.bbox.ymin << " "
       << rect.bbox.xmax << " " << rect.bbox.ymax
       << ", det type: " << rect.class_name << ", score:" << rect.score;
    RCLCPP_INFO(rclcpp::get_logger("PostProcessBase"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect.bbox.xmin);
    roi.rect.set__y_offset(rect.bbox.ymin);
    roi.rect.set__width(rect.bbox.xmax - rect.bbox.xmin);
    roi.rect.set__height(rect.bbox.ymax - rect.bbox.ymin);

    ai_msgs::msg::Target target;
    target.set__type(rect.class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  return pub_data;
}
