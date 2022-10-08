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

#include "dnn_node/util/output_parser/detection/ptq_yolo2_output_parser.h"

#include <queue>

#include "dnn_node/util/output_parser/detection/nms.h"
#include "dnn_node/util/output_parser/utils.h"
#include "rapidjson/document.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace dnn_node {
namespace parser_yolov2 {

/**
 * Finds the greatest element in the range [first, last)
 * @tparam[in] ForwardIterator: iterator type
 * @param[in] first: fist iterator
 * @param[in] last: last iterator
 * @return Iterator to the greatest element in the range [first, last)
 */
template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

/**
 * Config definition for Yolo2
 */
struct PTQYolo2Config {
  int stride;
  std::vector<std::pair<double, double>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

  std::string Str() {
    std::stringstream ss;
    ss << "stride: " << stride;
    ss << "; anchors_table: ";
    for (auto anchors : anchors_table) {
      ss << "[" << anchors.first << "," << anchors.second << "] ";
    }
    ss << "; class_num: " << class_num;
    return ss.str();
  }
};

PTQYolo2Config default_ptq_yolo2_config = {
    32,
    {{0.57273, 0.677385},
     {1.87446, 2.06253},
     {3.33843, 5.47434},
     {7.88282, 3.52778},
     {9.77052, 9.16828}},
    80,
    {"person",        "bicycle",      "car",
     "motorcycle",    "airplane",     "bus",
     "train",         "truck",        "boat",
     "traffic light", "fire hydrant", "stop sign",
     "parking meter", "bench",        "bird",
     "cat",           "dog",          "horse",
     "sheep",         "cow",          "elephant",
     "bear",          "zebra",        "giraffe",
     "backpack",      "umbrella",     "handbag",
     "tie",           "suitcase",     "frisbee",
     "skis",          "snowboard",    "sports ball",
     "kite",          "baseball bat", "baseball glove",
     "skateboard",    "surfboard",    "tennis racket",
     "bottle",        "wine glass",   "cup",
     "fork",          "knife",        "spoon",
     "bowl",          "banana",       "apple",
     "sandwich",      "orange",       "broccoli",
     "carrot",        "hot dog",      "pizza",
     "donut",         "cake",         "chair",
     "couch",         "potted plant", "bed",
     "dining table",  "toilet",       "tv",
     "laptop",        "mouse",        "remote",
     "keyboard",      "cell phone",   "microwave",
     "oven",          "toaster",      "sink",
     "refrigerator",  "book",         "clock",
     "vase",          "scissors",     "teddy bear",
     "hair drier",    "toothbrush"}};

int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                Perception &perception);

PTQYolo2Config yolo2_config_ = default_ptq_yolo2_config;
float score_threshold_ = 0.3;
float nms_threshold_ = 0.45;
int nms_top_k_ = 500;

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::shared_ptr<DnnParserResult> &result) {
  if (!result) {
    result = std::make_shared<DnnParserResult>();
  }

  int ret = PostProcess(node_output->output_tensors, result->perception);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("Yolo2_detection_parser"),
                "postprocess return error, code = %d",
                ret);
  }
  std::stringstream ss;
  ss << "Yolo2_detection_parser parse finished, predict result: "
     << result->perception;
  RCLCPP_DEBUG(
      rclcpp::get_logger("Yolo2_detection_parser"), "%s", ss.str().c_str());
  return ret;
}

int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &tensors,
                Perception &perception) {
  perception.type = Perception::DET;
  hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensors[0]->sysMem[0].virAddr);
  auto &anchors_table = yolo2_config_.anchors_table;
  int num_classes = yolo2_config_.class_num;
  float stride = static_cast<float>(yolo2_config_.stride);
  int num_pred = num_classes + 4 + 1;
  std::vector<Detection> dets;
  std::vector<float> class_pred(num_classes, 0.0);

  int height, width;
  hobot::dnn_node::output_parser::get_tensor_hw(tensors[0], &height, &width);
  // int *shape = tensor->data_shape.d;
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      for (size_t k = 0; k < anchors_table.size(); k++) {
        double anchor_x = anchors_table[k].first;
        double anchor_y = anchors_table[k].second;
        float *cur_data = data + k * num_pred;

        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());

        float confidence = (1.f / (1 + std::exp(-objness))) *
                           (1.f / (1 + std::exp(-class_pred[id])));

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;

        double box_scale_x = std::exp(scale_x) * anchor_x * stride;
        double box_scale_y = std::exp(scale_y) * anchor_y * stride;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        if (xmin > xmax || ymin > ymax) {
          continue;
        }

        Bbox bbox(xmin, ymin, xmax, ymax);
        dets.emplace_back(
            Detection(static_cast<int>(id),
                      confidence,
                      bbox,
                      yolo2_config_.class_names[static_cast<int>(id)].c_str()));
      }
      data = data + num_pred * anchors_table.size();
    }
  }

  nms(dets, nms_threshold_, nms_top_k_, perception.det, false);
  return 0;
}

}  // namespace parser_yolov2
}  // namespace dnn_node
}  // namespace hobot
