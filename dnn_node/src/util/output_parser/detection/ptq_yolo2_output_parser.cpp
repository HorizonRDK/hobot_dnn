// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "util/output_parser/detection/ptq_yolo2_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include <queue>
#include "rapidjson/document.h"
#include "util/output_parser/algorithm.h"
#include "util/output_parser/detection/nms.h"
#include "util/output_parser/utils.h"

namespace hobot {
namespace dnn_node {

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

int32_t Yolo2OutputParser::Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor)
{
  if (!output_tensor)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo2_detection_parser"),
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
    RCLCPP_INFO(rclcpp::get_logger("Yolo2_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  std::stringstream ss;
  ss << "Yolo2_detection_parser parse finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("Yolo2_detection_parser"),
              "%s", ss.str().c_str());
  return ret;
}

int Yolo2OutputParser::PostProcess(
  std::vector<std::shared_ptr<DNNTensor>> &tensors,
  Perception &perception)
{
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
  get_tensor_hw(*(tensors[0].get()), &height, &width);
  // int *shape = tensor->data_shape.d;
  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      for (size_t k = 0; k < anchors_table.size(); k++)
      {
        double anchor_x = anchors_table[k].first;
        double anchor_y = anchors_table[k].second;
        float *cur_data = data + k * num_pred;

        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index)
        {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());

        float confidence = (1.f / (1 + std::exp(-objness))) *
                           (1.f / (1 + std::exp(-class_pred[id])));

        if (confidence < score_threshold_)
        {
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

        if (xmin > xmax || ymin > ymax)
        {
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

}  // namespace dnn_node
}  // namespace hobot
