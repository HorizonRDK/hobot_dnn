// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.
#include "util/output_parser/detection/ptq_yolo5_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include "rapidjson/document.h"
#include "util/output_parser/algorithm.h"
#include "util/output_parser/detection/nms.h"
#include "util/output_parser/utils.h"

#include <arm_neon.h>
#include <iostream>
#include <queue>

namespace hobot {
namespace dnn_node {

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

PTQYolo5Config default_ptq_yolo5_config = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
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

int32_t Yolo5OutputParser::Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs)
{
  if (output_descriptions)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Yolo5_detection_parser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty())
    {
      RCLCPP_DEBUG(rclcpp::get_logger("Yolo5_detection_parser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Yolo5_detection_parser"),
              "dep out size: %d %d", depend_output_descs.size(),
              depend_output_tensors.size());
  if (depend_output_tensors.size() < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "depend out tensor size invalid cast");
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

  int ret = PostProcess(depend_output_tensors, result->perception);
  if (ret != 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("Yolo5_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  std::stringstream ss;
  ss << "Yolo5_detection_parser parse finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("Yolo5_detection_parser"),
              "%s", ss.str().c_str());
  return ret;
}

int Yolo5OutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    Perception &perception)
{
  perception.type = Perception::DET;
  std::vector<Detection> dets;
  for (size_t i = 0; i < output_tensors.size(); i++)
  {
    PostProcess(output_tensors[i], static_cast<int>(i), dets);
  }
  yolo5_nms(dets, nms_threshold_, nms_top_k_, perception.det, false);
  return 0;
}

void Yolo5OutputParser::PostProcess(std::shared_ptr<DNNTensor> tensor,
                                            int layer,
                                            std::vector<Detection> &dets)
{
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  int num_classes = yolo5_config_.class_num;
  int stride = yolo5_config_.strides[layer];
  int num_pred = yolo5_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo5_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo5_config_.anchors_table[layer];

  //  int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(*(tensor.get()), &height, &width);
  if (ret != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
      "get_tensor_hw failed");
  }

  int anchor_num = anchors.size();
  //  for (uint32_t a = 0; a < anchors.size(); a++) {
  if (has_dequanti_node_)
  {
    auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < height; h++)
    {
      for (int w = 0; w < width; w++)
      {
        for (int k = 0; k < anchor_num; k++)
        {
          double anchor_x = anchors[k].first;
          double anchor_y = anchors[k].second;
          float *cur_data = data + k * num_pred;
          float objness = cur_data[4];

          int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
          double x1 = 1 / (1 + std::exp(-objness)) * 1;
          double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
          double confidence = x1 * x2;

          if (confidence < score_threshold_)
          {
            continue;
          }

          float center_x = cur_data[0];
          float center_y = cur_data[1];
          float scale_x = cur_data[2];
          float scale_y = cur_data[3];

          double box_center_x =
              ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
          double box_center_y =
              ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

          double box_scale_x =
              std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
          double box_scale_y =
              std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

          double xmin = (box_center_x - box_scale_x / 2.0);
          double ymin = (box_center_y - box_scale_y / 2.0);
          double xmax = (box_center_x + box_scale_x / 2.0);
          double ymax = (box_center_y + box_scale_y / 2.0);

          if (xmax <= 0 || ymax <= 0)
          {
            continue;
          }

          if (xmin > xmax || ymin > ymax)
          {
            continue;
          }

          Bbox bbox(xmin, ymin, xmax, ymax);
          dets.emplace_back(static_cast<int>(id),
                    confidence,
                    bbox,
                    yolo5_config_.class_names[static_cast<int>(id)].c_str());
        }
        data = data + num_pred * anchors.size();
      }
    }
  } else {
    auto *data = reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr);
    auto dequantize_scale_ptr = yolo5_config_.dequantize_scale[layer].data();
    bool big_endian = false;
    // cls_data_vec for neon
    std::vector<float> cls_data_vec;
    cls_data_vec.resize(4);
    auto cls_data_ptr = cls_data_vec.data();
    for (int h = 0; h < height; h++)
    {
      for (int w = 0; w < width; w++)
      {
        for (int k = 0; k < anchor_num; k++)
        {
          double anchor_x = anchors[k].first;
          double anchor_y = anchors[k].second;
          int32_t *cur_data = data + k * num_pred;
          int offset = num_pred * k;

          float objness = Dequanti(
              cur_data[4], layer, big_endian, offset + 4, tensor->properties);

          double max_cls_data = std::numeric_limits<double>::lowest();
          int id = -1;
          for (int cls = 0; cls < num_classes; cls += 4)
          {
            float32x4_t q0 = vcvtq_f32_s32(vld1q_s32(cur_data + cls + 5));
            float32x4_t q1 = vld1q_f32(dequantize_scale_ptr + offset + cls + 5);
            float32x4_t q2 = vmulq_f32(q0, q1);
            vst1q_f32(cls_data_ptr, q2);
            for (int j = 0; j < 4; j++)
            {
              if (cls_data_vec[j] > max_cls_data)
              {
                max_cls_data = cls_data_vec[j];
                id = cls + j;
              }
            }
          }

          double x1 = 1 / (1 + std::exp(-objness)) * 1;
          double x2 = 1 / (1 + std::exp(-max_cls_data));
          double confidence = x1 * x2;

          if (confidence < score_threshold_)
          {
            continue;
          }

          float center_x = Dequanti(
              cur_data[0], layer, big_endian, offset, tensor->properties);
          float center_y = Dequanti(
              cur_data[1], layer, big_endian, offset + 1, tensor->properties);
          float scale_x = Dequanti(
              cur_data[2], layer, big_endian, offset + 2, tensor->properties);
          float scale_y = Dequanti(
              cur_data[3], layer, big_endian, offset + 3, tensor->properties);

          double box_center_x =
              ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
          double box_center_y =
              ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

          double box_scale_x =
              std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
          double box_scale_y =
              std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

          double xmin = (box_center_x - box_scale_x / 2.0);
          double ymin = (box_center_y - box_scale_y / 2.0);
          double xmax = (box_center_x + box_scale_x / 2.0);
          double ymax = (box_center_y + box_scale_y / 2.0);

          if (xmax <= 0 || ymax <= 0)
          {
            continue;
          }

          if (xmin > xmax || ymin > ymax)
          {
            continue;
          }

          Bbox bbox(xmin, ymin, xmax, ymax);
          dets.emplace_back(static_cast<int>(id),
                    confidence,
                    bbox,
                    yolo5_config_.class_names[static_cast<int>(id)].c_str());
        }
        data = data + num_pred * anchors.size() + 1;
      }
    }
  }
}

double Yolo5OutputParser::Dequanti(int32_t data,
                                           int layer,
                                           bool big_endian,
                                           int offset,
                                           hbDNNTensorProperties &properties) {
  return static_cast<double>(r_int32(data, big_endian)) *
         yolo5_config_.dequantize_scale[layer][offset];
}

}  // namespace dnn_node
}  // namespace hobot
