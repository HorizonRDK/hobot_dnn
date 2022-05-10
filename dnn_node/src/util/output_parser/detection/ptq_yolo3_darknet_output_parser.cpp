// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "util/output_parser/detection/ptq_yolo3_darknet_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include <queue>

#include "rapidjson/document.h"
#include "util/output_parser/algorithm.h"
#include "util/output_parser/detection/nms.h"
#include "util/output_parser/utils.h"

namespace hobot {
namespace dnn_node {

PTQYolo3DarknetConfig default_ptq_yolo3_darknet_config = {
    {32, 16, 8},
    {{{3.625, 2.8125}, {4.875, 6.1875}, {11.65625, 10.1875}},
     {{1.875, 3.8125}, {3.875, 2.8125}, {3.6875, 7.4375}},
     {{1.25, 1.625}, {2.0, 3.75}, {4.125, 2.875}}},
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

int32_t Yolo3DarknetOutputParser::Parse(
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
    RCLCPP_DEBUG(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty())
    {
      RCLCPP_DEBUG(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
              "dep out size: %d %d", depend_output_descs.size(),
              depend_output_tensors.size());
  if (depend_output_tensors.size() < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
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
    RCLCPP_INFO(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  std::stringstream ss;
  ss << "PTQYolo3DarknetOutputParser parse finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("Yolo3Darknet_detection_parser"),
              "%s", ss.str().c_str());
  return ret;
}

int Yolo3DarknetOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    Perception &perception)
{
  perception.type = Perception::DET;
  std::vector<Detection> dets;
  for (size_t i = 0; i < yolo3_config_.strides.size(); i++)
  {
    if (tensors[i]->properties.tensorLayout == HB_DNN_LAYOUT_NHWC)
    {
      PostProcessNHWC(tensors[i], i, dets);
    } else if (tensors[i]->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
      PostProcessNCHW(tensors[i], i, dets);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("dnn_ptq_yolo3"), "tensor layout error.");
    }
  }
  nms(dets, nms_threshold_, nms_top_k_, perception.det, false);
  return 0;
}

void Yolo3DarknetOutputParser::PostProcessNHWC(
    std::shared_ptr<DNNTensor> tensor,
    int layer,
    std::vector<Detection> &dets)
{
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = yolo3_config_.class_num;
  int stride = yolo3_config_.strides[layer];
  int num_pred = yolo3_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo3_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo3_config_.anchors_table[layer];

  // int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(*(tensor.get()), &height, &width);
  if (ret != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("dnn_ptq_yolo3"), "get_tensor_hw failed");
  }

  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      for (size_t k = 0; k < anchors.size(); k++)
      {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index)
        {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
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
        dets.push_back(Detection(static_cast<int>(id),
                      confidence,
                      bbox,
                      yolo3_config_.class_names[static_cast<int>(id)].c_str()));
      }
      data = data + num_pred * anchors.size();
    }
  }
}

void Yolo3DarknetOutputParser::PostProcessNCHW(
    std::shared_ptr<DNNTensor> tensor,
    int layer,
    std::vector<Detection> &dets)
{
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = yolo3_config_.class_num;
  int stride = yolo3_config_.strides[layer];
  int num_pred = yolo3_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo3_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo3_config_.anchors_table[layer];

  int height, width;
  auto ret = get_tensor_hw(*(tensor.get()), &height, &width);
  if (ret != 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("dnn_ptq_yolo3"), "get_tensor_hw failed");
  }
  int aligned_h = tensor->properties.validShape.dimensionSize[2];
  int aligned_w = tensor->properties.validShape.dimensionSize[3];
  int aligned_hw = aligned_h * aligned_w;

  for (size_t k = 0; k < anchors.size(); k++)
  {
    for (int h = 0; h < height; h++)
    {
      for (int w = 0; w < width; w++)
      {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        int stride_hw = h * aligned_w + w;

        float objness = data[(k * num_pred + 4) * aligned_hw + stride_hw];
        for (int index = 0; index < num_classes; ++index)
        {
          class_pred[index] =
              data[(k * num_pred + index + 5) * aligned_hw + stride_hw];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

        if (confidence < score_threshold_)
        {
          continue;
        }

        float center_x = data[(k * num_pred) * aligned_hw + stride_hw];
        float center_y = data[(k * num_pred + 1) * aligned_hw + stride_hw];
        float scale_x = data[(k * num_pred + 2) * aligned_hw + stride_hw];
        float scale_y = data[(k * num_pred + 3) * aligned_hw + stride_hw];

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
        dets.push_back(Detection(static_cast<int>(id),
                      confidence,
                      bbox,
                      yolo3_config_.class_names[static_cast<int>(id)].c_str()));
      }
    }
  }
}

}  // namespace dnn_node
}  // namespace hobot
