// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "util/output_parser/detection/fcos_output_parser.h"
#include "rclcpp/rclcpp.hpp"
#include "util/output_parser/algorithm.h"
#include "util/output_parser/detection/nms.h"
#include "util/output_parser/utils.h"

#include <iostream>
#include <queue>

namespace hobot {
namespace dnn_node {

FcosConfig default_fcos_config = {
    {{8, 16, 32, 64, 128}},
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
     "hair drier",    "toothbrush"},
    ""};

struct ScoreId {
  float score;
  int id;
};

int32_t FcosDetectionOutputParser::Parse(
    std::shared_ptr<DNNResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_descriptions,
    std::shared_ptr<DNNTensor> &output_tensor,
    std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
    std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
    std::vector<std::shared_ptr<DNNResult>> &depend_outputs) {
  if (output_descriptions) {
    RCLCPP_DEBUG(rclcpp::get_logger("fcos_detection_parser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("fcos_detection_parser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("fcos_detection_parser"),
              "dep out size: %d %d", depend_output_descs.size(),
              depend_output_tensors.size());
  if (depend_output_tensors.size() < 15) {
    RCLCPP_ERROR(rclcpp::get_logger("fcos_detection_parser"),
                 "depend out tensor size invalid cast");
    return -1;
  }

  std::shared_ptr<Dnn_Parser_Result> result;
  if (!output) {
    result = std::make_shared<Dnn_Parser_Result>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<Dnn_Parser_Result>(output);
  }

  int ret = PostProcess(depend_output_tensors, result->perception);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("fcos_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  return ret;
}

void FcosDetectionOutputParser::GetBboxAndScoresNHWC(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<Detection> &dets) {
  // fcos stride is {8, 16, 32, 64, 128}
  for (size_t i = 0; i < fcos_config_.strides.size(); ++i) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i]->sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5]->sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10]->sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    int *shape = tensors[i]->properties.alignedShape.dimensionSize;
    int tensor_h = shape[1];
    int tensor_w = shape[2];
    int tensor_c = shape[3];

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        int cls_offset = ce_offset * tensor_c;
        ScoreId tmp_score = {cls_data[cls_offset], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_offset + cls_c;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        int index = 4 * (h * tensor_w + w);
        double xmin = (w - bbox_data[index]);
        xmin *= fcos_config_.strides[i];

        double ymin = (h - bbox_data[index + 1]);
        ymin *= fcos_config_.strides[i];

        double xmax = (w + bbox_data[index + 2]);
        xmax *= fcos_config_.strides[i];

        double ymax = (h + bbox_data[index + 3]);
        ymax *= fcos_config_.strides[i];

        Detection detection;
        detection.bbox.xmin = xmin;
        detection.bbox.ymin = ymin;
        detection.bbox.xmax = xmax;
        detection.bbox.ymax = ymax;
        detection.score = tmp_score.score;
        detection.id = tmp_score.id;
        detection.class_name =
            fcos_config_.class_names[tmp_score.id].c_str();
        dets.push_back(detection);
      }
    }
  }
}

void FcosDetectionOutputParser::GetBboxAndScoresNCHW(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<Detection> &dets) {
  auto &strides = fcos_config_.strides;
  for (size_t i = 0; i < strides.size(); ++i) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i]->sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5]->sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10]->sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    int *shape = tensors[i]->properties.alignedShape.dimensionSize;
    int tensor_c = shape[1];
    int tensor_h = shape[2];
    int tensor_w = shape[3];
    int aligned_hw = tensor_h * tensor_w;

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        ScoreId tmp_score = {cls_data[offset + w], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_c * aligned_hw + offset + w;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        int index = 4 * (h * tensor_w + w);
        double xmin = (w - bbox_data[index]);
        xmin *= fcos_config_.strides[i];

        double ymin = (h - bbox_data[index + 1]);
        ymin *= fcos_config_.strides[i];

        double xmax = (w + bbox_data[index + 2]);
        xmax *= fcos_config_.strides[i];

        double ymax = (h + bbox_data[index + 3]);
        ymax *= fcos_config_.strides[i];

        Detection detection;
        detection.bbox.xmin = xmin;
        detection.bbox.ymin = ymin;
        detection.bbox.xmax = xmax;
        detection.bbox.ymax = ymax;
        detection.score = tmp_score.score;
        detection.id = tmp_score.id;
        detection.class_name =
            fcos_config_.class_names[tmp_score.id].c_str();
        dets.push_back(detection);
      }
    }
  }
}

int FcosDetectionOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    Perception &perception) {
  if (!tensors[0]) {
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "tensor layout error.");
    return -1;
  }
  int h_index, w_index, c_index;
  int ret = get_tensor_hwc_index(tensors[0], &h_index, &w_index, &c_index);
  if (ret != 0 &&
      fcos_config_.class_names.size() !=
          tensors[0]->properties.alignedShape.dimensionSize[c_index]) {
    RCLCPP_INFO(rclcpp::get_logger("fcos_detection_parser"),
                "User det_name_list in config file: %s, is not compatible with "
                "this model, %d  %d",
                fcos_config_.det_name_list.c_str(),
                fcos_config_.class_names.size(),
                tensors[0]->properties.alignedShape.dimensionSize[c_index]);
  }
  for (int i = 0; i < tensors.size(); i++) {
    if (!tensors[i]) {
      RCLCPP_INFO(rclcpp::get_logger("fcos_example"),
                  "tensor layout null, error.");
      return -1;
    }
    hbSysFlushMem(&(tensors[i]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }
  std::vector<std::vector<ScoreId>> scores;
  std::vector<Detection> dets;
  if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    GetBboxAndScoresNHWC(tensors, dets);
  } else if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    GetBboxAndScoresNCHW(tensors, dets);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "tensor layout error.");
  }

  yolo5_nms(dets, nms_threshold_, nms_top_k_, perception.det, false);
  return 0;
}

}  // namespace dnn_node
}  // namespace hobot
