// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/fcos_output_parser.h"

#include "dnn/hb_dnn_ext.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"

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

int get_tensor_hwc_index(std::shared_ptr<DNNTensor> tensor, int *h_index,
                         int *w_index, int *c_index) {
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    *h_index = 1;
    *w_index = 2;
    *c_index = 3;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    *c_index = 1;
    *h_index = 2;
    *w_index = 3;
  } else {
    return -1;
  }
  return 0;
}

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

  std::shared_ptr<FcosDetResult> result;
  if (!output) {
    result = std::make_shared<FcosDetResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<FcosDetResult>(output);
    result->Reset();
  }

  int ret = PostProcess(depend_output_tensors, result->boxes);
  if (ret != 0) {
    RCLCPP_INFO(rclcpp::get_logger("fcos_detection_parser"),
                "postprocess return error, code = %d", ret);
  }
  return ret;
}

void FcosDetectionOutputParser::GetBboxAndScoresNHWC(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<Perception> &dets) {
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
        tmp_score.score =
            tmp_score.score * ce_data[ce_offset];  // std::sqrt(tmp_score.score
                                                   // * ce_data[ce_offset]);
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

        Perception detection;
        detection.x1 = xmin;
        detection.y1 = ymin;
        detection.x2 = xmax;
        detection.y2 = ymax;
        detection.conf = tmp_score.score;
        detection.type = tmp_score.id;
        detection.category_name =
            fcos_config_.class_names[tmp_score.id].c_str();
        dets.push_back(detection);
        // std::cout << "fcos output, x1:" <<  detection.x1 << "  y1:" <<
        // detection.y1
        //      << "  x2:" << detection.x2 << "  y2:" << detection.y2 << "
        //      score:" << tmp_score.score
        //      << "  type:"<< detection.category_name<< std::endl;
      }
    }
  }
}

void FcosDetectionOutputParser::GetBboxAndScoresNCHW(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<Perception> &dets) {
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

        Perception detection;
        detection.x1 = xmin;
        detection.y1 = ymin;
        detection.x2 = xmax;
        detection.y2 = ymax;
        detection.conf = tmp_score.score;
        detection.type = tmp_score.id;

        detection.conf = tmp_score.score;
        detection.type = tmp_score.id;
        detection.category_name =
            fcos_config_.class_names[tmp_score.id].c_str();
        dets.push_back(detection);
      }
    }
  }
}

int FcosDetectionOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<Perception> &det_result) {
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
  std::vector<Perception> dets;
  if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    GetBboxAndScoresNHWC(tensors, dets);
  } else if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    GetBboxAndScoresNCHW(tensors, dets);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("fcos_example"), "tensor layout error.");
  }

  nms(dets, nms_threshold_, nms_top_k_, det_result, false);
  return 0;
}

void FcosDetectionOutputParser::nms(std::vector<Perception> &input,
                                    float iou_threshold, int top_k,
                                    std::vector<Perception> &result,
                                    bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<Perception>());
  if (input.size() > 400) {
    input.resize(400);
  }

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = abs(input[i].x2 - input[i].x1);
    float height = abs(input[i].y2 - input[i].y1);
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].type != input[j].type) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].x1, input[j].x1);
      float yy1 = std::max(input[i].y1, input[j].y1);
      float xx2 = std::min(input[i].x2, input[j].x2);
      float yy2 = std::min(input[i].y2, input[j].y2);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }
    result.push_back(input[i]);
  }
}
