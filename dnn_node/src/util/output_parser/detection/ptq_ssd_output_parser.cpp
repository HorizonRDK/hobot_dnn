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

#include "util/output_parser/detection/ptq_ssd_output_parser.h"
#include "rapidjson/document.h"
#include "util/output_parser/algorithm.h"
#include "util/output_parser/detection/nms.h"
#include "util/output_parser/utils.h"
#include <queue>

namespace hobot {
namespace dnn_node {

inline float fastExp(float x)
{
  union {
    uint32_t i;
    float f;
  } v;
  v.i = (12102203.1616540672f * x + 1064807160.56887296f);
  return v.f;
}

#define SSD_CLASS_NUM_P1 21

SSDConfig default_ssd_config = {
    {0.1, 0.1, 0.2, 0.2},
    {0, 0, 0, 0},
    {0.5, 0.5},
    {15, 30, 60, 100, 150, 300},
    {{60, -1}, {105, 150}, {150, 195}, {195, 240}, {240, 285}, {285, 300}},
    {{2, 0.5, 0, 0},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3}},
    20,
    {"aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"}};

int32_t SSDOutputParser::Parse(
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
    RCLCPP_DEBUG(rclcpp::get_logger("SSDOutputParser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty())
    {
      RCLCPP_DEBUG(rclcpp::get_logger("SSDOutputParser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("SSDOutputParser"),
              "dep out size: %d %d", depend_output_descs.size(),
              depend_output_tensors.size());
  if (depend_output_tensors.size() < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SSDOutputParser"),
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

  PostProcess(depend_output_tensors, result->perception);
  std::stringstream ss;
  ss << "PTQSSDPostProcessMethod DoProcess finished, predict result: "
      << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("SSDOutputParser"),
              "%s", ss.str().c_str());
  return 0;
}

int SSDOutputParser::PostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    Perception &perception)
{
  perception.type = Perception::DET;
  int layer_num = ssd_config_.step.size();
  if (anchors_table_.empty())
  {
    // Note: note thread safe
    anchors_table_.resize(layer_num);
    for (int i = 0; i < layer_num; i++)
    {
      int height, width;
      std::vector<Anchor> &anchors = anchors_table_[i];
      get_tensor_hw(tensors[i * 2], &height, &width);
      SsdAnchors(anchors_table_[i], i, height, width);
    }
  }

  std::vector<Detection> dets;
  for (int i = 0; i < layer_num; i++)
  {
    std::vector<Anchor> &anchors = anchors_table_[i];
    GetBboxAndScores(tensors[i * 2 + 1],
                     tensors[i * 2],
                     dets,
                     anchors,
                     ssd_config_.class_num + 1,
                     0.0001);
  }
  nms(dets, nms_threshold_, nms_top_k_, perception.det, false);
  return 0;
}

int SSDOutputParser::SsdAnchors(std::vector<Anchor> &anchors,
                                        int layer,
                                        int layer_height,
                                        int layer_width)
{
  int step = ssd_config_.step[layer];
  float min_size = ssd_config_.anchor_size[layer].first;
  float max_size = ssd_config_.anchor_size[layer].second;
  auto &anchor_ratio = ssd_config_.anchor_ratio[layer];
  for (int i = 0; i < layer_height; i++)
  {
    for (int j = 0; j < layer_width; j++)
    {
      float cy = (i + ssd_config_.offset[0]) * step;
      float cx = (j + ssd_config_.offset[1]) * step;
      anchors.emplace_back(Anchor(cx, cy, min_size, min_size));
      if (max_size > 0)
      {
        anchors.emplace_back(Anchor(cx,
                                    cy,
                                    std::sqrt(max_size * min_size),
                                    std::sqrt(max_size * min_size)));
      }
      for (int k = 0; k < 4; k++)
      {
        if (anchor_ratio[k] == 0) continue;
        float sr = std::sqrt(anchor_ratio[k]);
        float w = min_size * sr;
        float h = min_size / sr;
        anchors.emplace_back(Anchor(cx, cy, w, h));
      }
    }
  }
  return 0;
}

int SSDOutputParser::GetBboxAndScores(std::shared_ptr<DNNTensor> c_tensor,
                                      std::shared_ptr<DNNTensor> bbox_tensor,
                                      std::vector<Detection> &dets,
                                      std::vector<Anchor> &anchors,
                                      int class_num,
                                      float cut_off_threshold)
{
  int *shape = c_tensor->properties.validShape.dimensionSize;
  int32_t c_batch_size = shape[0];
  int h_idx, w_idx, c_idx;
  get_tensor_hwc_index(c_tensor, &h_idx, &w_idx, &c_idx);

  int32_t c_hnum = shape[h_idx];
  int32_t c_wnum = shape[w_idx];
  int32_t c_cnum = shape[c_idx];
  uint32_t anchor_num_per_pixel = c_cnum / class_num;

  shape = bbox_tensor->properties.validShape.dimensionSize;
  int32_t b_batch_size = shape[0];
  get_tensor_hwc_index(c_tensor, &h_idx, &w_idx, &c_idx);

  int32_t b_hnum = shape[h_idx];
  int32_t b_wnum = shape[w_idx];
  int32_t b_cnum = shape[c_idx];

  RCLCPP_DEBUG(rclcpp::get_logger("SSDOutputParser"),
    "PostProcess c_wnum:%d c_hnum:%d c_cnum:%d b_wnum:%d b_hnum:%d b_cnum: %d",
              c_wnum, c_hnum, c_cnum, b_wnum, b_hnum, b_cnum);

  assert(anchor_num_per_pixel == b_cnum / 4);
  assert(c_batch_size == b_batch_size && c_hnum == b_hnum && c_wnum == b_wnum);
  auto box_num = b_batch_size * b_hnum * b_wnum * anchor_num_per_pixel;

  hbSysFlushMem(&(c_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_cls_data = reinterpret_cast<float *>(c_tensor->sysMem[0].virAddr);

  hbSysFlushMem(&(bbox_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_box_data =
      reinterpret_cast<float *>(bbox_tensor->sysMem[0].virAddr);

  for (int i = 0; i < box_num; i++)
  {
    uint32_t res_id_cur_anchor = i * class_num;
    // get softmax sum
    double sum = 0;
    int max_id = 0;
    // TODO(@horizon.ai): fastExp only affect the final score value
    // confirm whether it affects the accuracy
    double background_score;
    if (is_performance_) {
      background_score = fastExp(raw_cls_data[res_id_cur_anchor]);
    } else {
      background_score = std::exp(raw_cls_data[res_id_cur_anchor]);
    }

    double max_score = 0;
    for (int cls = 0; cls < class_num; ++cls)
    {
      float cls_score;
      if (is_performance_) {
        cls_score = fastExp(raw_cls_data[res_id_cur_anchor + cls]);
      } else {
        cls_score = std::exp(raw_cls_data[res_id_cur_anchor + cls]);
      }
      sum += cls_score;
      /* scores should be larger than background score, or else will not be
      selected */
      if (cls != 0 && cls_score > max_score && cls_score > background_score)
      {
        max_id = cls - 1;
        max_score = cls_score;
      }
    }
    // get softmax score
    max_score = max_score / sum;

    if (max_score <= score_threshold_)
    {
      continue;
    }

    int start = i * 4;
    float dx = raw_box_data[start];
    float dy = raw_box_data[start + 1];
    float dw = raw_box_data[start + 2];
    float dh = raw_box_data[start + 3];

    auto x_min = (anchors[i].cx - anchors[i].w / 2);
    auto y_min = (anchors[i].cy - anchors[i].h / 2);
    auto x_max = (anchors[i].cx + anchors[i].w / 2);
    auto y_max = (anchors[i].cy + anchors[i].h / 2);

    auto prior_w = x_max - x_min;
    auto prior_h = y_max - y_min;
    auto prior_center_x = (x_max + x_min) / 2;
    auto prior_center_y = (y_max + y_min) / 2;
    auto decode_x = ssd_config_.std[0] * dx * prior_w + prior_center_x;
    auto decode_y = ssd_config_.std[1] * dy * prior_h + prior_center_y;
    auto decode_w = std::exp(ssd_config_.std[2] * dw) * prior_w;
    auto decode_h = std::exp(ssd_config_.std[3] * dh) * prior_h;

    auto xmin = (decode_x - decode_w * 0.5);
    auto ymin = (decode_y - decode_h * 0.5);
    auto xmax = (decode_x + decode_w * 0.5);
    auto ymax = (decode_y + decode_h * 0.5);

    xmin = std::max(xmin, 0.0);
    ymin = std::max(ymin, 0.0);

    if (xmax <= 0 || ymax <= 0) continue;
    if (xmin > xmax || ymin > ymax) continue;

    Bbox bbox(xmin, ymin, xmax, ymax);
    dets.emplace_back(static_cast<int>(max_id),
            max_score,
            bbox,
            ssd_config_.class_names[max_id].c_str());
  }
  return 0;
}

}  // namespace dnn_node
}  // namespace hobot
