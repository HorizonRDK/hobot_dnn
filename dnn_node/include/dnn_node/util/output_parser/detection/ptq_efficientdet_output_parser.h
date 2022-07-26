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

#ifndef _DETECTION_PTQ_EFFICIENTDET_OUTPUT_PARSER_H_
#define _DETECTION_PTQ_EFFICIENTDET_OUTPUT_PARSER_H_

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

namespace hobot {
namespace dnn_node {

/**
 * Config definition for EfficientDet
 */
struct EfficientDetConfig {
  std::vector<std::vector<double>> anchor_scales;
  std::vector<double> anchor_ratio;
  std::vector<int> feature_strides;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<float>> scales;
};

struct EDBaseAnchor {
  EDBaseAnchor(float x1, float y1, float x2, float y2)
      : x1_(x1), y1_(y1), x2_(x2), y2_(y2) {}
  float x1_;
  float y1_;
  float x2_;
  float y2_;
};

struct EDAnchor {
  EDAnchor(float c_x, float c_y, float w, float h)
      : c_x_(c_x), c_y_(c_y), w_(w), h_(h) {}
  float c_x_;
  float c_y_;
  float w_;
  float h_;
};

extern EfficientDetConfig default_efficient_det_config;

class EfficientDetAssistParser
    : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override {
    return 0;
  }
};

/**
 * Method for post processing
 */
class EfficientDetOutputParser
    : public MultiBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

  int Setdequanti_file(const std::string &cls_name_file);

 private:
  /**
   * Post process
   * @param[in] tensor: Model output tensors
   * @param[in] image_tensor: Input image tensor
   * @param[out] perception: Perception output data
   * @return 0 if success
   */
  int PostProcess(std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
                  Perception &perception);

  int GetAnchors(std::vector<EDAnchor> &anchors,
                 int layer,
                 int feat_height,
                 int feat_width);

  int GetBboxAndScores(std::shared_ptr<DNNTensor> c_tensor,
                       std::shared_ptr<DNNTensor> bbox_tensor,
                       std::vector<Detection> &dets,
                       std::vector<EDAnchor> &anchors,
                       int class_num,
                       int tensor_index);

 private:
  EfficientDetConfig efficient_det_config_ = default_efficient_det_config;
  float score_threshold_ = 0.05;
  float nms_threshold_ = 0.5;
  int nms_top_k_ = 100;
  bool anchor_init_ = false;
  std::vector<std::vector<EDAnchor>> anchors_table_;

  std::string dequanti_file_ = "";
  bool has_dequanti_node_ = true;
  std::mutex anchors_mtx;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_PTQ_EFFICIENTDET_OUTPUT_PARSER_H_
