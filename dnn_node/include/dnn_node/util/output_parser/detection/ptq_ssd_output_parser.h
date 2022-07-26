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

#ifndef _DETECTION_PTQ_SSD_OUTPUT_PARSER_H_
#define _DETECTION_PTQ_SSD_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"

namespace hobot {
namespace dnn_node {

/**
 * Config definition for SSD
 */
struct SSDConfig {
  std::vector<float> std;
  std::vector<float> mean;
  std::vector<float> offset;
  std::vector<int> step;
  std::vector<std::pair<float, float>> anchor_size;
  std::vector<std::vector<float>> anchor_ratio;
  int class_num;
  std::vector<std::string> class_names;
};

/**
 * Default ssd config
 * std: [0.1, 0.1, 0.2, 0.2]
 * mean: [0, 0, 0, 0]
 * offset: [0.5, 0.5]
 * step: [8, 16, 32, 64, 100, 300]
 * anchor_size: [[30, 60], [60, 111], [111, 162], [162, 213], [213, 264],
 *              [264,315]]
 * anchor_ratio: [[2, 0.5, 0, 0], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5,3, 1.0 / 3], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5, 0, 0], [2,0.5, 0, 0]]
 * class_num: 20
 * class_names: ["aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"]
 */
extern SSDConfig default_ssd_config;

class SSDAssistParser : public SingleBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_description,
      std::shared_ptr<DNNTensor> &output_tensor) override {
    return 0;
  }
};

class SSDOutputParser : public MultiBranchOutputParser<Dnn_Parser_Result> {
 public:
  int32_t Parse(
      std::shared_ptr<Dnn_Parser_Result> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

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

  int SsdAnchors(std::vector<Anchor> &anchors,
                 int layer,
                 int layer_height,
                 int layer_width);

  int GetBboxAndScores(std::shared_ptr<DNNTensor> c_tensor,
                       std::shared_ptr<DNNTensor> bbox_tensor,
                       std::vector<Detection> &dets,
                       std::vector<Anchor> &anchors,
                       int class_num,
                       float cut_off_threshold);

 private:
  SSDConfig ssd_config_ = default_ssd_config;
  std::vector<std::vector<Anchor>> anchors_table_;
  float score_threshold_ = 0.25;
  float nms_threshold_ = 0.45;
  bool is_performance_ = true;
  int nms_top_k_ = 200;
};

}  // namespace dnn_node
}  // namespace hobot
#endif  // _DETECTION_PTQ_SSD_OUTPUT_PARSER_H_
