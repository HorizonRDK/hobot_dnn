// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_BEV3D_OUTPUT_PARSER_H
#define EASY_DNN_BEV3D_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/output_parser.h"

namespace hobot {
namespace easy_dnn {

/* json data sample
{
    "task":"bev3d",
    "output_name":"bev3d_heatmap_output",
    "score_threshold":0.2,
    "properties":[
        {
            "channel_labels":[
                "pedestrian",
                "vehicle",
                "cyclist"
            ]
        }
    ],
    "vcs_origin_coord":[
        362,
        256
    ],
    "spatial_resolution":[
        0.2,
        0.2
    ],
    "visible_range":[
        72.4,
        -30,
        51.2,
        -51.2
    ]
}
 */
class Bev3DOutputDescription : public OutputDescription {
 public:
  Bev3DOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {}
  std::string output_name;
  float score_threshold = 0.0;
  float top_k = 100;
  float front_range = 72.4;
  float back_range = -30.0;
  float left_range = 51.2;
  float right_range = -51.2;
  std::vector<std::vector<float>> cls_average_dimension = {
      {1.6383535, 0.60629284, 0.5058226},
      {1.7761209, 1.8237954, 4.79461},
      {1.5518998, 0.73560804, 1.7083853}};
  float x_resolution;
  float y_resolution;
};

class Bev3DOutputDescriptionParser : public OutputDescriptionParser {
 public:
  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;

 private:
  std::vector<int> dependencies_;
};

class Bev3DResult : public DNNResult {
 public:
  Bev3DResult() = default;
  void Reset() override { bbox3d.clear(); }

  struct Bbox3D {
    float score;
    uint16_t cls;
    uint32_t grid_idx, grid_x, grid_y;
    float x, y, z, w, l, h, d, r;
    std::vector<std::vector<float>> corners2d;  // undistort, img ord
    std::vector<std::vector<float>> corners3d;  // undistort, camera ord
    inline static bool greater(const Bbox3D &a, const Bbox3D &b) {
      return a.score > b.score;
    }
    Bbox3D() {
      corners2d.resize(8, std::vector<float>(2));
      corners3d.resize(8, std::vector<float>(3));
    }
  };

 public:
  std::vector<Bbox3D> bbox3d;
  std::vector<std::vector<float>> feature_map;
};

class Bev3DOutputParser : public MultiBranchOutputParser {
 public:
  enum FeatureMap {
    HEATMAP = 0,
    DIMENSION = 1,
    ROT = 2,
    CT_OFFSET = 3,
    LOC_Z = 4,
    MAX_LAYER = 5
  };

  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

  void MaxPoolingRefine(std::vector<Bev3DResult::Bbox3D> &bbox3d,
                        const int32_t *hm,
                        const float *scale,
                        float score_threshold,
                        int valid_c,
                        int valid_h,
                        int valid_w,
                        int aligned_h,
                        int aligned_w);

  void DecodeBbox(std::vector<Bev3DResult::Bbox3D> &bbox3d,
                  std::vector<Bev3DOutputDescription *> output_descs,
                  std::vector<float> &dim,
                  std::vector<float> &rot,
                  std::vector<float> &ct_offset,
                  std::vector<float> &loc_z);

  void Get3DBboxCorners(Bev3DResult::Bbox3D &box3d);

  inline void Bev2Vcs(Bev3DOutputDescription *output_desc,
                      float bev_x,
                      float bev_y,
                      float &vcs_x,
                      float &vcs_y) {
    // bevcoord[y, x] -> vcscoord[x, y]
    vcs_x = output_desc->front_range - bev_y * output_desc->x_resolution;
    vcs_y = output_desc->left_range - bev_x * output_desc->y_resolution;
  }
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_BEV3D_OUTPUT_PARSER_H
