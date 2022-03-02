// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_REAL3D_OUTPUT_PARSER_H
#define EASY_DNN_REAL3D_OUTPUT_PARSER_H

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/output_parser.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

namespace hobot {
namespace easy_dnn {

/* json data sample
{
  "task": "camera_3d_detection",
  "output_name": "heatmap_output",
  "focal_length_default": 2411.0,
  "score_threshold": 0.3,
  "properties": [{"channel_labels": ["pedestrian", "vehicle", "cyclist"]}],
  "roi_input": {"fp_x": 480, "fp_y": 270, "width": 960, "height": 512},
  "vanishing_point": [480, 270],
  "undistort_point_method": "Pinhole"
}
 */

class Real3DOutputDescription : public OutputDescription {
 public:
  Real3DOutputDescription(Model *mode, int index, std::string type = "")
      : OutputDescription(mode, index, type) {
    output_name = "";
    score_threshold = 0.0f;
    use_multibin = 1;
    focal_length_default = 0.0f;
    score_thresh = 0.5f;
    input_resize_ratio = 1.0f;
    undistort_method = 0;
    model = nullptr;
    branch_id = -1;
    labels_.clear();
  }
  std::string output_name;
  float score_threshold;
  int use_multibin;
  float focal_length_default;
  float score_thresh;
  // 窄角模型比较特殊，第一个op是resize，所以本质上模型的尺寸是resize之后的，这个参数就是resize的倍数
  float input_resize_ratio;
  int undistort_method;
  std::vector<std::unordered_map<int, std::string>> labels_;
  Model *model;  // 为了得到模型信息
  int branch_id;
};

class Real3DOutputDescriptionParser : public OutputDescriptionParser {
 public:
  Real3DOutputDescriptionParser() = default;

  std::pair<std::shared_ptr<OutputDescription>, std::shared_ptr<OutputParser>>
  Parse(rapidjson::Document &desc_doc,
        Model *model,
        int32_t output_index) override;

 private:
  std::vector<int> dependencies_;
};

class Real3dResult : public DNNResult {
 public:
  Real3dResult() = default;
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
};

class Real3DOutputParser : public MultiBranchOutputParser {
 public:
  Real3DOutputParser() = default;

  // 这里禁止拷贝和赋值构造函数是因为成员变量once_flag不允许这种操作
  Real3DOutputParser(const Real3DOutputParser &) = delete;
  Real3DOutputParser &operator=(const Real3DOutputParser &) = delete;

  enum FeatureMap {
    HEATMAP = 0,
    DEPTH = 1,
    ROTATION = 2,
    DIMENSION = 3,
    LOCATION = 4,
    WH = 5,
    MAX_LAYER = 6
  };

  struct Bbox2D {
    float x1, y1, x2, y2;
    float score;
    uint32_t cls;
    inline static bool greater(const Bbox2D &a, const Bbox2D &b) {
      return a.score > b.score;
    }
  };

  int32_t Parse(
      std::shared_ptr<DNNResult> &output,
      std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
      std::shared_ptr<OutputDescription> &output_descriptions,
      std::shared_ptr<DNNTensor> &output_tensor,
      std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
      std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
      std::vector<std::shared_ptr<DNNResult>> &depend_outputs) override;

  void MaxPoolingRefine(std::vector<Real3dResult::Bbox3D> &bbox3d,
                        const int32_t *hm,
                        const float *scale,
                        float score_threshold,
                        int valid_c,
                        int valid_h,
                        int valid_w,
                        int aligned_h,
                        int aligned_w);

  /**
   * 初始化cam相关参数，暂时不考虑线程安全
   * @param[in] K 原图对应的内参矩阵
   * @param[in] dist_coeff 畸变参数
   * @param[in] pitch 俯仰角
   * @param[in] shift_x 原图crop的起始坐标
   * @param[in] shift_y 原图crop的起始坐标
   * @param[in] scale 原图与模型输入尺寸的比值
   * @return 0 if success, return defined error code otherwise
   */
  void InitCamParam(cv::Mat K,
                    cv::Mat dist_coeff,
                    float pitch,
                    float shift_x,
                    float shift_y,
                    float scale);

 private:
  void UpdateCaliMatrix(cv::Mat K, float shift_x, float shift_y);

  void DecodeBbox(const std::vector<std::vector<float>> &feature_map,
                  std::vector<Real3dResult::Bbox3D> &bbox3d,
                  float scale,
                  bool use_multibin,
                  float down_ratio,
                  int undistort_method);

  void ProjLocTo3D(const float cx,
                   float cy,
                   float depth,
                   std::vector<float> &loc3d);

  float GetMultiBinAlpha(const float *rot, uint32_t idx, uint32_t rot_dim = 8);

  float GetSimpleAlpha(const float *rot, uint32_t idx, uint32_t rot_dim = 2);

  float GetMultiBinRotY(float alpha, float cx);

  float GetSimpleRotY(float alpha, std::vector<float> &loc);

  void Get3DBboxCorners(std::vector<float> &loc3d, Real3dResult::Bbox3D *bbox);

  void ProjectToImage(Real3dResult::Bbox3D *bbox);

  int NmsBox2D(std::vector<Real3dResult::Bbox3D> &bbox3d,
               std::vector<Bbox2D> &bbox2d,
               bool suppress);

  void ConvertCornerToStandupBox(std::vector<Real3dResult::Bbox3D> &bbox3d,
                                 std::vector<Bbox2D> &bbox2d_bev);

  int NmsBev(std::vector<Real3dResult::Bbox3D> &bbox3d,
             std::vector<Bbox2D> &bbox2d_bev,
             bool suppress);

 private:
  cv::Mat dist_coeff_;  // 畸变系数
  // 缩放平移后的内参矩阵(一般模型输入是原图的下采样，并且可能会有crop操作，所以内参矩阵也需要做相应的计算)
  std::vector<std::vector<float>> scale_K_;
  float pitch_;    // 俯仰角
  float focal_u_;  // 原图对应的内参fu
  // 原图尺寸与模型输入的比值(如果带有resize op，则需要再乘以resize的倍数)
  float img_scale_;
  float shift_x_;  // 原图crop的起始坐标
  float shift_y_;  // 原图crop的起始坐标
  std::once_flag once_flag_;
};
}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_REAL3D_OUTPUT_PARSER_H
