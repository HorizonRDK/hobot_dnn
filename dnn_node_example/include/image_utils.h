// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_IMAGE_UTILS_H
#define EASY_DNN_IMAGE_UTILS_H

#include <memory>
#include <string>
#include <vector>

#include "easy_dnn/data_structure.h"
#include "easy_dnn/model.h"
#include "easy_dnn/model_manager.h"
#include "easy_dnn/output_parser/classification/classification_output_parser.h"
#include "easy_dnn/output_parser/detection/bev3d_output_parser.h"
#include "easy_dnn/output_parser/detection/detection_data_structure.h"
#include "easy_dnn/output_parser/detection/detection_output_parser.h"
#include "easy_dnn/output_parser/detection/filter2d_output_parser.h"
#include "easy_dnn/output_parser/detection/real3d_output_parser.h"
#include "easy_dnn/output_parser/parsing/depth_output_parser.h"
#include "easy_dnn/output_parser/parsing/elevation_output_parser.h"
#include "easy_dnn/output_parser/parsing/parsing_output_parser.h"
#include "easy_dnn/output_parser/parsing/residual_output_parser.h"
#include "easy_dnn/task_manager.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "include/fasterrcnn_kps_output_parser.h"

#define LOGE_AND_RETURN_IF_NULL(ptr)                   \
  if (ptr == nullptr) {                                \
    std::cerr << #ptr " is null pointer" << std::endl; \
    return;                                            \
  }

using hobot::easy_dnn::Bev3DOutputDescription;
using hobot::easy_dnn::Bev3DOutputParser;
using hobot::easy_dnn::Bev3DResult;
using hobot::easy_dnn::ClassificationOutputDescription;
using hobot::easy_dnn::ClassificationOutputParser;
using hobot::easy_dnn::ClassificationResult;
using hobot::easy_dnn::DepthOutputDescription;
using hobot::easy_dnn::DepthOutputParser;
using hobot::easy_dnn::DepthResult;
using hobot::easy_dnn::DetectionOutputDescription;
using hobot::easy_dnn::DetectionOutputParser;
using hobot::easy_dnn::DetectionResult;
using hobot::easy_dnn::DNNResult;
using hobot::easy_dnn::DNNTensor;
using hobot::easy_dnn::ElevationOutputDescription;
using hobot::easy_dnn::ElevationOutputParser;
using hobot::easy_dnn::ElevationResult;
using hobot::easy_dnn::Filter2DOutputDescription;
using hobot::easy_dnn::Filter2DOutputDescriptionParser;
using hobot::easy_dnn::Filter2DOutputParser;
using hobot::easy_dnn::Filter2DResult;
using hobot::easy_dnn::InputDescription;
using hobot::easy_dnn::Model;
using hobot::easy_dnn::ModelInferTask;
using hobot::easy_dnn::ModelManager;
using hobot::easy_dnn::ModelRoiInferTask;
using hobot::easy_dnn::NV12PyramidInput;
using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;
using hobot::easy_dnn::ParsingOutputDescription;
using hobot::easy_dnn::ParsingOutputParser;
using hobot::easy_dnn::ParsingResult;
using hobot::easy_dnn::PerceptionRect;
using hobot::easy_dnn::QueryFilter;
using hobot::easy_dnn::Real3DOutputParser;
using hobot::easy_dnn::Real3dResult;
using hobot::easy_dnn::ResidualOutputDescription;
using hobot::easy_dnn::ResidualOutputParser;
using hobot::easy_dnn::ResidualResult;
using hobot::easy_dnn::TaskManager;

#define M_PI_F 3.141592653f

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

static uint8_t bgr_putpalette[] = {
    128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
    153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
    0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
    0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

static std::vector<std::vector<int>> residual_color_map = {
    {255, 0, 0},   {255, 17, 0},  {255, 34, 0},  {255, 51, 0},  {255, 68, 0},
    {255, 85, 0},  {255, 102, 0}, {255, 119, 0}, {255, 136, 0}, {255, 153, 0},
    {255, 170, 0}, {255, 187, 0}, {255, 204, 0}, {255, 221, 0}, {255, 238, 0},
    {255, 255, 0}, {213, 255, 0}, {170, 255, 0}, {128, 255, 0}, {85, 255, 0},
    {43, 255, 0},  {0, 255, 0},   {0, 255, 63},  {0, 255, 127}, {0, 255, 191},
    {0, 255, 255}, {0, 232, 255}, {0, 209, 255}, {0, 186, 255}, {0, 163, 255},
    {0, 140, 255}, {0, 116, 255}, {0, 93, 255},  {0, 70, 255},  {0, 47, 255},
    {0, 24, 255},  {0, 0, 255},   {19, 0, 255},  {39, 0, 255},  {58, 0, 255},
    {78, 0, 255},  {98, 0, 255},  {117, 0, 255}, {137, 0, 255}, {156, 0, 255},
    {176, 0, 255}, {196, 0, 255}, {215, 0, 255}, {235, 0, 255}, {255, 0, 255},
    {255, 0, 213}, {255, 0, 170}, {255, 0, 128}, {255, 0, 85},  {255, 0, 43}};

static cv::Scalar colors[] = {
    cv::Scalar(255, 0, 0),    // red
    cv::Scalar(255, 255, 0),  // yellow
    cv::Scalar(0, 255, 0),    // green
    cv::Scalar(0, 0, 255),    // blue
};

enum class ImageType {
  BGR = 0,
  NV12 = 1
};

class ImageUtils {
 public:
  static std::shared_ptr<NV12PyramidInput> GetNV12Pyramid(
      const std::string &image_file, ImageType image_type,
      int scaled_img_height, int scaled_img_width);

  static std::shared_ptr<NV12PyramidInput> GetNV12Pyramid(
      const std::string &image_file,
      int scaled_img_height,
      int scaled_img_width,
      int &original_img_height,
      int &original_img_width);

  static std::shared_ptr<NV12PyramidInput> GetNV12Pyramid(
      const cv::Mat &image,
      int scaled_img_height,
      int scaled_img_width);

  static int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);

  static void GetNV12Tensor(std::string &image_file,
                            std::shared_ptr<DNNTensor> &tensor);

  static void DrawParsing(const std::string &saving_path,
                          ParsingResult *parsing_result);

  static void DrawDepth(const std::string &image_file,
                        const std::string &saving_path,
                        DepthResult *result);

  static void DrawResidual(const std::string &image_file,
                           const std::string &saving_path,
                           ParsingResult *parsing_result,
                           ResidualResult *result);
  /**
   * Render the image with elevation
   * @param[in] image_file: input image
   * @param[in] saving_path: the path to render the image
   * @param[in] K: camera Parameters
   * @param[in] desc: description of model output
   * @param[in] result: inference output
   */
  static void DrawElevation(const std::string &image_file,
                            const std::string &saving_path,
                            cv::Mat &K,
                            ElevationOutputDescription *desc,
                            ElevationResult *result);
  /**
   * Render the image with bev3d
   * @param[in] saving_path: the path to render the image
   * @param[in] result: inference output
   */
  static void DrawBev3D(const std::string &saving_path, Bev3DResult *result);

  static void RenderingFilter2d(const std::string &image_file,
                                const std::string &saving_path,
                                std::vector<PerceptionRect> &boxes,
                                int scaled_height = 0,
                                int scaled_width = 0);

  static void RenderingLmk(const std::string &image_file,
                                const std::string &saving_path,
                                std::vector<Landmarks> &lmks,
                                int scaled_height = 0,
                                int scaled_width = 0);

  static void RenderingClassification(const std::string &image_file,
                                      std::vector<hbDNNRoi> &boxes,
                                      const std::string &saving_path,
                                      std::vector<std::string> &class_names);

  static void RenderingReal3d(const std::string &image_file,
                              const std::string &save_file,
                              const Real3dResult *real3d_result,
                              cv::Mat K,
                              cv::Mat dist_coeff,
                              float scale);
  static void RendingBgrImg(const std::string &saving_path,
                            int8_t *data,
                            int height,
                            int width);
};

#endif  // EASY_DNN_IMAGE_UTILS_H
