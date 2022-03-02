// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <memory>

#include "include/image_utils.h"
#include "dnn/hb_sys.h"

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const std::string &image_file, ImageType image_type,
    int scaled_img_height, int scaled_img_width) {
  if (ImageType::BGR == image_type) {
    int original_img_height = 0, original_img_width = 0;
    return GetNV12Pyramid(image_file,
                          scaled_img_height,
                          scaled_img_width,
                          original_img_height,
                          original_img_width);
  } else if (ImageType::NV12 == image_type) {
    std::shared_ptr<NV12PyramidInput> pyramid = nullptr;
    std::ifstream ifs(image_file, std::ios::in | std::ios::binary);
    if (!ifs) {
      return pyramid;
    }
    ifs.seekg(0, std::ios::end);
    int len = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char *data = new char[len];
    ifs.read(data, len);

    int y_img_len = len / 3 * 2;
    int uv_img_len = len / 3;

    auto *y = new hbSysMem;
    auto *uv = new hbSysMem;

    auto w_stride = ALIGN_16(scaled_img_width);
    hbSysAllocCachedMem(y, scaled_img_height * w_stride);
    hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

    memcpy(reinterpret_cast<uint8_t *>(y->virAddr), data,
          y_img_len);
    memcpy(reinterpret_cast<uint8_t *>(uv->virAddr),
          data + y_img_len, uv_img_len);

    hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
    hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
    auto pym_in = new NV12PyramidInput;
    pym_in->width = scaled_img_width;
    pym_in->height = scaled_img_height;
    pym_in->y_vir_addr = y->virAddr;
    pym_in->y_phy_addr = y->phyAddr;
    pym_in->y_stride = w_stride;
    pym_in->uv_vir_addr = uv->virAddr;
    pym_in->uv_phy_addr = uv->phyAddr;
    pym_in->uv_stride = w_stride;
    pyramid = std::shared_ptr<NV12PyramidInput>(
        pym_in, [y, uv](NV12PyramidInput *pym_in) {
          // Release memory after deletion
          hbSysFreeMem(y);
          hbSysFreeMem(uv);
          delete y;
          delete uv;
          delete pym_in;
        });
    return pyramid;
  } else {
    return nullptr;
  }
}

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const std::string &image_file,
    int scaled_img_height,
    int scaled_img_width,
    int &original_img_height,
    int &original_img_width) {
  cv::Mat nv12_mat;
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  cv::Mat mat_tmp;
  mat_tmp.create(scaled_img_height, scaled_img_width, bgr_mat.type());
  cv::resize(bgr_mat, mat_tmp, mat_tmp.size(), 0, 0);
  // cv::imwrite("resized_img.jpg", mat_tmp);
  auto ret = ImageUtils::BGRToNv12(mat_tmp, nv12_mat);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }
  original_img_height = bgr_mat.rows;
  original_img_width = bgr_mat.cols;

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  // padding y
  for (int h = 0; h < scaled_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    for (int w = 0; w < scaled_img_width; ++w) {
      *raw++ = *data++;
    }
  }

  // padding uv
  auto uv_data = nv12_mat.data + scaled_img_height * scaled_img_width;
  for (int32_t h = 0; h < scaled_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    for (int32_t w = 0; w < scaled_img_width; ++w) {
      *raw++ = *uv_data++;
    }
  }

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pyramid = new NV12PyramidInput;
  pyramid->width = scaled_img_width;
  pyramid->height = scaled_img_height;
  pyramid->y_vir_addr = y->virAddr;
  pyramid->y_phy_addr = y->phyAddr;
  pyramid->y_stride = w_stride;
  pyramid->uv_vir_addr = uv->virAddr;
  pyramid->uv_phy_addr = uv->phyAddr;
  pyramid->uv_stride = w_stride;
  return std::shared_ptr<NV12PyramidInput>(
      pyramid, [y, uv](NV12PyramidInput *pyramid) {
        // Release memory after deletion
        hbSysFreeMem(y);
        hbSysFreeMem(uv);
        delete y;
        delete uv;
        delete pyramid;
      });
}

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const cv::Mat &bgr_mat,
    int scaled_img_height,
    int scaled_img_width) {
  cv::Mat nv12_mat;
  cv::Mat mat_tmp;
  mat_tmp.create(scaled_img_height, scaled_img_width, bgr_mat.type());
  cv::resize(bgr_mat, mat_tmp, mat_tmp.size(), 0, 0);
  // cv::imwrite("resized_img.jpg", mat_tmp);
  auto ret = ImageUtils::BGRToNv12(mat_tmp, nv12_mat);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  // padding y
  for (int h = 0; h < scaled_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    for (int w = 0; w < scaled_img_width; ++w) {
      *raw++ = *data++;
    }
  }

  // padding uv
  auto uv_data = nv12_mat.data + scaled_img_height * scaled_img_width;
  for (int32_t h = 0; h < scaled_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    for (int32_t w = 0; w < scaled_img_width; ++w) {
      *raw++ = *uv_data++;
    }
  }

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pyramid = new NV12PyramidInput;
  pyramid->width = scaled_img_width;
  pyramid->height = scaled_img_height;
  pyramid->y_vir_addr = y->virAddr;
  pyramid->y_phy_addr = y->phyAddr;
  pyramid->y_stride = w_stride;
  pyramid->uv_vir_addr = uv->virAddr;
  pyramid->uv_phy_addr = uv->phyAddr;
  pyramid->uv_stride = w_stride;
  return std::shared_ptr<NV12PyramidInput>(
      pyramid, [y, uv](NV12PyramidInput *pyramid) {
        // Release memory after deletion
        hbSysFreeMem(y);
        hbSysFreeMem(uv);
        delete y;
        delete uv;
        delete pyramid;
      });
}

int32_t ImageUtils::BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

void ImageUtils::GetNV12Tensor(std::string &image_file,
                               std::shared_ptr<DNNTensor> &dnn_tensor) {
  hbDNNTensorProperties properties = dnn_tensor->properties;
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  if (bgr_mat.empty()) {
    std::cout << "image file not exist!" << std::endl;
    return;
  }

  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  auto *yuv = yuv_mat.ptr<uint8_t>();
  cv::Mat img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int uv_height = height / 2;
  int uv_width = width / 2;

  // copy y data
  int y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }

  uint8_t *data = img_nv12.data;
  auto stride = properties.alignedShape.dimensionSize[3];
  auto *y = reinterpret_cast<uint8_t *>(dnn_tensor->sysMem[0].virAddr);
  for (int h = 0; h < height; ++h) {
    auto *raw = y + h * stride;
    for (int w = 0; w < width; ++w) {
      *raw++ = *data++;
    }
  }
  // Copy uv data to data1
  auto *uv = reinterpret_cast<uint8_t *>(dnn_tensor->sysMem[1].virAddr);
  memcpy(uv, img_nv12.data + height * width, height * width / 2);
  hbSysFlushMem(&dnn_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(&dnn_tensor->sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
}

void ImageUtils::DrawParsing(const std::string &saving_path,
                             ParsingResult *parsing_result) {
  auto data = parsing_result->data;
  auto parsing_height = parsing_result->height;
  auto parsing_width = parsing_result->width;
  // set parsing bgr
  RendingBgrImg(saving_path,
                reinterpret_cast<int8_t *>(data.data()),
                parsing_height,
                parsing_width);
  std::cout << "Draw parsing result to file: " << saving_path << std::endl;
}

void ImageUtils::DrawDepth(const std::string &image_file,
                           const std::string &saving_path,
                           DepthResult *result) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);
  // TODO(ruxin.song): By macro control
  if (mat.empty()) {
    std::cout << "image file not exist!" << std::endl;
    return;
  }
  auto data = result->depth;
  auto height = result->height;
  auto width = result->width;
  auto max_ele = max_element(data.begin(), data.end());
  float scale = 255 / *max_ele;
  cv::Mat depth_mat(height, width, CV_8UC1);
  auto *depth_img_ptr = depth_mat.ptr<uint8_t>();
  // set value from depth
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      float depth = data[h * width + w];
      *depth_img_ptr++ = static_cast<int>(depth);
    }
  }

  // use scale to control the scope
  depth_mat.convertTo(depth_mat, CV_8UC1, scale, 0);
  // use color map in opencv
  cv::applyColorMap(depth_mat, depth_mat, cv::COLORMAP_JET);
  cv::resize(depth_mat, depth_mat, mat.size(), 0, 0, cv::INTER_NEAREST);

  float alpha_f = 0.4;
  cv::Mat dst;
  // merged images
  addWeighted(mat, alpha_f, depth_mat, 1 - alpha_f, 0.0, dst);
  std::cout << "Draw depth result to file: " << saving_path << std::endl;
  cv::imwrite(saving_path, dst);
}

void ImageUtils::RenderingFilter2d(const std::string &image_file,
                                   const std::string &saving_path,
                                   std::vector<PerceptionRect> &boxes,
                                   int scaled_height,
                                   int scaled_width) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);

  float height_scale = !scaled_height ? 1.0f : mat.rows * 1.0f / scaled_height;
  float width_scale = !scaled_width ? 1.0f : mat.cols * 1.0f / scaled_width;

  for (auto &rect : boxes) {
    auto &color = colors[rect.perception_type % 4];
    cv::rectangle(
        mat,
        cv::Point(rect.left * width_scale, rect.top * height_scale),
        cv::Point(rect.right * width_scale, rect.bottom * height_scale),
        color, 3);
  }
  std::cout << "Draw filter2d result to file: " << saving_path << std::endl;
  cv::imwrite(saving_path, mat);
}

void ImageUtils::RenderingLmk(const std::string &image_file,
                                   const std::string &saving_path,
                                   std::vector<Landmarks> &lmks,
                                   int scaled_height,
                                   int scaled_width) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);

  float height_scale = !scaled_height ? 1.0f : mat.rows * 1.0f / scaled_height;
  float width_scale = !scaled_width ? 1.0f : mat.cols * 1.0f / scaled_width;

  size_t lmk_num = lmks.size();
  for (size_t idx = 0; idx < lmk_num; idx++) {
    const auto& lmk = lmks.at(idx);
    auto &color = colors[idx % 4];
    for (const auto& point : lmk) {
      cv::circle(
          mat,
          cv::Point(point.x * width_scale, point.y * height_scale),
          3, color, 3);
    }
  }
  std::cout << "Draw result to file: " << saving_path << std::endl;
  cv::imwrite(saving_path, mat);
}

void ImageUtils::DrawResidual(const std::string &image_file,
                              const std::string &saving_path,
                              ParsingResult *parsing_result,
                              ResidualResult *result) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);
  // TODO(ruxin.song): By macro control
  if (mat.empty()) {
    std::cout << "image file not exist!" << std::endl;
    return;
  }
  auto ComputeColor = [](float fx, float fy) {
    const auto &colors = residual_color_map;
    float rad = std::sqrt(fx * fx + fy * fy);
    float a = std::atan2(-fy, -fx) / M_PI_F;
    float fk = (a + 1.0f) / 2.0f * static_cast<float>((colors.size() - 1));
    auto k0 = static_cast<size_t>(fk);
    size_t k1 = (k0 + 1) % colors.size();
    float f = fk - static_cast<float>(k0);

    auto GetPiexlValue = [f, rad](uint8_t pixel_k0, uint8_t pixel_k1) {
      float col0 = static_cast<float>(pixel_k0) / 255.0f;
      float col1 = static_cast<float>(pixel_k1) / 255.0f;
      float col = col0 + f * (col1 - col0);
      if (rad <= 1.0f) {
        col = 1.0f - rad * (1.0f - col);
      } else {
        col *= .75f;
      }
      return static_cast<uint8_t>(255.0f * col);
    };

    // Calculate RGB values
    cv::Vec3b pixel;
    pixel[2] = GetPiexlValue(colors[k0][0], colors[k1][0]);
    pixel[1] = GetPiexlValue(colors[k0][1], colors[k1][1]);
    pixel[0] = GetPiexlValue(colors[k0][2], colors[k1][2]);

    return pixel;
  };

  auto data = result->data;
  auto height = result->height;
  auto width = result->width;
  auto area_size = height * width;
  // set parsing bgr
  cv::Mat mat_x = cv::Mat(height, width, CV_32FC1, data[0].data());
  cv::Mat mat_z = cv::Mat(height, width, CV_32FC1, data[2].data());

  // resize residual to the same size as parsing
  if (parsing_result->width != width || parsing_result->height != height) {
    const auto scale_w =
        static_cast<float>(parsing_result->width) / static_cast<float>(width);
    const auto resflow_h = static_cast<float>(height) * scale_w;

    cv::resize(mat_x,
               mat_x,
               cv::Size(parsing_result->width, static_cast<int>(resflow_h)));
    cv::resize(mat_z,
               mat_z,
               cv::Size(parsing_result->width, static_cast<int>(resflow_h)));
    height = mat_x.rows;
    width = mat_x.cols;
    area_size = mat_x.cols * mat_x.rows;
  }

  float max_rad = -1;
  float *x_data = mat_x.ptr<float>();
  float *z_data = mat_z.ptr<float>();
  for (int index = 0; index < area_size; index++) {
    const auto x_value = x_data[index] * x_data[index];
    const auto z_value = z_data[index] * z_data[index];
    auto rad = std::sqrt(x_value + z_value);
    if (max_rad < rad) {
      max_rad = rad;
    }
  }

  auto *parsing_data = reinterpret_cast<int8_t *>(parsing_result->data.data());
  cv::Mat residual_mat = cv::Mat::zeros(height, width, CV_8UC3);
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      int parsing_index = h * parsing_result->width + w;
      const auto pixel_parsing = parsing_data[parsing_index];
      // type filter
      if (pixel_parsing < 5 || pixel_parsing > 7) {
        continue;
      }
      const auto x_value = x_data[h * width + w] / max_rad;
      const auto z_value = z_data[h * width + w] / max_rad;
      residual_mat.at<cv::Vec3b>(h, w) = ComputeColor(x_value, z_value);
    }
  }

  float alpha_f = 0.5;
  cv::Mat dst;
  // merged images
  addWeighted(mat, alpha_f, residual_mat, 1 - alpha_f, 0.0, dst);
  cv::imwrite(saving_path, dst);
  std::cout << "Draw parsing result to file: " << saving_path << std::endl;
}

void ImageUtils::DrawElevation(const std::string &image_file,
                               const std::string &saving_path,
                               cv::Mat &K,
                               ElevationOutputDescription *desc,
                               ElevationResult *result) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);
  // TODO(ruxin.song): By macro control
  if (mat.empty()) {
    std::cout << "image file not exist!" << std::endl;
    return;
  }

  // TODO(ruxin.song): need to use model parameters
  // const float scale_k =
  //     static_cast<float>(mat.cols) / static_cast<float>(result->width);
  // cv::Mat k_cam = K / scale_k;
  // *k_cam.ptr<float>(2, 2) = 1.0f;
  // cv::Mat k_cam_inv = k_cam.inv();
  // const auto fx_inv = k_cam_inv.at<float>(0, 0);
  // const auto fy_inv = k_cam_inv.at<float>(1, 1);
  // const auto cx_inv = k_cam_inv.at<float>(0, 2);
  // const auto cy_inv = k_cam_inv.at<float>(1, 2);

  const auto fx_inv = 0.00165905;
  const auto fy_inv = 0.00165905;
  const auto cx_inv = -0.78422433;
  const auto cy_inv = -0.45798885;
  const auto cam_h = 1.7096162;
  const auto height_max_value = 0.8f;

  const auto &gamma_data = result->elevation;
  const auto *gamma_data_ptr = gamma_data.data();

  cv::Mat height_mat = cv::Mat::zeros(result->height, result->width, CV_8UC1);
  std::array<cv::Point2f, 2> roi_ratio{
      {cv::Point2f(0.1f, 0.4f), cv::Point2f(0.9f, 0.9f)}};
  std::vector<float> normal_vector = {0.0, 1.0, 0.0};

  // exclude unsupervised areas
  const float start_rows_f =
      static_cast<float>(height_mat.rows) * roi_ratio.at(0).y;
  const float start_cols_f =
      static_cast<float>(height_mat.cols) * roi_ratio.at(0).x;
  const float end_rows_f =
      static_cast<float>(height_mat.rows) * roi_ratio.at(1).y;
  const float end_cols_f =
      static_cast<float>(height_mat.cols) * roi_ratio.at(1).x;

  // check the roi
  const int start_rows = std::max(0, static_cast<int>(start_rows_f));
  const int start_cols = std::max(0, static_cast<int>(start_cols_f));
  const int end_rows = std::min(height_mat.rows, static_cast<int>(end_rows_f));
  const int end_cols = std::min(height_mat.cols, static_cast<int>(end_cols_f));
  if (start_rows >= end_rows || start_cols >= end_cols) {
    std::cout << "Elevation ROI config is error!" << std::endl;
    cv::imwrite("elevation.jpeg", cv::Mat());
    return;
  }

  // parse the height
  for (int row = start_rows; row < end_rows; row++) {
    for (int col = start_cols; col < end_cols; col++) {
      gamma_data_ptr = gamma_data.data() + row * height_mat.cols + col;
      float x = col * fx_inv + cx_inv;
      float y = row * fy_inv + cy_inv;
      float val = *gamma_data_ptr - (normal_vector[0] * x +
                                     normal_vector[1] * y + normal_vector[2]);
      if (std::abs(val) < 0.000001f) {
        val = 1e-3f;
      }

      float height = *gamma_data_ptr * cam_h / val;
      if (height >= height_max_value) {
        *height_mat.ptr<uint8_t>(row, col) = static_cast<uint8_t>(255);
      } else if (height > 0.0f) {
        *height_mat.ptr<uint8_t>(row, col) =
            static_cast<uint8_t>(height / height_max_value * 255.0f);
      } else {
      }
    }
  }

  // color map
  cv::Rect rect_roi(
      start_cols, start_rows, end_cols - start_cols, end_rows - start_rows);

  cv::Mat height_color_mat = cv::Mat::zeros(height_mat.size(), CV_8UC3);
  cv::Mat gray_mat_roi = height_mat(rect_roi);
  cv::Mat color_mat_roi = height_color_mat(rect_roi);
  cv::applyColorMap(gray_mat_roi, color_mat_roi, cv::COLORMAP_JET);

  // mat resize
  int target_h = mat.rows;
  int target_w = mat.cols;
  if (height_color_mat.rows != target_h || height_color_mat.cols != target_w) {
    const auto scale_w = static_cast<float>(target_w) /
                         static_cast<float>(height_color_mat.cols);
    const auto h_new = static_cast<float>(height_color_mat.rows) * scale_w;
    cv::Mat height_color_mat_new = cv::Mat::zeros(target_h, target_w, CV_8UC3);
    cv::Mat roi_img =
        height_color_mat_new(cv::Rect(0, 0, target_w, static_cast<int>(h_new)));
    cv::resize(
        height_color_mat, roi_img, cv::Size(target_w, static_cast<int>(h_new)));
    height_color_mat = height_color_mat_new;
  }

  float alpha_f = 0.5;
  cv::Mat dst;
  // merged images
  addWeighted(mat, alpha_f, height_color_mat, 1 - alpha_f, 0.0, dst);
  cv::imwrite(saving_path, dst);
  std::cout << "Draw parsing result to file: " << saving_path << std::endl;
}

void ImageUtils::DrawBev3D(const std::string &saving_path,
                           Bev3DResult *result) {
  // configure the output image size
  int image_width = 2000;
  int image_height = 1000;
  cv::Mat background =
      cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));
  int center_x = image_width / 2;
  int center_y = image_height / 2;
  float scale = 10.0;
  int self_width = 1.8 * scale;
  int self_length = 4.4 * scale;

  // render of self car
  cv::rectangle(
      background,
      cv::Point(center_x - self_length / 2, center_y - self_width / 2),
      cv::Point(center_x + self_length / 2, center_y + self_width / 2),
      cv::Scalar(0, 0, 255),
      3,
      4);

  // render the vehicle
  for (int i = 0; i < result->bbox3d.size(); i++) {
    auto &box = result->bbox3d[i];
    for (int j = 0; j < box.corners3d.size() - 1; j++) {
      float x1 = box.corners3d[j][0] * scale + center_x;
      float y1 = center_y - box.corners3d[j][1] * scale;
      float x2 = box.corners3d[j + 1][0] * scale + center_x;
      float y2 = center_y - box.corners3d[j + 1][1] * scale;
      cv::line(background,
               cv::Point(x1, y1),
               cv::Point(x2, y2),
               cv::Scalar(255, 255, 255),
               3,
               4);
    }
  }

  // image rotation
  cv::rotate(background, background, cv::ROTATE_90_COUNTERCLOCKWISE);

  cv::imwrite(saving_path, background);
  std::cout << "Draw bev3d result to file: " << saving_path << std::endl;
}

void ImageUtils::RenderingClassification(
    const std::string &image_file,
    std::vector<hbDNNRoi> &boxes,
    const std::string &saving_path,
    std::vector<std::string> &class_names) {
  cv::Mat mat = cv::imread(image_file, cv::IMREAD_COLOR);
  if (boxes.size() != class_names.size()) {
    std::cerr
        << "The number of boxes must be the same as the number of class_names"
        << std::endl;
  }
  for (int i = 0; i < boxes.size(); i++) {
    auto &color = colors[i % 4];
    cv::rectangle(mat,
                  cv::Point(boxes[i].left, boxes[i].top),
                  cv::Point(boxes[i].right, boxes[i].bottom),
                  color);
    cv::putText(mat,
                class_names[i],
                cv::Point(boxes[i].left, std::abs(boxes[i].top - 5)),
                cv::FONT_HERSHEY_SIMPLEX,
                0.3,
                color,
                1,
                cv::LINE_8);
  }
  std::cout << "Draw classification result to file: " << saving_path
            << std::endl;
  cv::imwrite(saving_path, mat);
}

struct Box3DRaw {
  cv::Point2f lower_lt;
  cv::Point2f lower_lb;
  cv::Point2f lower_rb;
  cv::Point2f lower_rt;
  cv::Point2f upper_lt;
  cv::Point2f upper_lb;
  cv::Point2f upper_rb;
  cv::Point2f upper_rt;
};

static void distortSinglePoint(const cv::Point2f &pt_in,
                               cv::Point2f &pt_out,
                               const cv::Mat &mat_intrinsics,
                               const cv::Mat &mat_dist_coeff) {
  float *pt_dist_coeff = reinterpret_cast<float *>(mat_dist_coeff.data);
  float k1 = pt_dist_coeff[0];
  float k2 = pt_dist_coeff[1];
  float p1 = pt_dist_coeff[2];
  float p2 = pt_dist_coeff[3];
  float *pt_intrinsics = reinterpret_cast<float *>(mat_intrinsics.data);
  float fx = pt_intrinsics[0];
  float fy = pt_intrinsics[4];
  float cx = pt_intrinsics[2];
  float cy = pt_intrinsics[5];

  float xd = pt_in.x;
  float yd = pt_in.y;

  xd = (xd - cx) / fx;
  yd = (yd - cy) / fy;

  float xp = 0.0f;
  float yp = 0.0f;
  float r2, r4;  // , r6;

  r2 = xd * xd + yd * yd;
  r4 = r2 * r2;
  // r6 = r4*r2;
  if (mat_dist_coeff.rows == 4) {
    xp = (1.0f + k1 * r2 + k2 * r4) * xd + 2.0f * p1 * xd * yd +
         p2 * (r2 + 2.0f * xd * xd);
    yp = (1.0f + k1 * r2 + k2 * r4) * yd + p1 * (r2 + 2.0f * yd * yd) +
         2.0f * p2 * xd * yd;
  } else if (mat_dist_coeff.rows == 5) {
    float k3 = pt_dist_coeff[4];
    float r6 = r4 * r2;
    xp = (1.0f + k1 * r2 + k2 * r4 + k3 * r6) * xd + 2.0f * p1 * xd * yd +
         p2 * (r2 + 2.0f * xd * xd);
    yp = (1.0f + k1 * r2 + k2 * r4 + k3 * r6) * yd +
         p1 * (r2 + 2.0f * yd * yd) + 2.0f * p2 * xd * yd;
  } else if (mat_dist_coeff.rows == 8) {
    float k3 = pt_dist_coeff[4];
    float k4 = pt_dist_coeff[5];
    float k5 = pt_dist_coeff[6];
    float k6 = pt_dist_coeff[7];
    float r6 = r4 * r2;
    xp = (1.0f + k1 * r2 + k2 * r4 + k3 * r6) /
             (1.0f + k4 * r2 + k5 * r4 + k6 * r6) * xd +
         2.0f * p1 * xd * yd + p2 * (r2 + 2.0f * xd * xd);
    yp = (1.0f + k1 * r2 + k2 * r4 + k3 * r6) /
             (1.0f + k4 * r2 + k5 * r4 + k6 * r6) * yd +
         p1 * (r2 + 2.0f * yd * yd) + 2.0f * p2 * xd * yd;
  } else {
  }

  xp = xp * fx + cx;
  yp = yp * fy + cy;

  pt_out.x = xp;
  pt_out.y = yp;
}

// 8 corners
/*
          4----------5    6----------7
        /|         /|   /|         /|
        / |        / |  / |        / |
      /  |       /  | /  |       /  |
      7---|------6   |5---|------4   |
      |   |      |   ||   |      |   |
      |   |      |   ||   |      |   |
      |   0------|---1|   2------|---3
      |  /       |  / |  /       |  /
      | /     ^  | /  | /     v  | /
      |/         |/   |/         |/
      3----------2    1----------0
  */
void DistortBoxPoints(std::vector<Box3DRaw> &box3d_raws,
                      const std::vector<Real3dResult::Bbox3D> &bbox3ds,
                      cv::Mat K,
                      cv::Mat dist_coeff) {
  auto bbox3d_size = bbox3ds.size();
  box3d_raws.resize(bbox3d_size);
  for (size_t box_index = 0U; box_index < bbox3d_size; ++box_index) {
    auto &bbox3d = bbox3ds[box_index];
    cv::Point2f distort_pts;
    auto &box3d_raw = box3d_raws[box_index];
    if ((-M_PI / 2) <= bbox3d.r || bbox3d.r <= (M_PI / 2)) {
      // distort
      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[0][0], bbox3d.corners2d[0][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_lt.x = distort_pts.x;
      box3d_raw.lower_lt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[3][0], bbox3d.corners2d[3][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_lb.x = distort_pts.x;
      box3d_raw.lower_lb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[2][0], bbox3d.corners2d[2][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_rb.x = distort_pts.x;
      box3d_raw.lower_rb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[1][0], bbox3d.corners2d[1][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_rt.x = distort_pts.x;
      box3d_raw.lower_rt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[4][0], bbox3d.corners2d[4][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_lt.x = distort_pts.x;
      box3d_raw.upper_lt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[7][0], bbox3d.corners2d[7][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_lb.x = distort_pts.x;
      box3d_raw.upper_lb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[6][0], bbox3d.corners2d[6][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_rb.x = distort_pts.x;
      box3d_raw.upper_rb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[5][0], bbox3d.corners2d[5][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_rt.x = distort_pts.x;
      box3d_raw.upper_rt.y = distort_pts.y;
    } else {
      // distort
      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[2][0], bbox3d.corners2d[2][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_lt.x = distort_pts.x;
      box3d_raw.lower_lt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[1][0], bbox3d.corners2d[1][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_lb.x = distort_pts.x;
      box3d_raw.lower_lb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[0][0], bbox3d.corners2d[0][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_rb.x = distort_pts.x;
      box3d_raw.lower_rb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[3][0], bbox3d.corners2d[3][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.lower_rt.x = distort_pts.x;
      box3d_raw.lower_rt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[6][0], bbox3d.corners2d[6][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_lt.x = distort_pts.x;
      box3d_raw.upper_lt.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[5][0], bbox3d.corners2d[5][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_lb.x = distort_pts.x;
      box3d_raw.upper_lb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[4][0], bbox3d.corners2d[4][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_rb.x = distort_pts.x;
      box3d_raw.upper_rb.y = distort_pts.y;

      distortSinglePoint(
          cv::Point2f(bbox3d.corners2d[7][0], bbox3d.corners2d[7][1]),
          distort_pts,
          K,
          dist_coeff);
      box3d_raw.upper_rt.x = distort_pts.x;
      box3d_raw.upper_rt.y = distort_pts.y;
    }
    // std::cout << "corners (distort):"
    //   << " " << box3d_raw.lower_lt.x
    //   << " " << box3d_raw.lower_lt.y
    //   << " " << box3d_raw.lower_lb.x
    //   << " " << box3d_raw.lower_lb.y
    //   << " " << box3d_raw.lower_rb.x
    //   << " " << box3d_raw.lower_rb.y
    //   << " " << box3d_raw.lower_rt.x
    //   << " " << box3d_raw.lower_rt.y
    //   << " " << box3d_raw.upper_lt.x
    //   << " " << box3d_raw.upper_lt.y
    //   << " " << box3d_raw.upper_lb.x
    //   << " " << box3d_raw.upper_lb.y
    //   << " " << box3d_raw.upper_rb.x
    //   << " " << box3d_raw.upper_rb.y
    //   << " " << box3d_raw.upper_rt.x
    //   << " " << box3d_raw.upper_rt.y << std::endl;
  }
}

void ImageUtils::RenderingReal3d(const std::string &image_file,
                                 const std::string &save_file,
                                 const Real3dResult *real3d_result,
                                 cv::Mat K,
                                 cv::Mat dist_coeff,
                                 float scale) {
  cv::Mat src = cv::imread(image_file, cv::IMREAD_COLOR);
  cv::Mat img;
  cv::resize(src, img, cv::Size(src.cols * scale, src.rows * scale));

  // 由于图像可能带有畸变，所以渲染的时候需要对坐标加畸变，这里得到的2D坐标是原图对应的坐标
  std::vector<Box3DRaw> box3d_raw;
  DistortBoxPoints(box3d_raw, real3d_result->bbox3d, K, dist_coeff);

  const cv::Scalar front_color = cv::Scalar(236, 157, 51);
  const cv::Scalar lower_color = cv::Scalar(234, 2, 86);
  const cv::Scalar rear_color = cv::Scalar(236, 75, 167);
  const cv::Scalar upper_color = cv::Scalar(255, 18, 252);
  const int thickness = 1;

  auto DrawLines = [&](const std::vector<cv::Point2f> &points,
                       const cv::Scalar &color) {
    if (points.size() <= 1) {
      return;
    }
    auto pre = points[0];
    for (size_t i = 1U; i < points.size(); ++i) {
      cv::line(img, pre, points[i], color, thickness);
      pre = points[i];
    }
  };

  for (auto &box_raw : box3d_raw) {
    // 先分别连线3dbox的上下两个面
    DrawLines({box_raw.upper_lt,
               box_raw.upper_rt,
               box_raw.upper_rb,
               box_raw.upper_lb,
               box_raw.upper_lt},
              upper_color);

    DrawLines({box_raw.lower_lt,
               box_raw.lower_rt,
               box_raw.lower_rb,
               box_raw.lower_lb,
               box_raw.lower_lt},
              lower_color);

    // 再连线两个面的四个顶点之间的四条线
    DrawLines({box_raw.upper_lt, box_raw.lower_lt}, front_color);
    DrawLines({box_raw.upper_rt, box_raw.lower_rt}, front_color);
    DrawLines({box_raw.upper_lb, box_raw.lower_lb}, rear_color);
    DrawLines({box_raw.upper_rb, box_raw.lower_rb}, rear_color);
  }

  cv::imwrite(save_file, img);
}

void ImageUtils::RendingBgrImg(const std::string &saving_path,
                               int8_t *data,
                               int height,
                               int width) {
  cv::Mat mat(height, width, CV_8UC3);
  auto *img_ptr = mat.ptr<uint8_t>();
  auto w_base = 1;
  auto h_base = 1;
  // set parsing bgr
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      int src_h = i * h_base;
      int src_w = j * w_base;
      int8_t id = data[src_h * width + src_w];
      if (id >= 19) continue;
      *img_ptr++ = bgr_putpalette[id * 3];
      *img_ptr++ = bgr_putpalette[id * 3 + 1];
      *img_ptr++ = bgr_putpalette[id * 3 + 2];
    }
  }
  cv::imwrite(saving_path, mat);
}
