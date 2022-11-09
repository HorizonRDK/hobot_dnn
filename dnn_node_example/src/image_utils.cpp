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

#include "include/image_utils.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_sys.h"

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const std::string &image_file,
    ImageType image_type,
    int scaled_img_height,
    int scaled_img_width) {
  if (ImageType::BGR == image_type) {
    int original_img_height = 0, original_img_width = 0;
    return GetNV12Pyramid(image_file,
                          scaled_img_height,
                          scaled_img_width,
                          original_img_height,
                          original_img_width);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ImageUtils"), "Only BGR is supported!!!");
    rclcpp::shutdown();
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
  original_img_width = bgr_mat.cols;
  original_img_height = bgr_mat.rows;

  auto w_stride = ALIGN_16(scaled_img_width);
  cv::Mat pad_frame;
  if (static_cast<uint32_t>(original_img_width) != w_stride ||
      original_img_height != scaled_img_height) {
    pad_frame =
        cv::Mat(scaled_img_height, w_stride, CV_8UC3, cv::Scalar::all(0));
    if (static_cast<uint32_t>(original_img_width) > w_stride ||
        original_img_height > scaled_img_height) {
      float ratio_w =
          static_cast<float>(original_img_width) / static_cast<float>(w_stride);
      float ratio_h = static_cast<float>(original_img_height) /
                      static_cast<float>(scaled_img_height);
      float dst_ratio = std::max(ratio_w, ratio_h);
      uint32_t resized_width =
          static_cast<float>(original_img_width) / dst_ratio;
      uint32_t resized_height =
          static_cast<float>(original_img_height) / dst_ratio;
      cv::resize(bgr_mat, bgr_mat, cv::Size(resized_width, resized_height));
      std::cout << ratio_w << " " << ratio_h << "\n";
    }

    // 复制到目标图像中间
    bgr_mat.copyTo(pad_frame(cv::Rect((w_stride - bgr_mat.cols) / 2,
                                      (scaled_img_height - bgr_mat.rows) / 2,
                                      bgr_mat.cols,
                                      bgr_mat.rows)));
  } else {
    pad_frame = bgr_mat;
  }
  // cv::imwrite("resized_img.jpg", pad_frame);
  auto ret = ImageUtils::BGRToNv12(pad_frame, nv12_mat);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }
  original_img_height = bgr_mat.rows;
  original_img_width = bgr_mat.cols;

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = nv12_mat.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);

  // padding y
  for (int h = 0; h < scaled_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    for (uint32_t w = 0; w < w_stride; ++w) {
      *raw++ = *data++;
    }
  }

  // padding uv
  auto uv_data = nv12_mat.data + scaled_img_height * w_stride;
  for (int32_t h = 0; h < scaled_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    for (uint32_t w = 0; w < w_stride; ++w) {
      *raw++ = *uv_data++;
    }
  }

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pyramid = new NV12PyramidInput;
  pyramid->width = w_stride;
  pyramid->height = scaled_img_height;
  pyramid->y_vir_addr = y->virAddr;
  pyramid->y_phy_addr = y->phyAddr;
  pyramid->y_stride = w_stride;
  pyramid->uv_vir_addr = uv->virAddr;
  pyramid->uv_phy_addr = uv->phyAddr;
  pyramid->uv_stride = w_stride;
  return std::shared_ptr<NV12PyramidInput>(pyramid,
                                           [y, uv](NV12PyramidInput *pyramid) {
                                             // Release memory after deletion
                                             hbSysFreeMem(y);
                                             hbSysFreeMem(uv);
                                             delete y;
                                             delete uv;
                                             delete pyramid;
                                           });
}

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const cv::Mat &bgr_mat, int scaled_img_height, int scaled_img_width) {
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
  return std::shared_ptr<NV12PyramidInput>(pyramid,
                                           [y, uv](NV12PyramidInput *pyramid) {
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

int ImageUtils::Render(
    const std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> &pyramid,
    const ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg) {
  if (!pyramid || !ai_msg) return -1;

  char *y_img = reinterpret_cast<char *>(pyramid->y_vir_addr);
  char *uv_img = reinterpret_cast<char *>(pyramid->uv_vir_addr);
  auto height = pyramid->height;
  auto width = pyramid->y_stride;
  auto img_y_size = height * width;
  auto img_uv_size = img_y_size / 2;
  char *buf = new char[img_y_size + img_uv_size];
  memcpy(buf, y_img, img_y_size);
  memcpy(buf + img_y_size, uv_img, img_uv_size);
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
  cv::Mat mat;
  cv::cvtColor(nv12, mat, CV_YUV2BGR_NV12);
  delete[] buf;

  RCLCPP_INFO(rclcpp::get_logger("ImageUtils"),
              "target size: %d",
              ai_msg->targets.size());
  bool hasRois = false;
  for (size_t idx = 0; idx < ai_msg->targets.size(); idx++) {
    const auto &target = ai_msg->targets.at(idx);
    if (target.rois.empty()) {
      continue;
    }
    hasRois = true;
    RCLCPP_INFO(rclcpp::get_logger("ImageUtils"),
                "target type: %s, rois.size: %d",
                target.type.c_str(),
                target.rois.size());
    auto &color = colors[idx % colors.size()];
    for (const auto &roi : target.rois) {
      RCLCPP_INFO(
          rclcpp::get_logger("ImageUtils"),
          "roi.type: %s, x_offset: %d y_offset: %d width: %d height: %d",
          roi.type.c_str(),
          roi.rect.x_offset,
          roi.rect.y_offset,
          roi.rect.width,
          roi.rect.height);
      cv::rectangle(mat,
                    cv::Point(roi.rect.x_offset, roi.rect.y_offset),
                    cv::Point(roi.rect.x_offset + roi.rect.width,
                              roi.rect.y_offset + roi.rect.height),
                    color,
                    3);
      std::string roi_type = target.type;
      if (!roi.type.empty()) {
        roi_type = roi.type;
      }
      if (!roi_type.empty()) {
        cv::putText(mat,
                    roi_type,
                    cv::Point2f(roi.rect.x_offset, roi.rect.y_offset - 10),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1.5);
      }
    }

    for (const auto &lmk : target.points) {
      for (const auto &pt : lmk.point) {
        cv::circle(mat, cv::Point(pt.x, pt.y), 3, color, 3);
      }
    }
  }

  if (!hasRois) {
    RCLCPP_WARN(rclcpp::get_logger("ImageUtils"),
                "Frame has no roi, skip the rendering");
    return 0;
  }
  std::string saving_path = "render_" + ai_msg->header.frame_id + "_" +
                            std::to_string(ai_msg->header.stamp.sec) + "_" +
                            std::to_string(ai_msg->header.stamp.nanosec) +
                            ".jpeg";
  RCLCPP_WARN(rclcpp::get_logger("ImageUtils"),
              "Draw result to file: %s",
              saving_path.c_str());
  cv::imwrite(saving_path, mat);
  return 0;
}
