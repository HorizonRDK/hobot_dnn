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

#include "util/image_proc.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_sys.h"

namespace hobot {
namespace dnn_node {

std::shared_ptr<NV12PyramidInput> ImageProc::GetNV12PyramidFromNV12Img(
    const char *in_img_data,
    const int &in_img_height,
    const int &in_img_width,
    const int &scaled_img_height,
    const int &scaled_img_width) {
  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;
  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);
  //内存初始化
  memset(y->virAddr, 0, scaled_img_height * w_stride);
  memset(uv->virAddr, 0, scaled_img_height / 2 * w_stride);

  const uint8_t *data = reinterpret_cast<const uint8_t *>(in_img_data);
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);
  int copy_w = std::min(in_img_width, scaled_img_width);
  int copy_h = std::min(in_img_height, scaled_img_height);

  // padding y
  for (int h = 0; h < copy_h; ++h) {
    auto *raw = hb_y_addr + h * w_stride;
    auto *src = data + h * in_img_width;
    memcpy(raw, src, copy_w);
  }

  // padding uv
  auto uv_data = in_img_data + in_img_height * in_img_width;
  for (int32_t h = 0; h < copy_h / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride;
    auto *src = uv_data + h * in_img_width;
    memcpy(raw, src, copy_w);
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

std::shared_ptr<NV12PyramidInput> ImageProc::GetNV12PyramidFromNV12Img(
    const char *in_img_data,
    const int &in_img_height,
    const int &in_img_width,
    const int &scaled_img_height,
    const int &scaled_img_width,
    int &padding_l,
    int &padding_t,
    int &padding_r,
    int &padding_b) {
  // 1 要求输入图片分辨率小于模型输入分辨率
  if (in_img_width > scaled_img_width && in_img_height > scaled_img_height) {
    return nullptr;
  }

  // 2 计算padding参数
  auto w_stride = ALIGN_16(scaled_img_width);
  if (w_stride > in_img_width) {
    // 需要在左边padding空相素
    padding_l = (w_stride - in_img_width) / 2;
    // 取偶数
    padding_l = padding_l % 2 == 0 ? padding_l : padding_l + 1;
  } else {
    padding_l = 0;
  }
  if (scaled_img_height > in_img_height) {
    // 需要在上方padding空相素
    padding_t = (scaled_img_height - in_img_height) / 2;
    padding_t = padding_t % 2 == 0 ? padding_t : padding_t + 1;
  } else {
    padding_t = 0;
  }
  padding_r = scaled_img_width - in_img_width - padding_l;
  padding_b = scaled_img_height - in_img_height - padding_t;

  // 3 申请内存并初始化
  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);
  memset(y->virAddr, 0, scaled_img_height * w_stride);
  memset(uv->virAddr, 0, scaled_img_height / 2 * w_stride);

  // 4 拷贝数据并padding
  const uint8_t *data = reinterpret_cast<const uint8_t *>(in_img_data);
  auto *hb_y_addr =
      reinterpret_cast<uint8_t *>(y->virAddr) + padding_t * w_stride;
  auto *hb_uv_addr =
      reinterpret_cast<uint8_t *>(uv->virAddr) + (padding_t / 2) * w_stride;
  // padding y
  for (uint32_t h = 0; h < in_img_height; ++h) {
    auto *raw = hb_y_addr + h * w_stride + padding_l;
    auto *src = data + h * in_img_width;
    memcpy(raw, src, in_img_width);
  }
  // padding uv
  auto uv_data = in_img_data + in_img_height * in_img_width;
  for (uint32_t h = 0; h < in_img_height / 2; ++h) {
    auto *raw = hb_uv_addr + h * w_stride + padding_l;
    auto *src = uv_data + h * in_img_width;
    memcpy(raw, src, in_img_width);
  }

  // 5 生成pym数据
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

}  // namespace dnn_node
}  // namespace hobot
