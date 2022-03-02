// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_DATA_STRUCTURE_H_
#define _EASY_DNN_DATA_STRUCTURE_H_

#include "dnn/hb_dnn.h"

namespace hobot {
namespace easy_dnn {

class DNNInput {
 public:
  virtual void Reset() {}
  virtual ~DNNInput() = default;
};

class NV12PyramidInput : public DNNInput {
 public:
  uint64_t y_phy_addr;
  void *y_vir_addr;
  uint64_t uv_phy_addr;
  void *uv_vir_addr;
  int32_t height;
  int32_t width;
  int32_t y_stride;
  int32_t uv_stride;
  void Reset() override {}
};

class DNNResult {
 public:
  virtual void Reset() {}
  virtual ~DNNResult() = default;
};

class DNNTensor : public hbDNNTensor {
 public:
  DNNTensor() : hbDNNTensor() {}
  virtual void Reset() {}
  virtual ~DNNTensor() = default;
};

class DNNInferCtrlParam : public hbDNNInferCtrlParam {
 public:
  DNNInferCtrlParam() : hbDNNInferCtrlParam() {
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(this);
  }
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_DATA_STRUCTURE_H_
