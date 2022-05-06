// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "util/output_parser/utils.h"

int get_tensor_hw(DNNTensor &tensor, int *height, int *width)
{
  int h_index = 0;
  int w_index = 0;
  if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NHWC)
  {
    h_index = 1;
    w_index = 2;
  } else if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    h_index = 2;
    w_index = 3;
  } else {
    return -1;
  }
  *height = tensor.properties.validShape.dimensionSize[h_index];
  *width = tensor.properties.validShape.dimensionSize[w_index];
  return 0;
}
