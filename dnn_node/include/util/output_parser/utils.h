// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _UTIL_UTILS_H_
#define _UTIL_UTILS_H_

#include <string>
#include <vector>
#include <memory>

#include "easy_dnn/data_structure.h"

using hobot::easy_dnn::DNNTensor;
/**
 *
 * @param[in] tensor
 * @param[out] h_index
 * @param[out] w_index
 * @param[out] c_index
 * @return 0f if success
 */
int get_tensor_hw(DNNTensor &tensor, int *height, int *width);

#endif  // _UTIL_UTILS_H_
