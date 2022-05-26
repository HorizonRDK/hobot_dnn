// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _OUTPUT_PARSER_UTILS_H_
#define _OUTPUT_PARSER_UTILS_H_

#include <string>
#include <vector>
#include <memory>

#include "dnn_node/dnn_node_data.h"

using hobot::dnn_node::DNNTensor;

/**
 *
 * @param[in] tensor
 * @param[out] h_index
 * @param[out] w_index
 * @param[out] c_index
 * @return 0f if success
 */
int get_tensor_hwc_index(std::shared_ptr<DNNTensor> tensor,
                         int *h_index,
                         int *w_index,
                         int *c_index);

/**
 *
 * @param[in] tensor
 * @param[out] h_index
 * @param[out] w_index
 * @param[out] c_index
 * @return 0f if success
 */
int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width);


/**
 *
 * @param tensor
 * @param height
 * @param width
 * @return
 */
int get_tensor_aligned_hw(std::shared_ptr<DNNTensor> tensor,
                          int *height, int *width);

#endif  // _OUTPUT_PARSER_UTILS_H_
