// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _INPUT_INPUT_DATA_H_
#define _INPUT_INPUT_DATA_H_

#include <ostream>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"

using hobot::dnn_node::NV12PyramidInput;
typedef std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> NV12PyramidInputPtr;


#endif  // _INPUT_INPUT_DATA_H_
