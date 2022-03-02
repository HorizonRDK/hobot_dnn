// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_STATUS_H_
#define _EASY_DNN_STATUS_H_

namespace hobot {
namespace easy_dnn {

typedef enum {
  DNN_SUCCESS = 0,
  DNN_INVALID_ARGUMENT = -6000001,
  DNN_INVALID_MODEL = -6000002,
  DNN_MODEL_NUMBER_EXCEED_LIMIT = -6000003,
  DNN_INVALID_PACKED_DNN_HANDLE = -6000004,
  DNN_INVALID_DNN_HANDLE = -6000005,
  DNN_CAN_NOT_OPEN_FILE = -6000006,
  DNN_OUT_OF_MEMORY = -6000007,
  DNN_TIMEOUT = -6000008,
  DNN_TASK_NUM_EXCEED_LIMIT = -6000009,
  DNN_TASK_BATCH_SIZE_EXCEED_LIMIT = -6000010,
  DNN_INVALID_TASK_HANDLE = -6000011,
  DNN_RUN_TASK_FAILED = -6000012,
  DNN_MODEL_IS_RUNNING = -6000013,
  DNN_INCOMPATIBLE_MODEL = -6000014,
  DNN_API_USE_ERROR = -6000015,
  // error codes above are same with libdnn [-6000001, -6000255]
  DNN_PROCESS_INPUT_FAILED = -6000256,
  DNN_PARSE_OUTPUT_FAILED = -6000257,
} DNNStatus;

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_STATUS_H_
