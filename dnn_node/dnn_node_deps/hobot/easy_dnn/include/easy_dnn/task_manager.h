// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_TASK_MANAGER_H_
#define _EASY_DNN_TASK_MANAGER_H_

#include <memory>

#include "easy_dnn/task.h"

namespace hobot {
namespace easy_dnn {

class TaskManager {
 public:
  static TaskManager* GetInstance();

  /**
   * Set max model infer task count allowed, you can also set by environment
   *    `HB_MAX_MODEL_INFER_TASK_COUNT_ALLOWED`
   * @param[in] count
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetMaxModelInferTaskCountAllowed(int32_t count) = 0;

  /**
   * Set max model roi infer task count allowed, you can also set by environment
   *    `HB_MAX_MODEL_ROI_INFER_TASK_COUNT_ALLOWED`
   * @param[in] count
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetMaxModelRoiInferTaskCountAllowed(int32_t count) = 0;

  /**
   * Set max multi model task count allowed, you can also set by environment
   *    `HB_MAX_MULTI_MODEL_INFER_TASK_COUNT_ALLOWED`
   * @param[in] count
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetMaxMultiModelTaskCountAllowed(int32_t count) = 0;

  /**
   * Get model infer task
   * @param[in] timeout, wait at most `timeout` milliseconds if > 0, otherwise
   *    wait util a task is available
   * @return nullptr if timeout
   */
  virtual std::shared_ptr<ModelInferTask> GetModelInferTask(
      int32_t timeout) = 0;

  /**
   * Get model roi infer task
   * @param[in] timeout, wait at most `timeout` milliseconds if > 0, otherwise
   *    wait util a task is available
   * @return nullptr if timeout
   */
  virtual std::shared_ptr<ModelRoiInferTask> GetModelRoiInferTask(
      int32_t timeout) = 0;

  /**
   * Get multi model task
   * @param[in] timeout, wait at most `timeout` milliseconds if > 0, otherwise
   *    wait util a task is available
   * @return nullptr if timeout
   */
  virtual std::shared_ptr<MultiModelTask> GetMultiModelTask(
      int32_t timeout) = 0;

  virtual ~TaskManager() = default;
};
}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_TASK_MANAGER_H_
