// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_INPUT_PROCESSOR_H_
#define _EASY_DNN_INPUT_PROCESSOR_H_

#include <memory>
namespace hobot {
namespace easy_dnn {

class DNNInput;
class DNNTensor;

class InputProcessor {
 public:
  /**
   * InputProcessor
   * @param[out] tensor
   * @param[in] input_desc
   * @param[in]  input
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t Process(std::shared_ptr<DNNTensor>& tensor,
                          std::shared_ptr<InputDescription>& input_desc,
                          std::shared_ptr<DNNInput>& input) = 0;
  virtual ~InputProcessor() = default;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_INPUT_PROCESSOR_H_
