// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _EASY_DNN_MODEL_IMPL_H_
#define _EASY_DNN_MODEL_IMPL_H_
#include <memory>
#include <string>

#include "dnn/hb_dnn.h"

namespace hobot {
namespace easy_dnn {

class InputDescription;
class InputProcessor;
class OutputDescription;
class OutputParser;

class Model {
 public:
  /**
   * Get model handle
   * @return model handle
   */
  virtual hbDNNHandle_t GetDNNHandle() = 0;

  /**
   * Get model name
   * @return model name
   */
  virtual const std::string& GetName() = 0;

  /**
   * Get input count
   * @return input count
   */
  virtual int32_t GetInputCount() = 0;

  /**
   * Get input name
   * @param[out] input_name
   * @param[in] input_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputName(std::string& input_name,
                               int32_t input_index) = 0;

  /**
   * Get input source
   * @param[out] input_source
   * @param[in] input_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputSource(int32_t& input_source,
                                 int32_t input_index) = 0;

  /**
   * Get input description
   * @param[out] input_desc
   * @param[in] input_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputDescription(std::string& input_desc,
                                      int32_t input_index) = 0;

  /**
   * Get input description
   * @param[out] input_desc
   * @param[in] input_input
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputDescription(
      std::shared_ptr<InputDescription>& input_desc, int32_t input_index) = 0;

  /**
   * Update input description
   * @param[in] input_desc
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetInputDescription(
      std::shared_ptr<InputDescription>& input_desc) = 0;

  /**
   * Get input processor
   * @param[out] input_processor
   * @param[in] input_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputProcessor(
      std::shared_ptr<InputProcessor>& input_processor,
      int32_t input_index) = 0;

  /**
   * Update input processor
   * @param[in] input_index
   * @param[in] input_processor
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetInputProcessor(
      int32_t input_index,
      std::shared_ptr<InputProcessor>& input_processor) = 0;

  /**
   * Get input tensor properties
   * @param[out] tensor_properties
   * @param[in] input_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputTensorProperties(
      hbDNNTensorProperties& tensor_properties, int32_t input_index) = 0;

  /**
   * Get input stage count
   * @return input stage count
   */
  virtual int32_t GetInputStageCount() = 0;

  /**
   * Get input stage required roi
   * @param[out] roi
   * @param[in] input_index
   * @param[in] stage_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetInputStageRequiredRoi(hbDNNRoi& roi,
                                           int32_t input_index,
                                           int32_t stage_index) = 0;

  /**
   * Get output count
   * @return output count
   */
  virtual int32_t GetOutputCount() = 0;

  /**
   * Get output name
   * @param[out] output_name
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputName(std::string& output_name,
                                int32_t output_index) = 0;

  /**
   * Get output description
   * @param[out] output_desc
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputDescription(std::string& output_desc,
                                       int32_t output_index) = 0;

  /**
   * Get output description
   * @param[out] output_desc
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputDescription(
      std::shared_ptr<OutputDescription>& output_desc,
      int32_t output_index) = 0;

  /**
   * Update output description
   * @param[in] output_desc
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetOutputDescription(
      std::shared_ptr<OutputDescription>& output_desc) = 0;

  /**
   * Get output parser
   * @param[out] output_parser
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputParser(std::shared_ptr<OutputParser>& output_parser,
                                  int32_t output_index) = 0;

  /**
   * Get output parser
   * @param[in] output_index
   * @param[in] output_parser
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t SetOutputParser(
      int32_t output_index, std::shared_ptr<OutputParser>& output_parser) = 0;

  /**
   * Get output tensor properties
   * @param[out] tensor_properties
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputTensorProperties(
      hbDNNTensorProperties& tensor_properties, int32_t output_index) = 0;

  /**
   * Get output operator type
   * @param[out] type
   * @param[in] output_index
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetOutputOperatorType(int& type, int32_t output_index) = 0;

  /**
   * Get model estimate execute latency, it's real-time calculated based
   *  on historical statistics
   * @return 0 if success, return defined error code otherwise
   */
  virtual int32_t GetEstimateLatency() = 0;

  virtual ~Model() = default;
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // _EASY_DNN_MODEL_IMPL_H_
