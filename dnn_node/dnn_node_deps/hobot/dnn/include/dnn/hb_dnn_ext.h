// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_HB_DNN_EXT_H_

#include "hb_dnn.h"
#include "hb_sys.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/**
 * Extra layout, supplement to hbDNNTensorLayout
 */
typedef enum {
  HB_DNN_LAYOUT_NHCW_NATIVE = 1,
  // TODO(@horizon.ai): complete layout, see hbrt_layout_type_t
} hbDNNExtraTensorLayout;

typedef enum {
  HB_DNN_INPUT_FROM_DDR = 0,
  HB_DNN_INPUT_FROM_RESIZER,
  HB_DNN_INPUT_FROM_PYRAMID,
} hbDNNInputSource;

typedef enum {
  HB_DNN_OUTPUT_OPERATOR_TYPE_UNKNOWN = 0,
  HB_DNN_OUTPUT_OPERATOR_TYPE_CONV = 1,
  HB_DNN_OUTPUT_OPERATOR_TYPE_DETECTION_POST_PROCESS = 2,
  HB_DNN_OUTPUT_OPERATOR_TYPE_RCNN_POST_PROCESS = 3,
  HB_DNN_OUTPUT_OPERATOR_TYPE_DETECTION_POST_PROCESS_STABLE_SORT = 4,
  HB_DNN_OUTPUT_OPERATOR_TYPE_CHANNEL_ARGMAX = 5,
  HB_DNN_OUTPUT_OPERATOR_TYPE_AUX_DPP_STABLE_SORT = 6,
  HB_DNN_OUTPUT_OPERATOR_TYPE_CHANNEL_ARGMAX_SPLIT = 7,
  HB_DNN_OUTPUT_OPERATOR_TYPE_FILTER = 8,
} hbDNNOutputOperatorType;

/**
 * Get model input source
 * @param[out] inputSource
 * @param[in] dnnHandle
 * @param[in] inputIndex
 * @return  0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputSource(int32_t *inputSource, hbDNNHandle_t dnnHandle,
                            int32_t inputIndex);

/**
 * Get model input description
 * @param[out] desc
 * @param[in] dnnHandle
 * @param[in] inputIndex
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputDesc(const char **desc, hbDNNHandle_t dnnHandle,
                          int32_t inputIndex);

/**
 * Get model output description
 * @param[out] desc
 * @param[in] dnnHandle
 * @param[in] outputIndex
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetOutputDesc(const char **desc, hbDNNHandle_t dnnHandle,
                           int32_t outputIndex);

/**
 * Get model output operator type
 * @param[out] operatorType
 * @param[in] dnnHandle
 * @param[in] outputIndex
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetOutputOperatorType(int32_t *operatorType,
                                   hbDNNHandle_t dnnHandle,
                                   int32_t outputIndex);

/**
 * Get model estimate execute latency, it's real-time calculated based
 *  on historical statistics
 * @param[out] estimateLatency
 * @param[in] dnnHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetEstimateLatency(int32_t *estimateLatency,
                                hbDNNHandle_t dnnHandle);

/**
 * Get estimate time for task
 * @param[out] estimate_time:
 * @param[in] taskHandle: pointer to the task
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetTaskEstimateTime(int32_t *estimate_time,
                                 hbDNNTaskHandle_t taskHandle);

#ifdef __cplusplus
}
#endif  // __cplusplus

#define DNN_HB_DNN_EXT_H_

#endif  // DNN_HB_DNN_EXT_H_
