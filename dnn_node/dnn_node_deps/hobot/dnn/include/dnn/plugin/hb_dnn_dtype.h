// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_PLUGIN_HB_DNN_DTYPE_H_
#define DNN_PLUGIN_HB_DNN_DTYPE_H_

#include <cstdint>
#include <cstdlib>

#define HB_DNN_SIZEOF_TYPE(type) (hobot::dnn::TypeSize[type])

namespace hobot {
namespace dnn {

enum TypeFlag {
  kBool = 0,
  kUInt8 = 1,
  kInt8 = 2,
  kUInt16 = 3,
  kInt16 = 4,
  kUInt32 = 5,
  kInt32 = 6,
  kUInt64 = 7,
  kInt64 = 8,
  kFloat16 = 9,
  kFloat32 = 10,
  kFloat64 = 11,
  kUnused
};  // enum TypeFlag

extern size_t TypeSize[static_cast<int32_t>(TypeFlag::kUnused) + 1];

template <typename DType>
struct DataType {
  static inline const int kFlag() { return kUnused; }
};

template <>
struct DataType<bool> {
  static inline const int kFlag() { return kBool; }
};

template <>
struct DataType<uint8_t> {
  static inline const int kFlag() { return kUInt8; }
};

template <>
struct DataType<int8_t> {
  static inline const int kFlag() { return kInt8; }
};

template <>
struct DataType<uint16_t> {
  static inline const int kFlag() { return kUInt16; }
};

template <>
struct DataType<int16_t> {
  static inline const int kFlag() { return kInt16; }
};

template <>
struct DataType<uint32_t> {
  static inline const int kFlag() { return kUInt32; }
};

template <>
struct DataType<int32_t> {
  static inline const int kFlag() { return kInt32; }
};

template <>
struct DataType<int64_t> {
  static inline const int kFlag() { return kInt64; }
};

template <>
struct DataType<uint64_t> {
  static inline const int kFlag() { return kUInt64; }
};

template <>
struct DataType<float> {
  static inline const int kFlag() { return kFloat32; }
};

template <>
struct DataType<double> {
  static inline const int kFlag() { return kFloat64; }
};

}  // namespace dnn
}  // namespace hobot
#endif  // DNN_PLUGIN_HB_DNN_DTYPE_H_
