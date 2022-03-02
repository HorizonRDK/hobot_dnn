// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef EASY_DNN_DETECTION_DATA_STRUCTURE_H
#define EASY_DNN_DETECTION_DATA_STRUCTURE_H

namespace hobot {
namespace easy_dnn {

struct PerceptionRect {
 public:
  float left;
  float top;
  float right;
  float bottom;
  float conf;
  int type;
  int perception_type;
  int conf_scale;
  int expand;
  int branch;
  friend bool operator>(const PerceptionRect &lhs, const PerceptionRect &rhs) {
    return (lhs.conf > rhs.conf);
  }
};

enum PerceptionType {
  PerceptionType_Person,
  PerceptionType_TrafficLight,
  PerceptionType_TrafficSign,
  PerceptionType_FullCar,
  PerceptionType_Veh_Rear,
  PerceptionType_Cyclist,
  PerceptionType_TrafficArrow,
  PerceptionType_TrafficCone,
  PerceptionType_Other
};

}  // namespace easy_dnn
}  // namespace hobot

#endif  // EASY_DNN_DETECTION_DATA_STRUCTURE_H
