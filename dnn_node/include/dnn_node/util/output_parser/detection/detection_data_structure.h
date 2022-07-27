// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
