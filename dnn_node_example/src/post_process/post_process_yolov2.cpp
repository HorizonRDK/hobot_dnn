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

#include "include/post_process/post_process_yolov2.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int Yolov2PostProcess::SetOutParser(Model* model_manage) {
  RCLCPP_INFO(rclcpp::get_logger("Yolov2PostProcess"), "Set out parser");
  output_index_ = model_output_count_ - 1;
  std::shared_ptr<OutputParser> parser =
      std::make_shared<hobot::dnn_node::Yolo2OutputParser>();
  model_manage->SetOutputParser(output_index_, parser);

  return 0;
}
