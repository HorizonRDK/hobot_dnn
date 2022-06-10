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

#include "include/post_process/post_process_yolov3.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int Yolov3PostProcess::SetOutParser(Model* model_manage) {
  output_index_ = model_output_count_ - 1;
  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<hobot::dnn_node::Yolo3_darknetAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // set yolov3 paser
  auto output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "yolov3_branch");
  for (int i = 0; i < output_index_; ++i) {
    output_desc->GetDependencies().push_back(i);
  }
  output_desc->SetType("yolov3");
  model_manage->SetOutputDescription(output_desc);
  std::shared_ptr<OutputParser> yolov3_out_parser =
      std::make_shared<hobot::dnn_node::Yolo3DarknetOutputParser>();
  model_manage->SetOutputParser(output_index_, yolov3_out_parser);
  return 0;
}
