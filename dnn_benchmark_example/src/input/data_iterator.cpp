// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "input/data_iterator.h"
#include "rclcpp/rclcpp.hpp"

int DataIterator::Init(std::string config_file,
                       std::string config_string)
{
  if (!config_file.empty()) {
    int ret_code = this->LoadConfigFile(config_file);
    if (ret_code != 0) {
      return ret_code;
    }
  }

  if (!config_string.empty()) {
    int ret_code = this->LoadConfig(config_string);
    if (ret_code != 0) {
      return ret_code;
    }
  }
  return 0;
}

int DataIterator::LoadConfigFile(std::string& config_file)
{
  std::ifstream ifs(config_file.c_str());
  if (!ifs) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Open config file %s failed!", config_file.c_str());
    return -1;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  std::string contents(buffer.str());
  return this->LoadConfig(contents);
}

DataIterator* DataIterator::GetImpl(const std::string& module_name)
{
  return DataIteratorFactory::GetInstance()->GetDataIterator(
      module_name.data());
}
