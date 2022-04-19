// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_BASE_PLUGIN_H_
#define _PLUGIN_BASE_PLUGIN_H_

#include <memory>
#include <string>
#include <utility>

#include "input/input_data.h"
#include "rclcpp/rclcpp.hpp"

class BasePlugin {
 public:
  BasePlugin() = default;
  virtual int Init(std::string config_file, std::string config_string);

  virtual int Start() = 0;

  virtual int Stop() = 0;

  virtual ~BasePlugin() = default;

 protected:
  virtual int LoadConfig(std::string &config_string)
  {
    RCLCPP_INFO(rclcpp::get_logger("example"),
        " %s ", config_string.c_str());
    return 0;
  }

 private:
  int LoadConfigFile(std::string &config_file);
};

#endif  // _PLUGIN_BASE_PLUGIN_H_
