// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_INPUT_PLUGIN_H_
#define _PLUGIN_INPUT_PLUGIN_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "base_plugin.h"
#include "input/data_iterator.h"
#include "workflow.h"

/**
 * Plugin for input producer
 */
class InputProducerPlugin : public BasePlugin {
 public:
  static InputProducerPlugin *GetInstance() {
    static InputProducerPlugin instance;
    return &instance;
  }

  /**
   * Init input producer plugin from config file & config string
   * @param[in] config_file: config file path
   *        the config_file should be in the json format
   *        example:
   *        {
   *            "input_type": "image", # one of [camera, image,
   *                                    # network,preprocessed_image] "limit":
   * 2,
   *            ...   # see `camera_data_iterator`
   *                  # `image_list_data_iterator`
   *                  # `network_data_iterator`
   *                  # `preprocessed_image_iterator` and so on
   *        }
   * @param[in] config_string: config string, same as config_file
   * @return 0 if success
   */
  int Init(std::string config_file, std::string config_string) override;

  void Run();

  int Release();

  bool IsRunning();

  int Start() override;

  int Stop() override;

  int LoadConfig(std::string &config_string);

  ~InputProducerPlugin() override;

  void registerWork(Workflow *work);

  void set_model_input_width_height(int model_input_w, int model_input_h);

 private:
  InputProducerPlugin() = default;

 private:
  DataIterator *data_iterator_ = nullptr;
  std::shared_ptr<std::thread> produce_thread_;
  std::atomic<bool> stop_;
  volatile int produced_count_ = 0;
  volatile int released_count_ = 0;
  int limit_ = 3;
  Workflow *work = nullptr;

  int model_input_width_ = -1;
  int model_input_height_ = -1;

  std::condition_variable cv_;
  std::mutex m_;
};

#endif  // _PLUGIN_INPUT_PLUGIN_H_
