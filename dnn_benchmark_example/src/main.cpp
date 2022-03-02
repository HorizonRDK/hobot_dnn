// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "include/workflow.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("example"),
                "This is dnn benchmark example!");

    auto node = std::make_shared<Workflow>("dnn_benchamark_node");

    auto ret_code = node->WorkflowInit();
    if (0 != ret_code)
    {
        RCLCPP_WARN(rclcpp::get_logger("example"),
                "Workflow init failed!");
        return -1;
    }
    node->WorkflowStart();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
