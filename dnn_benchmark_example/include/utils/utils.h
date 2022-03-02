// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _UTILS_UTILS_H_
#define _UTILS_UTILS_H_

#include <string>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"

#define RED_COMMENT_START "\033[31m "
#define RED_COMMENT_END " \033[0m"
#define YELLOW_COMMENT_START "\033[33m "
#define YELLOW_COMMENT_END "\033[0m"

/**
 * Dump rapidjson to string
 * @param[in] rapidjson::Value val
 * @return json string
 */
std::string json_to_string(rapidjson::Value &val);

uint64_t currentMicroseconds();

#endif  // _UTILS_UTILS_H_
