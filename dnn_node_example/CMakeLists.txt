# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(dnn_node_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_cv REQUIRED)

option(CV_BRIDGE_PKG "using cv_bridge pkg" OFF)
if (${CV_BRIDGE_PKG})
  message("using cv_bridge pkg")
  find_package(cv_bridge REQUIRED)
  add_definitions(-DCV_BRIDGE_PKG_ENABLED)
else()
  message("not using cv_bridge pkg")
endif ()

option(SHARED_MEM "using shared mem" ON)
if (${SHARED_MEM})
  message("using shared mem")
  # rclcpp definition
  add_definitions(-DUSING_HBMEM)
  find_package(hbm_img_msgs REQUIRED)
  add_definitions(-DSHARED_MEM_ENABLED)
endif ()

include_directories(include
${PROJECT_SOURCE_DIR}
)

set(SYS_ROOT ${CMAKE_SYSROOT})
message("SYS_ROOT is " ${SYS_ROOT})
include_directories(
  ${SYS_ROOT}/usr/include/opencv/
)

# 设置一个默认hobotcv库的安装路径
set(HOBOTCV_LIB_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
# 如果find_package成功，使用查找到的路径更新HOBOTCV_LIB_INSTALL_PATH
if(hobot_cv_LIBRARIES)
  message("hobot_cv_LIBRARIES is ${hobot_cv_LIBRARIES}")
  # 查找库路径的最后一个/符号
  string(FIND ${hobot_cv_LIBRARIES} "/" index REVERSE)
  # 获取库路径
  string(SUBSTRING ${hobot_cv_LIBRARIES} 0 ${index} HOBOTCV_LIB_INSTALL_PATH)
  message("update HOBOTCV_LIB_INSTALL_PATH as " ${HOBOTCV_LIB_INSTALL_PATH})
endif()

message("HOBOTCV_LIB_INSTALL_PATH is " ${HOBOTCV_LIB_INSTALL_PATH})

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/lib/hbmedia/
  ${HOBOTCV_LIB_INSTALL_PATH}
)

add_executable(example
src/example.cpp
src/dnn_example_node.cpp
src/image_utils.cpp
src/post_process/post_process_base.cpp
src/post_process/post_process_fasterrcnn.cpp
src/post_process/post_process_yolov2.cpp
src/post_process/post_process_yolov3.cpp
src/post_process/post_process_yolov5.cpp
src/post_process/post_process_classification.cpp
src/post_process/post_process_ssd.cpp
src/post_process/post_process_efficientdet.cpp
src/post_process/post_process_fcos.cpp
src/post_process/post_process_unet.cpp
)

ament_target_dependencies(
  example
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hobot_cv
)

if (${SHARED_MEM})
  target_link_libraries(example
    hbmem
  )
  ament_target_dependencies(
    example
    hbm_img_msgs
  )
endif ()

include_directories(include
${PROJECT_SOURCE_DIR}
)

if (${CV_BRIDGE_PKG})
ament_target_dependencies(
  example
  cv_bridge
)
endif()

# Install executables
install(
  TARGETS example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
