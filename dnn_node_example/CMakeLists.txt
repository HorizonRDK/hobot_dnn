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

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
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
