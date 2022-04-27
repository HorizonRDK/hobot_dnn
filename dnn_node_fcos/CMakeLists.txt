cmake_minimum_required(VERSION 3.5)
project(dnn_node_fcos)

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
find_package(dnn_node REQUIRED)

option(CV_BRIDGE_PKG "using cv_bridge pkg" ON)
if (${CV_BRIDGE_PKG})
  message("using cv_bridge pkg")
  find_package(cv_bridge REQUIRED)
  add_definitions(-DCV_BRIDGE_PKG_ENABLED)
else()
  message("not using cv_bridge pkg")
endif ()

option(SHARED_MEM "using shared mem" OFF)
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
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/include/opencv/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/lib/hobot/opencv
  ${SYS_ROOT}/usr/lib/hobot/hobotlog
  ${SYS_ROOT}/usr/lib/hbbpu/
)

add_executable(fcos_example
src/dnn_fcos_example.cpp
src/fcos_det_node.cpp
src/image_utils.cpp
src/fcos_output_parser.cpp
)

ament_target_dependencies(
  fcos_example
  rclcpp
  dnn_node
  sensor_msgs
)

if (${SHARED_MEM})
  target_link_libraries(fcos_example
    hbmem
  )
  ament_target_dependencies(
    fcos_example
    hbm_img_msgs
  )
endif ()

include_directories(include
${PROJECT_SOURCE_DIR}
)

if (${CV_BRIDGE_PKG})
ament_target_dependencies(
  fcos_example
  cv_bridge
)
endif()

# Install executables
install(
  TARGETS fcos_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

ament_package()
