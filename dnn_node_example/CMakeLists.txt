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
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dnn_node REQUIRED)

include_directories(include
${PROJECT_SOURCE_DIR}
)

set(DNN_NODE_INSTALL_PATH "../install/dnn_node")
message("DNN_NODE_INSTALL_PATH: " ${DNN_NODE_INSTALL_PATH})
include_directories(
  ${DNN_NODE_INSTALL_PATH}/include
  ${DNN_NODE_INSTALL_PATH}/include/dnn_node
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/dnn/include
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/include/
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/include
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/include/easy_dnn/
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/dnn/include
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/third-party/rapidjson/include
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/third-party/opencv/include
)

link_directories(
  ${DNN_NODE_INSTALL_PATH}/lib
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/dnn/lib
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/lib
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/third-party/opencv/lib
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/third-party/hobotlog/lib
)

add_executable(example
src/example.cpp
src/fasterrcnn_body_det_node.cpp
src/image_utils.cpp
src/fasterrcnn_kps_output_parser.cpp
src/image_subscriber.cpp
)

ament_target_dependencies(
  example
  rclcpp
  cv_bridge
  dnn_node
  sensor_msgs
)

target_link_libraries(example
  alog
  log
  easy_dnn
  dnn
  dnn_node
)

# Install executables
install(
  TARGETS example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/third-party/opencv/lib/
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/dnn/lib/
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY
  ${DNN_NODE_INSTALL_PATH}/dnn_node_deps/hobot/easy_dnn/lib/
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

ament_package()
