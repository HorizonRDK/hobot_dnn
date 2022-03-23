cmake_minimum_required(VERSION 3.5)
project(dnn_node)

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

include_directories(include include/util/)

message("SYS_ROOT is " ${SYS_ROOT})
include_directories(
  ${SYS_ROOT}/usr/include/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hobot/
  ${SYS_ROOT}/usr/lib/hobot/dnn
  ${SYS_ROOT}/usr/lib/hobot/easy_dnn
  ${SYS_ROOT}/usr/lib/hobot/opencv
  ${SYS_ROOT}/usr/lib/hobot/hobotlog
  ${SYS_ROOT}/usr/lib/hobot/hbbpu/

)

# Build image_transport library
add_library(${PROJECT_NAME} SHARED
  src/dnn_node.cpp
  src/util/threads/threadpool.cpp
)

target_link_libraries(${PROJECT_NAME}
  opencv_world
  alog
  log
  hobotlog.a
  easy_dnn
  dnn
  cnn_intf
  ion
  hbrt_bernoulli_aarch64
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install libraries
install(TARGETS dnn_node
  DESTINATION lib/)

# Install include directories
install(
  DIRECTORY include/dnn_node/
  DESTINATION include/dnn_node/
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_BUILDING_LIBRARY")

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
    
# specific order: dependents before dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

# Build example
# add_subdirectory(example)

if(BUILD_TESTING)
  # Build gtest
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()