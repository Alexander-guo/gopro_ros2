cmake_minimum_required(VERSION 3.8) 
project(gopro_ros2) 

set(CMAKE_CXX_STANDARD 14) 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# ===========================
# Dependencies
# ===========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ffmpeg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3 REQUIRED)

# ===========================
# Library
# ===========================
add_library(${PROJECT_NAME}
  src/main.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${FFMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag2_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${FFMPEG_LIBRARIES}
  ${OpenCV_LIBS}
  stdc++fs
)

# ===========================
# Executable
# ===========================
add_executable(gopro_to_db3 gopro_to_db3.cpp)
ament_target_dependencies(gopro_to_db3) 
target_link_libraries(gopro_to_db3 ${PROJECT_NAME})

# ===========================
# Install
# ===========================
install(TARGETS 
  ${PROJECT_NAME} 
  gopro_to_db3
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
