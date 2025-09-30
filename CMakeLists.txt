cmake_minimum_required(VERSION 3.8) 
project(gopro_ros2) 

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
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
find_package(Eigen3 REQUIRED)

# include_directories(
#   include
#   ${FFMPEG_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

# ===========================
# Build Common Library
# ===========================
# Gather all .cpp and .c files in src/
file(GLOB GOPRO_SRC_FILES "src/*.cpp" "src/*.c")
add_library(${PROJECT_NAME}
  ${GOPRO_SRC_FILES}
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    include
    ${FFMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
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
option(BUILD_GOPRO_TO_ASL "Build gopro_to_asl executable" OFF)

if(BUILD_GOPRO_TO_ASL)
  add_executable(gopro_to_asl gopro_to_asl.cpp)
  ament_target_dependencies(gopro_to_asl 
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    rosbag2_cpp
  )
  target_link_libraries(gopro_to_asl 
    ${PROJECT_NAME}
  )

  install(TARGETS gopro_to_asl DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(gopro_to_rosbag gopro_to_rosbag.cpp)
ament_target_dependencies(gopro_to_rosbag 
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag2_cpp
)
target_link_libraries(gopro_to_rosbag 
  ${PROJECT_NAME}
)

# ===========================
# Install
# ===========================
install(TARGETS 
  gopro_to_rosbag
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
