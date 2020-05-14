cmake_minimum_required(VERSION 2.8.3)
project(mpu6050_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(mpu_processing src/mpu_processing.cpp)
target_link_libraries(mpu_processing ${catkin_LIBRARIES})
