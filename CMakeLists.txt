cmake_minimum_required(VERSION 3.5.1)
project(cam_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
#set(CMAKE_C_COMPILER "/path/to/your/c/compiler/executable")
#set(CMAKE_CXX_COMPILER "/path/to/your/cpp/compiler/executable")
set(CMAKE_CXX_STANDARD 14)
set(GCC_COVERAGE_COMPILE_FLAGS " -Ofast")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set(USE_AVX_INSTRUCTIONS ON CACHE BOOL "Use AVX instructions")
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(OpenCV 3.4.7 QUIET)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

include_directories(include/camera_calibration
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_markers src/aruco_markers.cpp)
target_link_libraries(aruco_markers ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(aruco_markers cam_calibration_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(mono_calibration src/camera_calibration_node.cpp  src/camera_calibration.cpp)
target_link_libraries(mono_calibration ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(mono_calibration cam_calibration_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#add_executable(testing_file src/testing_file.cpp )
#target_link_libraries(testing_file ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereo_calibration src/stereo_calibration.cpp)
target_link_libraries(stereo_calibration ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(stereo_calibration cam_calibration_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

