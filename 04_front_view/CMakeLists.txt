cmake_minimum_required(VERSION 3.5)
  
project(front_view)

set(CMAKE_CXX_STANDARD 17)

# This instructs cmake to look for the most recent
find_package(Boost COMPONENTS system filesystem REQUIRED)

# vpi instance installed on the system.
find_package(vpi REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_DIR=${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")

find_package(Eigen3 REQUIRED NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
endif()

find_package(yaml-cpp REQUIRED)

include_directories(include)

set(LIB_MVS mvs)
add_library(${LIB_MVS} 
    src/camera_model/camera_model_base.cpp
    src/camera_model/double_sphere.cpp
    src/calib/kalibr_parser.cpp)

target_link_libraries(${LIB_MVS}
    yaml-cpp
    Eigen3::Eigen
    ${OpenCV_LIBRARIES})

# Creates the blur executable target
set(EXE ${PROJECT_NAME})
add_executable(${EXE} src/main.cpp)

# It uses vpi and opencv. CMake will automatically
# set up the correct header and library directories,
# and make hello_work link to these libraries.

# stdc++fs
target_link_libraries(${EXE} 
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    vpi 
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen
    ${LIB_MVS})

# # OpenCV < 3 uses a different library for image i/o
# if(OpenCV_VERSION VERSION_LESS 3)
#     target_link_libraries(${EXE} opencv_highgui)
# else()
#     target_link_libraries(${EXE} opencv_imgcodecs)
# endif()


