cmake_minimum_required(VERSION 3.5)
  
project(vpi_camera_model)

set(CMAKE_CXX_STANDARD 17)

# This instructs cmake to look for the most recent
# vpi instance installed on the system.
find_package(vpi REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_DIR=${OpenCV_DIR}")
message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")

# function(dump_cmake_variables)
#     get_cmake_property(_variableNames VARIABLES)
#     list (SORT _variableNames)
#     foreach (_variableName ${_variableNames})
#         if (ARGV0)
#             unset(MATCHED)
#             string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})
#             if (NOT MATCHED)
#                 continue()
#             endif()
#         endif()
#         message(STATUS "${_variableName}=${${_variableName}}")
#     endforeach()
# endfunction()

# dump_cmake_variables(OpenCV)

find_package(Eigen3 REQUIRED NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
endif()

include_directories(include)

# Creates the blur executable target
set(EXE vpi_camera_model)
add_executable(${EXE} src/camera_model/camera_model_base.cpp src/camera_model/double_sphere.cpp src/main.cpp)

# It uses vpi and opencv. CMake will automatically
# set up the correct header and library directories,
# and make hello_work link to these libraries.
target_link_libraries(${EXE} 
    stdc++fs
    vpi 
    ${OpenCV_LIBRARIES}
    Eigen3::Eigen)

# # OpenCV < 3 uses a different library for image i/o
# if(OpenCV_VERSION VERSION_LESS 3)
#     target_link_libraries(${EXE} opencv_highgui)
# else()
#     target_link_libraries(${EXE} opencv_imgcodecs)
# endif()
