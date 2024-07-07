cmake_minimum_required(VERSION 3.10.0)
project(fisheyeCalib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99") 
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenCV package
find_package(OpenCV 4 REQUIRED) 

if(OpenCV_FOUND)
    message(STATUS "OpenCV have found")
endif(OpenCV_FOUND)

add_executable(undistortFishEye  undistortFishEye.cpp)
target_include_directories(undistortFishEye PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(undistortFishEye  ${OpenCV_LIBS} )
set_target_properties(undistortFishEye PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

