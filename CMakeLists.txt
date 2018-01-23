cmake_minimum_required(VERSION 3.0)

project(KinectFusionApplication)

# Setting some paths
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Path of find modules
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# Targets
add_subdirectory(KinectFusionLib)
add_subdirectory(KinectFusionApp)
