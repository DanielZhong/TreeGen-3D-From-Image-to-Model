cmake_minimum_required(VERSION 3.18)

project(MyMaya)

set(MAYA_LOCATION "D:/App/Autodesk/Maya2024")

if (NOT EXISTS "${MAYA_LOCATION}")
  message(FATAL_ERROR "Maya not found at ${MAYA_LOCATION}, please set MAYA_LOCATION to the correct path!!!")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(LSystem_SOURCE_DIR ${CMAKE_SOURCE_DIR}/LSystem/src)

add_subdirectory(LSystem)
add_subdirectory(LSystemMaya)