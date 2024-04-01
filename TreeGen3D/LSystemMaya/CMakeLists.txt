cmake_minimum_required(VERSION 3.18)

project(LSystemMaya)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add source files to the project
file(GLOB_RECURSE R2_SOURCES "${PROJECT_SOURCE_DIR}/src/R2/*.cpp")
list(APPEND R2SOURCES ${R2_SOURCES})

file(GLOB_RECURSE R3_SOURCES "${PROJECT_SOURCE_DIR}/src/R3/*.cpp")
list(APPEND R3SOURCES ${R3_SOURCES})

set(SOURCES
src/cylinder.cpp
${LSystem_SOURCE_DIR}/LSystem.cpp
src/LSystemCmd.cpp
src/PluginMain.cpp
src/LSystemNode.cpp
src/ImportImageCmd.cpp
src/tree_structure.cpp
${LSystem_SOURCE_DIR}/vec.cpp
)


# Add header files to the project
file(GLOB_RECURSE R2_HEADERS "${PROJECT_SOURCE_DIR}/src/R2/*.h")
list(APPEND R2HEADERS ${R2_HEADERS})

file(GLOB_RECURSE R3_HEADERS "${PROJECT_SOURCE_DIR}/src/R3/*.h")
list(APPEND R3HEADERS ${R3_HEADERS})

set(HEADERS
    src/cylinder.h
    ${LSystem_SOURCE_DIR}/LSystem.h
    src/LSystemCmd.h
    src/LSystemNode.h
    src/ImportImageCmd.h
    src/tree_structure.h
    ${LSystem_SOURCE_DIR}/matrix.h
    ${LSystem_SOURCE_DIR}/vec.h
)

find_package(Maya REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya)
target_include_directories(${PROJECT_NAME} 
    PUBLIC ${LSystem_SOURCE_DIR}
    PRIVATE Maya::Maya
    PUBLIC "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}"
)
MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})