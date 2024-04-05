cmake_minimum_required(VERSION 3.18)

project(LSystemMaya)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set this to your actual Qt5 installation path
set(CMAKE_PREFIX_PATH "D:/App/Qt/5.15.2/msvc2019_64/lib/cmake" ${CMAKE_PREFIX_PATH})

# Set Boost include directory
SET(BOOST_ROOT "F:/Study/GitHubResources/IPML2d/L-system inference/boost_1_84_0")
find_package(Boost REQUIRED)

# Add source files to the project
file(GLOB_RECURSE R2_SOURCES "${PROJECT_SOURCE_DIR}/src/R2/*.cpp")
file(GLOB_RECURSE R3_SOURCES "${PROJECT_SOURCE_DIR}/src/R3/*.cpp")

set(SOURCES
    src/cylinder.cpp
    ${LSystem_SOURCE_DIR}/LSystem.cpp
    src/LSystemCmd.cpp
    src/PluginMain.cpp
    src/LSystemNode.cpp
    src/ImportImageCmd.cpp
    src/tree_structure.cpp
    src/turtle.cpp
    ${LSystem_SOURCE_DIR}/vec.cpp
    ${R2_SOURCES}
    ${R3_SOURCES}
)

# Add header files to the project
file(GLOB_RECURSE R2_HEADERS "${PROJECT_SOURCE_DIR}/src/R2/*.h")
file(GLOB_RECURSE R3_HEADERS "${PROJECT_SOURCE_DIR}/src/R3/*.h")

set(HEADERS
    src/cylinder.h
    ${LSystem_SOURCE_DIR}/LSystem.h
    src/LSystemCmd.h
    src/LSystemNode.h
    src/ImportImageCmd.h
    src/tree_structure.h
    src/turtle.h
    src/global_definations.h
    ${LSystem_SOURCE_DIR}/matrix.h
    ${LSystem_SOURCE_DIR}/vec.h
    ${R2_HEADERS}
    ${R3_HEADERS}
)

# Organize the source and header files in IDE
source_group("Source Files\\R2" FILES ${R2_SOURCES})
source_group("Header Files\\R2" FILES ${R2_HEADERS})
source_group("Source Files\\R3" FILES ${R3_SOURCES})
source_group("Header Files\\R3" FILES ${R3_HEADERS})

find_package(Maya REQUIRED)

# Find the necessary Qt5 components
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Svg REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Link the Qt5 libraries with your project
target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya
    Qt5::Core Qt5::Widgets Qt5::OpenGL Qt5::Concurrent Qt5::Test Qt5::Gui Qt5::PrintSupport Qt5::Svg
)

# Add the include directories for the Qt5 modules and Boost
target_include_directories(${PROJECT_NAME} 
    PUBLIC ${LSystem_SOURCE_DIR}
    PRIVATE Maya::Maya
    PUBLIC ${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Concurrent_INCLUDE_DIRS} ${Qt5Test_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5PrintSupport_INCLUDE_DIRS} ${Qt5Svg_INCLUDE_DIRS}
    PUBLIC ${Boost_INCLUDE_DIRS} # Add this line to include Boost
    PUBLIC "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}"
    PUBLIC "${PROJECT_SOURCE_DIR}/src/udgcd"
)

MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})
