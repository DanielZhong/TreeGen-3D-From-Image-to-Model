cmake_minimum_required(VERSION 3.18)

project(LSystem)


# Add source files to the project
set(SOURCES
    src/LSystem.cpp
    src/main.cpp
    src/vec.cpp
)

# Add header files to the project
set(HEADERS
    src/LSystem.h
    src/matrix.h
    src/vec.h
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Set additional configurations for the executable
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set runtime library for Debug|x64
set_property(TARGET ${PROJECT_NAME} PROPERTY
    MSVC_RUNTIME_LIBRARY "MultiThreadedDebugDLL"
)
