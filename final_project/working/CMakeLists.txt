cmake_minimum_required(VERSION 3.10)
project(realsense_pcl_capture)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
)

# Add definitions from PCL
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(realsense_pcl_capture main.cpp)

# Link libraries
target_link_libraries(realsense_pcl_capture
    ${realsense2_LIBRARY}
    ${PCL_LIBRARIES}
    glfw
    OpenGL::GL
)