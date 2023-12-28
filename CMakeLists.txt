cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MyProject)

# Find OpenCV package
set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)

# Collect all .cpp files from the src directory
file(GLOB SOURCES "src/*.cpp")
file(GLOB INC "inc/*")

# Print the value of SOURCES for debugging purposes
message("Sources: ${SOURCES}")

# Include directories for the project headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

# Add the executable target
add_executable(MyProject ${INC} ${SOURCES})

# Set C++17 standard
target_compile_features(MyProject PRIVATE cxx_std_17)

# Add include directories
target_include_directories(MyProject PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS} # Include the OpenCV directories
)

# Link OpenCV libraries
target_link_libraries(MyProject PRIVATE ${OpenCV_LIBS})
