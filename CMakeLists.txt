cmake_minimum_required(VERSION 3.15)
project(WebARKitLib)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch content from WebARKitLib repository
include(FetchContent)

FetchContent_Declare(
    WebARKitLib
    GIT_REPOSITORY https://github.com/webarkit/WebARKitLib.git
    GIT_TAG main # Change this to a specific tag or branch if needed
)

FetchContent_MakeAvailable(WebARKitLib)

# Add the library or executable
add_subdirectory(${webarkitlib_SOURCE_DIR} ${webarkitlib_BINARY_DIR})