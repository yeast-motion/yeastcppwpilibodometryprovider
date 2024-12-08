cmake_minimum_required(VERSION 3.10)

project(yeastcppwpilibodometryprovider)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES 
    "*.cpp"
    "*.c" 
)

file(GLOB_RECURSE HEADERS 
    "*.hpp" 
    "*.h"
)

# Add the library source files
add_library(yeastcppwpilibodometryprovider 
    ${SOURCES}
)

find_package(Eigen3 REQUIRED)

# Specify include directories for the library (if needed)
target_include_directories(yeastcppwpilibodometryprovider PUBLIC
    include
    /usr/local/include/eigen3
    /usr/local/include/wpiutil
    /usr/local/include/wpimath
    ${ADDITIONAL_INCLUDE_PATHS}
)

# Install the library and header files
install(TARGETS yeastcppwpilibodometryprovider DESTINATION lib)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.hpp" # select header files
)