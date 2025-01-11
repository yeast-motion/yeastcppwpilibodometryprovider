cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME "yeastcppwpilibodometryprovider")
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi -Wno-stringop-overflow -Wno-array-bounds)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(wpilib REQUIRED)

file(GLOB_RECURSE SOURCES
    "*.cpp"
    "*.c"
)

file(GLOB_RECURSE HEADERS
    "*.hpp"
    "*.h"
)

# Add the library source files
add_library(${PROJECT_NAME}
    ${SOURCES}
)

# Specify include directories for the library (if needed)
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ../yeastcpp/include
)

target_link_libraries(${PROJECT_NAME}
    wpimath
)

# Install the library and header files
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" # source directory
        DESTINATION "include" # target directory
        FILES_MATCHING # install only matched files
        PATTERN "*.hpp" # select header files
)