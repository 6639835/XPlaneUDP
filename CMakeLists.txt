cmake_minimum_required(VERSION 3.16)
project(XPlaneUDP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

add_executable(XPlaneUDP 
    main.cpp
    XPlaneUDP.cpp
    XPlaneUDP.hpp
)

# Include Boost directories
target_include_directories(XPlaneUDP PRIVATE ${Boost_INCLUDE_DIRS})

# Link libraries
target_link_libraries(XPlaneUDP ${Boost_LIBRARIES})

# Platform-specific libraries
if(WIN32)
    target_link_libraries(XPlaneUDP ws2_32)
endif()