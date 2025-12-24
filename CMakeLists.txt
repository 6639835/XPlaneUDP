cmake_minimum_required(VERSION 4.0)
project(XPlaneUDP)

set(CMAKE_CXX_STANDARD 20)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(XPlaneUDP main.cpp
        XPlaneUDP.cpp
        XPlaneUDP.hpp
)
target_link_libraries(XPlaneUDP ${Boost_LIBRARIES} ws2_32)

add_executable(performanceTest test.cpp
        XPlaneUDP.cpp
        XPlaneUDP.hpp
)
target_link_libraries(performanceTest ${Boost_LIBRARIES} ws2_32)
