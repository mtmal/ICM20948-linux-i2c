cmake_minimum_required(VERSION 3.10)

# set the project name
project(ICM20948)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-fPIC -g -pedantic -Wall -Wextra")

if(LOG) 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLOG=1")
endif()

include_directories(src)

# Build the actual library
add_library(ICM20948 SHARED src/AHRSMath.cpp src/MadgwickAHRS.cpp src/SimpleAHRS.cpp src/ICM20948.cpp)

# add the test application
add_executable(test_ICM20948 tests/ICM20948_app.cpp)
target_link_libraries(test_ICM20948 ICM20948)
