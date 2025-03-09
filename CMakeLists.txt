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

# add the I2C submodule
if (CUSTOM_I2C)
    include_directories(${CUSTOM_I2C}/src)
else()
    add_subdirectory(I2C)
    include_directories(I2C/src)
endif()

include_directories(src)

# Build the actual library
add_library(ICM20948 SHARED src/ahrs/ahrs_math.cpp src/ahrs/madgwick_ahrs.cpp src/ahrs/simple_ahrs.cpp src/icm20948.cpp)

# add the test application
add_executable(test_icm20948 tests/icm20948_app.cpp)
target_link_libraries(test_icm20948 ICM20948 I2C)
