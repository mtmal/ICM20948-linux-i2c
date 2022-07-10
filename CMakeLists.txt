cmake_minimum_required(VERSION 3.10)

# set the project name
project(ICM20948)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-fPIC -g -pedantic -Wall -Wextra")

# make sure we have the necessary submodule
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/I2C/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

if(LOG) 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLOG=1")
endif()

add_subdirectory(I2C)
include_directories(I2C/src)
include_directories(src)

# Build the actual library
add_library(ICM20948 SHARED src/AHRSMath.cpp src/MadgwickAHRS.cpp src/SimpleAHRS.cpp src/ICM20948.cpp)

# add the test application
add_executable(test_ICM20948 tests/ICM20948_app.cpp)
target_link_libraries(test_ICM20948 ICM20948 I2C)
