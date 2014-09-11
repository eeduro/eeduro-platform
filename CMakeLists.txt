cmake_minimum_required(VERSION 2.8)

# Load CMake modules
include(CheckCXXCompilerFlag)

# Additional include and link directories
include_directories(${ADDITIONAL_INCLUDE_DIRS})
link_directories(${ADDITIONAL_LINK_DIRS})

## Check and set C++ compiler flags
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler!")
endif()

# EEDURO Library
add_subdirectory(eeduro)

# EEDURO utilities
add_subdirectory(util)

# EEDURO tests
add_subdirectory(test)

# EEDURO Robots
add_subdirectory(delta)
add_subdirectory(omni)
add_subdirectory(7axis)
