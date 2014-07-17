set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER   /opt/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/bin/arm-linux-gnueabihf-g++)

# where is the target environment 
set(CMAKE_FIND_ROOT_PATH /opt/gcc-linaro-arm-linux-gnueabihf-4.7-2013.03-20130313_linux/arm-linux-gnueabihf)

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_CXX_FLAGS "-std=c++0x" CACHE STRING "" FORCE)
set(LINUX TRUE CACHE BOOL "" FORCE)
