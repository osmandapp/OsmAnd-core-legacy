set(CMAKE_TARGET_OS linux-gnueabi)
set(CMAKE_TARGET_CPU_ARCH arm)
set(CMAKE_C_COMPILER arm-linux-gnueabi-gcc-4.6)
set(CMAKE_CXX_COMPILER arm-linux-gnueabi-g++-4.6)
set(CMAKE_COMPILER_FAMILY gcc)
set(CMAKE_C_COMPILER_FLAGS "-O2 -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_CXX_COMPILER_FLAGS "-O2 -std=c++11 -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
