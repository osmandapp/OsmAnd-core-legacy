set(CMAKE_TARGET_OS linux)
set(CMAKE_TARGET_CPU_ARCH i686)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_COMPILER_FAMILY gcc)
set(CMAKE_C_COMPILER_FLAGS "-m32 -msse4.1 -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_CXX_COMPILER_FLAGS "-std=c++0x -m32 -msse4.1 -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
