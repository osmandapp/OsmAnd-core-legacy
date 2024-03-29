set(CMAKE_TARGET_OS linux)
set(CMAKE_TARGET_CPU_ARCH amd64)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_COMPILER_FAMILY clang)
set(CMAKE_C_COMPILER_FLAGS "-O2 -m64 -msse4.1 -fstack-protector-all -Wstack-protector -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
set(CMAKE_CXX_COMPILER_FLAGS "-O2 -std=c++11 -m64 -msse4.1 -fstack-protector-all -Wstack-protector -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function")
