project(harfbuzz_osmand)
set(SKIP_INSTALL_ALL ON)

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wno-unused-variable")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-unused-variable")
endif()

add_subdirectory("${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/harfbuzz" "harfbuzz")
