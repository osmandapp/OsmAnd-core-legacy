project(harfbuzz_osmand)
set(SKIP_INSTALL_ALL ON)
add_subdirectory("${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/harfbuzz" "harfbuzz")

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()
