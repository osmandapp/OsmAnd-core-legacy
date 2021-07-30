project(harfbuzz_osmand)

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

if(CMAKE_TARGET_OS STREQUAL "linux")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-threadsafe-statics")
endif()

set(HARFBUZZ_ROOT "${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/harfbuzz")

set(UPSTREAM "${HARFBUZZ_ROOT}")
include_directories(AFTER SYSTEM
	"${UPSTREAM}/src"
)

add_library(harfbuzz_osmand STATIC
	"${UPSTREAM}/src/harfbuzz.cc"
)