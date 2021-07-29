project(harfbuzz_osmand)
#set(SKIP_INSTALL_ALL ON)

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

#add_subdirectory("${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/harfbuzz" "harfbuzz")

set(HARFBUZZ_ROOT "${OSMAND_ROOT}/externals/skia/upstream.patched/third_party/externals/harfbuzz")

set(UPSTREAM "${HARFBUZZ_ROOT}")
include_directories(AFTER SYSTEM
	"${UPSTREAM}/src"
)

add_library(harfbuzz_osmand STATIC
	"${UPSTREAM}/src/harfbuzz.cc"
)
