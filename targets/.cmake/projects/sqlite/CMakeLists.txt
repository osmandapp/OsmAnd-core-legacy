project(sqlite_osmand)

set(ROOT "${OSMAND_ROOT}/externals/sqlite")
set(UPSTREAM "${ROOT}/upstream.patched")

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

add_library(sqlite_osmand STATIC
	"${UPSTREAM}/sqlite3.c"
)
target_compile_definitions(sqlite_osmand
	PRIVATE
		-DSQLITE_ENABLE_RTREE
		-DSQLITE_ENABLE_FTS4
		-DSQLITE_ENABLE_FTS5
		-DSQLITE_ENABLE_JSON1
		-DSQLITE_ENABLE_RBU
		-DSQLITE_ENABLE_STAT4
)
target_include_directories(sqlite_osmand
	PUBLIC
		"${UPSTREAM}"
)
