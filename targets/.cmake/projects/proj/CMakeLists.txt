project(proj_osmand)

set(ROOT "${OSMAND_ROOT}/externals/proj")
set(UPSTREAM "${ROOT}/upstream.patched")

include("proj_config.cmake")

# Target specific
set(target_specific_private_definitions "")
set(target_specific_libraries_public "")
if (CMAKE_TARGET_OS STREQUAL "windows")
	set(target_specific_private_definitions "${target_specific_private_definitions}"
		-DNOMINMAX
	)
else()
	find_library(PTHREAD_LIBRARY pthread)
	if (PTHREAD_LIBRARY)
		set(target_specific_libraries_public ${target_specific_libraries_public}
			${PTHREAD_LIBRARY}
		)
	endif()
endif()

file(GLOB src_core "${UPSTREAM}/src/*.c*")
file(GLOB src_projections "${UPSTREAM}/src/projections/*.cpp")
file(GLOB src_conversions "${UPSTREAM}/src/conversions/*.cpp")
file(GLOB src_transformations "${UPSTREAM}/src/transformations/*.cpp")
file(GLOB src_iso19111 "${UPSTREAM}/src/iso19111/*.cpp")
file(GLOB src_iso19111_operation "${UPSTREAM}/src/iso19111/operation/*.cpp")

add_library(proj_osmand STATIC
	${src_core}
	${src_projections}
	${src_conversions}
	${src_transformations}
	${src_iso19111}
	${src_iso19111_operation}
)
target_compile_definitions(proj_osmand
	PRIVATE
		${target_specific_private_definitions}
)
target_include_directories(proj_osmand
	PUBLIC
		"${ROOT}"
		"${CMAKE_CURRENT_BINARY_DIR}"
		"${UPSTREAM}/include"
		"${UPSTREAM}/src"
)
target_link_libraries(proj_osmand
	LINK_PUBLIC
		sqlite_osmand
		${target_specific_libraries_public}
)
