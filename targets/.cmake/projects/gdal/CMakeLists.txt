project(gdal_osmand)

set(ROOT "${OSMAND_ROOT}/externals/gdal")
set(UPSTREAM "${ROOT}/upstream.patched")

set(target_specific_sources "")
set(target_specific_public_libraries "")
set(target_specific_private_definitions "")

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

if (CMAKE_COMPILER_FAMILY STREQUAL "msvc")
	configure_file("${UPSTREAM}/port/cpl_config.h.vc.common" "${CMAKE_CURRENT_BINARY_DIR}/cpl_config.h" COPYONLY)
else()
	include("cpl_config.cmake")
endif()

if (WORDS_BIGENDIAN)
	set(HOST_FILLORDER "FILLORDER_MSB2LSB")
else()
	set(HOST_FILLORDER "FILLORDER_LSB2MSB")
endif()

if (CMAKE_TARGET_OS STREQUAL "android")
	# NOTE: See https://android.googlesource.com/platform/bionic/+/master/docs/32-bit-abi.md
	if (NOT UNIX_STDIO_64)
		set(target_specific_private_definitions ${target_specific_private_definitions}
			-DBUILD_WITHOUT_64BIT_OFFSET
		)
	endif()
endif()

file(GLOB gdal_alg "${UPSTREAM}/gdal/alg/*.c*")

file(GLOB gdal_apps "${UPSTREAM}/gdal/apps/*_lib.c*")
set(gdal_apps_common
	"${UPSTREAM}/gdal/apps/commonutils.cpp"
)

file(GLOB gdal_frmts "${UPSTREAM}/gdal/frmts/*.c*")
file(GLOB gdal_frmts_derived "${UPSTREAM}/gdal/frmts/derived/*.c*")
file(GLOB gdal_frmts_gtiff "${UPSTREAM}/gdal/frmts/gtiff/*.c*")
file(GLOB gdal_frmts_gtiff__remove
	"${UPSTREAM}/gdal/frmts/gtiff/*jxl.c*"
)
list(REMOVE_ITEM gdal_frmts_gtiff ${gdal_frmts_gtiff__remove})

file(GLOB gdal_frmts_gtiff_libtiff "${UPSTREAM}/gdal/frmts/gtiff/libtiff/*.c*")
file(GLOB gdal_frmts_gtiff_libgeotiff "${UPSTREAM}/gdal/frmts/gtiff/libgeotiff/*.c*")
file(GLOB gdal_frmts_hfa "${UPSTREAM}/gdal/frmts/hfa/*.c*")
file(GLOB gdal_frmts_kml "${UPSTREAM}/gdal/frmts/kml/*.c*")
file(GLOB gdal_frmts_mem "${UPSTREAM}/gdal/frmts/mem/*.c*")
file(GLOB gdal_frmts_png "${UPSTREAM}/gdal/frmts/png/*.c*")
file(GLOB gdal_frmts_vrt "${UPSTREAM}/gdal/frmts/vrt/*.c*")

# TODO: fizzers

file(GLOB gdal_gcore "${UPSTREAM}/gdal/gcore/*.c*")
file(GLOB gdal_gcore_mdreader "${UPSTREAM}/gdal/gcore/mdreader/*.c*")

# TODO: gnm

file(GLOB gdal_ogr "${UPSTREAM}/gdal/ogr/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts "${UPSTREAM}/gdal/ogr/ogrsf_frmts/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_generic "${UPSTREAM}/gdal/ogr/ogrsf_frmts/generic/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_geojson "${UPSTREAM}/gdal/ogr/ogrsf_frmts/geojson/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_geojson_libjson "${UPSTREAM}/gdal/ogr/ogrsf_frmts/geojson/libjson/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_kml "${UPSTREAM}/gdal/ogr/ogrsf_frmts/kml/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_mem "${UPSTREAM}/gdal/ogr/ogrsf_frmts/mem/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_mitab "${UPSTREAM}/gdal/ogr/ogrsf_frmts/mitab/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_vrt "${UPSTREAM}/gdal/ogr/ogrsf_frmts/vrt/*.c*")
file(GLOB gdal_ogr_ogrfs_frmts_shape "${UPSTREAM}/gdal/ogr/ogrsf_frmts/shape/*.c*")

set(gdal_port
	"${UPSTREAM}/gdal/port/cpl_alibaba_oss.cpp"
	"${UPSTREAM}/gdal/port/cpl_atomic_ops.cpp"
	"${UPSTREAM}/gdal/port/cpl_aws.cpp"
	"${UPSTREAM}/gdal/port/cpl_aws_win32.cpp"
	"${UPSTREAM}/gdal/port/cpl_azure.cpp"
	"${UPSTREAM}/gdal/port/cpl_base64.cpp"
	"${UPSTREAM}/gdal/port/cpl_compressor.cpp"
	"${UPSTREAM}/gdal/port/cpl_conv.cpp"
	"${UPSTREAM}/gdal/port/cpl_cpu_features.cpp"
	"${UPSTREAM}/gdal/port/cpl_csv.cpp"
	"${UPSTREAM}/gdal/port/cpl_error.cpp"
	"${UPSTREAM}/gdal/port/cpl_findfile.cpp"
	"${UPSTREAM}/gdal/port/cpl_getexecpath.cpp"
	"${UPSTREAM}/gdal/port/cpl_google_cloud.cpp"
	"${UPSTREAM}/gdal/port/cpl_google_oauth2.cpp"
	"${UPSTREAM}/gdal/port/cpl_hash_set.cpp"
	"${UPSTREAM}/gdal/port/cpl_http.cpp"
	"${UPSTREAM}/gdal/port/cpl_json.cpp"
	"${UPSTREAM}/gdal/port/cpl_json_streaming_parser.cpp"
	"${UPSTREAM}/gdal/port/cpl_json_streaming_writer.cpp"
	"${UPSTREAM}/gdal/port/cpl_list.cpp"
	"${UPSTREAM}/gdal/port/cpl_md5.cpp"
	"${UPSTREAM}/gdal/port/cpl_minixml.cpp"
	"${UPSTREAM}/gdal/port/cpl_minizip_ioapi.cpp"
	"${UPSTREAM}/gdal/port/cpl_minizip_unzip.cpp"
	"${UPSTREAM}/gdal/port/cpl_minizip_zip.cpp"
	"${UPSTREAM}/gdal/port/cpl_multiproc.cpp"
	"${UPSTREAM}/gdal/port/cpl_path.cpp"
	"${UPSTREAM}/gdal/port/cpl_progress.cpp"
	"${UPSTREAM}/gdal/port/cpl_quad_tree.cpp"
	"${UPSTREAM}/gdal/port/cpl_recode.cpp"
	"${UPSTREAM}/gdal/port/cpl_recode_iconv.cpp"
	"${UPSTREAM}/gdal/port/cpl_recode_stub.cpp"
	"${UPSTREAM}/gdal/port/cpl_sha1.cpp"
	"${UPSTREAM}/gdal/port/cpl_spawn.cpp"
	"${UPSTREAM}/gdal/port/cpl_string.cpp"
	"${UPSTREAM}/gdal/port/cpl_strtod.cpp"
	"${UPSTREAM}/gdal/port/cpl_swift.cpp"
	"${UPSTREAM}/gdal/port/cpl_time.cpp"
	"${UPSTREAM}/gdal/port/cpl_userfaultfd.cpp"
	"${UPSTREAM}/gdal/port/cpl_vax.cpp"
	"${UPSTREAM}/gdal/port/cpl_virtualmem.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsi_error.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsi_mem.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_abstract_archive.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_adls.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_az.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_buffered_reader.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_cache.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_crypt.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_curl.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_curl_streaming.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_gs.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_gzip.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_hdfs.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_oss.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_plugin.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_s3.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_sparsefile.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_stdin.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_stdout.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_subfile.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_swift.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_tar.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_unix_stdio_64.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_uploadonclose.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_webhdfs.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsil_win32.cpp"
	"${UPSTREAM}/gdal/port/cpl_vsisimple.cpp"
	"${UPSTREAM}/gdal/port/cpl_worker_thread_pool.cpp"
	"${UPSTREAM}/gdal/port/cpl_xml_validate.cpp"
	"${UPSTREAM}/gdal/port/cplgetsymbol.cpp"
	"${UPSTREAM}/gdal/port/cplkeywordparser.cpp"
	"${UPSTREAM}/gdal/port/cplstring.cpp"
	"${UPSTREAM}/gdal/port/cplstringlist.cpp"
)

add_library(gdal_osmand STATIC
	${gdal_alg}
	${gdal_apps}
	${gdal_apps_common}
	${gdal_frmts}
	${gdal_frmts_derived}
	${gdal_frmts_gtiff}
	${gdal_frmts_gtiff_libtiff}
	${gdal_frmts_gtiff_libgeotiff}
	${gdal_frmts_hfa}
	${gdal_frmts_kml}
	${gdal_frmts_mem}
	${gdal_frmts_png}
	${gdal_frmts_vrt}
	${gdal_gcore}
	${gdal_gcore_mdreader}
	${gdal_ogr}
	${gdal_ogr_ogrfs_frmts}
	${gdal_ogr_ogrfs_frmts_generic}
	${gdal_ogr_ogrfs_frmts_geojson}
	${gdal_ogr_ogrfs_frmts_geojson_libjson}
	${gdal_ogr_ogrfs_frmts_kml}
	${gdal_ogr_ogrfs_frmts_mem}
	${gdal_ogr_ogrfs_frmts_mitab}
	${gdal_ogr_ogrfs_frmts_vrt}
	${gdal_ogr_ogrfs_frmts_shape}
	${gdal_port}
	${target_specific_sources}
)

target_compile_definitions(gdal_osmand
	PRIVATE
		-DGDAL_COMPILATION
		-DHOST_FILLORDER=${HOST_FILLORDER}
		-DHAVE_LIBZ=1
		-DHAVE_EXPAT=1
		-DFRMT_derived
		-DFRMT_gtiff
		-DFRMT_hfa
		-DFRMT_kml
		-DFRMT_mem
		-DFRMT_png
		-DFRMT_vrt
		-DSHAPE_ENABLED=1
		${target_specific_private_definitions}
	PUBLIC
		-DCPL_DLL=
		-DCPL_INTERNAL=
		-DCPL_UNSTABLE_API=
)
target_include_directories(gdal_osmand
	PUBLIC
		"${ROOT}"
		"${CMAKE_CURRENT_BINARY_DIR}"
		"${UPSTREAM}/gdal/alg"
		"${UPSTREAM}/gdal/alg/marching_squares"
		"${UPSTREAM}/gdal/apps"
		"${UPSTREAM}/gdal/frmts"
		"${UPSTREAM}/gdal/frmts/gtiff"
		"${UPSTREAM}/gdal/frmts/gtiff/libtiff"
		"${UPSTREAM}/gdal/frmts/gtiff/libgeotiff"
		"${UPSTREAM}/gdal/frmts/mem"
		"${UPSTREAM}/gdal/frmts/png"
		"${UPSTREAM}/gdal/frmts/png/libpng"
		"${UPSTREAM}/gdal/frmts/vrt"
		"${UPSTREAM}/gdal/gcore"
		"${UPSTREAM}/gdal/gcore/mdreader"
		"${UPSTREAM}/gdal/ogr"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/generic"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/geojson"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/geojson/libjson"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/mem"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/vrt"
		"${UPSTREAM}/gdal/ogr/ogrsf_frmts/shape"
		"${UPSTREAM}/gdal/port"
)
target_link_libraries(gdal_osmand
	LINK_PUBLIC
		z_osmand
		png_osmand
		jpeg_osmand
		expat_osmand
		proj_osmand
		${target_specific_public_libraries}
)
