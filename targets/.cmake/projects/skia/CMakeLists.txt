project(skia_osmand)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include("../../common.cmake")

set(ROOT "${OSMAND_ROOT}/externals/skia")
set(UPSTREAM "${ROOT}/upstream.patched")
include_directories(AFTER SYSTEM
	"${UPSTREAM}"
	"${UPSTREAM}/src/sksl"
	"${UPSTREAM}/third_party/externals/expat/lib"
	"${UPSTREAM}/third_party/externals/freetype/include"
	"${UPSTREAM}/third_party/externals/freetype/src"
	"${UPSTREAM}/third_party/externals/libjpeg-turbo"
	"${UPSTREAM}/third_party/libpng"
	"${UPSTREAM}/third_party/externals/libpng"
	"${UPSTREAM}/third_party/externals/lua"
	"${UPSTREAM}/include/third_party/skcms"
)
add_definitions(-DSK_RELEASE -DSK_IGNORE_ETC1_SUPPORT -DSK_HAS_JPEG_LIBRARY -DSK_HAS_PNG_LIBRARY -DSK_SUPPORT_GPU=0)
set(target_specific_sources
	"${UPSTREAM}/src/opts/SkBitmapProcState_opts.h"
	"${UPSTREAM}/src/opts/SkBlitMask_opts.h"
	"${UPSTREAM}/src/opts/SkBlitRow_opts.h"
)

set(skia_opts_sse2
  "${UPSTREAM}/src/opts/Sk4px_SSE2.h"
)

set(skia_opts_ssse3 
  "${UPSTREAM}/src/opts/SkOpts_ssse3.cpp"
)

set(skia_opts_sse41 "${UPSTREAM}/src/opts/SkOpts_sse41.cpp")
set(skia_opts_sse42 "${UPSTREAM}/src/opts/SkOpts_sse42.cpp")
set(skia_opts_avx "${UPSTREAM}/src/opts/SkOpts_avx.cpp")
set(skia_opts_hsw "${UPSTREAM}/src/opts/SkOpts_hsw.cpp")

if((CMAKE_TARGET_OS STREQUAL "darwin") OR (CMAKE_TARGET_OS STREQUAL "macosx"))
	file(GLOB src_utils_mac "${UPSTREAM}/src/utils/mac/*")
    set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkDebug_stdio.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_mac.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_posix.cpp"
		"${UPSTREAM}/src/ports/SkTLS_pthread.cpp"		

		${skia_opts_sse2}
		${skia_opts_ssse3}
		${skia_opts_sse41}
		${skia_opts_sse42}
		${skia_opts_avx}
		${skia_opts_hsw}

		${src_utils_mac}
    )
    add_definitions(-DSK_BUILD_FOR_MACOS -DSK_USE_POSIX_THREADS   -DSK_FONTHOST_USES_FONTMGR)
endif()

if(CMAKE_TARGET_OS STREQUAL "linux")

	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/ports/SkFontMgr_fontconfig.cpp"
		"${UPSTREAM}/src/ports/SkFontMgr_fontconfig_factory.cpp"
		"${UPSTREAM}/src/ports/SkDebug_stdio.cpp"
		"${UPSTREAM}/src/ports/SkFontConfigInterface_direct.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_FreeType_common.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_FreeType.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_posix.cpp"
		"${UPSTREAM}/src/ports/SkTLS_pthread.cpp"
		${skia_opts_sse2}
		${skia_opts_ssse3}
		${skia_opts_sse41}
		${skia_opts_sse42}
		${skia_opts_avx}
		${skia_opts_hsw}	
	)
	add_definitions(-DSK_BUILD_FOR_UNIX  -DSK_USE_POSIX_THREADS   -DSK_FONTHOST_USES_FONTMGR)
endif()

if(CMAKE_TARGET_OS STREQUAL "windows")
# windows is not tested after upgrade !
	file(GLOB src_utils_mac "${UPSTREAM}/src/utils/win/*")
	set(target_specific_sources "${target_specific_sources}"
		"${UPSTREAM}/src/fonts/SkFontMgr_indirect.cpp"
		"${UPSTREAM}/src/ports/SkDebug_win.cpp"
		"${UPSTREAM}/src/ports/SkOSFile_win.cpp"
		"${UPSTREAM}/src/ports/SkTime_win.cpp"
		"${UPSTREAM}/src/ports/SkTLS_win.cpp"
		"${UPSTREAM}/src/utils/SkThreadUtils_win.cpp"
		"${UPSTREAM}/src/utils/win/SkAutoCoInitialize.cpp"
		"${UPSTREAM}/src/utils/win/SkHRESULT.cpp"
		"${UPSTREAM}/src/utils/win/SkIStream.cpp"
		"${UPSTREAM}/src/ports/SkFontHost_win.cpp"
		"${UPSTREAM}/src/ports/SkFontMgr_default_gdi.cpp"
	)
	add_definitions(
		-DSK_BUILD_FOR_WIN32
		-DSK_FONTHOST_USES_FONTMGR
		-D_CRT_SECURE_NO_WARNINGS
		-DGR_GL_FUNCTION_TYPE=__stdcall)
endif()

if(CMAKE_COMPILER_FAMILY STREQUAL "gcc" OR CMAKE_COMPILER_FAMILY STREQUAL "clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-rtti -fno-exceptions")
endif()

if(CMAKE_TARGET_CPU_ARCH_FAMILY STREQUAL "arm64")
 set(target_specific_sources "${target_specific_sources}"
  "${UPSTREAM}/src/opts/SkOpts_crc32.cpp"
 )
endif()


file(GLOB skia_core_files
	${UPSTREAM}/src/core/*.h
	${UPSTREAM}/src/core/*.cpp
	${UPSTREAM}/include/core/*.h
	${UPSTREAM}/src/images/*.h
	${UPSTREAM}/src/images/*.cpp
	${UPSTREAM}/include/private/*.h
	${UPSTREAM}/src/utils/*.h
	${UPSTREAM}/src/utils/*.cpp
	${UPSTREAM}/src/shaders/*.h
	${UPSTREAM}/src/shaders/*.cpp
	${UPSTREAM}/src/shaders/gradients/*.cpp
	${UPSTREAM}/src/pathops/*.cpp
	${UPSTREAM}/src/effects/*.cpp
	${UPSTREAM}/src/effects/imagefilters/*.cpp
	${UPSTREAM}/src/sksl/*.cpp
	${UPSTREAM}/src/sksl/ir/*.cpp
)

set(skia_core
	${skia_core_files}
	"${UPSTREAM}/include/ports/SkCFObject.h"
	"${UPSTREAM}/src/ports/SkOSFile_stdio.cpp"
	"${UPSTREAM}/src/ports/SkMemory_malloc.cpp"
	"${UPSTREAM}/src/image/SkSurface.cpp"
	"${UPSTREAM}/src/image/SkImage.cpp"
	"${UPSTREAM}/src/image/SkSurface_Raster.cpp"
	"${UPSTREAM}/src/image/SkImage_Raster.cpp"
	"${UPSTREAM}/src/ports/SkGlobalInitialization_default.cpp"
	"${UPSTREAM}/src/image/SkImage_Lazy.cpp"
	"${UPSTREAM}/src/ports/SkImageGenerator_skia.cpp"
	"${UPSTREAM}/src/sfnt/SkOTUtils.cpp"
	"${UPSTREAM}/src/sfnt/SkOTTable_name.cpp"
	"${UPSTREAM}/src/codec/SkCodec.cpp"
	"${UPSTREAM}/src/codec/SkCodecImageGenerator.cpp"
	"${UPSTREAM}/src/codec/SkBmpCodec.cpp"
	"${UPSTREAM}/src/codec/SkWbmpCodec.cpp"
	"${UPSTREAM}/src/codec/SkSampler.cpp"
	"${UPSTREAM}/src/codec/SkPngCodec.cpp"
	"${UPSTREAM}/src/codec/SkJpegCodec.cpp"
	"${UPSTREAM}/src/codec/SkIcoCodec.cpp"
	"${UPSTREAM}/src/codec/SkJpegDecoderMgr.cpp"
	"${UPSTREAM}/src/codec/SkBmpMaskCodec.cpp"
	"${UPSTREAM}/src/codec/SkBmpRLECodec.cpp"
	"${UPSTREAM}/src/codec/SkBmpStandardCodec.cpp"
	"${UPSTREAM}/src/codec/SkColorTable.cpp"
	"${UPSTREAM}/src/codec/SkEncodedInfo.cpp"
	"${UPSTREAM}/src/codec/SkMasks.cpp"
	"${UPSTREAM}/src/codec/SkParseEncodedOrigin.cpp"
	"${UPSTREAM}/src/codec/SkSwizzler.cpp"
	"${UPSTREAM}/src/codec/SkBmpBaseCodec.cpp"
	"${UPSTREAM}/src/codec/SkMaskSwizzler.cpp"
	"${UPSTREAM}/src/codec/SkJpegUtility.cpp"
	"${UPSTREAM}/third_party/skcms/skcms.cc"
)

add_library(skia_osmand STATIC
  	${skia_core}
	${target_specific_sources}
)

find_package(Freetype REQUIRED)
target_link_libraries(skia_osmand LINK_PUBLIC
	expat_osmand
	png_osmand
	jpeg_osmand
	${FREETYPE_LIBRARIES}
)
target_include_directories(skia_osmand PRIVATE ${FREETYPE_INCLUDE_DIRS})
	
if(CMAKE_TARGET_OS STREQUAL "linux")
	find_library(FONTCONFIG_LIBRARY fontconfig)
	target_link_libraries(skia_osmand LINK_PUBLIC
		pthread
		dl
		${FONTCONFIG_LIBRARY}
	)
endif()

if((CMAKE_TARGET_OS STREQUAL "darwin") OR (CMAKE_TARGET_OS STREQUAL "macosx"))
   	find_library(CORE_FOUNDATION_LIBRARY CoreFoundation)
        find_library(CORE_TEXT_LIBRARY CoreText)
        find_library(CORE_GRAPHICS_LIBRARY CoreGraphics)
        find_library(FONTCONFIG_LIBRARY fontconfig)
        target_link_libraries(skia_osmand LINK_PUBLIC
                pthread
                ${FONTCONFIG_LIBRARY}
                "${CORE_FOUNDATION_LIBRARY}/CoreFoundation.tbd"
                "${CORE_TEXT_LIBRARY}/CoreText.tbd"
                "${CORE_GRAPHICS_LIBRARY}/CoreGraphics.tbd"

        )
endif()

if(CMAKE_TARGET_OS STREQUAL "windows")
	target_link_libraries(skia_osmand LINK_PUBLIC
		usp10
	)
endif()
