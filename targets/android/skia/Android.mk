LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

PROJECT_ROOT_RELATIVE := ../../../../platforms/android/OsmAnd
OSMAND_SKIA_ROOT_RELATIVE := ../../../externals/skia
OSMAND_SKIA_ROOT := $(LOCAL_PATH)/$(OSMAND_SKIA_ROOT_RELATIVE)
OSMAND_SKIA_RELATIVE := ../../../externals/skia/upstream.patched
OSMAND_SKIA := $(LOCAL_PATH)/$(OSMAND_SKIA_RELATIVE)

LOCAL_ARM_NEON := true

LOCAL_C_INCLUDES += \
	${OSMAND_SKIA} \
	${OSMAND_SKIA}/src/core \
	${OSMAND_SKIA}/src/codec \
	${OSMAND_SKIA}/src/effects \
	${OSMAND_SKIA}/src/image \
	${OSMAND_SKIA}/src/images \
	${OSMAND_SKIA}/src/lazy \
	${OSMAND_SKIA}/src/ports \
	${OSMAND_SKIA}/src/opts \
	${OSMAND_SKIA}/src/pathops \
	${OSMAND_SKIA}/src/shaders \
	${OSMAND_SKIA}/src/sksl \
	${OSMAND_SKIA}/src/utils \
	${OSMAND_SKIA}/include/core \
	${OSMAND_SKIA}/include/private \
	${OSMAND_SKIA}/third_party/externals/expat/lib \
	${OSMAND_SKIA}/third_party/externals/freetype/include \
	${OSMAND_SKIA}/third_party/externals/libjpeg-turbo \
    ${OSMAND_SKIA}/third_party/libpng \
	${OSMAND_SKIA}/third_party/externals/libpng \
	${OSMAND_SKIA}/third_party/externals/lua \
	${OSMAND_SKIA}/include/third_party/skcms

#$(OSMAND_SKIA_RELATIVE)/third_party/skcms/skcms.cc
#$(OSMAND_SKIA_RELATIVE)/src/c/sk_surface.cpp
# core
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkAAClip.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkATrace.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkAlphaRuns.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkAnalyticEdge.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkAnnotation.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkArenaAlloc.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkAutoPixmapStorage.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBBHFactory.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBigPicture.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmap.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmapCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmapController.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmapDevice.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmapProcState.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBitmapProcState_matrixProcs.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlendMode.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitRow_D32.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitter_A8.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitter_ARGB32.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitter_RGB565.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlitter_Sprite.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlurMF.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBlurMask.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkBuffer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCachedData.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCanvas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCanvasPriv.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkClipStack.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkClipStackDevice.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkColor.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkColorFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkColorFilter_Matrix.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkColorSpace.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkColorSpaceXformSteps.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCompressedDataUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkContourMeasure.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkConvertPixels.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCpu.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCubicClipper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkCubicMap.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkData.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDataTable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDebug.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDeferredDisplayList.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDeferredDisplayListRecorder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDeque.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDescriptor.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDevice.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDistanceFieldGen.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDocument.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDraw.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDrawLooper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDrawShadowInfo.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDraw_atlas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDraw_text.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDraw_vertices.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkDrawable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkEdge.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkEdgeBuilder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkEdgeClipper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkExecutor.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFlattenable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFont.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFontDescriptor.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFontLCDConfig.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFontMgr.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFontStream.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkFont_serial.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGaussFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGeometry.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGlobalInitialization_core.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGlyph.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGlyphBuffer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGlyphRun.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGlyphRunPainter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGpuBlurUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkGraphics.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkHalf.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkICC.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkImageFilterCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkImageFilterTypes.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkImageGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkImageInfo.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkLatticeIter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkLineClipper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkLocalMatrixImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkM44.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMD5.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMalloc.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMallocPixelRef.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMask.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMaskBlurFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMaskCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMaskFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMaskGamma.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMatrix.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMatrix44.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMatrixImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMiniRecorder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkMipMap.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkModeColorFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkNormalFlatSource.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkNormalMapSource.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkNormalSource.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkOpts.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkOverdrawCanvas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPaint.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPaintPriv.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPathMeasure.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPathRef.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPath_serial.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPicture.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPictureData.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPictureFlat.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPictureImageGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPicturePlayback.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPictureRecord.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPictureRecorder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPixelRef.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPixmap.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPoint.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPoint3.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPromiseImageTexture.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkPtrRecorder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkQuadClipper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRRect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRTree.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRWBuffer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRasterClip.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRasterPipeline.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRasterPipelineBlitter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkReadBuffer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecord.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecordDraw.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecordOpts.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecordedDrawable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecorder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRecords.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRegion.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRegion_path.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRemoteGlyphCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkResourceCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkRuntimeEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScalar.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScalerCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScalerContext.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan_AAAPath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan_AntiPath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan_Antihair.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan_Hairline.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkScan_Path.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSemaphore.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSharedMutex.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSpecialImage.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSpecialSurface.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSpinlock.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSpriteBlitter_ARGB32.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSpriteBlitter_RGB565.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStream.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStrikeCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStrikeForGPU.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStrikeSpec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkString.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStringUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStroke.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStrokeRec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkStrokerPriv.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSurfaceCharacterization.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkSwizzle.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTLS.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTSearch.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTaskGroup.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTextBlob.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTextBlobTrace.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkThreadID.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTime.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTypeface.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTypefaceCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkTypeface_remote.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkUnPreMultiply.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkUtilsArm.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkVM.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkVMBlitter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkVertState.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkVertices.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkWriteBuffer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkWriter32.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkXfermode.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkXfermodeInterpretation.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkYUVASizeInfo.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkYUVMath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/core/SkYUVPlanesCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/images/SkImageEncoder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/images/SkJPEGWriteUtility.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/images/SkJpegEncoder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/images/SkPngEncoder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/images/SkWebpEncoder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkAddIntersections.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkDConicLineIntersection.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkDCubicLineIntersection.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkDCubicToQuads.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkDLineIntersection.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkDQuadLineIntersection.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkIntersections.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpAngle.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpBuilder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpCoincidence.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpContour.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpCubicHull.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpEdgeBuilder.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpSegment.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkOpSpan.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsAsWinding.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsCommon.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsConic.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsCubic.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsCurve.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsDebug.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsLine.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsOp.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsQuad.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsRect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsSimplify.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsTSect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsTightBounds.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsTypes.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathOpsWinding.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkPathWriter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/pathops/SkReduceOrder.cpp 

# utils
# $(OSMAND_SKIA_RELATIVE)/src/utils/SkLua.cpp
# $(OSMAND_SKIA_RELATIVE)/src/utils/SkLuaCanvas.cpp
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkAnimCodecPlayer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkBase64.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkCamera.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkCanvasStack.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkCanvasStateUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkCharToGlyphCache.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkClipStackUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkDashPath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkEventTracer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkFloatToDecimal.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkFrontBufferedStream.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkInterpolator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkJSON.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkJSONWriter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkMatrix22.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkMultiPictureDocument.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkNWayCanvas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkNullCanvas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkOSPath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkPaintFilterCanvas.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkParse.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkParseColor.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkParsePath.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkPatchUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkPolyUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkShadowTessellator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkShadowUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkShaperJSONWriter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkTextUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkThreadUtils_pthread.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkThreadUtils_win.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkUTF.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/utils/SkWhitelistTypefaces.cpp

#shaders
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkBitmapProcShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkColorFilterShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkColorShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkComposeShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkImageShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkLightingShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkLights.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkLocalMatrixShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkPerlinNoiseShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkPictureShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/SkShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/Sk4fGradientBase.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/Sk4fLinearGradient.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/SkGradientShader.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/SkLinearGradient.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/SkRadialGradient.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/SkSweepGradient.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/shaders/gradients/SkTwoPointConicalGradient.cpp

# effects
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/effects/Sk1DPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/Sk2DPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkColorMatrix.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkColorMatrixFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkCornerPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkDashPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkDiscretePathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkEmbossMask.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkEmbossMaskFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkHighContrastFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkLayerDrawLooper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkLumaColorFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkOpPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkOverdrawColorFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkPackBits.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkShaderMaskFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkTableColorFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkTableMaskFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/SkTrimPathEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkAlphaThresholdFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkArithmeticImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkBlurImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkColorFilterImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkComposeImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkDisplacementMapEffect.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkDropShadowImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkImageFilters.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkImageSource.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkLightingImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkMagnifierImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkMatrixConvolutionImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkMergeImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkMorphologyImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkOffsetImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkPaintImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkPictureImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkTileImageFilter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/effects/imagefilters/SkXfermodeImageFilter.cpp \

#	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLMain.cpp
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLASTNode.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLByteCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLCFGGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLCPPCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLCPPUniformCTypes.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLCompiler.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLGLSLCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLHCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLIRGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLLexer.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLMetalCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLOutputStream.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLParser.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLPipelineStageCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLSPIRVCodeGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLSPIRVtoHLSL.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLSectionAndParameterHelper.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLString.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/SkSLUtil.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/ir/SkSLSetting.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/ir/SkSLSymbolTable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/ir/SkSLType.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sksl/ir/SkSLVariableReference.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkMemory_malloc.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/image/SkSurface.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/image/SkImage.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/image/SkSurface_Raster.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/image/SkImage_Raster.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkGlobalInitialization_default.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/image/SkImage_Lazy.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkImageGenerator_skia.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sfnt/SkOTUtils.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/sfnt/SkOTTable_name.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkCodecImageGenerator.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkBmpCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkWbmpCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkSampler.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkPngCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkJpegCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkIcoCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkJpegDecoderMgr.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkBmpMaskCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkBmpRLECodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkBmpStandardCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkColorTable.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkEncodedInfo.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkMasks.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkParseEncodedOrigin.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkSwizzler.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkBmpBaseCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkMaskSwizzler.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkJpegUtility.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkAndroidCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkAndroidCodecAdapter.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/codec/SkSampledCodec.cpp \
	$(OSMAND_SKIA_RELATIVE)/third_party/skcms/skcms.cc \
	$(OSMAND_SKIA_RELATIVE)/src/lazy/SkDiscardableMemoryPool.cpp
	
#$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontHost_FreeType_common.cpp
LOCAL_SRC_FILES += \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkOSFile_stdio.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkDebug_android.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkDiscardableMemory_none.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontHost_FreeType_common.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontHost_FreeType.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontMgr_android.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontMgr_android_factory.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkFontMgr_android_parser.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkOSFile_posix.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/ports/SkTLS_pthread.cpp \
	$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_crc32.cpp
	
LOCAL_CFLAGS += \
	-DSK_BUILD_FOR_ANDROID \
	-DANDROID_LARGE_MEMORY_DEVICE \
	-DSK_USE_POSIX_THREADS \
	-DSK_IGNORE_ETC1_SUPPORT \
	-DSKRELEASE \
	-DSK_HAS_JPEG_LIBRARY \
	-DSK_HAS_PNG_LIBRARY \
	-DSK_SUPPORT_GPU=0

LOCAL_CPPFLAGS := \
	-fno-rtti \
	-fno-exceptions
	
LOCAL_MODULE := osmand_skia

ifneq ($(OSMAND_USE_PREBUILT),true)

LOCAL_ARM_MODE := arm

ifeq ($(TARGET_ARCH),arm64)
	LOCAL_ARM_NEON := true
	LOCAL_CFLAGS += \
		-DSK_ARM_HAS_OPTIONAL_NEON

else ifeq ($(TARGET_ARCH),arm)
	LOCAL_ARM_NEON := true
	LOCAL_CFLAGS += \
		-DSK_ARM_HAS_OPTIONAL_NEON

else

	LOCAL_SRC_FILES += \
		$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_sse41.cpp \
		$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_sse42.cpp \
		$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_ssse3.cpp \
		$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_avx.cpp \
		$(OSMAND_SKIA_RELATIVE)/src/opts/SkOpts_hsw.cpp 

		
endif

LOCAL_STATIC_LIBRARIES += \
	osmand_jpeg \
	osmand_ft2 \
	osmand_png \
	osmand_expat \
	osmand_lua \
	cpufeatures

#LOCAL_LDLIBS += -lz -llog

include $(BUILD_STATIC_LIBRARY)

$(call import-module,android/cpufeatures)

else
	LOCAL_SRC_FILES := \
		$(PROJECT_ROOT_RELATIVE)/libs/$(TARGET_ARCH_ABI)/lib$(LOCAL_MODULE).a
	include $(PREBUILT_STATIC_LIBRARY)
endif
