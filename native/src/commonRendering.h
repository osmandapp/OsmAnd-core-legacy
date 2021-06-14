#ifndef _OSMAND_COMMON_RENDERING_H
#define _OSMAND_COMMON_RENDERING_H

#include <SkBitmap.h>
#include <SkPaint.h>
#include <SkStream.h>
#include <SkTypeface.h>
#include <SkFont.h>
#include <SkFontMgr.h>
#include <hb-ot.h>
#include <iostream>

#include "CommonCollections.h"
#include "SkBlurDrawLooper.h"
#include "commonOsmAndCore.h"

// Better don't do this
using namespace std;

struct FontEntry {
	bool bold;
	bool italic;
	struct HBFDel { void operator()(hb_face_t *f) { hb_face_destroy(f); }};
	sk_sp<SkTypeface> fSkiaTypeface = nullptr;
	std::unique_ptr<hb_face_t, HBFDel> fHarfBuzzFace;
	string fontName;

	FontEntry() : FontEntry("/Users/macmini/Downloads/05_NotoSans-Regular.ttf", 0) {}

	FontEntry(const char *path, int index) {
		// fairly portable mmap impl
		auto data = SkData::MakeFromFileName(path);
		assert(data);
		if (!data) {
			return;
		}
		fSkiaTypeface = SkTypeface::MakeFromFile(path);
		assert(fSkiaTypeface);
		if (!fSkiaTypeface) {
			return;
		}
		auto destroy = [](void *d) { static_cast<SkData *>(d)->unref(); };
		const char *bytes = (const char *)data->data();
		unsigned int size = (unsigned int)data->size();
		hb_blob_t *blob = hb_blob_create(bytes,
										 size,
										 HB_MEMORY_MODE_READONLY,
										 data.release(),
										 destroy);
		assert(blob);
		hb_blob_make_immutable(blob);
		hb_face_t *face = hb_face_create(blob, (unsigned)index);
		hb_blob_destroy(blob);
		assert(face);
		if (!face) {
			fSkiaTypeface.reset();
			return;
		}
		hb_face_set_index(face, (unsigned)index);
		hb_face_set_upem(face, fSkiaTypeface->getUnitsPerEm());
		fHarfBuzzFace.reset(face);
	}
};

class FontRegistry {
	std::vector<FontEntry*> cache;

   public:
	const sk_sp<SkTypeface> registerStream(const char* data, uint32_t length, string fontName, bool bold, bool italic);
	void updateFontEntry(FontEntry* fontEntry, std::string text, bool bold, bool italic);
	void drawHbText(SkCanvas* cv, std::string textS, FontEntry* fontEntry, SkPaint& paint, SkFont& font, float centerX, float centerY);
	void drawHbTextOnPath(SkCanvas* canvas, std::string textS, SkPath& path, FontEntry* fontEntry, SkFont& font, SkPaint& paint, float h_offset, float v_offset);
	void drawSkiaTextOnPath(SkCanvas* canvas, std::string textS, SkPath& path, FontEntry* fontEntry, SkFont& font, SkPaint& paint, float h_offset, float v_offset);
};

extern FontRegistry globalFontRegistry;

struct IconDrawInfo {
	SkBitmap* bmp_1;
	SkBitmap* bmp;
	std::string bmpId;
	SkBitmap* bmp2;
	SkBitmap* bmp3;
	SkBitmap* bmp4;
	SkBitmap* bmp5;
	SkBitmap* shield;
	MapDataObject object;
	float x;
	float y;
	bool visible;
	float shiftPx;
	float shiftPy;
	int order;
	int secondOrder;
	float iconSize;
	float intersectionMargin;
	float intersectionSizeFactor;
	SkRect bbox;

	IconDrawInfo(MapDataObject* mo);
};

struct TextDrawInfo {
	TextDrawInfo(std::string, MapDataObject* mo);
	~TextDrawInfo();

	std::string text;
	MapDataObject object;
	SHARED_PTR<IconDrawInfo> icon;
	bool visible;
	bool combined;

	SkRect bounds;
	SkRect textBounds;
	float centerX;
	float centerY;

	float textSize;
	float minDistance;
	int textColor;
	int textShadow;
	int textShadowColor;
	uint32_t textWrap;
	bool bold;
	bool italic;
	std::string shieldRes;
	std::string shieldResIcon;
	int textOrder;
	int secondOrder;

	bool drawOnPath;
	SkPath* path;
	float pathRotate;
	float vOffset;
	float hOffset;
	float intersectionMargin;
	float intersectionSizeFactor;
};

static const int TILE_SIZE = 256;
struct RenderingContextResults;
struct RenderingContext {
   private:
	// parameters
	std::string preferredLocale;
	bool transliterate;
	float density;
	float screenDensityRatio;
	float textScale;

	double leftX;
	double topY;
	int width;
	int height;
	int defaultColor;

	int zoom;
	float rotate;
	// int shadowRenderingMode = 0; // no shadow (minumum CPU)
	// int shadowRenderingMode = 1; // classic shadow (the implementaton in master)
	// int shadowRenderingMode = 2; // blur shadow (most CPU, but still reasonable)
	// int shadowRenderingMode = 3; solid border (CPU use like classic version or even smaller)
	int shadowRenderingMode;
	int shadowRenderingColor;
	int waterwayArrows;
	int noHighwayOnewayArrows;
	string defaultIconsDir;

   public:
	// debug purpose
	int pointCount;
	int pointInsideCount;
	int visible;
	int allObjects;
	int lastRenderedKey;
	OsmAnd::ElapsedTimer textRendering;
	OsmAnd::ElapsedTimer nativeOperations;

	std::vector<SkPaint> oneWayPaints;
	std::vector<SkPaint> reverseWayPaints;

	// because they used in 3rd party functions
   public:
	// calculated
	double tileDivisor;
	float cosRotateTileSize;
	float sinRotateTileSize;

	// use to calculate points
	float calcX;
	float calcY;

	std::vector<SHARED_PTR<TextDrawInfo>> textToDraw;
	std::vector<SHARED_PTR<IconDrawInfo>> iconsToDraw;
	quad_tree<SHARED_PTR<TextDrawInfo>> textIntersect;
	quad_tree<SHARED_PTR<IconDrawInfo>> iconsIntersect;

	// not expect any shadow
	int shadowLevelMin;
	int shadowLevelMax;
	int polygonMinSizeToDisplay;
	int roadDensityZoomTile;
	int roadsDensityLimitPerTile;

   public:
	RenderingContext()
		: preferredLocale(""), transliterate(false), density(1), screenDensityRatio(1),
		  textScale(1),	 // leftX, topY, width, height
		  defaultColor(0xfff1eee8), zoom(15), rotate(0), shadowRenderingMode(2), shadowRenderingColor(0xff969696),
		  noHighwayOnewayArrows(0),	 // defaultIconsDir
		  pointCount(0), pointInsideCount(0), visible(0), allObjects(0), lastRenderedKey(0),
		  // textRendering, nativeOperations, oneWayPaints, reverseWayPaints
		  // tileDivisor, cosRotateTileSize, sinRotateTileSize,  calcX, calcY
		  // textToDraw, iconsToDraw,
		  textIntersect(), iconsIntersect(), shadowLevelMin(256), shadowLevelMax(0), polygonMinSizeToDisplay(0),
		  roadDensityZoomTile(0), roadsDensityLimitPerTile(0) {
	}
	virtual ~RenderingContext();

	virtual bool interrupted();
	virtual SkBitmap* getCachedBitmap(const std::string& bitmapResource);
	virtual std::string getTranslatedString(const std::string& src);
	virtual std::string getReshapedString(const std::string& src);

	void setDefaultIconsDir(string path) {
		defaultIconsDir = path;
	}

	void setZoom(int z) {
		this->zoom = z;
		this->tileDivisor = (1 << (31 - z));
	}

	void setTileDivisor(double tileDivisor) {
		this->tileDivisor = tileDivisor;
	}

	void setDefaultColor(int z) {
		this->defaultColor = z;
	}

	void setRotate(float rot) {
		this->rotate = rot;
		this->cosRotateTileSize = cos(toRadians(rot)) * TILE_SIZE;
		this->sinRotateTileSize = sin(toRadians(rot)) * TILE_SIZE;
	}

	void setLocation(double leftX, double topY) {
		this->leftX = leftX;
		this->topY = topY;
	}

	void setDimension(int width, int height) {
		this->width = width;
		this->height = height;
	}

	inline int getShadowRenderingMode() {
		return shadowRenderingMode;
	}

	int getShadowRenderingColor() {
		return shadowRenderingColor;
	}

	void setShadowRenderingColor(int color) {
		shadowRenderingColor = color;
	}

	void setWaterwayArrows(int arrows) {
		waterwayArrows = arrows;
	}

	int getWaterwayArrows() {
		return waterwayArrows;
	}
	void setNoHighwayOnewayArrows(int noarrows) {
		noHighwayOnewayArrows = noarrows;
	}
	int getNoHighwayOnewayArrows() {
		return noHighwayOnewayArrows;
	}
	inline int getWidth() {
		return width;
	}

	inline int getDefaultColor() {
		return defaultColor;
	}

	inline int getHeight() {
		return height;
	}

	inline int getZoom() {
		return zoom;
	}

	inline double getLeft() {
		return leftX;
	}

	inline double getTop() {
		return topY;
	}

	void setShadowRenderingMode(int mode) {
		this->shadowRenderingMode = mode;
	}

	void setDensityScale(float val) {
		density = val;
	}

	void setScreenDensityRatio(float v) {
		screenDensityRatio = v;
	}

	float getScreenDensityRatio() {
		return screenDensityRatio;
	}

	float getDensityValue(float val) {
		return val * density;
	}

	float getDensityValue(float val, int pxValues) {
		return val * density + pxValues;
	}

	void setTextScale(float v) {
		textScale = v;
	}

	float getTextScale() {
		return textScale;
	}

	void setPreferredLocale(std::string pref) {
		this->preferredLocale = pref;
	}

	std::string getPreferredLocale() {
		return this->preferredLocale;
	}

	void setTransliterate(bool pref) {
		this->transliterate = pref;
	}

	bool getTransliterate() {
		return this->transliterate;
	}

	friend struct RenderingContextResults;
};

struct RenderingContextResults {
   public:
	int zoom;
	float density;
	float screenDensityRatio;
	float textScale;

	double leftX;
	double topY;
	int width;
	int height;

	quad_tree<SHARED_PTR<TextDrawInfo>> textIntersect;
	quad_tree<SHARED_PTR<IconDrawInfo>> iconsIntersect;

	RenderingContextResults(RenderingContext* context);
};

string prepareIconValue(MapDataObject& object, string qval);

SkBitmap* getCachedBitmap(RenderingContext* rc, const std::string& bitmapResource);
void purgeCachedBitmaps();

bool GetResourceAsBitmap(const char* resourcePath, SkBitmap* dst);
SkStreamAsset* GetResourceAsStream(const char* resourcePath);
sk_sp<SkData> GetResourceAsData(const char* resourcePath);
sk_sp<SkTypeface> MakeResourceAsTypeface(const char* resourcePath);

#endif /*_OSMAND_COMMON_RENDERING_H*/
