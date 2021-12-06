#include <SkCanvas.h>
#include <SkFilterQuality.h>
#include <SkPaint.h>
#include <SkPath.h>
#include <SkTypeface.h>
#include <SkTypes.h>
#include <SkUtils.h>
#include <SkFont.h>
#include <SkFontMetrics.h>
#include <SkPathMeasure.h>
#include <SkRSXform.h>
#include <SkFontPriv.h>
#include <SkAutoMalloc.h>
#include <SkTextBlob.h>
#include <hb-ot.h>
#include <math.h>
#include <time.h>

#include <algorithm>
#include <set>
#include <vector>

#include "Logging.h"
#include "renderRules.h"
//#include "utf8.cpp"
#include "utf8/unchecked.h"

FontRegistry globalFontRegistry;
const double HARFBUZZ_FONT_SIZE_SCALE = 64.0f;

struct DebugTextInfo {
	bool debugTextDisplayBBox;
	bool debugTextDisplayShieldBBox;
	bool debugTextDoNotFindIntersections;
	bool debugTextDoNotFindIntersectionsSameName;
	bool debugTextDisplayShortRoadNames;
	DebugTextInfo(RenderingRuleSearchRequest* req) {
		req->clearState();
		// req->setIntFilter(req->props()->R_MINZOOM, rc->getZoom());
		if (req->searchRenderingAttribute("debugTextDisplayBBox")) {
			debugTextDisplayBBox = req->getBoolPropertyValue(req->props()->R_ATTR_BOOL_VALUE);
		} else {
			debugTextDisplayBBox = false;
		}
		req->clearState();
		if (req->searchRenderingAttribute("debugTextDisplayShieldBBox")) {
			debugTextDisplayShieldBBox = req->getBoolPropertyValue(req->props()->R_ATTR_BOOL_VALUE);
		} else {
			debugTextDisplayShieldBBox = false;
		}
		req->clearState();
		if (req->searchRenderingAttribute("debugTextDisplayShortRoadNames")) {
			debugTextDisplayShortRoadNames = req->getBoolPropertyValue(req->props()->R_ATTR_BOOL_VALUE);
		} else {
			debugTextDisplayShortRoadNames = false;
		}
		req->clearState();
		if (req->searchRenderingAttribute("debugTextDoNotFindIntersections")) {
			debugTextDoNotFindIntersections = req->getBoolPropertyValue(req->props()->R_ATTR_BOOL_VALUE);
		} else {
			debugTextDoNotFindIntersections = false;
		}
		req->clearState();
		if (req->searchRenderingAttribute("debugTextDoNotFindIntersectionsSameName")) {
			debugTextDoNotFindIntersectionsSameName = req->getBoolPropertyValue(req->props()->R_ATTR_BOOL_VALUE);
		} else {
			debugTextDoNotFindIntersectionsSameName = false;
		}
	}
};

// Check if all of the specified text has a corresponding non-zero glyph ID
bool containsText(SkTypeface* typeface, std::string textString) {
	const char *text = textString.c_str();
	size_t byteLength = textString.length();
	const char *stop = text + byteLength;
	while (text < stop) {
		if (0 == typeface->unicharToGlyph(SkUTF8_NextUnichar(&text))) {
			return false;
		}
	}
	return true;
}

void FontRegistry::registerFonts(const char* pathToFont, string fontName, bool bold,
													 bool italic) {
	OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Font path %s index %d", pathToFont, index);
	FontEntry* entry = new FontEntry(pathToFont, index);
	index++;
	if (!entry->fSkiaTypeface) {
		return;
	}
	entry->bold = bold;
	entry->italic = italic;
	entry->fontName = fontName;	
	cache.push_back(entry);
}


FontEntry* FontRegistry::updateFontEntry(std::string text, bool bold, bool italic) {
	if (cache.size() == 0) {
		OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Fonts are not registered. Set fonts by FontRegistry::registerFonts");
		FontEntry* entry = new FontEntry();//default system font
		cache.push_back(entry);
	}
	FontEntry* fontEntry = nullptr;
	for (uint i = 0; i < cache.size(); i++) {
		if (fontEntry != nullptr && (bold != cache[i]->bold || italic != cache[i]->italic)) {
			continue;
		}
		if (!containsText(cache[i]->fSkiaTypeface.get(), text)) {
			continue;
		}
		fontEntry = cache[i];
		// If this entry fully matches the request, stop search
		if (cache[i]->bold == bold && cache[i]->italic == italic) {
			break;
		}
	}

	if (fontEntry == nullptr) {
		fontEntry = cache[0];
	}
	return fontEntry;
}

inline float sqr(float a) {
	return a * a;
}

inline float absFloat(float a) {
	return a > 0 ? a : -a;
}

void fillTextProperties(RenderingContext* rc, SHARED_PTR<TextDrawInfo>& info, RenderingRuleSearchRequest* render,
						float cx, float cy) {
	info->centerX = cx;
	// used only for draw on path where centerY doesn't play role
	info->vOffset = getDensityValue(rc, render, render->props()->R_TEXT_DY) * rc->getTextScale();
	info->centerY = cy + info->vOffset;
	info->textColor = render->getIntPropertyValue(render->props()->R_TEXT_COLOR);
	if (info->textColor == 0) {
		info->textColor = 0xff000000;
	}
	info->textSize = getDensityValue(rc, render, render->props()->R_TEXT_SIZE) * rc->getTextScale();
	info->intersectionSizeFactor = render->getFloatPropertyValue(render->props()->R_INTERSECTION_SIZE_FACTOR, 1);
	info->intersectionMargin = getDensityValue(rc, render, render->props()->R_INTERSECTION_MARGIN);
	info->textShadow = getDensityValue(rc, render, render->props()->R_TEXT_HALO_RADIUS) * rc->getTextScale();
	info->textShadowColor = render->getIntPropertyValue(render->props()->R_TEXT_HALO_COLOR);
	if (info->textShadowColor == 0) {
		info->textShadowColor = 0xffffffff;
	}
	info->textWrap = render->getIntPropertyValue(render->props()->R_TEXT_WRAP_WIDTH);
	info->bold = render->getIntPropertyValue(render->props()->R_TEXT_BOLD, 0) > 0;
	info->italic = render->getIntPropertyValue(render->props()->R_TEXT_ITALIC, 0) > 0;
	info->minDistance = getDensityValue(rc, render, render->props()->R_TEXT_MIN_DISTANCE) * rc->getTextScale();
	if (info->minDistance == 0) {
		info->minDistance = rc->getDensityValue(150) * rc->getTextScale();
	}
	info->shieldRes = prepareIconValue(info->object, render->getStringPropertyValue(render->props()->R_TEXT_SHIELD));
	info->shieldResIcon = prepareIconValue(info->object, render->getStringPropertyValue(render->props()->R_ICON));
	info->textOrder = render->getIntPropertyValue(render->props()->R_TEXT_ORDER, 100);
}

bool isLetterOrDigit(char c) {
	return c != ' ';
}

void drawTextOnCanvas(RenderingContext* rc, SkCanvas* cv, const char* text, uint16_t len, float centerX, float centerY,
					  SkPaint& paintText, int textShadowColor, float textShadow, SkFont& skFontText, FontEntry* fontEntry) {
	std::string str(text, len);
	str = rc->getReshapedString(str);// bug with Arabic ligature here
	if (textShadow > 0) {
		int c = paintText.getColor();
		paintText.setStyle(SkPaint::kStroke_Style);
		paintText.setColor(textShadowColor);  // white
		paintText.setStrokeWidth(2 + textShadow);
		
		// ussual Skia draw text
		//cv->drawSimpleText(str.c_str(), str.length(), SkTextEncoding::kUTF8, centerX, centerY, skFontText, paintText);		
		
		// Harfbuzz draw
		globalFontRegistry.drawHbText(cv, str, fontEntry, paintText, skFontText, centerX, centerY);

		// reset
		paintText.setStrokeWidth(2);
		paintText.setStyle(SkPaint::kFill_Style);
		paintText.setColor(c);
	}
	globalFontRegistry.drawHbText(cv, str, fontEntry, paintText, skFontText, centerX, centerY);
	//cv->drawSimpleText(str.c_str(), str.length(), SkTextEncoding::kUTF8, centerX, centerY, skFontText, paintText);
}

int nextWord(uint8_t* s, uint* charRead) {
	uint8_t* init = s;
	while ((*s) != 0) {
		uint32_t tp = utf8::unchecked::next(s);
		(*charRead)++;
		if (tp == ' ' || tp == '\t') {
			return (s - init);
		}
	}
	return -1;
}

void drawWrappedText(RenderingContext* rc, SkCanvas* cv, SHARED_PTR<TextDrawInfo>& text, float textSize,
					 SkPaint& paintText, SkFont& skFontText, FontEntry* fontEntry) {
	if (text->textWrap == 0) {
		// set maximum for all text
		text->textWrap = 15;
	}
	if (text->text.length() > text->textWrap) {
		const char* c_str = text->text.c_str();

		int end = text->text.length();
		int line = 0;
		int pos = 0;
		int start = 0;
		while (start < end) {
			const char* p_str = c_str;
			uint charRead = 0;
			do {
				int lastSpace = nextWord((uint8_t*)p_str, &charRead);
				if (lastSpace == -1) {
					pos = end;
				} else {
					p_str += lastSpace;
					if (pos != start && charRead >= text->textWrap) {
						break;
					}
					pos += lastSpace;
				}
			} while (pos < end && charRead < text->textWrap);

			PROFILE_NATIVE_OPERATION(
				rc, drawTextOnCanvas(rc, cv, c_str, pos - start, text->centerX, text->centerY + line * (textSize + 2),
									 paintText, text->textShadowColor, text->textShadow, skFontText, fontEntry));
			c_str += (pos - start);
			start = pos;
			line++;
		}
	} else {
		PROFILE_NATIVE_OPERATION(
			rc, drawTextOnCanvas(rc, cv, text->text.c_str(), std::strlen(text->text.c_str()), text->centerX, text->centerY,
								 paintText, text->textShadowColor, text->textShadow, skFontText, fontEntry));
	}
}

bool calculatePathToRotate(RenderingContext* rc, SHARED_PTR<TextDrawInfo>& p, DebugTextInfo db) {
	if (p->path == NULL) {
		return true;
	}
	int len = p->path->countPoints();
	SkPoint* points = new SkPoint[len];
	p->path->getPoints(points, len);

	bool inverse = false;
	float roadLength = 0;
	bool prevInside = false;
	float visibleRoadLength = 0;
	float textw = p->bounds.width();
	int i;
	int startVisible = 0;
	std::vector<float> distances;
	distances.resize(roadLength, 0);

	float normalTextLen = 1.5 * textw;
	bool verySharpAngle = false;
	auto types = p->object.types;
	bool detectSharpAngle = false;
	for (uint i = 0; i < types.size(); i++) {
		if (types[i].first == "highway") {
			detectSharpAngle = true;
		}
	}
	for (i = 0; i < len; i++) {
		bool inside =
			points[i].fX >= 0 && points[i].fX <= rc->getWidth() && points[i].fY >= 0 && points[i].fY <= rc->getHeight();
		if (i > 0) {
			float d = sqrt((points[i].fX - points[i - 1].fX) * (points[i].fX - points[i - 1].fX) +
						   (points[i].fY - points[i - 1].fY) * (points[i].fY - points[i - 1].fY));
			if (i > 1) {
				float vx = (points[i].fX - points[i - 1].fX) / d;
				float vy = (points[i].fY - points[i - 1].fY) / d;
				float vvx = (points[i - 1].fX - points[i - 2].fX) / distances[i - 2];
				float vvy = (points[i - 1].fY - points[i - 2].fY) / distances[i - 2];
				float scalar = vx * vvx + vy * vvy;
				if (scalar < 0 && detectSharpAngle) {
					verySharpAngle = true;
				}
			}
			distances.push_back(d);
			roadLength += d;
			if (inside) {
				visibleRoadLength += d;
				if (!prevInside) {
					startVisible = i - 1;
				}
			} else if (prevInside) {
				if (visibleRoadLength >= normalTextLen) {
					break;
				}
				visibleRoadLength = 0;
			}
		}
		prevInside = inside;
	}

	if ((textw >= roadLength || verySharpAngle) && !db.debugTextDisplayShortRoadNames) {
		delete[] points;
		return false;
	}
	int startInd = 0;
	int endInd = len;

	if (textw < visibleRoadLength && i - startVisible > 1) {
		startInd = startVisible;
		endInd = i;
		// display long road name in center
		if (visibleRoadLength > 3 * textw) {
			bool ch;
			do {
				ch = false;
				if (endInd - startInd > 2 && visibleRoadLength - distances[startInd] > normalTextLen) {
					visibleRoadLength -= distances.at(startInd);
					startInd++;
					ch = true;
				}
				if (endInd - startInd > 2 && visibleRoadLength - distances[endInd - 2] > normalTextLen) {
					visibleRoadLength -= distances.at(endInd - 2);
					endInd--;
					ch = true;
				}
			} while (ch);
		}
	}

	if (!p->drawOnPath) {
		// int middle = startInd + 1 + (endInd - startInd - 1) / 2;

		float px = 0;
		float py = 0;
		float dx = 0;
		float dy = 0;
		for (i = startInd; i < endInd; i++) {
			px += points[i].fX;
			py += points[i].fY;
			if (i > 0) {
				dx += points[i].fX - points[i - 1].fX;
				dy += points[i].fY - points[i - 1].fY;
			}
		}
		px /= (endInd - startInd);
		py /= (endInd - startInd);
		float cx = 0;
		float cy = 0;
		float cd = -1;
		for (i = startInd + 1; i < endInd; i++) {
			float fd = sqr(px - points[i].fX) + sqr(py - points[i].fY);
			if (cd < 0 || fd < cd) {
				cx = points[i].fX;
				cy = points[i].fY;
				cd = fd;
			}
		}
		p->centerX = cx;
		p->centerY = cy;

		float rot = dy == 0 && dx == 0 ? 0 : atan2(dy, dx);
		if (rot < 0) {
			rot += M_PI * 2;
		}
		p->pathRotate = rot;
		p->hOffset = 0;
	} else {
		// shrink path to display more text

		if (startInd > 0 || endInd < len) {
			// find subpath
			SkPath* path = new SkPath;
			for (int i = startInd; i < endInd; i++) {
				if (i == startInd) {
					path->moveTo(points[i].fX, points[i].fY);
				} else {
					path->lineTo(points[i].fX, points[i].fY);
				}
			}
			if (p->path != NULL) {
				delete p->path;
			}
			p->path = path;
		}
		// calculate vector of the road (px, py) to proper rotate it
		float px = 0;
		float py = 0;
		for (i = startInd + 1; i < endInd; i++) {
			px += points[i].fX - points[i - 1].fX;
			py += points[i].fY - points[i - 1].fY;
		}
		float scale = 0.5f;
		float plen = sqrt(px * px + py * py);
		// vector ox,oy orthogonal to px,py to \ure height
		float ox = -py;
		float oy = px;
		if (plen > 0) {
			float rot = atan2(py, px);
			if (rot < 0) {
				rot += M_PI * 2;
			}
			if (rot > M_PI_2 && rot < 3 * M_PI_2) {
				rot += M_PI;
				inverse = true;
				ox = -ox;
				oy = -oy;
			}
			p->pathRotate = rot;
			ox *= (p->bounds.height() / plen) / 2;
			oy *= (p->bounds.height() / plen) / 2;
		}

		p->centerX = points[startInd].fX + scale * px + ox;
		p->centerY = points[startInd].fY + scale * py + oy;
		p->hOffset = 0;

		if (inverse) {
			SkPath* path = new SkPath;
			for (int i = endInd - 1; i >= startInd; i--) {
				if (i == (int)(endInd - 1)) {
					path->moveTo(points[i].fX, points[i].fY);
				} else {
					path->lineTo(points[i].fX, points[i].fY);
				}
			}
			if (p->path != NULL) {
				delete p->path;
			}
			p->path = path;
		}
	}
	delete[] points;
	return true;
}

void drawTestBox(SkCanvas* cv, SkRect* r, float rot, SkPaint* paintIcon, std::string text, SkPaint* paintText, SkFont* skFontText, FontEntry* fontEntry) {
	cv->save();
	cv->translate(r->centerX(), r->centerY());
	cv->rotate(rot * 180 / M_PI);
	SkRect rs = SkRect::MakeLTRB(-r->width() / 2, -r->height() / 2, r->width() / 2, r->height() / 2);
	cv->drawRect(rs, *paintIcon);
	if (paintText != NULL) {
		globalFontRegistry.drawHbText(cv, text, fontEntry, *paintText, *skFontText, rs.centerX(), rs.centerY());
	}
	cv->restore();
}

bool intersects(SkRect tRect, float tRot, SHARED_PTR<TextDrawInfo>& s) {
	float sRot = s->pathRotate;
	if (absFloat(tRot) < M_PI / 15 && absFloat(sRot) < M_PI / 15) {
		return SkRect::Intersects(tRect, s->bounds);
	}
	float dist = sqrt(sqr(tRect.centerX() - s->bounds.centerX()) + sqr(tRect.centerY() - s->bounds.centerY()));
	if (dist < 3) {
		return true;
	}
	SkRect sRect = s->bounds;

	// difference close to 90/270 degrees
	if (absFloat(cos(tRot - sRot)) < 0.3) {
		// rotate one rectangle to 90 degrees
		tRot += M_PI_2;
		tRect = SkRect::MakeXYWH(tRect.centerX() - tRect.height() / 2, tRect.centerY() - tRect.width() / 2,
								 tRect.height(), tRect.width());
	}

	// determine difference close to 180/0 degrees
	if (absFloat(sin(tRot - sRot)) < 0.3) {
		// rotate t box
		// (calculate offset for t center suppose we rotate around s center)
		float diff = atan2(tRect.centerY() - sRect.centerY(), tRect.centerX() - sRect.centerX());
		diff -= sRot;
		float left = sRect.centerX() + dist * cos(diff) - tRect.width() / 2;
		float top = sRect.centerY() - dist * sin(diff) - tRect.height() / 2;
		SkRect nRect = SkRect::MakeXYWH(left, top, tRect.width(), tRect.height());
		return SkRect::Intersects(nRect, sRect);
	}

	// TODO other cases not covered
	return SkRect::Intersects(tRect, sRect);
}

bool intersects(SHARED_PTR<TextDrawInfo>& t, SHARED_PTR<TextDrawInfo>& s) {
	return intersects(t->bounds, t->pathRotate, s);
}
#if defined(WIN32)
#undef max
#endif
inline float max(float a, float b) {
	return a > b ? a : b;
}

bool findTextIntersection(SkCanvas* cv, RenderingContext* rc, quad_tree<SHARED_PTR<TextDrawInfo>>& boundIntersections,
						  SHARED_PTR<TextDrawInfo>& text, SkPaint* paintText, SkPaint* paintIcon, DebugTextInfo db, SkFont* skFontText, FontEntry* fontEntry) {
	vector<SHARED_PTR<TextDrawInfo>> searchText;
	int textWrap = text->textWrap == 0 ? 22 : text->textWrap;
	int text1Line = text->text.length() > textWrap && !text->drawOnPath ? textWrap : text->text.length();
	skFontText->measureText(text->text.c_str(), text->text.length(), SkTextEncoding::kUTF8, &text->textBounds, paintText);
	text->bounds = text->textBounds;
	// make wider and multiline
	text->bounds.inset(-rc->getDensityValue(3),
					   -(rc->getDensityValue(5) + ((text->text.length() - 1) / text1Line) * text->bounds.height()));
	bool display = calculatePathToRotate(rc, text, db);
	if (!display) {
		return true;
	}

	if (text->path == NULL) {
		text->bounds.offset(text->centerX, text->centerY);
		// shift to match alignment
		text->bounds.offset(-text->bounds.width() / 2, 0);
	} else {
		text->bounds.offset(text->centerX - text->bounds.width() / 2, text->centerY - text->bounds.height() / 2);
	}
	text->bounds.inset(-text->intersectionMargin, -text->intersectionMargin);
	float cf = text->intersectionSizeFactor - 1;
	text->bounds.inset(-cf * text->textSize / 2, -cf * text->textSize / 2 - text->vOffset);

	// for text purposes
	if (db.debugTextDisplayBBox) {
		drawTestBox(cv, &text->bounds, text->pathRotate, paintIcon, text->text, NULL /*paintText*/, skFontText, fontEntry);
	}
	boundIntersections.query_in_box(text->bounds, searchText);
	for (uint32_t i = 0; i < searchText.size(); i++) {
		SHARED_PTR<TextDrawInfo> t = searchText.at(i);
		if (intersects(text, t) && !db.debugTextDoNotFindIntersections) {
			return true;
		}
	}
	if (text->minDistance > 0) {
		SkRect boundsSearch = text->bounds;
		boundsSearch.inset(-max(rc->getDensityValue(5.0f), text->minDistance),
						   -max(rc->getDensityValue(15.0f), text->minDistance));
		boundIntersections.query_in_box(boundsSearch, searchText);
		if (db.debugTextDisplayShieldBBox) {
			drawTestBox(cv, &boundsSearch, text->pathRotate, paintIcon, text->text, paintText, skFontText, fontEntry);
		}
		for (uint32_t i = 0; i < searchText.size(); i++) {
			SHARED_PTR<TextDrawInfo> t = searchText.at(i);
			if (t->minDistance > 0 && t->text == text->text && intersects(boundsSearch, text->pathRotate, t) &&
				!db.debugTextDoNotFindIntersectionsSameName) {
				return true;
			}
		}
	}

	boundIntersections.insert(text, text->bounds);

	return false;
}

bool textOrder(SHARED_PTR<TextDrawInfo>& text1, SHARED_PTR<TextDrawInfo>& text2) {
	if (text1->textOrder == text2->textOrder) {
		return text1->secondOrder < text2->secondOrder;
	}
	return text1->textOrder < text2->textOrder;
}

void drawShield(SHARED_PTR<TextDrawInfo>& textDrawInfo, std::string res, SkPaint* paintIcon, RenderingContext* rc,
				SkCanvas* cv, SkRect& r, SkFontMetrics fm) {
	if (res.length() == 0) {
		return;
	}
	SkBitmap* ico = getCachedBitmap(rc, res);
	if (ico != NULL) {
		float coef = rc->getDensityValue(rc->getScreenDensityRatio() * rc->getTextScale());
		float left = textDrawInfo->centerX - ico->width() / 2 * coef - 0.5f;  //
		float top =
			textDrawInfo->centerY - ico->height() / 2 * coef + fm.fTop / 3;	 // textDrawInfo->textBounds.height() / 2;
		SkIRect src = SkIRect::MakeXYWH(0, 0, ico->width(), ico->height());
		SkRect r = SkRect::MakeXYWH(left, top, ico->width() * coef, ico->height() * coef);
		PROFILE_NATIVE_OPERATION(rc, cv->drawBitmapRect(*ico, src, r, paintIcon));
	}
}

bool combine2Segments(std::vector<SkPoint>* pointsS, std::vector<SkPoint>* pointsP, SkPath* s, float combineGap,
					  float* gapMetric, bool combine) {
	float px0 = (*pointsP)[0].fX;
	float py0 = (*pointsP)[0].fY;
	float sxl = (*pointsS)[pointsS->size() - 1].fX;
	float syl = (*pointsS)[pointsS->size() - 1].fY;
	*gapMetric = abs(px0 - sxl) + abs(py0 - syl);
	if (*gapMetric < combineGap) {
		// calculate scalar product to maker sure connecting good line
		float px1 = (*pointsP)[1].fX;
		float py1 = (*pointsP)[1].fY;
		float sxl1 = (*pointsS)[pointsS->size() - 2].fX;
		float syl1 = (*pointsS)[pointsS->size() - 2].fY;
		float pvx = (px1 - px0) / sqrt((px1 - px0) * (px1 - px0) + (py1 - py0) * (py1 - py0));
		float pvy = (py1 - py0) / sqrt((px1 - px0) * (px1 - px0) + (py1 - py0) * (py1 - py0));
		float svx = (sxl - sxl1) / sqrt((sxl - sxl1) * (sxl - sxl1) + (syl - syl1) * (syl - syl1));
		float svy = (syl - syl1) / sqrt((sxl - sxl1) * (sxl - sxl1) + (syl - syl1) * (syl - syl1));

		if (pvx * svx + svy * pvy <= 0) {
			return false;
		}
		if (combine) {
			for (int k = 1; k < pointsP->size(); k++) {
				s->lineTo((*pointsP)[k].fX, (*pointsP)[k].fY);
			}
		}
		return true;
	}
	return false;
}

float calcLength(std::vector<SkPoint>* pointsP) {
	float len = 0;
	for (int i = 1; i < (*pointsP).size(); i++) {
		float dx = (*pointsP)[i].fX - (*pointsP)[i - 1].fX;
		float dy = (*pointsP)[i].fX - (*pointsP)[i - 1].fX;
		len += sqrt(dx * dx + dy * dy);
	}
	return len;
}

void combineSimilarText(RenderingContext* rc) {
	float combineGap = rc->getDensityValue(45);
	float combineMaxLength = rc->getDensityValue(550);	// max length
	UNORDERED(map)<std::string, vector<SHARED_PTR<TextDrawInfo>>> namesMap;
	for (auto it = rc->textToDraw.begin(); it != rc->textToDraw.end(); it++) {
		if ((*it)->drawOnPath && (*it)->path != NULL) {
			int len = (*it)->path->countPoints();
			if (len > 1) {
				std::string str = (*it)->text + ((char)(*it)->textOrder);
				namesMap[str].push_back(*it);
			}
		}
	}

	std::vector<SkPoint> pointsS;
	std::vector<SkPoint> pointsSCombine;
	std::vector<SkPoint> pointsP;

	for (auto it = namesMap.begin(); it != namesMap.end(); it++) {
		vector<SHARED_PTR<TextDrawInfo>> list = it->second;
		int combined = 20;	// max combined

		bool combineOnIteration = true;
		if (list.size() > 1) {
			vector<float> distances;
			// distances.resize(list.size());
			for (auto p = list.begin(); p != list.end(); p++) {
				int lenP = (*p)->path->countPoints();
				pointsP.resize(lenP);
				(*p)->path->getPoints(&pointsP[0], lenP);
				distances.push_back(calcLength(&pointsP));
			}

			while (combined > 0 && combineOnIteration) {
				combined--;
				combineOnIteration = false;
				int pi = 0;
				for (auto p = list.begin(); p != list.end() && !combineOnIteration; p++, pi++) {
					if ((*p)->combined) continue;
					int lenP = (*p)->path->countPoints();
					pointsP.resize(lenP);
					(*p)->path->getPoints(&pointsP[0], lenP);
					if (distances[pi] > combineMaxLength) continue;

					auto sToCombine = p;
					float minGap = combineGap;
					int siCombine = pi;
					float gapMeasure = 0;

					auto s = p;
					int si = pi + 1;
					for (s++; s != list.end(); s++, si++) {
						if ((*s)->combined || p == s) continue;
						if (distances[si] > combineMaxLength) continue;
						int lenS = (*s)->path->countPoints();
						pointsS.resize(lenS);
						(*s)->path->getPoints(&pointsS[0], lenS);
						// debug
						// float xGap = abs(pointsP[0].fX - pointsS[lenS - 1].fX);
						// float yGap = abs(pointsP[0].fY - pointsS[lenS - 1].fY);
						// auto pid = (*p)->object.id / 128;
						// auto sid = (*s)->object.id / 128;
						// printf("? Combine ? %f %f out of %s %d %d %ld %ld \n",  xGap, yGap, (*p)->text.c_str(), lenS,
						// lenP, pid, sid); debug
						if (combine2Segments(&pointsS, &pointsP, (*s)->path, combineGap, &gapMeasure, false) ||
							combine2Segments(&pointsP, &pointsS, (*p)->path, combineGap, &gapMeasure, false)) {
							if (minGap > gapMeasure) {
								// debugP("? Combine ? %f\n", gapMeasure),
								minGap = gapMeasure;
								sToCombine = s;
								siCombine = si;
								pointsSCombine = pointsS;
							}
						}
					}
					if (sToCombine != p) {
						if (combine2Segments(&pointsSCombine, &pointsP, (*sToCombine)->path, combineGap, &gapMeasure,
											 true)) {
							(*p)->combined = true;
							combineOnIteration = true;
							// printf("Combined %s - %d of %d %f ++ %f\n", (*p)->text.c_str(),(20 - combined),
							// list.size(), distances[siCombine],distances[pi]);
							distances[siCombine] += distances[pi];

						} else if (combine2Segments(&pointsP, &pointsSCombine, (*p)->path, combineGap, &gapMeasure,
													true)) {
							(*sToCombine)->combined = true;
							combineOnIteration = true;

							// printf("Combined %s - %d of %d %f ++ %f\n", (*p)->text.c_str(),(20 - combined),
							// list.size(), distances[pi], distances[siCombine]);
							distances[pi] += distances[siCombine];
						}
					}
				}
			}
		}
	}
}

static sk_sp<SkTypeface> sDefaultTypeface = nullptr;
static sk_sp<SkTypeface> sItalicTypeface = nullptr;
static sk_sp<SkTypeface> sBoldTypeface = nullptr;
static sk_sp<SkTypeface> sBoldItalicTypeface = nullptr;

void drawTextOverCanvas(RenderingContext* rc, RenderingRuleSearchRequest* req, SkCanvas* cv) {
	SkRect r = SkRect::MakeLTRB(0, 0, rc->getWidth(), rc->getHeight());
	r.inset(-rc->getDensityValue(25), -rc->getDensityValue(25));
	quad_tree<SHARED_PTR<TextDrawInfo>> boundsIntersect(r, 4, 0.6);
	DebugTextInfo db(req);

	SkPaint paintIcon;
	paintIcon.setStyle(SkPaint::kStroke_Style);
	paintIcon.setStrokeWidth(1);
	paintIcon.setColor(0xff000000);
	paintIcon.setFilterQuality(SkFilterQuality::kLow_SkFilterQuality);
	SkPaint paintText;
	paintText.setStyle(SkPaint::kFill_Style);
	paintText.setStrokeWidth(1);
	paintText.setColor(0xff000000);
	paintText.setAntiAlias(true);
	SkFontMetrics fm;
	SkFont skFontText;
	FontEntry* fontEntry = nullptr;

	// 1. Sort text using text order
	std::sort(rc->textToDraw.begin(), rc->textToDraw.end(), textOrder);

	combineSimilarText(rc);

	// 2. Calculate intersections and choose what text to draw
	for (auto itdi = rc->textToDraw.begin(); itdi != rc->textToDraw.end(); ++itdi) {
		SHARED_PTR<TextDrawInfo> textDrawInfo = *itdi;

		// Skip empty text
		if (textDrawInfo->text.length() <= 0) continue;
		if (textDrawInfo->combined) continue;
		if (textDrawInfo->icon && !textDrawInfo->icon->visible) continue;

		fontEntry = globalFontRegistry.updateFontEntry(rc->getReshapedString(textDrawInfo->text.c_str()),
										  textDrawInfo->bold,  // false,
										  textDrawInfo->italic);
		// set text size before finding intersection (it is used there)
		float textSize = textDrawInfo->textSize;
		skFontText.setSize(textSize);
		skFontText.getMetrics(&fm);

		// calculate if there is intersection
		if (textDrawInfo->icon && textDrawInfo->icon->bmp) {
			textDrawInfo->centerY += textDrawInfo->icon->bmp->height() / 2;
			textDrawInfo->centerY += ((-fm.fAscent));
		}
		bool intersects = findTextIntersection(cv, rc, boundsIntersect, textDrawInfo, &paintText, &paintIcon, db, &skFontText, fontEntry);
		if (!intersects) {
			if (rc->interrupted()) {
				return;
			}
			textDrawInfo->visible = true;
		} else {
			textDrawInfo->visible = false;
		}
	}

	// 3. Draw selected text in reverse order
	for (auto itdi = rc->textToDraw.rbegin(); itdi != rc->textToDraw.rend(); ++itdi) {
		SHARED_PTR<TextDrawInfo> textDrawInfo = *itdi;

		fontEntry = globalFontRegistry.updateFontEntry(rc->getReshapedString(textDrawInfo->text.c_str()),
										  textDrawInfo->bold,
										  textDrawInfo->italic);
		float textSize = textDrawInfo->textSize;
		skFontText.setSize(textSize);
		paintText.setColor(textDrawInfo->textColor);
		// align center y
		skFontText.getMetrics(&fm);

		if (textDrawInfo->visible) {
			//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Text %s font %s", textDrawInfo->text.c_str(), fontEntry->pathToFont.c_str());
			if (rc->interrupted()) return;
			if (textDrawInfo->drawOnPath && textDrawInfo->path != NULL) {
				textDrawInfo->text = rc->getReshapedString(textDrawInfo->text);				
				if (textDrawInfo->textShadow > 0) {
					paintText.setColor(textDrawInfo->textShadowColor);
					paintText.setStyle(SkPaint::kStroke_Style);
					paintText.setStrokeWidth(2 + textDrawInfo->textShadow);
					rc->nativeOperations.Pause();
					globalFontRegistry.drawHbTextOnPath(cv, textDrawInfo->text, *textDrawInfo->path, fontEntry, skFontText, paintText, textDrawInfo->hOffset, textDrawInfo->vOffset - fm.fTop / 4);
					rc->nativeOperations.Start();
					// reset
					paintText.setStyle(SkPaint::kFill_Style);
					paintText.setStrokeWidth(2);
					paintText.setColor(textDrawInfo->textColor);
				}
				rc->nativeOperations.Pause();
				globalFontRegistry.drawHbTextOnPath(cv, textDrawInfo->text, *textDrawInfo->path, fontEntry, skFontText, paintText, textDrawInfo->hOffset, textDrawInfo->vOffset - fm.fTop / 4);
				rc->nativeOperations.Start();
			} else {
				drawShield(textDrawInfo, textDrawInfo->shieldRes, &paintIcon, rc, cv, r, fm);
				drawShield(textDrawInfo, textDrawInfo->shieldResIcon, &paintIcon, rc, cv, r, fm);
				drawWrappedText(rc, cv, textDrawInfo, textSize, paintText, skFontText, fontEntry);
			}
		}
	}

	// add all text for debug
	for (auto itdi = rc->textToDraw.begin(); itdi != rc->textToDraw.end(); ++itdi) {
		SHARED_PTR<TextDrawInfo> textDrawInfo = *itdi;
		if (!textDrawInfo->visible) boundsIntersect.insert(textDrawInfo, textDrawInfo->bounds);
	}
	rc->textIntersect = boundsIntersect;
}

// Not used. For debug only
void FontRegistry::drawSkiaTextOnPath(SkCanvas *canvas, std::string textS, SkPath &path, FontEntry *face, SkFont &font, SkPaint &paint, float h_offset, float v_offset) {

	font.setTypeface(face->fSkiaTypeface);

	char *str = (char *)textS.c_str();	// utf-8 string
	char *str_i = str;					// string iterator
	char *end = str + strlen(str) + 1;	// end iterator
	unsigned char symbol[5] = {0, 0, 0, 0, 0};
	std::vector<SkScalar> measureX;
	SkScalar x = 0;

	do {
		uint32_t code = utf8::unchecked::next(str_i); // get 32 bit code of a utf-8 symbol
		if (code == 0)
			continue;
		memset(symbol, 0, sizeof(symbol));
		utf8::unchecked::append(code, symbol); // initialize array `symbol`
		measureX.push_back(x);
		x += font.measureText(symbol, sizeof(symbol), SkTextEncoding::kUTF8, nullptr, &paint);
	} while (str_i < end);

	SkPoint xy[measureX.size()];	
	for (int i = 0; i < measureX.size(); ++i) {
		xy[i].set(measureX[i], 0);
	}

	const char *text = textS.c_str();
	const int length = strlen(text);	
	unsigned int count = font.countText(text, length, SkTextEncoding::kUTF8);

	if (count != measureX.size())
		return;

	size_t size = count * (sizeof(SkRSXform) + sizeof(SkScalar));
	SkAutoSMalloc<512> storage(size);
	SkRSXform *xform = (SkRSXform *)storage.get();
	SkScalar *widths = (SkScalar *)(xform + count);

	SkAutoTArray<SkGlyphID> glyphs(count);
	font.textToGlyphs(text, length, SkTextEncoding::kUTF8, glyphs.get(), count);
	font.getWidths(glyphs.get(), count, widths);
	SkPathMeasure meas(path, false);

	// set text to the middle of the path
	float textLength = xy[count - 1].x() + widths[count - 1];
	float startOffset = h_offset + (meas.getLength() - textLength) / 2;

	for (int i = 0; i < count; ++i) {
		// we want to position each character on the center of its advance
		const SkScalar offset = SkScalarHalf(widths[i]);

		float pathOffset = startOffset + xy[i].x() + offset;
		if (pathOffset < 0) {
			// centering of the start glyph is out of range
			pathOffset = 0;
		}

		SkPoint pos;
		SkVector tan;
		if (pathOffset <= meas.getLength() && meas.getPosTan(pathOffset, &pos, &tan)) {
			pos += SkVector::Make(-tan.fY, tan.fX) * v_offset;
			xform[i].fSCos = tan.x();
			xform[i].fSSin = tan.y();
			xform[i].fTx = pos.x() - tan.x() * offset;
			xform[i].fTy = pos.y() - tan.y() * offset;
		} else {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, 
				"Rendering error \"OUT of meas in drawHbTextOnPath\". Values: xy[i].x() %f pathOffset %f meas.getLength() %f startOffset %f offset %f",
				xy[i].x(), pathOffset, meas.getLength(), startOffset, offset);
			return;
		}
	}

	canvas->drawTextBlob(SkTextBlob::MakeFromRSXform(glyphs.get(), count * sizeof(SkGlyphID),
													 &xform[0], font, SkTextEncoding::kGlyphID),
						 0, 0, paint);

    // For debug only
    const SkRect fontb = SkFontPriv::GetFontBounds(font);
    const SkScalar max = std::max(std::max(SkScalarAbs(fontb.fLeft), SkScalarAbs(fontb.fRight)),
                                std::max(SkScalarAbs(fontb.fTop), SkScalarAbs(fontb.fBottom)));
    const SkRect bounds = path.getBounds().makeOutset(max, max);
    SkPaint p;
    p.setStyle(SkPaint::kStroke_Style);
    canvas->drawRect(bounds, p);
}

void FontRegistry::drawHbTextOnPath(SkCanvas *canvas, std::string textS, SkPath &path, FontEntry *face, SkFont &font, SkPaint &paint, float h_offset, float v_offset) {

	if (!face->fHarfBuzzFace) {
		// fonts are not initialized
		return;
	}
	font.setTypeface(face->fSkiaTypeface);
	const char *text = textS.c_str();

	hb_font_t *hb_font = hb_font_create(face->fHarfBuzzFace.get());
	hb_font_set_scale(hb_font,
					  HARFBUZZ_FONT_SIZE_SCALE * font.getSize(),
					  HARFBUZZ_FONT_SIZE_SCALE * font.getSize());
	hb_ot_font_set_funcs(hb_font);
	hb_buffer_t *hb_buffer = hb_buffer_create();
	hb_buffer_add_utf8(hb_buffer, text, -1, 0, -1);
	hb_buffer_guess_segment_properties(hb_buffer);
	
	hb_shape(hb_font, hb_buffer, NULL, 0);

	unsigned int length = hb_buffer_get_length(hb_buffer);
	if (length == 0) {
		return;
	}

	SkAutoTArray<SkGlyphID> glyphs(length);
	hb_glyph_info_t *info = hb_buffer_get_glyph_infos(hb_buffer, NULL);
	hb_glyph_position_t *pos = hb_buffer_get_glyph_positions(hb_buffer, NULL);
	double x = 0, y = 0;
	SkPoint xy[length];
	for (unsigned int i = 0; i < length; i++) {
		glyphs[i] = info[i].codepoint;
		xy[i].set(x + pos[i].x_offset / HARFBUZZ_FONT_SIZE_SCALE, y - pos[i].y_offset / HARFBUZZ_FONT_SIZE_SCALE);
		x += pos[i].x_advance / HARFBUZZ_FONT_SIZE_SCALE;
		y += pos[i].y_advance / HARFBUZZ_FONT_SIZE_SCALE;
	}
	
	//destroy Harfbuzz variables
	hb_buffer_destroy(hb_buffer);
	hb_font_destroy(hb_font);

	SkPathMeasure meas(path, false);
	// check correlation between harfbuzz and skia
	if (xy[length - 1].x() > meas.getLength()) {
		SkScalar correlation = meas.getLength() / xy[length - 1].x();
		if (correlation < 0.5) {
			// avoid show glyphs over each other 
			return;
		}
		for (int i = 0; i < length; ++i) {
			xy[i].set(xy[i].x() * correlation, xy[i].y());
		}
	}


	size_t size = length * (sizeof(SkRSXform) + sizeof(SkScalar));
	SkAutoSMalloc<512> storage(size);
	SkRSXform *xform = (SkRSXform *)storage.get();
	SkScalar *widths = (SkScalar *)(xform + length);

	font.getWidths(glyphs.get(), length, widths);
	float textLength = xy[length - 1].x() + widths[length - 1];
	
	// Make text straight if path is too short
	if (length <= 4) {		
		SkPath simplePath;
		int countPoints = path.countPoints();
		// select point on 1/3 and 2/3 of path
		SkPoint firstPoint = path.getPoint((int)(countPoints / 3));
		SkPoint lastPoint = path.getPoint((int)((countPoints / 3) * 2));
		SkScalar dx = lastPoint.x() - firstPoint.x();
		SkScalar dy = lastPoint.y() - firstPoint.y();		
		// increase path between points 3 times
		lastPoint.set(lastPoint.x() +  dx, lastPoint.y() + dy);
		firstPoint.set(firstPoint.x() - dx, firstPoint.y() - dy);
		simplePath.moveTo(firstPoint);
		simplePath.lineTo(lastPoint);
		meas.setPath(&simplePath, false);
		if (meas.getLength() < textLength) {
			// path is very crooked
		 	return;			
		}		
	}

	// set text to the middle of the path
	float startOffset = h_offset + (meas.getLength() - textLength) / 2;
	for (int i = 0; i < length; ++i) {
		// we want to position each character on the center of its advance
		const SkScalar offset = SkScalarHalf(widths[i]);

		float pathOffset = startOffset + xy[i].x() + offset;
		if (pathOffset < 0) {
			// centering of the start glyph is out of range
			pathOffset = 0;
		}

		SkPoint pos;
		SkVector tan;
		if (pathOffset <= meas.getLength() && meas.getPosTan(pathOffset, &pos, &tan)) {
			pos += SkVector::Make(-tan.fY, tan.fX) * v_offset;
			xform[i].fSCos = tan.x();
			xform[i].fSSin = tan.y();
			xform[i].fTx = pos.x() - tan.y() * xy[i].y() - tan.x() * offset;
			xform[i].fTy = pos.y() + tan.x() * xy[i].y() - tan.y() * offset;
		} else {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, 
				"Rendering error \"OUT of meas in drawHbTextOnPath\". Values: xy[i].x() %f pathOffset %f meas.getLength() %f startOffset %f offset %f",
				xy[i].x(), pathOffset, meas.getLength(), startOffset, offset);
			return;
		}
	}
	sk_sp<SkTextBlob> blob = SkTextBlob::MakeFromRSXform(glyphs.get(), length * sizeof(SkGlyphID),
	 												 &xform[0], font, SkTextEncoding::kGlyphID);
	canvas->drawTextBlob(blob, 0, 0, paint);
	
	// For debug only, don't remove
	// const SkRect fontb = SkFontPriv::GetFontBounds(font);
	// const SkScalar max = std::max(std::max(SkScalarAbs(fontb.fLeft), SkScalarAbs(fontb.fRight)),
	//  							  std::max(SkScalarAbs(fontb.fTop), SkScalarAbs(fontb.fBottom)));
	// const SkRect bounds = path.getBounds().makeOutset(max, max);
	// SkPaint p;
	// p.setStyle(SkPaint::kStroke_Style);
	// p.setColor(0xFFFF0000);//argb - red color
	// canvas->drawRect(bounds, p);
	// p.setColor(0xFF0000FF);//argb - blue color
	// canvas->drawRect(blob->bounds(), p);
}

void FontRegistry::drawHbText(SkCanvas *cv, std::string textS, FontEntry *face, SkPaint &paint, SkFont &font, float centerX, float centerY) {

	if (!face->fHarfBuzzFace) {
		// fonts are not initialized
		return;
	}
	font.setTypeface(face->fSkiaTypeface);
	trimspec(textS);
	const char *text = textS.c_str();

	hb_font_t *hb_font = hb_font_create(face->fHarfBuzzFace.get());
	hb_font_set_scale(hb_font,
					  HARFBUZZ_FONT_SIZE_SCALE * font.getSize(),
					  HARFBUZZ_FONT_SIZE_SCALE * font.getSize());
	hb_ot_font_set_funcs(hb_font);

	hb_buffer_t *hb_buffer = hb_buffer_create();
	hb_buffer_add_utf8(hb_buffer, text, -1, 0, -1);
	hb_buffer_guess_segment_properties(hb_buffer);

	hb_shape(hb_font, hb_buffer, NULL, 0);

	unsigned int length = hb_buffer_get_length(hb_buffer);
	if (length == 0) {
		return;
	}	
	hb_glyph_info_t *info = hb_buffer_get_glyph_infos(hb_buffer, NULL);
	hb_glyph_position_t *pos = hb_buffer_get_glyph_positions(hb_buffer, NULL);

	SkTextBlobBuilder textBlobBuilder;
	auto runBuffer = textBlobBuilder.allocRunPos(font, SkToInt(length));

	double x = 0;
	double y = 0;
	for (unsigned int i = 0; i < length; i++) {
		if (face->delCodePoints.count(info[i].codepoint)) {
			runBuffer.glyphs[i] = face->repCodePoint;
		} else {
			runBuffer.glyphs[i] = info[i].codepoint;
		}
		reinterpret_cast<SkPoint *>(runBuffer.pos)[i] =
			SkPoint::Make(SkDoubleToScalar(x + pos[i].x_offset / HARFBUZZ_FONT_SIZE_SCALE),
						  SkDoubleToScalar(y - pos[i].y_offset / HARFBUZZ_FONT_SIZE_SCALE));
		x += pos[i].x_advance / HARFBUZZ_FONT_SIZE_SCALE;
		y += pos[i].y_advance / HARFBUZZ_FONT_SIZE_SCALE;
	}
	cv->drawTextBlob(textBlobBuilder.make(), centerX - x/2, centerY, paint);
	//cv->drawSimpleText(text, textS.length(), SkTextEncoding::kUTF8, centerX, centerY, font, paint);	
	
	hb_buffer_destroy(hb_buffer);
	hb_font_destroy(hb_font);
}
