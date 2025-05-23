#ifndef _MULTIPOLYGONS_CPP
#define _MULTIPOLYGONS_CPP
#include "multipolygons.h"
#include <limits.h>

#include "Logging.h"

const bool DEBUG_LINE = false;

void printLine(OsmAnd::LogSeverityLevel level, std::string msg, int64_t id, coordinates& c, int leftX, int rightX,
			   int bottomY, int topY) {
	if (!DEBUG_LINE) {
		return;
	}
	if (c.size() == 0) {
		return;
	}
	double h = bottomY - topY;
	double w = rightX - leftX;
	OsmAnd::LogPrintf(
		level, "%s %lld (osm %lld) sx=%.4f sy=%.4f ex=%.4f ey=%.4f - top/left [%d, %d] width/height [%.0f, %.0f]",
		msg.c_str(), id, id / 64, (c.at(0).first - leftX) / w, (c.at(0).second - topY) / h,
		(c.at(c.size() - 1).first - leftX) / w, (c.at(c.size() - 1).second - topY) / h, leftX, topY, w, h);
}

// returns true if coastlines were added!
bool processCoastlines(std::vector<FoundMapDataObject>& coastLines, int leftX, int rightX, int bottomY, int topY,
					   int zoom, bool showIfThereIncompleted, bool addDebugIncompleted,
					   std::vector<FoundMapDataObject>& res) {
	// try out (quite dirty fix to align boundaries to grid)
	leftX = (leftX >> 5) << 5;
	rightX = (rightX >> 5) << 5;
	bottomY = (bottomY >> 5) << 5;
	topY = (topY >> 5) << 5;

	std::vector<coordinates> completedRings;
	std::vector<coordinates> uncompletedRings;
	std::vector<FoundMapDataObject>::iterator val = coastLines.begin();
	int64_t dbId = 0;
	for (; val != coastLines.end(); val++) {
		MapDataObject* o = val->obj;
		int len = o->points.size();
		if (len < 2) {
			continue;
		}
		dbId = -(o->id >> 1);
		coordinates cs;
		int px = o->points.at(0).first;
		int py = o->points.at(0).second;
		int x = px;
		int y = py;
		bool pinside = leftX <= x && x <= rightX && y >= topY && y <= bottomY;
		if (pinside) {
			cs.push_back(int_pair(x, y));
		}
		for (int i = 1; i < len; i++) {
			x = o->points.at(i).first;
			y = o->points.at(i).second;
			bool inside = leftX <= x && x <= rightX && y >= topY && y <= bottomY;
			bool lineEnded = calculateLineCoordinates(inside, x, y, pinside, px, py, leftX, rightX, bottomY, topY, cs);
			if (lineEnded) {
				printLine(OsmAnd::LogSeverityLevel::Debug, "Ocean: line ", -dbId, cs, leftX, rightX, bottomY, topY);
				combineMultipolygonLine(completedRings, uncompletedRings, cs);
				// create new line if it goes outside
				cs.clear();
			}
			px = x;
			py = y;
			pinside = inside;
		}
		if (cs.size() > 0) {
			printLine(OsmAnd::LogSeverityLevel::Debug, "Ocean: line ", -dbId, cs, leftX, rightX, bottomY, topY);
		}
		combineMultipolygonLine(completedRings, uncompletedRings, cs);
	}
	if (completedRings.size() == 0 && uncompletedRings.size() == 0) {
		// printf("No completed & uncompleted");
		//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Ocean: no completed & incompleted coastlines %d",
		//				  coastLines.size());
		return false;  // Fix 5833
					   // fix is not fully correct cause now zoom in causes land
					   // return coastLines.size() != 0;
	}
	
	bool coastlineCrossScreen = uncompletedRings.size() > 0;
	bool addWaterPolygonIfMissing = true;
	if (coastlineCrossScreen)
		addWaterPolygonIfMissing = !unifyIncompletedRings(uncompletedRings, completedRings, leftX, rightX, bottomY, topY, dbId, zoom);

	if (addDebugIncompleted) {
		// draw uncompleted for debug purpose
		for (uint i = 0; i < uncompletedRings.size(); i++) {
			MapDataObject* o = new MapDataObject();
			o->points = uncompletedRings[i];
			o->types.push_back(tag_value("natural", "coastline_broken"));
			res.push_back(FoundMapDataObject(o, NULL, zoom));
		}
		// draw completed for debug purpose
		for (uint i = 0; i < completedRings.size(); i++) {
			MapDataObject* o = new MapDataObject();
			o->points = completedRings[i];
			o->types.push_back(tag_value("natural", "coastline_line"));
			res.push_back(FoundMapDataObject(o, NULL, zoom));
		}
	}
	if (!showIfThereIncompleted && uncompletedRings.size() > 0) {
		// printf("There are ignored uncompleted");
		//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Ocean: incompleted coastlines %d from %d",
		//				  uncompletedRings.size(), coastLines.size());
		return false;
	}
	int landFound = 0;
	int waterFound = 0;
	for (uint i = 0; i < completedRings.size(); i++) {
		bool clockwise = isClockwiseWay(completedRings[i]);
		MapDataObject* o = new MapDataObject();
		o->points = completedRings[i];
		if (clockwise) {
			waterFound++;
			o->types.push_back(tag_value("natural", "coastline"));
		} else {
			landFound++;
			o->types.push_back(tag_value("natural", "land"));
		}
		o->id = dbId--;
		o->area = true;
		res.push_back(FoundMapDataObject(o, NULL, zoom));
	}
	//OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Debug, "Ocean: islands %d, closed water %d, coastline touches screen %d",
	//				  landFound, waterFound, unifiedUncompletedRings);
	if (!waterFound && addWaterPolygonIfMissing) {
		// add complete water tile
		MapDataObject* o = new MapDataObject();
		o->points.push_back(int_pair(leftX, topY));
		o->points.push_back(int_pair(rightX, topY));
		o->points.push_back(int_pair(rightX, bottomY));
		o->points.push_back(int_pair(leftX, bottomY));
		o->points.push_back(int_pair(leftX, topY));
		o->id = dbId--;
		o->types.push_back(tag_value("natural", "coastline"));
		res.push_back(FoundMapDataObject(o, NULL, zoom));
	}
	return true;
}

/**
 * Return true if two line segments intersect inside the segment
 *
 * source: http://www.java-gaming.org/index.php?topic=22590.0
 * @param x1 line 1 point 1 latitude
 * @param y1 line 1 point 1 longitude
 * @param x2 line 1 point 2 latitude
 * @param y2 line 1 point 2 longitude
 * @param x3 line 2 point 1 latitude
 * @param y3 line 2 point 1 longitude
 * @param x4 line 2 point 2 latitude
 * @param y4 line 2 point 2 longitude
 * @return
 */

bool linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
	// Return false if either of the lines have zero length
	if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4)) {
		return false;
	}

	// Fastest method, based on Franklin Antonio's "Faster Line Segment Intersection" topic "in Graphics Gems III" book
	// (http://www.graphicsgems.org/)

	double ax = x2 - x1;
	double ay = y2 - y1;
	double bx = x3 - x4;
	double by = y3 - y4;
	double cx = x1 - x3;
	double cy = y1 - y3;

	double alphaNumerator = by * cx - bx * cy;
	double commonDenominator = ay * bx - ax * by;
	if (commonDenominator > 0) {
		if (alphaNumerator < 0 || alphaNumerator > commonDenominator) {
			return false;
		}
	} else if (commonDenominator < 0) {
		if (alphaNumerator > 0 || alphaNumerator < commonDenominator) {
			return false;
		}
	}

	double betaNumerator = ax * cy - ay * cx;
	if (commonDenominator > 0) {
		if (betaNumerator < 0 || betaNumerator > commonDenominator) {
			return false;
		}
	} else if (commonDenominator < 0) {
		if (betaNumerator > 0 || betaNumerator < commonDenominator) {
			return false;
		}
	}

	if (commonDenominator == 0) {
		// This code wasn't in Franklin Antonio's method. It was added by Keith Woodward.
		// The lines are parallel.
		// Check if they're collinear.
		double y3LessY1 = y3 - y1;
		double collinearityTestForP3 =
			x1 * (y2 - y3) + x2 * (y3LessY1) + x3 * (y1 - y2);	// see http://mathworld.wolfram.com/Collinear.html
		// If p3 is collinear with p1 and p2 then p4 will also be collinear, since p1-p2 is parallel with p3-p4

		if (collinearityTestForP3 == 0) {
			// The lines are collinear. Now check if they overlap.
			if ((x1 >= x3 && x1 <= x4) || (x1 <= x3 && x1 >= x4) || (x2 >= x3 && x2 <= x4) || (x2 <= x3 && x2 >= x4) ||
				(x3 >= x1 && x3 <= x2) || (x3 <= x1 && x3 >= x2)) {
				if ((y1 >= y3 && y1 <= y4) || (y1 <= y3 && y1 >= y4) || (y2 >= y3 && y2 <= y4) ||
					(y2 <= y3 && y2 >= y4) || (y3 >= y1 && y3 <= y2) || (y3 <= y1 && y3 >= y2)) {
					return true;
				}
			}
		}
		return false;
	}
	return true;
}

// Copied from MapAlgorithms
int ray_intersect_x(int prevX, int prevY, int x, int y, int middleY) {
	// prev node above line
	// x,y node below line
	if (prevY > y) {
		int tx = x;
		int ty = y;
		x = prevX;
		y = prevY;
		prevX = tx;
		prevY = ty;
	}
	if (y == middleY || prevY == middleY) {
		middleY -= 1;
	}
	if (prevY > middleY || y < middleY) {
		return INT_MIN;
	} else {
		if (y == prevY) {
			// the node on the boundary !!!
			return x;
		}
		// that tested on all cases (left/right)
		double rx = x + ((double)middleY - y) * ((double)x - prevX) / (((double)y - prevY));
		return (int)rx;
	}
}

int ray_intersect_y(int prevX, int prevY, int x, int y, int middleX) {
	if (prevX > x) {
		int tx = x;
		int ty = y;
		x = prevX;
		y = prevY;
		prevX = tx;
		prevY = ty;
	}
	if (x == middleX || prevX == middleX) {
		middleX -= 1;
	}
	if (prevX > middleX || x < middleX) {
		return INT_MIN;
	} else {
		if (x == prevX) {
			return y;
		}
		double ry = y + ((double)middleX - x) * ((double)y - prevY) / (((double)x - prevX));
		return (int)ry;
	}
}

// Copied from MapAlgorithms
bool isClockwiseWay(std::vector<int_pair>& c) {
	if (c.size() == 0) {
		return true;
	}

	// calculate middle Y
	int64_t middleY = 0;
	for (size_t i = 0; i < c.size(); i++) {
		middleY += c.at(i).second;
	}
	middleY /= c.size();

	double clockwiseSum = 0;

	bool firstDirectionUp = false;
	int previousX = INT_MIN;
	int firstX = INT_MIN;

	int prevX = c.at(0).first;
	int prevY = c.at(0).second;

	for (size_t i = 1; i < c.size(); i++) {
		int x = c.at(i).first;
		int y = c.at(i).second;
		int rX = ray_intersect_x(prevX, prevY, x, y, (int)middleY);
		if (rX != INT_MIN) {
			bool skipSameSide = (y <= middleY) == (prevY <= middleY);
			if (skipSameSide) {
				continue;
			}
			bool directionUp = prevY >= middleY;
			if (firstX == INT_MIN) {
				firstDirectionUp = directionUp;
				firstX = rX;
			} else {
				bool clockwise = (!directionUp) == (previousX < rX);
				if (clockwise) {
					clockwiseSum += abs(previousX - rX);
				} else {
					clockwiseSum -= abs(previousX - rX);
				}
			}
			previousX = rX;
		}
		prevX = x;
		prevY = y;
	}

	if (firstX != INT_MIN) {
		bool clockwise = (!firstDirectionUp) == (previousX < firstX);
		if (clockwise) {
			clockwiseSum += abs(previousX - firstX);
		} else {
			clockwiseSum -= abs(previousX - firstX);
		}
	}

	return clockwiseSum >= 0;
}

void combineMultipolygonLine(std::vector<coordinates>& completedRings, std::vector<coordinates>& incompletedRings,
							 coordinates& coordinates) {
	if (coordinates.size() > 0) {
		if (coordinates.at(0) == coordinates.at(coordinates.size() - 1)) {
			if (coordinates.size() > 2)
				completedRings.push_back(coordinates);
		} else {
			bool add = true;
			for (size_t k = 0; k < incompletedRings.size();) {
				bool remove = false;
				std::vector<int_pair> i = incompletedRings.at(k);
				if (coordinates.at(0) == i.at(i.size() - 1)) {
					std::vector<int_pair>::iterator tit = coordinates.begin();
					i.insert(i.end(), ++tit, coordinates.end());
					remove = true;
					coordinates = i;
				} else if (coordinates.at(coordinates.size() - 1) == i.at(0)) {
					std::vector<int_pair>::iterator tit = i.begin();
					coordinates.insert(coordinates.end(), ++tit, i.end());
					remove = true;
				}
				if (remove) {
					std::vector<std::vector<int_pair> >::iterator ti = incompletedRings.begin();
					ti += k;
					incompletedRings.erase(ti);
				} else {
					k++;
				}
				if (coordinates.at(0) == coordinates.at(coordinates.size() - 1)) {
					completedRings.push_back(coordinates);
					add = false;
					break;
				}
			}
			if (add) {
				incompletedRings.push_back(coordinates);
			}
		}
	}
}

int safelyAddDelta(int number, int delta) {
	int res = number + delta;
	if (delta > 0 && INT_MAX - delta < number) {
		return INT_MAX;
	} else if (delta < 0 && -delta > number) {
		return 0;
	}
	return res;
}

bool closeIncompletedRing(coordinates& ring, int leftX, int rightX, int bottomY, int topY, int evalDelta) {
	if (ring.size() >= 3) {
		const auto firstIntersectionPoint = ring.front();
		const auto secondIntersectionPoint = ring.back();

		const auto firstX = firstIntersectionPoint.first;
		const auto firstY = firstIntersectionPoint.second;
		const auto secondX = secondIntersectionPoint.first;
		const auto secondY = secondIntersectionPoint.second;

		bool shortLeftGap = firstX == leftX && secondX == leftX && firstY <= safelyAddDelta(secondY, evalDelta);
		bool shortTopGap = firstY == topY && secondY == topY && firstX >= safelyAddDelta(secondX, -evalDelta);
		bool shortRightGap = firstX == rightX && secondX == rightX && firstY >= safelyAddDelta(secondY, -evalDelta);
		bool shortBottomGap = firstY == bottomY && secondY == bottomY && firstX <= safelyAddDelta(secondX, evalDelta);

		if (shortLeftGap || shortTopGap || shortRightGap || shortBottomGap)
		{
			// Close ring
			ring.push_back(ring.front());
			return true;
		}
	}

	return false;
}

bool unifyIncompletedRings(std::vector<std::vector<int_pair> >& toProccess,
						   std::vector<std::vector<int_pair> >& completedRings, int leftX, int rightX, int bottomY,
						   int topY, int64_t dbId, int zoom) {
	std::set<int> nonvisitedRings;
	std::vector<coordinates> incompletedRings(toProccess);
	toProccess.clear();
	std::vector<coordinates>::iterator ir = incompletedRings.begin();
	int j = 0;
	for (j = 0; ir != incompletedRings.end(); ir++, j++) {
		int x = ir->at(0).first;
		int y = ir->at(0).second;
		int sx = ir->at(ir->size() - 1).first;
		int sy = ir->at(ir->size() - 1).second;
		bool st = y == topY || x == rightX || y == bottomY || x == leftX;
		bool end = sy == topY || sx == rightX || sy == bottomY || sx == leftX;
		// something goes wrong
		// These exceptions are used to check logic about processing multipolygons
		// However this situation could happen because of broken multipolygons (so it should data causes app error)
		// that's why these exceptions could be replaced with return; statement.
		if (!end || !st) {
			printLine(OsmAnd::LogSeverityLevel::Error, "Error processing multipolygon", 0, *ir, leftX, rightX, bottomY,
					  topY);
			toProccess.push_back(*ir);
		} else {
			if (DEBUG_LINE) {
				printLine(OsmAnd::LogSeverityLevel::Debug, "Ocean line touch:  ", -dbId, *ir, leftX, rightX, bottomY,
						  topY);
			}
			nonvisitedRings.insert(j);
		}
	}

	const int EVAL_DELTA = 2 << (22 - zoom);

	// Fix https://github.com/osmandapp/OsmAnd/issues/16898#issue-1655569408
	// If there is one unclosed ring with gap <= EVAL_DELTA, close it without unification
	// to fix wrong unified polygon. Closing 2 or more rings in such way can break unification of
	// other unclosed rings
	if (nonvisitedRings.size() == 1) {
		auto index = *nonvisitedRings.begin();
		auto ring = incompletedRings.at(index);

		if (closeIncompletedRing(ring, leftX, rightX, bottomY, topY, EVAL_DELTA)) {
			completedRings.push_back(ring);
			return false;
		}
	}

	ir = incompletedRings.begin();
	for (j = 0; ir != incompletedRings.end(); ir++, j++) {
		if (nonvisitedRings.find(j) == nonvisitedRings.end()) {
			continue;
		}
		int x = ir->at(ir->size() - 1).first;
		int y = ir->at(ir->size() - 1).second;
		const int UNDEFINED_MIN_DIFF = -1 - EVAL_DELTA;
		const double h = bottomY - topY;
		const double w = rightX - leftX;
		if (DEBUG_LINE) {
			OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Visit incomplete ring %.4f %.4f %.4f %.4f",
							  (ir->at(0).first - leftX) / w, (ir->at(0).second - topY) / h, (x - leftX) / w,
							  (y - topY) / h);
		}
		while (true) {
			int st = 0;	 // st already checked to be one of the four
			if (y == topY) {
				st = 0;
			} else if (x == rightX) {
				st = 1;
			} else if (y == bottomY) {
				st = 2;
			} else if (x == leftX) {
				st = 3;
			}
			int nextRingIndex = -1;
			// BEGIN go clockwise around rectangle
			for (int h = st; h <= st + 4; h++) {
				// BEGIN find closest nonvisited start (including current)
				int mindiff = UNDEFINED_MIN_DIFF;
				std::vector<std::vector<int_pair> >::iterator cni = incompletedRings.begin();
				int cnik = 0;
				for (; cni != incompletedRings.end(); cni++, cnik++) {
					if (nonvisitedRings.find(cnik) == nonvisitedRings.end()) {
						continue;
					}
					int csx = cni->at(0).first;
					int csy = cni->at(0).second;
					bool lastSegment = h == st + 4;
					int currentStartPoint = 0;
					int prevEndPoint = 0;
					int currentStartPointIsGreater = -1;
					if (h % 4 == 0 && csy == topY) {
						// top
						if (csy == topY && csx >= safelyAddDelta(x, -EVAL_DELTA)) {
							if (mindiff == UNDEFINED_MIN_DIFF || (csx - x) <= mindiff) {
								mindiff = (csx - x);
								nextRingIndex = cnik;
							}
						}
					} else if (h % 4 == 1) {
						// right
						if (csx == rightX && csy >= safelyAddDelta(y, -EVAL_DELTA)) {
							if (mindiff == UNDEFINED_MIN_DIFF || (csy - y) <= mindiff) {
								mindiff = (csy - y);
								nextRingIndex = cnik;
							}
						}
					} else if (h % 4 == 2 && csy == bottomY) {
						// bottom
						if (csy == bottomY && csx <= safelyAddDelta(x, EVAL_DELTA)) {
							if (mindiff == UNDEFINED_MIN_DIFF || (x - csx) <= mindiff) {
								mindiff = (x - csx);
								nextRingIndex = cnik;
							}
						}
					} else if (h % 4 == 3) {
						// left
						if (csx == leftX && csy <= safelyAddDelta(y, EVAL_DELTA)) {
							if (mindiff == UNDEFINED_MIN_DIFF || (y - csy) <= mindiff) {
								mindiff = (y - csy);
								nextRingIndex = cnik;
							}
						}
					}
					if (currentStartPointIsGreater >= 0) {
						bool checkMinDiff = currentStartPointIsGreater == 1
												? prevEndPoint <= safelyAddDelta(currentStartPoint, EVAL_DELTA)
												: safelyAddDelta(prevEndPoint, EVAL_DELTA) >= currentStartPoint;
						int delta = abs(currentStartPoint - prevEndPoint);
						if (checkMinDiff && (mindiff == UNDEFINED_MIN_DIFF || delta <= mindiff)) {
							mindiff = delta;
							nextRingIndex = cnik;
						}
					}
				}
				// END find closest start (including current)

				// we found start point
				if (mindiff != UNDEFINED_MIN_DIFF) {
					break;
				} else {
					if (h % 4 == 0) {
						// top
						y = topY;
						x = rightX;
					} else if (h % 4 == 1) {
						// right
						y = bottomY;
						x = rightX;
					} else if (h % 4 == 2) {
						// bottom
						y = bottomY;
						x = leftX;
					} else if (h % 4 == 3) {
						y = topY;
						x = leftX;
					}
					ir->push_back(int_pair(x, y));
				}

			}  // END go clockwise around rectangle

			if (nextRingIndex == -1) {
				// error - current start should always be found
				OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Error, "Could not find next ring %d %d", x - leftX,
								  y - topY);
				ir->push_back(ir->at(0));
				nonvisitedRings.erase(j);
				break;
			} else if (nextRingIndex == j) {
				if (DEBUG_LINE) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Ring is closed as island");
				}
				ir->push_back(ir->at(0));
				nonvisitedRings.erase(j);
				break;
			} else {
				std::vector<int_pair> p = incompletedRings.at(nextRingIndex);
				int csx = p.at(0).first;
				int csy = p.at(0).second;
				ir->insert(ir->end(), p.begin(), p.end());
				nonvisitedRings.erase(nextRingIndex);
				// get last point and start again going clockwise
				x = ir->at(ir->size() - 1).first;
				y = ir->at(ir->size() - 1).second;
				if (DEBUG_LINE) {
					OsmAnd::LogPrintf(OsmAnd::LogSeverityLevel::Info, "Attach line from %.4f %.4f to %.4f %.4f",
									  (csx - leftX) / w, (csy - topY) / h, (x - leftX) / w, (y - topY) / h);
				}
			}
		}

		completedRings.push_back(*ir);
	}

	return true;
}

bool calculateLineCoordinates(bool inside, int x, int y, bool pinside, int px, int py, int leftX, int rightX,
							  int bottomY, int topY, std::vector<int_pair>& coordinates) {
	bool lineEnded = false;
	int_pair b(x, y);
	if (pinside) {
		if (!inside) {
			bool is = calculateIntersection(x, y, px, py, leftX, rightX, bottomY, topY, b);
			if (!is) {
				b.first = px;
				b.second = py;
			}
			coordinates.push_back(b);
			lineEnded = true;
		} else {
			coordinates.push_back(b);
		}
	} else {
		bool is = calculateIntersection(x, y, px, py, leftX, rightX, bottomY, topY, b);
		if (inside) {
			// assert is != -1;
			coordinates.push_back(b);
			int_pair n(x, y);
			coordinates.push_back(n);
		} else if (is) {
			coordinates.push_back(b);
			calculateIntersection(x, y, b.first, b.second, leftX, rightX, bottomY, topY, b);
			coordinates.push_back(b);
			lineEnded = true;
		}
	}

	return lineEnded;
}

#endif
