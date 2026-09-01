# AGENTS.md - AI Agent Guide for OsmAnd-core-legacy

This document provides essential information for AI agents working on the OsmAnd legacy native core.

## 1. Project Overview
OsmAnd-core-legacy is the C++ engine behind OsmAnd's non-OpenGL rendering and its routing. It is
built into `libosmand` and reached from Java through JNI, so the same code serves the Android app,
the tile server, and the MapCreator command line utilities.

- **Language:** C++ (C++11), built with CMake.
- **Consumers:** Android (`legacy` core flavour), `OsmAndServer` tile rendering, `OsmAndMapCreator`.
- **Java side:** `net.osmand.NativeLibrary` / `net.osmand.NativeJavaRendering` in the OsmAnd repos.

## 2. Project Structure
- `native/src/` - the engine itself.
  - `binaryRead.cpp/.h` - reads OBF map data: index traversal, map objects, coastlines, ocean tiles.
  - `multipolygons.cpp/.h` - closes coastline rings, decides water vs land, ray casting helpers.
  - `rendering.cpp`, `commonRendering.cpp` - drawing primitives on a Skia canvas.
  - `renderRules.cpp` - the rendering style engine (`.render.xml` rules).
  - `binaryRoutePlanner.cpp`, `generalRouter.cpp`, `routePlannerFrontEnd.cpp` - routing.
  - `proto/` - generated protobuf for the OBF format.
- `targets/` - per-platform configure scripts and pre-baked CMake build directories.
- `externals/` - vendored dependencies (skia, protobuf, gdal, proj, sqlite, expat, harfbuzz).
- `binaries/<os>/<arch>/` - the built shared library that the Java side loads.

## 3. Key Concepts
- **31-bit coordinates.** The whole engine works in a 2^31 square, y growing downwards. A tile at
  zoom `z` starts at `x << (31 - z)`. The last column and row end exactly at 2^31 and overflow a
  signed int - clamp to `INT_MAX`.
- **Zoom regimes** (`binaryRead.cpp`): `zoomOnlyForBasemaps = 11` - at or below it only basemaps are
  drawn; `zoomMaxDetailedForCoastlines = 16`; `zoomForOceanTiles = 12` - the grid the ocean/land bit
  of a map data box is stored on, and therefore the grid coastlines are read and closed on;
  `zoomForBaseRouteRendering = 13`, `detailedZoomStartForRouteSection = 13`.
- **Coastlines are closed against a box.** `processCoastlines` clips coastline ways to a rectangle
  and closes the loose ends along its border. The rectangle must be a property of the map, not of
  how large an area the caller asked for - otherwise the same tile renders as land when requested
  alone and as water when requested inside a metatile.
- **Rendering requests differ in size.** The tile server renders 8x8 metatiles, the app renders the
  visible screen, the test utilities render single 256 px tiles. Anything that depends on the size
  of the requested rectangle is a bug.

## 4. Building
```bash
cd targets/<platform>-clang-<platform>-clang.baked   # e.g. arm64-macosx-clang-arm64-macosx-clang.baked
make osmand -j8
```
The link step writes straight into `binaries/<os>/<arch>/libosmand.<dylib|so|dll>`, which is the file
the Java side loads, so a rebuild is immediately visible to any Java harness pointed at it.

- Building the `osmand` target alone is enough for engine changes; the vendored externals are already
  built in the baked directories.
- `externals/configure.sh` re-fetches and rebuilds the dependencies. It takes a long time - do not
  run it unless a dependency actually changed.

## 5. Testing a change
There is no unit test harness in this repository; the engine is verified through its Java consumers.

- **Rendering:** `OsmAndMapCreator/utilities.sh test-coastline-rendering` (in the OsmAnd-tools repo)
  renders tiles and compares the water mask against `tile.osmand.net`. Pass
  `-native=<path>/binaries/<os>/<arch>/libosmand.<ext>` to test a freshly built library.
- **Comparing against the server:** `https://test.osmand.net/tile/df/{z}/{x}/{y}.png` renders the
  same style from the full map set. A difference between a local single-tile render and the server
  usually means the result depends on the size of the requested rectangle.
- **Map coverage matters.** A tile with no detailed map for its region falls back to the basemap and
  will differ from the server for reasons that have nothing to do with the change under test. Before
  concluding anything from a failing tile, confirm that detailed data for it was actually read.
- **Measure the cost.** Widening what is read is easy and expensive: read boxes are shared between
  object kinds, so expanding one for the sake of coastlines pulls in every building in a city as
  well. Always time a dense city tile at zoom 14 and 16 before and after.

## 6. Coding Standards
- Follow the surrounding style: tabs for indentation, `lowerCamelCase` for functions and variables,
  braces on the same line.
- Keep comments about *why*, not *what*, and keep them next to the constant or branch they explain.
- Prefer a narrow, well-named helper over repeating index arithmetic; four copies of the same
  bit-shift are how off-by-one errors survive.
- Watch object ownership. `ResultPublisher::clear()` does not delete objects; whatever a read pass
  produces must end up in exactly one vector that is later passed to `deleteObjects`.
- Guard every shift and addition on 31-bit coordinates against overflow.

## 7. Restrictions
- **Never run `externals/configure.sh`** or otherwise rebuild the vendored dependencies unless the
  task is explicitly about them.
- **Never edit generated files** under `native/src/proto/`; change the `.proto` source instead.
- **New files for git:** when creating source or documentation files intended for the change, add
  them to VCS. Do not add temporary, generated, local, or diagnostic files, and never add build
  output or `binaries/`. Never change `.gitignore` unless explicitly requested. Do not commit unless
  explicitly requested.

## 8. Pull requests
These rules are mandatory. A pull request that does not follow them is not ready to open.

- **English only.** Title, description, commit messages, and every comment in the code are written
  in English, regardless of the language the work was discussed in.
- **Images are required.** Every pull request that changes rendering must show the result: attach
  before/after images of the same tiles, side by side, with the reference render included when one
  exists. Upload the images to the pull request body itself so they survive; do not link to a local
  file or to a host that may expire.
- **AI disclaimer is required.** When a change was produced with the help of an AI agent, the
  description must carry a disclaimer that names the tool, states what was run and on what, and says
  what a human still has to verify.
- **The disclaimer must summarise how the change came about.** One to three paragraphs: what the
  human proposed, what the agent tried, and how it was improved - including the approaches that were
  rejected and the measurement that rejected them. A reviewer needs to know which dead ends are
  already explored and which decisions came from the human, without walking any of it again. Write
  it as a summary of the reasoning, not as a transcript: nobody needs the wording of the exchange.
- **Describe the mechanism, not just the symptom.** State what the engine did wrong, why, and which
  measurements back the fix - including the ones that failed. Numbers beat adjectives.
- **State the cost.** Any change to what is read or drawn carries a performance claim; include the
  timings that support it.

*Note: This file is a living document and should be updated as the project evolves.*
