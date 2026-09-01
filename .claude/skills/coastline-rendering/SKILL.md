---
name: coastline-rendering
description: Debug or change how the legacy native core decides water vs land - processCoastlines, the ocean/land bit, basemap fallback, and the read boxes in binaryRead.cpp. Use when a tile renders sea as land or land as sea, when a local render disagrees with tile.osmand.net or test.osmand.net, or when touching coastline, ocean tile or map read box code.
---

# Coastline rendering in the legacy native core

## The invariant

**What a tile renders must not depend on how large an area was requested.** The tile server renders
8x8 metatiles, the app renders the visible screen, test utilities render single 256 px tiles. Any
result that changes with the size of the request is a bug, and it is invisible to the server.

## How the decision is made

`searchObjectsForRendering` in `binaryRead.cpp`:

1. Detailed coastlines that were read are passed to `processCoastlines` over the query box. It clips
   the ways and closes the loose ends along the border of that box.
2. If that produced nothing (`coastlinesWereAdded == false`), the **basemap** coastlines are
   processed instead, over a box expanded to the zoom 11 grid - zoom 11 geometry drawn on a detailed
   tile.
3. If neither produced anything, the ocean flag of the map data box (`ocean / oceanTiles`, stored per
   zoom 12 tile) fills the rectangle whole.

Step 2 is the trap. A box with no coastline crossing it is not a box with no answer - it is a box
entirely on one side of the coast. `processCoastlines` cannot tell those apart, which is what its
own `// Fix 5833 / fix is not fully correct cause now zoom in causes land` note is about.

`zoomForOceanTiles = 12` exists because that is the grid the ocean/land bit is stored on, so it is
the finest grid on which the map itself answers "sea or land". Coastlines of the detailed maps are
read a second time over that grid, coastlines only (`SearchQuery::coastlinesOnly`).

## Diagnosing a wrong tile

Turn on the diagnostic that is already in the code - change `#ifdef DEBUG_NAT_OPERATIONS` above the
`"Detailed coastlines = %d, basemap coastlines %d..."` log in `searchObjectsForRendering` to `#if 1`,
rebuild, and render the tile. Read it like this:

- `detailed = 0` - **no detailed map for this region is loaded.** The tile says nothing about the
  renderer. Stop here and check the map folder before concluding anything.
- `detailed >= 1, detAdded = 0` - the coastline was read and thrown away. This is the bug class.
- `detailed >= 1, detAdded = 1` - the detailed coastline was used; a wrong result has another cause.

Revert the `#if 1` before committing.

## Ground truth

- `https://test.osmand.net/tile/df/{z}/{x}/{y}.png` - the same style and engine, rendered as part of
  an 8x8 metatile from the full map set.
- `https://tile.osmand.net/hd/{z}/{x}/{y}.png` - the reference the coastline test compares against.
  It uses carto water `#aad3df`, not OsmAnd `#5cc3e5` - compare shapes, not hues.

The two agree with each other in practice. If a local render disagrees with **both**, the local
render is the outlier. If it matches the server but not the reference, the map versions differ and
it is not a rendering bug.

A single tile against its own metatile is the cheapest test of the invariant: render `z/x/y` alone,
then render the 8x8 metatile containing it and crop the same 256 px out. They must be identical.

## Rules of evidence

- **Missing maps look exactly like bugs.** In one investigation 21 of 24 failing tiles were regions
  with no map downloaded. Confirm `detailed >= 1` before spending time on a tile.
- **Time a city tile before and after.** Read boxes are shared between object kinds: widening one for
  the sake of coastlines pulls in every building. Zoom 14 and zoom 16 over Berlin, Paris, London,
  Amsterdam, 64 tiles, before and after.
- **A change that fixes the hand-picked tile may fix nothing else.** Run the whole failing set.

## Already tried and rejected

Do not spend time re-deriving these; all four were built and measured.

| approach | result |
|---|---|
| Snap the coastline **processing** box to the metatile grid | Changed nothing - the objects were never read |
| Widen the shared detailed **read** box (`sleft..sright`) to zoom 12 | Fixed the tiles, but 5.2 s per zoom 16 city tile against 28 ms |
| Decide sea/land by ray casting against the **basemap** coastline | Wrong where it matters - the basemap is too generalized right next to a coast |
| Decide by the side of the **last** detailed coastline segment | 35 failures against 24 - the last way read can be hundreds of km away; only the nearest segment carries meaning |

## Ownership

`ResultPublisher::clear()` does not delete objects. Anything a read pass produces must end up in
exactly one vector that is later passed to `deleteObjects`. Putting the same pointer in two vectors
gives a use-after-free in `sortObjectsByProperOrder` during rendering, not at the read site.

## Building and running

```bash
cd targets/arm64-macosx-clang-arm64-macosx-clang.baked   # or the matching platform
make osmand -j8                                          # links into binaries/<os>/<arch>/
```

Then point a Java harness at that library:

```bash
OsmAndMapCreator/utilities.sh test-coastline-rendering -maps.dir=~/osmand/maps \
  -random -randomTilesK=1 -minzoom=12 -maxzoom=16 \
  -native=<repo>/core-legacy/binaries/darwin/arm64/libosmand.dylib
```

Keep the reference tile cache between runs - `tile.osmand.net` throttles per IP with a token bucket
(10000 tiles burst, then 1/s), and a throttled request is delayed, not rejected.
