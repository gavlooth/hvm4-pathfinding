# Maritime Edge Evolution Module

**File:** `maritime-evolution.hvm`  
**Date:** 2026-02-15  
**Status:** Phase 1 Complete ✅

## Implementation Summary

Successfully implemented HVM4 edge evolution for maritime routing with discrete 6-hour weather lookups.

### Core Functions Implemented

1. **`@edge_weight_at_time`** - Calculates time-dependent edge weights
   - Discrete 6-hour time bucket snapping
   - Weather lookup at edge midpoint
   - Speed loss formula: `speed_loss = hs × 0.3 × cos(β)`
   - Returns travel time in hours

2. **`@lookup_3d`** - 3D weather tree lookup with DUP memoization
   - Nested lookups: lat → lon → time
   - DUP memoization at each level to prevent tree duplication
   - Returns Weather structure or default calm conditions

3. **`@evolve_edges`** - Tree-structured time step evolution
   - Maps over time steps (0h, 6h, 12h)
   - Maps over edges at each time step
   - Returns nested structure: `[[time, [weights]], ...]`

### Bend Patterns Followed

✅ **Tree-structured data** - All collections use `<>` cons trees, not lists  
✅ **DUP memoization** - `!` bindings for weather lookups prevent recomputation  
✅ **Recursive patterns** - Separate helper functions for nested matching  
✅ **Distinct variable names** - Avoided identical suffixes (no `_start` collisions)  
✅ **Church encodings** - Weather and Edge as lambda abstractions

### Test Data Included

**3×3 Grid, 3 Time Steps:**
- 9 nodes: (0,0) through (2,2)
- 12 edges: horizontal and vertical connections
- 27 weather points: 3 lat × 3 lon × 3 time

**Wave height gradient (Hs):**
```
t=0h:  2.0→4.0 (SE increasing)
t=6h:  2.5→4.5 (storm peak)
t=12h: 2.0→4.0 (storm passed)
```

### Data Structures

**Weather Entry:**
```hvm4
@Weather(hs, tp, wave_dir, wind_spd, wind_dir)
```

**Edge Entry:**
```hvm4
@Edge(from_id, to_id, distance, from_lat, from_lon, to_lat, to_lon)
```

**Weather Tree (nested):**
```
lat_idx <> [lon_idx <> [time_idx <> weather] ] <> rest
```

### Math Helpers

- `@abs` - Absolute value
- `@min_val`, `@max_val` - Min/max comparison
- `@cos_approx` - Simplified cosine (Taylor series)
- `@atan2_simple` - Placeholder heading calculator
- `@snap_time_bucket` - Round time to 6-hour intervals

### Entry Point

```hvm4
@main = @evolve_edges(@test_edges)(@test_weather_tree)
```

## Expected Behavior

1. Routes at t=6h should have higher costs (storm peak, Hs up to 4.5m)
2. Routes through SE quadrant should be slower (higher wave heights)
3. Edge weights scale inversely with ship speed (higher Hs → slower → longer time)

## Next Steps (Phase 2)

- [ ] C3 marshaling layer (`c3lib/maritime/weather.c3`)
- [ ] 3D graph assembly (`c3lib/maritime/graph3d.c3`)
- [ ] Integration test with Bellman-Ford pathfinding
- [ ] Full Lang-Mao 2020 speed loss formulas
- [ ] Real GRIB2 weather data ingestion

## Files Created

```
~/Documents/code/hvm4-pathfinding/
└── bend/
    ├── maritime-evolution.hvm     ← New (9.4 KB)
    └── MARITIME-README.md         ← This file
```

## Technical Notes

- **No list overhead:** All structures use binary trees for O(log n) operations
- **Memoization strategy:** DUP at tree root, propagate down recursively
- **Toy test simplifications:**
  - Direct coordinate→index mapping (no lat/lon projection)
  - Constant heading (45°) for all edges
  - Linear cosine approximation
  - These will be replaced with proper implementations in Phase 2

---

**Implementation Status:** ✅ Ready for integration testing
