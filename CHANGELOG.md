# Changelog

Notable changes to LZAN, newest first. Versions follow semantic versioning.

## [1.0.3] - 2026-07-23

### Fixed

- Moved the default zero-page base for zx0 (to `$F7`) and zx02 (to `$F6`); the
  old `$80` overlapped the CHRGET routine and broke a self-extracting BASIC
  program. Streams are unchanged.
- Moved the upkr and Shrinkler probability-table scratch defaults to `$C000`,
  off the screen and the BASIC program area.

### Added

- `Decruncher::run_basic_when_done()` for starting a decrunched BASIC program,
  and a zero-page safety check (`ZpClass`, `regions_hit`) that warns or errors
  when a span would break BASIC or the KERNAL on return.

## [1.0.2] - 2026-07-20

### Changed

- LZSA1 encoding uses an output-sensitive exact match finder,
  `matchfinder::find_matches_fast`: a suffix array with an LCP-interval sweep,
  replacing the brute-force offset scan. Emitted streams are unchanged — the new
  finder produces the same `MatchSet` as `find_matches_exact`, which is retained
  as the reference oracle for the equivalence tests. Encoding a 64 KB block is
  roughly 200-350x faster.

### Added

- `lzsa1::compress_lzsa1_with`, which selects the match finder explicitly.

## [1.0.1] - 2026-07-20

### Added

- BoltLZ, a byte-oriented LZ77 format with no bit reader (`bolt` module),
  encoded by `lzan::bolt`. It ships forward and backward 6510 decoders, each in
  a size-oriented default and a speed-optimized variant.
- Priority-speed decoder selection in `lzan-c64`
  (`Decruncher::priority_speed()` / `registry::pick_speed_routine`): it picks the
  fastest available decoder for a format, falling back to the balanced default
  when no faster variant exists. Speed-optimized decoders are provided for LZSA1
  (a port of John Brandwood's faster decoder, BSL-1.0), TSCrunch, BoltLZ, and the
  LZAN decoders.

## [1.0.0] - 2026-07-19

Initial public release: the LZAN container format, encoders and decoders for a
set of established retro compression formats, and the C64 decruncher collection.
