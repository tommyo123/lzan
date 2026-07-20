# Changelog

Notable changes to LZAN, newest first. Versions follow semantic versioning.

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
