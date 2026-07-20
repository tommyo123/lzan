# LZAN

LZAN is a pure-Rust collection of compression encoders and decoders for C64 and
other 6502-based systems. It includes the LZAN container format, implementations
of established retro compression formats, and configurable 6502 decruncher
sources. The core crate uses only the Rust standard library.

## Included formats

The collection covers the 15 formats listed below, each with a 6502 decoder in
[`decrunchers/`](decrunchers/). The `format` module provides a common `compress` and
`decompress` interface; format modules also expose native APIs for
format-specific settings.

| Format | Module | Stream type |
|---|---|---|
| LZAN (full) | `zx` | LZAN container |
| LZAN (minimal) | `zx` | LZAN container, minimal decoder profile |
| ZX0 v2 | `zx0compat`, `zx0opt` | Raw stream |
| ZX02 | `zx02` | Raw stream |
| LZSA1 | `lzsa1` | Raw block |
| LZSA2 | `lzsa2` | Raw block |
| Exomizer 3 | `exo3` | Raw stream |
| upkr | `upkr` | Raw stream |
| ByteBoozer2 | `bb2` | Raw stream and B2 container |
| TSCrunch | `tscrunch` | Raw stream |
| aPLib / apultra | `apultra` | Raw stream |
| Shrinkler | `shrinkler` | Raw stream |
| Subsizer | `subsizer` | Raw stream |
| PuCrunch | `pucrunch` | Raw stream |
| BoltLZ | `bolt` | Byte-only LZ, no bit reader |

BoltLZ is the project's own byte-oriented LZ77: every field is a whole byte and
dispatch is a single sign-bit test, trading ratio for a small decoder (97 bytes,
no undocumented opcodes). Several formats, BoltLZ included, provide forward and
backward streams for in-place decoding, and a speed-optimized decoder variant
alongside the size-oriented default.

## Library use

```rust
// LZAN container.
let packed = lzan::compress(input);
let original = lzan::decompress(&packed).unwrap();

// A selected format through the common interface.
use lzan::format::{compress, decompress, Format};
let packed = compress(Format::Zx0, input, 1, false);
let original = decompress(Format::Zx0, &packed, false);
```

The normalized `level` argument controls encoder effort. Higher values may use
more time to produce smaller output.

## Command-line tool

```text
cargo build --release

target/release/lzan c <input> <output>  # create an LZAN container
target/release/lzan d <input> <output>  # extract an LZAN container
target/release/lzan rt <file>            # verify a round trip
```

The binary also provides per-format commands such as `zx0c` and `upkrc` for
writing raw streams. Run `lzan` without arguments to see the complete command
list.

## C64 decrunchers

`decrunchers/` contains 6502/6510 routines in asm6502 syntax. Each routine has
a small metadata header used by the `lzan-c64` crate. The crate can generate a
relocated routine or a self-extracting C64 program. See
[`lzan-c64/README.md`](lzan-c64/README.md) and
[`decrunchers/README.md`](decrunchers/README.md).

## Build and test

```text
cargo test
cd lzan-c64
cargo test
```

The root crate and `lzan-c64` are separate Cargo workspaces.

## License and attribution

LZAN is distributed under the [MIT License](LICENSE). Attribution and applicable
third-party license texts for the included format implementations and decoder
sources are in [THIRD_PARTY.md](THIRD_PARTY.md) and [`licenses/`](licenses/).
