# Third-party licenses

This directory contains license texts and detailed credits for the compression
format implementations and 6502 decoder sources included with LZAN. See
[`../THIRD_PARTY.md`](../THIRD_PARTY.md) for the summary.

| Format | Modules / decrunchers | License | File |
|---|---|---|---|
| ZX0 | `zx0opt`, `zx0compat`; `zx0-negativecharge-acorn` | BSD-3-Clause (Einar Saukas) | LICENSE-zx0.md |
| ZX02 | `zx02`; `zx02*` | MIT (Daniel Serpell) | LICENSE-zx02.md |
| LZSA1/2 | `lzsa1`, `lzsa2`; `lzsa*-marty` | Zlib (Emmanuel Marty) + BSL-1.0 faster LZSA1 decoder (`lzsa1-brandwood-faster`, John Brandwood) | LICENSE-lzsa.md |
| Exomizer | `exo3`; `exomizer-lind-mem` | Zlib (Magnus Lind) | LICENSE-exomizer.md |
| upkr | `upkr`; `upkr-pfusik` | Unlicense (exoticorn) + Zlib decoder (Piotr Fusik) | LICENSE-upkr.md |
| ByteBoozer2 | `bb2`; `byteboozer2-difraia` | MIT (Luigi Di Fraia) | LICENSE-byteboozer2.md |
| TSCrunch | `tscrunch`; `tscrunch-savon` | Apache-2.0 (Antonio Savona) | LICENSE-tscrunch.md |
| aPLib / apultra | `apultra`; `aplib-apultra-*` | Zlib (Marty) + BSL-1.0 forward decoder (Brandwood) | LICENSE-apultra.md |
| Shrinkler | `shrinkler`; `shrinkler-atari8xxl-unshrinkler` | Shrinkler license (Christensen) + Zlib decoder (Dudek/Fusik) | LICENSE-shrinkler.md |
| Subsizer | `subsizer`; `subsizer-tlr` | BSD-style permissive (Daniel Kahlin) | LICENSE-subsizer.md |

The LZAN container itself, the `lzan-decoder` / `pucrunch-lzan` 6502 decoders,
and the project's own `bolt` (BoltLZ) format and its 6502
decoders are original work under the project's MIT license (see `../LICENSE`).

The PuCrunch codec and the `pucrunch-lzan` decoders are distributed under the
project's MIT license.
