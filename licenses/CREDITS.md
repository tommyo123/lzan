# Credits

This project ships independent reimplementations of established compression
formats, plus 6502/6510 decrunchers ported from published originals. Every
upstream work is credited below, and its license text is in this directory
(`LICENSE-<format>.md`). The LZAN container and the `lzan-decoder` /
`pucrunch-lzan` decoders are original work under the project's MIT license
(`../LICENSE`).

## Decrunchers and their upstream works

Each decoder source contains its upstream attribution in the header.

| Decruncher family | Upstream work | Author(s) | License | Text |
|---|---|---|---|---|
| `zx0-negativecharge-acorn` | BeebAsm ZX0 decoder (port of XXL Dudek's decoder); ZX0 format | NegativeCharge (port); Krzysztof "XXL" Dudek (decoder); Einar Saukas (format) | BSD-3-Clause (format); decoder is attribution-only, no LICENSE file | `LICENSE-zx0.md` |
| `zx02-small-dmsc` | zx02 6502 decoder | Daniel Serpell (DMSC) | MIT | `LICENSE-zx02.md` |
| `lzsa1-marty` | `decompress_small_v1.asm` | Emmanuel Marty | Zlib | `LICENSE-lzsa.md` |
| `lzsa2-marty` | `decompress_small_v2.asm` | Emmanuel Marty | Zlib | `LICENSE-lzsa.md` |
| `exomizer-lind-mem` | `exodecrunch.s` | Magnus Lind | Zlib | `LICENSE-exomizer.md` |
| `upkr-pfusik` | `unupkr.asx` (decoder); upkr format | Piotr Fusik (decoder); exoticorn (format) | Zlib (decoder); Unlicense (format) | `LICENSE-upkr.md` |
| `byteboozer2-difraia` | ByteBoozer2 `Decruncher.inc` | Luigi Di Fraia (2018); HCL (decruncher, 2003); David Malmborg (B2 format, 2014) | MIT | `LICENSE-byteboozer2.md` |
| `tscrunch-savon` | TSCrunch `decrunch.asm` | Antonio Savona | Apache-2.0 | `LICENSE-tscrunch.md` |
| `tscrunch-negativecharge-beebasm-extreme` | BeebAsm/Acorn port of TSCrunch | NegativeCharge (port); Antonio Savona (format) | Apache-2.0 (format); port has no separate LICENSE file | `LICENSE-tscrunch.md` |
| `aplib-apultra-brandwood-6502` | `aplib_6502.asm` (forward) | John Brandwood (2019) | BSL-1.0 | `LICENSE-apultra.md` |
| `aplib-apultra-marty` | `aplib_6502_b.asm` (backward) | Emmanuel Marty (2020), parts after John Brandwood and Peter Ferrie | Zlib | `LICENSE-apultra.md` |
| `shrinkler-atari8xxl-unshrinkler` | unShrinkler (6502 port of Shrinkler) | Krzysztof "XXL" Dudek and Piotr Fusik (decoder, 2021); Aske Simon Christensen (Shrinkler) | Zlib (decoder); Shrinkler license (format) | `LICENSE-shrinkler.md` |
| `subsizer-tlr-standalone` | Subsizer + `standalone/decrunch_normal.asm` | Daniel Kahlin ("tlr") | BSD-style permissive | `LICENSE-subsizer.md` |
| `lzan-decoder` | original | LZAN project | MIT | `../LICENSE` |
| `pucrunch-lzan` | original, clean-room | LZAN project | MIT | `../LICENSE` |

Every family above also has `-backward`, `-opt-size`, `-opt-speed`, `-legal`, and
`-zpstack` variants; each carries the same upstream credit as its base family.

## Contributors credited

* Einar Saukas: ZX0 format
* Krzysztof "XXL" Dudek: ZX0 6502 decoder, unShrinkler
* NegativeCharge: BeebAsm/Acorn 6502 ports (ZX0, TSCrunch)
* Daniel Serpell (DMSC): ZX02
* Emmanuel Marty: LZSA1/LZSA2, apultra, aPLib backward 6502 decoder
* Magnus Lind: Exomizer
* exoticorn: upkr format
* Piotr Fusik: unupkr, unShrinkler
* Luigi Di Fraia: ByteBoozer2 packaging and release
* HCL: ByteBoozer2 6502 decruncher (2003)
* David Malmborg: ByteBoozer 2.0 format (2014)
* Antonio Savona: TSCrunch
* John Brandwood: aPLib 6502 forward decoder
* Peter Ferrie: aPLib decoder contributions
* Aske Simon Christensen: Shrinkler
* Daniel Kahlin ("tlr"): Subsizer

## PuCrunch

The PuCrunch codec (`src/pucrunch.rs`) and the `pucrunch-lzan` 6502 decoders
are distributed under the project's MIT license.
