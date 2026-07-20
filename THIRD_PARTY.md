# Third-party attribution

The Rust codecs are implementations of the formats listed below. The 6502/6510
sources in `decrunchers/` retain the attribution and license information for
their upstream works. Full license texts are in [`licenses/`](licenses/).

| Format or source family | Attribution | License |
|---|---|---|
| ZX0 | Einar Saukas; 6502 port by NegativeCharge from work by Krzysztof "XXL" Dudek | BSD 3-Clause |
| ZX02 | Daniel Serpell (DMSC) | MIT |
| LZSA1 / LZSA2 | Emmanuel Marty; faster LZSA1 decoder by John Brandwood | Zlib; BSL-1.0 |
| Exomizer | Magnus Lind | Zlib |
| upkr | exoticorn; 6502 decoder by Piotr Fusik | Unlicense and Zlib |
| ByteBoozer2 | Luigi Di Fraia, HCL, and David Malmborg | MIT |
| TSCrunch | Antonio Savona | Apache-2.0 |
| aPLib / apultra | John Brandwood, Emmanuel Marty, and Peter Ferrie | BSL-1.0 and Zlib |
| Shrinkler | Aske Simon Christensen; 6502 decoder by Krzysztof Dudek and Piotr Fusik | Shrinkler license and Zlib |
| Subsizer | Daniel Kahlin | BSD-style |
| PuCrunch format | Pasi Ojala | The implementation in this repository is MIT-licensed |

`asm6502/` is included as a local dependency of `lzan-c64` and retains its own
[license](asm6502/LICENSE).

The LZAN format and its 6502 decoders are distributed under this repository's
[MIT License](LICENSE).
