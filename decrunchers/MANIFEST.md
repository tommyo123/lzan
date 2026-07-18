# Decruncher collection catalog

Every 6502/6510 decruncher in this directory, one per compression format, in asm6502
syntax. Each format has a baseline forward decoder and, where applicable, backward
(in-place), legal-only (no undocumented opcodes), size, and speed variants. Every file
carries the `;@...` machine-readable header consumed by `lzan-c64::registry` and an
overridable `; ---- config-defaults ----` block; see [README.md](README.md) for the
contract.

`code bytes` is the assembled decruncher body only (`origin..comp_data`, payload
excluded). The metadata is consumed by `lzan-c64` when it selects and configures
a decoder for a generated program.

## Formats

| Format | Baseline decoder | Author | License | Bytes |
|--------|------------------|--------|---------|------:|
| ZX02 | `zx02-small-dmsc.s` | DMSC (Daniel Serpell) | MIT | 121 |
| LZSA1 | `lzsa1-marty-small.s` | Emmanuel Marty | Zlib | 165 |
| LZSA2 | `lzsa2-marty-small.s` | Emmanuel Marty | Zlib | 240 |
| ZX0 | `zx0-negativecharge-acorn.s` | Einar Saukas (format), XXL Dudek / NegativeCharge (port) | BSD-3 | 183 |
| apultra | `aplib-apultra-brandwood-6502.s` | John Brandwood | Boost-1.0 | 225 |
| TSCrunch | `tscrunch-savon.s` | Antonio Savona | Apache-2.0 | 160 |
| ByteBoozer2 | `byteboozer2-difraia.s` | Luigi Di Fraia (B2 by David Malmborg) | MIT | 175 |
| Exomizer | `exomizer-lind-mem-forward.s` | Magnus Lind | Zlib | 276 |
| upkr | `upkr-pfusik.s` | Piotr Fusik | Zlib | 222 |
| Shrinkler | `shrinkler-atari8xxl-unshrinkler.s` | XXL Dudek and Piotr Fusik (6502 port of Aske Simon Christensen's Shrinkler) | Zlib | 323 |
| Subsizer | `subsizer-tlr-standalone.s` | Daniel Kahlin ("tlr") | BSD | 225 |
| PuCrunch | `pucrunch-lzan.s` | clean-room (pucrunch format by Pasi Ojala) | MIT | 279 |
| LZAN minimal | `lzan-decoder-min.s` | this project | MIT | 181 |
| LZAN full | `lzan-decoder-full.s` | this project | MIT | 315 |

The TSCrunch collection also includes `tscrunch-negativecharge-beebasm-extreme.s`, a
BeebAsm/Acorn port of the same format (261 bytes).

## Variants

- Backward / in-place: every format ships a backward counterpart (write head trails read
  head, so packed and unpacked data can overlap and a file can be decrunched over itself).
  Subsizer is backward-native and instead has a forward variant.
- Legal-only (`-legal`): the four formats whose baseline decoder uses undocumented opcodes
  (LZSA1, LZSA2, TSCrunch, upkr; the ops `LAX (zp),Y` and `ALR #imm`) also ship a
  legal-only decoder in both directions that expands those to documented instruction
  pairs, reproducing A, X, Z, N and C exactly, so the decode is byte-identical. Any packer
  can pick legal or illegal freely.
- `-opt-size`: for a few formats, a smaller but slower decoder that de-inlines shared
  bit/byte-fetch routines (apultra, Subsizer, ByteBoozer2, TSCrunch, Shrinkler).
- `-opt-speed`: for the two LZAN decoders, a larger but faster decoder that inlines the hot
  bit reader.
- `pucrunch-lzan-zpstack.s`: an extra-small forward pucrunch body (211 bytes) meant for the
  `$0100` stack-page slot; the generator selects it when its staged blob fits there and
  falls back to the baseline otherwise.

## Notes

- The forward and backward `exomizer-lind-mem-*.s` decoders carry comment-based gate
  markers (`;>>> gate`, `;=== else`, `;g`, `;<<< gate`) that the assembler ignores.
  lzan-c64's `decoder_gates` / `decoder_tailoring` modules compose a trimmed body from a
  stream's measured traits: a stream with no literal sequences drops the `exit_or_lit_seq`
  handler. The stream bytes are unchanged, still standard `exomizer raw`.
- `pucrunch-lzan*.s` are clean-room implementations written from the pucrunch token
  grammar; no original pucrunch source was consulted. The one illegal op in the standard
  body (`SBX #$FF`) is expanded to `TAX`/`INX` in the `-legal` twin.
