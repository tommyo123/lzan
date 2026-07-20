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
| BoltLZ | `bolt.s` | this project | MIT | 97 |

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
- `-opt-speed`: a larger-but-faster decoder, selected by the `lzan-c64` speed-priority flag
  (`Decruncher::priority_speed()` / `registry::pick_speed_routine`); the default stays on the
  balanced `standard` variant, and formats with no opt-speed variant fall back to it. Present today
  for the two LZAN decoders (inline the hot bit reader), for **LZSA1**
  (`lzsa1-brandwood-faster.s` - John Brandwood's `decompress_faster_v1`, Boost-1.0; 191 B = the
  upstream size, legal opcodes), for **TSCrunch** (the parity-split "extreme" decoder
  `tscrunch-negativecharge-beebasm-extreme.s`), and for **BoltLZ** in both directions
  (`bolt-opt-speed.s`, 147 B forward, and `bolt-opt-speed-backward.s`, 219 B backward): the standard
  decoder with its per-command pointer advances inlined (no JSR/RTS overhead) and
  2-byte-per-iteration copy loops. The smaller `bolt.s`/`bolt-backward.s` remain the size/default
  choice.
- `;@seed: caller` marks a **caller-seeded** decoder: it has no seed preamble and expects the caller
  to seed its zero-page pointers before entry - `[zp_base+0..1] = comp_data` (source),
  `[zp_base+2..3] = out_addr` (destination). The `lzan-c64` generator emits one **shared** seed for
  all such decoders (`Decruncher` does this automatically), so the decoder body matches its upstream
  size (`bolt.s` 97 B, `lzsa1-brandwood-faster.s` 191 B). Seeding is one-time and off the hot path,
  so decode speed is unchanged; self-seeding decoders (the default) carry their own ZP-init or
  baked-in SMC operands.
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
- `bolt.s` (BoltLZ) is this project's **purely byte-oriented** decoder: it contains no bit reader at
  all - every field is whole bytes - trading ratio for a small (97 B), fast decoder; it is smaller
  than the other byte-oriented decoders in this collection. It is a sign-bit-dispatch LZ77 (token
  `$00`=EOF, `$01..$7F`=literal run, `$80..$FF`=match + 2-byte offset), legal-opcode-only, encoded by
  `lzan::bolt`. The forward decoders (`bolt.s` 97 B, `bolt-opt-speed.s` 147 B) are caller-seeded; the
  backward / in-place decoders (`bolt-backward.s` 174 B, `bolt-opt-speed-backward.s` 219 B) self-seed
  their end pointers and copy DESCENDING from `dst + d` (the overlap/RLE-safe direction downward), so
  a BoltLZ file can also be decrunched over itself on a full 64 KB span.
