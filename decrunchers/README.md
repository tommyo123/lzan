# 6502/6510 decrunchers

This directory contains 6502/6510 decrunch routines in asm6502 syntax. The
`lzan-c64` crate reads their metadata and emits configured routines or complete
C64 programs. The available routines are listed in [MANIFEST.md](MANIFEST.md).

## Metadata

Every source file starts with a machine-readable header. The `;@...` entries
identify the format, direction, variant, entry point, required symbols, and
memory requirements.

```asm
;@format: zx02
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 10
;@scratch: none
;@illegal: no
;@smc: no
```

The block between `; ---- config-defaults ----` markers defines default
zero-page and scratch locations. `lzan-c64` replaces that block when a routine
is relocated.

## Manual use

To assemble a routine directly, define the symbols named by its `;@needs:`
entry, provide the packed data at `comp_data`, and call the `;@entry:` label.
The default configuration block allows each source file to assemble without
additional relocation settings.
