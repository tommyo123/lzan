# lzan-c64

`lzan-c64` generates configurable 6502 decrunch routines and self-extracting
C64 programs in asm6502 syntax. It uses the decoder collection in
[`../decrunchers/`](../decrunchers/).

## Choosing a routine

```rust
use lzan_c64::{all_routines, Decruncher, Direction, Format, Variant};

let standard = Decruncher::new(Format::Zx02, Direction::Forward)?;
let compact = Decruncher::with_variant(
    Format::Subsizer,
    Direction::Backward,
    Variant::OptSize,
)?;

for routine in all_routines() {
    println!("{routine}");
}
```

`upkr` and `lzan-min` are available only for forward decoding.

## Configuration

```rust
let program = standard
    .code_address(0x0801)
    .scratch_address(0x0740)
    .pack(&data)
    .output(0x4000)
    .output_len(0x2000);
```

The builder can embed compressed data, reference an external payload, add a
BASIC stub, control C64 memory banking, move packed data, stage a decruncher,
and set the completion address.

Each routine's default `zero_page()` base is placed so BASIC and the KERNAL
survive the decrunch; the span is checked against a C64 zero-page map
(`ZpClass`, `regions_hit`) when control returns to BASIC. `run_basic_when_done()`
starts a decrunched BASIC program (relink, set `VARTAB`, `CLR`, enter the
interpreter loop), which a plain `jmp_when_done()` cannot do.

## Output

```rust
let source = program.program_source()?;
program.write_source("selfx.s")?;

let assembled = program.assemble()?;
let prg = program.prg()?;
program.write_prg("selfx.prg")?;
```

`routine_source()` returns the configured routine, while `program_source()`
returns the complete assembly program. See the public API documentation for
the full set of builder options.

## Build and test

```text
cargo test
cargo run --example dump_sfx_source
```
