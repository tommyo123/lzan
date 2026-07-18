//! Dump the generated SFX assembly source for one format/direction, plus a
//! per-section byte accounting, a tool for inspecting boot-glue overhead.
//!
//! Usage: cargo run --example dump_sfx_source -- <format> <forward|backward> <in.prg>

use lzan_c64::{compress_for, pick_routine, Decruncher, Direction, Format};

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let (fmt_s, dir_s, path) = (&a[0], &a[1], &a[2]);
    let all = [
        Format::Zx02,
        Format::Zx0,
        Format::Lzsa1,
        Format::Lzsa2,
        Format::Aplib,
        Format::TsCrunch,
        Format::ByteBoozer2,
        Format::Exomizer,
        Format::Shrinkler,
        Format::Subsizer,
        Format::Upkr,
        Format::PuCrunch,
        Format::LzanMin,
        Format::LzanFull,
    ];
    let format = *all
        .iter()
        .find(|f| format!("{f:?}").eq_ignore_ascii_case(fmt_s))
        .expect("unknown format");
    let direction = match dir_s.as_str() {
        "forward" => Direction::Forward,
        "backward" => Direction::Backward,
        _ => panic!("direction"),
    };

    let bytes = std::fs::read(path).expect("read prg");
    let load = u16::from_le_bytes([bytes[0], bytes[1]]);
    let data = &bytes[2..];
    let out_end = load.wrapping_add(data.len() as u16);
    let (stream, mode_byte) = compress_for(format, direction, data, Some(out_end)).expect("pack");
    let clen = stream.len();

    let routine = pick_routine(format, direction, true).expect("routine");
    let stage_at = if routine.code_bytes as u32 + 16 <= 0xE0 { 0x0100 } else { 0x0400 };
    let mut b = Decruncher::with_variant(format, direction, routine.variant)
        .expect("decr")
        .basic_stub()
        .packed_inline(stream)
        .output(load)
        .output_len(data.len() as u16)
        .stage_decruncher_at(stage_at)
        .jmp_when_done(0x080D);
    if Decruncher::with_variant(format, direction, routine.variant)
        .expect("decr")
        .spec()
        .scratch
        .is_some()
    {
        b = b.scratch_address(0x0600);
    }
    b = match direction {
        Direction::Forward => b.move_packed_to_top(0xFFFF),
        Direction::Backward => b.move_packed_to(load.saturating_sub(0x20)).mover_at(0x0334),
    };
    b = b.all_ram_with(0x30, None);
    if let Some(m) = mode_byte {
        b = b.mode_byte(m);
    }

    let src = b.program_source().expect("source");
    let built = b.assemble().expect("assemble");
    println!("{src}");
    eprintln!(
        "== stream {} B, total {} B (origin ${:04X}), glue = {} B",
        clen,
        built.bytes.len(),
        built.origin,
        built.bytes.len() - clen
    );
}
