//! # lzan-c64 - C64 decrunch framework
//!
//! Emits configurable 6502 decrunch routines / complete self-extracting
//! programs in [asm6502](https://github.com/tommyo123/asm6502) format, from
//! the curated collection in the repo-root `decrunchers/` directory.
//!
//! Design principles: payloads are dynamic
//! (caller-chosen src/dst), machinery defaults to well-known places
//! (`$0334` tables, `$0100` staging, page-aligned probability RAM), backward
//! decoders enable in-place decrunching, and the classic SFX layout is
//! `$0801` BASIC stub → bank out ROM → move payload high → decrunch → `JMP`.
//!
//! ## Example: self-extracting program
//!
//! ```no_run
//! use lzan_c64::{Decruncher, Direction, Format};
//!
//! let data = std::fs::read("bitmap.bin").unwrap();
//! let prg = Decruncher::new(Format::Zx02, Direction::Forward).unwrap()
//!     .basic_stub()                  // 0 SYS2061 autostart
//!     .all_ram()                     // SEI + $01=$34: all 64K visible
//!     .pack(&data)                   // compress + embed payload
//!     .output(0x4000)                // decrunch to $4000
//!     .prg()                         // [$01,$08] + assembled program
//!     .unwrap();
//! std::fs::write("bitmap.prg", prg).unwrap();
//! ```
//!
//! ## Example: Exomizer-style staging
//!
//! Move the packed blob down to `$0400`, run the decruncher from the stack
//! page, decrunch backward over the program's own load address:
//!
//! ```no_run
//! use lzan_c64::{Decruncher, Direction, Format};
//!
//! let data = vec![0u8; 0x2000];
//! let src = Decruncher::new(Format::Zx02, Direction::Backward).unwrap()
//!     .basic_stub()
//!     .pack(&data)
//!     .move_packed_to(0x0400)        // payload out of harm's way first
//!     .stage_decruncher_at(0x0100)   // decoder runs from the stack page
//!     .output(0x0801)                // unpack over the program itself
//!     .jmp_when_done(0x0801)         // machine-code payload: enter at $0801
//!     .program_source()              // asm6502 source, straight from RAM
//!     .unwrap();
//! ```
//!
//! ## Example: self-extracting BASIC program
//!
//! [`Decruncher::run_basic_when_done`] relinks the program, sets `VARTAB`, runs
//! `CLR` and enters the interpreter loop - a plain `JMP` cannot start BASIC:
//!
//! ```no_run
//! use lzan_c64::{Decruncher, Direction, Format};
//!
//! let basic = std::fs::read("game.prg").unwrap();  // a tokenized BASIC program
//! let prg = Decruncher::new(Format::Zx02, Direction::Backward).unwrap()
//!     .basic_stub()
//!     .pack(&basic[2..])             // drop the $01,$08 load address
//!     .move_packed_to(0x0400)
//!     .stage_decruncher_at(0x0100)
//!     .output(0x0801)                // BASIC text goes where TXTTAB points
//!     .run_basic_when_done()
//!     .prg()
//!     .unwrap();
//! # let _ = prg;
//! ```

mod builder;
mod decoder_gates;
mod decoder_tailoring;
mod emit;
mod registry;
mod zp_safety;

pub use builder::{
    Decruncher, Done, GenError, Issue, MoveData, MoveSrc, Packed, Severity,
};
pub use emit::{compress_for, Built};
pub use decoder_tailoring::{tailored_body, DecoderTailoring};
pub use registry::{
    all_routines, find_routine, pick_routine, pick_speed_routine, pick_zp_stack_routine, Direction,
    EofKind, Format, Needs, PayloadAbi, RoutineSpec, ScratchSpec, Variant, CONFIG_BLOCK_BEGIN,
    CONFIG_BLOCK_END,
};
pub use zp_safety::{first_safe_base, regions_hit, ZpClass, ZpRegion};
/// PuCrunch in-place safety metrics (see `lzan::pucrunch`): callers placing a
/// PuCrunch container for in-place decode must verify the write head cannot
/// reach unread stream bytes - the format's escaped literals locally EXPAND,
/// so fixed layout margins are not automatically sufficient.
pub use lzan::pucrunch::{container_max_gap, container_max_gap_backward};
/// apultra (aPLib) in-place safety margins (see `lzan::apultra`): apultra
/// literals cost 9 bits, so an incompressible run decoded late makes the running
/// compression peak above its final value. A fixed in-place margin is then too
/// small and the decoder's write head clobbers unread compressed bytes. Callers
/// placing an apultra stream in-place must size the margin (backward) / top gap
/// (forward) to at least these values.
pub use lzan::apultra::{
    max_gap_backward as aplib_gap_backward, max_gap_forward as aplib_gap_forward,
};
/// ByteBoozer2 in-place safety margins (see `lzan::bb2`): identical in kind to
/// apultra's - the bit-oriented LZ stream can be momentarily larger than the
/// output it has produced, so an incompressible run decoded late overruns a
/// fixed margin. (This is the same quantity ByteBoozer2's `compute_margin`
/// derives at pack time, but measured from the compressed stream.)
pub use lzan::bb2::{
    max_gap_backward as bb2_gap_backward, max_gap_forward as bb2_gap_forward,
};
/// upkr and Subsizer in-place safety margins - same class as apultra/bb2 (their
/// streams can be momentarily larger than the output decoded so far, so an
/// incompressible run decoded late overruns a fixed in-place margin).
pub use lzan::subsizer::{
    max_gap_backward as subsizer_gap_backward, max_gap_forward as subsizer_gap_forward,
};
pub use lzan::upkr::{
    max_gap_backward as upkr_gap_backward, max_gap_forward as upkr_gap_forward,
};
/// lzan-min (own minimal-EOF codec) in-place safety margins.
pub use lzan::zx::{
    max_gap_min_backward as lzan_min_gap_backward, max_gap_min_forward as lzan_min_gap_forward,
};
/// In-place safety margins for the remaining forward/backward in-place formats.
pub use lzan::exo3::{max_gap_backward as exo_gap_backward, max_gap_forward as exo_gap_forward};
pub use lzan::lzsa1::{
    max_gap_backward as lzsa1_gap_backward, max_gap_forward as lzsa1_gap_forward,
};
pub use lzan::lzsa2::{
    max_gap_backward as lzsa2_gap_backward, max_gap_forward as lzsa2_gap_forward,
};
pub use lzan::zx0compat::{
    max_gap_backward as zx0_gap_backward, max_gap_forward as zx0_gap_forward,
};
pub use lzan::zx02::{max_gap_backward as zx02_gap_backward, max_gap_forward as zx02_gap_forward};
/// BoltLZ in-place safety margins (see `lzan::bolt`): BoltLZ is byte-aligned
/// (whole-byte literals and offsets), so - like LZSA1 - the running compression
/// never balloons far above the output produced, and the in-place gap is only a
/// handful of bytes. Without this the workshop's placer must conservatively
/// reject BoltLZ forward whenever the packed stream overlaps the output.
pub use lzan::bolt::{max_gap_backward as bolt_gap_backward, max_gap_forward as bolt_gap_forward};
/// TSCrunch in-place layout safety (see `lzan::tscrunch`): how many bytes
/// ABOVE the end-aligned reference position the packed stream must be placed
/// so the 6502 decoder's per-token write overshoot (descending literal copy,
/// RLE/LZ runs) never touches unread stream bytes. 0 = the reference
/// `tscrunch -p -i` end-alignment is already safe. The 6502 routine takes
/// explicit comp_data/out_addr symbols (the embedded load_to is ignored) and
/// its tail copy is ascending, so an upward shift is sound.
pub use lzan::tscrunch::inplace_required_shift as tscrunch_inplace_shift;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_parses_all_47() {
        let all = all_routines();
        assert_eq!(all.len(), 52, "expected 52 curated routines");
        // Every (format, direction, variant) triple is unique.
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert!(
                    !(a.format == b.format && a.direction == b.direction && a.variant == b.variant),
                    "duplicate routine key: {a} vs {b}"
                );
            }
        }
    }

    #[test]
    fn registry_entry_labels_exist_in_source() {
        for r in all_routines() {
            assert!(
                r.source.contains(&format!("{}:", r.entry)),
                "{}: entry label {} not found",
                r.file,
                r.entry
            );
        }
    }

    #[test]
    fn priority_speed_picks_opt_speed_when_present_else_falls_back() {
        // LzanMin has a dedicated opt-speed decoder: the speed picker and the
        // builder flag must select it.
        let sp = pick_speed_routine(Format::LzanMin, Direction::Forward, true)
            .expect("lzan-min forward exists");
        assert_eq!(sp.variant, Variant::OptSpeed, "speed picker should choose opt-speed");
        let d = Decruncher::new(Format::LzanMin, Direction::Forward)
            .unwrap()
            .priority_speed();
        assert_eq!(d.spec().variant, Variant::OptSpeed, "builder flag should choose opt-speed");

        // TSCrunch's faster "extreme" decoder is the opt-speed variant.
        let ts = pick_speed_routine(Format::TsCrunch, Direction::Forward, true)
            .expect("tscrunch forward exists");
        assert_eq!(ts.variant, Variant::OptSpeed, "tscrunch speed pick = extreme decoder");

        // A format with no opt-speed variant falls back to the balanced baseline,
        // and the flag is a safe no-op.
        let sp2 = pick_speed_routine(Format::Zx02, Direction::Forward, true)
            .expect("zx02 forward exists");
        assert_eq!(sp2.variant, Variant::Standard, "no opt-speed -> Standard baseline");
        let d2 = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .priority_speed();
        assert_eq!(d2.spec().variant, Variant::Standard, "flag is a no-op when no opt-speed exists");
    }

    #[test]
    fn no_default_zp_span_touches_persistent_zero_page() {
        // A default span may sit on state the KERNAL re-derives, but never on
        // state that only a reset restores.
        for r in all_routines() {
            if r.zp_len == 0 {
                continue;
            }
            let base = r.zp_base_default.expect("zp_len > 0 implies a default base");
            let bad: Vec<_> = regions_hit(base, r.zp_len)
                .into_iter()
                .filter(|x| x.class == ZpClass::Persistent)
                .map(|x| x.name)
                .collect();
            assert!(
                bad.is_empty(),
                "{r}: default zp_base ${base:02X}+{} hits {bad:?}",
                r.zp_len
            );
        }
    }

    #[test]
    fn zp_span_over_chrget_is_rejected_when_basic_must_survive() {
        // $80-$89 lands inside CHRGET ($73-$8A), which BASIC cannot survive.
        let err = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .output(0x4000)
            .zero_page(0x80)
            .build();
        match err {
            Err(GenError::Validation(v)) => assert!(
                v.iter().any(|i| i.severity == Severity::Error && i.msg.contains("CHRGET")),
                "expected a CHRGET error, got {v:?}"
            ),
            other => panic!("expected validation failure, got {other:?}"),
        }
        // The same span is only a warning when the payload takes the machine.
        let built = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .all_ram()
            .pack(&sample_input())
            .output(0x4000)
            .zero_page(0x80)
            .jmp_when_done(0x4000)
            .build();
        assert!(built.is_ok(), "Jmp payload should not be blocked: {built:?}");
    }

    #[test]
    fn run_basic_emits_relink_and_interpreter_entry() {
        // The classic BASIC SFX layout: payload down to $0400, decoder on the
        // stack page, decrunch backward over $0801, then start the program.
        // The tail lands inside the staged blob, so check the assembled bytes.
        let built = Decruncher::new(Format::Zx02, Direction::Backward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .move_packed_to(0x0400)
            .stage_decruncher_at(0x0100)
            .output(0x0801)
            .run_basic_when_done()
            .assemble()
            .unwrap();
        // The relink stops at the end-of-program link, so VARTAB is $22 + 2.
        let tail = [
            0x20, 0x33, 0xA5, // JSR $A533
            0xA5, 0x22, 0x18, 0x69, 0x02, 0x85, 0x2D, // LDA $22 / CLC / ADC #$02 / STA $2D
            0xA5, 0x23, 0x69, 0x00, 0x85, 0x2E, // LDA $23 / ADC #$00 / STA $2E
            0x20, 0x59, 0xA6, // JSR $A659
            0x4C, 0xAE, 0xA7, // JMP $A7AE
        ];
        assert!(
            built.bytes.windows(tail.len()).any(|w| w == tail),
            "relink + interpreter entry missing from the assembled program"
        );
    }

    #[test]
    fn run_basic_needs_the_basic_rom_banked_in() {
        let err = Decruncher::new(Format::Zx02, Direction::Backward)
            .unwrap()
            .basic_stub()
            .all_ram() // $01 = $34, never restored
            .pack(&sample_input())
            .move_packed_to(0x0400)
            .stage_decruncher_at(0x0100)
            .output(0x0801)
            .run_basic_when_done()
            .build();
        match err {
            Err(GenError::Validation(v)) => assert!(
                v.iter().any(|i| i.severity == Severity::Error && i.msg.contains("banked in")),
                "expected a banking error, got {v:?}"
            ),
            other => panic!("expected validation failure, got {other:?}"),
        }
    }

    #[test]
    fn scratch_over_the_program_image_is_rejected() {
        let err = Decruncher::new(Format::Shrinkler, Direction::Forward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .output(0x4000)
            .scratch_address(0x0800) // page-aligned, but on top of the program
            .build();
        match err {
            Err(GenError::Validation(v)) => assert!(
                v.iter().any(|i| i.severity == Severity::Error
                    && i.msg.contains("scratch region")
                    && i.msg.contains("program image")),
                "expected a scratch/program overlap error, got {v:?}"
            ),
            other => panic!("expected validation failure, got {other:?}"),
        }
    }

    #[test]
    fn zero_page_range_too_small_is_rejected() {
        let e = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .zero_page_range(0xFA, 0xFE); // 5 bytes, zx02 needs 10
        assert!(e.is_err());
    }

    #[test]
    fn unknown_routine_is_rejected() {
        // Every format now has forward+backward Standard routines (upkr and
        // lzan-min gained backward variants), so probe a missing VARIANT.
        assert!(
            Decruncher::with_variant(Format::Zx02, Direction::Forward, Variant::OptSize).is_err()
        );
    }

    fn sample_input() -> Vec<u8> {
        // Something compressible but non-trivial.
        (0u32..3000).map(|i| ((i / 7) ^ (i / 13)) as u8).collect()
    }

    #[test]
    fn sfx_program_assembles_with_stub_and_start() {
        let built = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .all_ram()
            .pack(&sample_input())
            .output(0x4000)
            .assemble()
            .unwrap();
        assert_eq!(built.origin, 0x0801);
        // BASIC stub: next-line ptr $080B, line 0, $9E "2061", EOL, end.
        assert_eq!(
            &built.bytes[..12],
            &[0x0B, 0x08, 0x00, 0x00, 0x9E, 0x32, 0x30, 0x36, 0x31, 0x00, 0x00, 0x00]
        );
        assert_eq!(built.symbols["start"], 0x080D);
        // prg() prepends the $01 $08 load address.
        let prg = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .all_ram()
            .pack(&sample_input())
            .output(0x4000)
            .prg()
            .unwrap();
        assert_eq!(&prg[..2], &[0x01, 0x08]);
        assert_eq!(&prg[2..], &built.bytes[..]);
    }

    #[test]
    fn zp_override_lands_in_generated_source() {
        let src = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .pack(&sample_input())
            .output(0x4000)
            .zero_page(0x40)
            .program_source()
            .unwrap();
        assert!(src.contains("zp_base = $40"), "override missing:\n{src}");
    }

    #[test]
    fn output_over_program_requires_staging() {
        let d = Decruncher::new(Format::Zx02, Direction::Backward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .move_packed_to(0x0400)
            .output(0x0801); // over the program image
        let err = d.clone().build();
        assert!(matches!(err, Err(GenError::Validation(_))), "expected staging error");
        // With staging it builds.
        let built = d.stage_decruncher_at(0x0100).build().unwrap();
        assert!(built.symbols.contains_key("blob_staged_decruncher"));
    }

    #[test]
    fn missing_output_len_for_backward_is_rejected() {
        // packed_inline gives no out_len; backward decoders need it.
        let stream = compress_for(Format::Zx02, Direction::Backward, &sample_input(), None)
            .unwrap()
            .0;
        let err = Decruncher::new(Format::Zx02, Direction::Backward)
            .unwrap()
            .packed_inline(stream)
            .output(0x4000)
            .build();
        assert!(err.is_err());
    }

    #[test]
    fn every_standard_forward_routine_generates_and_assembles() {
        use Format::*;
        let input = sample_input();
        for format in [
            Zx02, Zx0, Lzsa1, Lzsa2, Aplib, TsCrunch, ByteBoozer2, Exomizer, Shrinkler,
            Subsizer, Upkr, PuCrunch, LzanMin, LzanFull,
        ] {
            let built = Decruncher::new(format, Direction::Forward)
                .unwrap()
                .pack(&input)
                .output(0x8000)
                .assemble()
                .unwrap_or_else(|e| panic!("{format:?}: {e}"));
            assert!(!built.bytes.is_empty(), "{format:?}: empty program");
        }
    }

    // ---- legal-opcode (no-illegal) variant coverage -----------------------

    /// The 14 formats that all have forward+backward Standard routines.
    const ALL_FORMATS: [Format; 14] = {
        use Format::*;
        [
            Zx02, Zx0, Lzsa1, Lzsa2, Aplib, TsCrunch, ByteBoozer2, Exomizer, Shrinkler, Subsizer,
            Upkr, PuCrunch, LzanMin, LzanFull,
        ]
    };

    /// Every format/direction must have a legal (no illegal opcodes) decoder:
    /// the already-legal Standard where possible, else the dedicated `Legal`
    /// variant. And `allow_illegal = true` must keep returning Standard.
    #[test]
    fn pick_routine_has_a_legal_decoder_for_every_format_and_direction() {
        for format in ALL_FORMATS {
            for direction in [Direction::Forward, Direction::Backward] {
                let legal = pick_routine(format, direction, false).unwrap_or_else(|| {
                    panic!("{format:?}/{direction:?}: no legal decoder")
                });
                assert!(
                    !legal.illegal,
                    "{format:?}/{direction:?}: pick_routine(false) returned an illegal routine ({legal})"
                );
                let std = find_routine(format, direction, Variant::Standard).unwrap();
                assert_eq!(
                    pick_routine(format, direction, true).unwrap().variant,
                    std.variant,
                    "{format:?}/{direction:?}: allow_illegal must pick Standard"
                );
            }
        }
    }

    /// The four formats whose Standard decoder uses illegal opcodes each gained
    /// a dedicated legal-only variant in both directions.
    #[test]
    fn illegal_formats_have_a_dedicated_legal_variant() {
        for format in
            [Format::Lzsa1, Format::Lzsa2, Format::TsCrunch, Format::Upkr, Format::PuCrunch]
        {
            for direction in [Direction::Forward, Direction::Backward] {
                let std = find_routine(format, direction, Variant::Standard).unwrap();
                assert!(std.illegal, "{format:?}/{direction:?}: Standard expected illegal");
                let legal = find_routine(format, direction, Variant::Legal).unwrap_or_else(|| {
                    panic!("{format:?}/{direction:?}: missing Legal variant")
                });
                assert!(!legal.illegal, "{format:?}/{direction:?}: Legal must be legal");
                // Expanding an illegal op to legal pairs never shrinks the body.
                assert!(
                    legal.code_bytes >= std.code_bytes,
                    "{format:?}/{direction:?}: legal {} < standard {}",
                    legal.code_bytes,
                    std.code_bytes
                );
            }
        }
    }

    /// The generator path (`with_variant(.., Legal)`) assembles a full forward
    /// SFX for the illegal formats - proving the legal sources wire up.
    #[test]
    fn legal_variant_generates_and_assembles_forward() {
        for format in
            [Format::Lzsa1, Format::Lzsa2, Format::TsCrunch, Format::Upkr, Format::PuCrunch]
        {
            let built = Decruncher::with_variant(format, Direction::Forward, Variant::Legal)
                .unwrap()
                .basic_stub()
                .all_ram()
                .pack(&sample_input())
                .output(0x8000)
                .assemble()
                .unwrap_or_else(|e| panic!("{format:?} legal: {e}"));
            assert!(!built.bytes.is_empty(), "{format:?}: empty program");
        }
    }

    // ---- regression tests for the reviewed findings -----------------------

    /// Exact-256 zero-page span (aplib $F7+9, exomizer $F7+9) must not panic in
    /// overflow-checked (debug) builds when rendering the header comment.
    #[test]
    fn zp_span_ending_at_ff_does_not_overflow() {
        for format in [Format::Aplib, Format::Exomizer] {
            let src = Decruncher::new(format, Direction::Forward)
                .unwrap()
                .pack(&sample_input())
                .output(0x8000)
                .program_source()
                .unwrap_or_else(|e| panic!("{format:?}: {e}"));
            assert!(src.contains("zp $F7-$FF"), "{format:?} header span wrong");
        }
    }

    /// A >256-byte decoder staged at $0100 spills past the stack page and must
    /// be rejected (not silently accepted).
    #[test]
    fn oversize_stage_at_0100_is_rejected() {
        let err = Decruncher::new(Format::Exomizer, Direction::Forward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .move_packed_to(0x0400)
            .stage_decruncher_at(0x0100)
            .output(0x2000)
            .build();
        match err {
            Err(GenError::Validation(v)) => assert!(
                v.iter().any(|i| i.msg.contains("stack headroom")),
                "wrong error: {v:?}"
            ),
            other => panic!("expected stack-headroom validation error, got {other:?}"),
        }
    }

    /// Forward stream-EOF format with an inline payload and NO output_len must
    /// still catch an output that starts on top of the program image.
    #[test]
    fn output_over_program_caught_without_out_len() {
        let stream = compress_for(Format::Zx02, Direction::Forward, &sample_input(), None)
            .unwrap()
            .0;
        let err = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .packed_inline(stream)
            .output(0x0801) // onto the program image, length unknown
            .build();
        assert!(matches!(err, Err(GenError::Validation(_))), "should reject, got {err:?}");
        // ...and into the stack page.
        let stream = compress_for(Format::Zx02, Direction::Forward, &sample_input(), None)
            .unwrap()
            .0;
        let err = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .packed_inline(stream)
            .output(0x0100)
            .build();
        assert!(matches!(err, Err(GenError::Validation(_))), "stack overlap should reject");
    }

    /// A move onto the program image with mover_at but NO staging must error
    /// (control would `JMP after_moves` into overwritten memory).
    #[test]
    fn move_over_program_needs_staging_too() {
        let err = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .packed_at(0xC000)
            .packed_len(400)
            .move_packed_to(0x0803)
            .mover_at(0x0334)
            .output(0x4000)
            .build();
        match err {
            Err(GenError::Validation(v)) => assert!(
                v.iter().any(|i| i.msg.contains("stage_decruncher_at")),
                "wrong error: {v:?}"
            ),
            other => panic!("expected staging-required error, got {other:?}"),
        }
    }

    /// routine_source() for an embedded pack() payload must be self-contained
    /// (define comp_data + embed the bytes) and assemble standalone.
    #[test]
    fn routine_source_is_self_contained_for_pack() {
        let d = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .code_address(0x1000)
            .pack(&sample_input())
            .output(0x4000);
        let src = d.routine_source().unwrap();
        assert!(src.contains("comp_data:"), "no comp_data label:\n{src}");
        let mut asm = asm6502::Assembler6502::new();
        asm.set_origin(0x1000);
        asm.assemble_bytes(&src).expect("routine_source assembles standalone");
    }

    /// An overlapping EMBEDDED payload move (dst > src) must select the
    /// descending copy: the direction consults the payload's resolved
    /// load address, not just an explicit numeric source.
    #[test]
    fn overlapping_payload_move_selects_descending() {
        // Probe: learn where the payload lands, then move it onto itself with
        // dst > src (overlap) - which lands on the program image, so it needs
        // mover_at + staging. Inspect the mover fragment for the descending copy.
        let probe = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .move_packed_to(0x4000)
            .output(0x8000)
            .assemble()
            .unwrap();
        let payload = probe.symbols["payload_data"];
        let len = probe.bytes.len() as u16 - (payload - probe.origin); // payload..end
        let dst = payload + len / 2; // dst > src, overlapping

        let built = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .basic_stub()
            .pack(&sample_input())
            .move_packed_to(dst)
            .mover_at(0x0334)
            .stage_decruncher_at(0x0100)
            .output(0x8000)
            .assemble()
            .unwrap();
        let mover = built
            .fragments
            .iter()
            .find(|(n, _, _)| n == "relocated_mover")
            .expect("relocated mover fragment");
        assert!(
            mover.2.contains("descending"),
            "overlapping payload move did not use a descending copy:\n{}",
            mover.2.lines().take(6).collect::<Vec<_>>().join("\n")
        );
    }

    /// packed_incbin with a wrong length is rejected when the file is reachable.
    #[test]
    fn packed_incbin_wrong_len_is_rejected() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!("lzan_c64_incbin_{}.bin", std::process::id()));
        std::fs::write(&path, [0u8; 100]).unwrap();
        let err = Decruncher::new(Format::Zx02, Direction::Forward)
            .unwrap()
            .packed_incbin(path.to_str().unwrap(), 50) // claims 50, file is 100
            .output(0x4000)
            .output_len(200)
            .build();
        let _ = std::fs::remove_file(&path);
        assert!(matches!(err, Err(GenError::Config(_))), "wrong-len incbin should reject");
    }
}
