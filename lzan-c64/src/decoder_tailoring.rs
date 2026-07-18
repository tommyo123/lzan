//! Per-crunch tailored decoder bodies - the library's explicit-choice surface.
//!
//! The lzan-c64 library requires the caller to choose the tailoring mode
//! EXPLICITLY ([`DecoderTailoring`]); there is no auto-selection here. A host
//! that wants "smallest of standard / tailored" (LazyCruncher's Auto) builds
//! both and compares. Correctness of a tailored body rests on two proven
//! invariants (see [`crate::decoder_gates`]): the composer only ever removes sections
//! for stream features the measured stream does not use, and a body composed
//! for the exact measured traits is a strict subset of the fully-featured
//! (static) decoder that the test matrix pins byte-for-byte.

use crate::decoder_gates;
use crate::registry::{find_routine, Direction, Format, Variant};
use std::collections::BTreeSet;

/// Which decoder body a build installs. The library forces an explicit choice;
/// LazyCruncher's Auto tries both and keeps the smaller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecoderTailoring {
    /// The routine's static, fully general decoder body.
    Standard,
    /// A decoder tailored to the feature traits of THIS stream: sections for
    /// stream features the stream never uses are removed. The stream bytes are
    /// unchanged - still `exomizer raw` / native-tool decodable.
    Tailored,
}

/// Compose a trait-tailored decoder body for `stream` (the exact compressed
/// stream a build will embed) in the given format/direction, or `None` when no
/// tailoring applies: the format is unsupported (only Exomizer today), the
/// routine carries no gates, the stream exercises every gated feature (so the
/// tailored body would equal the static one), or composition fails.
///
/// The returned string is a full routine source with its config-defaults block
/// intact, suitable for [`crate::Decruncher::body_override`]. It is guaranteed
/// to assemble no larger than the static body.
pub fn tailored_body(format: Format, direction: Direction, stream: &[u8]) -> Option<String> {
    if format != Format::Exomizer {
        return None;
    }
    let spec = find_routine(format, direction, Variant::Standard)?;
    let all_gates = decoder_gates::gates(spec.source);
    if all_gates.is_empty() {
        return None;
    }

    let traits = match direction {
        Direction::Forward => lzan::exo3::stream_traits(stream),
        Direction::Backward => lzan::exo3::stream_traits_backward(stream),
    };

    // A gate feature is PRESENT (its section kept) iff the stream uses it. An
    // unrecognised gate is kept unconditionally - a feature we cannot measure
    // must never be trimmed.
    let mut present: BTreeSet<String> = BTreeSet::new();
    for g in &all_gates {
        let keep = match g.as_str() {
            "litseq" => traits.litseq,
            "len16" => traits.len16,
            _ => true,
        };
        if keep {
            present.insert(g.clone());
        }
    }
    // Every gate present => tailored == static: nothing to gain.
    if present.len() == all_gates.len() {
        return None;
    }

    decoder_gates::compose(spec.source, &present).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Assemble a decoder body standalone: the source's own config-defaults
    /// block defines `zp_base`/`table_ram`, so only the external stream/output
    /// symbols and an origin are supplied. Returns the assembled bytes.
    fn asm_body(source: &str, org: u16) -> Vec<u8> {
        let src = format!(
            "comp_data = $4000\nout_addr = $8000\ncomp_data_len = $0100\nout_len = $0100\n*= ${org:04X}\n{source}"
        );
        let mut a = asm6502::Assembler6502::new();
        a.set_origin(org);
        a.assemble_bytes(&src)
            .unwrap_or_else(|e| panic!("body did not assemble standalone: {e:?}\n{src}"))
    }

    fn all_gates(dir: Direction) -> BTreeSet<String> {
        let spec = find_routine(Format::Exomizer, dir, Variant::Standard).unwrap();
        decoder_gates::gates(spec.source).into_iter().collect()
    }

    /// ANCHOR: `compose(ALL gates)` must assemble byte-for-byte identically to
    /// the raw static source - the invariant the whole scheme rests on.
    #[test]
    fn anchor_compose_all_equals_static() {
        for dir in [Direction::Forward, Direction::Backward] {
            let spec = find_routine(Format::Exomizer, dir, Variant::Standard).unwrap();
            let composed = decoder_gates::compose(spec.source, &all_gates(dir)).unwrap();
            let static_bytes = asm_body(spec.source, 0x1000);
            assert_eq!(
                static_bytes,
                asm_body(&composed, 0x1000),
                "{dir:?}: compose(ALL) diverged from the static body"
            );
            // The decoder_gates markers must stay zero-byte comments: the annotated
            // static source still assembles to the registry's declared size.
            assert_eq!(
                static_bytes.len() as u16,
                spec.code_bytes,
                "{dir:?}: annotated static body ({} B) != declared code_bytes ({})",
                static_bytes.len(),
                spec.code_bytes
            );
        }
    }

    /// Assembly matrix: every gate subset composes, assembles, and is no larger
    /// than the fully-featured body (dropping a feature never grows the body).
    #[test]
    fn assembly_matrix_is_size_monotone() {
        for dir in [Direction::Forward, Direction::Backward] {
            let spec = find_routine(Format::Exomizer, dir, Variant::Standard).unwrap();
            let gates: Vec<String> = decoder_gates::gates(spec.source);
            let full = asm_body(spec.source, 0x1000).len();
            for mask in 0..(1u32 << gates.len()) {
                let present: BTreeSet<String> = gates
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| mask & (1 << i) != 0)
                    .map(|(_, g)| g.clone())
                    .collect();
                let composed = decoder_gates::compose(spec.source, &present).unwrap();
                let size = asm_body(&composed, 0x1000).len();
                assert!(
                    size <= full,
                    "{dir:?}: subset {present:?} assembled {size} B > full {full} B"
                );
                if present.len() == gates.len() {
                    assert_eq!(size, full, "{dir:?}: ALL subset must equal full");
                }
            }
        }
    }

    /// A litseq-free stream yields a strictly smaller tailored body in both
    /// directions, and it still assembles.
    #[test]
    fn litseq_free_stream_tailors_smaller_both_directions() {
        let input = vec![0x41u8; 4096];
        for dir in [Direction::Forward, Direction::Backward] {
            let stream = match dir {
                Direction::Forward => lzan::exo3::compress_exo3(&input),
                Direction::Backward => lzan::exo3::compress_exo3_backward(&input),
            };
            let tailored = tailored_body(Format::Exomizer, dir, &stream)
                .unwrap_or_else(|| panic!("{dir:?}: litseq-free stream should tailor"));
            let spec = find_routine(Format::Exomizer, dir, Variant::Standard).unwrap();
            let full = asm_body(spec.source, 0x1000).len();
            let small = asm_body(&tailored, 0x1000).len();
            assert!(small < full, "{dir:?}: tailored {small} B not smaller than {full} B");
        }
    }

    #[test]
    fn non_exomizer_never_tailors() {
        assert!(tailored_body(Format::Zx02, Direction::Forward, &[0u8; 16]).is_none());
    }

    /// A highly repetitive input compresses to matches with no literal
    /// sequences, so the forward exomizer decoder tailors away `exit_or_lit_seq`
    /// - the composed body is smaller and lacks the literal-sequence handler.
    #[test]
    fn repetitive_input_drops_litseq_handler_forward() {
        let input = vec![0x41u8; 4096];
        let stream = lzan::exo3::compress_exo3(&input);
        let traits = lzan::exo3::stream_traits(&stream);
        assert!(!traits.litseq, "expected no literal sequences for a flat run");
        let body = tailored_body(Format::Exomizer, Direction::Forward, &stream)
            .expect("litseq-free stream should tailor");
        // The tailored body keeps decode structure but not the lit-seq handler:
        // the two get_crunched_byte calls of the handler are gone.
        assert!(body.contains("exit_or_lit_seq:"));
        assert!(!body.contains("STA zp_len_hi\n        JSR get_crunched_byte"));
    }
}
