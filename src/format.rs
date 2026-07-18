//! Uniform interface over the supported formats: pick a `Format`, then `compress` / `decompress`
//! with the same signature for any of them.
//!
//! `level` is a NORMALIZED effort tier clamped to `1..=max_level()`: **level 1 is the fastest
//! (a single native-quality pass), and `max_level()` is the absolute best (smallest) output**;
//! size is monotone non-increasing in `level`. `max_level()` is small (1-3): single-algorithm
//! formats report 1 (all levels route to the one algorithm); formats with a fast anchor + a
//! best-of report 2; the few with several native efforts report 3. A format with N native levels
//! (e.g. upkr's 0-9) maps the tiers onto representative points (upkr: 1→1, 2→6, 3→9).
//!
//! Each foreign module ALSO exposes a per-algorithm native/special API - `compress_native(input,
//! <real knob>, backward)` - to set its real parameter directly (upkr's 0-9 level, Shrinkler's
//! `PackParams`, Exomizer's trajectory count, apultra's arrival count, …) for callers that want
//! the exact native setting rather than a normalized tier.
//!
//! `backward` selects the in-place (reverse) variant.

use crate::codec::Options;
use crate::{
    apultra, bb2, exo3, lzsa1, lzsa2, shrinkler, subsizer, tscrunch, upkr, zx, zx02, zx0compat,
};

/// A supported compression format.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// LZAN container format: best ratio on c64 data, own decoder.
    Lzan,
    /// ZX0 v2, decoded by dzx0 and 6502 ZX0 decoders.
    Zx0,
    /// LZSA1 raw block.
    Lzsa1,
    /// LZSA2 raw block.
    Lzsa2,
    /// Exomizer 3 raw.
    Exomizer,
    /// upkr.
    Upkr,
    /// ByteBoozer2.
    ByteBoozer2,
    /// TSCrunch.
    Tscrunch,
    /// ZX02: 6502-tuned ZX0 variant, tiny decoder.
    Zx02,
    /// aPLib / apultra.
    Apultra,
    /// Shrinkler: top ratio, no-parity mode.
    Shrinkler,
    /// Subsizer.
    Subsizer,
}

impl Format {
    /// Lowercase identifier.
    pub fn name(self) -> &'static str {
        match self {
            Format::Lzan => "lzan",
            Format::Zx0 => "zx0",
            Format::Lzsa1 => "lzsa1",
            Format::Lzsa2 => "lzsa2",
            Format::Exomizer => "exomizer",
            Format::Upkr => "upkr",
            Format::ByteBoozer2 => "bb2",
            Format::Tscrunch => "tscrunch",
            Format::Zx02 => "zx02",
            Format::Apultra => "apultra",
            Format::Shrinkler => "shrinkler",
            Format::Subsizer => "subsizer",
        }
    }

    /// Look up a format by name (`exo` is accepted for Exomizer).
    pub fn from_name(s: &str) -> Option<Format> {
        Some(match s {
            "lzan" => Format::Lzan,
            "zx0" => Format::Zx0,
            "lzsa1" => Format::Lzsa1,
            "lzsa2" => Format::Lzsa2,
            "exomizer" | "exo" => Format::Exomizer,
            "upkr" => Format::Upkr,
            "bb2" | "byteboozer2" => Format::ByteBoozer2,
            "tscrunch" | "tsc" => Format::Tscrunch,
            "zx02" => Format::Zx02,
            "apultra" | "aplib" | "apl" => Format::Apultra,
            "shrinkler" | "shr" => Format::Shrinkler,
            "subsizer" | "sub" => Format::Subsizer,
            _ => return None,
        })
    }

    /// Highest optimization level. A single-algorithm format reports 1.
    pub fn max_level(self) -> u8 {
        match self {
            Format::Lzan => 3,
            Format::Zx0 => zx0compat::MAX_LEVEL,
            Format::Lzsa1 => lzsa1::MAX_LEVEL,
            Format::Lzsa2 => lzsa2::MAX_LEVEL,
            Format::Exomizer => exo3::MAX_LEVEL,
            Format::Upkr => upkr::MAX_LEVEL,
            Format::ByteBoozer2 => bb2::MAX_LEVEL,
            Format::Tscrunch => tscrunch::MAX_LEVEL,
            Format::Zx02 => zx02::MAX_LEVEL,
            Format::Apultra => apultra::MAX_LEVEL,
            Format::Shrinkler => shrinkler::MAX_LEVEL,
            Format::Subsizer => subsizer::MAX_LEVEL,
        }
    }

    /// All formats, in a stable order.
    pub fn all() -> [Format; 12] {
        [
            Format::Lzan,
            Format::Zx0,
            Format::Lzsa1,
            Format::Lzsa2,
            Format::Exomizer,
            Format::Upkr,
            Format::ByteBoozer2,
            Format::Tscrunch,
            Format::Zx02,
            Format::Apultra,
            Format::Shrinkler,
            Format::Subsizer,
        ]
    }
}

/// Compress `input` with `format` at `level` (clamped to `1..=max_level`). `backward` selects the
/// in-place (reverse) variant where the format supports it.
pub fn compress(format: Format, input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let level = level.clamp(1, format.max_level());
    match format {
        Format::Lzan => lzan_compress(input, level, backward),
        Format::Zx0 => zx0compat::compress(input, level, backward),
        Format::Lzsa1 => lzsa1::compress(input, level, backward),
        Format::Lzsa2 => lzsa2::compress(input, level, backward),
        Format::Exomizer => exo3::compress(input, level, backward),
        Format::Upkr => upkr::compress(input, level, backward),
        Format::ByteBoozer2 => bb2::compress(input, level, backward),
        Format::Tscrunch => tscrunch::compress(input, level, backward),
        Format::Zx02 => zx02::compress(input, level, backward),
        Format::Apultra => apultra::compress(input, level, backward),
        Format::Shrinkler => shrinkler::compress(input, level, backward),
        Format::Subsizer => subsizer::compress(input, level, backward),
    }
}

/// Decompress a stream produced by [`compress`] with the same `format` and direction.
pub fn decompress(format: Format, input: &[u8], backward: bool) -> Vec<u8> {
    match format {
        Format::Lzan => lzan_decompress(input, backward),
        Format::Zx0 => zx0compat::decompress(input, backward),
        Format::Lzsa1 => lzsa1::decompress(input, backward),
        Format::Lzsa2 => lzsa2::decompress(input, backward),
        Format::Exomizer => exo3::decompress(input, backward),
        Format::Upkr => upkr::decompress(input, backward),
        Format::ByteBoozer2 => bb2::decompress(input, backward),
        Format::Tscrunch => tscrunch::decompress(input, backward),
        Format::Zx02 => zx02::decompress(input, backward),
        Format::Apultra => apultra::decompress(input, backward),
        Format::Shrinkler => shrinkler::decompress(input, backward),
        Format::Subsizer => subsizer::decompress(input, backward),
    }
}

// LZAN's own format: the container forward; a length-prefixed reversed-payload blob backward.

fn lzan_compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    if backward {
        let blob = zx::compress_backward_best_of(input, level);
        let mut out = Vec::with_capacity(4 + blob.len());
        out.extend_from_slice(&(input.len() as u32).to_le_bytes());
        out.extend_from_slice(&blob);
        out
    } else {
        let opts = Options {
            effort: level,
            ..Options::default()
        };
        crate::compress_with(input, &opts)
    }
}

fn lzan_decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let orig_len = u32::from_le_bytes([input[0], input[1], input[2], input[3]]) as usize;
        zx::decode_backward(&input[4..], orig_len)
    } else {
        crate::decompress(input).expect("lzan stream")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(fmt: Format, data: &[u8]) {
        for &backward in &[false, true] {
            let c = compress(fmt, data, fmt.max_level(), backward);
            let d = decompress(fmt, &c, backward);
            assert_eq!(
                d,
                data,
                "{} backward={} len {}",
                fmt.name(),
                backward,
                data.len()
            );
        }
    }

    #[test]
    fn all_formats_roundtrip() {
        let mut data = Vec::new();
        let base = b"the quick brown fox jumps over the lazy dog. abracadabra ";
        for _ in 0..40 {
            data.extend_from_slice(base);
        }
        for &fmt in Format::all().iter() {
            roundtrip(fmt, &data);
            roundtrip(fmt, &[1, 2, 3, 4, 5, 1, 2, 3, 4, 5]);
        }
    }

    #[test]
    fn level_is_clamped() {
        let data = b"aaaaaaaaaabbbbbbbbbbccccccccccdddddddddd".repeat(8);
        for &fmt in Format::all().iter() {
            let lo = compress(fmt, &data, 0, false);
            let hi = compress(fmt, &data, 255, false);
            assert_eq!(decompress(fmt, &lo, false), data);
            assert_eq!(decompress(fmt, &hi, false), data);
        }
    }
}
