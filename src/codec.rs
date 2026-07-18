//! Codec options for the LZAN container.
//!
//! This `lzan` crate is the c64-only library: it compresses via the table-free ZX0-style backend
//! (`zx.rs`, mode `MODE_ZX`) and the foreign-format emitters. The m68k/Amiga max-ratio codec (the
//! multi-stream tANS / range-coder container) lives in the sibling `lzan-amiga` crate. This module
//! holds the shared `Options` struct (the c64 path reads `effort` and `window`) and the `MIN_MATCH`
//! re-export.

use crate::parse;

pub const MIN_MATCH: u32 = parse::MIN_MATCH;

#[derive(Clone, Copy)]
pub struct Options {
    pub window: usize,
    pub max_chain: usize,
    pub max_len: usize,
    pub parse_iters: u32,
    pub use_entropy: bool,
    pub use_rep: bool,
    /// Allow the adaptive range coder in the final encode. The range coder lives in `lzan-amiga`,
    /// so in this c64-only crate the flag has no effect (the ZX0 backend is always used).
    pub use_rc: bool,
    /// Use the table-free ZX0-style backend (`zx.rs`): Elias-gamma codes + rep0-3, no decode tables,
    /// small multiplication-free decoder suited to a C64.
    pub use_zx: bool,
    /// Encoder effort level (1/2/3), applied across every ZX mode/format. Higher = better ratio,
    /// slower encode, more memory; the decoder is identical regardless (effort only selects which
    /// parse the encoder runs, never the bitstream grammar).
    ///   1 = FAST     - single multi-arrival DP pass, no rep-seeding / reparse / reduce.
    ///   2 = BALANCED - seeding + reparse rounds + reduce (`parse_zx3`).
    ///   3 = OPTIMAL  - brute-force complete-candidate parse (`parse_zx3_complete`); for rep0-only
    ///                  this is ZX0-exact. Memory-heavy, size-gated at ~33 KB.
    /// Default = 3.
    pub effort: u8,
}

impl Default for Options {
    /// Defaults for the 6510-decodable codec: 64 KB window and the table-free ZX0-style backend.
    fn default() -> Self {
        Options {
            window: 1 << 16, // 64 KB - realistic for a C64-class decoder
            max_chain: 512,
            max_len: 1 << 20,
            parse_iters: 5,
            use_entropy: true,
            use_rep: true,
            use_rc: false,
            use_zx: true, // table-free ZX0-style backend
            effort: 3,    // optimal brute-force parse
        }
    }
}

impl Options {
    /// 6510 / C64 target: the table-free ZX0-style backend, 64 KB window. Decoder is small and fast
    /// on an 8-bit CPU (no hardware multiply, no entropy tables).
    pub fn c64() -> Self {
        Options::default()
    }

    /// 68000 / Amiga-ST target. The m68k max-ratio codec lives in the sibling `lzan-amiga` crate;
    /// in this c64-only crate these options still drive the ZX0 backend (`use_rc` has no effect).
    pub fn m68k() -> Self {
        Options {
            window: 1 << 24,
            use_rc: true,
            use_zx: false,
            ..Options::default()
        }
    }

    /// Alias retained for compatibility.
    pub fn max_ratio() -> Self {
        Self::m68k()
    }
}
