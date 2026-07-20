//! LZAN container.
//!
//! Layout: `"LZAN"` magic, version, mode, original length, then payload.
//!   mode 0 = STORED (raw bytes; output never exceeds orig + header)
//!   mode 2 = ZX     (table-free ZX0-style blob; see `zx`)
//!
//! This crate is the c64-only build and emits only ZX or STORED. Mode 1 (the m68k multi-stream
//! tANS/range-coder container) lives in the sibling `lzan-amiga` crate.

pub mod apultra;
pub mod bb2;
pub mod bolt;
pub mod codec;
pub mod exo3;
pub mod format;
pub mod lzsa1;
pub mod lzsa2;
pub mod matchfinder;
pub mod parse;
pub mod pucrunch;
pub mod shrinkler;
pub mod subsizer;
// Test corpora, shared by the equivalence tests and the `finderbench` binary.
#[doc(hidden)]
pub mod testcorpus;
pub mod tscrunch;
pub mod upkr;
pub mod zx;
pub mod zx02;
pub mod zx0compat;
pub mod zx0opt;

pub use codec::Options;

const MAGIC: &[u8; 4] = b"LZAN";
const VERSION: u8 = 1;
const HEADER_LEN: usize = 4 + 1 + 1 + 4; // magic + version + mode + orig_len(u32 LE)

const MODE_STORED: u8 = 0;
const MODE_ZX: u8 = 2; // table-free ZX0-style backend (zx.rs); payload self-describes rep_slots

/// Compress with default options.
pub fn compress(input: &[u8]) -> Vec<u8> {
    compress_with(input, &Options::default())
}

/// Compress with explicit options. Picks the smaller of the ZX payload vs STORED, so output
/// never exceeds `input.len() + HEADER_LEN`.
pub fn compress_with(input: &[u8], opts: &Options) -> Vec<u8> {
    // Encode with the table-free ZX0-style backend, taking the best of its variants per file.
    // The ZX blob's mode byte self-describes the variant used
    // (rep_slots | near_rep<<4 | am_near_rep<<5). Empty input yields an empty payload and falls
    // through to STORED.
    let (lz_mode, payload) = (MODE_ZX, zx_best_of(input, opts.effort));

    let mut out = Vec::with_capacity(HEADER_LEN + payload.len().min(input.len()));
    out.extend_from_slice(MAGIC);
    out.push(VERSION);

    if payload.len() < input.len() {
        out.push(lz_mode);
        out.extend_from_slice(&(input.len() as u32).to_le_bytes());
        out.extend_from_slice(&payload);
    } else {
        out.push(MODE_STORED);
        out.extend_from_slice(&(input.len() as u32).to_le_bytes());
        out.extend_from_slice(input);
    }
    out
}

/// Run the five ZX best-of variants and return the smallest blob. The variants share nothing
/// mutable and run on scratch threads via `std::thread::scope`; the result is byte-identical to a
/// sequential `min_by_key`. `effort` (1=fast / 2=balanced / 3=optimal) applies to every variant
/// (modes: rep0-only 0x01, rep0-3 0x04, near-rep 0x14, after-match 0x24, both 0x34). The decoder is
/// identical regardless of effort.
pub fn zx_best_of(input: &[u8], effort: u8) -> Vec<u8> {
    type Job = fn(&[u8], u8) -> Vec<u8>;
    let jobs: [Job; 5] = [
        |x, e| zx::compress_e(x, 1, e),
        |x, e| zx::compress_e(x, 4, e),
        |x, e| zx::compress3_e(x, 4, true, false, e),
        |x, e| zx::compress3_e(x, 4, false, true, e),
        |x, e| zx::compress3_e(x, 4, true, true, e),
    ];
    // LZAN_ZX_PIN env hook: pin a single variant (0..=5) instead of best-of, so every file emits
    // the same mode byte for measuring the 6510 decoder against a fixed grammar.
    //   0 = rep0-only (pure ZX0)         mode byte 0x01
    //   1 = rep0-3                        mode byte 0x04
    //   2 = rep0-3 + after-lit near-rep   mode byte 0x14
    //   3 = rep0-3 + after-match near-rep mode byte 0x24
    //   4 = rep0-3 + both near-reps       mode byte 0x34  (full grammar)
    //   5 = rep0-only + in-stream EOF     mode byte 0x41  (minimal decoder; see zx::compress_min_eof)
    if let Ok(s) = std::env::var("LZAN_ZX_PIN") {
        if let Ok(idx) = s.trim().parse::<usize>() {
            if idx == 5 {
                return zx::compress_min_eof_e(input, effort);
            }
            if idx < jobs.len() {
                return jobs[idx](input, effort);
            }
        }
    }
    let blobs: Vec<Vec<u8>> = std::thread::scope(|s| {
        let handles: Vec<_> = jobs
            .iter()
            .map(|j| s.spawn(move || j(input, effort)))
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("zx variant thread panicked"))
            .collect()
    });
    blobs.into_iter().min_by_key(|b| b.len()).unwrap()
}

/// Errors returned by `decompress`.
#[derive(Debug)]
pub enum DecompressError {
    TooShort,
    BadMagic,
    BadVersion(u8),
    BadMode(u8),
}

impl std::fmt::Display for DecompressError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DecompressError::TooShort => write!(f, "input too short for header"),
            DecompressError::BadMagic => write!(f, "bad magic (not an LZAN stream)"),
            DecompressError::BadVersion(v) => write!(f, "unsupported version {}", v),
            DecompressError::BadMode(m) => write!(f, "unknown mode {}", m),
        }
    }
}
impl std::error::Error for DecompressError {}

/// Decompress an LZAN stream.
pub fn decompress(data: &[u8]) -> Result<Vec<u8>, DecompressError> {
    if data.len() < HEADER_LEN {
        return Err(DecompressError::TooShort);
    }
    if &data[0..4] != MAGIC {
        return Err(DecompressError::BadMagic);
    }
    let version = data[4];
    if version != VERSION {
        return Err(DecompressError::BadVersion(version));
    }
    let mode = data[5];
    let orig_len = u32::from_le_bytes([data[6], data[7], data[8], data[9]]) as usize;
    let body = &data[HEADER_LEN..];

    match mode {
        MODE_STORED => Ok(body[..orig_len].to_vec()),
        MODE_ZX => Ok(zx::decode(body, orig_len)),
        other => Err(DecompressError::BadMode(other)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(input: &[u8]) {
        let c = compress(input);
        let d = decompress(&c).expect("decompress");
        assert_eq!(d, input, "container roundtrip len {}", input.len());
        // never worse than stored + header
        assert!(c.len() <= input.len() + HEADER_LEN);
    }

    #[test]
    fn container_roundtrip() {
        rt(&[]);
        rt(&[0]);
        rt(&[1, 2, 3, 4, 5]);
        let base = b"abcabcabcabc abracadabra ";
        let mut v = Vec::new();
        for _ in 0..1000 {
            v.extend_from_slice(base);
        }
        rt(&v);
    }
}
