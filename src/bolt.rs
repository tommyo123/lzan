//! BoltLZ: a purely byte-oriented LZ77 for the MOS 6510.
//!
//! The design goal is a small, fast decoder that contains no bit reader at all - every field is a
//! whole byte and dispatch is a single sign-bit test. Compression ratio is traded for that
//! (always-2-byte offsets, no repeat-offset, in-token lengths). The matching 6510 forward decoder
//! (`decrunchers/bolt.s`) is 97 bytes and takes about 18 cycles per copied byte, using no
//! undocumented opcodes; for comparison the other byte-oriented decoders here are larger (LZSA1
//! 165, TSCrunch 160).
//!
//! # Stream format (raw block, forward)
//!
//! A sequence of commands, each introduced by one token byte `T`, terminated by a single `$00`:
//!
//! - `T == $00`            -> end of stream.
//! - `T == $01..=$7F`      -> literal run: `N = T` (1..=127) raw bytes follow inline.
//! - `T == $80..=$FF`      -> match: length `L = (T & $7F) + 3` (3..=130), then a 2-byte
//!                            little-endian NEGATED offset `NEG = (65536 - d) & $FFFF` where `d`
//!                            is the back-distance in `1..=65535`. The decoder forms the match
//!                            source with a plain 16-bit add `mptr = dst + NEG == dst - d`.
//!
//! `$00` is unambiguous as EOF: literal tokens are `>= $01`, match tokens are `>= $80`, and literal
//! payload / offset bytes are consumed by count, never re-dispatched. Match copy is ascending, so
//! self-overlap (`d < L`, i.e. RLE) reproduces correctly. See `design`-level notes in
//! `decrunchers/bolt.s`.

use crate::matchfinder::find_matches;

/// Minimum match length. A match costs 3 bytes (token + 2 offset); length 3 is break-even vs 3
/// literals, length 4+ strictly wins, length 2 would always expand.
const MIN_MATCH: usize = 3;
/// Longest match encodable in one token: `(T & $7F) + 3` with `T & $7F` up to 127.
const MAX_MATCH: usize = 130;
/// Longest literal run encodable in one token (`$01..=$7F`).
const MAX_LIT: usize = 127;
/// Largest back-reference distance (full 64 KB reach).
const MAX_OFFSET: usize = 0xffff;
/// Match-finder chain depth. Ratio is sacrificed by design, so a bounded hash-chain search (fast,
/// O(n*chain)) is used rather than the exact O(n*window) finder.
const MAX_CHAIN: usize = 256;

/// Highest supported compression level (single algorithm).
pub const MAX_LEVEL: u8 = 1;

/// Compress `input` to a BoltLZ raw block. `level` is ignored (single algorithm); `backward`
/// selects the in-place orientation.
pub fn compress(input: &[u8], _level: u8, backward: bool) -> Vec<u8> {
    if backward {
        compress_bolt_backward(input)
    } else {
        compress_bolt(input)
    }
}

/// Decompress a BoltLZ raw block. `backward` selects the in-place orientation.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = decode_bolt(&rev);
        out.reverse();
        out
    } else {
        decode_bolt(input)
    }
}

/// A single emitted command chosen by the parse.
#[derive(Clone, Copy)]
enum Cmd {
    /// Literal run of `usize` bytes (1..=127).
    Lit(usize),
    /// Match of (length 3..=130, distance 1..=65535).
    Match(usize, usize),
}

/// Compress `input` to a forward BoltLZ raw block.
///
/// Cost-optimal shortest-path parse over whole-byte costs: a literal run of length `r` (1..=127)
/// costs `r + 1` bytes (payload + token); a match of length 3..=130 costs 3 bytes regardless of
/// length. Longer runs/matches are split by the parse into per-token fragments that each stay in
/// range (literal runs <= 127, match fragments in 3..=130 - never a sub-min-match remainder,
/// because the parse only ever relaxes match lengths in `[3, 130]`).
pub fn compress_bolt(input: &[u8]) -> Vec<u8> {
    let n = input.len();
    if n == 0 {
        return vec![0x00];
    }

    let ms = find_matches(input, MIN_MATCH, MAX_OFFSET, MAX_MATCH, MAX_CHAIN);

    const INF: u64 = u64::MAX / 2;
    let mut cost = vec![INF; n + 1];
    let mut back: Vec<(usize, Cmd)> = vec![(0, Cmd::Lit(0)); n + 1];
    cost[0] = 0;

    for i in 0..n {
        let ci = cost[i];
        if ci == INF {
            continue;
        }

        // Literal-run edges: length r in 1..=min(127, n-i), cost r + 1 (payload + one token). A
        // single longer edge always beats two shorter ones, so the min-cost path uses the optimal
        // token grouping; runs longer than 127 are forced to split across edges.
        let maxr = (n - i).min(MAX_LIT);
        for r in 1..=maxr {
            let j = i + r;
            let nc = ci + r as u64 + 1;
            if nc < cost[j] {
                cost[j] = nc;
                back[j] = (i, Cmd::Lit(r));
            }
        }

        // Match edges: each candidate length in [MIN_MATCH, min(130, maxlen)] costs a flat 3 bytes.
        // Candidates arrive in increasing length, each with its smallest offset; `covered` tracks
        // the highest length already relaxed so every length is relaxed exactly once with the
        // smallest offset achieving it.
        let mut covered = MIN_MATCH - 1;
        for cand in ms.matches_for(i) {
            let clen = (cand.length as usize).min(MAX_MATCH).min(n - i);
            if clen <= covered {
                continue;
            }
            let off = cand.offset as usize;
            for l in (covered + 1)..=clen {
                let j = i + l;
                let nc = ci + 3;
                if nc < cost[j] {
                    cost[j] = nc;
                    back[j] = (i, Cmd::Match(l, off));
                }
            }
            covered = clen;
            if covered >= MAX_MATCH {
                break;
            }
        }
    }

    // Backtrace into forward command order.
    let mut path: Vec<(usize, Cmd)> = Vec::new();
    let mut i = n;
    while i > 0 {
        let (p, cmd) = back[i];
        path.push((p, cmd));
        i = p;
    }
    path.reverse();

    // Emit.
    let mut out: Vec<u8> = Vec::with_capacity(n / 2 + 16);
    for (p, cmd) in path {
        match cmd {
            Cmd::Lit(r) => {
                debug_assert!((1..=MAX_LIT).contains(&r));
                out.push(r as u8); // $01..$7F
                out.extend_from_slice(&input[p..p + r]);
            }
            Cmd::Match(l, off) => {
                debug_assert!((MIN_MATCH..=MAX_MATCH).contains(&l));
                debug_assert!((1..=MAX_OFFSET).contains(&off));
                out.push(0x80 | ((l - MIN_MATCH) as u8)); // $80..$FF
                let neg = (0x1_0000usize - off) as u16; // (65536 - d) & 0xFFFF, d in 1..=65535
                out.push((neg & 0xff) as u8);
                out.push((neg >> 8) as u8);
            }
        }
    }
    out.push(0x00); // EOF
    out
}

/// Compress `input` to a backward (in-place) BoltLZ raw block.
///
/// `reverse(compress_bolt(reverse(input)))`, mirroring the repo's other backward encoders: the
/// input bytes are reversed, the forward encoder runs, and the whole output buffer is reversed. A
/// backward 6510 decoder walks both pointers downward.
pub fn compress_bolt_backward(input: &[u8]) -> Vec<u8> {
    let mut rev: Vec<u8> = input.to_vec();
    rev.reverse();
    let mut out = compress_bolt(&rev);
    out.reverse();
    out
}

/// Decode a forward BoltLZ raw block.
pub fn decode_bolt(data: &[u8]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    let mut ip = 0usize;
    let end = data.len();
    while ip < end {
        let t = data[ip];
        ip += 1;
        if t == 0 {
            break; // EOF
        }
        if t < 0x80 {
            // Literal run of N = t bytes.
            let n = t as usize;
            out.extend_from_slice(&data[ip..ip + n]);
            ip += n;
        } else {
            // Match: length (t & 0x7f) + 3, 2-byte negated offset.
            let l = (t & 0x7f) as usize + MIN_MATCH;
            let neg = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
            ip += 2;
            let d = (0x1_0000 - neg) & 0xffff; // distance, neg in 1..=0xFFFF -> d in 1..=0xFFFF
            let src = out.len() - d;
            for k in 0..l {
                let v = out[src + k];
                out.push(v);
            }
        }
    }
    out
}

/// Decode and also return the in-place safety gap (bytes): the peak of
/// `produced - consumed` over the decode, minus its final value. Any in-place
/// layout (forward top-aligned or backward) must keep the write head at least
/// this many bytes clear of the read head. BoltLZ is byte-aligned (literals and
/// offsets are whole bytes), so this gap is small - a handful of bytes - just
/// like LZSA1's; the model is identical. See [`max_gap_forward`] /
/// [`max_gap_backward`].
fn decode_bolt_with_gap(data: &[u8]) -> (Vec<u8>, i32) {
    if data.is_empty() {
        return (Vec::new(), 0);
    }
    let mut out: Vec<u8> = Vec::new();
    let mut ip = 0usize;
    let end = data.len();
    // Peak of (produced - consumed) at a token boundary. It rises during a match
    // copy (output advances, input barely moves) and falls during literal runs.
    let mut max_gap = 0i32;
    while ip < end {
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let t = data[ip];
        ip += 1;
        if t == 0 {
            break; // EOF
        }
        if t < 0x80 {
            let n = t as usize;
            out.extend_from_slice(&data[ip..ip + n]);
            ip += n;
        } else {
            let l = (t & 0x7f) as usize + MIN_MATCH;
            let neg = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
            ip += 2;
            let d = (0x1_0000 - neg) & 0xffff;
            let src = out.len() - d;
            for k in 0..l {
                let v = out[src + k];
                out.push(v);
            }
        }
    }
    // The read head consumes the whole `data.len()`-byte block; use it (not `ip`,
    // which stops at EOF) so the final gap is the true end state.
    let final_gap = out.len() as i32 - data.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD BoltLZ stream: the top-aligned
/// packed block must start at least this many bytes above the output end, or the
/// decoder's write head overtakes unread compressed data. Small (byte-aligned).
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        decode_bolt_with_gap(stream).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD BoltLZ stream: the packed block
/// must sit at least this many bytes below the span start. A backward block is
/// `reverse(compress_bolt(reverse(input)))`, and a backward decoder reads the
/// stored stream from its END - which is exactly a forward decode of the
/// reversed stream, so the gap sequence matches.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        let rev: Vec<u8> = stream.iter().rev().copied().collect();
        decode_bolt_with_gap(&rev).1.max(0) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(data: &[u8]) {
        let f = compress_bolt(data);
        assert_eq!(decode_bolt(&f), data, "forward roundtrip len {}", data.len());

        let b = compress_bolt_backward(data);
        assert_eq!(decompress(&b, true), data, "backward roundtrip len {}", data.len());

        // Uniform API.
        assert_eq!(decompress(&compress(data, 1, false), false), data);
        assert_eq!(decompress(&compress(data, 1, true), true), data);

        // Never wildly larger than the input: fixed overhead is tiny (worst case ~ +0.8%).
        assert!(f.len() <= data.len() + data.len() / 100 + 16, "expansion too large: {} -> {}", data.len(), f.len());
    }

    #[test]
    fn edge_cases() {
        rt(&[]);
        rt(&[0x42]);
        rt(&[0, 0, 1]); // note: raw input, not a stream
        rt(b"X");
        rt(b"AB");
        rt(b"ABC");
    }

    #[test]
    fn literal_boundaries() {
        // Runs of exactly 127 (one token) and 128 (must split 127+1).
        rt(&(0..127u32).map(|i| (i * 37 + 1) as u8).collect::<Vec<_>>());
        rt(&(0..128u32).map(|i| (i * 37 + 1) as u8).collect::<Vec<_>>());
        rt(&(0..300u32).map(|i| (i * 37 + 1) as u8).collect::<Vec<_>>());
    }

    #[test]
    fn match_length_boundaries() {
        // A long run forces match splitting at 130 and, critically, must never leave a <3
        // remainder (131 -> 128+3, 132 -> 129+3, 261 -> 130+128+3, ...).
        for len in [129usize, 130, 131, 132, 133, 260, 261, 262, 391, 392] {
            let data = vec![0xABu8; len + 4];
            rt(&data);
        }
    }

    #[test]
    fn rle_overlap() {
        // d = 1 self-overlap (RLE) up to and beyond one token.
        rt(&vec![0x5Au8; 4]);
        rt(&vec![0x5Au8; 200]);
        // d = 2 period.
        rt(&(0..400).map(|i| (i % 2) as u8).collect::<Vec<_>>());
        // d = 3 period.
        rt(&(0..600).map(|i| (i % 3) as u8).collect::<Vec<_>>());
    }

    #[test]
    fn far_offset() {
        // Force a back-reference near the 64 KB reach and offset bytes that are $00.
        let mut data: Vec<u8> = (0..1000).map(|i| (i & 0xff) as u8).collect();
        let head = data.clone();
        data.extend_from_slice(&head); // distance 1000 back-ref (offset low byte can be 0)
        rt(&data);
    }

    #[test]
    fn incompressible_expansion_bounded() {
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        let f = compress_bolt(&noise);
        assert_eq!(decode_bolt(&f), noise);
        // ceil(8192/127) tokens + payload + 1 EOF ~= +0.8%.
        assert!(f.len() <= noise.len() + noise.len() / 100 + 16);
    }

    #[test]
    fn text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..300 {
            data.extend_from_slice(base);
        }
        rt(&data);
    }

    #[test]
    fn repetitive_periods() {
        for &period in &[1usize, 2, 3, 4, 5, 7, 8, 16, 17, 129, 130, 131, 255, 256, 257] {
            let v: Vec<u8> = (0..3000).map(|i| (i % period) as u8).collect();
            rt(&v);
        }
    }

    /// BoltLZ is byte-aligned, so the in-place safety gap is tiny (a handful of
    /// bytes) for every kind of input - never the large gap a bit-packed format
    /// can need. This is what lets the workshop place a BoltLZ stream forward
    /// in-place instead of rejecting it. The `with_gap` decode must also match a
    /// plain decode (the instrumentation cannot change the output).
    #[test]
    fn in_place_gap_is_small_and_measured() {
        let zeros = vec![0u8; 8192];
        let periodic: Vec<u8> = (0..8192u32).map(|i| (i % 7) as u8).collect();
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        for data in [&zeros, &periodic, &noise] {
            let f = compress_bolt(data);
            assert_eq!(&decode_bolt_with_gap(&f).0, data, "with_gap altered output");
            let g = max_gap_forward(&f);
            assert!(g < 512, "forward gap {g} unexpectedly large for byte-aligned data");
            let b = compress_bolt_backward(data);
            assert!(max_gap_backward(&b) < 512, "backward gap too large");
        }
        assert_eq!(max_gap_forward(&[]), 0);
        assert_eq!(max_gap_backward(&[]), 0);
    }
}
