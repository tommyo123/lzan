//! apultra / aPLib optimal compressor and decompressor.
//!
//! Produces the aPLib/apultra bitstream and matches the apultra reference output byte-for-byte
//! (`apultra` and `apultra -b`), serving as the no-regression anchor. On top of that it runs a
//! best-of wider-beam parse (more arrivals per position plus a higher per-position length-jump cap)
//! and keeps the smaller result, so the output is always `<=` the reference and decodes identically
//! (the bitstream grammar is unchanged, only the parse choices differ). See [`apultra_compress`]
//! for the orchestration.
//!
//! aPLib is an Elias-gamma-style LZ. Commands (MSB-first bit reader, literal bytes stored raw
//! and byte-aligned, interleaved with the bitstream):
//!   `0`             literal: the next raw byte is copied to the output.
//!   `10`            "large" match: a gamma2 value encodes (offset>>8)+2 (or +3 if it follows a
//!                   literal); a raw byte gives the low 8 offset bits; a gamma2 value gives the
//!                   length (biased by offset class). Special case: when (gamma2 - followsLiteral)
//!                   is negative the command is a rep-match reusing the previous offset, and only a
//!                   gamma2 length follows.
//!   `110` + byte    7-bit offset + 1-bit length (offsets 1..127, lengths 2..3). Byte 0x00 = EOD.
//!   `111` + 4 bits  4-bit short offset (1..15) writing one byte; offset 0 writes a literal zero.
//!
//! The match finder (suffix array) is std-only via prefix-doubling; only a correct suffix array is
//! required, then the LCP-interval builder and match enumeration produce the match set that drives
//! the parse.

// ---------------------------------------------------------------------------------------------
// Format constants
// ---------------------------------------------------------------------------------------------

const MIN_OFFSET: i32 = 1;
const MAX_OFFSET: i32 = 0x1fffff;
const MAX_VARLEN: i32 = 0x1fffff;
const BLOCK_SIZE: usize = 0x100000;
const MIN_MATCH_SIZE: i32 = 1;
const MINMATCH3_OFFSET: i32 = 1280;
const MINMATCH4_OFFSET: i32 = 32000;

// Parser constants
const LCP_BITS: u32 = 15;
const TAG_BITS: u32 = 4;
const LCP_MAX: i32 = ((1u32 << (LCP_BITS - TAG_BITS)) - 1) as i32; // 2047
const LCP_AND_TAG_MAX: usize = ((1u32 << LCP_BITS) - 1) as usize;
const LCP_SHIFT: u32 = 63 - LCP_BITS;
const LCP_MASK: u64 = (((1u64 << LCP_BITS) - 1) as u64) << LCP_SHIFT;
const POS_MASK: u64 = (1u64 << LCP_SHIFT) - 1;
const VISITED_FLAG: u64 = 0x8000000000000000;
const EXCL_VISITED_MASK: u64 = 0x7fffffffffffffff;

const NARRIVALS_PER_POSITION_MAX: usize = 62;
const NARRIVALS_PER_POSITION_NORMAL: usize = 46;
const NARRIVALS_PER_POSITION_SMALL: usize = 9;

const NMATCHES_PER_INDEX: usize = 64;

const LEAVE_ALONE_MATCH_SIZE: i32 = 120;

const TOKEN_SIZE_LARGE_MATCH: i32 = 2;
const TOKEN_CODE_LARGE_MATCH: i32 = 2;
const TOKEN_SIZE_7BIT_MATCH: i32 = 3;
const TOKEN_CODE_7BIT_MATCH: i32 = 6;
const TOKEN_SIZE_4BIT_MATCH: i32 = 3;
const TOKEN_CODE_4BIT_MATCH: i32 = 7;

/// Two compression tiers. Level 1 = the reference-arrival anchor (byte-identical to `apultra`,
/// fast); level 2 = the wide-beam best-of (smallest, never larger than level 1).
pub const MAX_LEVEL: u8 = 2;

// ---------------------------------------------------------------------------------------------
// Public uniform API
// ---------------------------------------------------------------------------------------------

/// Compress `input` to an aPLib stream. `level` is NORMALIZED into `1..=MAX_LEVEL`.
///   level 1 = the reference-arrival anchor (byte-identical to `apultra`, fast).
///   level 2 = the wide-beam best-of (smallest, never larger than level 1).
/// When `backward` is set, the stream is the `apultra -b` layout (reverse in / reverse out).
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    if level >= 2 {
        // Tier 2: wide-beam best-of.
        if backward {
            compress_apultra_backward(input)
        } else {
            compress_apultra(input)
        }
    } else {
        // Tier 1: reference-arrival anchor (byte-identical to native apultra).
        if backward {
            apultra_anchor_backward(input)
        } else {
            apultra_compress_with_arrivals(input, None)
        }
    }
}

/// Native API: expose the real aPLib DP knob directly. Force exactly `arrivals` arrivals per
/// position (clamped up to the reference floor, so the supplemental-candidate gate still fires),
/// and take the best-of against the reference anchor so the output is never larger than native
/// `apultra`. When `backward` is set the stream is the `apultra -b` layout.
pub fn compress_native(input: &[u8], arrivals: usize, backward: bool) -> Vec<u8> {
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = apultra_compress_native_forward(&rev, arrivals);
        out.reverse();
        out
    } else {
        apultra_compress_native_forward(input, arrivals)
    }
}

/// Forward `compress_native`: best-of(reference anchor, forced-`arrivals` beam). The anchor floor
/// keeps the output never larger than native `apultra`, exactly as the wide-beam best-of does.
fn apultra_compress_native_forward(input: &[u8], arrivals: usize) -> Vec<u8> {
    let anchor = apultra_compress_with_arrivals(input, None);
    if input.is_empty() {
        return anchor;
    }
    let wide = apultra_compress_with_arrivals(input, Some(arrivals));
    if wide.len() < anchor.len() {
        wide
    } else {
        anchor
    }
}

/// Backward reference-arrival anchor: `apultra -b` layout over the reference (tier-1) parse.
fn apultra_anchor_backward(input: &[u8]) -> Vec<u8> {
    let mut rev = input.to_vec();
    rev.reverse();
    let mut out = apultra_compress_with_arrivals(&rev, None);
    out.reverse();
    out
}

/// Decompress an aPLib stream produced by [`compress`]. When `backward` is set the stream is the
/// `apultra -b` layout (reverse compressed / reverse decompressed).
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = apultra_decompress(&rev);
        out.reverse();
        out
    } else {
        apultra_decompress(input)
    }
}

/// Forward compress: byte-identical to `apultra`, then improved by a best-of wider-beam parse that
/// can only ever shrink the result (never grow it past the reference). See [`apultra_compress`].
pub fn compress_apultra(input: &[u8]) -> Vec<u8> {
    apultra_compress(input)
}

/// Backward compress: the `apultra -b` layout (reverse in, compress, reverse out). Never larger than
/// `apultra -b` - the reverse compress shares the same best-of-vs-anchor floor as the forward path.
pub fn compress_apultra_backward(input: &[u8]) -> Vec<u8> {
    let mut rev = input.to_vec();
    rev.reverse();
    let mut out = apultra_compress(&rev);
    out.reverse();
    out
}

// ---------------------------------------------------------------------------------------------
// Gamma2 size table (_gamma2_size, computed at runtime)
// ---------------------------------------------------------------------------------------------

fn build_gamma2_size() -> [i8; 2048] {
    // _gamma2_size[n] = number of bits a gamma2 encoding of n occupies, for 0..2047.
    // The reference stores a precomputed table; we compute it identically via the size routine.
    let mut t = [0i8; 2048];
    for n in 2..2048usize {
        // mirror apultra_get_gamma2_size's general path: 2 * number-of-significant-bits-after-top
        let mut v = n as i32;
        let mut bits = 0i32;
        // CountShift sequence
        if v >> 16 != 0 {
            v >>= 16;
            bits += 16;
        }
        if v >> 8 != 0 {
            v >>= 8;
            bits += 8;
        }
        if v >> 4 != 0 {
            v >>= 4;
            bits += 4;
        }
        if v >> 2 != 0 {
            v >>= 2;
            bits += 2;
        }
        if v >> 1 != 0 {
            bits += 1;
        }
        t[n] = (bits << 1) as i8;
    }
    t
}

#[inline]
fn gamma2_size(table: &[i8; 2048], value: i32) -> i32 {
    if value >= 0 && value < 2048 {
        table[value as usize] as i32
    } else {
        let mut v = value;
        let mut n = 0i32;
        if v >> 16 != 0 {
            v >>= 16;
            n += 16;
        }
        if v >> 8 != 0 {
            v >>= 8;
            n += 8;
        }
        if v >> 4 != 0 {
            v >>= 4;
            n += 4;
        }
        if v >> 2 != 0 {
            v >>= 2;
            n += 2;
        }
        if v >> 1 != 0 {
            n += 1;
        }
        n << 1
    }
}

// ---------------------------------------------------------------------------------------------
// Bitstream writer (apultra_write_bits / apultra_write_gamma2_value)
// ---------------------------------------------------------------------------------------------

struct BitWriter {
    out: Vec<u8>,
    cur_bits_offset: usize,
    cur_bit_shift: i32, // -1 means "need a new byte"
}

impl BitWriter {
    fn new() -> Self {
        BitWriter {
            out: Vec::new(),
            cur_bits_offset: 0,
            cur_bit_shift: -1,
        }
    }

    #[inline]
    fn write_bits(&mut self, value: i32, nbits: i32) {
        for i in (0..nbits).rev() {
            if self.cur_bit_shift == -1 {
                self.cur_bits_offset = self.out.len();
                self.cur_bit_shift = 7;
                self.out.push(0);
            }
            self.out[self.cur_bits_offset] |= (((value >> i) & 1) << self.cur_bit_shift) as u8;
            self.cur_bit_shift -= 1;
        }
    }

    #[inline]
    fn write_byte(&mut self, b: u8) {
        self.out.push(b);
    }

    #[inline]
    fn write_gamma2(&mut self, value: i32) {
        // msb = 30; while ((value >> msb--) == 0);  -> msb ends one below the top set bit index
        let mut msb = 30i32;
        loop {
            let m = msb;
            msb -= 1;
            if (value >> m) != 0 {
                break;
            }
        }
        // now emit
        while msb > 0 {
            msb -= 1;
            let bit = (value >> msb) & 2;
            self.write_bits(bit | 1, 2);
        }
        self.write_bits((value & 1) << 1, 2);
    }
}

// ---------------------------------------------------------------------------------------------
// Offset / match varlen size helpers
// ---------------------------------------------------------------------------------------------

#[inline]
fn get_offset_varlen_size(g: &[i8; 2048], length: i32, offset: i32, follows_literal: i32) -> i32 {
    if length <= 3 && offset < 128 {
        8 + TOKEN_SIZE_7BIT_MATCH
    } else if follows_literal != 0 {
        8 + TOKEN_SIZE_LARGE_MATCH + gamma2_size(g, (offset >> 8) + 3)
    } else {
        8 + TOKEN_SIZE_LARGE_MATCH + gamma2_size(g, (offset >> 8) + 2)
    }
}

#[inline]
fn get_match_varlen_size(g: &[i8; 2048], length: i32, offset: i32) -> i32 {
    if length <= 3 && offset < 128 {
        0
    } else if offset < 128 || offset >= MINMATCH4_OFFSET {
        gamma2_size(g, length - 2)
    } else if offset < MINMATCH3_OFFSET {
        gamma2_size(g, length)
    } else {
        gamma2_size(g, length - 1)
    }
}

// ---------------------------------------------------------------------------------------------
// Match data structures
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
struct Match {
    length: u32, // :11
    offset: u32, // :21
}

#[derive(Clone, Copy, Default)]
struct FinalMatch {
    length: i32,
    offset: i32,
}

#[derive(Clone, Copy)]
struct Arrival {
    cost: i32,
    from_pos: u32,        // :21
    from_slot: i32,       // :7 (signed)
    follows_literal: u32, // :1
    rep_offset: u32,      // :21
    short_offset: u32,    // :4
    rep_pos: u32,         // :21
    match_len: u32,       // :11
    score: i32,
}

impl Default for Arrival {
    fn default() -> Self {
        Arrival {
            cost: 0,
            from_pos: 0,
            from_slot: 0,
            follows_literal: 0,
            rep_offset: 0,
            short_offset: 0,
            rep_pos: 0,
            match_len: 0,
            score: 0,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Compressor context
// ---------------------------------------------------------------------------------------------

struct Compressor {
    // matchfinder working arrays (sized to window = total input here, single block)
    intervals: Vec<u64>,
    pos_data: Vec<u64>,
    open_intervals: Vec<u64>,
    // per-position match arrays
    match_: Vec<Match>,          // block_size * NMATCHES_PER_INDEX
    match_depth: Vec<u16>,       // block_size * NMATCHES_PER_INDEX
    match1: Vec<u8>,             // block_size
    best_match: Vec<FinalMatch>, // block_size
    arrival: Vec<Arrival>,       // (block_size+1) * max_arrivals
    // rle_len reuses intervals as i32 in the C; we keep a dedicated buffer
    rle_len: Vec<i32>,
    // visited reuses pos_data as i32 in the C; we keep a dedicated buffer
    visited: Vec<i32>,
    first_offset_for_byte: Vec<i32>, // 65536
    next_offset_for_pos: Vec<i32>,   // block_size
    offset_cache: Vec<i32>,          // 2048

    block_size: usize,
    max_offset: i32,
    max_arrivals: usize,
    /// When set, the parse considers every match length (no length-jump pruning) and expands the
    /// near-offset depth chain fully. Off in the anchor (byte-identical to the reference); on in the
    /// wider-parse candidates. Never changes correctness, only how thoroughly the DP searches.
    thorough: bool,
}

impl Compressor {
    fn new(block_size: usize, max_window_size: usize, max_arrivals: usize) -> Self {
        Compressor {
            intervals: vec![0u64; max_window_size],
            pos_data: vec![0u64; max_window_size],
            open_intervals: vec![0u64; LCP_AND_TAG_MAX + 1],
            match_: vec![Match::default(); block_size * NMATCHES_PER_INDEX],
            match_depth: vec![0u16; block_size * NMATCHES_PER_INDEX],
            match1: vec![0u8; block_size],
            best_match: vec![FinalMatch::default(); block_size],
            arrival: vec![Arrival::default(); (block_size + 1) * max_arrivals],
            rle_len: vec![0i32; max_window_size],
            visited: vec![0i32; max_window_size],
            first_offset_for_byte: vec![0i32; 65536],
            next_offset_for_pos: vec![0i32; block_size.max(1)],
            offset_cache: vec![0i32; 2048],
            block_size,
            max_offset: MAX_OFFSET,
            max_arrivals,
            thorough: false,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Suffix array (std-only, prefix-doubling). Produces SA identical in *content* to libdivsufsort;
// only ordering of equal suffixes can differ, but suffixes are never equal (each ends uniquely),
// so the SA is unique and matches libdivsufsort exactly.
// ---------------------------------------------------------------------------------------------

fn build_suffix_array_sa(data: &[u8]) -> Vec<i32> {
    let n = data.len();
    if n == 0 {
        return Vec::new();
    }
    // Prefix-doubling (Manber-Myers) with O(n log n) sort. Suffixes of a string are all distinct,
    // so the resulting order is total and unique.
    let mut sa: Vec<i32> = (0..n as i32).collect();
    let mut rank: Vec<i32> = data.iter().map(|&b| b as i32).collect();
    let mut tmp: Vec<i32> = vec![0; n];
    let mut k = 1usize;
    loop {
        // sort by (rank[i], rank[i+k])
        let key = |i: i32| -> (i32, i32) {
            let i = i as usize;
            let second = if i + k < n { rank[i + k] } else { -1 };
            (rank[i], second)
        };
        sa.sort_by(|&a, &b| key(a).cmp(&key(b)));
        // recompute ranks
        tmp[sa[0] as usize] = 0;
        let mut r = 0i32;
        for w in 1..n {
            if key(sa[w]) != key(sa[w - 1]) {
                r += 1;
            }
            tmp[sa[w] as usize] = r;
        }
        rank.copy_from_slice(&tmp);
        if (r as usize) == n - 1 {
            break;
        }
        k <<= 1;
        if k >= n {
            break;
        }
    }
    sa
}

// ---------------------------------------------------------------------------------------------
// apultra_get_index_tag
// ---------------------------------------------------------------------------------------------

#[inline]
fn get_index_tag(n_index: u32) -> i32 {
    ((n_index as u64).wrapping_mul(11400714819323198485u64) >> (64u64 - TAG_BITS as u64)) as i32
}

// ---------------------------------------------------------------------------------------------
// apultra_build_suffix_array: LCP + interval build
// ---------------------------------------------------------------------------------------------

fn build_suffix_array(c: &mut Compressor, win: &[u8], n: usize) -> i32 {
    let sa = build_suffix_array_sa(win);
    let intervals = &mut c.intervals;

    for i in (0..n).rev() {
        intervals[i] = sa[i] as u64;
    }

    // PLCP via Karkkainen method, using pos_data (as i32) as scratch.
    // PLCP/Phi share the same buffer.
    let plcp = &mut c.pos_data;
    // We need i32 semantics; emulate Phi[x] = -1 as u64 all-ones low bits? The C casts pos_data to
    // int*. We use a separate i32 vector to be safe and exact.
    let mut phi = vec![0i32; n];
    phi[(intervals[0] & 0xffffffff) as usize] = -1;
    for i in 1..n {
        phi[intervals[i] as usize] = intervals[i - 1] as i32;
    }
    let mut plcp_i = vec![0i32; n];
    let mut cur_len = 0i32;
    for i in 0..n {
        if phi[i] == -1 {
            plcp_i[i] = 0;
            continue;
        }
        let p = phi[i] as usize;
        let max_len = if i > p { n - i } else { n - p };
        while (cur_len as usize) < max_len && win[i + cur_len as usize] == win[p + cur_len as usize]
        {
            cur_len += 1;
        }
        plcp_i[i] = cur_len;
        if cur_len > 0 {
            cur_len -= 1;
        }
    }
    let _ = plcp;

    intervals[0] &= POS_MASK;
    for i in 1..n {
        let n_index = (intervals[i] & POS_MASK) as usize;
        let mut n_len = plcp_i[n_index];
        if n_len < MIN_MATCH_SIZE {
            n_len = 0;
        }
        if n_len > LCP_MAX {
            n_len = LCP_MAX;
        }
        let mut n_tagged_len = 0i64;
        if n_len != 0 {
            n_tagged_len = ((n_len << TAG_BITS)
                | (get_index_tag(n_index as u32) & ((1 << TAG_BITS) - 1)))
                as i64;
        }
        intervals[i] = (n_index as u64) | ((n_tagged_len as u64) << LCP_SHIFT);
    }

    // Build intervals (wimlib method)
    let sa_and_lcp = &mut c.intervals;
    let pos_data = &mut c.pos_data;
    let top_stack = &mut c.open_intervals;
    let mut top: usize = 0; // index into top_stack
    let mut next_interval_idx: u64 = 1;
    let mut prev_pos = sa_and_lcp[0] & POS_MASK;

    top_stack[0] = 0;
    sa_and_lcp[0] = 0;

    for r in 1..n {
        let next_pos = sa_and_lcp[r] & POS_MASK;
        let next_lcp = sa_and_lcp[r] & LCP_MASK;
        let top_lcp = top_stack[top] & LCP_MASK;

        if next_lcp == top_lcp {
            pos_data[prev_pos as usize] = top_stack[top];
        } else if next_lcp > top_lcp {
            top += 1;
            top_stack[top] = next_lcp | next_interval_idx;
            next_interval_idx += 1;
            pos_data[prev_pos as usize] = top_stack[top];
        } else {
            pos_data[prev_pos as usize] = top_stack[top];
            loop {
                let closed_interval_idx = top_stack[top] & POS_MASK;
                top -= 1;
                let superinterval_lcp = top_stack[top] & LCP_MASK;

                if next_lcp == superinterval_lcp {
                    sa_and_lcp[closed_interval_idx as usize] = top_stack[top];
                    break;
                } else if next_lcp > superinterval_lcp {
                    top += 1;
                    top_stack[top] = next_lcp | next_interval_idx;
                    next_interval_idx += 1;
                    sa_and_lcp[closed_interval_idx as usize] = top_stack[top];
                    break;
                } else {
                    sa_and_lcp[closed_interval_idx as usize] = top_stack[top];
                }
            }
        }
        prev_pos = next_pos;
    }

    pos_data[prev_pos as usize] = top_stack[top];
    while top > 0 {
        let idx = (top_stack[top] & POS_MASK) as usize;
        sa_and_lcp[idx] = top_stack[top - 1];
        top -= 1;
    }

    0
}

// ---------------------------------------------------------------------------------------------
// apultra_find_matches_at
// ---------------------------------------------------------------------------------------------

fn find_matches_at(
    c: &mut Compressor,
    n_offset: usize,
    matches: &mut [Match],
    match_depth: &mut [u16],
    n_max_matches: usize,
    self_contained: bool,
) -> (usize, u8) {
    let n_max_offset = c.max_offset;
    let mut match1: u8 = 0;

    let intervals = &mut c.intervals;
    let pos_data = &mut c.pos_data;

    let mut refv = pos_data[n_offset];
    pos_data[n_offset] = 0;

    let mut super_ref;
    loop {
        super_ref = intervals[(refv & POS_MASK) as usize];
        if super_ref & LCP_MASK == 0 {
            break;
        }
        intervals[(refv & POS_MASK) as usize] = n_offset as u64 | VISITED_FLAG;
        refv = super_ref;
    }

    if super_ref == 0 {
        if refv != 0 {
            intervals[(refv & POS_MASK) as usize] = n_offset as u64 | VISITED_FLAG;
        }
        return (0, match1);
    }

    let mut match_pos = super_ref & EXCL_VISITED_MASK;
    let mut mptr = 0usize; // index into matches/match_depth
    let mut n_prev_offset = 0i32;
    let mut n_prev_len = 0i32;
    let mut n_cur_depth = 0i32;
    let mut cur_depth_idx: Option<usize> = None;

    if self_contained {
        let n_match_offset = (n_offset - match_pos as usize) as i32;
        if mptr < n_max_matches {
            let n_match_len = (refv >> (LCP_SHIFT + TAG_BITS)) as i32;
            if n_match_offset <= n_max_offset {
                matches[mptr].length = n_match_len as u32;
                matches[mptr].offset = n_match_offset as u32;
                match_depth[mptr] = 0;
                n_cur_depth = 0;
                cur_depth_idx = Some(mptr);
                mptr += 1;
                n_prev_len = n_match_len;
                n_prev_offset = n_match_offset;
            }
        }
    }

    loop {
        super_ref = pos_data[match_pos as usize];
        if super_ref > refv {
            match_pos = intervals[(super_ref & POS_MASK) as usize] & EXCL_VISITED_MASK;

            if self_contained {
                let n_match_offset = (n_offset - match_pos as usize) as i32;
                if mptr < n_max_matches {
                    let n_match_len = (refv >> (LCP_SHIFT + TAG_BITS)) as i32;
                    if n_match_offset <= n_max_offset && (n_prev_offset - n_match_offset) >= 128 {
                        if n_prev_offset != 0
                            && n_prev_len > 2
                            && n_match_offset == (n_prev_offset - 1)
                            && n_match_len == (n_prev_len - 1)
                            && cur_depth_idx.is_some()
                            && n_cur_depth < LCP_MAX
                        {
                            n_cur_depth += 1;
                            match_depth[cur_depth_idx.unwrap()] = (n_cur_depth as u16) | 0x8000;
                        } else {
                            matches[mptr].length = n_match_len as u32;
                            matches[mptr].offset = n_match_offset as u32;
                            n_cur_depth = 0;
                            match_depth[mptr] = 0x8000;
                            cur_depth_idx = Some(mptr);
                            mptr += 1;
                        }
                        n_prev_len = n_match_len;
                        n_prev_offset = n_match_offset;
                    }
                }
            }
        }

        loop {
            super_ref = pos_data[match_pos as usize];
            if super_ref <= refv {
                break;
            }
            match_pos = intervals[(super_ref & POS_MASK) as usize] & EXCL_VISITED_MASK;

            if self_contained {
                let n_match_offset = (n_offset - match_pos as usize) as i32;
                if mptr < n_max_matches {
                    let n_match_len = (refv >> (LCP_SHIFT + TAG_BITS)) as i32;
                    if n_match_offset <= n_max_offset
                        && (n_match_len >= 3 || (n_match_len >= 2 && mptr < (n_max_matches - 1)))
                        && n_match_len < 1280
                        && (n_prev_offset - n_match_offset) >= 128
                    {
                        if n_prev_offset != 0
                            && n_prev_len > 2
                            && n_match_offset == (n_prev_offset - 1)
                            && n_match_len == (n_prev_len - 1)
                            && cur_depth_idx.is_some()
                            && n_cur_depth < LCP_MAX
                        {
                            n_cur_depth += 1;
                            match_depth[cur_depth_idx.unwrap()] = (n_cur_depth as u16) | 0x8000;
                        } else {
                            matches[mptr].length = n_match_len as u32;
                            matches[mptr].offset = n_match_offset as u32;
                            n_cur_depth = 0;
                            match_depth[mptr] = 0x8000;
                            cur_depth_idx = Some(mptr);
                            mptr += 1;
                        }
                        n_prev_len = n_match_len;
                        n_prev_offset = n_match_offset;
                    }
                }
            }
        }

        intervals[(refv & POS_MASK) as usize] = n_offset as u64 | VISITED_FLAG;
        pos_data[match_pos as usize] = refv;

        let n_main_match_offset = (n_offset - match_pos as usize) as i32;
        let n_main_match_len = (refv >> (LCP_SHIFT + TAG_BITS)) as i32;

        if mptr < n_max_matches {
            if n_main_match_offset <= n_max_offset && n_main_match_offset != n_prev_offset {
                if n_prev_offset != 0
                    && n_prev_len > 2
                    && n_main_match_offset == (n_prev_offset - 1)
                    && n_main_match_len == (n_prev_len - 1)
                    && cur_depth_idx.is_some()
                    && n_cur_depth < LCP_MAX
                {
                    n_cur_depth += 1;
                    match_depth[cur_depth_idx.unwrap()] = n_cur_depth as u16;
                } else {
                    matches[mptr].length = n_main_match_len as u32;
                    matches[mptr].offset = n_main_match_offset as u32;
                    match_depth[mptr] = 0;
                    n_cur_depth = 0;
                    cur_depth_idx = Some(mptr);
                    mptr += 1;
                }
                n_prev_len = n_main_match_len;
                n_prev_offset = n_main_match_offset;
            }
        }

        if n_main_match_offset != 0 && n_main_match_offset < 16 && n_main_match_len != 0 {
            match1 = n_main_match_offset as u8;
        }

        if super_ref == 0 {
            break;
        }
        refv = super_ref;
        match_pos = intervals[(refv & POS_MASK) as usize] & EXCL_VISITED_MASK;

        if self_contained {
            let n_match_offset = (n_offset - match_pos as usize) as i32;
            if mptr < n_max_matches {
                let n_match_len = (refv >> (LCP_SHIFT + TAG_BITS)) as i32;
                if n_match_offset <= n_max_offset
                    && n_match_len >= 2
                    && (n_prev_offset - n_match_offset) >= 128
                {
                    if n_prev_offset != 0
                        && n_prev_len > 2
                        && n_match_offset == (n_prev_offset - 1)
                        && n_match_len == (n_prev_len - 1)
                        && cur_depth_idx.is_some()
                        && n_cur_depth < LCP_MAX
                    {
                        n_cur_depth += 1;
                        match_depth[cur_depth_idx.unwrap()] = (n_cur_depth as u16) | 0x8000;
                    } else {
                        matches[mptr].length = n_match_len as u32;
                        matches[mptr].offset = n_match_offset as u32;
                        n_cur_depth = 0;
                        match_depth[mptr] = 0x8000;
                        cur_depth_idx = Some(mptr);
                        mptr += 1;
                    }
                    n_prev_len = n_match_len;
                    n_prev_offset = n_match_offset;
                }
            }
        }
    }

    (mptr, match1)
}

fn skip_matches(c: &mut Compressor, start: usize, end: usize) {
    let mut m = [Match::default(); 1];
    let mut d = [0u16; 1];
    for i in start..end {
        let _ = find_matches_at(c, i, &mut m, &mut d, 0, false);
    }
}

fn find_all_matches(
    c: &mut Compressor,
    n_matches_per_offset: usize,
    start: usize,
    end: usize,
    block_flags: i32,
) {
    let self_contained = (block_flags & 3) == 3;
    for i in start..end {
        // Work on local scratch then write back into c.match_/c.match_depth to satisfy borrow rules.
        let base = (i - start) * n_matches_per_offset;
        let mut local_m = vec![Match::default(); n_matches_per_offset];
        let mut local_d = vec![0u16; n_matches_per_offset];
        let (n_matches, m1) = find_matches_at(
            c,
            i,
            &mut local_m,
            &mut local_d,
            n_matches_per_offset,
            self_contained,
        );
        // index into the per-position arrays uses NMATCHES_PER_INDEX stride; but here the find_all
        // store stride is n_matches_per_offset (== NMATCHES_PER_INDEX in practice).
        for k in 0..n_matches_per_offset {
            if k < n_matches {
                c.match_[base + k] = local_m[k];
                c.match_depth[base + k] = local_d[k];
            } else {
                c.match_[base + k] = Match::default();
                c.match_depth[base + k] = 0;
            }
        }
        c.match1[i - start] = m1;
    }
}

// ---------------------------------------------------------------------------------------------
// apultra_insert_forward_match
// ---------------------------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn insert_forward_match(
    c: &mut Compressor,
    win: &[u8],
    i: usize,
    n_match_offset: i32,
    start_offset: usize,
    end_offset: usize,
    n_arrivals_per_position: usize,
    depth: i32,
) {
    let arrival_base = (i - start_offset) * n_arrivals_per_position;

    let mut j = 0;
    while j < n_arrivals_per_position && c.arrival[arrival_base + j].from_slot != 0 {
        if c.arrival[arrival_base + j].follows_literal != 0 {
            let n_rep_offset = c.arrival[arrival_base + j].rep_offset as i32;

            if n_match_offset != n_rep_offset {
                let n_rep_pos = c.arrival[arrival_base + j].rep_pos as usize;

                if n_rep_pos >= start_offset
                    && (n_rep_pos + 1) < end_offset
                    && c.visited[n_rep_pos] != n_match_offset
                {
                    c.visited[n_rep_pos] = n_match_offset;

                    let fwd_base = (n_rep_pos - start_offset) * NMATCHES_PER_INDEX;

                    if c.match_[fwd_base + (NMATCHES_PER_INDEX - 1)].length == 0 {
                        if (n_rep_pos as i32) >= n_match_offset {
                            let ps = n_rep_pos;
                            if win[ps..ps + 2]
                                == win
                                    [ps - n_match_offset as usize..ps - n_match_offset as usize + 2]
                            {
                                if n_rep_offset != 0 {
                                    let n_len0 = c.rle_len[n_rep_pos - n_match_offset as usize];
                                    let n_len1 = c.rle_len[n_rep_pos];
                                    let n_min_len = if n_len0 < n_len1 { n_len0 } else { n_len1 };

                                    let mut n_max_rep_len = (end_offset - n_rep_pos) as i32;
                                    if n_max_rep_len > LCP_MAX {
                                        n_max_rep_len = LCP_MAX;
                                    }

                                    let win_max = ps + n_max_rep_len as usize;
                                    let mut p = ps + n_min_len as usize;
                                    if p > win_max {
                                        p = win_max;
                                    }
                                    let mo = n_match_offset as usize;
                                    while p + 8 < win_max
                                        && win[p..p + 8] == win[p - mo..p - mo + 8]
                                    {
                                        p += 8;
                                    }
                                    while p + 4 < win_max
                                        && win[p..p + 4] == win[p - mo..p - mo + 4]
                                    {
                                        p += 4;
                                    }
                                    while p < win_max && win[p] == win[p - mo] {
                                        p += 1;
                                    }
                                    let n_cur_rep_len = (p - ps) as u32;

                                    let mut r = 0usize;
                                    while c.match_[fwd_base + r].length != 0 {
                                        if c.match_[fwd_base + r].offset as i32 == n_match_offset
                                            && (c.match_depth[fwd_base + r] & 0x3fff) == 0
                                        {
                                            if (c.match_[fwd_base + r].length) < n_cur_rep_len {
                                                c.match_[fwd_base + r].length = n_cur_rep_len;
                                                c.match_depth[fwd_base + r] = 0;
                                            }
                                            break;
                                        }
                                        r += 1;
                                    }

                                    if c.match_[fwd_base + r].length == 0 {
                                        c.match_[fwd_base + r].length = n_cur_rep_len;
                                        c.match_[fwd_base + r].offset = n_match_offset as u32;
                                        c.match_depth[fwd_base + r] = 0;

                                        if depth < 9 {
                                            insert_forward_match(
                                                c,
                                                win,
                                                n_rep_pos,
                                                n_match_offset,
                                                start_offset,
                                                end_offset,
                                                n_arrivals_per_position,
                                                depth + 1,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        j += 1;
    }
}

// ---------------------------------------------------------------------------------------------
// apultra_optimize_forward: the multi-arrival optimal parse
// ---------------------------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn optimize_forward(
    c: &mut Compressor,
    g: &[i8; 2048],
    win: &[u8],
    start_offset: usize,
    end_offset: usize,
    insert_forward_reps: bool,
    cur_rep_match_offset: &mut i32,
    block_flags: i32,
    napp: usize, // n_arrivals_per_position
) {
    let off = start_offset * napp;

    if (end_offset - start_offset) > c.block_size {
        return;
    }

    // clear arrivals for [start_offset ..= end_offset]
    for i in (start_offset..=end_offset).map(|p| p * napp - off) {
        for j in 0..napp {
            c.arrival[i + j] = Arrival::default();
            c.arrival[i + j].cost = 0x40000000;
        }
    }

    {
        let idx = start_offset * napp - off;
        c.arrival[idx].cost = 0;
        c.arrival[idx].from_slot = -1;
        c.arrival[idx].rep_offset = *cur_rep_match_offset as u32;
    }

    if insert_forward_reps {
        for v in c.visited[start_offset..end_offset].iter_mut() {
            *v = 0;
        }
    }

    // rep_idx holds up to 2*napp+1 entries (a (slot, max_rep_len) pair per arrival, plus the -1
    // terminator). The C uses a fixed `2 * NARRIVALS_PER_POSITION_MAX + 1` stack array; we size it
    // to the actual arrival count so the wider-beam variants stay in bounds.
    let mut rep_idx = vec![0i32; 2 * napp + 1];

    let mut i = start_offset;
    while i != end_offset {
        let cur = (i - start_offset) * napp;
        let next = cur + napp;

        let n_match1_offs = c.match1[i - start_offset];
        let n_short_offset;
        let n_short_len;
        let n_literal_score;
        let n_literal_cost;

        if (win[i] != 0 && n_match1_offs == 0) || (i == start_offset && (block_flags & 1) != 0) {
            n_short_offset = 0;
            n_short_len = 0;
            n_literal_score = 1;
            n_literal_cost = 9;
        } else {
            n_short_offset = if win[i] != 0 { n_match1_offs as i32 } else { 0 };
            n_short_len = 1;
            n_literal_score = if n_short_offset != 0 { 3 } else { 1 };
            n_literal_cost = 4 + TOKEN_SIZE_4BIT_MATCH;
        }

        if c.arrival[next].from_slot != 0 {
            let mut j = 0;
            while j < napp && c.arrival[cur + j].from_slot != 0 {
                let n_coding_choice_cost = c.arrival[cur + j].cost + n_literal_cost;
                let n_score = c.arrival[cur + j].score + n_literal_score;
                let n_rep_offset = c.arrival[cur + j].rep_offset;

                if n_coding_choice_cost < c.arrival[next + napp - 1].cost
                    || (n_coding_choice_cost == c.arrival[next + napp - 1].cost
                        && n_score < c.arrival[next + napp - 1].score
                        && n_rep_offset != c.arrival[next + napp - 1].rep_offset)
                {
                    let mut exists = false;
                    let mut n = 0usize;
                    while c.arrival[next + n].cost < n_coding_choice_cost {
                        if c.arrival[next + n].rep_offset == n_rep_offset {
                            exists = true;
                            break;
                        }
                        n += 1;
                    }

                    if !exists {
                        while c.arrival[next + n].cost == n_coding_choice_cost
                            && n_score >= c.arrival[next + n].score
                        {
                            if c.arrival[next + n].rep_offset == n_rep_offset {
                                exists = true;
                                break;
                            }
                            n += 1;
                        }

                        if !exists {
                            let mut z = n;
                            while z < napp - 1 && c.arrival[next + z].cost == n_coding_choice_cost {
                                if c.arrival[next + z].rep_offset == n_rep_offset {
                                    exists = true;
                                    break;
                                }
                                z += 1;
                            }

                            if !exists {
                                while z < napp - 1 && c.arrival[next + z].from_slot != 0 {
                                    if c.arrival[next + z].rep_offset == n_rep_offset {
                                        break;
                                    }
                                    z += 1;
                                }

                                for t in (n..z).rev() {
                                    c.arrival[next + t + 1] = c.arrival[next + t];
                                }

                                let rep_pos = c.arrival[cur + j].rep_pos;
                                let pd = &mut c.arrival[next + n];
                                pd.cost = n_coding_choice_cost;
                                pd.from_pos = i as u32;
                                pd.from_slot = (j + 1) as i32;
                                pd.follows_literal = 1;
                                pd.rep_offset = n_rep_offset;
                                pd.short_offset = n_short_offset as u32;
                                pd.rep_pos = rep_pos;
                                pd.match_len = n_short_len as u32;
                                pd.score = n_score;
                            }
                        }
                    }
                }
                j += 1;
            }
        } else {
            let mut j = 0;
            let mut d = 0usize;
            while j < napp && c.arrival[cur + j].from_slot != 0 {
                let src = c.arrival[cur + j];
                let pd = &mut c.arrival[next + d];
                pd.cost = src.cost + n_literal_cost;
                pd.from_pos = i as u32;
                pd.from_slot = (j + 1) as i32;
                pd.follows_literal = 1;
                pd.rep_offset = src.rep_offset;
                pd.short_offset = n_short_offset as u32;
                pd.rep_pos = src.rep_pos;
                pd.match_len = n_short_len as u32;
                pd.score = src.score + n_literal_score;
                j += 1;
                d += 1;
            }
        }

        if i == start_offset && (block_flags & 1) != 0 {
            i += 1;
            continue;
        }

        let match_base = (i - start_offset) * NMATCHES_PER_INDEX;
        let mut n_num_arrivals_for_this_pos = 0usize;
        while n_num_arrivals_for_this_pos < napp
            && c.arrival[cur + n_num_arrivals_for_this_pos].from_slot != 0
        {
            n_num_arrivals_for_this_pos += 1;
        }

        let mut n_overall_min_rep_len = 0i32;
        let mut n_overall_max_rep_len = 0i32;

        let mut n_num_rep = 0usize;

        if (i + 2) <= end_offset {
            let mut n_max_rep_len_for_pos = (end_offset - i) as i32;
            if n_max_rep_len_for_pos > LCP_MAX {
                n_max_rep_len_for_pos = LCP_MAX;
            }
            let win_max = i + n_max_rep_len_for_pos as usize;

            for j in 0..n_num_arrivals_for_this_pos {
                if c.arrival[cur + j].follows_literal != 0 {
                    let n_rep_offset = c.arrival[cur + j].rep_offset as i32;
                    if (i as i32) >= n_rep_offset && n_rep_offset != 0 {
                        let ro = n_rep_offset as usize;
                        if win[i..i + 2] == win[i - ro..i - ro + 2] {
                            let n_len0 = c.rle_len[i - ro];
                            let n_len1 = c.rle_len[i];
                            let n_min_len = if n_len0 < n_len1 { n_len0 } else { n_len1 };
                            let mut p = i + n_min_len as usize;
                            if p > win_max {
                                p = win_max;
                            }
                            while p + 8 < win_max && win[p..p + 8] == win[p - ro..p - ro + 8] {
                                p += 8;
                            }
                            while p + 4 < win_max && win[p..p + 4] == win[p - ro..p - ro + 4] {
                                p += 4;
                            }
                            while p < win_max && win[p] == win[p - ro] {
                                p += 1;
                            }
                            let n_cur_max_len = (p - i) as i32;

                            rep_idx[n_num_rep] = j as i32;
                            n_num_rep += 1;
                            rep_idx[n_num_rep] = n_cur_max_len;
                            n_num_rep += 1;

                            if n_overall_max_rep_len < n_cur_max_len {
                                n_overall_max_rep_len = n_cur_max_len;
                            }
                        }
                    }
                }
            }
        }
        rep_idx[n_num_rep] = -1;

        let mut m = 0usize;
        while m < NMATCHES_PER_INDEX && c.match_[match_base + m].length != 0 {
            let mut n_orig_match_len = c.match_[match_base + m].length as i32;
            let n_orig_match_offset = c.match_[match_base + m].offset as i32;
            let n_orig_match_depth = (c.match_depth[match_base + m] & 0x3fff) as u32;
            let n_score_penalty = 3 + (c.match_depth[match_base + m] >> 15) as i32;

            if (i + n_orig_match_len as usize) > end_offset {
                n_orig_match_len = (end_offset - i) as i32;
            }

            let mut d: u32 = 0;
            loop {
                let n_match_len = n_orig_match_len - d as i32;
                let n_match_offset = n_orig_match_offset - d as i32;

                if insert_forward_reps {
                    insert_forward_match(
                        c,
                        win,
                        i,
                        n_match_offset,
                        start_offset,
                        end_offset,
                        napp,
                        0,
                    );
                }

                if n_match_len >= 2 {
                    let n_starting_match_len;
                    let n_jump_match_len;
                    let mut n_no_rep_match_offset_cost_for_lit = [0i32; 2];
                    let n_no_rep_match_offset_cost_delta;
                    let n_min_match_len_for_offset;
                    let n_no_rep_cost_adjustment = if n_match_len >= LCP_MAX { 1 } else { 0 };

                    if n_match_offset < MINMATCH3_OFFSET {
                        n_min_match_len_for_offset = 2;
                    } else if n_match_offset < MINMATCH4_OFFSET {
                        n_min_match_len_for_offset = 3;
                    } else {
                        n_min_match_len_for_offset = 4;
                    }

                    if n_match_len >= LEAVE_ALONE_MATCH_SIZE && (i as i32) >= n_match_len {
                        // Reference heuristic: a match >= 120 long is only tried at its full length
                        // (intermediate cuts are assumed never to win). Kept in both tiers; the
                        // thorough beam instead widens the per-position jump cap below.
                        n_starting_match_len = n_match_len;
                    } else {
                        n_starting_match_len = 2;
                    }

                    // Reference heuristic: for long matches only lengths 2..=90 and the full length
                    // are tried as parse cuts; the middle is skipped. The thorough beam raises this
                    // cap to THOROUGH_JUMP so more intermediate cuts are considered, while still
                    // bounding the work on periodic data (where matches span the whole input).
                    let jump_cap = if c.thorough { THOROUGH_JUMP } else { 90 };
                    if (block_flags & 3) == 3 && n_match_len > jump_cap && i as i32 >= jump_cap {
                        n_jump_match_len = jump_cap;
                    } else {
                        n_jump_match_len = n_match_len + 1;
                    }

                    if n_starting_match_len <= 3 && n_match_offset < 128 {
                        n_no_rep_match_offset_cost_for_lit[1] = 8 + TOKEN_SIZE_7BIT_MATCH;
                        n_no_rep_match_offset_cost_for_lit[0] = 8 + TOKEN_SIZE_7BIT_MATCH;
                    } else {
                        n_no_rep_match_offset_cost_for_lit[0] =
                            8 + TOKEN_SIZE_LARGE_MATCH + gamma2_size(g, (n_match_offset >> 8) + 2);
                        n_no_rep_match_offset_cost_for_lit[1] =
                            8 + TOKEN_SIZE_LARGE_MATCH + gamma2_size(g, (n_match_offset >> 8) + 3);
                    }
                    n_no_rep_match_offset_cost_delta = n_no_rep_match_offset_cost_for_lit[1]
                        - n_no_rep_match_offset_cost_for_lit[0];

                    let mut k = n_starting_match_len;
                    while k <= n_match_len {
                        let n_rep_match_match_len_cost = gamma2_size(g, k);
                        let dest = cur + (k as usize) * napp;

                        if k >= n_min_match_len_for_offset {
                            let n_no_rep_match_match_len_cost;
                            if k <= 3 && n_match_offset < 128 {
                                n_no_rep_match_match_len_cost = 0;
                            } else if n_match_offset < 128 || n_match_offset >= MINMATCH4_OFFSET {
                                n_no_rep_match_match_len_cost = gamma2_size(g, k - 2);
                            } else if n_match_offset < MINMATCH3_OFFSET {
                                n_no_rep_match_match_len_cost = n_rep_match_match_len_cost;
                            } else {
                                n_no_rep_match_match_len_cost = gamma2_size(g, k - 1);
                            }

                            let mut j = 0usize;
                            while j < n_num_arrivals_for_this_pos {
                                let n_follows_literal = c.arrival[cur + j].follows_literal as usize;
                                if n_match_offset != c.arrival[cur + j].rep_offset as i32
                                    || n_follows_literal == 0
                                {
                                    let n_match_cmd_cost = n_no_rep_match_match_len_cost
                                        + n_no_rep_match_offset_cost_for_lit[n_follows_literal];
                                    let n_coding_choice_cost =
                                        c.arrival[cur + j].cost + n_match_cmd_cost;

                                    if n_coding_choice_cost <= (c.arrival[dest + napp - 1].cost + 1)
                                    {
                                        let n_score = c.arrival[cur + j].score + n_score_penalty;

                                        if n_coding_choice_cost < c.arrival[dest + napp - 2].cost
                                            || (n_coding_choice_cost
                                                == c.arrival[dest + napp - 2].cost
                                                && n_score < c.arrival[dest + napp - 2].score
                                                && (n_coding_choice_cost
                                                    != c.arrival[dest + napp - 1].cost
                                                    || n_match_offset
                                                        != c.arrival[dest + napp - 1].rep_offset
                                                            as i32))
                                        {
                                            let mut exists = false;
                                            let mut n = 0usize;
                                            while c.arrival[dest + n].cost < n_coding_choice_cost {
                                                if c.arrival[dest + n].rep_offset as i32
                                                    == n_match_offset
                                                {
                                                    exists = true;
                                                    break;
                                                }
                                                n += 1;
                                            }

                                            if !exists {
                                                let n_revised_coding_choice_cost =
                                                    n_coding_choice_cost - n_no_rep_cost_adjustment;

                                                while n < napp - 1
                                                    && c.arrival[dest + n].cost
                                                        == n_revised_coding_choice_cost
                                                    && n_score >= c.arrival[dest + n].score
                                                {
                                                    if c.arrival[dest + n].rep_offset as i32
                                                        == n_match_offset
                                                    {
                                                        exists = true;
                                                        break;
                                                    }
                                                    n += 1;
                                                }

                                                if !exists && n < napp - 1 {
                                                    let mut z = n;
                                                    while z < napp - 1
                                                        && c.arrival[dest + z].cost
                                                            == n_coding_choice_cost
                                                    {
                                                        if c.arrival[dest + z].rep_offset as i32
                                                            == n_match_offset
                                                        {
                                                            exists = true;
                                                            break;
                                                        }
                                                        z += 1;
                                                    }

                                                    if !exists {
                                                        while z < napp - 1
                                                            && c.arrival[dest + z].from_slot != 0
                                                        {
                                                            if c.arrival[dest + z].rep_offset as i32
                                                                == n_match_offset
                                                            {
                                                                break;
                                                            }
                                                            z += 1;
                                                        }

                                                        for t in (n..z).rev() {
                                                            c.arrival[dest + t + 1] =
                                                                c.arrival[dest + t];
                                                        }

                                                        let pd = &mut c.arrival[dest + n];
                                                        pd.cost = n_revised_coding_choice_cost;
                                                        pd.from_pos = i as u32;
                                                        pd.from_slot = (j + 1) as i32;
                                                        pd.follows_literal = 0;
                                                        pd.rep_offset = n_match_offset as u32;
                                                        pd.short_offset = 0;
                                                        pd.rep_pos = i as u32;
                                                        pd.match_len = k as u32;
                                                        pd.score = n_score;
                                                    }
                                                }
                                            } else if (n_coding_choice_cost
                                                - c.arrival[dest + n].cost)
                                                >= n_no_rep_match_offset_cost_delta
                                            {
                                                break;
                                            }
                                        }
                                        if c.arrival[cur + j].follows_literal == 0
                                            || n_no_rep_match_offset_cost_delta == 0
                                        {
                                            break;
                                        }
                                    } else {
                                        break;
                                    }
                                }
                                j += 1;
                            }
                        }

                        if k == 3 && n_match_offset < 128 {
                            n_no_rep_match_offset_cost_for_lit[1] = 8 + TOKEN_SIZE_LARGE_MATCH + 2;
                            n_no_rep_match_offset_cost_for_lit[0] = 8 + TOKEN_SIZE_LARGE_MATCH + 2;
                        }

                        if k > n_overall_min_rep_len && k <= n_overall_max_rep_len {
                            let n_rep_match_cmd_cost =
                                TOKEN_SIZE_LARGE_MATCH + 2 + n_rep_match_match_len_cost;

                            if k <= 90 {
                                n_overall_min_rep_len = k;
                            } else if n_overall_max_rep_len == k {
                                n_overall_max_rep_len -= 1;
                            }

                            let mut nc = 0usize;
                            loop {
                                let jj = rep_idx[nc];
                                if jj < 0 {
                                    break;
                                }
                                let j = jj as usize;
                                if rep_idx[nc + 1] >= k {
                                    let n_rep_coding_choice_cost =
                                        c.arrival[cur + j].cost + n_rep_match_cmd_cost;
                                    let n_score = c.arrival[cur + j].score + 2;
                                    let n_rep_offset = c.arrival[cur + j].rep_offset;

                                    if n_rep_coding_choice_cost < c.arrival[dest + napp - 1].cost
                                        || (n_rep_coding_choice_cost
                                            == c.arrival[dest + napp - 1].cost
                                            && n_score < c.arrival[dest + napp - 1].score
                                            && n_rep_offset
                                                != c.arrival[dest + napp - 1].rep_offset)
                                    {
                                        let mut exists = false;
                                        let mut n = 0usize;
                                        while c.arrival[dest + n].cost < n_rep_coding_choice_cost {
                                            if c.arrival[dest + n].rep_offset == n_rep_offset {
                                                exists = true;
                                                break;
                                            }
                                            n += 1;
                                        }

                                        if !exists {
                                            while c.arrival[dest + n].cost
                                                == n_rep_coding_choice_cost
                                                && n_score >= c.arrival[dest + n].score
                                            {
                                                if c.arrival[dest + n].rep_offset == n_rep_offset {
                                                    exists = true;
                                                    break;
                                                }
                                                n += 1;
                                            }

                                            if !exists {
                                                let mut z = n;
                                                while z < napp - 1
                                                    && c.arrival[dest + z].cost
                                                        == n_rep_coding_choice_cost
                                                {
                                                    if c.arrival[dest + z].rep_offset
                                                        == n_rep_offset
                                                    {
                                                        exists = true;
                                                        break;
                                                    }
                                                    z += 1;
                                                }

                                                if !exists {
                                                    while z < napp - 1
                                                        && c.arrival[dest + z].from_slot != 0
                                                    {
                                                        if c.arrival[dest + z].rep_offset
                                                            == n_rep_offset
                                                        {
                                                            break;
                                                        }
                                                        z += 1;
                                                    }

                                                    for t in (n..z).rev() {
                                                        c.arrival[dest + t + 1] =
                                                            c.arrival[dest + t];
                                                    }

                                                    let pd = &mut c.arrival[dest + n];
                                                    pd.cost = n_rep_coding_choice_cost;
                                                    pd.from_pos = i as u32;
                                                    pd.from_slot = (j + 1) as i32;
                                                    pd.follows_literal = 0;
                                                    pd.rep_offset = n_rep_offset;
                                                    pd.short_offset = 0;
                                                    pd.rep_pos = i as u32;
                                                    pd.match_len = k as u32;
                                                    pd.score = n_score;
                                                }
                                            }
                                        }
                                    } else {
                                        break;
                                    }
                                }
                                nc += 2;
                            }
                        }

                        if k == n_jump_match_len {
                            k = n_match_len - 1;
                        }
                        k += 1;
                    }
                }

                if n_orig_match_len >= 512 {
                    break;
                }

                let step = if n_orig_match_depth != 0 {
                    n_orig_match_depth
                } else {
                    1
                };
                d += step;
                if d > n_orig_match_depth {
                    break;
                }
            }
            m += 1;
        }

        i += 1;
    }

    if !insert_forward_reps {
        let mut end_arrival = (i - start_offset) * napp;
        while c.arrival[end_arrival].from_slot > 0
            && (c.arrival[end_arrival].from_pos as usize) < end_offset
        {
            let from_pos = c.arrival[end_arrival].from_pos as usize;
            let from_slot = c.arrival[end_arrival].from_slot;
            let match_len = c.arrival[end_arrival].match_len as i32;
            let rep_offset = c.arrival[end_arrival].rep_offset as i32;
            let short_offset = c.arrival[end_arrival].short_offset as i32;

            c.best_match[from_pos - start_offset].length = match_len;
            c.best_match[from_pos - start_offset].offset = if match_len >= 2 {
                rep_offset
            } else {
                short_offset
            };

            end_arrival = (from_pos - start_offset) * napp + (from_slot as usize - 1);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// apultra_reduce_commands
// ---------------------------------------------------------------------------------------------

fn reduce_commands(
    c: &mut Compressor,
    g: &[i8; 2048],
    win: &[u8],
    start_offset: usize,
    end_offset: usize,
    cur_rep_match_offset: i32,
    block_flags: i32,
) -> bool {
    // pBestMatch is biased by -start_offset in C; here best_match[idx-start_offset].
    let bm = |c: &Compressor, idx: usize| c.best_match[idx - start_offset];
    let mut n_rep_match_offset = cur_rep_match_offset;
    let mut n_follows_literal = 0i32;
    let mut n_did_reduce = false;
    let mut n_last_match_len = 0i32;
    // match1 is biased by -start_offset
    let m1 = |c: &Compressor, idx: usize| c.match1[idx - start_offset];

    let mut i = start_offset + (block_flags & 1) as usize;
    while i < end_offset {
        let p_len = bm(c, i).length;

        if p_len <= 1
            && (i + 1) < end_offset
            && bm(c, i + 1).length >= 2
            && bm(c, i + 1).length < MAX_VARLEN
            && bm(c, i + 1).offset != 0
            && (i as i32) >= bm(c, i + 1).offset
            && (i + bm(c, i + 1).length as usize + 1) <= end_offset
            && {
                let l = bm(c, i + 1).length as usize + 1;
                let o = bm(c, i + 1).offset as usize;
                win[i - o..i - o + l] == win[i..i + l]
            }
        {
            let nxt = bm(c, i + 1);
            if nxt.offset < MINMATCH4_OFFSET
                || (nxt.length + 1) >= 4
                || (nxt.offset == n_rep_match_offset && n_follows_literal != 0)
            {
                let mut n_cur_partial = if p_len == 1 {
                    TOKEN_SIZE_4BIT_MATCH + 4
                } else {
                    1 + 8
                };
                if nxt.offset == n_rep_match_offset {
                    n_cur_partial += TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, nxt.length);
                } else {
                    n_cur_partial += get_offset_varlen_size(g, nxt.length, nxt.offset, 1)
                        + get_match_varlen_size(g, nxt.length, nxt.offset);
                }

                let n_reduced_partial;
                if nxt.offset == n_rep_match_offset && n_follows_literal != 0 {
                    n_reduced_partial = TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, nxt.length + 1);
                } else {
                    n_reduced_partial =
                        get_offset_varlen_size(g, nxt.length + 1, nxt.offset, n_follows_literal)
                            + get_match_varlen_size(g, nxt.length + 1, nxt.offset);
                }

                if n_reduced_partial < n_cur_partial
                    || (n_follows_literal == 0 && n_last_match_len >= LCP_MAX)
                {
                    c.best_match[i - start_offset].length = nxt.length + 1;
                    c.best_match[i - start_offset].offset = nxt.offset;
                    c.best_match[i + 1 - start_offset].length = 0;
                    c.best_match[i + 1 - start_offset].offset = 0;
                    n_did_reduce = true;
                    continue;
                }
            }
        }

        if bm(c, i).length >= 2 {
            let p_match = bm(c, i);
            if p_match.length < LCP_MAX {
                let mut n_next_index = i + p_match.length as usize;
                let mut n_next_follows_literal = 0i32;

                while n_next_index < end_offset && bm(c, n_next_index).length < 2 {
                    n_next_index += 1;
                    n_next_follows_literal = 1;
                }

                if n_next_index < end_offset && bm(c, n_next_index).length >= 2 {
                    let mut n_cannot_encode = 0i32;
                    let next_m = bm(c, n_next_index);

                    if n_rep_match_offset != 0
                        && n_rep_match_offset != p_match.offset
                        && next_m.offset != 0
                        && p_match.offset != next_m.offset
                        && n_next_follows_literal != 0
                    {
                        if (i as i32) >= next_m.offset
                            && (i + p_match.length as usize) <= end_offset
                        {
                            if (next_m.offset < MINMATCH3_OFFSET || p_match.length >= 3)
                                && (next_m.offset < MINMATCH4_OFFSET || p_match.length >= 4)
                            {
                                let mut n_max_len = 0i32;
                                let pos = i;
                                let no = next_m.offset as usize;
                                while (n_max_len + 8) < p_match.length
                                    && win[pos + n_max_len as usize - no
                                        ..pos + n_max_len as usize - no + 8]
                                        == win
                                            [pos + n_max_len as usize..pos + n_max_len as usize + 8]
                                {
                                    n_max_len += 8;
                                }
                                while (n_max_len + 4) < p_match.length
                                    && win[pos + n_max_len as usize - no
                                        ..pos + n_max_len as usize - no + 4]
                                        == win
                                            [pos + n_max_len as usize..pos + n_max_len as usize + 4]
                                {
                                    n_max_len += 4;
                                }
                                while n_max_len < p_match.length
                                    && win[pos + n_max_len as usize - no]
                                        == win[pos + n_max_len as usize]
                                {
                                    n_max_len += 1;
                                }

                                if n_max_len >= p_match.length {
                                    c.best_match[i - start_offset].offset = next_m.offset;
                                    n_did_reduce = true;
                                } else if n_max_len >= 2
                                    && ((n_follows_literal != 0
                                        && n_rep_match_offset == next_m.offset)
                                        || ((next_m.offset < MINMATCH3_OFFSET || n_max_len >= 3)
                                            && (next_m.offset < MINMATCH4_OFFSET
                                                || n_max_len >= 4)))
                                {
                                    let mut n_partial_before = get_offset_varlen_size(
                                        g,
                                        p_match.length,
                                        p_match.offset,
                                        n_follows_literal,
                                    );
                                    n_partial_before +=
                                        get_match_varlen_size(g, p_match.length, p_match.offset);
                                    n_partial_before +=
                                        get_offset_varlen_size(g, next_m.length, next_m.offset, 1);
                                    n_partial_before +=
                                        get_match_varlen_size(g, next_m.length, next_m.offset);

                                    let mut n_partial_after = get_offset_varlen_size(
                                        g,
                                        n_max_len,
                                        next_m.offset,
                                        n_follows_literal,
                                    );
                                    if n_follows_literal != 0 && n_rep_match_offset == next_m.offset
                                    {
                                        n_partial_after += gamma2_size(g, n_max_len);
                                    } else {
                                        n_partial_after +=
                                            get_match_varlen_size(g, n_max_len, next_m.offset);
                                    }
                                    n_partial_after += TOKEN_SIZE_LARGE_MATCH + 2;
                                    n_partial_after += gamma2_size(g, next_m.length);

                                    for jj in n_max_len..p_match.length {
                                        let idx = i + jj as usize;
                                        if win[idx] == 0 || m1(c, idx) != 0 {
                                            n_partial_after += TOKEN_SIZE_4BIT_MATCH + 4;
                                        } else {
                                            n_partial_after += 1 + 8;
                                        }
                                    }

                                    if n_partial_after < n_partial_before {
                                        let n_orig_len = p_match.length;
                                        c.best_match[i - start_offset].offset = next_m.offset;
                                        c.best_match[i - start_offset].length = n_max_len;
                                        for jj in n_max_len..n_orig_len {
                                            let idx = i + jj as usize;
                                            c.best_match[idx - start_offset].offset =
                                                m1(c, idx) as i32;
                                            c.best_match[idx - start_offset].length =
                                                if win[idx] != 0 && m1(c, idx) == 0 {
                                                    0
                                                } else {
                                                    1
                                                };
                                        }
                                        n_did_reduce = true;
                                        continue;
                                    }
                                }
                            }
                        }
                    }

                    // Reload p_match (offset may have changed above)
                    let p_match = bm(c, i);
                    let next_m = bm(c, n_next_index);

                    let n_cur_command_size;
                    if p_match.offset == n_rep_match_offset && n_follows_literal != 0 {
                        n_cur_command_size =
                            TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, p_match.length);
                    } else {
                        n_cur_command_size =
                            get_offset_varlen_size(
                                g,
                                p_match.length,
                                p_match.offset,
                                n_follows_literal,
                            ) + get_match_varlen_size(g, p_match.length, p_match.offset);
                    }

                    let n_next_command_size;
                    if next_m.offset == p_match.offset
                        && n_next_follows_literal != 0
                        && next_m.length >= 2
                    {
                        n_next_command_size =
                            TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, next_m.length);
                    } else {
                        n_next_command_size =
                            get_offset_varlen_size(
                                g,
                                next_m.length,
                                next_m.offset,
                                n_next_follows_literal,
                            ) + get_match_varlen_size(g, next_m.length, next_m.offset);
                    }

                    let n_original_combined = n_cur_command_size + n_next_command_size;

                    let mut n_reduced_command_size = 0i32;
                    for jj in 0..p_match.length {
                        let idx = i + jj as usize;
                        if win[idx] == 0 || m1(c, idx) != 0 {
                            n_reduced_command_size += TOKEN_SIZE_4BIT_MATCH + 4;
                        } else {
                            n_reduced_command_size += 1 + 8;
                        }
                    }

                    if next_m.offset == n_rep_match_offset && next_m.length >= 2 {
                        n_reduced_command_size +=
                            TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, next_m.length);
                    } else if (next_m.length < 3 && next_m.offset >= MINMATCH3_OFFSET)
                        || (next_m.length < 4 && next_m.offset >= MINMATCH4_OFFSET)
                    {
                        n_cannot_encode = 1;
                    } else {
                        n_reduced_command_size +=
                            get_offset_varlen_size(g, next_m.length, next_m.offset, 1)
                                + get_match_varlen_size(g, next_m.length, next_m.offset);
                    }

                    if n_original_combined > n_reduced_command_size && n_cannot_encode == 0 {
                        let n_match_len = p_match.length;
                        for jj in 0..n_match_len {
                            let idx = i + jj as usize;
                            c.best_match[idx - start_offset].offset = m1(c, idx) as i32;
                            c.best_match[idx - start_offset].length =
                                if win[idx] != 0 && m1(c, idx) == 0 {
                                    0
                                } else {
                                    1
                                };
                        }
                        n_did_reduce = true;
                        continue;
                    }
                }
            }

            // Join large matches
            let p_match = bm(c, i);
            let tail = i + p_match.length as usize;
            if tail < end_offset
                && p_match.offset > 0
                && bm(c, tail).offset > 0
                && bm(c, tail).length >= 2
                && (p_match.length + bm(c, tail).length) <= MAX_VARLEN
                && (tail as i32) >= p_match.offset
                && (tail as i32) >= bm(c, tail).offset
                && (tail + bm(c, tail).length as usize) <= end_offset
                && {
                    let a = tail - p_match.offset as usize;
                    let b = tail - bm(c, tail).offset as usize;
                    let l = bm(c, tail).length as usize;
                    win[a..a + l] == win[b..b + l]
                }
            {
                let n_match_len = p_match.length;
                let tail_m = bm(c, tail);
                let mut n_next_index = i + n_match_len as usize + tail_m.length as usize;
                let mut n_next_follows_literal = 0i32;
                let mut n_cannot_encode = 0i32;

                while n_next_index < end_offset && bm(c, n_next_index).length < 2 {
                    n_next_index += 1;
                    n_next_follows_literal = 1;
                }

                let mut n_cur_command_size;
                if p_match.offset == n_rep_match_offset && n_follows_literal != 0 {
                    n_cur_command_size = TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, n_match_len);
                } else {
                    n_cur_command_size =
                        get_offset_varlen_size(g, n_match_len, p_match.offset, n_follows_literal)
                            + get_match_varlen_size(g, n_match_len, p_match.offset);
                }

                n_cur_command_size += get_offset_varlen_size(g, tail_m.length, tail_m.offset, 0)
                    + get_match_varlen_size(g, tail_m.length, tail_m.offset);

                if n_next_index < end_offset && bm(c, n_next_index).length >= 2 {
                    let nm = bm(c, n_next_index);
                    if nm.offset == tail_m.offset && n_next_follows_literal != 0 {
                        n_cur_command_size +=
                            TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, nm.length);
                    } else {
                        n_cur_command_size +=
                            get_offset_varlen_size(g, nm.length, nm.offset, n_next_follows_literal)
                                + get_match_varlen_size(g, nm.length, nm.offset);
                    }
                }

                let mut n_reduced_command_size;
                if p_match.offset == n_rep_match_offset && n_follows_literal != 0 {
                    n_reduced_command_size =
                        TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, n_match_len + tail_m.length);
                } else {
                    n_reduced_command_size =
                        get_offset_varlen_size(
                            g,
                            n_match_len + tail_m.length,
                            p_match.offset,
                            n_follows_literal,
                        ) + get_match_varlen_size(g, n_match_len + tail_m.length, p_match.offset);
                }

                if n_next_index < end_offset && bm(c, n_next_index).length >= 2 {
                    let nm = bm(c, n_next_index);
                    if nm.offset == p_match.offset && n_next_follows_literal != 0 {
                        n_reduced_command_size +=
                            TOKEN_SIZE_LARGE_MATCH + 2 + gamma2_size(g, nm.length);
                    } else {
                        n_reduced_command_size +=
                            get_offset_varlen_size(g, nm.length, nm.offset, n_next_follows_literal)
                                + get_match_varlen_size(g, nm.length, nm.offset);
                        if (nm.offset >= MINMATCH3_OFFSET && nm.length < 3)
                            || (nm.offset >= MINMATCH4_OFFSET && nm.length < 4)
                        {
                            n_cannot_encode = 1;
                        }
                    }
                }

                if n_cur_command_size >= n_reduced_command_size && n_cannot_encode == 0 {
                    c.best_match[i - start_offset].length += tail_m.length;
                    c.best_match[tail - start_offset].length = 0;
                    c.best_match[tail - start_offset].offset = 0;
                    n_did_reduce = true;
                    continue;
                }
            }

            let p_match = bm(c, i);
            n_rep_match_offset = p_match.offset;
            n_follows_literal = 0;
            n_last_match_len = p_match.length;
            i += p_match.length as usize;
        } else {
            i += 1;
            n_follows_literal = 1;
            n_last_match_len = 0;
        }
    }

    n_did_reduce
}

// ---------------------------------------------------------------------------------------------
// apultra_write_block
// ---------------------------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn write_block(
    c: &mut Compressor,
    w: &mut BitWriter,
    win: &[u8],
    start_offset: usize,
    end_offset: usize,
    follows_literal: &mut i32,
    cur_rep_match_offset: &mut i32,
    block_flags: i32,
) {
    let n_max_offset = c.max_offset;
    let mut n_rep_match_offset = *cur_rep_match_offset;
    let mut n_cur_follows_literal = *follows_literal;

    if block_flags & 1 != 0 {
        w.write_byte(win[start_offset]);
        n_cur_follows_literal = 1;
    }

    let bm = |c: &Compressor, idx: usize| c.best_match[idx - start_offset];

    let mut i = start_offset + (block_flags & 1) as usize;
    while i < end_offset {
        let p_match = bm(c, i);

        if p_match.length >= 2 {
            let n_match_len = p_match.length;
            let n_match_offset = p_match.offset;

            // (we trust the encoder produced valid offsets; MIN/MAX checks omitted as panics)
            let _ = (MIN_OFFSET, n_max_offset);

            if n_match_offset == n_rep_match_offset && n_cur_follows_literal != 0 {
                w.write_bits(TOKEN_CODE_LARGE_MATCH, TOKEN_SIZE_LARGE_MATCH);
                w.write_bits(0, 2);
                w.write_gamma2(n_match_len);
                n_cur_follows_literal = 0;
            } else if n_match_len <= 3 && n_match_offset < 128 {
                w.write_bits(TOKEN_CODE_7BIT_MATCH, TOKEN_SIZE_7BIT_MATCH);
                w.write_byte((((n_match_offset & 0x7f) << 1) | (n_match_len - 2)) as u8);
                n_cur_follows_literal = 0;
                n_rep_match_offset = n_match_offset;
            } else {
                w.write_bits(TOKEN_CODE_LARGE_MATCH, TOKEN_SIZE_LARGE_MATCH);
                w.write_gamma2((n_match_offset >> 8) + 2 + (n_cur_follows_literal & 1));
                w.write_byte((n_match_offset & 0xff) as u8);
                if n_match_offset < 128 || n_match_offset >= MINMATCH4_OFFSET {
                    w.write_gamma2(n_match_len - 2);
                } else if n_match_offset < MINMATCH3_OFFSET {
                    w.write_gamma2(n_match_len);
                } else {
                    w.write_gamma2(n_match_len - 1);
                }
                n_cur_follows_literal = 0;
                n_rep_match_offset = n_match_offset;
            }

            i += n_match_len as usize;
        } else if p_match.length == 1 {
            let n_match_offset = p_match.offset;
            w.write_bits(TOKEN_CODE_4BIT_MATCH, TOKEN_SIZE_4BIT_MATCH);
            w.write_bits(n_match_offset, 4);
            i += 1;
            n_cur_follows_literal = 1;
        } else {
            w.write_bits(0, 1);
            w.write_byte(win[i]);
            i += 1;
            n_cur_follows_literal = 1;
        }
    }

    if block_flags & 2 != 0 {
        w.write_bits(TOKEN_CODE_7BIT_MATCH, TOKEN_SIZE_7BIT_MATCH);
        w.write_byte(0x00);
    }

    *cur_rep_match_offset = n_rep_match_offset;
    *follows_literal = n_cur_follows_literal;
}

// ---------------------------------------------------------------------------------------------
// apultra_optimize_and_write_block
// ---------------------------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn optimize_and_write_block(
    c: &mut Compressor,
    g: &[i8; 2048],
    win: &[u8],
    n_previous_block_size: usize,
    n_in_data_size: usize,
    w: &mut BitWriter,
    cur_follows_literal: &mut i32,
    cur_rep_match_offset: &mut i32,
    block_flags: i32,
) {
    let end_offset = n_previous_block_size + n_in_data_size;
    let napp = c.max_arrivals;

    for bm in c.best_match[..c.block_size].iter_mut() {
        *bm = FinalMatch::default();
    }

    // rle_len
    let mut i = 0;
    while i < end_offset {
        let mut range_start = i;
        let ch = win[range_start];
        loop {
            i += 1;
            if !(i < end_offset && win[i] == ch) {
                break;
            }
        }
        while range_start < i {
            c.rle_len[range_start] = (i - range_start) as i32;
            range_start += 1;
        }
    }

    if (block_flags & 3) == 3 {
        // Supplement 2 and 3-byte matches
        for v in c.first_offset_for_byte.iter_mut() {
            *v = -1;
        }
        for v in c.next_offset_for_pos[..n_in_data_size].iter_mut() {
            *v = -1;
        }

        for n_position in n_previous_block_size..(end_offset.saturating_sub(1)) {
            let key = (win[n_position] as usize) | ((win[n_position + 1] as usize) << 8);
            c.next_offset_for_pos[n_position - n_previous_block_size] =
                c.first_offset_for_byte[key];
            c.first_offset_for_byte[key] = n_position as i32;
        }

        for n_position in (n_previous_block_size + 1)..(end_offset.saturating_sub(1)) {
            let mbase = (n_position - n_previous_block_size) * NMATCHES_PER_INDEX;
            let mut m = 0usize;
            let mut n_inserted = 0;

            while m < 15 && c.match_[mbase + m].length != 0 {
                m += 1;
            }

            let mut n_match_pos = c.next_offset_for_pos[n_position - n_previous_block_size];
            while m < 15 && n_match_pos >= 0 {
                let n_match_offset = n_position as i32 - n_match_pos;
                if n_match_offset <= c.max_offset {
                    let mut n_already_exists = false;
                    for e in 0..m {
                        if c.match_[mbase + e].offset as i32 == n_match_offset
                            || (c.match_[mbase + e].offset as i32
                                - (c.match_depth[mbase + e] & 0x3fff) as i32)
                                == n_match_offset
                        {
                            n_already_exists = true;
                            break;
                        }
                    }
                    if !n_already_exists {
                        c.match_[mbase + m].length = if n_position < (end_offset - 2)
                            && win[n_match_pos as usize + 2] == win[n_position + 2]
                        {
                            3
                        } else {
                            2
                        };
                        c.match_[mbase + m].offset = n_match_offset as u32;
                        c.match_depth[mbase + m] = 0x4000;
                        m += 1;
                        n_inserted += 1;
                        if n_inserted >= 6 {
                            break;
                        }
                    }
                } else {
                    break;
                }
                n_match_pos = c.next_offset_for_pos[n_match_pos as usize - n_previous_block_size];
            }
        }
    }

    optimize_forward(
        c,
        g,
        win,
        n_previous_block_size,
        end_offset,
        true,
        cur_rep_match_offset,
        block_flags,
        napp,
    );

    // The supplemental match injection fires at the max-arrival tier in the reference. We gate on
    // `>=` so the wider-beam variants (napp > 62) keep this richer candidate set instead of falling
    // back to the leaner one; at napp == 62 this is byte-identical to the reference.
    if (block_flags & 3) == 3 && napp >= NARRIVALS_PER_POSITION_MAX {
        for v in c.offset_cache.iter_mut() {
            *v = -1;
        }

        for n_position in (n_previous_block_size + 1)..(end_offset.saturating_sub(1)) {
            let mbase = (n_position - n_previous_block_size) * NMATCHES_PER_INDEX;
            if (c.match_[mbase].length as i32) < 8 {
                let mut m = 0usize;
                let mut n_inserted = 0;
                let mut n_max_forward_pos = n_position + 2 + 1 + 5;
                if n_max_forward_pos > (end_offset - 2) {
                    n_max_forward_pos = end_offset - 2;
                }

                while m < 46 && c.match_[mbase + m].length != 0 {
                    c.offset_cache[(c.match_[mbase + m].offset & 2047) as usize] =
                        n_position as i32;
                    c.offset_cache[((c.match_[mbase + m].offset as i32
                        - (c.match_depth[mbase + m] & 0x3fff) as i32)
                        & 2047) as usize] = n_position as i32;
                    m += 1;
                }

                let mut n_match_pos = c.next_offset_for_pos[n_position - n_previous_block_size];
                while m < 46 && n_match_pos >= 0 {
                    let n_match_offset = n_position as i32 - n_match_pos;
                    if n_match_offset <= c.max_offset {
                        let mut n_already_exists = false;

                        if c.offset_cache[(n_match_offset & 2047) as usize] == n_position as i32 {
                            for e in 0..m {
                                if c.match_[mbase + e].offset as i32 == n_match_offset
                                    || (c.match_[mbase + e].offset as i32
                                        - (c.match_depth[mbase + e] & 0x3fff) as i32)
                                        == n_match_offset
                                {
                                    n_already_exists = true;
                                    if c.match_depth[mbase + e] == 0x4000 {
                                        let mut n_match_len = 2i32;
                                        while (n_match_len + 8) < 16
                                            && (n_position + n_match_len as usize + 8) < end_offset
                                            && win[n_match_pos as usize + n_match_len as usize
                                                ..n_match_pos as usize + n_match_len as usize + 8]
                                                == win[n_position + n_match_len as usize
                                                    ..n_position + n_match_len as usize + 8]
                                        {
                                            n_match_len += 8;
                                        }
                                        while (n_match_len + 4) < 16
                                            && (n_position + n_match_len as usize + 4) < end_offset
                                            && win[n_match_pos as usize + n_match_len as usize
                                                ..n_match_pos as usize + n_match_len as usize + 4]
                                                == win[n_position + n_match_len as usize
                                                    ..n_position + n_match_len as usize + 4]
                                        {
                                            n_match_len += 4;
                                        }
                                        while n_match_len < 16
                                            && (n_position + n_match_len as usize) < end_offset
                                            && win[n_match_pos as usize + n_match_len as usize]
                                                == win[n_position + n_match_len as usize]
                                        {
                                            n_match_len += 1;
                                        }
                                        if n_match_len > c.match_[mbase + e].length as i32 {
                                            c.match_[mbase + e].length = n_match_len as u32;
                                        }
                                    }
                                    break;
                                }
                            }
                        }

                        if !n_already_exists {
                            let mut n_forward_pos = n_position + 2 + 1;
                            if n_forward_pos >= n_match_offset as usize {
                                let mut n_got_match = false;
                                while n_forward_pos < n_max_forward_pos {
                                    let mo = n_match_offset as usize;
                                    if win[n_forward_pos..n_forward_pos + 2]
                                        == win[n_forward_pos - mo..n_forward_pos - mo + 2]
                                    {
                                        n_got_match = true;
                                        break;
                                    }
                                    n_forward_pos += 1;
                                }

                                if n_got_match {
                                    let mut n_match_len = 2i32;
                                    while (n_match_len + 8) < 16
                                        && (n_position + n_match_len as usize + 8) < end_offset
                                        && win[n_match_pos as usize + n_match_len as usize
                                            ..n_match_pos as usize + n_match_len as usize + 8]
                                            == win[n_position + n_match_len as usize
                                                ..n_position + n_match_len as usize + 8]
                                    {
                                        n_match_len += 8;
                                    }
                                    while (n_match_len + 4) < 16
                                        && (n_position + n_match_len as usize + 4) < end_offset
                                        && win[n_match_pos as usize + n_match_len as usize
                                            ..n_match_pos as usize + n_match_len as usize + 4]
                                            == win[n_position + n_match_len as usize
                                                ..n_position + n_match_len as usize + 4]
                                    {
                                        n_match_len += 4;
                                    }
                                    while n_match_len < 16
                                        && (n_position + n_match_len as usize) < end_offset
                                        && win[n_match_pos as usize + n_match_len as usize]
                                            == win[n_position + n_match_len as usize]
                                    {
                                        n_match_len += 1;
                                    }
                                    c.match_[mbase + m].length = n_match_len as u32;
                                    c.match_[mbase + m].offset = n_match_offset as u32;
                                    c.match_depth[mbase + m] = 0;
                                    m += 1;

                                    insert_forward_match(
                                        c,
                                        win,
                                        n_position,
                                        n_match_offset,
                                        n_previous_block_size,
                                        end_offset,
                                        napp,
                                        8,
                                    );

                                    n_inserted += 1;
                                    if n_inserted >= 18 || (n_inserted >= 15 && m >= 38) {
                                        break;
                                    }
                                }
                            }
                        }
                    } else {
                        break;
                    }
                    n_match_pos =
                        c.next_offset_for_pos[n_match_pos as usize - n_previous_block_size];
                }
            }
        }
    }

    optimize_forward(
        c,
        g,
        win,
        n_previous_block_size,
        end_offset,
        false,
        cur_rep_match_offset,
        block_flags,
        napp,
    );

    let mut passes = 0;
    loop {
        let did = reduce_commands(
            c,
            g,
            win,
            n_previous_block_size,
            end_offset,
            *cur_rep_match_offset,
            block_flags,
        );
        passes += 1;
        if !(did && passes < 20) {
            break;
        }
    }

    write_block(
        c,
        w,
        win,
        n_previous_block_size,
        end_offset,
        cur_follows_literal,
        cur_rep_match_offset,
        block_flags,
    );
}

// ---------------------------------------------------------------------------------------------
// apultra_compress (top-level orchestration)
// ---------------------------------------------------------------------------------------------

// Wider-beam arrival counts to try in addition to the reference (anchor) tier (62 for <= 256 KB,
// 46 above). Taken best-of against the anchor. Empirically the DP's win as a
// function of beam width is NOT monotonic: most files peak at 80..96 arrivals, and wider beams
// (>= 112) frequently *regress* (the cost/score tie-break can evict a good arrival). The best-of
// anchor floor discards every such regression, so widening is always safe; we keep the set small
// and centered on the productive 80..128 band to capture the wins cheaply. (Measured: geo peaks at
// 96, obj2 at 80, news saturates by 96; >=112 hurts jumpman/bib but best-of drops those.)
const WIDE_ARRIVALS: &[usize] = &[80, 96, 128];

/// Per-position "jump" cap for the thorough beam. The reference only tries match cuts at lengths
/// 2..=90 plus the full length; the thorough beam raises this to 320, considering more intermediate
/// cuts. Kept finite so periodic data (matches spanning the whole input) stays tractable.
const THOROUGH_JUMP: i32 = 320;

/// Forward compress, byte-identical to the reference `apultra`. This is the no-regression anchor.
fn apultra_compress(input: &[u8]) -> Vec<u8> {
    // Anchor: exact reference arrival tiers (byte-identical to native apultra).
    let anchor = apultra_compress_with_arrivals(input, None);
    if input.is_empty() {
        return anchor;
    }

    // Wider-beam candidates, best-of against the anchor. Each wider beam is a strict superset of the
    // anchor's search (same match set, more arrivals + the richer supplemental candidate gate), but
    // the multi-arrival DP's tie-breaking is not monotonic in the beam width, so we keep the anchor
    // as a floor and never emit anything larger than the reference.
    let mut best = anchor;
    let candidates: Vec<Vec<u8>> = std::thread::scope(|s| {
        let handles: Vec<_> = WIDE_ARRIVALS
            .iter()
            .map(|&a| s.spawn(move || apultra_compress_with_arrivals(input, Some(a))))
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("apultra wide-beam thread panicked"))
            .collect()
    });
    for cand in candidates {
        if cand.len() < best.len() {
            best = cand;
        }
    }
    best
}

/// Core compressor. `arrivals_override`:
///   `None`    => exact reference arrival tiers (the byte-identical anchor).
///   `Some(n)` => force `n` arrivals per position (a wider DP beam), clamped to the reference floor.
fn apultra_compress_with_arrivals(input: &[u8], arrivals_override: Option<usize>) -> Vec<u8> {
    let n_input_size = input.len();
    if n_input_size == 0 {
        return Vec::new();
    }

    let g = build_gamma2_size();

    let mut n_max_arrivals = NARRIVALS_PER_POSITION_SMALL;
    let n_block_size = if n_input_size < BLOCK_SIZE {
        if n_input_size < 1024 {
            1024
        } else {
            n_input_size
        }
    } else {
        BLOCK_SIZE
    };

    // Replicate the nMaxArrivals selection (nDictionarySize == 0 here).
    {
        let mut n_in_data_size = n_input_size;
        if n_in_data_size > n_block_size {
            n_in_data_size = n_block_size;
        }
        if n_in_data_size > 0 && n_in_data_size >= n_input_size {
            if n_input_size <= 262144 {
                n_max_arrivals = NARRIVALS_PER_POSITION_MAX;
            } else {
                n_max_arrivals = NARRIVALS_PER_POSITION_NORMAL;
            }
        }
    }

    // A wider-beam variant forces a higher arrival count (never below the reference tier, so the
    // supplemental-candidate gate `napp >= NARRIVALS_PER_POSITION_MAX` still fires).
    if let Some(a) = arrivals_override {
        if a > n_max_arrivals {
            n_max_arrivals = a;
        }
    }

    let mut c = Compressor::new(n_block_size, n_block_size * 2, n_max_arrivals);
    c.max_offset = MAX_OFFSET;
    // Wider-beam candidates also run the thorough parse (a higher per-position length-jump cap).
    c.thorough = arrivals_override.is_some();

    let mut w = BitWriter::new();

    let mut n_original_size = 0usize;
    let mut n_previous_block_size = 0usize;
    let mut n_cur_follows_literal = 0i32;
    let mut n_block_flags = 1i32;
    let mut n_cur_rep_match_offset = 0i32;

    while n_original_size < n_input_size {
        let mut n_in_data_size = n_input_size - n_original_size;
        if n_in_data_size > n_block_size {
            n_in_data_size = n_block_size;
        }
        if n_in_data_size == 0 {
            break;
        }

        if (n_original_size + n_in_data_size) >= n_input_size {
            n_block_flags |= 2;
        }

        // window starts at (n_original_size - n_previous_block_size)
        let win_start = n_original_size - n_previous_block_size;
        let win = &input[win_start..win_start + n_previous_block_size + n_in_data_size];

        // build SA + find matches for this block
        let n = n_previous_block_size + n_in_data_size;
        if build_suffix_array(&mut c, win, n) == 0 {
            if n_previous_block_size != 0 {
                skip_matches(&mut c, 0, n_previous_block_size);
            }
            find_all_matches(
                &mut c,
                NMATCHES_PER_INDEX,
                n_previous_block_size,
                n_previous_block_size + n_in_data_size,
                n_block_flags,
            );

            optimize_and_write_block(
                &mut c,
                &g,
                win,
                n_previous_block_size,
                n_in_data_size,
                &mut w,
                &mut n_cur_follows_literal,
                &mut n_cur_rep_match_offset,
                n_block_flags,
            );
        }

        n_block_flags &= !1;
        n_original_size += n_in_data_size;
        n_previous_block_size = n_in_data_size;

        // For multi-block streams the C carries the partial bit byte across blocks via
        // nCurBitsOffset adjustment; because our BitWriter is a single continuous stream this
        // is handled automatically (no per-block output buffer). Single block for files <=1MB.
    }

    w.out
}

// ---------------------------------------------------------------------------------------------
// decompressor (forward)
// ---------------------------------------------------------------------------------------------

struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    cur_bit_mask: i32,
    bits: u8,
}

impl<'a> BitReader<'a> {
    fn new(data: &'a [u8], pos: usize) -> Self {
        BitReader {
            data,
            pos,
            cur_bit_mask: 0,
            bits: 0,
        }
    }

    #[inline]
    fn read_bit(&mut self) -> i32 {
        if self.cur_bit_mask == 0 {
            if self.pos >= self.data.len() {
                return -1;
            }
            self.bits = self.data[self.pos];
            self.pos += 1;
            self.cur_bit_mask = 128;
        }
        let nbit = if self.bits & 128 != 0 { 1 } else { 0 };
        self.bits <<= 1;
        self.cur_bit_mask >>= 1;
        nbit
    }

    #[inline]
    fn read_gamma2(&mut self) -> i32 {
        let mut v: u32 = 1;
        loop {
            v = (v << 1) + self.read_bit() as u32;
            let bit = self.read_bit();
            if bit < 0 {
                return bit;
            }
            if bit == 0 {
                break;
            }
        }
        v as i32
    }

    #[inline]
    fn read_byte(&mut self) -> u8 {
        let b = self.data[self.pos];
        self.pos += 1;
        b
    }
}

fn apultra_decompress(input: &[u8]) -> Vec<u8> {
    apultra_decompress_with_gap(input).0
}

/// Decompress and also return the in-place safety gap (bytes) the stream needs:
/// `max(output_produced - input_consumed)` over the decode, minus its final
/// value. Any in-place layout (forward top-aligned or backward) must keep the
/// write head at least this many bytes clear of the read head, or it will
/// clobber unread compressed bytes - apultra literals cost 9 bits, so an
/// incompressible run decoded LATE makes the running compression peak above its
/// final value and the fixed margin is no longer enough. See
/// [`max_gap_forward`] / [`max_gap_backward`].
fn apultra_decompress_with_gap(input: &[u8]) -> (Vec<u8>, i32) {
    if input.is_empty() {
        return (Vec::new(), 0);
    }

    let mut out: Vec<u8> = Vec::new();
    let mut r = BitReader::new(input, 0);
    let mut n_match_offset: i32 = -1;
    let mut n_follows_literal = 3i32;
    // Peak of (produced - consumed) at any token boundary. The gap grows during
    // a match (output advances, input does not) and peaks at the match's end,
    // which is the state observed at the next loop iteration's top.
    let mut max_gap = 0i32;

    // first literal
    out.push(r.read_byte());

    loop {
        let gap = out.len() as i32 - r.pos as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let n_result = r.read_bit();
        if n_result < 0 {
            break;
        }

        if n_result == 0 {
            // literal
            out.push(r.read_byte());
            n_follows_literal = 3;
        } else {
            let n_result = r.read_bit();
            if n_result == 0 {
                // '10': 8+n bits offset
                let n_match_len;
                let mut n_match_offset_hi = r.read_gamma2();
                n_match_offset_hi -= n_follows_literal;
                if n_match_offset_hi >= 0 {
                    n_match_offset = (n_match_offset_hi as u32 as i32) << 8;
                    n_match_offset |= r.read_byte() as i32;
                    let mut ml = r.read_gamma2();
                    if n_match_offset < 128 || n_match_offset >= MINMATCH4_OFFSET {
                        ml += 2;
                    } else if n_match_offset >= MINMATCH3_OFFSET {
                        ml += 1;
                    }
                    n_match_len = ml;
                } else {
                    n_match_len = r.read_gamma2();
                }

                n_follows_literal = 2;
                let src = out.len() as i32 - n_match_offset;
                let mut s = src as usize;
                for _ in 0..n_match_len {
                    let b = out[s];
                    out.push(b);
                    s += 1;
                }
            } else {
                let n_result = r.read_bit();
                if n_result == 0 {
                    // '110': 7 bits offset + 1 bit length
                    let n_command = r.read_byte() as i32;
                    if n_command == 0x00 {
                        break; // EOD
                    }
                    n_match_offset = n_command >> 1;
                    let n_match_len = (n_command & 1) + 2;
                    n_follows_literal = 2;
                    let mut s = out.len() - n_match_offset as usize;
                    for _ in 0..n_match_len {
                        let b = out[s];
                        out.push(b);
                        s += 1;
                    }
                } else {
                    // '111': 4 bit offset
                    let mut n_short: i32 = 0;
                    n_short |= r.read_bit() << 3;
                    n_short |= r.read_bit() << 2;
                    n_short |= r.read_bit() << 1;
                    n_short |= r.read_bit();
                    n_follows_literal = 3;
                    if n_short != 0 {
                        let s = out.len() - n_short as usize;
                        let b = out[s];
                        out.push(b);
                    } else {
                        out.push(0);
                    }
                }
            }
        }
    }

    // The read head consumes the whole `input.len()`-byte block; use it (not
    // `r.pos`, which stops at EOD) so the final gap is the true end state.
    let final_gap = out.len() as i32 - input.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD apultra stream: the top-aligned
/// packed block must start at least this many bytes above the output end, or the
/// decoder's write head overtakes unread compressed data. See
/// [`apultra_decompress_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    apultra_decompress_with_gap(stream).1.max(0) as usize
}

/// In-place safety margin (bytes) for a BACKWARD (`apultra -b`) stream: the
/// packed block must sit at least this many bytes below the span start. The
/// 6502 backward decoder reads the stored stream from its END, which is exactly
/// a forward decode of the reversed stream - so the gap sequence matches.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    apultra_decompress_with_gap(&rev).1.max(0) as usize
}

// ---------------------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(data: &[u8]) {
        let c = compress_apultra(data);
        let d = apultra_decompress(&c);
        assert_eq!(d, data, "forward roundtrip len {}", data.len());

        let cb = compress_apultra_backward(data);
        let db = decompress(&cb, true);
        assert_eq!(db, data, "backward roundtrip len {}", data.len());
    }

    #[test]
    fn roundtrip_small() {
        rt(&[]);
        rt(&[0]);
        rt(&[42]);
        rt(&[1, 2, 3, 4, 5]);
        rt(b"hello hello hello world world");
        rt(&[0u8; 1000]);
        rt(&[7u8; 70000]);
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        // Incompressible data (9-bit literals) expands, so an in-place layout
        // needs a margin far larger than the fixed 32-byte default.
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        assert!(
            max_gap_forward(&compress_apultra(&noise)) > 32,
            "incompressible forward gap must exceed the fixed 32-byte margin"
        );
        assert!(
            max_gap_backward(&compress_apultra_backward(&noise)) > 32,
            "incompressible backward gap must exceed the fixed 32-byte margin"
        );
        // Highly compressible data barely expands: the default margin is fine.
        let zeros = vec![0u8; 8192];
        assert!(
            max_gap_backward(&compress_apultra_backward(&zeros)) <= 32,
            "compressible data should fit within the default margin"
        );
    }

    #[test]
    fn roundtrip_patterned() {
        let mut v = Vec::new();
        let base = b"abracadabra abracadabra 0123456789 ";
        for k in 0..500 {
            v.extend_from_slice(base);
            v.push((k & 0xff) as u8);
        }
        rt(&v);
    }

    #[test]
    fn roundtrip_pseudo_random() {
        let mut v = Vec::with_capacity(40000);
        let mut s: u32 = 0x1234_5678;
        for _ in 0..40000 {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            v.push((s >> 24) as u8);
        }
        rt(&v);
    }
}
