//! PuCrunch: clean-room reimplementation of the pucrunch 1.14 format. No original pucrunch
//! source code was consulted; the original tool is used only as a black-box interop oracle in
//! out-of-tree verification.
//!
//! ## Format recap
//!
//! Four token classes behind a variable-width escape prefix, all integers in a
//! bounded Elias gamma code (`maxGamma` = 5..7, we always emit 7), bits MSB
//! first:
//!
//! * ordinary literal: 8 bits total (escape-width selector != current escape)
//! * escape change:    escape + gamma(1) + `10` + new escape + low literal bits
//! * LZ77:  escape + gamma(len-1) + gamma(highpos+1) + extra bits + ~low byte
//!   (length-2 form: escape + gamma(1) + `0` + ~low byte, dist <= 256)
//! * RLE:   escape + gamma(1) + `11` + gamma-coded length + ranked byte
//! * EOF:   escape + gamma(2) + gamma(sentinel)
//!
//! ## What this implementation adds over the original compressor
//!
//! The original uses a heuristic backward parse (length refinement only at
//! power-of-two boundaries), selects `escBits` from an overhead estimate, and
//! never re-runs the parse after re-ranking the RLE table. Here:
//!
//! * the parse is an exhaustive backward DP over *every* candidate length of
//!   every Pareto-optimal match ([`crate::matchfinder`]), every RLE length up
//!   to a dense bound plus all gamma-cost boundaries, and the literal;
//! * escape selection (`escBits` 0..=8) and `extraLZPosBits` (0..=4) are
//!   chosen by running the full parse per candidate and comparing exact
//!   emitted sizes (including the exact escape-change overhead from the
//!   optimal escape-state DP) - not estimates;
//! * the RLE rank table is re-ranked from the *selected* RLE tokens and the
//!   parse is re-run until stable (the original stops after one ranking).
//!
//! Output is therefore never larger - and normally smaller - than the
//! original's, while remaining bit-exact format compatible.
//!
//! ## Public API
//!
//! * [`compress_pucrunch_prg`] / [`decompress_pucrunch`] - the standalone
//!   `p`,`u`-header file format (`-c0` style), PRG in/out.
//! * [`compress_pucrunch_6502`] / [`compress_pucrunch_6502_backward`] - the
//!   compact parameter-block container decoded by
//!   `decrunchers/pucrunch-lzan*.s`, plus Rust reference decoders.

use crate::matchfinder::{find_matches, find_matches_exact, MatchSet};

/// We always emit maxGamma = 7 (the pucrunch default; largest length/run
/// range). The decoder side accepts 5..=7 per the spec.
const MAX_GAMMA: u32 = 7;
/// Largest gamma-codable value: `(2 << maxGamma) - 1`.
const VALUE_MAX: u32 = (2 << MAX_GAMMA) - 1; // 255
/// Reserved high-position gamma value (EOF marker).
const SENTINEL: u32 = VALUE_MAX; // 255
/// Longest LZ match: `2 << maxGamma`.
const MAX_LZ_LEN: usize = (2 << MAX_GAMMA) as usize; // 256
/// Short-RLE limit: `1 << maxGamma` (run lengths 2..=128 use the short form).
const SHORT_RLE_LIMIT: usize = (1 << MAX_GAMMA) as usize; // 128
/// Longest RLE token we emit. The study lists 32256 for maxGamma = 7 (its
/// derivation formula would allow more); we stay within the listed bound -
/// longer runs are split, which costs nothing measurable.
const MAX_RLE_LEN: usize = 32256;

// ---------------------------------------------------------------------------
// Bit I/O (MSB first)
// ---------------------------------------------------------------------------

struct BitWriter {
    bytes: Vec<u8>,
    cur: u8,
    nbits: u8,
}

impl BitWriter {
    fn new() -> Self {
        BitWriter {
            bytes: Vec::new(),
            cur: 0,
            nbits: 0,
        }
    }
    #[inline]
    fn bit(&mut self, b: u32) {
        self.cur = (self.cur << 1) | (b as u8 & 1);
        self.nbits += 1;
        if self.nbits == 8 {
            self.bytes.push(self.cur);
            self.cur = 0;
            self.nbits = 0;
        }
    }
    /// Write the low `count` bits of `value`, MSB first.
    fn bits(&mut self, value: u32, count: u32) {
        for shift in (0..count).rev() {
            self.bit((value >> shift) & 1);
        }
    }
    /// Bounded gamma code for 1..=VALUE_MAX (study §4.2).
    fn gamma(&mut self, v: u32) {
        debug_assert!((1..=VALUE_MAX).contains(&v));
        let n = 31 - v.leading_zeros();
        for _ in 0..n {
            self.bit(1);
        }
        if n < MAX_GAMMA {
            self.bit(0);
        }
        self.bits(v & !(1 << n), n);
    }
    /// Current position in bits (for the emission-offset bookkeeping).
    fn bit_len(&self) -> usize {
        self.bytes.len() * 8 + self.nbits as usize
    }
    fn finish(mut self) -> Vec<u8> {
        if self.nbits > 0 {
            self.bytes.push(self.cur << (8 - self.nbits));
        }
        self.bytes
    }
}

struct BitReader<'a> {
    bytes: &'a [u8],
    pos: usize, // bit position
}

impl<'a> BitReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        BitReader { bytes, pos: 0 }
    }
    #[inline]
    fn bit(&mut self) -> Result<u32, String> {
        let byte = self.pos / 8;
        if byte >= self.bytes.len() {
            return Err("bitstream overrun".into());
        }
        let b = (self.bytes[byte] >> (7 - (self.pos & 7))) & 1;
        self.pos += 1;
        Ok(b as u32)
    }
    fn bits(&mut self, count: u32) -> Result<u32, String> {
        let mut v = 0u32;
        for _ in 0..count {
            v = (v << 1) | self.bit()?;
        }
        Ok(v)
    }
    /// Bounded gamma decode (study §4.2), for a given maxGamma.
    fn gamma(&mut self, max_gamma: u32) -> Result<u32, String> {
        let mut n = 0u32;
        while n < max_gamma && self.bit()? == 1 {
            n += 1;
        }
        Ok((1 << n) | self.bits(n)?)
    }
}

/// Encoded bit length of gamma(v) (study §4.2).
#[inline]
fn gamma_cost(v: u32) -> u32 {
    debug_assert!((1..=VALUE_MAX).contains(&v));
    let n = 31 - v.leading_zeros();
    if n < MAX_GAMMA {
        2 * n + 1
    } else {
        2 * n
    }
}

// ---------------------------------------------------------------------------
// Token model and costs (study §12)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
enum Tok {
    Lit,
    /// Normal LZ (len 3..=256) or the length-2 short form (dist <= 256).
    Lz {
        len: u32,
        dist: u32,
    },
    Rle {
        len: u32,
    },
}

/// Per-byte RLE code cost given a rank table: gamma(rank) if ranked, else
/// gamma(16 + hi-nibble) + 4 = 9 + 4 (all codes 16..=31 cost the same for
/// maxGamma >= 5).
#[inline]
fn rle_byte_cost(rank_of: &[u8; 256], byte: u8) -> u32 {
    let r = rank_of[byte as usize];
    if r != 0 {
        gamma_cost(r as u32)
    } else {
        9 + 4
    }
}

#[inline]
fn lz_cost(esc_bits: u32, extra: u32, len: u32, dist: u32) -> u32 {
    if len == 2 {
        esc_bits + 10
    } else {
        esc_bits + 8 + extra + gamma_cost(len - 1) + gamma_cost(((dist - 1) >> (8 + extra)) + 1)
    }
}

#[inline]
fn rle_cost(esc_bits: u32, len: u32, byte_cost: u32) -> u32 {
    let n = len - 1;
    if (n as usize) < SHORT_RLE_LIMIT {
        esc_bits + 3 + gamma_cost(n) + byte_cost
    } else {
        esc_bits + 3 + MAX_GAMMA + 8 + gamma_cost((n >> 8) + 1) + byte_cost
    }
}

/// Largest normal-LZ distance representable with `extra` middle bits:
/// high group <= SENTINEL-2 (the maximum gamma is reserved), so
/// d-1 <= (253 << (8+extra)) | ((2^extra - 1) << 8) | 255.
#[inline]
fn max_lz_dist(extra: u32) -> usize {
    let d = ((SENTINEL - 2) << (8 + extra)) | (((1u32 << extra) - 1) << 8) | 0xFF;
    d as usize + 1
}

// ---------------------------------------------------------------------------
// Parse: exhaustive backward DP
// ---------------------------------------------------------------------------

/// Remaining equal-byte run length at each position (`rle[i]` = how many bytes
/// starting at `i` equal `data[i]`).
fn run_lengths(data: &[u8]) -> Vec<u32> {
    let n = data.len();
    let mut rle = vec![1u32; n];
    let mut i = n;
    while i >= 2 {
        i -= 1;
        if data[i] == data[i - 1] {
            rle[i - 1] = rle[i].saturating_add(1);
        }
    }
    rle
}

struct Parse {
    tokens: Vec<Tok>, // in input order
    /// DP stream bits (literals at 8, control tokens with escBits; no escape
    /// overhead, no EOF).
    stream_bits: u64,
}

/// Enumerate every non-literal edge at position `i`: all LZ candidate lengths
/// on the Pareto front (each at its minimum offset), and all RLE lengths up to
/// a dense bound plus every gamma/short-long cost boundary. RLE edges are
/// yielded last so a `<=` acceptance gives them the original's tie preference.
#[allow(clippy::too_many_arguments)] // shared hot-path enumerator; a context struct would obscure it
fn for_each_edge(
    data: &[u8],
    ms: &MatchSet,
    rle: &[u32],
    esc_bits: u32,
    extra: u32,
    rank_of: &[u8; 256],
    i: usize,
    mut f: impl FnMut(Tok, u32),
) {
    let max_dist = max_lz_dist(extra);

    // LZ: walk the Pareto front, relaxing every length 2..=max.
    let cands = ms.matches_for(i);
    if !cands.is_empty() {
        let mut ci = 0usize;
        let max_l = cands.last().unwrap().length.min(MAX_LZ_LEN as u32);
        let mut l = 2u32;
        while l <= max_l {
            while cands[ci].length < l {
                ci += 1;
            }
            let dist = cands[ci].offset;
            let ok = if l == 2 {
                dist <= 256
            } else {
                (dist as usize) <= max_dist
            };
            if ok {
                f(Tok::Lz { len: l, dist }, lz_cost(esc_bits, extra, l, dist));
            }
            l += 1;
        }
    }

    // RLE: dense lengths up to a bound, then all gamma/short-long cost
    // boundaries, plus the (capped) full run.
    let run = rle[i] as usize;
    if run >= 2 {
        let byte_cost = rle_byte_cost(rank_of, data[i]);
        let cap = run.min(MAX_RLE_LEN);
        let mut try_len = |len: usize| {
            f(
                Tok::Rle { len: len as u32 },
                rle_cost(esc_bits, len as u32, byte_cost),
            );
        };
        let dense = cap.min(320);
        for len in 2..=dense {
            try_len(len);
        }
        if cap > 320 {
            // Above the dense range only the long form applies, and its cost
            // steps only where gamma((len-1 >> 8) + 1) crosses a power of two:
            // (len-1)>>8 + 1 = 2^a  <=>  len = 256*(2^a - 1) + 1. Probe both
            // sides of every step (the longest cheap length and the first of
            // the next tier), plus the capped full run.
            for a in 1..=7usize {
                let step = 256 * ((1 << a) - 1);
                for len in [step - 1, step, step + 1] {
                    if (321..=cap).contains(&len) {
                        try_len(len);
                    }
                }
            }
            try_len(cap);
        }
    }
}

/// One full backward DP over the input with fixed (escBits, extra, table).
/// Literals cost a flat 8 bits; escape-change overhead is optimized separately
/// afterwards (the original's model, study §14).
fn parse(
    data: &[u8],
    ms: &MatchSet,
    rle: &[u32],
    esc_bits: u32,
    extra: u32,
    rank_of: &[u8; 256],
) -> Parse {
    let n = data.len();
    // cost[i] = min stream bits for data[i..]; step[i] = chosen token.
    let mut cost = vec![u64::MAX; n + 1];
    let mut step = vec![Tok::Lit; n];
    cost[n] = 0;

    for i in (0..n).rev() {
        let mut best = 8 + cost[i + 1];
        let mut tok = Tok::Lit;
        for_each_edge(data, ms, rle, esc_bits, extra, rank_of, i, |t, bits| {
            let len = match t {
                Tok::Lz { len, .. } => len,
                Tok::Rle { len } => len,
                Tok::Lit => unreachable!(),
            } as usize;
            let c = bits as u64 + cost[i + len];
            // `<=` gives later edges (RLE) the tie win, matching the
            // original's RLE > LZ > literal preference.
            if c <= best {
                best = c;
                tok = t;
            }
        });
        cost[i] = best;
        step[i] = tok;
    }

    // Walk forward collecting tokens.
    let mut tokens = Vec::new();
    let mut i = 0usize;
    while i < n {
        let t = step[i];
        tokens.push(t);
        i += match t {
            Tok::Lit => 1,
            Tok::Lz { len, .. } => len as usize,
            Tok::Rle { len } => len as usize,
        };
    }
    Parse {
        tokens,
        stream_bits: cost[0],
    }
}

/// Result of the exact joint DP: tokens plus the escape plan, with the true
/// total stream bits (escape-change overhead included, EOF excluded).
struct JointParse {
    tokens: Vec<Tok>,
    newesc: Vec<u8>,
    start_esc: u8,
    bits: u64,
}

/// Exact joint DP over (position, current escape state) - study §13 + §14 in
/// one pass. The original optimizes the token parse with flat 8-bit literals
/// and fixes up escape changes afterwards; the joint DP sees the real cost of
/// a colliding literal (`11 + escBits`, with a free choice of successor
/// state), so it can prefer a match or run where the separate passes cannot.
/// State count is `2^escBits`, so this is used where `2^escBits * n` is small
/// (the separate passes remain for the rest).
fn parse_joint(
    data: &[u8],
    ms: &MatchSet,
    rle: &[u32],
    esc_bits: u32,
    extra: u32,
    rank_of: &[u8; 256],
) -> JointParse {
    let n = data.len();
    let states = 1usize << esc_bits;
    let idx = |i: usize, s: usize| i * states + s;

    // f[idx(i,s)] = min bits for data[i..] entering with escape state s.
    let mut f = vec![u64::MAX; (n + 1) * states];
    let mut step: Vec<Tok> = vec![Tok::Lit; n * states];
    let mut nxt = vec![0u8; n * states]; // successor state for colliding literals
    for s in 0..states {
        f[idx(n, s)] = 0;
    }

    let mut edges: Vec<(Tok, u32)> = Vec::with_capacity(512);
    for i in (0..n).rev() {
        edges.clear();
        for_each_edge(data, ms, rle, esc_bits, extra, rank_of, i, |t, bits| {
            edges.push((t, bits));
        });

        // min over successor states of f[i+1][*] (for colliding literals).
        let (mut esc_min, mut esc_arg) = (u64::MAX, 0usize);
        for s in 0..states {
            let c = f[idx(i + 1, s)];
            if c < esc_min {
                esc_min = c;
                esc_arg = s;
            }
        }
        let sel = if esc_bits == 0 {
            0
        } else {
            (data[i] >> (8 - esc_bits)) as usize
        };

        for s in 0..states {
            // Literal: ordinary when the selector differs, else the
            // escape-change form with a free successor-state choice.
            let mut best;
            if s != sel && esc_bits > 0 {
                best = 8 + f[idx(i + 1, s)];
            } else {
                best = (11 + esc_bits) as u64 + esc_min;
                nxt[idx(i, s)] = esc_arg as u8;
            }
            let mut tok = Tok::Lit;
            for &(t, bits) in &edges {
                let len = match t {
                    Tok::Lz { len, .. } => len,
                    Tok::Rle { len } => len,
                    Tok::Lit => unreachable!(),
                } as usize;
                let c = bits as u64 + f[idx(i + len, s)];
                if c <= best {
                    best = c;
                    tok = t;
                }
            }
            f[idx(i, s)] = best;
            step[idx(i, s)] = tok;
        }
    }

    // Cheapest initial state, then walk the path.
    let (mut bits, mut s) = (u64::MAX, 0usize);
    for cand in 0..states {
        if f[idx(0, cand)] < bits {
            bits = f[idx(0, cand)];
            s = cand;
        }
    }
    let start_esc = s as u8;
    let mut tokens = Vec::new();
    let mut newesc = Vec::new();
    let mut i = 0usize;
    while i < n {
        let t = step[idx(i, s)];
        tokens.push(t);
        match t {
            Tok::Lit => {
                let sel = if esc_bits == 0 {
                    0
                } else {
                    (data[i] >> (8 - esc_bits)) as usize
                };
                if s == sel || esc_bits == 0 {
                    let ns = nxt[idx(i, s)];
                    newesc.push(ns);
                    s = ns as usize;
                } else {
                    newesc.push(0); // unused: ordinary literal
                }
                i += 1;
            }
            Tok::Lz { len, .. } => i += len as usize,
            Tok::Rle { len } => i += len as usize,
        }
    }
    JointParse {
        tokens,
        newesc,
        start_esc,
        bits,
    }
}

// ---------------------------------------------------------------------------
// Escape-state optimization (study §14) - exact DP
// ---------------------------------------------------------------------------

struct EscPlan {
    start_esc: u8,
    /// For each literal (in input order): the new escape to switch to when the
    /// running state collides with the literal's selector.
    newesc: Vec<u8>,
    /// Number of escape-change events with the optimal start state.
    changes: u32,
}

/// Positions and selector of every literal in the token list.
fn literal_selectors(data: &[u8], tokens: &[Tok], esc_bits: u32) -> Vec<u8> {
    let mut sel = Vec::new();
    let mut i = 0usize;
    for t in tokens {
        match *t {
            Tok::Lit => {
                sel.push(if esc_bits == 0 {
                    0
                } else {
                    data[i] >> (8 - esc_bits)
                });
                i += 1;
            }
            Tok::Lz { len, .. } => i += len as usize,
            Tok::Rle { len } => i += len as usize,
        }
    }
    sel
}

fn optimize_escapes(selectors: &[u8], esc_bits: u32) -> EscPlan {
    let states = 1usize << esc_bits;
    if esc_bits == 0 {
        // Single state: every literal collides.
        return EscPlan {
            start_esc: 0,
            newesc: vec![0; selectors.len()],
            changes: selectors.len() as u32,
        };
    }
    // cost[s] = escape changes needed for the remaining literals when the
    // state before them is s. Scan backward (study §14 recurrence).
    let mut cost = vec![0u32; states];
    let mut newesc = vec![0u8; selectors.len()];
    for (j, &k) in selectors.iter().enumerate().rev() {
        let (mut mn, mut arg) = (u32::MAX, 0usize);
        for (s, &c) in cost.iter().enumerate() {
            if c < mn {
                mn = c;
                arg = s;
            }
        }
        newesc[j] = arg as u8;
        cost[k as usize] = 1 + mn;
    }
    let (mut mn, mut arg) = (u32::MAX, 0usize);
    for (s, &c) in cost.iter().enumerate() {
        if c < mn {
            mn = c;
            arg = s;
        }
    }
    EscPlan {
        start_esc: arg as u8,
        newesc,
        changes: mn,
    }
}

// ---------------------------------------------------------------------------
// RLE rank table
// ---------------------------------------------------------------------------

/// Rank by descending count, ties by ascending byte value; up to 15 entries
/// with nonzero count. Returns (table, rank_of) where rank_of[b] = 1..=15 or 0.
fn rank_table(counts: &[u32; 256]) -> (Vec<u8>, [u8; 256]) {
    let mut idx: Vec<usize> = (0..256).filter(|&b| counts[b] > 0).collect();
    idx.sort_by(|&a, &b| counts[b].cmp(&counts[a]).then(a.cmp(&b)));
    idx.truncate(15);
    let mut rank_of = [0u8; 256];
    let table: Vec<u8> = idx.iter().map(|&b| b as u8).collect();
    for (r, &b) in table.iter().enumerate() {
        rank_of[b as usize] = r as u8 + 1;
    }
    (table, rank_of)
}

/// Initial ranking: histogram of run starts (study §11.2).
fn initial_table(data: &[u8], rle: &[u32]) -> (Vec<u8>, [u8; 256]) {
    let mut counts = [0u32; 256];
    let mut i = 0usize;
    while i < data.len() {
        let r = rle[i] as usize;
        if r >= 2 {
            counts[data[i] as usize] += 1;
            i += r;
        } else {
            i += 1;
        }
    }
    rank_table(&counts)
}

/// Re-rank from the RLE tokens the parse actually selected (study §16).
fn retune_table(data: &[u8], tokens: &[Tok]) -> (Vec<u8>, [u8; 256]) {
    let mut counts = [0u32; 256];
    let mut i = 0usize;
    for t in tokens {
        match *t {
            Tok::Lit => i += 1,
            Tok::Lz { len, .. } => i += len as usize,
            Tok::Rle { len } => {
                counts[data[i] as usize] += 1;
                i += len as usize;
            }
        }
    }
    rank_table(&counts)
}

// ---------------------------------------------------------------------------
// Emission
// ---------------------------------------------------------------------------

/// Emitted stream plus the per-input-position compressed byte offsets needed
/// for the standalone placement-margin computation (study §17).
struct Emitted {
    bitstream: Vec<u8>,
    start_esc: u8,
    /// Compressed byte offset at the start of the token covering position p.
    off_at: Vec<u32>,
}

fn emit(
    data: &[u8],
    tokens: &[Tok],
    esc_bits: u32,
    extra: u32,
    rank_of: &[u8; 256],
    plan: &EscPlan,
) -> Emitted {
    let mut w = BitWriter::new();
    let mut off_at = vec![0u32; data.len()];
    let mut esc = plan.start_esc as u32;
    let mut i = 0usize;
    let mut lit_no = 0usize;

    let put_esc = |w: &mut BitWriter, esc: u32| w.bits(esc, esc_bits);

    for t in tokens {
        let tok_off = (w.bit_len() / 8) as u32;
        match *t {
            Tok::Lit => {
                let byte = data[i] as u32;
                let sel = if esc_bits == 0 {
                    0
                } else {
                    byte >> (8 - esc_bits)
                };
                if sel != esc && esc_bits > 0 {
                    // Ordinary literal: 8 bits.
                    w.bits(byte, 8);
                } else {
                    // Escape change (study §6): old escape prefixes the byte.
                    let new_esc = plan.newesc[lit_no] as u32;
                    put_esc(&mut w, esc);
                    w.gamma(1);
                    w.bit(1);
                    w.bit(0);
                    w.bits(new_esc, esc_bits);
                    w.bits(byte & ((1 << (8 - esc_bits)) - 1), 8 - esc_bits);
                    esc = new_esc;
                }
                off_at[i] = tok_off;
                lit_no += 1;
                i += 1;
            }
            Tok::Lz { len, dist } => {
                put_esc(&mut w, esc);
                let d = dist - 1;
                if len == 2 {
                    w.gamma(1);
                    w.bit(0);
                    w.bits((d & 0xFF) ^ 0xFF, 8);
                } else {
                    w.gamma(len - 1);
                    w.gamma((d >> (8 + extra)) + 1);
                    w.bits(d >> 8, extra);
                    w.bits((d & 0xFF) ^ 0xFF, 8);
                }
                off_at[i..i + len as usize].fill(tok_off);
                i += len as usize;
            }
            Tok::Rle { len } => {
                let byte = data[i];
                put_esc(&mut w, esc);
                w.gamma(1);
                w.bit(1);
                w.bit(1);
                let n = len - 1;
                if (n as usize) < SHORT_RLE_LIMIT {
                    w.gamma(n);
                } else {
                    w.gamma((1 << MAX_GAMMA) + ((n & 0xFF) >> (8 - MAX_GAMMA)));
                    w.bits(n & 0xFF, 8 - MAX_GAMMA);
                    w.gamma((n >> 8) + 1);
                }
                let r = rank_of[byte as usize];
                if r != 0 {
                    w.gamma(r as u32);
                } else {
                    w.gamma(16 + (byte as u32 >> 4));
                    w.bits(byte as u32, 4);
                }
                off_at[i..i + len as usize].fill(tok_off);
                i += len as usize;
            }
        }
    }
    // Canonical EOF (study §7.4).
    put_esc(&mut w, esc);
    w.gamma(2);
    w.gamma(SENTINEL);

    Emitted {
        bitstream: w.finish(),
        start_esc: plan.start_esc,
        off_at,
    }
}

// ---------------------------------------------------------------------------
// Core compressor: parameter search
// ---------------------------------------------------------------------------

struct Packed {
    bitstream: Vec<u8>,
    start_esc: u8,
    esc_bits: u32,
    extra: u32,
    table: Vec<u8>,
    off_at: Vec<u32>,
}

/// EOF token bits (study §7.4).
#[inline]
fn eof_bits(esc_bits: u32) -> u64 {
    (esc_bits + gamma_cost(2) + gamma_cost(SENTINEL)) as u64
}

/// One fully evaluated (escBits, extra) candidate, ready to emit.
struct Solution {
    tokens: Vec<Tok>,
    newesc: Vec<u8>,
    start_esc: u8,
    /// Exact stream bits including escape-change overhead and EOF.
    total: u64,
    esc_bits: u32,
    extra: u32,
}

/// Separate passes: flat-literal parse then exact escape DP (the original's
/// two-stage model, both stages optimal on their own).
fn solve_separate(
    data: &[u8],
    ms: &MatchSet,
    rle: &[u32],
    e: u32,
    x: u32,
    rank_of: &[u8; 256],
) -> Solution {
    let p = parse(data, ms, rle, e, x, rank_of);
    let sel = literal_selectors(data, &p.tokens, e);
    let plan = optimize_escapes(&sel, e);
    Solution {
        total: p.stream_bits + plan.changes as u64 * (3 + e) as u64 + eof_bits(e),
        tokens: p.tokens,
        newesc: plan.newesc,
        start_esc: plan.start_esc,
        esc_bits: e,
        extra: x,
    }
}

/// Joint (position, escape state) DP - exact; affordable when `2^e * n` is
/// modest.
fn joint_budget_ok(n: usize, e: u32) -> bool {
    (1u64 << e) * n as u64 <= 4_000_000
}

fn solve_joint(
    data: &[u8],
    ms: &MatchSet,
    rle: &[u32],
    e: u32,
    x: u32,
    rank_of: &[u8; 256],
) -> Solution {
    let jp = parse_joint(data, ms, rle, e, x, rank_of);
    Solution {
        total: jp.bits + eof_bits(e),
        tokens: jp.tokens,
        newesc: jp.newesc,
        start_esc: jp.start_esc,
        esc_bits: e,
        extra: x,
    }
}

/// `hdr_bits_per_entry`: what one RLE table entry costs in the enclosing
/// container - 8 for the standalone header (16 + rleUsed bytes), 0 for the
/// fixed-15-slot 6502 container (padding is free, so pruning never pays).
fn compress_core(data: &[u8], hdr_bits_per_entry: u64) -> Packed {
    if data.is_empty() {
        // Just the EOF token.
        let plan = EscPlan {
            start_esc: 0,
            newesc: Vec::new(),
            changes: 0,
        };
        let e = emit(data, &[], 2, 0, &[0u8; 256], &plan);
        return Packed {
            bitstream: e.bitstream,
            start_esc: e.start_esc,
            esc_bits: 2,
            extra: 0,
            table: Vec::new(),
            off_at: Vec::new(),
        };
    }

    let n = data.len();
    let rle = run_lengths(data);
    // Window: the largest distance any extra-bits setting can represent.
    let window = max_lz_dist(4).min(65535);
    let ms = if n <= 16 * 1024 {
        find_matches_exact(data, 2, window, MAX_LZ_LEN)
    } else {
        find_matches(data, 2, window, MAX_LZ_LEN, 8192)
    };

    // Phase-1 grid (cheap separate passes) over every legal width.
    let esc_grid: Vec<u32> = (0..=8).collect();
    let extra_grid: Vec<u32> = (0..=4).collect();

    // Comparisons across table changes must include the header cost of the
    // table itself (8 bits per entry in the standalone header).
    let with_hdr = |s: &Solution, table: &[u8]| s.total + hdr_bits_per_entry * table.len() as u64;

    let (mut table, mut rank_of) = initial_table(data, &rle);
    let mut best: Option<(Solution, Vec<u8>, [u8; 256])> = None;

    for _round in 0..4 {
        // Phase 1: separate-pass grid to locate the best (e, x).
        let mut phase1: Option<Solution> = None;
        for &e in &esc_grid {
            for &x in &extra_grid {
                let s = solve_separate(data, &ms, &rle, e, x, &rank_of);
                if phase1.as_ref().is_none_or(|b| s.total < b.total) {
                    phase1 = Some(s);
                }
            }
        }
        let p1 = phase1.unwrap();
        let (be, bx) = (p1.esc_bits, p1.extra);

        // Phase 2: exact joint DP on the winner and its (e, x) neighbors.
        let mut round_best = p1;
        let mut neighbors: Vec<(u32, u32)> = vec![(be, bx)];
        if be > 0 {
            neighbors.push((be - 1, bx));
        }
        if be < 8 {
            neighbors.push((be + 1, bx));
        }
        if bx > 0 {
            neighbors.push((be, bx - 1));
        }
        if bx < 4 {
            neighbors.push((be, bx + 1));
        }
        for (e, x) in neighbors {
            if !joint_budget_ok(n, e) {
                continue;
            }
            let s = solve_joint(data, &ms, &rle, e, x, &rank_of);
            if s.total < round_best.total {
                round_best = s;
            }
        }

        // Re-rank the table from the selected RLE tokens; stop when stable or
        // no longer improving.
        let (new_table, new_rank_of) = retune_table(data, &round_best.tokens);
        let improved = best
            .as_ref()
            .is_none_or(|b| with_hdr(&round_best, &table) < with_hdr(&b.0, &b.1));
        let stable = new_table == table;
        if improved {
            best = Some((round_best, table.clone(), rank_of));
        }
        if stable || !improved {
            break;
        }
        table = new_table;
        rank_of = new_rank_of;
    }

    // Table pruning: a tail rank whose few uses save less than its 8 header
    // bits is dead weight - drop entries while the total (stream + header)
    // keeps shrinking.
    loop {
        if hdr_bits_per_entry == 0 {
            break; // fixed-slot containers: pruning never pays
        }
        let Some((sol, tbl, _)) = &best else { break };
        if tbl.is_empty() {
            break;
        }
        let mut t2 = tbl.clone();
        t2.pop();
        let mut r2 = [0u8; 256];
        for (r, &b) in t2.iter().enumerate() {
            r2[b as usize] = r as u8 + 1;
        }
        let (e, x) = (sol.esc_bits, sol.extra);
        let s2 = if joint_budget_ok(n, e) {
            solve_joint(data, &ms, &rle, e, x, &r2)
        } else {
            solve_separate(data, &ms, &rle, e, x, &r2)
        };
        if with_hdr(&s2, &t2) < with_hdr(sol, tbl) {
            best = Some((s2, t2, r2));
        } else {
            break;
        }
    }

    let (sol, table, rank_of) = best.unwrap();
    let plan = EscPlan {
        start_esc: sol.start_esc,
        newesc: sol.newesc.clone(),
        changes: 0,
    };
    let emitted = emit(data, &sol.tokens, sol.esc_bits, sol.extra, &rank_of, &plan);
    Packed {
        bitstream: emitted.bitstream,
        start_esc: emitted.start_esc,
        esc_bits: sol.esc_bits,
        extra: sol.extra,
        table,
        off_at: emitted.off_at,
    }
}

// ---------------------------------------------------------------------------
// Core stream decoder (shared by all container flavors)
// ---------------------------------------------------------------------------

struct StreamParams {
    start_esc: u32,
    esc_bits: u32,
    max_gamma: u32,
    extra: u32,
    table: Vec<u8>,
}

/// Decode a token stream until the canonical EOF (study §10). Strict: rejects
/// delta-LZ sentinels, invalid rank references and out-of-range distances.
/// Also returns the in-place safety metric: the maximum over all output bytes
/// of `output_position - bitstream_byte_offset_at_the_token_start` (the same
/// quantity the standalone placement derives from `off_at`, study §17).
fn decode_stream_gap(r: &mut BitReader, prm: &StreamParams) -> Result<(Vec<u8>, i64), String> {
    let g = prm.max_gamma;
    let sentinel = (2u32 << g) - 1;
    let mut esc = prm.start_esc;
    let mut out: Vec<u8> = Vec::new();
    let mut max_gap = i64::MIN;

    loop {
        let tok_off = (r.pos / 8) as i64;
        let mut note = |from: usize, to: usize| {
            // gap for the bytes [from, to) produced by this token; the last
            // byte dominates, so one max() suffices.
            let _ = from;
            if to > 0 {
                max_gap = max_gap.max(to as i64 - 1 - tok_off);
            }
        };
        let sel = r.bits(prm.esc_bits)?;
        if prm.esc_bits > 0 && sel != esc {
            let rest = r.bits(8 - prm.esc_bits)?;
            out.push(((sel << (8 - prm.esc_bits)) | rest) as u8);
            note(out.len() - 1, out.len());
            continue;
        }
        let a = r.gamma(g)?;
        if a == 1 {
            if r.bit()? == 0 {
                // Short LZ, length 2.
                let dist = (r.bits(8)? ^ 0xFF) + 1;
                if dist as usize > out.len() {
                    return Err("LZ2 distance underrun".into());
                }
                for _ in 0..2 {
                    let b = out[out.len() - dist as usize];
                    out.push(b);
                }
                note(out.len() - 2, out.len());
                continue;
            }
            if r.bit()? == 0 {
                // Escape change + literal (old escape prefixes the byte).
                let old = esc;
                esc = r.bits(prm.esc_bits)?;
                let rest = r.bits(8 - prm.esc_bits)?;
                out.push(((old << (8 - prm.esc_bits)) | rest) as u8);
                note(out.len() - 1, out.len());
                continue;
            }
            // RLE.
            let first = r.gamma(g)?;
            let n = if first < (1 << g) {
                first
            } else {
                let low = ((first - (1 << g)) << (8 - g)) | r.bits(8 - g)?;
                let high = r.gamma(g)? - 1;
                (high << 8) | low
            };
            let code = r.gamma(g)?;
            let byte = if code < 16 {
                if code as usize > prm.table.len() {
                    return Err(format!(
                        "RLE rank {code} beyond table ({})",
                        prm.table.len()
                    ));
                }
                prm.table[code as usize - 1]
            } else {
                if code > 31 {
                    return Err(format!("invalid RLE byte code {code}"));
                }
                (((code - 16) << 4) | r.bits(4)?) as u8
            };
            let start = out.len();
            for _ in 0..=n {
                out.push(byte);
            }
            note(start, out.len());
            continue;
        }
        // Normal LZ, EOF, or (unsupported) delta LZ.
        let b = r.gamma(g)?;
        if b == sentinel {
            if a == 2 {
                break; // canonical EOF
            }
            return Err("delta-LZ token (unsupported by this decoder)".into());
        }
        let high = b - 1;
        let mid = r.bits(prm.extra)?;
        let low = r.bits(8)? ^ 0xFF;
        let dist = ((high << (8 + prm.extra)) | (mid << 8) | low) + 1;
        let len = a + 1;
        if dist as usize > out.len() {
            return Err("LZ distance underrun".into());
        }
        let start = out.len();
        for _ in 0..len {
            let b = out[out.len() - dist as usize];
            out.push(b);
        }
        note(start, out.len());
    }
    Ok((out, max_gap))
}

/// Compatibility wrapper: decode without the gap metric.
fn decode_stream(r: &mut BitReader, prm: &StreamParams) -> Result<Vec<u8>, String> {
    Ok(decode_stream_gap(r, prm)?.0)
}

// ---------------------------------------------------------------------------
// Standalone `p`,`u` file format (study §3)
// ---------------------------------------------------------------------------

/// Detect a BASIC `SYS<addr>` near the beginning of a program loading at
/// `load` (study §11.1); the execution address for the standalone header.
/// Walks the tokenized line links for the first few lines and tries every
/// `$9E` (SYS) token in each body - a leading REM line or a `$9E` byte inside
/// a string must not hide the real SYS.
fn detect_sys(load: u16, payload: &[u8]) -> Option<u16> {
    let mut line_at = 0usize; // offset of the current line within the payload
    for _ in 0..8 {
        if line_at + 5 > payload.len() {
            return None;
        }
        let next_ptr = u16::from_le_bytes([payload[line_at], payload[line_at + 1]]);
        if next_ptr == 0 {
            return None; // end of program
        }
        // Body: after the 2-byte link and 2-byte line number, up to the $00.
        let body_start = line_at + 4;
        let body_end = body_start + payload[body_start..].iter().position(|&b| b == 0)?;
        let body = &payload[body_start..body_end];
        // Try every SYS token in the line, not only the first $9E byte.
        for (sys, _) in body.iter().enumerate().filter(|(_, &b)| b == 0x9E) {
            let mut i = sys + 1;
            while i < body.len() && (body[i] == b' ' || body[i] == 0xA0 || body[i] == b'(') {
                i += 1;
            }
            let mut v = 0u32;
            let mut any = false;
            while i < body.len() && body[i].is_ascii_digit() {
                v = v * 10 + (body[i] - b'0') as u32;
                any = true;
                if v > 0xFFFF {
                    any = false;
                    break;
                }
                i += 1;
            }
            if any && v > 0 {
                return Some(v as u16);
            }
        }
        // Follow the line link (an in-memory address; convert to an offset).
        let next = next_ptr as usize;
        let base = load as usize;
        if next <= base + line_at || next - base > payload.len() {
            return None; // corrupt / non-BASIC link
        }
        line_at = next - base;
    }
    None
}

/// Compress raw bytes into a standalone pucrunch file (`p`,`u` header) that
/// decompresses to `out_pos` and reports `exec` as the execution address.
pub fn compress_pucrunch_raw(data: &[u8], out_pos: u16, exec: u16) -> Vec<u8> {
    let p = compress_core(data, 8);

    // Placement (study §17): the packed stream must sit far enough above the
    // output that the forward write head never catches the unread bitstream.
    // The original's decoder simulates exactly `INPOS + bitstreamOffset` as
    // the read address versus `OUTPOS + p` as the write address (the header
    // bytes below the bitstream are consumed before the first write), so
    // INPOS must clear the largest write-vs-read gap. +4: strict inequality
    // plus the original's small safety allowance.
    let mut max_gap = 0i64;
    for (pos, &off) in p.off_at.iter().enumerate() {
        max_gap = max_gap.max(pos as i64 - off as i64);
    }
    let rle_used = p.table.len();
    let in_mem_size = 14 + rle_used + p.bitstream.len(); // header minus load addr
                                                         // The safe placement must also FIT below $10000. When it cannot (a nearly
                                                         // incompressible payload reaching the top of memory), clamp to the highest
                                                         // loadable address - the stream still decodes anywhere, but an in-place
                                                         // decode from the PRG load address is impossible for such a payload (the
                                                         // original's decoder reports "target exceeds source" for the same case).
    let want = out_pos as i64 + max_gap + 4;
    let highest = (0x1_0000i64 - in_mem_size as i64).max(0);
    let inpos = want.min(highest).max(0) as u16;
    let packed_end_minus_page = (inpos as i64 + in_mem_size as i64 - 256).clamp(0, 0xFFFF) as u16;

    let mut out = Vec::with_capacity(16 + rle_used + p.bitstream.len());
    out.extend_from_slice(&inpos.to_le_bytes());
    out.extend_from_slice(b"pu");
    out.extend_from_slice(&packed_end_minus_page.to_le_bytes());
    out.push(p.start_esc);
    out.extend_from_slice(&out_pos.to_le_bytes());
    out.push(p.esc_bits as u8);
    out.push(MAX_GAMMA as u8 + 1);
    out.push(SHORT_RLE_LIMIT as u8); // 128 wraps to $80, correct
    out.push(p.extra as u8);
    out.extend_from_slice(&exec.to_le_bytes());
    out.push(rle_used as u8);
    out.extend_from_slice(&p.table);
    out.extend_from_slice(&p.bitstream);
    out
}

/// Compress a PRG (2-byte load address + payload) into a standalone pucrunch
/// file. The execution address is auto-detected from a `SYS` line, falling
/// back to the load address (the original requires `-x` in that case; we
/// default instead of failing).
pub fn compress_pucrunch_prg(prg: &[u8]) -> Vec<u8> {
    assert!(prg.len() >= 2, "PRG needs a load address");
    let load = u16::from_le_bytes([prg[0], prg[1]]);
    let payload = &prg[2..];
    let exec = detect_sys(load, payload).unwrap_or(load);
    compress_pucrunch_raw(payload, load, exec)
}

/// Decode a standalone pucrunch file back into a PRG (2-byte load address +
/// payload). Validates the header per study §3.3.
pub fn decompress_pucrunch(file: &[u8]) -> Result<Vec<u8>, String> {
    if file.len() < 16 {
        return Err("file too short for a pucrunch header".into());
    }
    if &file[2..4] != b"pu" {
        return Err("bad magic (not a pucrunch file)".into());
    }
    let start_esc = file[6] as u32;
    let out_pos = u16::from_le_bytes([file[7], file[8]]);
    let esc_bits = file[9] as u32;
    // checked: a crafted maxGammaPlusOne of 0 must reject, not underflow.
    let Some(max_gamma) = (file[10] as u32).checked_sub(1) else {
        return Err("maxGammaPlusOne 0 out of range".into());
    };
    let short_rle = file[11] as u32;
    let extra = file[12] as u32;
    let rle_used = file[15] as usize;
    if esc_bits > 8 {
        return Err(format!("escBits {esc_bits} out of range"));
    }
    if !(5..=7).contains(&max_gamma) {
        return Err(format!("maxGamma {max_gamma} out of range"));
    }
    if short_rle != (1u32 << max_gamma) & 0xFF {
        return Err("shortRleLimit does not match maxGamma".into());
    }
    if extra > 4 {
        return Err(format!("extraLZPosBits {extra} out of range"));
    }
    if rle_used > 15 {
        return Err(format!("rleUsed {rle_used} out of range"));
    }
    if file.len() < 16 + rle_used {
        return Err("file too short for the RLE table".into());
    }
    let table = file[16..16 + rle_used].to_vec();
    let prm = StreamParams {
        start_esc,
        esc_bits,
        max_gamma,
        extra,
        table,
    };
    let mut r = BitReader::new(&file[16 + rle_used..]);
    let payload = decode_stream(&mut r, &prm)?;
    let mut out = Vec::with_capacity(payload.len() + 2);
    out.extend_from_slice(&out_pos.to_le_bytes());
    out.extend_from_slice(&payload);
    Ok(out)
}

// ---------------------------------------------------------------------------
// 6502 container (decoded by decrunchers/pucrunch-lzan*.s)
// ---------------------------------------------------------------------------
//
// Compact fixed-shape header the tiny 6502 decoder can consume with absolute
// addressing (comp_data is an assembly-time constant):
//
//   +0  startEsc     (right-aligned)
//   +1  escBits
//   +2  8 - escBits  (precomputed so the decoder needs no arithmetic)
//   +3  extraLZPosBits
//   +4  rank table, always 15 slots (unused slots zero)
//   +19 bitstream (maxGamma is always 7)
//
// The backward container is the byte-reversed forward stream with the header
// re-laid-out at the top end so a descending reader sees the same sequence and
// the table can still be indexed with ascending X:
//
//   [reverse(bitstream)] [rank1..rank15] [extra] [8-escBits] [escBits] [startEsc]

const HDR_6502: usize = 19;

pub fn compress_pucrunch_6502(input: &[u8]) -> Vec<u8> {
    let p = compress_core(input, 0);
    let mut out = Vec::with_capacity(HDR_6502 + p.bitstream.len());
    out.push(p.start_esc);
    out.push(p.esc_bits as u8);
    out.push(8 - p.esc_bits as u8);
    out.push(p.extra as u8);
    let mut tab = [0u8; 15];
    tab[..p.table.len()].copy_from_slice(&p.table);
    out.extend_from_slice(&tab);
    out.extend_from_slice(&p.bitstream);
    out
}

/// Backward/in-place variant: compress the reversed input, then lay the
/// stream out for a descending reader (see the layout note above).
pub fn compress_pucrunch_6502_backward(input: &[u8]) -> Vec<u8> {
    let rev: Vec<u8> = input.iter().rev().copied().collect();
    let p = compress_core(&rev, 0);
    let mut out = Vec::with_capacity(HDR_6502 + p.bitstream.len());
    out.extend(p.bitstream.iter().rev());
    let mut tab = [0u8; 15];
    tab[..p.table.len()].copy_from_slice(&p.table);
    out.extend_from_slice(&tab);
    out.push(p.extra as u8);
    out.push(8 - p.esc_bits as u8);
    out.push(p.esc_bits as u8);
    out.push(p.start_esc);
    out
}

fn decode_6502_params(hdr: &[u8]) -> Result<StreamParams, String> {
    let esc_bits = hdr[1] as u32;
    if esc_bits > 8 {
        return Err(format!("escBits {esc_bits} out of range"));
    }
    if hdr[2] as u32 != 8 - esc_bits {
        return Err("litBits field does not match escBits".into());
    }
    if hdr[3] > 4 {
        return Err(format!("extraLZPosBits {} out of range", hdr[3]));
    }
    Ok(StreamParams {
        start_esc: hdr[0] as u32,
        esc_bits,
        max_gamma: MAX_GAMMA,
        extra: hdr[3] as u32,
        table: hdr[4..19].to_vec(),
    })
}

/// In-place safety metric for the FORWARD 6502 container: the maximum over
/// all output bytes of `output_position - bitstream_byte_offset_at_token_start`
/// (study §17's closest-approach quantity, exact from a decode pass).
///
/// An end-aligned forward in-place layout (container occupying
/// `[top - len, top)`, output ascending from `span_start`) never lets the
/// write head reach an unread stream byte iff
/// `max_gap < (top - len + 19) - span_start`.
pub fn container_max_gap(stream: &[u8]) -> Result<i64, String> {
    if stream.len() < HDR_6502 {
        return Err("container too short".into());
    }
    let prm = decode_6502_params(&stream[..HDR_6502])?;
    let mut r = BitReader::new(&stream[HDR_6502..]);
    Ok(decode_stream_gap(&mut r, &prm)?.1)
}

/// As [`container_max_gap`], for the BACKWARD container. By mirror symmetry a
/// bottom-placed backward in-place layout (container at
/// `[packed_start, packed_start + len)`, output descending from
/// `span_end - 1`) is safe iff
/// `max_gap < (span_end - packed_start) - (len - 19)`.
pub fn container_max_gap_backward(stream: &[u8]) -> Result<i64, String> {
    if stream.len() < HDR_6502 {
        return Err("container too short".into());
    }
    let n = stream.len();
    let mut fwd = Vec::with_capacity(n);
    fwd.push(stream[n - 1]);
    fwd.push(stream[n - 2]);
    fwd.push(stream[n - 3]);
    fwd.push(stream[n - 4]);
    fwd.extend_from_slice(&stream[n - 19..n - 4]);
    fwd.extend(stream[..n - 19].iter().rev());
    container_max_gap(&fwd)
}

/// Reference decoder for the forward 6502 container.
pub fn decompress_pucrunch_6502(stream: &[u8]) -> Result<Vec<u8>, String> {
    if stream.len() < HDR_6502 {
        return Err("container too short".into());
    }
    let prm = decode_6502_params(&stream[..HDR_6502])?;
    let mut r = BitReader::new(&stream[HDR_6502..]);
    decode_stream(&mut r, &prm)
}

/// Reference decoder for the backward container: rebuild the forward shape,
/// decode, and un-reverse.
pub fn decompress_pucrunch_6502_backward(stream: &[u8]) -> Result<Vec<u8>, String> {
    if stream.len() < HDR_6502 {
        return Err("container too short".into());
    }
    let n = stream.len();
    let mut fwd = Vec::with_capacity(n);
    fwd.push(stream[n - 1]); // startEsc
    fwd.push(stream[n - 2]); // escBits
    fwd.push(stream[n - 3]); // 8-escBits
    fwd.push(stream[n - 4]); // extra
    fwd.extend_from_slice(&stream[n - 19..n - 4]); // table, rank-ascending
    fwd.extend(stream[..n - 19].iter().rev()); // bitstream
    let out = decompress_pucrunch_6502(&fwd)?;
    Ok(out.into_iter().rev().collect())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- gamma code against the study §4.2 table ----------------------------

    #[test]
    fn gamma_examples_match_study() {
        let cases: &[(u32, &str)] = &[
            (1, "0"),
            (2, "100"),
            (3, "101"),
            (4, "11000"),
            (5, "11001"),
            (6, "11010"),
            (7, "11011"),
            (8, "1110000"),
            (127, "1111110111111"),
            (128, "11111110000000"),
            (255, "11111111111111"),
        ];
        for &(v, bits) in cases {
            let mut w = BitWriter::new();
            w.gamma(v);
            assert_eq!(w.bit_len(), bits.len(), "gamma({v}) length");
            let bytes = w.finish();
            let got: String = (0..bits.len())
                .map(|i| {
                    let b = (bytes[i / 8] >> (7 - (i % 8))) & 1;
                    char::from(b'0' + b)
                })
                .collect();
            assert_eq!(got, bits, "gamma({v}) bits");
            assert_eq!(gamma_cost(v), bits.len() as u32, "gamma_cost({v})");
            // Round-trip.
            let mut r = BitReader::new(&bytes);
            assert_eq!(r.gamma(MAX_GAMMA).unwrap(), v);
        }
    }

    // ---- container round-trips ----------------------------------------------

    fn rt_6502(input: &[u8], label: &str) {
        let c = compress_pucrunch_6502(input);
        let d = decompress_pucrunch_6502(&c).unwrap_or_else(|e| panic!("[{label}] fwd: {e}"));
        assert_eq!(d, input, "[{label}] forward container");
        let cb = compress_pucrunch_6502_backward(input);
        let db = decompress_pucrunch_6502_backward(&cb)
            .unwrap_or_else(|e| panic!("[{label}] back: {e}"));
        assert_eq!(db, input, "[{label}] backward container");
    }

    #[test]
    fn container_roundtrips() {
        rt_6502(&[], "empty");
        rt_6502(b"X", "single");
        rt_6502(b"AA", "two");
        rt_6502(
            b"ABCABCABCABCABCABCABC the quick brown fox 123451234512345",
            "repetitive",
        );
        let run: Vec<u8> = std::iter::repeat_n(0x5A, 5000).collect();
        rt_6502(&run, "long_run");
        let mut mixed = Vec::new();
        for i in 0..4096usize {
            mixed.push(((i / 64) ^ (i % 7)) as u8);
        }
        rt_6502(&mixed, "mixed");
        let cyc: Vec<u8> = (0..=255u8).cycle().take(1000).collect();
        rt_6502(&cyc, "allbytes");
    }

    /// Conformance edges from study §23.4: gamma boundaries, LZ distances and
    /// lengths, RLE lengths around the short/long switch and 256-boundaries.
    #[test]
    fn conformance_edges() {
        // LZ distances 1, 255, 256, 257 and length boundaries: construct data
        // with a unique block repeated at controlled gaps.
        for gap in [1usize, 253, 254, 255, 256, 257] {
            let mut v: Vec<u8> = (0..64u8).map(|i| i.wrapping_mul(37)).collect(); // block A
            v.extend((0..gap).map(|i| 128 + ((i * 11) % 97) as u8)); // filler
            v.extend((0..64u8).map(|i| i.wrapping_mul(37))); // block A again
            rt_6502(&v, &format!("lz_gap_{gap}"));
        }
        // LZ lengths 2, 3, 255, 256 via long runs of a 2-byte pattern.
        for len in [2usize, 3, 255, 256, 300] {
            let mut v: Vec<u8> = b"start".to_vec();
            let pat: Vec<u8> = (0..len)
                .map(|i| if i % 2 == 0 { 0x11 } else { 0x22 })
                .collect();
            v.extend_from_slice(&pat);
            v.extend_from_slice(b"mid");
            v.extend_from_slice(&pat);
            rt_6502(&v, &format!("lz_len_{len}"));
        }
        // RLE lengths at the short/long switch and around 256/257 and beyond.
        for len in [2usize, 127, 128, 129, 255, 256, 257, 511, 513, 1000, 40000] {
            let mut v: Vec<u8> = b"abc".to_vec();
            v.extend(std::iter::repeat_n(0xEE, len));
            v.extend_from_slice(b"xyz");
            rt_6502(&v, &format!("rle_{len}"));
        }
        // Unranked RLE bytes: 16+ distinct run bytes forces codes >= 16.
        let mut v = Vec::new();
        for b in 0..20u8 {
            v.extend(std::iter::repeat_n(b.wrapping_mul(13), 30));
        }
        rt_6502(&v, "unranked_rle");
    }

    // ---- standalone format against the reference file -----------------------

    fn fixture(name: &str) -> Option<Vec<u8>> {
        std::fs::read(format!("test/pucrunch/{name}")).ok()
    }

    /// The committed reference stream (made by the original pucrunch, `-c0`
    /// C64 defaults) must decode byte-for-byte to the original PRG.
    #[test]
    fn reference_stream_decodes() {
        let (Some(bin), Some(prg)) = (fixture("test.bin"), fixture("test.prg")) else {
            eprintln!("skipping: test/pucrunch reference fixtures are not present");
            return;
        };
        let out = decompress_pucrunch(&bin).expect("reference stream decodes");
        assert_eq!(out, prg, "test.bin must decode to test.prg");
    }

    /// Our compressor must round-trip the reference PRG and beat the original
    /// compressor's size on it (1077 bytes total, 1046-byte bitstream).
    #[test]
    fn compresses_reference_prg_smaller_than_original() {
        let (Some(bin), Some(prg)) = (fixture("test.bin"), fixture("test.prg")) else {
            eprintln!("skipping: test/pucrunch reference fixtures are not present");
            return;
        };
        let ours = compress_pucrunch_prg(&prg);
        let back = decompress_pucrunch(&ours).expect("our stream decodes");
        assert_eq!(back, prg, "round-trip");
        assert!(
            ours.len() < bin.len(),
            "must beat the original: ours {} vs original {}",
            ours.len(),
            bin.len()
        );
        // Headers must agree on the essentials the original would produce.
        assert_eq!(&ours[2..4], b"pu");
        eprintln!(
            "pucrunch: ours {} bytes vs original {} bytes ({} saved)",
            ours.len(),
            bin.len(),
            bin.len() - ours.len()
        );
    }

    /// Cross-check: our standalone file for the reference PRG also decodes
    /// with our own strict decoder at every escBits we might emit, and the
    /// 6502 container carries identical payload.
    #[test]
    fn standalone_and_container_agree() {
        let Some(prg) = fixture("test.prg") else {
            eprintln!("skipping: test/pucrunch reference fixtures are not present");
            return;
        };
        let payload = &prg[2..];
        let c = compress_pucrunch_6502(payload);
        let d = decompress_pucrunch_6502(&c).unwrap();
        assert_eq!(d, payload);
        let cb = compress_pucrunch_6502_backward(payload);
        let db = decompress_pucrunch_6502_backward(&cb).unwrap();
        assert_eq!(db, payload);
        eprintln!(
            "pucrunch 6502 container: fwd {} B, back {} B for {} B payload",
            c.len(),
            cb.len(),
            payload.len()
        );
    }

    // ---- malformed input ------------------------------------------------------

    #[test]
    fn malformed_streams_rejected() {
        // Truncated header.
        assert!(decompress_pucrunch(&[0x01, 0x08, b'p']).is_err());
        // Bad magic.
        let mut f = compress_pucrunch_raw(b"hello world", 0x0801, 0x0801);
        f[2] = b'q';
        assert!(decompress_pucrunch(&f).is_err());
        // Truncated bitstream (missing EOF).
        let f = compress_pucrunch_raw(b"hello world hello world", 0x0801, 0x0801);
        let cut = &f[..f.len() - 2];
        assert!(decompress_pucrunch(cut).is_err());
        // Bad escBits.
        let mut f = compress_pucrunch_raw(b"hello", 0x0801, 0x0801);
        f[9] = 9;
        assert!(decompress_pucrunch(&f).is_err());
        // maxGammaPlusOne = 0 must reject, not underflow (debug-build panic).
        let mut f = compress_pucrunch_raw(b"hello", 0x0801, 0x0801);
        f[10] = 0;
        assert!(decompress_pucrunch(&f).is_err());
    }

    /// Placement fields must stay sane (no u16 wrap) even when the safe
    /// placement cannot fit below $10000; the stream itself still decodes.
    #[test]
    fn placement_clamps_instead_of_wrapping() {
        // 32000 zero bytes at $8300: output ends exactly at $10000 and the
        // stream is a single RLE token, so the naive placement would wrap.
        let f = compress_pucrunch_raw(&vec![0u8; 32000], 0x8300, 0x8300);
        let inpos = u16::from_le_bytes([f[0], f[1]]);
        let in_mem = f.len() - 2;
        assert!(
            inpos as usize + in_mem <= 0x1_0000,
            "INPOS ${inpos:04X} + {in_mem} B wraps past $FFFF"
        );
        assert!(
            inpos >= 0x8300,
            "INPOS ${inpos:04X} wrapped below the output"
        );
        let out = decompress_pucrunch(&f).unwrap();
        assert_eq!(out.len(), 32002);
        // Same idea near the very top of memory with a compressible payload.
        let f = compress_pucrunch_raw(&vec![0u8; 63000], 0x0801, 0x0801);
        let inpos = u16::from_le_bytes([f[0], f[1]]);
        assert!(inpos as usize + (f.len() - 2) <= 0x1_0000);
    }

    /// SYS detection must survive a leading REM line and a $9E byte inside a
    /// string before the real SYS (study §11.1: "scan near the beginning").
    #[test]
    fn detect_sys_walks_lines_and_retries() {
        // Line 1: 1 REM "x", line 2: 2 SYS2061, at $0801.
        let mut pl: Vec<u8> = Vec::new();
        let l2_at = 0x0801u16 + 8; // line 1 is 8 bytes: link+num+3-byte body+$00
        pl.extend_from_slice(&l2_at.to_le_bytes()); // link
        pl.extend_from_slice(&[0x01, 0x00, 0x8F, b'x', 0x9E, 0x00]); // 1 REM x <$9E-in-junk>
                                                                     // (the $9E inside line 1's body has no digits after it)
        let l3_at = l2_at + 10;
        pl.extend_from_slice(&l3_at.to_le_bytes());
        pl.extend_from_slice(&[0x02, 0x00, 0x9E, b'2', b'0', b'6', b'1', 0x00]); // 2 SYS2061
        pl.extend_from_slice(&[0x00, 0x00]); // end of program
        assert_eq!(detect_sys(0x0801, &pl), Some(2061));
        // No SYS anywhere -> None.
        let plain = [
            0x0B, 0x08, 0x0A, 0x00, 0x99, b'"', b'H', b'"', 0x00, 0x00, 0x00,
        ];
        assert_eq!(detect_sys(0x0801, &plain), None);
    }

    #[test]
    fn empty_input_is_just_eof() {
        let f = compress_pucrunch_raw(&[], 0x0801, 0x0801);
        let out = decompress_pucrunch(&f).unwrap();
        assert_eq!(out, vec![0x01, 0x08]);
    }
}
