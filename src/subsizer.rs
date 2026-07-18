//! Subsizer: bit-coded LZ with an iterated, adaptive entropy cost model and an optimal
//! (cheapest-path) parse. This is an independent Rust implementation of the Subsizer 0.6 stream
//! format, not the original source. See THIRD_PARTY.md for attribution and license.
//!
//! ## Format (the `-r` raw stream)
//! Subsizer emits, in order:
//!   1. an 8-bit "side byte" `endm` (the length value reserved as the end marker),
//!   2. five per-class "bits-base" encoding tables, each a list of 4-bit "part widths":
//!      `bitsl` (match length, 16 parts, unary prefix), `bits2` (offset for len==2, 16 parts,
//!      binary prefix), `bits3` (len==3, 16), `bits` (len>=4, 16), `bits1` (len==1, 4),
//!   3. a bit stream of tokens: `1` + side-byte literal, or `0` + length(`bitsl`) +
//!      offset(`bits1/2/3/bits`). A token whose decoded length equals `endm` is the end marker
//!      (its offset is not read).
//!
//! A "bits-base" code splits a value range into `n` consecutive parts; part `i` covers
//! `2^width[i]` values starting at a running base (`floor` initially). A value is coded as a
//! prefix selecting the part (`ceil(log2(n))` binary bits, or unary) followed by `width[i]`
//! raw bits for the offset within the part. The compressor *learns* the part widths from a
//! histogram of the symbols actually used on the current cheapest path, then re-parses against
//! the new code, iterating to a fixed point (Exomizer-class semi-static model).
//!
//! ## Bit packing (`BITMODE_SIDEBYTE`, no `BITMODE_PRESHIFT` in raw mode)
//! Bits are packed MSB-first within each byte; "side bytes" (literals, `endm`) are written as
//! whole bytes into the stream at the current byte position, interleaved with the bit buffer.
//! This is the bit-function layer.
//!
//! ## Uniform API
//!   - [`MAX_LEVEL`] = 2.  **level 1** = the FAST anchor (the single native iteration trajectory,
//!     byte-identical to `subsizer -r`); **level 2** = the multi-seed best-of (always `<=` level 1).
//!   - [`compress`] / [`decompress`] take a `backward: bool`
//!   - [`compress_native`] exposes the real knob: run the best-of over exactly `seeds` trajectories
//!     (`seeds == 1` ⇒ the anchor only; the anchor is always included so output `<=` native).
//!   - [`compress_subsizer`] / [`compress_subsizer_backward`] are the direct best-of entry points
//!
//! `decompress` is level-independent (one stream format across levels).  Backward mode mirrors the
//! reference `subsizer -r -b`: reverse the input, compress forward, reverse the output (and the
//! decoder reverses the stream, decodes forward, reverses the result).

/// Two-tier level system. **level 1 = fastest** (the single native iteration trajectory, the
/// no-regression ANCHOR - byte-identical to `subsizer -r`), **level 2 = absolute best** (the
/// multi-seed best-of, always `<=` level 1).
pub const MAX_LEVEL: u8 = 2;

// ---------------------------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------------------------

const LEN_PARTS: usize = 16;
const SINGLE_BYTE_PARTS: usize = 4;
const TWO_BYTE_PARTS: usize = 16;
const THREE_BYTE_PARTS: usize = 16;
const LONG_MATCH_PARTS: usize = 16;
const MIN_MATCH: i32 = 1;
const MAX_PARTS: usize = 16;
const MAX_VALUES: usize = 0x10000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Prefix {
    Binary,
    Unary,
}

// ---------------------------------------------------------------------------------------------
// Integer ceil(log2(x)).  Upstream uses ceil(log(x)/log(2)); for the values that occur (powers
// of two for table selectors, and 1..=N for the *_init costs) this is exactly equal to the
// integer ceil_log2 (verified against the reference's libm output).  ceil_log2(0)=0, (1)=0.
// ---------------------------------------------------------------------------------------------
#[inline]
fn ceil_log2(v: i64) -> i64 {
    if v <= 1 {
        return 0;
    }
    let mut k = 0i64;
    let mut p = 1i64;
    while p < v {
        p <<= 1;
        k += 1;
    }
    k
}

// ---------------------------------------------------------------------------------------------
// Encoding (bits-base code).
// ---------------------------------------------------------------------------------------------
#[derive(Clone, PartialEq, Eq)]
struct Encoding {
    floor: i32,
    n: usize,
    prefix: Prefix,
    parts: [u8; MAX_PARTS], // part widths (0..=15)
}

impl Encoding {
    fn zeroed() -> Encoding {
        Encoding {
            floor: 0,
            n: 0,
            prefix: Prefix::Binary,
            parts: [0; MAX_PARTS],
        }
    }
}

// cost_unary / write_unary / read_unary  (pol == 0 for PRE_UNARY)
#[inline]
fn cost_unary(v: i32, lim: i32) -> i32 {
    if v == lim - 1 {
        v
    } else {
        v + 1
    }
}

// cost_enc
fn cost_enc(enc: &Encoding, v: i32) -> i32 {
    let mut base = enc.floor;
    let mut of: i32 = -1;
    for i in 0..enc.n {
        base += 1 << enc.parts[i];
        if v < base {
            of = i as i32;
            break;
        }
    }
    if of < 0 {
        return 0x100000;
    }
    let of = of as usize;
    match enc.prefix {
        Prefix::Binary => ceil_log2(enc.n as i64) as i32 + enc.parts[of] as i32,
        Prefix::Unary => cost_unary(of as i32, enc.n as i32) + enc.parts[of] as i32,
    }
}

// ---------------------------------------------------------------------------------------------
// Encoding set (the five learned tables + end marker).
// ---------------------------------------------------------------------------------------------
#[derive(Clone, PartialEq, Eq)]
struct EncodingSet {
    endm: i32,
    bitsl: Encoding,
    bits1: Encoding,
    bits2: Encoding,
    bits3: Encoding,
    bits: Encoding,
}

impl EncodingSet {
    fn zeroed() -> EncodingSet {
        EncodingSet {
            endm: 0,
            bitsl: Encoding::zeroed(),
            bits1: Encoding::zeroed(),
            bits2: Encoding::zeroed(),
            bits3: Encoding::zeroed(),
            bits: Encoding::zeroed(),
        }
    }
}

// cost functions ------------------------------------------------------------------------------

// The "real" cost model (cfs): literal = 9 bits, lengths/offsets via learned tables.
#[inline]
fn cost_lit(_es: &EncodingSet, l: i32) -> i32 {
    l * 9
}
#[inline]
fn cost_mlen(es: &EncodingSet, l: i32) -> i32 {
    cost_enc(&es.bitsl, l)
}
#[inline]
fn cost_moffs(es: &EncodingSet, of: i32, l: i32) -> i32 {
    match l {
        1 => cost_enc(&es.bits1, of),
        2 => cost_enc(&es.bits2, of),
        3 => cost_enc(&es.bits3, of),
        _ => cost_enc(&es.bits, of),
    }
}

// The "init" cost model (cfs_init), used only for the very first parse before any table exists.
#[inline]
fn cost_lit_init(_es: &EncodingSet, l: i32) -> i32 {
    l * 9
}
#[inline]
fn cost_mlen_init(_es: &EncodingSet, l: i32) -> i32 {
    // ceil(0 + log(l)/log(2))
    ceil_log2(l as i64) as i32
}
#[inline]
fn cost_moffs_init(_es: &EncodingSet, of: i32, l: i32) -> i32 {
    if l == 1 {
        ceil_log2(of as i64) as i32
    } else {
        2 + ceil_log2(of as i64) as i32
    }
}

/// Which cost model is in force (selects between cfs and cfs_init).
#[derive(Clone, Copy, PartialEq, Eq)]
enum CostModel {
    Init,
    Real,
}

impl CostModel {
    #[inline]
    fn cost_lit(self, es: &EncodingSet, l: i32) -> i32 {
        match self {
            CostModel::Init => cost_lit_init(es, l),
            CostModel::Real => cost_lit(es, l),
        }
    }
    #[inline]
    fn cost_mlen(self, es: &EncodingSet, l: i32) -> i32 {
        match self {
            CostModel::Init => cost_mlen_init(es, l),
            CostModel::Real => cost_mlen(es, l),
        }
    }
    #[inline]
    fn cost_moffs(self, es: &EncodingSet, of: i32, l: i32) -> i32 {
        match self {
            CostModel::Init => cost_moffs_init(es, of, l),
            CostModel::Real => cost_moffs(es, of, l),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Histogram (no sliding window, window == 0 in all uses here).
// `bin[v]` counts occurrences, `cost[v]` accumulates the "alternative cost" delta used by the
// encoding optimizer as `cost_left`.
// ---------------------------------------------------------------------------------------------
struct Hist {
    range: usize,
    bin: Vec<u32>,
    cost: Vec<i64>,
}

impl Hist {
    fn new(range: usize) -> Hist {
        Hist {
            range,
            bin: vec![0; range],
            cost: vec![0; range],
        }
    }
    #[inline]
    fn add(&mut self, v: i32, cost: i32) {
        let v = v as usize;
        self.bin[v] += 1;
        self.cost[v] += cost as i64;
    }
}

// ---------------------------------------------------------------------------------------------
// optimize_enc: given a histogram of values + their alternative costs, find the
// part widths that minimise total encoded bits, with a cache.
// ---------------------------------------------------------------------------------------------
struct EncOptimizer {
    base_cost: [i64; MAX_PARTS],
    so_far: Vec<i64>,    // prefix sum of bin counts, length MAX_VALUES+1
    cost_left: Vec<i64>, // suffix sum of alternative costs, length MAX_VALUES+1
    // cache[base][bit] -> (len, parts[bit..])  (CacheEntry cache[CACHE_ENTRIES][MAX_PARTS])
    cache_len: Vec<[i64; MAX_PARTS]>,
    cache_parts: Vec<[[i8; MAX_PARTS]; MAX_PARTS]>,
}

impl EncOptimizer {
    fn new() -> EncOptimizer {
        EncOptimizer {
            base_cost: [0; MAX_PARTS],
            so_far: vec![0; MAX_VALUES + 1],
            cost_left: vec![0; MAX_VALUES + 1],
            cache_len: vec![[-1; MAX_PARTS]; MAX_VALUES],
            cache_parts: vec![[[0; MAX_PARTS]; MAX_PARTS]; MAX_VALUES],
        }
    }

    fn invalidate_cache(&mut self) {
        for e in self.cache_len.iter_mut() {
            *e = [-1; MAX_PARTS];
        }
    }

    fn build_arrays(&mut self, h: &Hist) {
        for v in self.so_far.iter_mut() {
            *v = 0;
        }
        for v in self.cost_left.iter_mut() {
            *v = 0;
        }
        let mut acc: i64 = 0;
        let mut i = 0;
        while i < h.range {
            self.so_far[i] = acc;
            acc += h.bin[i] as i64;
            i += 1;
        }
        self.so_far[i] = acc;

        let mut cost: i64 = 0;
        self.cost_left[h.range] = 0;
        let mut i = h.range as i64 - 1;
        while i >= 0 {
            cost += h.cost[i as usize];
            self.cost_left[i as usize] = cost;
            i -= 1;
        }
    }

    // recursive calc_enc(n_b, bit, base, enc) -> min_len; writes chosen widths into `enc[bit..]`.
    fn calc_enc(&mut self, n_b: usize, bit: usize, base: usize, enc: &mut [i8; MAX_PARTS]) -> i64 {
        let mut min_len: i64 = 0x10000000;
        let mut min_enc = [0i8; MAX_PARTS];

        for i in 0..16usize {
            let mut lim = base + (1usize << i);
            let cost = self.base_cost[bit] + i as i64;

            if lim > MAX_VALUES - 1 {
                if bit < n_b - 1 {
                    break;
                }
                lim = MAX_VALUES;
            }

            let mut len = (self.so_far[lim] - self.so_far[base]) * cost;

            if bit < n_b - 1 {
                if self.cost_left[lim] != 0 {
                    let tmp = self.find_cache(n_b, bit + 1, lim, &mut min_enc);
                    if tmp >= 0 {
                        len += tmp;
                    } else {
                        len += self.calc_enc(n_b, bit + 1, lim, &mut min_enc);
                    }
                } else {
                    // didn't use all entries
                    break;
                }
            } else {
                // out of bits, return the alternative cost
                len += self.cost_left[lim];
            }

            if len < min_len {
                min_len = len;
                enc[bit] = i as i8;
                for j in (bit + 1)..n_b {
                    enc[j] = min_enc[j];
                }
            }
        }

        self.add_cache(n_b, bit, base, enc, min_len);
        min_len
    }

    fn add_cache(&mut self, n_b: usize, bit: usize, base: usize, enc: &[i8; MAX_PARTS], len: i64) {
        let id = base; // hash_func(base) == base
        self.cache_len[id][bit] = len;
        for i in bit..n_b {
            self.cache_parts[id][bit][i] = enc[i];
        }
    }

    fn find_cache(&self, n_b: usize, bit: usize, base: usize, enc: &mut [i8; MAX_PARTS]) -> i64 {
        let id = base;
        let l = self.cache_len[id][bit];
        if l >= 0 {
            for i in bit..n_b {
                enc[i] = self.cache_parts[id][bit][i];
            }
        }
        l
    }

    fn optimize_enc(&mut self, h: &Hist, floor: i32, n: usize, prefix: Prefix, enc: &mut Encoding) {
        enc.floor = floor;
        enc.n = n;
        enc.prefix = prefix;

        self.build_arrays(h);

        for i in 0..n {
            self.base_cost[i] = match prefix {
                Prefix::Binary => ceil_log2(n as i64),
                Prefix::Unary => cost_unary(i as i32, n as i32) as i64,
            };
        }

        self.invalidate_cache();

        let mut bits = [0i8; MAX_PARTS];
        self.calc_enc(n, 0, floor as usize, &mut bits);

        for i in 0..n {
            enc.parts[i] = bits[i] as u8;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Match finder.  Per position, a list of candidate matches/RLE terminated by an "end"
// marker.  We store candidates flat and index into them per position.  Ordering is preserved
// exactly (RLE first when applicable, then matches by increasing offset).
// ---------------------------------------------------------------------------------------------
#[derive(Clone, Copy)]
enum Cand {
    Match { offs: i32, len: i32 },
    Rle { len: i32 }, // RLE: offset is always 1
}

struct MatchTree {
    len: usize,
    // candidates[cur] = slice of candidate matches at position cur (may be empty == literal-only)
    starts: Vec<u32>, // index into `cands` where position cur's list begins
    counts: Vec<u32>, // number of candidates at position cur
    cands: Vec<Cand>,
}

const MT_MIN_OFFS: i32 = 1;
const MT_MAX_OFFS: i32 = 0xFFFF;
const MT_MAX_OFFS1: i32 = 32;
const MT_MAX_OFFS2: i32 = 0x4000;
const MT_MIN_LEN: i32 = 1;
const MT_MAX_LEN: i32 = 0x100;
const MT_MIN_RLE: i32 = 2;
const MT_MAX_RLE: i32 = 0x100;
const MT_RLE_HOLDOFF: i32 = 8;

fn build_match(buf: &[u8]) -> MatchTree {
    let len = buf.len();
    let mut starts = vec![0u32; len];
    let mut counts = vec![0u32; len];
    let mut cands: Vec<Cand> = Vec::new();

    let mut rcnt: i32 = 0;
    let mut cur: usize = 0;
    while cur < len {
        let v = buf[cur];
        let list_start = cands.len();

        // check rle
        let mut rlen: i32 = 1;
        while (cur + rlen as usize) < len && buf[cur + rlen as usize] == v {
            rlen += 1;
            if rlen == MT_MAX_RLE {
                break;
            }
        }
        let mut holdoff_skip = false;
        if rlen >= MT_MIN_RLE {
            // skip the first rle
            if rcnt > 0 {
                cands.push(Cand::Rle { len: rlen });
            }
            rcnt += 1;
            if rcnt > MT_RLE_HOLDOFF {
                holdoff_skip = true;
            }
        } else {
            rcnt = 0;
        }

        if !holdoff_skip {
            // max search range from the current offset
            let window = if (cur as i32) < MT_MAX_OFFS {
                cur as i32
            } else {
                MT_MAX_OFFS
            };

            let mut i = MT_MIN_OFFS;
            while i <= window {
                let moffs = i;
                let mut mlen: i32 = 0;

                if buf[cur - i as usize] == v {
                    mlen += 1;
                    while (cur + mlen as usize) < len
                        && buf[cur - i as usize + mlen as usize] == buf[cur + mlen as usize]
                    {
                        mlen += 1;
                        if mlen == MT_MAX_LEN {
                            break;
                        }
                    }
                }

                if mlen >= MT_MIN_LEN
                    && ((mlen >= 1 && moffs <= MT_MAX_OFFS1)
                        || (mlen >= 2 && moffs <= MT_MAX_OFFS2)
                        || (mlen >= 3))
                {
                    cands.push(Cand::Match {
                        offs: moffs,
                        len: mlen,
                    });
                }

                i += 1;
            }
        }

        starts[cur] = list_start as u32;
        counts[cur] = (cands.len() - list_start) as u32;

        cur += 1;
    }

    MatchTree {
        len,
        starts,
        counts,
        cands,
    }
}

// ---------------------------------------------------------------------------------------------
// PrimaryPath + find_cheapest_path
// ---------------------------------------------------------------------------------------------
#[derive(Clone, Copy)]
enum PathStep {
    Literal, // single literal byte
    Match { offs: i32, len: i32 },
}

struct PrimaryPath {
    path: Vec<PathStep>,
    // `cost` (total parse bits) and `len` (source length) are carried for fidelity with the C
    // `PrimaryPath` struct; the raw-mode pipeline does not read them back.
    #[allow(dead_code)]
    cost: i64,
    #[allow(dead_code)]
    len: usize,
}

/// prepare_fast + find_cheapest_path.
fn find_cheapest_path(mt: &MatchTree, model: CostModel, es: &EncodingSet) -> PrimaryPath {
    let len = mt.len;

    // Fast cost tables (prepare_fast): litcost/lencost/offscost{1,2,3,}.
    // The match finder's window allows an offset of exactly MT_MAX_OFFS (0x10000), one past the
    // value range the C build's fixed `[0x10000]` arrays cover - upstream then reads one element
    // out of bounds (UB; it segfaults on `pic`). We instead size the offset tables to MAX_VALUES+1
    // and compute the genuine cost for index 0x10000 too (which is the `0x100000` "fault" cost
    // whenever the learned table can't represent that offset), so such a match is simply never the
    // cheapest - no UB, no panic. On every file the native handles, no offset-0x10000 candidate
    // exists, so the parse (and thus the output) is unchanged; only the otherwise-uncompressable
    // `pic` benefits. litcost/lencost are indexed by length (<= MT_MAX_LEN) so they keep size N.
    let mut litcost = vec![0i32; MAX_VALUES];
    let mut lencost = vec![0i32; MAX_VALUES];
    let mut offscost1 = vec![0i32; MAX_VALUES + 1];
    let mut offscost2 = vec![0i32; MAX_VALUES + 1];
    let mut offscost3 = vec![0i32; MAX_VALUES + 1];
    let mut offscost = vec![0i32; MAX_VALUES + 1];
    for i in 0..MAX_VALUES {
        let iv = i as i32;
        litcost[i] = model.cost_lit(es, iv);
        lencost[i] = model.cost_mlen(es, iv);
    }
    for i in 0..=MAX_VALUES {
        let iv = i as i32;
        offscost1[i] = model.cost_moffs(es, iv, 1);
        offscost2[i] = model.cost_moffs(es, iv, 2);
        offscost3[i] = model.cost_moffs(es, iv, 3);
        offscost[i] = model.cost_moffs(es, iv, 4);
    }
    let fast_moffs = |of: i32, l: i32| -> i32 {
        match l {
            1 => offscost1[of as usize],
            2 => offscost2[of as usize],
            3 => offscost3[of as usize],
            _ => offscost[of as usize],
        }
    };

    let mut dist = vec![i64::MAX; len + 1];
    let mut prev = vec![-1i64; len + 1];
    let mut path = vec![PathStep::Literal; len + 1];
    dist[0] = 0;

    let mut cur: usize = 0;
    while cur < len {
        let dcur = dist[cur];
        if dcur != i64::MAX {
            // iterate the candidate list at this position
            let start = mt.starts[cur] as usize;
            let count = mt.counts[cur] as usize;
            for k in 0..count {
                match mt.cands[start + k] {
                    Cand::Match { offs, len: mlen0 } => {
                        if mlen0 >= MIN_MATCH {
                            let of = offs;
                            let mut l = mlen0;
                            let mut c = 4;
                            while c > 0 && l >= MIN_MATCH {
                                let w = (1 + lencost[l as usize] + fast_moffs(of, l)) as i64;
                                let vtx = cur + l as usize;
                                if dist[vtx] > dcur + w {
                                    dist[vtx] = dcur + w;
                                    prev[vtx] = cur as i64;
                                    path[vtx] = PathStep::Match { offs: of, len: l };
                                }
                                l -= 1;
                                c -= 1;
                            }
                        }
                    }
                    Cand::Rle { len: rlen0 } => {
                        if rlen0 >= MIN_MATCH {
                            let of = 1;
                            let mut l = rlen0;
                            let mut c = 4;
                            while c > 0 && l >= MIN_MATCH {
                                let w = (1 + lencost[l as usize] + fast_moffs(of, l)) as i64;
                                let vtx = cur + l as usize;
                                if dist[vtx] > dcur + w {
                                    dist[vtx] = dcur + w;
                                    prev[vtx] = cur as i64;
                                    path[vtx] = PathStep::Match { offs: of, len: l };
                                }
                                l -= 1;
                                c -= 1;
                            }
                        }
                    }
                }
            }

            // literal
            let w = litcost[1] as i64;
            let vtx = cur + 1;
            if dist[vtx] > dcur + w {
                dist[vtx] = dcur + w;
                prev[vtx] = cur as i64;
                path[vtx] = PathStep::Literal;
            }
        }
        cur += 1;
    }

    // Backtrack: count steps.
    let mut i: i64 = len as i64;
    let mut j: usize = 0;
    while i > 0 {
        j += 1;
        i = prev[i as usize];
    }

    let n = j;
    let mut pp = PrimaryPath {
        path: vec![PathStep::Literal; n],
        cost: dist[len],
        len,
    };

    // Backtrack: fill steps in order.
    let mut i: i64 = len as i64;
    let mut j: i64 = n as i64 - 1;
    while i > 0 {
        pp.path[j as usize] = path[i as usize];
        j -= 1;
        i = prev[i as usize];
    }

    pp
}

// ---------------------------------------------------------------------------------------------
// optimize_encoding: walk the cheapest path, build histograms with alternative
// costs, then optimize each table and pick an end marker.
// ---------------------------------------------------------------------------------------------
fn optimize_encoding(
    pp: &PrimaryPath,
    buf: &[u8],
    model: CostModel,
    es: &mut EncodingSet,
    opt: &mut EncOptimizer,
) {
    let mut h_lit = Hist::new(0x100);
    let mut h_len = Hist::new(0x10000);
    let mut h_offs1 = Hist::new(0x10000);
    let mut h_offs2 = Hist::new(0x10000);
    let mut h_offs3 = Hist::new(0x10000);
    let mut h_offs = Hist::new(0x10000);
    // h_lit_run / h_mat_run are accumulated but never used to optimize an encoding; skip them.

    let mut src: usize = 0;
    for step in &pp.path {
        match *step {
            PathStep::Match { offs, len: l } if l >= MIN_MATCH => {
                let of = offs;
                let cost_lt = model.cost_lit(es, l);
                let cost_of = 1 + model.cost_moffs(es, of, l);
                let cost_l = 1 + model.cost_mlen(es, l);

                h_len.add(l, cost_lt - cost_of);
                match l {
                    1 => h_offs1.add(of, cost_lt - cost_l),
                    2 => h_offs2.add(of, cost_lt - cost_l),
                    3 => h_offs3.add(of, cost_lt - cost_l),
                    _ => h_offs.add(of, cost_lt - cost_l),
                }
                src += l as usize;
            }
            _ => {
                // literal (also covers a degenerate Match with len < MIN_MATCH, which never occurs)
                let lit = buf[src];
                h_lit.add(lit as i32, 0);
                src += 1;
            }
        }
    }

    opt.optimize_enc(&h_len, MIN_MATCH, LEN_PARTS, Prefix::Unary, &mut es.bitsl);
    opt.optimize_enc(
        &h_offs1,
        1,
        SINGLE_BYTE_PARTS,
        Prefix::Binary,
        &mut es.bits1,
    );
    opt.optimize_enc(&h_offs2, 1, TWO_BYTE_PARTS, Prefix::Binary, &mut es.bits2);
    opt.optimize_enc(&h_offs3, 1, THREE_BYTE_PARTS, Prefix::Binary, &mut es.bits3);
    opt.optimize_enc(&h_offs, 1, LONG_MATCH_PARTS, Prefix::Binary, &mut es.bits);

    // find end marker candidate: first length >= MIN_MATCH whose bin count is 0.
    es.endm = 0;
    for i in (MIN_MATCH as usize)..h_len.range {
        if h_len.bin[i] == 0 {
            es.endm = i as i32;
            break;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Bit writer, BITMODE_SIDEBYTE, MSB-first, no preshift in raw mode.
// ---------------------------------------------------------------------------------------------
struct BitWriter {
    out: Vec<u8>,
    pos: usize,     // next byte position to allocate
    buf: u32,       // current partial byte accumulator (low `bit` bits significant)
    bit: i32,       // number of bits currently buffered (0..7)
    bpos: usize,    // byte position reserved for the current partial byte
    preshift: bool, // BITMODE_PRESHIFT: marker-bit reservoir framing (memory/sfx mode)
}

impl BitWriter {
    fn new() -> BitWriter {
        BitWriter {
            out: Vec::new(),
            pos: 0,
            buf: 0,
            bit: 0,
            bpos: 0,
            preshift: false,
        }
    }

    /// `BITMODE_PRESHIFT` writer (memory/executable mode), with
    /// `BITMODE_PRESHIFT`: a single leading bit is reserved up front, and on `flush` the first
    /// byte is `rol`:ed with a `1` inserted in its LSB.  That `1` becomes the sentinel marker bit
    /// the standalone 6502 decoder (`get_bit` = `asl buf_zp / bne ok / refill`) keys off:
    /// every byte the decoder pulls in carries a leading `1` marker, except the seeded first byte.
    fn new_preshift() -> BitWriter {
        let mut bw = BitWriter {
            out: Vec::new(),
            pos: 0,
            buf: 0,
            bit: 0,
            bpos: 0,
            preshift: true,
        };
        // Reserve the leading bit (applied as a shift-in `1` at flush time).
        bw.write(0, 1);
        bw
    }

    #[inline]
    fn ensure(&mut self, idx: usize) {
        if idx >= self.out.len() {
            self.out.resize(idx + 1, 0);
        }
    }

    fn write(&mut self, data: u32, n: i32) {
        let mut n = n;
        let data = if n >= 32 {
            data
        } else {
            data & ((1u32 << n) - 1)
        };
        while n > 0 {
            let fr = 8 - self.bit;
            let nn = if n > fr { fr } else { n };
            if self.bit == 0 {
                self.bpos = self.pos;
                self.pos += 1;
            }
            self.buf <<= nn;
            self.buf |= data >> (n - nn);
            self.bit += nn;
            if self.bit == 8 {
                self.ensure(self.bpos);
                self.out[self.bpos] = self.buf as u8;
                self.bit = 0;
            }
            n -= nn;
        }
    }

    // bitwr_write8s with SIDEBYTE flag set: write a whole byte at the current byte position.
    fn write8s(&mut self, data: u8) {
        self.ensure(self.pos);
        self.out[self.pos] = data;
        self.pos += 1;
    }

    fn flush(&mut self) -> usize {
        if self.bit > 0 {
            let pad = 8 - self.bit;
            self.write(0, pad);
        }
        if self.preshift {
            // First byte: rol one bit with a `1` inserted (the MSB was reserved/unused). Mirrors
            // `bitwr_flush`'s `ptr[0] = (ptr[0] << 1) | 0x01`.
            self.ensure(0);
            self.out[0] = (self.out[0] << 1) | 0x01;
        }
        self.pos
    }
}

// ---------------------------------------------------------------------------------------------
// write_unary / write_enc / write_mlen / write_moffs / write_endm
// ---------------------------------------------------------------------------------------------
fn write_unary(bw: &mut BitWriter, v: i32, lim: i32) {
    // pol == 0: mask = 0, endbit = 1
    bw.write(0, v);
    if v < lim - 1 {
        bw.write(1, 1);
    }
}

fn write_enc(bw: &mut BitWriter, enc: &Encoding, v: i32) {
    let mut base = enc.floor;
    let mut of: i32 = -1;
    for i in 0..enc.n {
        base += 1 << enc.parts[i];
        if v < base {
            of = i as i32;
            break;
        }
    }
    // upstream leaves `of == -1` to "fault" silently; that never happens for valid streams.
    let of = of as usize;
    match enc.prefix {
        Prefix::Binary => bw.write(of as u32, ceil_log2(enc.n as i64) as i32),
        Prefix::Unary => write_unary(bw, of as i32, enc.n as i32),
    }
    bw.write((v - base) as u32, enc.parts[of] as i32);
}

fn write_mlen(bw: &mut BitWriter, es: &EncodingSet, l: i32) {
    write_enc(bw, &es.bitsl, l);
}
fn write_moffs(bw: &mut BitWriter, es: &EncodingSet, of: i32, l: i32) {
    match l {
        1 => write_enc(bw, &es.bits1, of),
        2 => write_enc(bw, &es.bits2, of),
        3 => write_enc(bw, &es.bits3, of),
        _ => write_enc(bw, &es.bits, of),
    }
}
fn write_endm(bw: &mut BitWriter, es: &EncodingSet) {
    bw.write(0, 1);
    write_mlen(bw, es, es.endm);
}

/// `cost_enc` returns this sentinel when a value is not representable by an `Encoding`.
const ENC_FAULT: i32 = 0x100000;

/// True iff every token on `pp` and the end marker are representable by `es` - i.e. `generate`
/// would write each without hitting the `of == -1` fault path.  Lengths use `bitsl`, offsets the
/// per-length-class table; the literal side bytes are always representable (raw 8-bit).
fn path_encodable(pp: &PrimaryPath, es: &EncodingSet) -> bool {
    if cost_enc(&es.bitsl, es.endm) >= ENC_FAULT {
        return false;
    }
    for step in &pp.path {
        if let PathStep::Match { offs, len } = *step {
            if cost_mlen(es, len) >= ENC_FAULT || cost_moffs(es, offs, len) >= ENC_FAULT {
                return false;
            }
        }
    }
    true
}

// ---------------------------------------------------------------------------------------------
// generate: emit the side byte, the five tables, the token stream, end marker.
// ---------------------------------------------------------------------------------------------
fn generate(pp: &PrimaryPath, buf: &[u8], es: &EncodingSet) -> Vec<u8> {
    generate_into(BitWriter::new(), pp, buf, es)
}

/// `generate` against the `BITMODE_PRESHIFT` (marker-bit reservoir) writer - the framing the
/// standalone memory/executable 6502 decoder reads.  Identical token stream to [`generate`], only
/// the bit IO framing differs (a reserved leading bit + the first byte `rol`:ed with a `1`).
fn generate_preshift(pp: &PrimaryPath, buf: &[u8], es: &EncodingSet) -> Vec<u8> {
    generate_into(BitWriter::new_preshift(), pp, buf, es)
}

/// Shared body of [`generate`] / [`generate_preshift`]: emit the side byte, the five tables, the
/// token stream and the end marker into `bw` (which already carries the chosen framing mode).
fn generate_into(mut bw: BitWriter, pp: &PrimaryPath, buf: &[u8], es: &EncodingSet) -> Vec<u8> {
    bw.write8s(es.endm as u8);

    for i in 0..es.bitsl.n {
        bw.write(es.bitsl.parts[i] as u32, 4);
    }
    for i in 0..es.bits2.n {
        bw.write(es.bits2.parts[i] as u32, 4);
    }
    for i in 0..es.bits3.n {
        bw.write(es.bits3.parts[i] as u32, 4);
    }
    for i in 0..es.bits.n {
        bw.write(es.bits.parts[i] as u32, 4);
    }
    for i in 0..es.bits1.n {
        bw.write(es.bits1.parts[i] as u32, 4);
    }

    let mut l: usize = 0;
    for step in &pp.path {
        match *step {
            PathStep::Match { offs, len } => {
                bw.write(0, 1);
                write_mlen(&mut bw, es, len);
                write_moffs(&mut bw, es, offs, len);
                l += len as usize;
            }
            PathStep::Literal => {
                bw.write(1, 1);
                bw.write8s(buf[l]);
                l += 1;
            }
        }
    }

    write_endm(&mut bw, es);

    let n = bw.flush();
    bw.out.truncate(n);
    bw.out
}

// ---------------------------------------------------------------------------------------------
// Iterated convergence core (optimize_tree) - parameterised so several seed trajectories can be
// tried.  Given a *starting* EncodingSet, run the Real-cost fixed-point loop exactly as upstream:
//   repeat { last = es; pp = find_cheapest_path(Real); es = optimize_encoding(pp, Real) }
//   until es == last  (or max_passes reached).
// Returns the converged path + encoding (ready for `generate`).
//
// The native trajectory is reproduced exactly by `run_native` (Init parse → optimize(Init) → the
// Real loop, ≤16 passes); that result is the no-regression ANCHOR.  Other seeds only change the
// *start* of the iteration, never the iteration itself, so every candidate is a legitimate Subsizer
// stream - and because the anchor is always in the set, the best-of can never exceed native.
// ---------------------------------------------------------------------------------------------
fn converge(
    mt: &MatchTree,
    buf: &[u8],
    opt: &mut EncOptimizer,
    es: &mut EncodingSet,
    max_passes: u32,
) -> PrimaryPath {
    let mut pp = find_cheapest_path(mt, CostModel::Real, es);
    optimize_encoding(&pp, buf, CostModel::Real, es, opt);
    for _ in 0..max_passes {
        let last_es = es.clone();
        pp = find_cheapest_path(mt, CostModel::Real, es);
        optimize_encoding(&pp, buf, CostModel::Real, es, opt);
        if *es == last_es {
            break;
        }
    }
    pp
}

/// The exact native trajectory: Init-cost parse, optimise with the Init model, then the Real-cost
/// fixed-point loop (≤16 passes).  This is the ANCHOR - `generate` of its result is byte-identical
/// to `subsizer -r`.
fn run_native(mt: &MatchTree, buf: &[u8], opt: &mut EncOptimizer) -> (PrimaryPath, EncodingSet) {
    let mut es = EncodingSet::zeroed();

    let mut pp = find_cheapest_path(mt, CostModel::Init, &es);
    optimize_encoding(&pp, buf, CostModel::Init, &mut es, opt);

    let max_passes = 16;
    for _ in 0..max_passes {
        let last_es = es.clone();
        pp = find_cheapest_path(mt, CostModel::Real, &es);
        optimize_encoding(&pp, buf, CostModel::Real, &mut es, opt);
        if es == last_es {
            break;
        }
    }
    (pp, es)
}

/// Build an EncodingSet directly from a histogram of the symbols on a given path.  The
/// alternative-cost deltas MUST be computed with a cost model that does not depend on a learned
/// table - i.e. [`CostModel::Init`] - because a zeroed/garbage table makes `cost_enc` return the
/// huge `0x100000` "fault" cost for most values, which in turn blows up `optimize_enc`'s
/// `calc_enc` recursion (the cache keys on running bases that then never repeat).  The native only
/// ever seeds the histogram via the Init model for exactly this reason.
fn seed_es_from_path(pp: &PrimaryPath, buf: &[u8], opt: &mut EncOptimizer) -> EncodingSet {
    let mut es = EncodingSet::zeroed();
    optimize_encoding(pp, buf, CostModel::Init, &mut es, opt);
    es
}

/// Perturb every learned part width of a (sane, converged) EncodingSet by `delta`, clamped to
/// 0..=15, leaving table shapes intact.  The result still spans the full value range, so feeding it
/// to the Real cost model is safe (no fault costs) - a gentle nudge to a neighbouring fixed point.
fn perturb_encoding_set(src: &EncodingSet, delta: i32) -> EncodingSet {
    let mut es = src.clone();
    let nudge = |e: &mut Encoding| {
        for i in 0..e.n {
            let v = e.parts[i] as i32 + delta;
            e.parts[i] = v.clamp(0, 15) as u8;
        }
    };
    nudge(&mut es.bitsl);
    nudge(&mut es.bits1);
    nudge(&mut es.bits2);
    nudge(&mut es.bits3);
    nudge(&mut es.bits);
    es
}

/// A greedy parse: at each position take the single longest available match (RLE counts as a
/// match at offset 1), else a literal.  Offset-class length capping is *not* applied (we keep the
/// match-finder's reported length, clamped so the candidate is encodable), giving a different
/// starting histogram than the Init-cost optimal parse.
fn greedy_path(mt: &MatchTree, buf: &[u8]) -> PrimaryPath {
    let len = mt.len;
    let mut steps: Vec<PathStep> = Vec::new();
    let mut cur = 0usize;
    while cur < len {
        let start = mt.starts[cur] as usize;
        let count = mt.counts[cur] as usize;
        let mut best_len: i32 = 0;
        let mut best_offs: i32 = 0;
        for k in 0..count {
            let (offs, mlen) = match mt.cands[start + k] {
                Cand::Match { offs, len } => (offs, len),
                Cand::Rle { len } => (1, len),
            };
            if mlen > best_len {
                best_len = mlen;
                best_offs = offs;
            }
        }
        if best_len >= 2 {
            steps.push(PathStep::Match {
                offs: best_offs,
                len: best_len,
            });
            cur += best_len as usize;
        } else {
            steps.push(PathStep::Literal);
            cur += 1;
        }
    }
    let _ = buf;
    PrimaryPath {
        path: steps,
        cost: 0,
        len,
    }
}

/// An all-literal parse - the most neutral histogram seed (no matches at all).
fn literal_path(len: usize) -> PrimaryPath {
    PrimaryPath {
        path: vec![PathStep::Literal; len],
        cost: 0,
        len,
    }
}

/// Total number of seed trajectories available to the best-of (the ANCHOR plus
/// every extra candidate). `compress_native(_, NUM_SEEDS, _)` runs the full
/// best-of; `compress_native(_, 1, _)` runs the anchor only.
const NUM_SEEDS: usize = 9;

// ---------------------------------------------------------------------------------------------
// crunch_normal_int: best-of over up to `seeds` iteration trajectories.  Emits the SMALLEST valid
// (round-trippable) generated stream.  The native trajectory (the ANCHOR) is candidate 0 and is
// always run (even for `seeds == 1`), so the result is guaranteed `<=` `subsizer -r` on every
// input.  `seeds` is clamped to `1..=NUM_SEEDS`; `seeds == NUM_SEEDS` is the full best-of.
// ---------------------------------------------------------------------------------------------
fn crunch_normal_int(buf: &[u8]) -> Vec<u8> {
    crunch_normal_int_seeds(buf, NUM_SEEDS)
}

fn crunch_normal_int_seeds(buf: &[u8], seeds: usize) -> Vec<u8> {
    // Always run at least the anchor; never more than the candidate set we have.
    let seeds = seeds.clamp(1, NUM_SEEDS);
    let prof = std::env::var("SUBSIZER_PROFILE").is_ok();
    let t0 = std::time::Instant::now();
    let mt = build_match(buf);
    if prof {
        eprintln!(
            "  build_match: {:?} ({} cands)",
            t0.elapsed(),
            mt.cands.len()
        );
    }
    let mut opt = EncOptimizer::new();

    // --- Candidate 0: the native trajectory (ANCHOR). Always valid, always <= native by def. ---
    let tc = std::time::Instant::now();
    let (anchor_pp, anchor_es) = run_native(&mt, buf, &mut opt);
    let mut best = generate(&anchor_pp, buf, &anchor_es);
    if prof {
        eprintln!("  anchor: {:?} -> {} B", tc.elapsed(), best.len());
    }

    // `extra` is the number of additional candidate trajectories to try past the anchor.
    let extra = seeds - 1;
    if extra == 0 {
        return best; // seeds == 1: anchor only (byte-identical to native `subsizer -r`).
    }

    // Helper: run a seed→converge trajectory, generate, and keep it if strictly smaller AND it
    // round-trips to `buf` (so we never emit an invalid stream).
    //
    // A candidate is only *encodable* if every token on its path - and the end marker - is
    // representable by its final EncodingSet.  When the Real loop is stopped mid-oscillation (it hit
    // the pass cap without `es == last_es`), the returned `pp` was parsed against the previous `es`
    // and a value it uses may no longer fit the final, possibly-narrower tables; upstream `write_enc`
    // would then silently emit a corrupt token (`of == -1`).  We instead detect that with `cost_enc`'s
    // `0x100000` fault sentinel and skip the candidate, so `generate` is only ever called on a fully
    // encodable pair (no panic, no corrupt output).  The final `decrunch == buf` check is the
    // belt-and-suspenders guarantee.
    let consider = |es: &mut EncodingSet, pp: PrimaryPath, best: &mut Vec<u8>| {
        if es.endm == 0 {
            return; // no valid end marker - skip (matches the native "couldn't find" error path).
        }
        if !path_encodable(&pp, es) {
            return; // pp/es inconsistent (stopped mid-oscillation) - would fault in generate.
        }
        let out = generate(&pp, buf, es);
        if out.len() < best.len() && decrunch_normal_int(&out) == buf {
            *best = out;
        }
    };

    // Each closure is one extra candidate trajectory; we run them in order and stop once `extra`
    // of them have been tried, so `seeds` linearly selects how hard the best-of searches.
    let mut tried = 0usize;
    let stage = |label: &str, es: &mut EncodingSet, pp: PrimaryPath, best: &mut Vec<u8>| {
        let t = std::time::Instant::now();
        let before = best.len();
        consider(es, pp, best);
        if prof {
            eprintln!(
                "  {label}: {:?} -> {} B (best {})",
                t.elapsed(),
                before,
                best.len()
            );
        }
    };

    // Every seed below is *table-safe*: it is either the converged anchor, a positively-perturbed
    // (widened) copy of it, or an EncodingSet derived from the Init cost model - never a zeroed or
    // narrow table, which would make the Real cost model emit `0x100000` fault costs and blow up
    // `optimize_enc`'s recursion.

    // --- Candidate 1: native start, but keep iterating past 16 passes (different fixed point if
    //     the 16-pass cap stopped before convergence). ---
    if tried < extra {
        let mut es = anchor_es.clone();
        let pp = converge(&mt, buf, &mut opt, &mut es, 30);
        stage("cand1-morepasses", &mut es, pp, &mut best);
        tried += 1;
    }

    // --- Candidate 2: greedy-bootstrap start (longest-match parse → Init histogram → iterate). ---
    if tried < extra {
        let gp = greedy_path(&mt, buf);
        let mut es = seed_es_from_path(&gp, buf, &mut opt);
        let pp = converge(&mt, buf, &mut opt, &mut es, 30);
        stage("cand2-greedy", &mut es, pp, &mut best);
        tried += 1;
    }

    // --- Candidate 3: all-literal start (neutral Init histogram seed). ---
    if tried < extra {
        let lp = literal_path(buf.len());
        let mut es = seed_es_from_path(&lp, buf, &mut opt);
        let pp = converge(&mt, buf, &mut opt, &mut es, 30);
        stage("cand3-literal", &mut es, pp, &mut best);
        tried += 1;
    }

    // --- Candidates 4a/4b: extra Init warm-up passes (2, 3) before the Real loop. The native uses
    //     exactly one; more warm-up nudges the starting histogram. ---
    for warmups in [2u32, 3u32] {
        if tried >= extra {
            break;
        }
        let mut es = EncodingSet::zeroed();
        let mut pp = find_cheapest_path(&mt, CostModel::Init, &es);
        optimize_encoding(&pp, buf, CostModel::Init, &mut es, &mut opt);
        for _ in 1..warmups {
            pp = find_cheapest_path(&mt, CostModel::Init, &es);
            optimize_encoding(&pp, buf, CostModel::Init, &mut es, &mut opt);
        }
        let _ = pp;
        let pp = converge(&mt, buf, &mut opt, &mut es, 30);
        stage(&format!("cand4-warmup{warmups}"), &mut es, pp, &mut best);
        tried += 1;
    }

    // --- Candidates 5..: perturb the converged anchor by widening every part width by +1/+2/+3,
    //     then re-converge.  Widening only adds coverage (never faults), so it is table-safe; it
    //     can knock the iteration into a neighbouring, sometimes smaller, fixed point. ---
    for delta in [1i32, 2i32, 3i32] {
        if tried >= extra {
            break;
        }
        let mut es = perturb_encoding_set(&anchor_es, delta);
        let pp = converge(&mt, buf, &mut opt, &mut es, 30);
        stage(&format!("cand5-widen{delta}"), &mut es, pp, &mut best);
        tried += 1;
    }

    best
}

/// The level-1 / seeds-1 anchor: the single native iteration trajectory, generated to a stream
/// byte-identical to `subsizer -r`. This is the FAST tier (one `run_native`, no best-of).
fn crunch_normal_anchor(buf: &[u8]) -> Vec<u8> {
    let mt = build_match(buf);
    let mut opt = EncOptimizer::new();
    let (pp, es) = run_native(&mt, buf, &mut opt);
    generate(&pp, buf, &es)
}

/// The anchor trajectory, generated with `BITMODE_PRESHIFT` (marker-bit reservoir) framing - the
/// raw memory-mode stream, byte-identical to the inner `crunch_normal_int(_, BITMODE_PRESHIFT)`
/// the native `crunch_normal_mem` produces.  Returns `(stream, endm)`: `stream` is the preshifted
/// crunched bytes (forward), `endm` the end-marker side byte the mem wrapper later adjusts.
fn crunch_normal_anchor_preshift(buf: &[u8]) -> (Vec<u8>, u8) {
    let mt = build_match(buf);
    let mut opt = EncOptimizer::new();
    let (pp, es) = run_native(&mt, buf, &mut opt);
    (generate_preshift(&pp, buf, &es), es.endm as u8)
}

/// Build the standalone memory-mode body for `input`, byte-identical to `subsizer -m` (sans the
/// 2-byte PRG load address the native `save_file_from_memory` prepends).
///
/// `dest_end` is the address one past the last decompressed byte (`smem->high` in the native): the
/// standalone 6502 decoder writes the output **backward** starting from `dest_end - 1` down to
/// `dest_end - input.len()`.  The native derives `dest_end` from the input PRG's load address +
/// length; here the caller supplies it (the harness sets it to `out_addr + input.len()`).
///
/// Layout produced (the memory-mode layout):
///   1. reverse `input`, crunch it with the `BITMODE_PRESHIFT` anchor → `stream` (`first` byte at
///      `stream[len-1]`, `endm` side byte at `stream[len-2]`),
///   2. reverse `stream`,
///   3. emit `stream[0..len-2]` then the 4-byte trailer `first, dest_lo, dest_hi, endm-1`.
///
/// The decoder reads this whole buffer **backward** via its `dc_get_byte`: the last 4 bytes are
/// the prologue (`endm-1 → endm_zp, dest_hi, dest_lo, first → buf_zp`), then the marker-framed bit
/// stream.
fn crunch_normal_marker_int(input: &[u8], dest_end: u16) -> Vec<u8> {
    // 1. crunch the reversed input with the preshift (marker-reservoir) framing.
    let mut rev = input.to_vec();
    rev.reverse();
    let (mut stream, endm) = crunch_normal_anchor_preshift(&rev);

    // 2. reverse the crunched stream (mem mode reverses the output).
    stream.reverse();

    // 3. pop `first` (last byte) and `endm` side byte (second-to-last), then append the 4-byte
    //    prologue/trailer the standalone decoder reads.  `endm` recovered from `stream[len-2]`
    //    equals the EncodingSet's `endm` (the side byte written first, now last after reversal).
    let len = stream.len();
    debug_assert!(
        len >= 2,
        "preshifted stream always has >= 2 bytes (endm + data)"
    );
    let first = stream[len - 1];
    let endm_byte = stream[len - 2];
    debug_assert_eq!(
        endm_byte, endm,
        "recovered endm side byte must match the EncodingSet endm"
    );

    let mut out = Vec::with_capacity(len + 2);
    out.extend_from_slice(&stream[..len - 2]);
    out.push(first);
    out.push((dest_end & 0xff) as u8);
    out.push((dest_end >> 8) as u8);
    out.push(endm_byte.wrapping_sub(1)); // endm - 1 (adjusted as in crunch_normal_mem)
    out
}

// ---------------------------------------------------------------------------------------------
// Bit reader for the pure-Rust decoder.  BITMODE_SIDEBYTE, no preshift.
// ---------------------------------------------------------------------------------------------
struct BitReader<'a> {
    ptr: &'a [u8],
    pos: usize,
    buf: u32,
    bit: i32,
}

impl<'a> BitReader<'a> {
    fn new(src: &'a [u8]) -> BitReader<'a> {
        BitReader {
            ptr: src,
            pos: 0,
            buf: 0,
            bit: 0,
        }
    }

    fn read(&mut self, n: i32) -> u32 {
        let mut n = n;
        let mut data: u32 = 0;
        while n > 0 {
            if self.bit == 0 {
                self.buf = *self.ptr.get(self.pos).unwrap_or(&0) as u32;
                self.pos += 1;
                self.bit = 8;
            }
            let nn = if n > self.bit { self.bit } else { n };
            data <<= nn;
            data |= self.buf >> (8 - nn);
            self.buf = (self.buf << nn) & 0xff;
            self.bit -= nn;
            n -= nn;
        }
        data
    }

    // bitrd_read8s with SIDEBYTE: read a whole byte at the current byte position.
    fn read8s(&mut self) -> u8 {
        let b = *self.ptr.get(self.pos).unwrap_or(&0);
        self.pos += 1;
        b
    }
}

/// `BITMODE_PRESHIFT` bit reader (memory/executable mode), used by the pure-Rust marker decoder
/// ([`decompress_subsizer_marker`]) to validate round-tripping.  Owns its buffer because the
/// preshift init mutates byte 0 (`ptr[0] >>= 1`) before popping the sentinel marker bit - a
/// `BITMODE_PRESHIFT` bit reader init.
struct MarkerBitReader {
    ptr: Vec<u8>,
    pos: usize,
    buf: u32,
    bit: i32,
}

impl MarkerBitReader {
    fn new(src: &[u8]) -> MarkerBitReader {
        let mut r = MarkerBitReader {
            ptr: src.to_vec(),
            pos: 0,
            buf: 0,
            bit: 0,
        };
        // bitrd_init PRESHIFT: tmp = ptr[0]; ptr[0] >>= 1; read(1); ptr[0] = tmp.
        if !r.ptr.is_empty() {
            let tmp = r.ptr[0];
            r.ptr[0] >>= 1;
            r.read(1);
            r.ptr[0] = tmp;
        }
        r
    }
    fn read(&mut self, n: i32) -> u32 {
        let mut n = n;
        let mut data: u32 = 0;
        while n > 0 {
            if self.bit == 0 {
                self.buf = *self.ptr.get(self.pos).unwrap_or(&0) as u32;
                self.pos += 1;
                self.bit = 8;
            }
            let nn = if n > self.bit { self.bit } else { n };
            data <<= nn;
            data |= self.buf >> (8 - nn);
            self.buf = (self.buf << nn) & 0xff;
            self.bit -= nn;
            n -= nn;
        }
        data
    }
    fn read8s(&mut self) -> u8 {
        let b = *self.ptr.get(self.pos).unwrap_or(&0);
        self.pos += 1;
        b
    }
}

// read_unary / read_enc / read_mlen / read_moffs
fn read_unary(br: &mut BitReader, lim: i32) -> i32 {
    // pol == 0
    let mut n = 0;
    while br.read(1) == 0 {
        n += 1;
        if n == lim - 1 {
            break;
        }
    }
    n
}

fn read_enc(br: &mut BitReader, enc: &Encoding) -> i32 {
    let of = match enc.prefix {
        Prefix::Binary => br.read(ceil_log2(enc.n as i64) as i32) as i32,
        Prefix::Unary => read_unary(br, enc.n as i32),
    };
    let mut base = enc.floor;
    for i in 0..(of as usize) {
        base += 1 << enc.parts[i];
    }
    base + br.read(enc.parts[of as usize] as i32) as i32
}

fn read_mlen(br: &mut BitReader, es: &EncodingSet) -> i32 {
    read_enc(br, &es.bitsl)
}
fn read_moffs(br: &mut BitReader, es: &EncodingSet, l: i32) -> i32 {
    match l {
        1 => read_enc(br, &es.bits1),
        2 => read_enc(br, &es.bits2),
        3 => read_enc(br, &es.bits3),
        _ => read_enc(br, &es.bits),
    }
}

// ---------------------------------------------------------------------------------------------
// decrunch_normal_int, pure-Rust forward decoder.
// ---------------------------------------------------------------------------------------------
fn decrunch_normal_int(src: &[u8]) -> Vec<u8> {
    decrunch_normal_int_with_gap(src).0
}

/// Like [`decrunch_normal_int`], plus the in-place safety gap (bytes): the peak
/// of `output_produced - input_consumed` over the decode minus its final value.
/// `br.pos` is the input byte position. See [`max_gap_forward`].
fn decrunch_normal_int_with_gap(src: &[u8]) -> (Vec<u8>, i32) {
    let mut br = BitReader::new(src);

    let mut es = EncodingSet::zeroed();
    es.endm = br.read8s() as i32;

    es.bitsl.floor = MIN_MATCH;
    es.bitsl.n = LEN_PARTS;
    es.bitsl.prefix = Prefix::Unary;
    for i in 0..es.bitsl.n {
        es.bitsl.parts[i] = br.read(4) as u8;
    }

    es.bits2.floor = 1;
    es.bits2.n = TWO_BYTE_PARTS;
    es.bits2.prefix = Prefix::Binary;
    for i in 0..es.bits2.n {
        es.bits2.parts[i] = br.read(4) as u8;
    }

    es.bits3.floor = 1;
    es.bits3.n = THREE_BYTE_PARTS;
    es.bits3.prefix = Prefix::Binary;
    for i in 0..es.bits3.n {
        es.bits3.parts[i] = br.read(4) as u8;
    }

    es.bits.floor = 1;
    es.bits.n = LONG_MATCH_PARTS;
    es.bits.prefix = Prefix::Binary;
    for i in 0..es.bits.n {
        es.bits.parts[i] = br.read(4) as u8;
    }

    es.bits1.floor = 1;
    es.bits1.n = SINGLE_BYTE_PARTS;
    es.bits1.prefix = Prefix::Binary;
    for i in 0..es.bits1.n {
        es.bits1.parts[i] = br.read(4) as u8;
    }

    let mut out: Vec<u8> = Vec::new();
    let mut max_gap = 0i32;
    loop {
        let gap = out.len() as i32 - br.pos as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        if br.read(1) != 0 {
            let c = br.read8s();
            out.push(c);
        } else {
            let len = read_mlen(&mut br, &es);
            if len == es.endm {
                break;
            }
            let offs = read_moffs(&mut br, &es, len);
            let cur = out.len() as i32;
            if offs > cur {
                // offset out of range, corrupt stream; stop
                break;
            }
            for _ in 0..len {
                let b = out[(out.len() as i32 - offs) as usize];
                out.push(b);
            }
        }
    }

    let final_gap = out.len() as i32 - src.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

// ---------------------------------------------------------------------------------------------
// decrunch_preshift_int - pure-Rust decoder for the BITMODE_PRESHIFT (marker reservoir) stream.
// Identical token grammar to decrunch_normal_int, only the bit reader framing differs.
// ---------------------------------------------------------------------------------------------
fn decrunch_preshift_int(dbf: &[u8]) -> Vec<u8> {
    decrunch_preshift_int_with_gap(dbf).0
}

/// Like [`decrunch_preshift_int`], plus the in-place safety gap (bytes). The
/// 6502 backward decoder reads the stored stream top-down, which reproduces this
/// forward read of `dbf`, so `out.len() - br.pos` is the write-vs-read gap (the
/// reconstruction's constant byte offset cancels in `max - final`).
fn decrunch_preshift_int_with_gap(dbf: &[u8]) -> (Vec<u8>, i32) {
    let mut br = MarkerBitReader::new(dbf);

    let mut es = EncodingSet::zeroed();
    es.endm = br.read8s() as i32;

    es.bitsl.floor = MIN_MATCH;
    es.bitsl.n = LEN_PARTS;
    es.bitsl.prefix = Prefix::Unary;
    for i in 0..es.bitsl.n {
        es.bitsl.parts[i] = br.read(4) as u8;
    }
    es.bits2.floor = 1;
    es.bits2.n = TWO_BYTE_PARTS;
    es.bits2.prefix = Prefix::Binary;
    for i in 0..es.bits2.n {
        es.bits2.parts[i] = br.read(4) as u8;
    }
    es.bits3.floor = 1;
    es.bits3.n = THREE_BYTE_PARTS;
    es.bits3.prefix = Prefix::Binary;
    for i in 0..es.bits3.n {
        es.bits3.parts[i] = br.read(4) as u8;
    }
    es.bits.floor = 1;
    es.bits.n = LONG_MATCH_PARTS;
    es.bits.prefix = Prefix::Binary;
    for i in 0..es.bits.n {
        es.bits.parts[i] = br.read(4) as u8;
    }
    es.bits1.floor = 1;
    es.bits1.n = SINGLE_BYTE_PARTS;
    es.bits1.prefix = Prefix::Binary;
    for i in 0..es.bits1.n {
        es.bits1.parts[i] = br.read(4) as u8;
    }

    let mut out: Vec<u8> = Vec::new();
    let mut max_gap = 0i32;
    loop {
        let gap = out.len() as i32 - br.pos as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        if br.read(1) != 0 {
            let c = br.read8s();
            out.push(c);
        } else {
            let len = read_enc_m(&mut br, &es.bitsl);
            if len == es.endm {
                break;
            }
            let offs = read_moffs_m(&mut br, &es, len);
            let cur = out.len() as i32;
            if offs > cur {
                break;
            }
            for _ in 0..len {
                let b = out[(out.len() as i32 - offs) as usize];
                out.push(b);
            }
        }
    }
    let final_gap = out.len() as i32 - dbf.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

// MarkerBitReader variants of read_unary / read_enc / read_moffs (sidebyte framing differs only in
// the underlying reader; the grammar is identical to the non-preshift path).
fn read_unary_m(br: &mut MarkerBitReader, lim: i32) -> i32 {
    let mut n = 0;
    while br.read(1) == 0 {
        n += 1;
        if n == lim - 1 {
            break;
        }
    }
    n
}
fn read_enc_m(br: &mut MarkerBitReader, enc: &Encoding) -> i32 {
    let of = match enc.prefix {
        Prefix::Binary => br.read(ceil_log2(enc.n as i64) as i32) as i32,
        Prefix::Unary => read_unary_m(br, enc.n as i32),
    };
    let mut base = enc.floor;
    for i in 0..(of as usize) {
        base += 1 << enc.parts[i];
    }
    base + br.read(enc.parts[of as usize] as i32) as i32
}
fn read_moffs_m(br: &mut MarkerBitReader, es: &EncodingSet, l: i32) -> i32 {
    match l {
        1 => read_enc_m(br, &es.bits1),
        2 => read_enc_m(br, &es.bits2),
        3 => read_enc_m(br, &es.bits3),
        _ => read_enc_m(br, &es.bits),
    }
}

// ---------------------------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------------------------

/// Compress `input` to a raw forward Subsizer stream, byte-identical to `subsizer -r input`.
pub fn compress_subsizer(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        // The reference rejects empty input (no end marker possible); return an empty stream so
        // the uniform API is total.  decompress() returns empty for empty input.
        return Vec::new();
    }
    crunch_normal_int(input)
}

/// Compress `input` to a raw backward Subsizer stream, byte-identical to `subsizer -r -b input`.
pub fn compress_subsizer_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut rev = input.to_vec();
    rev.reverse();
    let mut out = crunch_normal_int(&rev);
    out.reverse();
    out
}

/// Compress `input` into the standalone **memory-mode** stream decodable by the
/// `subsizer-tlr-standalone` 6502 decruncher.
///
/// This is the marker-bit-reservoir framing (`BITMODE_PRESHIFT`) plus the 4-byte prologue the
/// standalone reads, and the data is emitted so the decoder writes its output **backward**.  It is
/// byte-identical to `subsizer -m` (minus the 2-byte PRG load address `save_file_from_memory`
/// prepends).  See [`crunch_normal_marker_int`] for the exact layout.
///
/// `dest_end` is the address one past the last decompressed byte: the decoder writes `input.len()`
/// bytes downward, ending just below `dest_end` (i.e. the output occupies
/// `[dest_end - input.len() .. dest_end)`).  To match native `subsizer -m` on a PRG with load
/// address `la`, pass `dest_end = la + input.len()`.
pub fn compress_subsizer_marker_at(input: &[u8], dest_end: u16) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    crunch_normal_marker_int(input, dest_end)
}

/// Compress `input` into the standalone memory-mode stream (see [`compress_subsizer_marker_at`]),
/// using `dest_end = input.len()` - i.e. the decompressed data is positioned as if loaded at
/// address `0`, so the decoder writes it backward into `[0 .. input.len())`.  The harness fixes up
/// the 2-byte dest in the prologue for its actual output address.
pub fn compress_subsizer_marker(input: &[u8]) -> Vec<u8> {
    compress_subsizer_marker_at(input, input.len() as u16)
}

/// Pure-Rust decoder for the standalone memory-mode stream produced by
/// [`compress_subsizer_marker_at`] / [`compress_subsizer_marker`], a model of the
/// `subsizer-tlr-standalone` 6502 decruncher.  Reads the 4-byte prologue (backward from
/// the end), de-frames the `BITMODE_PRESHIFT` (marker reservoir) bitstream, decodes it, and
/// reverses the result (the encoder crunched the reversed input, so the decoder un-reverses).
/// Returns the original `input`.  `expected_len` is required because the stream encodes a backward
/// length only implicitly via the end marker; we decode until the marker and trust the token count.
pub fn decompress_subsizer_marker(stream: &[u8]) -> Vec<u8> {
    if stream.len() < 4 {
        return Vec::new();
    }
    // Un-wrap the 4-byte prologue/trailer to recover the preshift `dbf` (forward).
    //   stream = dbf_rev[0..n-2] + first + dest_lo + dest_hi + (endm-1)
    // so dbf_rev = stream[0..n-4] ++ [endm_byte, first], then dbf = reverse(dbf_rev).
    let n = stream.len();
    let first = stream[n - 4];
    let endm_byte = stream[n - 1].wrapping_add(1);
    let mut dbf_rev: Vec<u8> = stream[..n - 4].to_vec();
    dbf_rev.push(endm_byte);
    dbf_rev.push(first);
    let mut dbf = dbf_rev;
    dbf.reverse();

    // Decode the preshift stream (gives the reversed input), then reverse back.
    let mut out = decrunch_preshift_int(&dbf);
    out.reverse();
    out
}

/// Uniform API entry: compress `input` at `level` (clamped to 1..=[`MAX_LEVEL`]).
///
/// * **level 1** - FAST anchor: the single native iteration trajectory
///   ([`crunch_normal_anchor`]), byte-identical to `subsizer -r`.
/// * **level 2** - BEST: the multi-seed best-of ([`crunch_normal_int`]), `<=` level 1 on every
///   input (the anchor is always one of its candidates).
///
/// `backward` mirrors `subsizer -r -b`: reverse input, compress forward, reverse output.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let level = level.clamp(1, MAX_LEVEL);
    if input.is_empty() {
        // The reference rejects empty input (no end marker possible); the uniform API stays total.
        return Vec::new();
    }
    let crunch = |data: &[u8]| -> Vec<u8> {
        if level == 1 {
            crunch_normal_anchor(data)
        } else {
            crunch_normal_int(data)
        }
    };
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = crunch(&rev);
        out.reverse();
        out
    } else {
        crunch(input)
    }
}

/// Native API: run the best-of over exactly `seeds` seed trajectories (clamped to 1..=[`NUM_SEEDS`]).
/// `seeds == 1` ⇒ the anchor only (byte-identical to `subsizer -r`); larger values add extra
/// trajectories. The anchor is always included, so the output is always `<=` native `subsizer -r`.
///
/// `backward` mirrors `subsizer -r -b`: reverse input, compress forward, reverse output.
pub fn compress_native(input: &[u8], seeds: usize, backward: bool) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = crunch_normal_int_seeds(&rev, seeds);
        out.reverse();
        out
    } else {
        crunch_normal_int_seeds(input, seeds)
    }
}

/// Decompress a forward Subsizer stream (pure Rust; byte-identical decode of `subsizer -r`).
pub fn decompress_subsizer(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    decrunch_normal_int(input)
}

/// Decompress a backward Subsizer stream (`subsizer -r -b` output): reverse, decode, reverse.
pub fn decompress_subsizer_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut rev = input.to_vec();
    rev.reverse();
    let mut out = decrunch_normal_int(&rev);
    out.reverse();
    out
}

/// In-place safety margin (bytes) for a FORWARD subsizer stream - the peak by
/// which the compressed stream is momentarily larger than the output produced.
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    decrunch_normal_int_with_gap(stream).1.max(0) as usize
}

/// In-place safety margin (bytes) for a BACKWARD subsizer stream (the `-r -b`
/// marker / standalone format from [`compress_subsizer_marker_at`]). The 6502
/// decoder reads the stored stream top-down; reconstruct the forward preshift
/// `dbf` exactly as [`decompress_subsizer_marker`] does, then measure the gap.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.len() < 4 {
        return 0;
    }
    let n = stream.len();
    let first = stream[n - 4];
    let endm_byte = stream[n - 1].wrapping_add(1);
    let mut dbf_rev: Vec<u8> = stream[..n - 4].to_vec();
    dbf_rev.push(endm_byte);
    dbf_rev.push(first);
    let mut dbf = dbf_rev;
    dbf.reverse();
    decrunch_preshift_int_with_gap(&dbf).1.max(0) as usize
}

/// Uniform API entry: pure-Rust decoder, forward or backward.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        decompress_subsizer_backward(input)
    } else {
        decompress_subsizer(input)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(data: &[u8]) {
        // forward
        let c = compress_subsizer(data);
        let d = decompress_subsizer(&c);
        assert_eq!(d, data, "subsizer forward roundtrip len {}", data.len());
        // backward
        let cb = compress_subsizer_backward(data);
        let db = decompress_subsizer_backward(&cb);
        assert_eq!(db, data, "subsizer backward roundtrip len {}", data.len());

        // marker (standalone memory-mode) roundtrip: compress -> pure-Rust marker decode == data.
        // (The marker path runs the anchor pipeline once; for very large inputs the second,
        // dest_end-parameterised variant is redundant - the prologue dest never affects the token
        // stream - so we only exercise it on smaller inputs to keep the test cheap.)
        if !data.is_empty() {
            let cm = compress_subsizer_marker(data);
            let dm = decompress_subsizer_marker(&cm);
            assert_eq!(dm, data, "subsizer marker roundtrip len {}", data.len());
            if data.len() <= 8192 {
                // dest_end-parameterised variant must round-trip identically (the prologue dest is
                // data the decoder reads but never affects the token stream).
                let cm2 =
                    compress_subsizer_marker_at(data, 0x4000u16.wrapping_add(data.len() as u16));
                assert_eq!(
                    decompress_subsizer_marker(&cm2),
                    data,
                    "subsizer marker_at roundtrip len {}",
                    data.len()
                );
            }
        }

        // uniform API: both levels roundtrip both directions
        for level in 1..=MAX_LEVEL {
            assert_eq!(
                decompress(&compress(data, level, false), false),
                data,
                "uniform fwd L{level}"
            );
            assert_eq!(
                decompress(&compress(data, level, true), true),
                data,
                "uniform bwd L{level}"
            );
        }

        // level 1 == the native anchor (== seeds=1); level 2 == the full best-of (== subsizer).
        assert_eq!(
            compress(data, 1, false),
            compress_native(data, 1, false),
            "L1 == anchor"
        );
        assert_eq!(
            compress(data, 2, false),
            compress_subsizer(data),
            "L2 == best-of"
        );
        // best-of (L2) is never larger than the anchor (L1).
        assert!(
            compress(data, 2, false).len() <= compress(data, 1, false).len(),
            "L2 <= L1 len {}",
            data.len()
        );
        // clamp: 0 -> 1, 255 -> MAX_LEVEL
        assert_eq!(compress(data, 0, false), compress(data, 1, false));
        assert_eq!(compress(data, 255, false), compress(data, MAX_LEVEL, false));

        // native API: seeds=1 is the anchor; full seeds equals the best-of; both roundtrip.
        assert_eq!(
            decompress(&compress_native(data, 1, false), false),
            data,
            "native seeds=1 fwd"
        );
        assert_eq!(
            decompress(&compress_native(data, NUM_SEEDS, true), true),
            data,
            "native full bwd"
        );
        assert_eq!(
            compress_native(data, NUM_SEEDS, false),
            compress(data, 2, false),
            "native full == L2"
        );
    }

    #[test]
    fn roundtrip_basic() {
        rt(&[]);
        rt(&[0]);
        rt(&[42]);
        rt(&[1, 2, 3, 4, 5]);
        rt(b"abracadabra abracadabra abracadabra");
        rt(b"hello world hello world hello world hello world");
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        // Incompressible data overruns the fixed 32-byte in-place margin.
        assert!(max_gap_forward(&compress_subsizer(&noise)) > 32);
        assert!(max_gap_backward(&compress_subsizer_marker(&noise)) > 32);
        // Highly compressible data stays within it.
        assert!(max_gap_backward(&compress_subsizer_marker(&vec![0u8; 8192])) <= 32);
    }

    #[test]
    fn roundtrip_rle_and_repeats() {
        let mut v = Vec::new();
        for _ in 0..2000 {
            v.extend_from_slice(b"The quick brown fox. ");
        }
        rt(&v);
        rt(&vec![0xAA; 5000]);
        let mut z = vec![0u8; 1000];
        for (i, b) in z.iter_mut().enumerate() {
            *b = (i % 7) as u8;
        }
        rt(&z);
    }

    #[test]
    fn roundtrip_pseudo_random() {
        // deterministic LCG, mildly compressible
        let mut s: u32 = 0x1234_5678;
        let mut v = Vec::with_capacity(4096);
        for _ in 0..4096 {
            s = s.wrapping_mul(1103515245).wrapping_add(12345);
            v.push(((s >> 16) & 0xff) as u8);
        }
        rt(&v);
    }
}
