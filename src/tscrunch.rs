//! TSCrunch cruncher and decruncher.
//!
//! TSCrunch is a C64 binary cruncher with an optimal (Dijkstra shortest-path) parse over a token
//! graph. The encoder produces output byte-identical to `tscrunch -p -q` (forward) and
//! `tscrunch -p -i -q` (in-place). The decoder is pure Rust for both the forward and in-place
//! layouts.
//!
//! A few behaviours follow the Go build of the reference tool where the Go and C sources differ:
//! the LZ match search uses the prefix-array path (`findall_into`/`lz_best`), and a ZERORUN may
//! end exactly at the buffer end (`zerorun_at`). The integer cost model and graph construction are
//! common to both.
//!
//! ## Token format (1 token = 1 graph edge)
//!
//! | token    | first byte                                  | extra            | meaning                       |
//! |----------|---------------------------------------------|------------------|-------------------------------|
//! | LITERAL  | `0x00 \| size` (1..=31)                      | `size` raw bytes | copy `size` literal bytes     |
//! | LZ2      | `0x00 \| (127 - offset)` (offset 1..=94)     | -                | 2-byte match, short offset    |
//! | RLE      | `0x81 \| ((size-1)<<1)` (size 2..=64)        | rle byte         | run of `rlebyte`              |
//! | ZERORUN  | `0x81` (the `0x7e` field is zero)            | -                | run of `optimalRun` zeros     |
//! | LZ short | `0x80 \| ((size-1)<<2) \| 2` (size 3..=32)    | offset lo byte   | match, offset 1..=255         |
//! | LZ long  | `0x80 \| (((size-1)>>1)<<2)`                 | neg lo, neg hi   | match, long offset/length     |
//!
//! The stream is `optimalRun-1`, then the tokens, then `TERMINATOR` (0x20). In-place mode wraps it
//! with a 2-byte load address, the `optimalRun-1` byte, a remainder byte, the (possibly truncated)
//! token stream, the terminator, and the literal remainder tail. See `compress`/`decompress`.

// --- format constants ---
const LONGEST_RLE: i32 = 64;
const LONGEST_LONG_LZ: i32 = 64;
const LONGEST_LZ: i32 = 32;
const LONGEST_LITERAL: i32 = 31;
const MIN_RLE: i32 = 2;
const MIN_LZ: i32 = 3;
const LZ_OFFSET: i32 = 256;
const LONG_LZ_OFFSET: i32 = 32767;
const LZ2_OFFSET: i32 = 94;
const LZ2_SIZE: i32 = 2;

const RLEMASK: u8 = 0x81;
const LZMASK: u8 = 0x80;
const LITERALMASK: u8 = 0x00;
const LZ2MASK: u8 = 0x00;

const TERMINATOR: u8 = (LONGEST_LITERAL + 1) as u8; // 0x20

/// Two compression tiers. Level 1 = the byte-identical `tscrunch -p` anchor (fastest, the
/// native-quality baseline); level 2 = the rich-matchfinder best-of (smallest, never larger than
/// level 1).
pub const MAX_LEVEL: u8 = 2;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TokenType {
    Literal,
    Rle,
    Lz,
    Lz2,
    Zerorun,
}

#[derive(Clone, Copy, Debug)]
struct Token {
    ttype: TokenType,
    pos: i32,
    size: i32,
    offset: i32,
    rlebyte: u8,
}

impl Token {
    fn empty() -> Token {
        Token {
            ttype: TokenType::Lz,
            pos: 0,
            size: 0,
            offset: 0,
            rlebyte: 0,
        }
    }
}

#[inline]
fn lz_is_long(t: &Token) -> bool {
    t.offset >= LZ_OFFSET || t.size > LONGEST_LZ
}

/// Integer-scaled edge cost (no floats, so the
/// Dijkstra path is deterministic and byte-identical to the reference).
#[inline]
fn token_cost(t: &Token) -> i64 {
    let mdiv: i64 = LONGEST_LITERAL as i64 * 65536;
    let size = t.size as i64;
    match t.ttype {
        TokenType::Lz => {
            if lz_is_long(t) {
                mdiv * 3 + 138 - size
            } else {
                mdiv * 2 + 134 - size
            }
        }
        TokenType::Rle => mdiv * 2 + 128 - size,
        TokenType::Zerorun => mdiv,
        TokenType::Lz2 => mdiv + 132 - size,
        TokenType::Literal => mdiv * (size + 1) + 130 - size,
    }
}

/// Encoded payload length in bytes (matches `payload_len`).
#[inline]
fn payload_len(t: &Token) -> i32 {
    match t.ttype {
        TokenType::Literal => 1 + t.size,
        TokenType::Rle => 2,
        TokenType::Zerorun => 1,
        TokenType::Lz2 => 1,
        TokenType::Lz => {
            if lz_is_long(t) {
                3
            } else {
                2
            }
        }
    }
}

/// Pick the optimal zero-run length (matches `find_optimal_zero`). Scores each candidate run by
/// `run * count^1.1`, breaking ties by first-seen order (lowest wins).
fn find_optimal_zero(src: &[u8]) -> i32 {
    let len = src.len() as i32;
    let mut counts = [0i32; 257];
    let mut first_seen = [-1i32; 257];
    let mut order = 0i32;
    let mut i = 0i32;

    while i < len - 1 {
        if src[i as usize] == 0 {
            let mut j = i + 1;
            while j < len && src[j as usize] == 0 && (j - i) < 256 {
                j += 1;
            }
            let run = j - i;
            if run >= MIN_RLE && run <= 256 {
                if first_seen[run as usize] < 0 {
                    first_seen[run as usize] = order;
                    order += 1;
                }
                counts[run as usize] += 1;
            }
            i = j;
        } else {
            i += 1;
        }
    }

    let mut best_run = LONGEST_RLE;
    let mut best_score = 0.0f64;
    let mut best_first = i32::MAX;
    let mut run = MIN_RLE;
    while run <= 256 {
        if counts[run as usize] > 0 {
            let score = (run as f64) * (counts[run as usize] as f64).powf(1.1);
            if score > best_score
                || (score == best_score
                    && first_seen[run as usize] >= 0
                    && first_seen[run as usize] < best_first)
            {
                best_score = score;
                best_run = run;
                best_first = first_seen[run as usize];
            }
        }
        run += 1;
    }
    best_run
}

/// Run length of identical bytes at `pos`, capped at LONGEST_RLE+1 (matches `rle_length`).
#[inline]
fn rle_length(src: &[u8], pos: i32) -> i32 {
    let len = src.len() as i32;
    let mut x = 0i32;
    while pos + x < len && x < LONGEST_RLE + 1 && src[(pos + x) as usize] == src[pos as usize] {
        x += 1;
    }
    x
}

/// LZ2 (2-byte) nearest match offset within LZ2_OFFSET, or -1 (matches `lz2_offset`).
#[inline]
fn lz2_offset(src: &[u8], pos: i32) -> i32 {
    let len = src.len() as i32;
    if pos + LZ2_SIZE >= len {
        return -1;
    }
    let mut start = pos - LZ2_OFFSET;
    if start < 0 {
        start = 0;
    }
    let mut j = pos - 1;
    while j >= start {
        if src[j as usize] == src[pos as usize] && src[(j + 1) as usize] == src[(pos + 1) as usize]
        {
            return pos - j;
        }
        j -= 1;
    }
    -1
}

/// Prefix array: every position of each 3-byte (MINLZ) prefix, in ascending order. Mirrors the Go
/// reference's `fillPrefixArray` (`usePrefixArray = true` is the Go default, and the reference
/// binary is the Go build - so byte-identity requires replicating the prefix-array search, not the
/// C brute-force `rfind`).
struct PrefixArray {
    map: std::collections::HashMap<[u8; 3], Vec<i32>>,
}

impl PrefixArray {
    fn build(data: &[u8]) -> PrefixArray {
        let mut map: std::collections::HashMap<[u8; 3], Vec<i32>> =
            std::collections::HashMap::new();
        let n = data.len() as i32;
        let mut i = 0i32;
        // Go: for i in 0..len(data)-MINLZ  (note: strict, so the last MINLZ-1 positions are excluded)
        while i < n - MIN_LZ {
            let key = [
                data[i as usize],
                data[(i + 1) as usize],
                data[(i + 2) as usize],
            ];
            map.entry(key).or_default().push(i);
            i += 1;
        }
        PrefixArray { map }
    }
}

/// Yield candidate match positions for the 3-byte prefix at `pos`, in the exact order the Go
/// `findall` prefix-array path yields them (binary-search to a landing index near `pos`, then walk
/// downward while `> x0`). Replicates the Go logic - including its landing-index behavior - so the
/// downstream best-match selection is byte-identical.
fn findall_into(pa: &PrefixArray, data: &[u8], pos: i32, minlz: i32, out: &mut Vec<i32>) {
    out.clear();
    let len = data.len() as i32;
    // Go guard: len(prefix) < MINLZ || len(data)==0 || minlz < MINLZ || i >= len(data)
    if minlz < MIN_LZ || len == 0 || pos >= len {
        return;
    }
    // prefix = data[pos..pos+minlz]; key = first MINLZ bytes (zero-filled if short, but pos+MINLZ<=len here)
    if pos + MIN_LZ > len {
        return;
    }
    let key = [
        data[pos as usize],
        data[(pos + 1) as usize],
        data[(pos + 2) as usize],
    ];
    let parray = match pa.map.get(&key) {
        Some(v) => v,
        None => return,
    };
    if parray.is_empty() {
        return;
    }
    let x0 = (pos - LONG_LZ_OFFSET).max(0);

    // Binary search (verbatim from Go) to land `mid` near `pos`.
    let mut l = 0i32;
    let mut h = parray.len() as i32 - 1;
    let mut mid = 0i32;
    while l < h {
        mid = (h + l) >> 1;
        let pv = parray[mid as usize];
        if pv < pos {
            l = mid + 1;
        } else if pv > pos {
            h = mid - 1;
        } else {
            h = mid;
            l = mid;
        }
    }

    // Walk downward from `mid` while parray[o] > x0, yielding qualifying positions.
    let prefix = &data[pos as usize..(pos + minlz) as usize];
    let mut o = mid;
    while o >= 0 && o < parray.len() as i32 && parray[o as usize] > x0 {
        let p = parray[o as usize];
        if p < pos && p + minlz <= len && &data[p as usize..(p + minlz) as usize] == prefix {
            out.push(p);
        }
        o -= 1;
    }
}

/// Best LZ match at `pos` with minimum length `minlz` (ports the Go `LZ` constructor). Uses the
/// prefix-array `findall` to enumerate candidate positions, then applies the reference acceptance
/// test to pick `(bestpos, bestlen)`.
fn lz_best(pa: &PrefixArray, src: &[u8], pos: i32, minlz: i32, scratch: &mut Vec<i32>) -> Token {
    let len = src.len() as i32;
    let mut t = Token {
        ttype: TokenType::Lz,
        pos,
        size: 0,
        offset: 0,
        rlebyte: 0,
    };

    let mut bestpos = pos - 1;
    let mut bestlen = 0i32;

    // Go: if i+minlz <= len(src) { prefixes := findall(...) ... }
    if pos + minlz <= len {
        findall_into(pa, src, pos, minlz, scratch);
        for &j in scratch.iter() {
            let mut l = minlz;
            while pos + l < len
                && l < LONGEST_LONG_LZ
                && src[(j + l) as usize] == src[(pos + l) as usize]
            {
                l += 1;
            }
            if (l > bestlen
                && (pos - j < LZ_OFFSET || pos - bestpos >= LZ_OFFSET || l > LONGEST_LZ))
                || (l > bestlen + 1)
            {
                bestpos = j;
                bestlen = l;
            }
        }
    }

    t.size = bestlen;
    t.offset = pos - bestpos;
    t
}

/// Richer LZ enumeration: for position `pos`, fill `best_off[len]` with the CLOSEST source offset
/// (smallest offset = cheapest, since a sub-`LZ_OFFSET` offset is a 2-byte short LZ while a larger
/// one is a 3-byte long LZ) that achieves a match of at least length `len`, for every length in
/// `minlz..=cap`. Lengths with no match keep offset 0.
///
/// The native `lz_best`/`findall` path binary-searches to a landing index then walks DOWNWARD only,
/// keeps a single best (longest) match, and reuses that one offset for *every* shorter length - so
/// a shorter length whose cheapest (near, short-LZ) offset differs from the long match's offset is
/// encoded with the wrong, dearer offset. This enumerates the full candidate list for the 3-byte
/// prefix and records the closest offset per length, exposing those cheaper short-LZ edges to the
/// optimal parse. `best_off[len]` is the minimal offset, so `lz_is_long` picks the short encoding
/// whenever any near source of that length exists.
fn lz_per_length(
    pa: &PrefixArray,
    src: &[u8],
    pos: i32,
    minlz: i32,
    cap: i32,
    best_off: &mut [i32; 257],
) -> i32 {
    let len = src.len() as i32;
    for v in best_off.iter_mut() {
        *v = 0;
    }
    if minlz < MIN_LZ || pos + MIN_LZ > len {
        return 0;
    }
    let key = [
        src[pos as usize],
        src[(pos + 1) as usize],
        src[(pos + 2) as usize],
    ];
    let parray = match pa.map.get(&key) {
        Some(v) => v,
        None => return 0,
    };
    let x0 = (pos - LONG_LZ_OFFSET).max(0);
    let cap = cap.min(LONGEST_LONG_LZ);
    let mut max_len = 0i32;

    // Binary-search the landing index = largest index whose value is < pos (parray is strictly
    // ascending). Without this we would linearly skip every occurrence above `pos`, which is
    // O(occurrences) per call and quadratic on text with hot 3-byte prefixes.
    let n = parray.len() as i32;
    let mut lo = 0i32;
    let mut hi = n; // first index with parray[idx] >= pos
    while lo < hi {
        let mid = (lo + hi) >> 1;
        if parray[mid as usize] < pos {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    let mut idx = lo - 1; // largest index with parray[idx] < pos (closest source)

    // Walk downward (closest first). Closest-first means the first source to reach a length owns the
    // smallest offset for it, so we only fill not-yet-filled lengths. Stop once every length in
    // minlz..=cap is filled (no farther source can improve an already-filled, cheaper length).
    let want = (cap - minlz + 1).max(0);
    let mut filled = 0i32;
    while idx >= 0 {
        let j = parray[idx as usize];
        idx -= 1;
        if j <= x0 {
            break; // remaining entries are even farther (smaller j) - all out of window
        }
        // LCP between src[pos..] and src[j..], capped at `cap`.
        let mut l = 0i32;
        while l < cap && pos + l < len && src[(j + l) as usize] == src[(pos + l) as usize] {
            l += 1;
        }
        if l < minlz {
            continue;
        }
        let offset = pos - j;
        let mut len_i = l;
        while len_i >= minlz && best_off[len_i as usize] == 0 {
            best_off[len_i as usize] = offset;
            len_i -= 1;
            filled += 1;
        }
        if l > max_len {
            max_len = l;
        }
        if filled >= want {
            break;
        }
    }
    max_len
}

/// True if there is a run of `run` zeros starting at `pos` (matches `zerorun_at`).
#[inline]
fn zerorun_at(src: &[u8], pos: i32, run: i32) -> bool {
    let len = src.len() as i32;
    if run <= 0 {
        return false;
    }
    // Go ZERORUN: `for x = 0; x < optimalRun && i+x < len && src[i+x]==0; x++`; valid iff x reaches
    // optimalRun. This permits the zero run to END exactly at the buffer end (the C reference used a
    // stricter `pos+run < len`, which is wrong for the Go-built reference binary).
    let mut x = 0i32;
    while x < run && pos + x < len && src[(pos + x) as usize] == 0 {
        x += 1;
    }
    x == run
}

/// Append the encoded payload of one token to `out` (matches `emit_token`).
fn emit_token(out: &mut Vec<u8>, src: &[u8], t: &Token) {
    match t.ttype {
        TokenType::Literal => {
            out.push(LITERALMASK | (t.size as u8 & 0x1f));
            out.extend_from_slice(&src[t.pos as usize..(t.pos + t.size) as usize]);
        }
        TokenType::Rle => {
            out.push(RLEMASK | ((((t.size - 1) << 1) as u8) & 0x7f));
            out.push(t.rlebyte);
        }
        TokenType::Zerorun => {
            out.push(RLEMASK);
        }
        TokenType::Lz2 => {
            out.push(LZ2MASK | ((127 - t.offset) as u8));
        }
        TokenType::Lz => {
            if lz_is_long(t) {
                let neg = (0u32.wrapping_sub(t.offset as u32)) as u16;
                out.push(LZMASK | (((((t.size - 1) >> 1) << 2) as u8) & 0x7f));
                out.push((neg & 0xff) as u8);
                out.push((((neg >> 8) as u8) & 0x7f) | ((((t.size - 1) & 1) as u8) << 7));
            } else {
                out.push(LZMASK | ((((t.size - 1) << 2) as u8) & 0x7f) | 2);
                out.push((t.offset & 0xff) as u8);
            }
        }
    }
}

/// A graph edge: destination vertex, integer cost, and the token it represents.
#[derive(Clone, Copy)]
struct Edge {
    dest: i32,
    cost: i64,
    token: Token,
}

/// Binary min-heap entry keyed on `dist` only (a manual
/// array-backed binary heap - std `BinaryHeap` would give a different pop order on ties and could
/// diverge, so we replicate the exact heap).
#[derive(Clone, Copy)]
struct PqItem {
    vertex: i32,
    dist: i64,
}

struct PriorityQueue {
    items: Vec<PqItem>,
}

impl PriorityQueue {
    fn new(cap: usize) -> PriorityQueue {
        PriorityQueue {
            items: Vec::with_capacity(cap.max(16)),
        }
    }

    fn push(&mut self, vertex: i32, dist: i64) {
        self.items.push(PqItem { vertex, dist });
        let mut idx = self.items.len() - 1;
        while idx > 0 {
            let parent = (idx - 1) / 2;
            if self.items[parent].dist <= self.items[idx].dist {
                break;
            }
            self.items.swap(parent, idx);
            idx = parent;
        }
    }

    fn pop(&mut self) -> Option<PqItem> {
        let n = self.items.len();
        if n == 0 {
            return None;
        }
        let out = self.items[0];
        let last = self.items.pop().unwrap();
        if n > 1 {
            self.items[0] = last;
            let size = self.items.len();
            let mut idx = 0usize;
            loop {
                let left = idx * 2 + 1;
                let right = idx * 2 + 2;
                let mut smallest = idx;
                if left < size && self.items[left].dist < self.items[smallest].dist {
                    smallest = left;
                }
                if right < size && self.items[right].dist < self.items[smallest].dist {
                    smallest = right;
                }
                if smallest == idx {
                    break;
                }
                self.items.swap(idx, smallest);
                idx = smallest;
            }
        }
        Some(out)
    }
}

/// Core cruncher. `addr` is the 2-byte PRG load address used only
/// by in-place mode. Returns the crunched stream and the chosen optimal-run length.
fn crunch(src: &[u8], inplace: bool, addr: [u8; 2]) -> (Vec<u8>, i32) {
    crunch_inner(src, inplace, addr, false)
}

/// As [`crunch`], but `rich` swaps the native single-best LZ token generation for the per-length
/// closest-offset enumeration ([`lz_per_length`]). Everything else - the cost model, RLE/LZ2/
/// ZERORUN/LITERAL tokens, the Dijkstra parse, and the emitter - is byte-for-byte the same, so a
/// `rich` stream is a valid TSCrunch stream that the 6502 decruncher and [`decompress_forward`]/
/// [`decompress_inplace`] decode. The optimal parse over the richer edge set can only ever be
/// `<=` the native one (more/cheaper candidates, never fewer).
fn crunch_inner(src: &[u8], inplace: bool, addr: [u8; 2], rich: bool) -> (Vec<u8>, i32) {
    // Empty input: nothing to crunch. (The reference rejects len<=0; we return a minimal stream
    // that our own decoder roundtrips: just optimalRun-1 + terminator for forward, and the
    // wrapped form for in-place.)
    let mut work_src: &[u8] = src;
    let mut remainder_byte = 0u8;
    if inplace {
        if !work_src.is_empty() {
            remainder_byte = work_src[work_src.len() - 1];
            work_src = &work_src[..work_src.len() - 1];
        }
    }

    let work_len = work_src.len() as i32;
    let optimal_run = find_optimal_zero(work_src);

    // Prefix array for the LZ search (the Go reference's default path).
    let pa = PrefixArray::build(work_src);
    let mut scratch: Vec<i32> = Vec::new();
    let mut best_off = [0i32; 257]; // per-length closest offset (rich path only)

    // --- build the token graph (one EdgeList per source position) ---
    let mut graph: Vec<Vec<Edge>> = vec![Vec::new(); (work_len + 1) as usize];

    let max_token_size = 256i32;

    for i in 0..work_len {
        // present[size] + tokens[size]: at most one token per length, like the C version.
        let mut present = [false; 257];
        let mut tokens = [Token::empty(); 257];
        let mut max_size = 0i32;

        let rle_size = rle_length(work_src, i);
        let rle_cap = rle_size.min(LONGEST_RLE);

        if rich {
            // Richer path: one LZ token per length, each at its CLOSEST (cheapest) offset.
            if rle_cap < LONGEST_LONG_LZ - 1 {
                let minlz = (rle_cap + 1).max(MIN_LZ);
                let max_len =
                    lz_per_length(&pa, work_src, i, minlz, LONGEST_LONG_LZ, &mut best_off);
                let mut size = max_len.min(LONGEST_LONG_LZ);
                while size >= minlz && size > rle_cap {
                    let off = best_off[size as usize];
                    if off > 0 {
                        let t = Token {
                            ttype: TokenType::Lz,
                            pos: i,
                            size,
                            offset: off,
                            rlebyte: 0,
                        };
                        tokens[size as usize] = t;
                        present[size as usize] = true;
                        if size > max_size {
                            max_size = size;
                        }
                    }
                    size -= 1;
                }
            }
        } else {
            let mut lz = if rle_cap < LONGEST_LONG_LZ - 1 {
                let minlz = (rle_cap + 1).max(MIN_LZ);
                lz_best(&pa, work_src, i, minlz, &mut scratch)
            } else {
                Token {
                    ttype: TokenType::Lz,
                    pos: i,
                    size: 1,
                    offset: 0,
                    rlebyte: 0,
                }
            };

            // LZ tokens for every length from best down to MINLZ (above rle_cap), same offset.
            while lz.size >= MIN_LZ && lz.size > rle_cap {
                let mut t = lz;
                t.size = lz.size;
                tokens[t.size as usize] = t;
                present[t.size as usize] = true;
                if t.size > max_size {
                    max_size = t.size;
                }
                lz.size -= 1;
            }
        }

        // RLE tokens.
        if rle_size > LONGEST_RLE {
            let t = Token {
                ttype: TokenType::Rle,
                pos: i,
                size: LONGEST_RLE,
                offset: 0,
                rlebyte: work_src[i as usize],
            };
            tokens[t.size as usize] = t;
            present[t.size as usize] = true;
            if t.size > max_size {
                max_size = t.size;
            }
        } else {
            let mut size = rle_size;
            while size >= MIN_RLE {
                let t = Token {
                    ttype: TokenType::Rle,
                    pos: i,
                    size,
                    offset: 0,
                    rlebyte: work_src[i as usize],
                };
                tokens[size as usize] = t;
                present[size as usize] = true;
                if size > max_size {
                    max_size = size;
                }
                size -= 1;
            }
        }

        // LZ2 token.
        let lz2 = lz2_offset(work_src, i);
        if lz2 > 0 {
            let t = Token {
                ttype: TokenType::Lz2,
                pos: i,
                size: LZ2_SIZE,
                offset: lz2,
                rlebyte: 0,
            };
            tokens[t.size as usize] = t;
            present[t.size as usize] = true;
            if t.size > max_size {
                max_size = t.size;
            }
        }

        // ZERORUN token.
        if zerorun_at(work_src, i, optimal_run) {
            let t = Token {
                ttype: TokenType::Zerorun,
                pos: i,
                size: optimal_run,
                offset: 0,
                rlebyte: 0,
            };
            if t.size <= max_token_size {
                tokens[t.size as usize] = t;
                present[t.size as usize] = true;
                if t.size > max_size {
                    max_size = t.size;
                }
            }
        }

        // LITERAL tokens fill any remaining lengths 1..=min(LONGEST_LITERAL, work_len-i).
        let lit_max = LONGEST_LITERAL.min(work_len - i);
        for size in 1..=lit_max {
            if !present[size as usize] {
                let t = Token {
                    ttype: TokenType::Literal,
                    pos: i,
                    size,
                    offset: 0,
                    rlebyte: 0,
                };
                present[size as usize] = true;
                tokens[size as usize] = t;
                if size > max_size {
                    max_size = size;
                }
            }
        }

        // Emit edges in ascending size order (this order matters: Dijkstra relaxes edges in this
        // order and keeps the first-found predecessor on ties, matching the C reference).
        for size in 1..=max_size {
            if !present[size as usize] {
                continue;
            }
            if size <= 0 || i + size > work_len {
                continue;
            }
            let t = tokens[size as usize];
            graph[i as usize].push(Edge {
                dest: i + size,
                cost: token_cost(&t),
                token: t,
            });
        }
    }

    // --- Dijkstra shortest path 0 -> work_len ---
    let n = work_len;
    let big = i64::MAX / 4;
    let mut dist = vec![big; (n + 1) as usize];
    let mut prev = vec![-1i32; (n + 1) as usize];
    let mut prev_token = vec![Token::empty(); (n + 1) as usize];
    dist[0] = 0;

    let mut pq = PriorityQueue::new((n + 1) as usize);
    pq.push(0, 0);

    while let Some(item) = pq.pop() {
        let u = item.vertex;
        if item.dist != dist[u as usize] {
            continue;
        }
        if u == n {
            break;
        }
        for edge in &graph[u as usize] {
            let v = edge.dest;
            let alt = dist[u as usize] + edge.cost;
            if alt < dist[v as usize] {
                dist[v as usize] = alt;
                prev[v as usize] = u;
                prev_token[v as usize] = edge.token;
                pq.push(v, alt);
            }
        }
    }

    // Reconstruct token list (forward order). For empty/degenerate inputs prev[n] may be -1.
    let mut token_list: Vec<Token> = Vec::new();
    if n > 0 && prev[n as usize] >= 0 {
        let mut v = n;
        while v > 0 {
            token_list.push(prev_token[v as usize]);
            v = prev[v as usize];
        }
        token_list.reverse();
    }
    let token_count = token_list.len();

    // --- emit ---
    let mut out: Vec<u8> = Vec::new();

    if inplace {
        // Trim a "safe" suffix that does not compress; ship it as a literal remainder tail.
        let mut safety = token_count;
        let mut segment_uncrunched = 0i32;
        let mut segment_crunched = 0i32;
        let mut total_uncrunched = 0i32;

        let mut i = token_count as i32 - 1;
        while i >= 0 {
            segment_crunched += payload_len(&token_list[i as usize]);
            segment_uncrunched += token_list[i as usize].size;
            if segment_uncrunched <= segment_crunched {
                safety = i as usize;
                total_uncrunched += segment_uncrunched;
                segment_uncrunched = 0;
                segment_crunched = 0;
            }
            i -= 1;
        }

        // remainder = src[work_len-total_uncrunched..] (truncated tail) + remainder_byte
        let mut remainder: Vec<u8> = Vec::new();
        if total_uncrunched > 0 {
            remainder.extend_from_slice(
                &work_src[(work_len - total_uncrunched) as usize..work_len as usize],
            );
        }
        remainder.push(remainder_byte);

        for i in 0..safety {
            emit_token(&mut out, work_src, &token_list[i]);
        }
        out.push(TERMINATOR);
        // remainder[1:]  (everything after the first byte)
        if remainder.len() > 1 {
            out.extend_from_slice(&remainder[1..]);
        }

        // final = addr + (optimalRun-1) + remainder[0] + out
        let mut final_out: Vec<u8> = Vec::new();
        final_out.extend_from_slice(&addr);
        final_out.push((optimal_run - 1) as u8);
        final_out.push(remainder[0]);
        final_out.extend_from_slice(&out);
        out = final_out;
    } else {
        out.push((optimal_run - 1) as u8);
        for t in &token_list {
            emit_token(&mut out, work_src, t);
        }
        out.push(TERMINATOR);
    }

    (out, optimal_run)
}

/// Forward crunch, byte-identical to `tscrunch -p -q` (the crunched stream only - the caller has
/// already stripped the 2-byte PRG load address). This is the no-regression ANCHOR.
pub fn compress_tscrunch(input: &[u8]) -> Vec<u8> {
    let (out, _) = crunch(input, false, [0, 0]);
    out
}

/// Forward crunch using the richer per-length LZ candidate set ([`crunch_inner`] with `rich=true`).
/// A valid TSCrunch stream decodable by [`decompress_forward`] and the 6502 decruncher.
pub fn compress_tscrunch_rich(input: &[u8]) -> Vec<u8> {
    let (out, _) = crunch_inner(input, false, [0, 0], true);
    out
}

/// No-regression forward crunch: the smaller of the byte-identical [`compress_tscrunch`] anchor and
/// the richer [`compress_tscrunch_rich`]. Never larger than `tscrunch -p -q`.
pub fn compress_tscrunch_best(input: &[u8]) -> Vec<u8> {
    let anchor = compress_tscrunch(input);
    let rich = compress_tscrunch_rich(input);
    if rich.len() < anchor.len() {
        rich
    } else {
        anchor
    }
}

/// In-place crunch, byte-identical to `tscrunch -p -i -q`. The 2-byte PRG load address goes into
/// the in-place wrapper; pass it via `addr`. The convenience `compress_tscrunch_backward` derives
/// the load address the way the reference does.
pub fn compress_tscrunch_backward_with_addr(input: &[u8], addr: [u8; 2]) -> Vec<u8> {
    let (crunched, _) = crunch(input, true, addr);
    wrap_inplace(input, addr, crunched)
}

/// Prepend the 2-byte computed `load_to` to an in-place-crunched body (the C reference's `main()`
/// step). `load_to` depends on the crunched length, so each best-of candidate must be wrapped
/// independently before comparing.
fn wrap_inplace(input: &[u8], addr: [u8; 2], crunched: Vec<u8>) -> Vec<u8> {
    let decrunch_to = addr[0] as i32 + 256 * addr[1] as i32;
    let crunch_len = input.len() as i32; // data length (load address already stripped)
    let decrunch_end = (decrunch_to + crunch_len - 1) & 0xffff;
    let load_to = (decrunch_end - crunched.len() as i32 + 1) & 0xffff;
    let mut out = Vec::with_capacity(crunched.len() + 2);
    out.push((load_to & 0xff) as u8);
    out.push(((load_to >> 8) & 0xff) as u8);
    out.extend_from_slice(&crunched);
    out
}

/// In-place crunch with the richer per-length LZ candidate set. Valid in-place TSCrunch stream,
/// decodable by [`decompress_inplace`] and the 6502 decruncher.
pub fn compress_tscrunch_rich_backward_with_addr(input: &[u8], addr: [u8; 2]) -> Vec<u8> {
    let (crunched, _) = crunch_inner(input, true, addr, true);
    wrap_inplace(input, addr, crunched)
}

/// No-regression in-place crunch: the smaller of the byte-identical anchor
/// ([`compress_tscrunch_backward_with_addr`]) and the richer
/// ([`compress_tscrunch_rich_backward_with_addr`]). Never larger than `tscrunch -p -i -q`.
pub fn compress_tscrunch_best_backward_with_addr(input: &[u8], addr: [u8; 2]) -> Vec<u8> {
    let anchor = compress_tscrunch_backward_with_addr(input, addr);
    let rich = compress_tscrunch_rich_backward_with_addr(input, addr);
    if rich.len() < anchor.len() {
        rich
    } else {
        anchor
    }
}

/// In-place crunch using a default load address of $0000 for the wrapper, best-of (the tier-2
/// quality). For PRG inputs the load address is part of the format; use `compress` (which threads
/// the address through) for exact reference parity on real .prg files.
pub fn compress_tscrunch_backward(input: &[u8]) -> Vec<u8> {
    compress_tscrunch_best_backward_with_addr(input, [0, 0])
}

/// In-place crunch using a default load address of $0000, byte-identical to `tscrunch -p -i` (the
/// tier-1 anchor - no rich matchfinder). See [`compress_tscrunch_backward_with_addr`].
pub fn compress_tscrunch_anchor_backward(input: &[u8]) -> Vec<u8> {
    compress_tscrunch_backward_with_addr(input, [0, 0])
}

/// Uniform API: compress. `level` is NORMALIZED into `1..=MAX_LEVEL`.
///   level 1 = the byte-identical `tscrunch -p` anchor (fastest, native quality).
///   level 2 = the rich-matchfinder best-of (smallest, never larger than level 1).
/// `backward` selects the in-place / reverse layout.
///
/// NOTE: the input here is the raw data with the PRG load address ALREADY stripped (the `-p`
/// convention). For the in-place wrapper the load address is taken as $0000; the `tscc` CLI in
/// main.rs reads the address from the .prg and calls `compress_tscrunch_backward_with_addr` so
/// byte-identity holds on real files.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let rich = level >= 2;
    compress_native(input, rich, backward)
}

/// Native API: expose the real TSCrunch knob directly.
///   `rich = false` => the byte-identical `tscrunch -p[-i]` anchor (tier 1).
///   `rich = true`  => the rich-matchfinder best-of (tier 2; never larger than the anchor).
/// `backward` selects the in-place / reverse layout (load address $0000; see [`compress`]).
pub fn compress_native(input: &[u8], rich: bool, backward: bool) -> Vec<u8> {
    match (rich, backward) {
        (false, false) => compress_tscrunch(input),
        (true, false) => compress_tscrunch_best(input),
        (false, true) => compress_tscrunch_anchor_backward(input),
        (true, true) => compress_tscrunch_backward(input),
    }
}

// ----------------------------------------------------------------------------------------------
// Decoder, pure Rust for the forward and in-place layouts.
// ----------------------------------------------------------------------------------------------

/// Decode a forward-crunched stream (the output of `compress_tscrunch`). Runs the decruncher
/// token loop. `optimalRun` comes from the first byte.
pub fn decompress_forward(src: &[u8]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    if src.is_empty() {
        return out;
    }
    let optimal_run = src[0] as i32 + 1;
    let mut i = 1usize;

    while i < src.len() && src[i] != TERMINATOR {
        let code = src[i];
        if (code & 0x80) == LITERALMASK && (code & 0x7f) < 32 {
            // LITERAL
            let run = (code & 0x1f) as usize;
            out.extend_from_slice(&src[i + 1..i + 1 + run]);
            i += run + 1;
        } else if (code & 0x80) == LZ2MASK {
            // LZ2 (2-byte short-offset match). Note: this branch is reached only when the LITERAL
            // test above failed, i.e. (code & 0x7f) >= 32 with the top bit clear.
            let run = LZ2_SIZE as usize;
            let offset = (127 - (code & 0x7f) as i32) as usize;
            let p = out.len();
            for l in 0..run {
                out.push(out[p - offset + l]);
            }
            i += 1;
        } else if (code & 0x81) == RLEMASK && (code & 0x7e) != 0 {
            // RLE
            let run = (((code & 0x7f) >> 1) + 1) as usize;
            let b = src[i + 1];
            out.extend(std::iter::repeat(b).take(run));
            i += 2;
        } else if (code & 0x81) == RLEMASK && (code & 0x7e) == 0 {
            // ZERORUN
            let run = optimal_run as usize;
            out.extend(std::iter::repeat(0u8).take(run));
            i += 1;
        } else {
            // LZ
            let run;
            let offset;
            if (code & 2) == 2 {
                run = (((code & 0x7f) >> 2) + 1) as usize;
                offset = src[i + 1] as usize;
                i += 2;
            } else {
                let lookahead = src[i + 2];
                run = (1
                    + (((code & 0x7f) >> 2) << 1) as i32
                    + if (lookahead & 128) == 128 { 1 } else { 0 }) as usize;
                offset = (32768 - (src[i + 1] as i32 + 256 * (lookahead & 0x7f) as i32)) as usize;
                i += 3;
            }
            let p = out.len();
            for l in 0..run {
                out.push(out[p - offset + l]);
            }
        }
    }
    out
}

/// Decode an in-place crunched stream (the output of `compress_tscrunch_backward*`). The wrapper is
/// `[load_lo load_hi] [addr_lo addr_hi] [optRun-1] [remainder_byte] <tokens...> TERMINATOR <tail>`.
///
/// Reconstruction (see the encoder): the token body decodes to `D[0 .. n-1-total]`, then comes
/// `remainder_byte` (= `D[n-1-total]`), then the literal tail (= `D[n-total .. n]`). So the original
/// is `token_body ++ remainder_byte ++ tail`. The token back-references never reach past the body,
/// so the body decodes standalone.
pub fn decompress_inplace(src: &[u8]) -> Vec<u8> {
    if src.len() < 6 {
        return Vec::new();
    }
    // src[0..2] = computed load address (ignored for a raw decode)
    // src[2..4] = original PRG load address (ignored for a raw decode)
    let optimal_run = src[4] as i32 + 1;
    let remainder_byte = src[5];

    let mut out: Vec<u8> = Vec::new();
    let mut i = 6usize;
    let mut tail_start = src.len();
    while i < src.len() {
        let code = src[i];
        if code == TERMINATOR {
            tail_start = i + 1;
            break;
        }
        if (code & 0x80) == LITERALMASK && (code & 0x7f) < 32 {
            let run = (code & 0x1f) as usize;
            out.extend_from_slice(&src[i + 1..i + 1 + run]);
            i += run + 1;
        } else if (code & 0x80) == LZ2MASK {
            let run = LZ2_SIZE as usize;
            let offset = (127 - (code & 0x7f) as i32) as usize;
            let p = out.len();
            for l in 0..run {
                out.push(out[p - offset + l]);
            }
            i += 1;
        } else if (code & 0x81) == RLEMASK && (code & 0x7e) != 0 {
            let run = (((code & 0x7f) >> 1) + 1) as usize;
            let b = src[i + 1];
            out.extend(std::iter::repeat(b).take(run));
            i += 2;
        } else if (code & 0x81) == RLEMASK && (code & 0x7e) == 0 {
            let run = optimal_run as usize;
            out.extend(std::iter::repeat(0u8).take(run));
            i += 1;
        } else {
            let run;
            let offset;
            if (code & 2) == 2 {
                run = (((code & 0x7f) >> 2) + 1) as usize;
                offset = src[i + 1] as usize;
                i += 2;
            } else {
                let lookahead = src[i + 2];
                run = (1
                    + (((code & 0x7f) >> 2) << 1) as i32
                    + if (lookahead & 128) == 128 { 1 } else { 0 }) as usize;
                offset = (32768 - (src[i + 1] as i32 + 256 * (lookahead & 0x7f) as i32)) as usize;
                i += 3;
            }
            let p = out.len();
            for l in 0..run {
                out.push(out[p - offset + l]);
            }
        }
    }

    // original = token_body ++ remainder_byte ++ tail
    out.push(remainder_byte);
    if tail_start < src.len() {
        out.extend_from_slice(&src[tail_start..]);
    }
    out
}

/// Uniform API: decompress. `backward` selects the in-place decoder.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        decompress_inplace(input)
    } else {
        decompress_forward(input)
    }
}

/// How many bytes ABOVE the end-aligned reference position an in-place stream
/// must be placed so the 6502 decoder never overwrites unread stream bytes.
///
/// The reference `tscrunch -p -i` layout end-aligns the stream with the output
/// (last stream byte = last output byte), which keeps the ascending write head
/// behind the ascending read head *between* tokens. But the 6502 token copies
/// overshoot that boundary check: the literal loop copies DESCENDING (its
/// highest write lands `run-1` above the write head while `run-1` payload
/// bytes are still unread), and RLE/LZ runs write up to `run-1` above the head
/// with only the 1-3 token bytes consumed. A stream whose gap gets tight while
/// such tokens remain (long incompressible stretches decoded late) therefore
/// self-corrupts in the reference layout.
///
/// Per token at boundary gap `d` (= read addr − write addr) the copy is safe
/// iff:
///   literal, run r (r+1 stream bytes):  d >= r-1
///   RLE, 2-byte token, run r:           d >= r-2
///   RLE, 1-byte token, run optRun:      d >= optRun-1
///   LZ2, 1-byte token:                  d >= 1
///   LZ, 2-byte token, run r:            d >= r-2
///   LZ, 3-byte token, run r:            d >= r-3
///   tail copy (ascending, 1:1):         d >= 0
///
/// Returns the smallest upward shift K (0 = the reference layout is already
/// safe) such that every token satisfies its bound when the stream is placed
/// K bytes above end-alignment. The caller must have K free bytes above the
/// output end. `src` is the wrapped stream (2-byte load_to prefix included).
pub fn inplace_required_shift(src: &[u8]) -> usize {
    if src.len() < 6 {
        return 0;
    }
    let optimal_run = src[4] as i64 + 1;
    // End-aligned gap at a token: gap(t) = (out_len - src_len) + i(t) - out(t).
    // out_len is only known after the walk, so track the largest
    // `need - (i - out)`; the required constant is C >= max(need - (i-out)),
    // and the reference layout provides C0 = out_len - src_len.
    let mut i = 6usize;
    let mut out: i64 = 0;
    let mut max_req = i64::MIN; // max over tokens of need - (i - out)
    let req = |need: i64, i: usize, out: i64| -> i64 { need - (i as i64 - out) };
    let mut tail_len: i64 = 0;
    while i < src.len() {
        let code = src[i];
        if code == TERMINATOR {
            // Tail copy is ascending and 1:1: safe for gap >= 0.
            max_req = max_req.max(req(0, i, out));
            tail_len = (src.len() - (i + 1)) as i64;
            break;
        }
        if (code & 0x80) == LITERALMASK && (code & 0x7f) < 32 {
            let run = (code & 0x1f) as i64;
            max_req = max_req.max(req(run - 1, i, out));
            out += run;
            i += run as usize + 1;
        } else if (code & 0x80) == LZ2MASK {
            max_req = max_req.max(req(1, i, out));
            out += LZ2_SIZE as i64;
            i += 1;
        } else if (code & 0x81) == RLEMASK && (code & 0x7e) != 0 {
            let run = (((code & 0x7f) >> 1) + 1) as i64;
            max_req = max_req.max(req(run - 2, i, out));
            out += run;
            i += 2;
        } else if (code & 0x81) == RLEMASK {
            max_req = max_req.max(req(optimal_run - 1, i, out));
            out += optimal_run;
            i += 1;
        } else if (code & 2) == 2 {
            let run = (((code & 0x7f) >> 2) + 1) as i64;
            max_req = max_req.max(req(run - 2, i, out));
            out += run;
            i += 2;
        } else {
            let lookahead = src[i + 2];
            let run = 1
                + ((((code & 0x7f) >> 2) << 1) as i64)
                + if (lookahead & 128) == 128 { 1 } else { 0 };
            max_req = max_req.max(req(run - 3, i, out));
            out += run;
            i += 3;
        }
    }
    // original = token body ++ remainder byte ++ tail
    let out_len = out + 1 + tail_len;
    let provided = out_len - src.len() as i64; // gap constant of the reference layout
    (max_req - provided).max(0) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt_forward(data: &[u8]) {
        let c = compress_tscrunch(data);
        let d = decompress_forward(&c);
        assert_eq!(d, data, "forward roundtrip len {}", data.len());
    }

    fn rt_backward(data: &[u8]) {
        let c = compress_tscrunch_backward_with_addr(data, [0x01, 0x08]);
        let d = decompress_inplace(&c);
        assert_eq!(d, data, "backward roundtrip len {}", data.len());
    }

    /// The richer and best-of variants must decode back to the input (both directions) and never be
    /// larger than the byte-identical anchor.
    fn rich_no_regression(data: &[u8]) {
        let anchor = compress_tscrunch(data);
        let rich = compress_tscrunch_rich(data);
        assert_eq!(
            decompress_forward(&rich),
            data,
            "rich fwd roundtrip len {}",
            data.len()
        );
        let best = compress_tscrunch_best(data);
        assert_eq!(
            decompress_forward(&best),
            data,
            "best fwd roundtrip len {}",
            data.len()
        );
        assert!(
            best.len() <= anchor.len(),
            "fwd best {} > anchor {}",
            best.len(),
            anchor.len()
        );

        let addr = [0x01, 0x08];
        let anchor_b = compress_tscrunch_backward_with_addr(data, addr);
        let rich_b = compress_tscrunch_rich_backward_with_addr(data, addr);
        assert_eq!(
            decompress_inplace(&rich_b),
            data,
            "rich bwd roundtrip len {}",
            data.len()
        );
        let best_b = compress_tscrunch_best_backward_with_addr(data, addr);
        assert_eq!(
            decompress_inplace(&best_b),
            data,
            "best bwd roundtrip len {}",
            data.len()
        );
        assert!(
            best_b.len() <= anchor_b.len(),
            "bwd best {} > anchor {}",
            best_b.len(),
            anchor_b.len()
        );
    }

    /// A simulated in-place decode with the write head placed `shift` bytes
    /// below the end-aligned read head, modeling the 6502 copy loops' write
    /// overshoot per token. Returns true when no write touches an unread
    /// stream byte - the ground truth `inplace_required_shift` must satisfy.
    fn inplace_safe_at_shift(src: &[u8], shift: i64) -> bool {
        let out = decompress_inplace(src);
        // Addresses relative to the output start; the stream's last byte sits
        // at out.len()-1 + shift.
        let stream_base = out.len() as i64 + shift - src.len() as i64;
        let optimal_run = src[4] as i64 + 1;
        let mut i = 6usize;
        let mut o: i64 = 0;
        while i < src.len() {
            let code = src[i];
            let gap = (stream_base + i as i64) - o;
            if code == TERMINATOR {
                if gap < 0 {
                    return false;
                }
                break;
            }
            let need = if (code & 0x80) == LITERALMASK && (code & 0x7f) < 32 {
                let run = (code & 0x1f) as i64;
                let need = run - 1;
                o += run;
                i += run as usize + 1;
                need
            } else if (code & 0x80) == LZ2MASK {
                o += LZ2_SIZE as i64;
                i += 1;
                1
            } else if (code & 0x81) == RLEMASK && (code & 0x7e) != 0 {
                let run = (((code & 0x7f) >> 1) + 1) as i64;
                o += run;
                i += 2;
                run - 2
            } else if (code & 0x81) == RLEMASK {
                o += optimal_run;
                i += 1;
                optimal_run - 1
            } else if (code & 2) == 2 {
                let run = (((code & 0x7f) >> 2) + 1) as i64;
                o += run;
                i += 2;
                run - 2
            } else {
                let lookahead = src[i + 2];
                let run = 1
                    + ((((code & 0x7f) >> 2) << 1) as i64)
                    + if (lookahead & 128) == 128 { 1 } else { 0 };
                o += run;
                i += 3;
                run - 3
            };
            if gap < need {
                return false;
            }
        }
        true
    }

    /// `inplace_required_shift` must return the exact safety threshold: the
    /// layout is safe at the returned shift and (when > 0) unsafe one below.
    #[test]
    fn inplace_required_shift_is_tight() {
        let mut x: u32 = 0x8badf00d;
        let mut rnd = |n: usize| -> Vec<u8> {
            (0..n)
                .map(|_| {
                    x = x.wrapping_mul(1664525).wrapping_add(1013904223);
                    (x >> 24) as u8
                })
                .collect()
        };
        let compressible: Vec<u8> = (0..24000usize)
            .map(|i| ((i / 64) as u8) ^ ((i % 7) as u8))
            .collect();

        // Well-compressible data: the reference end-aligned layout is safe.
        let c = compress_tscrunch_best_backward_with_addr(&compressible, [0x01, 0x08]);
        assert_eq!(
            inplace_required_shift(&c),
            0,
            "compressible data needs no shift"
        );
        assert!(inplace_safe_at_shift(&c, 0));

        // Encoder-produced stream over data with a late high-entropy stretch:
        // whatever the encoder emits, the returned shift must be tight against
        // the ground-truth simulation.
        let mut data = compressible.clone();
        data.extend(rnd(20000));
        let c = compress_tscrunch_best_backward_with_addr(&data, [0x01, 0x08]);
        let k = inplace_required_shift(&c);
        assert!(
            inplace_safe_at_shift(&c, k as i64),
            "returned shift must be safe"
        );
        if k > 0 {
            assert!(
                !inplace_safe_at_shift(&c, k as i64 - 1),
                "shift must be tight"
            );
        }

        // Hand-crafted stream: compression gains up front (RLE), then fat
        // literals arriving exactly where the gap is tight - the encoder's own
        // suffix-trim keeps the BOUNDARY gap >= 0 but does not model the
        // 6502 literal loop's descending write overshoot (run-1 above the
        // write head), which is what corrupted real already-packed payloads.
        // 10x RLE(64) then 20x literal(31), no tail:
        //   C = out_len - src_len = 1261 - 667 = 594
        //   first literal: gap = 594 + 26 - 640 = -20, need = 30 -> shift 50.
        let mut s: Vec<u8> = vec![0, 0, 0, 0, 0, 0x55]; // header; remainder=0x55
        for _ in 0..10 {
            s.extend_from_slice(&[0xFF, 0xAA]); // RLE run 64 of 0xAA
        }
        for j in 0..20u8 {
            s.push(0x1F); // literal run 31
            s.extend(std::iter::repeat(j).take(31));
        }
        s.push(TERMINATOR);
        let k = inplace_required_shift(&s);
        assert_eq!(k, 50, "crafted stream needs the modeled overshoot shift");
        assert!(inplace_safe_at_shift(&s, 50));
        assert!(!inplace_safe_at_shift(&s, 49));
        // A boundary-only model would claim 20 is enough; the literal
        // overshoot makes that corrupt.
        assert!(!inplace_safe_at_shift(&s, 20));
    }

    #[test]
    fn rich_variants_no_regression() {
        rich_no_regression(&[1, 2, 3, 4, 5]);
        rich_no_regression(&[0, 0, 0, 0, 0, 0, 0, 0]);
        let base = b"abcabcabcabc abracadabra ";
        let mut v = Vec::new();
        for _ in 0..500 {
            v.extend_from_slice(base);
        }
        rich_no_regression(&v);
        let mut s: Vec<u8> = Vec::new();
        s.extend(std::iter::repeat(0u8).take(300));
        s.extend(std::iter::repeat(0x41u8).take(200));
        s.extend_from_slice(b"hello world hello world");
        rich_no_regression(&s);
        // pseudo-random structured mix
        let mut v2: Vec<u8> = Vec::new();
        let mut x: u32 = 0x1234_5678;
        for k in 0..20000usize {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            let b = match (k / 137) % 5 {
                0 => 0u8,
                1 => (x >> 24) as u8,
                2 => b"abracadabra "[k % 12],
                3 => 0x41,
                _ => (k & 0xff) as u8,
            };
            v2.push(b);
        }
        rich_no_regression(&v2);
    }

    #[test]
    fn roundtrip_small() {
        rt_forward(&[]);
        rt_forward(&[0]);
        rt_forward(&[1, 2, 3, 4, 5]);
        rt_forward(&[0, 0, 0, 0, 0, 0, 0, 0]);
        let base = b"abcabcabcabc abracadabra ";
        let mut v = Vec::new();
        for _ in 0..500 {
            v.extend_from_slice(base);
        }
        rt_forward(&v);
    }

    #[test]
    fn roundtrip_backward_small() {
        rt_backward(&[1, 2, 3, 4, 5]);
        rt_backward(&[0, 0, 0, 0, 0, 0, 0, 0]);
        let base = b"abcabcabcabc abracadabra ";
        let mut v = Vec::new();
        for _ in 0..500 {
            v.extend_from_slice(base);
        }
        rt_backward(&v);
    }

    #[test]
    fn roundtrip_zeros_and_rle() {
        let mut v = Vec::new();
        v.extend(std::iter::repeat(0u8).take(300));
        v.extend(std::iter::repeat(0x41u8).take(200));
        v.extend_from_slice(b"hello world hello world");
        rt_forward(&v);
        rt_backward(&v);
    }

    #[test]
    fn roundtrip_pseudo_random_and_structured() {
        // A mix that exercises LZ long/short, LZ2, RLE, ZERORUN and literal-run boundaries.
        let mut v: Vec<u8> = Vec::new();
        let mut x: u32 = 0x1234_5678;
        for k in 0..20000usize {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            // bias toward repetition and zeros so the LZ/RLE/ZERORUN paths fire
            let b = match (k / 137) % 5 {
                0 => 0u8,
                1 => (x >> 24) as u8,
                2 => b"abracadabra "[k % 12],
                3 => 0x41,
                _ => (k & 0xff) as u8,
            };
            v.push(b);
        }
        rt_forward(&v);
        rt_backward(&v);
    }
}
