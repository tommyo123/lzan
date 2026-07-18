//! Exomizer v3.1.3 "raw" (-P39 default) codec.
//!
//! Encoder produces a stream that `exomizer raw -d` decodes back to the input;
//! decoder is a pure-Rust raw decruncher.
//!
//! Flags_proto = 39 = BE | COPY_GT_7 | IMPL_1LITERAL | REUSE_OFFSET. Forward mode
//! reverses the input, crunches the reversed buffer, then reverses the output.
//! Stream layout (forward):
//!   byte0          -> initial bit_buffer
//!   encoding table -> lengths(16 nibbles), offsets3(16), offsets2(16), offsets1(4)
//!   implicit first literal byte (IMPL_1LITERAL): first decoded byte is a raw literal
//!   then tokens (literal / sequence / literal-block / EOF gamma(16)).
//!
//! Scoring uses f32 to match the encoder's `float` arithmetic; the parse
//! tie-break depends on equal/less-than comparisons of accumulated costs.

const PFLAG_BITS_ORDER_BE: i32 = 1 << 0;
const PFLAG_BITS_COPY_GT_7: i32 = 1 << 1;
const PFLAG_IMPL_1LITERAL: i32 = 1 << 2;
/// Bit 3. Not part of -P39.
#[allow(dead_code)]
const PFLAG_BITS_ALIGN_START: i32 = 1 << 3;
const PFLAG_4_OFFSET_TABLES: i32 = 1 << 4;
const PFLAG_REUSE_OFFSET: i32 = 1 << 5;

/// Default raw flags: -P39.
const FLAGS_PROTO: i32 =
    PFLAG_BITS_ORDER_BE | PFLAG_BITS_COPY_GT_7 | PFLAG_IMPL_1LITERAL | PFLAG_REUSE_OFFSET;

const TFLAG_LIT_SEQ: i32 = 1 << 0;
const TFLAG_LEN1_SEQ: i32 = 1 << 1;
const TFLAG_LEN0123_SEQ_MIRRORS: i32 = 1 << 2;

const MAX_LEN: usize = 65535;
const MAX_OFFSET: usize = 65535;
const MAX_PASSES: usize = 100;

/// Highest normalized compression level for the uniform-tier API. Small by
/// design (1 = fastest, [`MAX_LEVEL`] = absolute best/smallest). The three tiers
/// map onto the internal 1..=[`MAX_TRAJECTORIES`] best-of-trajectory machinery
/// via [`trajectories_for_level`]: tier 1 = the single anchor trajectory
/// (fastest), tier 2 = best-of-4, tier 3 = best-of-8 (= absolute best).
pub const MAX_LEVEL: u8 = 3;

/// Number of distinct best-of trajectory configs available internally (see
/// `crunch_core`). Exposed through [`compress_native`] as the real knob; the
/// three public tiers select a subset count {1, 4, 8}.
pub const MAX_TRAJECTORIES: u32 = 8;

/// Map a normalized tier (1..=[`MAX_LEVEL`]) onto an internal trajectory/pass
/// count. Tier 1 → 1 pass (the no-regression anchor, fastest), tier 2 → 4
/// (best-of the original Parity reference-cache group == the previously shipped
/// output), tier 3 → 8 (the full best-of == absolute best). Higher tier ⇒ more
/// passes ⇒ size ≤ the lower tier (best-of can only shrink or tie).
fn trajectories_for_level(level: u8) -> u32 {
    match level.clamp(1, MAX_LEVEL) {
        1 => 1,
        2 => 4,
        _ => MAX_TRAJECTORIES,
    }
}

// ---------------------------------------------------------------------------
// Bit output. Bytes are appended to `buf`; the caller reverses `buf` to
// produce the forward stream.
// ---------------------------------------------------------------------------

struct OutputCtx {
    bitbuf: u8,
    bitcount: u8,
    buf: Vec<u8>,
    flags_proto: i32,
}

impl OutputCtx {
    fn new(flags_proto: i32) -> Self {
        OutputCtx {
            bitbuf: 0,
            bitcount: 0,
            buf: Vec::new(),
            flags_proto,
        }
    }

    fn output_byte(&mut self, byte: u8) {
        self.buf.push(byte);
    }

    fn bitbuf_bit(&mut self, bit: i32) {
        if self.flags_proto & PFLAG_BITS_ORDER_BE != 0 {
            self.bitbuf >>= 1;
            if bit != 0 {
                self.bitbuf |= 0x80;
            }
            self.bitcount += 1;
            if self.bitcount == 8 {
                self.output_bits_flush(false);
            }
        } else {
            self.bitbuf <<= 1;
            if bit != 0 {
                self.bitbuf |= 0x01;
            }
            self.bitcount += 1;
            if self.bitcount == 8 {
                self.output_bits_flush(false);
            }
        }
    }

    fn output_bits_flush(&mut self, add_marker_bit: bool) {
        if add_marker_bit {
            if self.flags_proto & PFLAG_BITS_ORDER_BE != 0 {
                self.bitbuf |= 0x80 >> self.bitcount;
            } else {
                self.bitbuf |= 0x01 << self.bitcount;
            }
            self.bitcount += 1;
        }
        if self.bitcount > 0 {
            let b = self.bitbuf;
            self.output_byte(b);
            self.bitbuf = 0;
            self.bitcount = 0;
        }
    }

    fn output_bits_int(&mut self, mut count: i32, mut val: i32) {
        if self.flags_proto & PFLAG_BITS_COPY_GT_7 != 0 {
            while count > 7 {
                self.output_byte((val & 0xFF) as u8);
                count -= 8;
                val >>= 8;
            }
        }
        while count > 0 {
            count -= 1;
            self.bitbuf_bit(val & 1);
            val >>= 1;
        }
    }

    fn output_bits(&mut self, count: i32, val: i32) {
        self.output_bits_int(count, val);
    }

    fn output_gamma_code(&mut self, mut code: i32) {
        self.output_bits_int(1, 1);
        while code > 0 {
            code -= 1;
            self.output_bits_int(1, 0);
        }
    }
}

// ---------------------------------------------------------------------------
// Interval-node encoding tables.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct IntervalNode {
    start: i32,
    bits: i32,   // extra bits, 0..=15
    depth: i32,  // index within the list (0-based)
    prefix: i32, // flags>=0 ? flags : depth+1
    flags: i32,
    score: i64,
    next: Option<Box<IntervalNode>>,
}

impl IntervalNode {
    fn init(start: i32, depth: i32, flags: i32) -> IntervalNode {
        IntervalNode {
            start,
            bits: 0,
            depth,
            prefix: if flags >= 0 { flags } else { depth + 1 },
            flags,
            score: -1,
            next: None,
        }
    }
}

/// A length/offset bucket returned by optimal_encode_int.
#[derive(Clone, Copy, Default)]
struct IntBucket {
    start: u32,
    end: u32,
}

/// Encode an int against an interval list. Returns the cost in bits (f32). If
/// `out` is provided, emits the bits. If `eib` is provided, fills the bucket.
fn optimal_encode_int(
    arg: i32,
    list: &Option<Box<IntervalNode>>,
    mut out: Option<&mut OutputCtx>,
    eib: Option<&mut IntBucket>,
) -> f32 {
    let mut node = list.as_deref();
    let mut found: Option<&IntervalNode> = None;
    let mut end = 0i32;
    while let Some(n) = node {
        end = n.start + (1 << n.bits);
        if arg >= n.start && arg < end {
            found = Some(n);
            break;
        }
        node = n.next.as_deref();
    }
    let mut val: f32 = 100000000.0;
    match found {
        Some(n) => {
            val = (n.prefix + n.bits) as f32;
            if let Some(b) = eib {
                b.start = n.start as u32;
                b.end = end as u32;
            }
            if let Some(o) = out.as_deref_mut() {
                o.output_bits(n.bits, arg - n.start);
                if n.flags < 0 {
                    o.output_gamma_code(n.depth);
                } else {
                    o.output_bits(n.prefix, n.depth);
                }
            }
        }
        None => {
            val += (arg - end) as f32;
            if let Some(b) = eib {
                b.start = 0;
                b.end = 0;
            }
        }
    }
    val
}

// --- optimize() DP: picks interval bit-widths from cumulative stats ---

struct OptimizeArg<'a> {
    stats: &'a [i32],
    stats2: Option<&'a [i32]>,
    max_depth: i32,
    flags: i32,
    cache: std::collections::HashMap<i32, Box<IntervalNode>>,
}

const STATS_LIMIT: i32 = 1_000_000;

fn optimize1(arg: &mut OptimizeArg, start: i32, depth: i32) -> Option<Box<IntervalNode>> {
    if start as usize >= arg.stats.len() || arg.stats[start as usize] == 0 {
        return None;
    }
    let key = start * 32 + depth;
    if let Some(c) = arg.cache.get(&key) {
        return Some(c.clone());
    }

    let mut best: Option<Box<IntervalNode>> = None;
    for i in 0..16i32 {
        let mut node = IntervalNode::init(start, depth, arg.flags);
        node.bits = i;
        let end = start + (1 << i);

        let start_count = if start < STATS_LIMIT && (start as usize) < arg.stats.len() {
            arg.stats[start as usize] as i64
        } else {
            0
        };
        let end_count =
            if start < STATS_LIMIT && end < STATS_LIMIT && (end as usize) < arg.stats.len() {
                arg.stats[end as usize] as i64
            } else {
                0
            };

        let mut score = (start_count - end_count) * (node.prefix + node.bits) as i64;

        if end_count > 0 {
            let mut next: Option<Box<IntervalNode>> = None;
            if depth + 1 < arg.max_depth {
                next = optimize1(arg, end, depth + 1);
            }
            let mut penalty: i64 = 100_000_000;
            if let Some(s2) = arg.stats2 {
                if (end as usize) < s2.len() {
                    penalty = s2[end as usize] as i64;
                }
            }
            if let Some(ref nx) = next {
                if nx.score < penalty {
                    penalty = nx.score;
                }
            }
            score += penalty;
            node.next = next;
        }
        node.score = score;

        match &best {
            None => best = Some(Box::new(node)),
            Some(b) if node.score < b.score => best = Some(Box::new(node)),
            _ => {}
        }
    }

    if let Some(ref b) = best {
        arg.cache.insert(key, b.clone());
    }
    best
}

fn optimize(
    stats: &[i32],
    stats2: Option<&[i32]>,
    max_depth: i32,
    flags: i32,
) -> Option<Box<IntervalNode>> {
    let mut arg = OptimizeArg {
        stats,
        stats2,
        max_depth,
        flags,
        cache: std::collections::HashMap::new(),
    };
    optimize1(&mut arg, 1, 0)
}

// ---------------------------------------------------------------------------
// Encoding tables: len + 8 offset classes.
// ---------------------------------------------------------------------------

struct Encoding {
    len: Option<Box<IntervalNode>>,
    offsets: [Option<Box<IntervalNode>>; 8],
}

impl Encoding {
    fn empty() -> Encoding {
        Encoding {
            len: None,
            offsets: Default::default(),
        }
    }
}

/// Cost in bits of a match (offset,len) given prev_offset, optionally emitting
/// and/or filling len+offset buckets.
fn optimal_encode(
    enc: &Encoding,
    offset: u32,
    len: u32,
    prev_offset: u32,
    flags_notrait: i32,
    mut out: Option<&mut OutputCtx>,
    mut embp: Option<&mut (IntBucket, IntBucket)>, // (len, offset)
) -> f32 {
    let mut bits: f32 = 0.0;

    if len > 255
        && (flags_notrait & TFLAG_LEN0123_SEQ_MIRRORS) != 0
        && (len & 255)
            < (if FLAGS_PROTO & PFLAG_4_OFFSET_TABLES != 0 {
                4
            } else {
                3
            })
    {
        bits += 100000000.0;
    }

    if offset == 0 {
        bits += 9.0 * len as f32;
    } else {
        bits += 1.0;
        if offset != prev_offset {
            // local mutable bucket for the offset table
            let mut off_eib = IntBucket::default();
            let want_eib = embp.is_some();
            match len {
                1 => {
                    if flags_notrait & TFLAG_LEN1_SEQ != 0 {
                        bits += 100000000.0;
                    } else {
                        bits += optimal_encode_int(
                            offset as i32,
                            &enc.offsets[0],
                            out.as_deref_mut(),
                            if want_eib { Some(&mut off_eib) } else { None },
                        );
                    }
                }
                2 => {
                    bits += optimal_encode_int(
                        offset as i32,
                        &enc.offsets[1],
                        out.as_deref_mut(),
                        if want_eib { Some(&mut off_eib) } else { None },
                    );
                }
                3 => {
                    if FLAGS_PROTO & PFLAG_4_OFFSET_TABLES != 0 {
                        bits += optimal_encode_int(
                            offset as i32,
                            &enc.offsets[2],
                            out.as_deref_mut(),
                            if want_eib { Some(&mut off_eib) } else { None },
                        );
                    } else {
                        bits += optimal_encode_int(
                            offset as i32,
                            &enc.offsets[7],
                            out.as_deref_mut(),
                            if want_eib { Some(&mut off_eib) } else { None },
                        );
                    }
                }
                _ => {
                    bits += optimal_encode_int(
                        offset as i32,
                        &enc.offsets[7],
                        out.as_deref_mut(),
                        if want_eib { Some(&mut off_eib) } else { None },
                    );
                }
            }
            if let Some(e) = embp.as_deref_mut() {
                e.1 = off_eib;
            }
        } else if let Some(e) = embp.as_deref_mut() {
            // offset == prev_offset: no offset bits; bucket stays zero.
            e.1 = IntBucket::default();
        }
        if prev_offset > 0 {
            bits += 1.0;
            if let Some(o) = out.as_deref_mut() {
                o.output_bits(1, if offset == prev_offset { 1 } else { 0 });
            }
        }
        let mut len_eib = IntBucket::default();
        let want_eib = embp.is_some();
        bits += optimal_encode_int(
            len as i32,
            &enc.len,
            out.as_deref_mut(),
            if want_eib { Some(&mut len_eib) } else { None },
        );
        if let Some(e) = embp.as_deref_mut() {
            e.0 = len_eib;
        }
    }

    if let Some(e) = embp.as_deref_mut() {
        // Reset both buckets if either is empty.
        if e.0.start + e.0.end == 0 || e.1.start + e.1.end == 0 {
            e.0 = IntBucket::default();
            e.1 = IntBucket::default();
        }
    }

    bits
}

// ---------------------------------------------------------------------------
// Encoding-table emission.
// ---------------------------------------------------------------------------

fn interval_out(
    out: &mut OutputCtx,
    list: &Option<Box<IntervalNode>>,
    size: usize,
    flags_proto: i32,
) {
    let mut vals: Vec<i32> = Vec::new();
    let mut node = list.as_deref();
    while let Some(n) = node {
        vals.push(n.bits);
        node = n.next.as_deref();
    }
    let count = vals.len();
    let mut nibbles: Vec<i32> = Vec::with_capacity(size);
    for _ in 0..(size - count) {
        nibbles.push(0);
    }
    for &v in vals.iter().rev() {
        nibbles.push(v);
    }
    for &b in &nibbles {
        if flags_proto & PFLAG_BITS_COPY_GT_7 != 0 {
            out.output_bits(1, b >> 3);
            out.output_bits(3, b & 7);
        } else {
            out.output_bits(4, b);
        }
    }
}

fn optimal_out(out: &mut OutputCtx, enc: &Encoding) {
    interval_out(out, &enc.offsets[0], 4, FLAGS_PROTO);
    interval_out(out, &enc.offsets[1], 16, FLAGS_PROTO);
    if FLAGS_PROTO & PFLAG_4_OFFSET_TABLES != 0 {
        interval_out(out, &enc.offsets[2], 16, FLAGS_PROTO);
    }
    interval_out(out, &enc.offsets[7], 16, FLAGS_PROTO);
    interval_out(out, &enc.len, 16, FLAGS_PROTO);
}

// ---------------------------------------------------------------------------
// Match structure. Operates on the buffer `buf`.
//
// A "match" is (offset, len): buf[i..i+len] == buf[i+offset..i+offset+len],
// offset >= 1 for sequences, offset == 0 for the literal (len 1).
// matches_calc(i) returns a list ordered literal-first, then matches with
// increasing len.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Mtch {
    offset: u32,
    len: u32,
}

struct MatchCtx {
    buf: Vec<u8>,
    len: usize,
    max_len: usize,
    max_offset: usize,
    rle: Vec<u16>,
    rle_r: Vec<u16>,
    // cache[i] = list of matches for index i (literal first).
    cache: Vec<Vec<Mtch>>,
    // Richer, unpruned candidate set: for each index, the K smallest offsets
    // achieving each distinct match length (plus the literal and the len-1
    // near-offset candidates the reference also emits). Built lazily; see
    // `build_rich_cache_k`. A strict superset (in optimization power) of `cache`:
    // every (offset,len) the reference emits is reachable here too, because the
    // reference walks a pruned subset of the same source positions.
    rich_cache: Vec<Vec<Mtch>>,
}

impl MatchCtx {
    fn new(buf: Vec<u8>, max_len: usize, max_offset: usize) -> MatchCtx {
        let n = buf.len();
        let mut ctx = MatchCtx {
            buf,
            len: n,
            max_len,
            max_offset,
            rle: vec![0u16; n + 1],
            rle_r: vec![0u16; n + 1],
            cache: Vec::new(),
            rich_cache: Vec::new(),
        };
        ctx.init();
        ctx
    }

    fn init(&mut self) {
        let n = self.len;
        let buf = &self.buf;
        let max_len = self.max_len;

        // rle forward: rle[i] = run of equal bytes ending at i (count of equal
        // preceding bytes), capped at max_len. rle[0] = 0.
        if n > 0 {
            let mut val = buf[0];
            for i in 1..n {
                if buf[i] == val {
                    let mut l = self.rle[i - 1] as usize + 1;
                    if l > max_len {
                        l = max_len;
                    }
                    self.rle[i] = l as u16;
                } else {
                    self.rle[i] = 0;
                }
                val = buf[i];
            }
            // rle_r reverse: rle_r[i] = run of equal following bytes, capped.
            // Seed `val` from buf[0], iterate i down from buf_len-2 comparing
            // buf[i] against buf[i+1]. rle_r[buf_len-1] stays 0.
            let mut val = buf[0];
            let mut i = n as isize - 2;
            while i >= 0 {
                let ii = i as usize;
                if buf[ii] == val {
                    let mut l = self.rle_r[ii + 1] as usize + 1;
                    if l > max_len {
                        l = max_len;
                    }
                    self.rle_r[ii] = l as u16;
                } else {
                    self.rle_r[ii] = 0;
                }
                val = buf[ii];
                i -= 1;
            }
        }

        // Build per-char chains via a two-pass construction with rle_map dedup
        // and trailing_np fixups. A position is a "node" if it links into the
        // chain; is_node[i] marks nodes and node_next[i] is the index the node
        // points to. matches_calc walks node_next from a starting position.
        let mut single = vec![usize::MAX; n];
        let none = usize::MAX;
        let mut is_node = vec![false; n];
        let mut node_next = vec![none; n];

        let mut rle_map = vec![0u8; 65536];
        for c in 0..256u32 {
            let c = c as u8;
            // forward pass
            for v in rle_map.iter_mut() {
                *v = 0;
            }
            let mut prev_np = none;
            let mut trailing_np = none;
            for i in 0..n {
                if buf[i] != c {
                    continue;
                }
                let rle_len = self.rle[i] as usize;
                if rle_map[rle_len] == 0 && self.rle_r[i] as usize > 16 {
                    continue;
                }

                // create node at i
                is_node[i] = true;
                node_next[i] = none;
                rle_map[rle_len] = 1;

                if prev_np != none {
                    node_next[prev_np] = i;
                    trailing_np = prev_np;
                }
                if trailing_np != none {
                    while trailing_np != prev_np {
                        let tmp = node_next[trailing_np];
                        node_next[trailing_np] = i;
                        trailing_np = tmp;
                    }
                    trailing_np = none;
                }
                single[i] = i; // mark node at i
                prev_np = i;
            }
            while trailing_np != none {
                let tmp = node_next[trailing_np];
                node_next[trailing_np] = none;
                trailing_np = tmp;
            }

            // backward pass
            for v in rle_map.iter_mut() {
                *v = 0;
            }
            let mut prev_np = none;
            let mut i = n as isize - 1;
            while i >= 0 {
                let ii = i as usize;
                if buf[ii] != c {
                    i -= 1;
                    continue;
                }
                let rle_len = self.rle_r[ii] as usize;
                if !is_node[ii] {
                    if rle_map[rle_len] != 0 && prev_np != none && rle_len > 0 {
                        // create node at ii pointing to prev_np
                        is_node[ii] = true;
                        node_next[ii] = prev_np;
                        single[ii] = ii;
                    }
                } else {
                    prev_np = ii;
                }

                if self.rle_r[ii] as usize > 0 {
                    i -= 1;
                    continue;
                }
                let rle_len2 = self.rle[ii] as usize + 1;
                if rle_len2 < rle_map.len() {
                    rle_map[rle_len2] = 1;
                }
                i -= 1;
            }
        }
        let _ = single;

        // Compute the match cache for each index.
        let mut cache: Vec<Vec<Mtch>> = vec![Vec::new(); n];
        for i in (0..n).rev() {
            cache[i] = self.matches_calc(i, &is_node, &node_next);
        }
        self.cache = cache;
    }

    /// Returns the match list for `index`: literal(len1,off0), then sequences
    /// with strictly increasing len.
    fn matches_calc(&self, index: usize, is_node: &[bool], node_next: &[usize]) -> Vec<Mtch> {
        let buf = &self.buf;
        let none = usize::MAX;
        let max_len = self.max_len;
        let max_offset = self.max_offset;

        let mut matches: Vec<Mtch> = Vec::new();
        // literal match
        matches.push(Mtch { offset: 0, len: 1 });
        // current best match is matches.last(); walk the chain from index.
        let mut np = if is_node[index] {
            node_next[index]
        } else {
            none
        };

        while np != none {
            if np > index + max_offset {
                break;
            }
            let mp_last = *matches.last().unwrap();
            let mp_len: usize = if mp_last.offset > 0 {
                mp_last.len as usize
            } else {
                0
            };

            let offset = np - index;
            // Compare the first <previous len> bytes backwards (skip first byte).
            let mut len = mp_len as isize;
            let mut pos = index as isize + 1 - len;
            while len > 1 {
                let p = pos as usize;
                let q = (pos + offset as isize) as usize;
                if p >= buf.len() || q >= buf.len() || buf[p] != buf[q] {
                    break;
                }
                let offset1 = self.rle_r[p] as isize;
                let offset2 = self.rle_r[q] as isize;
                let off = if offset1 < offset2 { offset1 } else { offset2 };
                len -= 1 + off;
                pos += 1 + off;
            }
            if len > 1 {
                // sequence too short, skip this match
                np = node_next[np];
                continue;
            }

            if offset < 17 {
                matches.push(Mtch {
                    offset: offset as u32,
                    len: 1,
                });
            }

            // Extend forwards from mp_len.
            let mut len = mp_len as isize;
            let mut pos = index as isize - len;
            while (len as usize) <= max_len && pos >= 0 && {
                let p = pos as usize;
                let q = (pos + offset as isize) as usize;
                q < buf.len() && buf[p] == buf[q]
            } {
                len += 1;
                pos -= 1;
            }
            if len > mp_len as isize {
                let mlen = index as isize - pos;
                let mut ml = mlen;
                if ml as usize > max_len {
                    ml = max_len as isize;
                }
                matches.push(Mtch {
                    offset: offset as u32,
                    len: ml as u32,
                });
            } else if len == mp_len as isize {
                let mlen = index as isize - pos;
                let mut ml = mlen;
                if ml as usize > max_len {
                    ml = max_len as isize;
                }
                matches.push(Mtch {
                    offset: offset as u32,
                    len: ml as u32,
                });
            }
            if len as usize > max_len {
                break;
            }
            if pos < 0 {
                break;
            }
            np = node_next[np];
        }

        // The list head is the longest match and the tail is the literal.
        // search_buffer and match_cache_peek both depend on this order for
        // tie-breaking and litp selection.
        matches.reverse();
        matches
    }

    fn matches_get(&self, index: usize) -> &[Mtch] {
        &self.cache[index]
    }

    /// Build the richer (unpruned) candidate cache.
    ///
    /// Exomizer's `matches_calc` walks a per-char chain whose nodes are an
    /// rle-deduplicated *subset* of the source positions (positions deep inside
    /// an rle run, or sharing an rle length already seen, are dropped). For each
    /// achievable match length it therefore emits whatever offset that pruned
    /// chain happens to surface - not necessarily the smallest.
    ///
    /// This builder instead considers *every* prior occurrence (via end-anchored
    /// 3-byte hash chains) and emits, for each distinct achievable length, the
    /// smallest offsets that reach it - the cheapest-to-encode offsets per length
    /// tier. It also mirrors the reference's extra emissions: the literal head
    /// `(0,1)` and, for any candidate with `offset < 17`, a len-1 candidate at
    /// that near offset (so short reuse stays available).
    ///
    /// The list is ordered longest-first with the literal at the tail, matching
    /// the layout `search_buffer` and the cache consumers expect. Matches follow
    /// the reference convention: a match at `i` covers `[i-L+1, i]` copied from
    /// the *higher* positions `[i-L+1+o, i+o]`, so lengths extend backward from
    /// `i` and are capped by the buffer start (`L <= i+1`).
    ///
    /// `max_chain` caps the chain walk per position for tractability on large
    /// inputs (offline use favours a generous cap). The result is still a strict
    /// superset (in optimization power) of the reference emissions whenever the
    /// cap is not hit, since the nearest offset per length surfaces early.
    /// `k_off` keeps up to that many smallest distinct offsets per achievable
    /// length tier (not just the single smallest). Extra offsets let the optimal
    /// parse pick whichever lands in a cheaper encoding bucket under the current
    /// tables - the lever that can actually beat the heuristic. `k_off == 1`
    /// reproduces the pure min-offset-per-length staircase.
    fn build_rich_cache_k(&mut self, max_chain: usize, k_off: usize) {
        let n = self.len;
        if n == 0 {
            self.rich_cache = Vec::new();
            return;
        }
        let buf = &self.buf;
        let max_len = self.max_len;
        let max_offset = self.max_offset;

        // Matches use the same convention as the reference `matches_calc`: a
        // match at index `i` with offset `o` covers positions `[i-L+1, i]` and
        // copies from the *higher* positions `[i-L+1+o, i+o]` (sources at higher
        // indices because the stream is built back-to-front). The match length
        // therefore extends BACKWARD from `i` (decreasing index) and is capped by
        // the buffer start (`L <= i+1`).
        //
        // We index positions by an end-anchored 3-byte key `buf[i-2..=i]` so a
        // chained candidate shares the last three bytes ending at `i` (the bytes
        // the backward extension consumes first). `head[h]` holds the lowest
        // inserted position with key h; inserting in decreasing index order means
        // walking `prev` from `head[h]` visits candidates in increasing index ==
        // increasing offset, so the nearest (cheapest) offset per length surfaces
        // first.
        const HBITS: u32 = 16;
        const HSIZE: usize = 1 << HBITS;
        let hash3_end = |p: usize| -> usize {
            // Only valid when p >= 2: keys the three bytes ending at p.
            let a = buf[p - 2] as u32;
            let b = buf[p - 1] as u32;
            let c = buf[p] as u32;
            let h = (a.wrapping_mul(506832829))
                ^ (b.wrapping_mul(2654435761))
                ^ (c.wrapping_mul(40503));
            ((h >> (32 - HBITS)) & (HSIZE as u32 - 1)) as usize
        };

        let mut head = vec![usize::MAX; HSIZE];
        let mut prev = vec![usize::MAX; n];
        let mut rich: Vec<Vec<Mtch>> = vec![Vec::new(); n];

        // Scratch reused per position: every distinct match length reached by a
        // chain candidate, paired with the K smallest offsets achieving it.
        // Indexed by length; only `touched` entries are live and get cleared.
        let cap_total = (max_len + 2).min(n + 2);
        let mut per_len: Vec<Vec<u32>> = vec![Vec::new(); cap_total];
        let mut touched: Vec<usize> = Vec::new();

        for i in (0..n).rev() {
            let mut list: Vec<Mtch> = Vec::new();

            // Walk the chain: nearest source first (increasing offset). The first
            // offset to reach a length is the smallest; longer reaches dominate
            // all shorter lengths, so a candidate of length L makes offsets
            // available for every length 1..=L. We record, for each length value
            // actually attained, up to `k_off` smallest offsets.
            let mut best_len: usize = 0;
            let mut emitted_len1_near = false;
            if i >= 2 {
                let h = hash3_end(i);
                let mut j = head[h];
                let mut steps = 0usize;
                while j != usize::MAX {
                    if steps >= max_chain {
                        break;
                    }
                    steps += 1;
                    debug_assert!(j > i);
                    let offset = j - i;
                    if offset > max_offset {
                        break;
                    }
                    if j >= n {
                        j = prev[j];
                        continue;
                    }
                    let cap = max_len.min(i + 1); // len <= i+1 (pos >= 0)
                    let mut len = 0usize;
                    while len < cap && buf[i - len] == buf[j - len] {
                        len += 1;
                    }
                    // Near len-1 candidate (mirror of matches_calc's offset<17).
                    if offset < 17 && len >= 1 && !emitted_len1_near {
                        list.push(Mtch {
                            offset: offset as u32,
                            len: 1,
                        });
                        emitted_len1_near = true;
                    }
                    if len > best_len {
                        // Record this offset at its peak length tier. Lengths
                        // 1..best_len already have an equal-or-smaller offset from
                        // an earlier (nearer) candidate, so we only need the new
                        // tiers (best_len, len]; recording at `len` and letting
                        // the parse try shorter tlen covers them.
                        let slot = &mut per_len[len];
                        if slot.len() < k_off {
                            if slot.is_empty() {
                                touched.push(len);
                            }
                            slot.push(offset as u32);
                        }
                        best_len = len;
                        if best_len >= max_len {
                            break;
                        }
                    } else if k_off > 1 && len >= 1 {
                        // Alternative offset for an already-reached length tier:
                        // keep it (up to k_off) so the parse has a choice of
                        // offsets that may price more cheaply under the tables.
                        let slot = &mut per_len[len];
                        if slot.len() < k_off {
                            if slot.is_empty() {
                                touched.push(len);
                            }
                            slot.push(offset as u32);
                        }
                    }
                    j = prev[j];
                }
            }

            // Insert i into its chain (only if a full 3-byte end-key exists).
            if i >= 2 {
                let h = hash3_end(i);
                prev[i] = head[h];
                head[h] = i;
            }

            // Emit the collected (len, offset) candidates and clear scratch.
            for &l in &touched {
                for &off in &per_len[l] {
                    list.push(Mtch {
                        offset: off,
                        len: l as u32,
                    });
                }
                per_len[l].clear();
            }
            touched.clear();

            // Order longest-first, literal at tail (the layout consumers expect).
            list.sort_by(|a, b| b.len.cmp(&a.len).then(a.offset.cmp(&b.offset)));
            list.push(Mtch { offset: 0, len: 1 });
            rich[i] = list;
        }

        self.rich_cache = rich;
    }

    fn matches_get_rich(&self, index: usize) -> &[Mtch] {
        &self.rich_cache[index]
    }
}

/// Diagnostic: for the (already direction-adjusted) `buf`, compare the richer
/// cache against the reference cache and report how richer it actually is.
/// Returns (positions, ref_better_or_equal, rich_strictly_smaller_offset,
/// rich_extra_length_tiers, max_extra_len). A non-trivial second/third figure
/// means the richer set can in principle change the parse.
pub fn rich_vs_ref_diag(buf: &[u8]) -> (usize, usize, usize, usize, usize) {
    if buf.is_empty() {
        return (0, 0, 0, 0, 0);
    }
    let mut ctx = MatchCtx::new(buf.to_vec(), MAX_LEN, MAX_OFFSET);
    let n = ctx.len;
    ctx.build_rich_cache_k(8192, 4);

    let mut smaller_off = 0usize; // rich offers a strictly smaller offset for some length
    let mut extra_tier = 0usize; // rich offers a length the reference cache lacks
    let mut max_extra_len = 0usize;

    // For each position, build maps len -> min offset for both caches and compare.
    for i in 0..n {
        let rref = ctx.matches_get(i);
        let rich = ctx.matches_get_rich(i);
        // min offset per length (sequences only).
        let mut ref_min: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
        for m in rref {
            if m.offset == 0 {
                continue;
            }
            let e = ref_min.entry(m.len).or_insert(u32::MAX);
            if m.offset < *e {
                *e = m.offset;
            }
        }
        let mut rich_min: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
        for m in rich {
            if m.offset == 0 {
                continue;
            }
            let e = rich_min.entry(m.len).or_insert(u32::MAX);
            if m.offset < *e {
                *e = m.offset;
            }
        }
        for (len, &roff) in &rich_min {
            match ref_min.get(len) {
                Some(&foff) => {
                    if roff < foff {
                        smaller_off += 1;
                    }
                }
                None => {
                    extra_tier += 1;
                    if *len as usize > max_extra_len {
                        max_extra_len = *len as usize;
                    }
                }
            }
        }
    }
    (n, n, smaller_off, extra_tier, max_extra_len)
}

// ---------------------------------------------------------------------------
// match_cache_enum: produces the bootstrap (pass-1) encoding before any search.
// ---------------------------------------------------------------------------

fn match_keep_this(len: u32, offset: u32) -> bool {
    if len == 1 && offset > 34 {
        return false;
    }
    true
}

/// Returns the chosen literal-path match (`litp`) and best sequence (`seqp`)
/// at `pos`.
fn match_cache_peek(ctx: &MatchCtx, pos: isize) -> (Option<Mtch>, Option<Mtch>) {
    if pos < 0 {
        return (None, None);
    }
    let p = pos as usize;
    let val_list = ctx.matches_get(p);

    // litp = first match with offset == 0.
    let mut litp: Option<Mtch> = None;
    for m in val_list {
        if m.offset == 0 {
            litp = Some(*m);
            break;
        }
    }

    // Iteration list = optional injected rle match, then val_list.
    let mut seqp: Option<Mtch> = None;

    // inject extra rle match if rle_r[pos] > 0 && rle[pos+1] > 0
    let mut injected: Option<Mtch> = None;
    if ctx.rle_r[p] > 0 && (p + 1 <= ctx.len) && ctx.rle[p + 1] > 0 {
        injected = Some(Mtch {
            offset: 1,
            len: ctx.rle[p + 1] as u32,
        });
    }

    let mut iterate: Vec<Mtch> = Vec::with_capacity(val_list.len() + 1);
    if let Some(m) = injected {
        iterate.push(m);
    }
    iterate.extend_from_slice(val_list);

    for v in &iterate {
        if v.offset != 0 {
            if match_keep_this(v.len, v.offset) {
                let better = match seqp {
                    None => true,
                    Some(s) => v.len > s.len || (v.len == s.len && v.offset < s.offset),
                };
                if better {
                    seqp = Some(*v);
                }
            }
            // litp update when it has no offset or a larger one.
            let litp_off = litp.map(|m| m.offset).unwrap_or(0);
            if litp_off == 0 || litp_off > v.offset {
                let src = p + v.offset as usize;
                let diff = if src <= ctx.len {
                    ctx.rle[src] as u32
                } else {
                    0
                };
                let mut off2 = v.offset;
                if off2 > diff {
                    off2 -= diff;
                } else {
                    off2 = 1;
                }
                if match_keep_this(1, off2) {
                    litp = Some(Mtch {
                        offset: off2,
                        len: 1,
                    });
                }
            }
        }
    }

    (litp, seqp)
}

struct MatchCacheEnum {
    pos: isize,
}

fn match_cache_enum_next(ctx: &MatchCtx, e: &mut MatchCacheEnum) -> Option<Mtch> {
    let (lit, seq) = match_cache_peek(ctx, e.pos);
    let mut val = lit;
    if lit.is_none() {
        e.pos = ctx.len as isize - 1;
    } else if let Some(s) = seq {
        // peek the next position's best sequence
        let (_l2, next) = match_cache_peek(ctx, e.pos - 1);
        let use_seq = match next {
            None => true,
            Some(nx) => {
                let bonus = if (e.pos & 1) != 0 && nx.len < 3 { 1 } else { 0 };
                s.len >= nx.len + bonus
            }
        };
        if use_seq {
            val = Some(s);
        }
    }
    if let Some(v) = val {
        e.pos -= v.len as isize;
    }
    val
}

// ---------------------------------------------------------------------------
// search_buffer: optimal-parse shortest path over the buffer. Scores
// accumulate in f32.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct SearchNode {
    index: i32,
    m_offset: u32,
    m_len: u32,
    total_offset: u32,
    total_score: f32,
    prev: i32, // index of prev node, or -1
    latest_offset: u32,
}

fn update_snp(
    nodes: &mut [SearchNode],
    idx: usize,
    total_score: f32,
    total_offset: u32,
    prev: usize,
    m_offset: u32,
    m_len: u32,
    flags_proto: i32,
) {
    let mut latest_offset = 0u32;
    if flags_proto & PFLAG_REUSE_OFFSET != 0 && m_offset == 0 {
        // literal node: inherit a preceding sequence's offset for reuse
        let prev_off = nodes[prev].m_offset;
        if prev_off > 0 {
            latest_offset = prev_off;
        }
    }
    nodes[idx].total_score = total_score;
    nodes[idx].total_offset = total_offset;
    nodes[idx].prev = prev as i32;
    nodes[idx].m_offset = m_offset;
    nodes[idx].m_len = m_len;
    nodes[idx].latest_offset = latest_offset;
}

fn search_buffer(
    ctx: &MatchCtx,
    enc: &Encoding,
    flags_proto: i32,
    flags_notrait: i32,
    max_sequence_length: usize,
    greedy: bool,
    use_rich: bool,
) -> Vec<SearchNode> {
    let n = ctx.len;
    let arr_len = n + 1;
    let mut nodes = vec![
        SearchNode {
            index: 0,
            m_offset: 0,
            m_len: 0,
            total_offset: 0,
            total_score: 0.0,
            prev: -1,
            latest_offset: 0,
        };
        arr_len
    ];

    let use_literal_sequences = (flags_notrait & TFLAG_LIT_SEQ) == 0;
    let mut skip_len0123_mirrors = flags_notrait & TFLAG_LEN0123_SEQ_MIRRORS;
    if skip_len0123_mirrors != 0 {
        skip_len0123_mirrors = if flags_proto & PFLAG_4_OFFSET_TABLES != 0 {
            4
        } else {
            3
        };
    }

    // terminal node at index n
    let mut len = n;
    nodes[len].index = len as i32;
    nodes[len].m_offset = 0;
    nodes[len].m_len = 0;
    nodes[len].total_offset = 0;
    nodes[len].total_score = 0.0;
    nodes[len].prev = -1;
    nodes[len].latest_offset = 0;

    let mut best_copy_snp = len; // index into nodes
    let mut best_copy_len: i64 = 0;
    let mut best_rle_snp: isize = -1;

    loop {
        if use_literal_sequences {
            let snp_idx = len;
            let snp = nodes[snp_idx];
            if (snp.m_offset != 0 || snp.m_len != 1)
                && (nodes[best_copy_snp].total_score + best_copy_len as f32 * 8.0 - snp.total_score
                    > 0.0
                    || best_copy_len > max_sequence_length as i64)
            {
                best_copy_snp = snp_idx;
                best_copy_len = 0;
            } else {
                let copy_score = best_copy_len as f32 * 8.0 + (1.0 + 17.0 + 17.0);
                let total_copy_score = nodes[best_copy_snp].total_score + copy_score;
                if snp.total_score > total_copy_score
                    && best_copy_len <= max_sequence_length as i64
                    && !(skip_len0123_mirrors != 0
                        && best_copy_len > 255
                        && (best_copy_len & 255) < 2)
                {
                    let bcs = best_copy_snp;
                    let bcs_to = nodes[bcs].total_offset;
                    update_snp(
                        &mut nodes,
                        snp_idx,
                        total_copy_score,
                        bcs_to,
                        bcs,
                        0,
                        best_copy_len as u32,
                        flags_proto,
                    );
                }
            }
        }

        // RLE optimization
        {
            let snp_idx = len;
            let snp_index = nodes[snp_idx].index as usize;
            let rle_here = ctx.rle[snp_index] as i64;
            let rle_r_here = ctx.rle_r[snp_index] as i64;

            let need_reset = best_rle_snp < 0
                || (snp_index as i64 + max_sequence_length as i64)
                    < nodes[best_rle_snp as usize].index as i64
                || (snp_index as i64 + rle_r_here) < nodes[best_rle_snp as usize].index as i64;

            if need_reset {
                if rle_here > 0 {
                    best_rle_snp = snp_idx as isize;
                } else {
                    best_rle_snp = -1;
                }
            } else if rle_here > 0
                && (snp_index as i64 + rle_r_here) >= nodes[best_rle_snp as usize].index as i64
            {
                let brs = best_rle_snp as usize;
                let brs_index = nodes[brs].index as usize;
                // best_rle_score
                let best_rle_score = optimal_encode(
                    enc,
                    1,
                    ctx.rle[brs_index] as u32,
                    nodes[brs].latest_offset,
                    flags_notrait,
                    None,
                    None,
                );
                let total_best_rle_score = nodes[brs].total_score + best_rle_score;
                let snp_rle_score = optimal_encode(
                    enc,
                    1,
                    ctx.rle[snp_index] as u32,
                    nodes[snp_idx].latest_offset,
                    flags_notrait,
                    None,
                    None,
                );
                let total_snp_rle_score = nodes[snp_idx].total_score + snp_rle_score;
                if total_snp_rle_score <= total_best_rle_score {
                    best_rle_snp = snp_idx as isize;
                }
            }

            if best_rle_snp >= 0 && best_rle_snp as usize != snp_idx {
                let brs = best_rle_snp as usize;
                let local_len = (nodes[brs].index - nodes[snp_idx].index) as u32;
                let rle_score = optimal_encode(
                    enc,
                    1,
                    local_len,
                    nodes[brs].latest_offset,
                    flags_notrait,
                    None,
                    None,
                );
                let total_rle_score = nodes[brs].total_score + rle_score;
                if nodes[snp_idx].total_score > total_rle_score {
                    let to = nodes[brs].total_offset + 1;
                    update_snp(
                        &mut nodes,
                        snp_idx,
                        total_rle_score,
                        to,
                        brs,
                        1,
                        local_len,
                        flags_proto,
                    );
                }
            }
        }

        if len == 0 {
            break;
        }

        // matches at index len-1
        let mp_list = if use_rich {
            ctx.matches_get_rich(len - 1)
        } else {
            ctx.matches_get(len - 1)
        };

        let prev_score = nodes[len].total_score;
        let latest_offset_sum = nodes[len].total_offset;

        for mp in mp_list {
            // Iterate all matches including the literal head (offset 0, len 1);
            // each is tried from mp.len down to 1 (offset 0 costs 9 bits).
            let mut bucket_len_start: u32 = 0;
            let prev_snp_idx = len;
            let prev_latest = nodes[prev_snp_idx].latest_offset;

            let mut score: f32 = 0.0;
            let end_len = 1u32;
            let mut tlen = mp.len;
            while tlen >= end_len {
                // bucket-skip optimization
                let recompute = bucket_len_start == 0
                    || tlen < 4
                    || tlen < bucket_len_start
                    || (skip_len0123_mirrors != 0
                        && tlen > 255
                        && (tlen & 255) < skip_len0123_mirrors as u32);
                if recompute {
                    let mut embp = (IntBucket::default(), IntBucket::default());
                    score = optimal_encode(
                        enc,
                        mp.offset,
                        tlen,
                        prev_latest,
                        flags_notrait,
                        None,
                        Some(&mut embp),
                    );
                    bucket_len_start = embp.0.start;
                }

                let total_score = prev_score + score;
                let total_offset = latest_offset_sum + mp.offset;
                let snp_idx = len - tlen as usize;

                if total_score < 100000000.0
                    && (nodes[snp_idx].m_len == 0
                        || total_score < nodes[snp_idx].total_score
                        || (total_score == nodes[snp_idx].total_score
                            && total_offset < nodes[snp_idx].total_offset
                            && (greedy
                                || (nodes[snp_idx].m_len == 1 && nodes[snp_idx].m_offset > 8)
                                || mp.offset > 48
                                || tlen > 15)))
                {
                    nodes[snp_idx].index = snp_idx as i32;
                    update_snp(
                        &mut nodes,
                        snp_idx,
                        total_score,
                        total_offset,
                        prev_snp_idx,
                        mp.offset,
                        tlen,
                        flags_proto,
                    );
                }

                tlen -= 1;
            }
        }

        len -= 1;
        best_copy_len += 1;
    }

    nodes
}

// ---------------------------------------------------------------------------
// optimal_optimize: build a new Encoding from a stream of matches. Two passes:
// lengths first, then offsets.
// ---------------------------------------------------------------------------

fn optimal_optimize(matches: &[Mtch]) -> Encoding {
    let len_cap = MAX_LEN + 2;
    let mut len_arr = vec![0i32; len_cap];
    for m in matches {
        if m.offset > 0 {
            len_arr[m.len as usize] += 1;
        }
    }
    for i in (0..len_cap - 1).rev() {
        len_arr[i] = len_arr[i].wrapping_add(len_arr[i + 1]);
    }
    let len_enc = optimize(&len_arr, None, 16, -1);

    let mut enc = Encoding::empty();
    enc.len = len_enc;

    let off_max = MAX_OFFSET + 2;
    let mut off_arr: Vec<Vec<i32>> = vec![vec![0i32; off_max]; 8];
    let mut off_parr: Vec<Vec<i32>> = vec![vec![0i32; off_max]; 8];

    for m in matches {
        if m.offset == 0 {
            continue;
        }
        let lc = optimal_encode_int(m.len as i32, &enc.len, None, None);
        let treshold = m.len as i32 * 9 - (1 + lc as i32);
        let idx = match m.len {
            1 => 0,
            2 => 1,
            3 => {
                if FLAGS_PROTO & PFLAG_4_OFFSET_TABLES != 0 {
                    2
                } else {
                    7
                }
            }
            _ => 7,
        };
        let o = m.offset as usize;
        if o < off_max {
            off_parr[idx][o] = off_parr[idx][o].wrapping_add(treshold);
            off_arr[idx][o] = off_arr[idx][o].wrapping_add(1);
        }
    }
    for i in (0..off_max - 1).rev() {
        for j in 0..8 {
            off_arr[j][i] = off_arr[j][i].wrapping_add(off_arr[j][i + 1]);
            off_parr[j][i] = off_parr[j][i].wrapping_add(off_parr[j][i + 1]);
        }
    }

    enc.offsets[0] = optimize(&off_arr[0], Some(&off_parr[0]), 1 << 2, 2);
    enc.offsets[1] = optimize(&off_arr[1], Some(&off_parr[1]), 1 << 4, 4);
    for j in 2..8 {
        enc.offsets[j] = optimize(&off_arr[j], Some(&off_parr[j]), 1 << 4, 4);
    }
    enc
}

// ---------------------------------------------------------------------------
// Walk the search-node chain from node 0 along ->prev until match.len==0 to
// produce the match list.
// ---------------------------------------------------------------------------

fn matches_from_snp(nodes: &[SearchNode]) -> Vec<Mtch> {
    let mut out = Vec::new();
    let mut cur: i32 = 0;
    loop {
        if cur < 0 {
            break;
        }
        let node = nodes[cur as usize];
        if node.m_len == 0 {
            break;
        }
        out.push(Mtch {
            offset: node.m_offset,
            len: node.m_len,
        });
        cur = node.prev;
    }
    out
}

// ---------------------------------------------------------------------------
// Encoding export, for convergence checks.
// ---------------------------------------------------------------------------

fn export_helper(np: &Option<Box<IntervalNode>>, mut depth: i32, out: &mut String) {
    let mut node = np.as_deref();
    while let Some(n) = node {
        out.push_str(&format!("{:X}", n.bits));
        node = n.next.as_deref();
        depth -= 1;
    }
    while depth > 0 {
        out.push('0');
        depth -= 1;
    }
}

fn encoding_export(enc: &Encoding) -> String {
    let mut s = String::new();
    export_helper(&enc.len, 16, &mut s);
    s.push(',');
    export_helper(&enc.offsets[0], 4, &mut s);
    s.push(',');
    export_helper(&enc.offsets[1], 16, &mut s);
    if FLAGS_PROTO & PFLAG_4_OFFSET_TABLES != 0 {
        s.push(',');
        export_helper(&enc.offsets[2], 16, &mut s);
    }
    s.push(',');
    export_helper(&enc.offsets[7], 16, &mut s);
    s
}

// ---------------------------------------------------------------------------
// Final emission. Builds the stream backwards (caller reverses it for forward
// mode). Walks the search-node chain from node 0 along ->prev.
// ---------------------------------------------------------------------------

fn do_output_backwards(
    ctx: &MatchCtx,
    nodes: &[SearchNode],
    enc: &Encoding,
    flags_proto: i32,
    output_header: bool,
) -> Vec<u8> {
    let mut out = OutputCtx::new(flags_proto);

    // The EOF gamma(16) marker is emitted whenever there is a terminal node,
    // including empty input where node 0 is the terminal with match.len == 0.
    let has_terminal = !nodes.is_empty();

    if has_terminal {
        out.output_gamma_code(16);
        out.output_bits(1, 0);
    }

    let mut cur: i32 = if has_terminal { 0 } else { -1 };
    while cur >= 0 {
        let node = nodes[cur as usize];
        if node.m_len == 0 {
            break;
        }
        let m_offset = node.m_offset;
        let m_len = node.m_len as usize;
        let snp_index = node.index as usize;

        if m_offset == 0 {
            // splitLitSeq = prev->match.len == 0 && IMPL_1LITERAL
            let prev = node.prev;
            let prev_len0 = prev < 0 || nodes[prev as usize].m_len == 0;
            let split_lit_seq = prev_len0 && (flags_proto & PFLAG_IMPL_1LITERAL != 0);

            let mut i = 0usize;
            if m_len > 1 {
                let mut len = m_len;
                if split_lit_seq {
                    len -= 1;
                }
                // 6510 litseq convention: the copy loop enters with X = len_lo
                // and DEX-wraps, so X = 0 copies a WHOLE extra 256-byte block -
                // a length with low byte 0 decodes as len + 256 on the metal.
                // Never emit such a length; move one byte to the singles tail
                // below. (The reference decoder mirrors the wrap, so a
                // regression here fails roundtrip loudly.)
                if len > 1 && len & 0xFF == 0 {
                    len -= 1;
                }
                for off in 0..len {
                    out.output_byte(ctx.buf[snp_index + off]);
                }
                out.output_bits(16, len as i32);
                out.output_gamma_code(17);
                out.output_bits(1, 0);
                i = len;
            }
            // Singles tail: [i..m_len) - 0, 1 or 2 bytes (the split byte, the
            // lo0-shaved byte, or both). Each is an explicit-flag literal
            // except the structural implicit one (split_lit_seq; always the
            // LAST index, which after the backwards-buffer reversal is the
            // stream-initial byte). Ascending emission puts the highest index
            // first in the final stream - the decode order.
            while i < m_len {
                out.output_byte(ctx.buf[snp_index + i]);
                let implicit = split_lit_seq && i == m_len - 1;
                if !implicit {
                    out.output_bits(1, 1);
                }
                i += 1;
            }
        } else {
            let prev = node.prev;
            let latest_offset = if prev >= 0 {
                nodes[prev as usize].latest_offset
            } else {
                0
            };
            optimal_encode(
                enc,
                m_offset,
                node.m_len,
                latest_offset,
                0,
                Some(&mut out),
                None,
            );
            out.output_bits(1, 0);
        }

        cur = node.prev;
    }

    if output_header {
        optimal_out(&mut out, enc);
    }

    out.output_bits_flush(true);
    out.buf
}

// ---------------------------------------------------------------------------
// Best-of-trajectories crunch.
//
// The parse<->encoding iteration is a heuristic whose f32 tie-resolution makes
// the converged encoding sensitive to the pass-1 seed. Every (nodes, enc) pair
// produced at any pass is internally consistent and emits a valid decodable
// stream, so multiple trajectories can run and the global smallest emission is
// kept. The first trajectory (CacheEnum, flip=false) is the no-regression
// anchor; additional trajectories can only shrink the result.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum BootKind {
    /// Seed the pass-1 encoding from `match_cache_enum`.
    CacheEnum,
    /// Same bootstrap plus one warm-up parse+optimize before the loop, which
    /// shifts the iteration phase onto a different trajectory.
    CacheEnumWarm,
}

/// Per-pass greedy schedule. Exomizer's f32 tie-resolution makes the converged
/// encoding sensitive to which passes parse greedily, so different schedules
/// reach different fixpoints - an independent best-of lever from the candidate
/// set.
#[derive(Clone, Copy, PartialEq)]
enum GreedyMode {
    /// greedy = (pass & 1) == 0, optionally flipped by `flip_greedy`.
    Parity,
    /// Every pass parses greedily.
    AllGreedy,
    /// Every pass parses lazily (non-greedy).
    AllLazy,
}

#[derive(Clone, Copy)]
struct TrajCfg {
    boot: BootKind,
    /// Flip the per-pass greedy parity (only used when `greedy == Parity`).
    flip_greedy: bool,
    /// Per-pass greedy schedule.
    greedy: GreedyMode,
    /// Use the richer (unpruned, K-offsets-per-length) candidate cache in the
    /// optimal parse. The bootstrap still seeds from the reference cache, so a
    /// rich trajectory only changes which matches the parse may pick.
    use_rich: bool,
}

/// Seed encoding from the `match_cache_enum` bootstrap.
fn boot_cache_enum(ctx: &MatchCtx) -> Encoding {
    let mut boot_matches: Vec<Mtch> = Vec::new();
    let mut e = MatchCacheEnum {
        pos: ctx.len as isize - 1,
    };
    while let Some(m) = match_cache_enum_next(ctx, &mut e) {
        boot_matches.push(m);
    }
    optimal_optimize(&boot_matches)
}

/// Run one trajectory of the parse<->encoding loop and return the smallest
/// emitted stream across all of its passes (header included).
fn run_trajectory(ctx: &MatchCtx, flags_proto: i32, flags_notrait: i32, cfg: TrajCfg) -> Vec<u8> {
    let max_len = MAX_LEN;
    let max_passes = MAX_PASSES;

    // bootstrap encoding
    let mut enc = boot_cache_enum(ctx);
    if cfg.boot == BootKind::CacheEnumWarm {
        // One warm-up parse+optimize (greedy) to shift the iteration phase.
        let warm = search_buffer(
            ctx,
            &enc,
            flags_proto,
            flags_notrait,
            max_len,
            true,
            cfg.use_rich,
        );
        let parse = matches_from_snp(&warm);
        let warm_enc = optimal_optimize(&parse);
        // Adopt the warm encoding only if non-degenerate; on all-literal inputs
        // it is empty and the cache-enum bootstrap is kept.
        if encoding_export(&warm_enc) != encoding_export(&Encoding::empty()) {
            enc = warm_enc;
        }
    }

    let mut prev_enc = encoding_export(&enc);
    let mut old_size = 100_000_000.0f32;
    let mut last_waltz = false;
    let mut pass = 1usize;

    // Track the smallest emitted stream over all passes.
    let mut best: Option<Vec<u8>> = None;
    let consider = |nodes: &[SearchNode], enc: &Encoding, best: &mut Option<Vec<u8>>| {
        let blob = do_output_backwards(ctx, nodes, enc, flags_proto, true);
        match best {
            Some(b) if b.len() <= blob.len() => {}
            _ => *best = Some(blob),
        }
    };

    loop {
        let greedy = match cfg.greedy {
            GreedyMode::Parity => {
                let mut g = (pass & 1) == 0;
                if cfg.flip_greedy {
                    g = !g;
                }
                g
            }
            GreedyMode::AllGreedy => true,
            GreedyMode::AllLazy => false,
        };
        let snpp = search_buffer(
            ctx,
            &enc,
            flags_proto,
            flags_notrait,
            max_len,
            greedy,
            cfg.use_rich,
        );
        let size = snpp[0].total_score;

        // Emit and measure this pass's (parse, enc) pair, keeping the byte
        // minimum rather than the f32-converged one.
        consider(&snpp, &enc, &mut best);

        if last_waltz {
            break;
        }

        pass += 1;
        if size >= old_size {
            // Final pass: re-run the search with the same enc/greedy.
            last_waltz = true;
            continue;
        }
        old_size = size;

        if pass > max_passes {
            break;
        }

        let parse_matches = matches_from_snp(&snpp);
        enc = optimal_optimize(&parse_matches);
        let new_enc = encoding_export(&enc);
        if new_enc == prev_enc {
            break;
        }
        prev_enc = new_enc;
    }

    best.expect("every trajectory emits at least one pass")
}

// ---------------------------------------------------------------------------
// Top-level.
// ---------------------------------------------------------------------------

/// Shared core for both forward and backward Exomizer-3 raw output.
///
/// The match finder, optimal parse, encoding optimization, and bit emitter are
/// direction-agnostic: they operate on `buf` (already direction-adjusted by the
/// caller) and build the stream backwards, last token first. Returns the
/// backward-built crunched stream with no final reversal applied.
///
/// `trajectories` (1..=MAX_LEVEL) is the number of best-of trajectory configs to
/// run; the first config is the no-regression anchor and the first four
/// reproduce the previously shipped output.
fn crunch_core(buf: Vec<u8>, trajectories: usize) -> Vec<u8> {
    let flags_proto = FLAGS_PROTO;
    let flags_notrait = 0i32;

    if buf.is_empty() {
        // Empty input: an empty encoding plus a single terminal node, emitting
        // the EOF gamma(16) marker and the header.
        let enc = optimal_optimize(&[]);
        let ctx = MatchCtx::new(Vec::new(), MAX_LEN, MAX_OFFSET);
        let nodes = vec![SearchNode {
            index: 0,
            m_offset: 0,
            m_len: 0,
            total_offset: 0,
            total_score: 0.0,
            prev: -1,
            latest_offset: 0,
        }];
        return do_output_backwards(&ctx, &nodes, &enc, flags_proto, true);
    }

    let mut ctx = MatchCtx::new(buf, MAX_LEN, MAX_OFFSET);

    // Best-of trajectories. Each is an independent run of the parse<->encoding
    // loop returning its smallest emission; the global minimum is kept. The
    // first config is the no-regression anchor; the rest perturb the bootstrap,
    // greedy parity, and candidate set. Trajectories share nothing mutable and
    // run on scratch threads; the result is byte-identical to a sequential
    // best-of, so it can only shrink (or tie) the anchor - never grow.
    //
    // Configs 0..3 are the original reference-cache Parity trajectories (config 0
    // is the no-regression ANCHOR; `compress(_, 4, _)` reproduces them == the
    // previously shipped output). Configs 4..5 add iteration-schedule diversity
    // (all-greedy / all-lazy) on the reference cache - an independent best-of
    // lever from the candidate set. Configs 6..7 drive the optimal parse with the
    // richer, unpruned candidate cache (K smallest offsets per length tier).
    use BootKind::*;
    use GreedyMode::*;
    let mk = |boot, flip_greedy, greedy, use_rich| TrajCfg {
        boot,
        flip_greedy,
        greedy,
        use_rich,
    };
    let cfgs: [TrajCfg; 8] = [
        // 0..3: original Parity reference-cache (ANCHOR group). `compress(_,4,_)`
        // reproduces these == the previously shipped output.
        mk(CacheEnum, false, Parity, false), // ANCHOR
        mk(CacheEnum, true, Parity, false),
        mk(CacheEnumWarm, false, Parity, false),
        mk(CacheEnumWarm, true, Parity, false),
        // 4..5: reference-cache, all-greedy / all-lazy schedules. The f32
        // tie-break converges to a different, occasionally smaller, fixpoint per
        // schedule - this is where the sub-anchor wins on the corpus come from.
        mk(CacheEnum, false, AllGreedy, false),
        mk(CacheEnum, false, AllLazy, false),
        // 6..7: richer-cache trajectories - the all-distinct-offset candidate set
        // (K smallest offsets per length tier) feeding the optimal parse, in the
        // greedy and lazy schedules. On this corpus the reference cache already
        // supplies the min offset for every economically useful length, so these
        // tie rather than shrink; they remain in the best-of so the result can
        // never regress and so any input where the richer set *does* help is
        // captured for free.
        mk(CacheEnum, false, AllGreedy, true),
        mk(CacheEnum, false, AllLazy, true),
    ];
    let n_traj = trajectories.clamp(1, cfgs.len());

    // Build the richer cache only if at least one selected trajectory uses it.
    // The chain-depth cap trades thoroughness for time; offline use favours a
    // generous cap, scaled down for very large inputs to stay tractable.
    let any_rich = cfgs[..n_traj].iter().any(|c| c.use_rich);
    if any_rich {
        let n = ctx.len;
        // The nearest occurrence (smallest offset) for each length surfaces in
        // the first handful of chain steps, so a deep walk adds candidates the
        // parse never uses (measured: deeper chains do not change any output).
        // A modest cap keeps the build tractable on large inputs.
        let max_chain = if n <= 1 << 18 { 1024 } else { 256 };
        // K=4: keep up to four smallest offsets per length tier so the parse can
        // pick whichever lands in a cheaper encoding bucket. K=4 strictly
        // contains the K=1 min-offset-per-length staircase.
        ctx.build_rich_cache_k(max_chain, 4);
    }

    let blobs: Vec<Vec<u8>> = std::thread::scope(|s| {
        let handles: Vec<_> = cfgs[..n_traj]
            .iter()
            .map(|cfg| {
                let ctx = &ctx;
                let cfg = *cfg;
                s.spawn(move || run_trajectory(ctx, flags_proto, flags_notrait, cfg))
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("exo3 trajectory thread panicked"))
            .collect()
    });

    blobs.into_iter().min_by_key(|b| b.len()).unwrap()
}

/// Compress `input` into an Exomizer-3 raw (-P39) compatible forward stream,
/// decodable by `exomizer raw -d`. Runs the full best-of ([`MAX_TRAJECTORIES`])
/// trajectories.
///
/// Forward mode reverses the input before crunching and reverses the crunched
/// output afterwards, yielding a stream the `raw -d` decruncher reads
/// front-to-back.
pub fn compress_exo3(input: &[u8]) -> Vec<u8> {
    compress_exo3_n(input, MAX_TRAJECTORIES as usize)
}

/// Forward compress with an explicit best-of trajectory count (1..=[`MAX_TRAJECTORIES`]).
fn compress_exo3_n(input: &[u8], trajectories: usize) -> Vec<u8> {
    let mut rbuf = input.to_vec();
    rbuf.reverse();

    let mut buf = crunch_core(rbuf, trajectories);
    buf.reverse();
    buf
}

/// Compress `input` into an Exomizer-3 raw (-P39) compatible backward stream,
/// decodable by `exomizer raw -d -b`. Runs the full best-of
/// ([`MAX_TRAJECTORIES`]) trajectories.
///
/// Backward mode does not reverse the input or the output. On decode,
/// `exomizer raw -d -b` reverses the input, runs the same front-to-back
/// decruncher, then reverses the output, recovering the original bytes.
pub fn compress_exo3_backward(input: &[u8]) -> Vec<u8> {
    compress_exo3_backward_n(input, MAX_TRAJECTORIES as usize)
}

/// Backward compress with an explicit best-of trajectory count (1..=[`MAX_TRAJECTORIES`]).
fn compress_exo3_backward_n(input: &[u8], trajectories: usize) -> Vec<u8> {
    crunch_core(input.to_vec(), trajectories)
}

/// Compress `input` to an Exomizer-3 raw (-P39) stream at a normalized tier
/// `level` (clamped to 1..=[`MAX_LEVEL`]). 1 = fastest (single anchor
/// trajectory), [`MAX_LEVEL`] = absolute best (full best-of-8); higher tier ⇒
/// size ≤ lower tier. The tier is mapped onto an internal trajectory count
/// (1 → 1, 2 → 4, 3 → 8) and driven through [`compress_native`]. `backward`
/// selects the backward variant (`exomizer raw -d -b`).
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    compress_native(input, trajectories_for_level(level), backward)
}

/// Compress `input` to an Exomizer-3 raw (-P39) stream, driving the exact number
/// of best-of trajectory/pass configs `trajectories` (clamped to
/// 1..=[`MAX_TRAJECTORIES`]). This is the algorithm's real knob, exposed
/// directly; [`compress`] is the normalized-tier wrapper over it. `backward`
/// selects the backward variant (`exomizer raw -d -b`).
pub fn compress_native(input: &[u8], trajectories: u32, backward: bool) -> Vec<u8> {
    let trajectories = trajectories.clamp(1, MAX_TRAJECTORIES) as usize;
    if backward {
        compress_exo3_backward_n(input, trajectories)
    } else {
        compress_exo3_n(input, trajectories)
    }
}

/// Decompress an Exomizer-3 raw (-P39) stream produced by [`compress`].
///
/// `backward` must match the value used to compress. For backward streams the
/// input is reversed, decoded front-to-back, and the output reversed.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = ExoDecoder::decode(&rev, MAX_OFFSET + 2);
        out.reverse();
        out
    } else {
        ExoDecoder::decode(input, MAX_OFFSET + 2)
    }
}

// ===========================================================================
// Raw decruncher (default -P39).
// ===========================================================================

pub struct ExoDecoder<'a> {
    inp: &'a [u8],
    pos: usize,
    bit_buffer: u8,
    lengths: Vec<(u8, u16)>,
    offsets1: Vec<(u8, u16)>,
    offsets2: Vec<(u8, u16)>,
    offsets3: Vec<(u8, u16)>,
    window: Vec<u8>,
    out: Vec<u8>,
    eof: bool,
}

impl<'a> ExoDecoder<'a> {
    fn read_byte(&mut self) -> u8 {
        if self.pos >= self.inp.len() {
            self.eof = true;
            return 0;
        }
        let v = self.inp[self.pos];
        self.pos += 1;
        v
    }
    fn rotate(&mut self, carry: i32) -> i32 {
        let carry_out = if self.bit_buffer & 0x80 != 0 { 1 } else { 0 };
        self.bit_buffer = self.bit_buffer.wrapping_shl(1);
        if carry != 0 {
            self.bit_buffer |= 0x01;
        }
        carry_out
    }
    fn read_bits(&mut self, bit_count: i32) -> i32 {
        let byte_copy = bit_count & 8;
        let mut bits = 0i32;
        let mut bc = bit_count & 7;
        while bc > 0 {
            bc -= 1;
            let mut carry = self.rotate(0);
            if self.bit_buffer == 0 {
                self.bit_buffer = self.read_byte();
                carry = self.rotate(1);
            }
            bits = (bits << 1) | carry;
        }
        if byte_copy != 0 {
            let b = self.read_byte();
            bits = (bits << 8) | b as i32;
        }
        bits
    }
    fn gen_table(&mut self, size: usize) -> Vec<(u8, u16)> {
        let mut t = Vec::with_capacity(size);
        let mut base = 1u16;
        for _ in 0..size {
            let mut bits = self.read_bits(3) as u8;
            bits |= (self.read_bits(1) as u8) << 3;
            t.push((bits, base));
            base = base.wrapping_add(1u16 << bits);
        }
        t
    }
    fn gamma(&mut self) -> i32 {
        let mut g = 0;
        while self.read_bits(1) == 0 {
            g += 1;
        }
        g
    }

    pub fn decode(inp: &'a [u8], max_offset: usize) -> Vec<u8> {
        Self::decode_with_gap(inp, max_offset).0
    }

    /// Decode and also return the in-place safety gap (bytes) the stream needs:
    /// `max(output_produced - input_consumed)` over the decode, minus its final
    /// value. `build_sfx` sizes the in-place decrunch margin from this so the
    /// write head never clobbers unread compressed bytes - offset-matches emit
    /// output without consuming input, so a late-decoded incompressible run makes
    /// the running expansion peak above its final value and a fixed margin is no
    /// longer enough. See [`max_gap_forward`] / [`max_gap_backward`].
    fn decode_with_gap(inp: &'a [u8], max_offset: usize) -> (Vec<u8>, i32) {
        let mut d = ExoDecoder {
            inp,
            pos: 0,
            bit_buffer: 0,
            lengths: Vec::new(),
            offsets1: Vec::new(),
            offsets2: Vec::new(),
            offsets3: Vec::new(),
            window: vec![0u8; max_offset.max(1)],
            out: Vec::new(),
            eof: false,
        };
        d.bit_buffer = d.read_byte();
        d.lengths = d.gen_table(16);
        d.offsets3 = d.gen_table(16);
        d.offsets2 = d.gen_table(16);
        d.offsets1 = d.gen_table(4);

        let mut reuse_offset_state: u32 = 1;
        let mut window_pos: usize = 0;
        let window_length = d.window.len();
        let mut offset: usize = 0;

        // Peak of (produced - consumed) at any token boundary. During an
        // offset-match the output advances while the input does not, so the gap
        // peaks at the match's end - the state observed at the next loop
        // iteration's top. A literal / literal-block consumes one input byte per
        // output byte, so it holds the gap flat. `d.pos` is the reader's byte
        // position; a constant offset would cancel in `max_gap - final_gap`.
        let mut max_gap = 0i32;

        let mut first = true;
        loop {
            let gap = d.out.len() as i32 - d.pos as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            if d.eof {
                break;
            }
            let is_literal;
            if first {
                is_literal = true;
                first = false;
            } else {
                is_literal = d.read_bits(1) == 1;
            }
            if is_literal {
                let c = d.read_byte();
                if d.eof {
                    break;
                }
                reuse_offset_state = (reuse_offset_state << 1) | 1;
                d.out.push(c);
                d.window[window_pos] = c;
                window_pos += 1;
                if window_pos == window_length {
                    window_pos = 0;
                }
                continue;
            }
            let length_index = d.gamma();
            if length_index == 17 {
                let mut length = (d.read_byte() as i32) << 8;
                length |= d.read_byte() as i32;
                // Mirror the 6510 copy loop exactly: it enters with X = len_lo
                // and DEX-wraps, so a low byte of 0 copies a whole extra
                // 256-byte block. The encoder never emits such lengths; if it
                // ever regresses, this mirror makes the roundtrip fail loudly
                // instead of hiding a hardware divergence.
                if length > 0 && length & 0xFF == 0 {
                    length += 256;
                }
                for _ in 0..length {
                    let c = d.read_byte();
                    d.out.push(c);
                    d.window[window_pos] = c;
                    window_pos += 1;
                    if window_pos == window_length {
                        window_pos = 0;
                    }
                }
                reuse_offset_state = (reuse_offset_state << 1) | 1;
                continue;
            } else if length_index == 16 {
                break;
            }
            let (lbits, lbase) = d.lengths[length_index as usize];
            let length = lbase as i32 + d.read_bits(lbits as i32);

            if (reuse_offset_state & 3) != 1 || d.read_bits(1) == 0 {
                let (obits, obase) = match length {
                    1 => {
                        let i = d.read_bits(2) as usize;
                        d.offsets1[i]
                    }
                    2 => {
                        let i = d.read_bits(4) as usize;
                        d.offsets2[i]
                    }
                    _ => {
                        let i = d.read_bits(4) as usize;
                        d.offsets3[i]
                    }
                };
                offset = (obase as i32 + d.read_bits(obits as i32)) as usize;
            }
            for _ in 0..length {
                let mut read_pos = window_pos as isize - offset as isize;
                if read_pos < 0 {
                    read_pos += window_length as isize;
                }
                let c = d.window[read_pos as usize];
                d.out.push(c);
                d.window[window_pos] = c;
                window_pos += 1;
                if window_pos == window_length {
                    window_pos = 0;
                }
            }
            reuse_offset_state <<= 1;
        }
        // The read head stops at the EOF marker, so use `inp.len()` (not `d.pos`)
        // for the final gap - the true end state of the packed block.
        let final_gap = d.out.len() as i32 - inp.len() as i32;
        (d.out, (max_gap - final_gap).max(0))
    }
}

/// In-place safety margin (bytes) for a FORWARD Exomizer-3 (-P39) stream: the
/// top-aligned packed block must start at least this many bytes above the output
/// end, or the decoder's write head overtakes unread compressed data. See
/// [`ExoDecoder::decode_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        ExoDecoder::decode_with_gap(stream, MAX_OFFSET + 2).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD Exomizer-3 stream. The backward
/// stream is `reverse(compress_exo3(reverse(input)))` (== `compress_exo3_backward`,
/// which is `crunch_core(input)`), and `exomizer raw -d -b` reverses the stored
/// stream before running the same front-to-back decruncher - so the gap sequence
/// is exactly a forward decode of the reversed stream.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    ExoDecoder::decode_with_gap(&rev, MAX_OFFSET + 2).1.max(0) as usize
}

/// Decode a stream and return its token list as (offset,len) pairs (offset 0 =
/// literal byte; a literal block is reported as offset 0 with its length).
/// [`trace_tokens`] biases a literal-SEQUENCE token's length by this amount so
/// it is distinguishable from a match length (a match length never reaches this
/// value: `MAX_LEN` is 65535 < 100000). [`stream_traits`] reverses the bias.
pub const LITSEQ_TOKEN_BIAS: u32 = 100_000;

pub fn trace_tokens(inp: &[u8], max_offset: usize) -> Vec<(u32, u32)> {
    let mut d = ExoDecoder {
        inp,
        pos: 0,
        bit_buffer: 0,
        lengths: Vec::new(),
        offsets1: Vec::new(),
        offsets2: Vec::new(),
        offsets3: Vec::new(),
        window: vec![0u8; max_offset.max(1)],
        out: Vec::new(),
        eof: false,
    };
    d.bit_buffer = d.read_byte();
    d.lengths = d.gen_table(16);
    d.offsets3 = d.gen_table(16);
    d.offsets2 = d.gen_table(16);
    d.offsets1 = d.gen_table(4);
    let mut toks = Vec::new();
    let mut reuse_offset_state: u32 = 1;
    let mut window_pos: usize = 0;
    let window_length = d.window.len();
    let mut offset: usize = 0;
    let mut first = true;
    loop {
        if d.eof {
            break;
        }
        let is_literal;
        if first {
            is_literal = true;
            first = false;
        } else {
            is_literal = d.read_bits(1) == 1;
        }
        if is_literal {
            let _c = d.read_byte();
            if d.eof {
                break;
            }
            reuse_offset_state = (reuse_offset_state << 1) | 1;
            toks.push((0u32, 1u32));
            window_pos += 1;
            if window_pos == window_length {
                window_pos = 0;
            }
            continue;
        }
        let length_index = d.gamma();
        if length_index == 17 {
            let mut length = (d.read_byte() as i32) << 8;
            length |= d.read_byte() as i32;
            // Mirror the 6510 lo0 wrap (see decode()).
            if length > 0 && length & 0xFF == 0 {
                length += 256;
            }
            for _ in 0..length {
                let _c = d.read_byte();
                window_pos += 1;
                if window_pos == window_length {
                    window_pos = 0;
                }
            }
            reuse_offset_state = (reuse_offset_state << 1) | 1;
            toks.push((0u32, length as u32 + LITSEQ_TOKEN_BIAS)); // literal block: biased length
            continue;
        } else if length_index == 16 {
            break;
        }
        let (lbits, lbase) = d.lengths[length_index as usize];
        let length = lbase as i32 + d.read_bits(lbits as i32);
        if (reuse_offset_state & 3) != 1 || d.read_bits(1) == 0 {
            let (obits, obase) = match length {
                1 => {
                    let i = d.read_bits(2) as usize;
                    d.offsets1[i]
                }
                2 => {
                    let i = d.read_bits(4) as usize;
                    d.offsets2[i]
                }
                _ => {
                    let i = d.read_bits(4) as usize;
                    d.offsets3[i]
                }
            };
            offset = (obase as i32 + d.read_bits(obits as i32)) as usize;
        }
        for _ in 0..length {
            window_pos += 1;
            if window_pos == window_length {
                window_pos = 0;
            }
        }
        toks.push((offset as u32, length as u32));
        reuse_offset_state <<= 1;
    }
    toks
}

/// Feature traits a specific exomizer stream exercises, driving per-crunch
/// decoder tailoring. Absent traits mark decoder sections the stream never
/// reaches, which a tailored decoder can drop while decoding the SAME stream
/// bytes (the stream stays `exomizer raw`-compatible).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExoStreamTraits {
    /// The stream contains at least one literal SEQUENCE (the multi-byte
    /// literal block, gamma index 17). When false the decoder's
    /// exit-or-literal-sequence handler is dead - only the plain end-of-stream
    /// marker reaches it.
    pub litseq: bool,
    /// Some match or literal-sequence length exceeds 256, so a decoded length's
    /// high byte (`zp_len_hi`) becomes nonzero and the 16-bit length machinery
    /// (hi-byte compute + copy-loop page repeat) is live. When false, every
    /// length fits one page and that machinery is dead.
    ///
    /// The boundary is `> 256`, not `>= 256`: the copy loop treats a low byte
    /// of 0 as a whole 256-byte page, so a length of exactly 256 decodes with
    /// `zp_len_hi == 0` (see the `len & 0xFF == 0` handling in `trace_tokens`
    /// and the encoder).
    pub len16: bool,
}

/// Measure the [`ExoStreamTraits`] of a FORWARD exomizer raw stream (the output
/// of [`compress_exo3`]) by replaying its tokens. Over-reporting a trait is
/// safe (it only forgoes a size saving); the predicate here is exact.
pub fn stream_traits(inp: &[u8]) -> ExoStreamTraits {
    let mut t = ExoStreamTraits::default();
    for (_offset, length) in trace_tokens(inp, MAX_OFFSET + 2) {
        if length >= LITSEQ_TOKEN_BIAS {
            t.litseq = true;
            if length - LITSEQ_TOKEN_BIAS > 256 {
                t.len16 = true;
            }
        } else if length > 256 {
            // Non-litseq token with a >256 length: a long match.
            t.len16 = true;
        }
        // A single literal byte is (offset 0, length 1): neither trait.
    }
    t
}

/// Measure the [`ExoStreamTraits`] of a BACKWARD exomizer raw stream (the output
/// of [`compress_exo3_backward`], as stored in an SFX). The backward stream is
/// the byte-reverse of the sequence its decoder consumes; reversing it yields
/// the forward token stream, whose traits are identical.
pub fn stream_traits_backward(inp: &[u8]) -> ExoStreamTraits {
    let mut rev = inp.to_vec();
    rev.reverse();
    stream_traits(&rev)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(input: &[u8]) {
        let comp = compress_exo3(input);
        let dec = ExoDecoder::decode(&comp, MAX_OFFSET + 2);
        assert_eq!(dec, input, "roundtrip mismatch for {} bytes", input.len());
    }

    /// Backward roundtrip, mirroring `exomizer raw -d -b`: reverse the backward
    /// stream, decode with the (forward) decruncher, then reverse the output.
    fn roundtrip_backward(input: &[u8]) {
        let comp = compress_exo3_backward(input);
        let mut rev = comp.clone();
        rev.reverse();
        let mut dec = ExoDecoder::decode(&rev, MAX_OFFSET + 2);
        dec.reverse();
        assert_eq!(
            dec,
            input,
            "backward roundtrip mismatch for {} bytes",
            input.len()
        );
    }

    #[test]
    fn t_backward_small() {
        roundtrip_backward(b"ABRACADABRA_ABRACADABRA_ABRACADABRA_HELLO_HELLO_WORLD");
    }
    #[test]
    fn t_backward_single() {
        roundtrip_backward(b"A");
    }
    #[test]
    fn t_backward_two_byte() {
        roundtrip_backward(b"AB");
    }
    #[test]
    fn t_backward_repeat() {
        roundtrip_backward(&vec![0x42u8; 1000]);
    }

    #[test]
    fn stream_traits_detects_litseq_and_len16() {
        // A long flat run compresses to one big match (offset 1, len ~4095):
        // no literal sequence, but a length well over 256.
        let flat = vec![0x41u8; 4096];
        let t = stream_traits(&compress_exo3(&flat));
        assert!(!t.litseq, "flat run should use no literal sequences");
        assert!(t.len16, "a 4096-byte run yields a match length > 256");

        // A two-byte input is just literals: neither trait.
        let tiny = stream_traits(&compress_exo3(b"AB"));
        assert!(!tiny.litseq && !tiny.len16);

        // Backward traits equal forward traits (same token set, reversed bytes).
        let back = stream_traits_backward(&compress_exo3_backward(&flat));
        assert_eq!(back, t);
    }
    #[test]
    fn t_backward_abcde() {
        roundtrip_backward(b"ABCDEABCDE");
    }
    #[test]
    fn t_backward_mixed() {
        let mut v = Vec::new();
        for i in 0..5000u32 {
            v.push((i.wrapping_mul(2654435761) >> 13) as u8);
        }
        let tail = v[100..400].to_vec();
        v.extend(tail);
        roundtrip_backward(&v);
    }

    #[test]
    fn t_small() {
        roundtrip(b"ABRACADABRA_ABRACADABRA_ABRACADABRA_HELLO_HELLO_WORLD");
    }
    #[test]
    fn t_single() {
        roundtrip(b"A");
    }
    #[test]
    fn t_repeat() {
        roundtrip(&vec![0x42u8; 1000]);
    }
    #[test]
    fn t_two_byte() {
        roundtrip(b"AB");
    }
    #[test]
    fn t_abcde() {
        roundtrip(b"ABCDEABCDE");
    }
    #[test]
    fn t_mixed() {
        let mut v = Vec::new();
        for i in 0..5000u32 {
            v.push((i.wrapping_mul(2654435761) >> 13) as u8);
        }
        let tail = v[100..400].to_vec();
        v.extend(tail);
        roundtrip(&v);
    }

    fn sample_inputs() -> Vec<Vec<u8>> {
        let mut mixed = Vec::new();
        for i in 0..5000u32 {
            mixed.push((i.wrapping_mul(2654435761) >> 13) as u8);
        }
        let tail = mixed[100..400].to_vec();
        mixed.extend(tail);
        vec![
            Vec::new(),
            b"A".to_vec(),
            b"AB".to_vec(),
            b"ABCDEABCDE".to_vec(),
            b"ABRACADABRA_ABRACADABRA_ABRACADABRA_HELLO_HELLO_WORLD".to_vec(),
            vec![0x42u8; 1000],
            mixed,
        ]
    }

    #[test]
    fn t_api_roundtrip_all_levels() {
        for input in sample_inputs() {
            // The IMPL_1LITERAL raw format always emits at least one literal, so
            // zero-length output is not representable and does not round-trip.
            if input.is_empty() {
                continue;
            }
            for level in 1..=MAX_LEVEL {
                for &backward in &[false, true] {
                    let comp = compress(&input, level, backward);
                    let dec = decompress(&comp, backward);
                    assert_eq!(
                        dec,
                        input,
                        "api roundtrip mismatch: {} bytes, level {}, backward {}",
                        input.len(),
                        level,
                        backward
                    );
                }
            }
        }
    }

    #[test]
    fn t_api_level_clamping() {
        // level 0 clamps up to tier 1; level > MAX_LEVEL clamps down to MAX_LEVEL.
        for input in sample_inputs() {
            assert_eq!(compress(&input, 0, false), compress(&input, 1, false));
            assert_eq!(
                compress(&input, 255, false),
                compress(&input, MAX_LEVEL, false)
            );
        }
    }

    #[test]
    fn t_native_clamping() {
        // trajectories 0 clamps up to 1; trajectories > MAX_TRAJECTORIES clamps
        // down to MAX_TRAJECTORIES.
        for input in sample_inputs() {
            assert_eq!(
                compress_native(&input, 0, false),
                compress_native(&input, 1, false)
            );
            assert_eq!(
                compress_native(&input, 999, false),
                compress_native(&input, MAX_TRAJECTORIES, false)
            );
        }
    }

    #[test]
    fn t_api_full_level_matches_existing() {
        // Tier MAX_LEVEL (== best-of-8) is exactly compress_exo3 / _backward (the
        // public entry points run the full best-of), and equals compress_native
        // at MAX_TRAJECTORIES.
        for input in sample_inputs() {
            assert_eq!(compress(&input, MAX_LEVEL, false), compress_exo3(&input));
            assert_eq!(
                compress(&input, MAX_LEVEL, true),
                compress_exo3_backward(&input)
            );
            assert_eq!(
                compress(&input, MAX_LEVEL, false),
                compress_native(&input, MAX_TRAJECTORIES, false)
            );
            assert_eq!(
                compress(&input, MAX_LEVEL, true),
                compress_native(&input, MAX_TRAJECTORIES, true)
            );
        }
    }

    #[test]
    fn t_tiers_map_to_native() {
        // The three public tiers are exactly native trajectory counts {1, 4, 8}.
        for input in sample_inputs() {
            for &backward in &[false, true] {
                assert_eq!(
                    compress(&input, 1, backward),
                    compress_native(&input, 1, backward)
                );
                assert_eq!(
                    compress(&input, 2, backward),
                    compress_native(&input, 4, backward)
                );
                assert_eq!(
                    compress(&input, 3, backward),
                    compress_native(&input, 8, backward)
                );
            }
        }
    }

    #[test]
    fn t_api_tier1_is_anchor() {
        // Tier 1 runs the single anchor trajectory; the full best-of (tier 3) can
        // only be <= the tier-1 output: adding trajectories never grows it.
        // Sizes are monotone non-increasing across tiers.
        for input in sample_inputs() {
            if input.is_empty() {
                continue;
            }
            for &backward in &[false, true] {
                let t1 = compress(&input, 1, backward);
                let t2 = compress(&input, 2, backward);
                let t3 = compress(&input, 3, backward);
                assert!(
                    t2.len() <= t1.len(),
                    "tier2 {} > tier1 {} ({} bytes, backward {})",
                    t2.len(),
                    t1.len(),
                    input.len(),
                    backward
                );
                assert!(
                    t3.len() <= t2.len(),
                    "tier3 {} > tier2 {} ({} bytes, backward {})",
                    t3.len(),
                    t2.len(),
                    input.len(),
                    backward
                );
                // And all decode back to the input.
                assert_eq!(decompress(&t1, backward), input);
                assert_eq!(decompress(&t2, backward), input);
                assert_eq!(decompress(&t3, backward), input);
            }
        }
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        // Incompressible data (~9-bit literals) expands, so an in-place layout
        // needs a margin far larger than the fixed 32-byte default. The gap must
        // exceed it in BOTH directions.
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        assert!(
            max_gap_forward(&compress_exo3(&noise)) > 32,
            "incompressible forward gap must exceed the fixed 32-byte margin"
        );
        assert!(
            max_gap_backward(&compress_exo3_backward(&noise)) > 32,
            "incompressible backward gap must exceed the fixed 32-byte margin"
        );
        // Highly compressible data barely expands: the default margin is fine.
        let zeros = vec![0u8; 8192];
        assert!(
            max_gap_backward(&compress_exo3_backward(&zeros)) <= 32,
            "compressible data should fit within the default margin"
        );
    }
}
