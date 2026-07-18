//! LZSA2 raw-block encoder and decoder (`lzsa -f 2 -r`).
//!
//! The encoder runs an LCP-interval match finder, supplement-match passes, a forward-arrivals
//! optimal parse with rep-offset state, a command-count merge, and byte-exact nibble emission.
//! Raw blocks cap at 64 KB of input; the encoder returns an empty vector for larger inputs.

// ---- format constants ----
const MIN_MATCH: i32 = 2;
const LITERALS_RUN_LEN: i32 = 3;
const MATCH_RUN_LEN: i32 = 7;
#[allow(dead_code)]
const MIN_OFFSET: i32 = 1;
const MAX_OFFSET: i32 = 0xffff;
const MAX_VARLEN: i32 = 0xffff;
const MAX_BLOCK: usize = 0x10000; // 64 KB raw cap

// ---- match-finder interval constants ----
const LCP_BITS: u32 = 14;
const TAG_BITS: u32 = 4;
const LCP_MAX: i32 = ((1u32 << (LCP_BITS - TAG_BITS)) - 1) as i32; // 1023
const LCP_SHIFT: u32 = 31 - LCP_BITS; // 17
const LCP_MASK: u32 = ((1u32 << LCP_BITS) - 1) << LCP_SHIFT;
const POS_MASK: u32 = (1u32 << LCP_SHIFT) - 1;
const VISITED_FLAG: u32 = 0x8000_0000;
const EXCL_VISITED_MASK: u32 = 0x7fff_ffff;

const NARRIVALS_PER_POSITION_V2_BIG: usize = 32;
const NARRIVALS_PER_POSITION_V2_MAX: usize = 64;
const NMATCHES_PER_INDEX_V2: usize = 64;

const LEAVE_ALONE_MATCH_SIZE: i32 = 300;
// Mode-switch penalty (ratio-favoring config uses 0).
const MODESWITCH_PENALTY: i32 = 0;

const INITIAL_COST: i32 = 0x4000_0000;

/// One match candidate or best-match slot. In candidate arrays `length` may carry the 0x8000
/// tag bit; mask it off to get the true length.
#[derive(Clone, Copy, Default)]
struct Match {
    length: u16,
    offset: u16,
}

// =====================================================================================
// Cost-sizing helpers: varlen field sizes in bits.
// =====================================================================================

#[inline]
fn literals_varlen_size(n: i32) -> i32 {
    if n < LITERALS_RUN_LEN {
        0
    } else if n < LITERALS_RUN_LEN + 15 {
        4
    } else if n < 256 {
        4 + 8
    } else {
        4 + 24
    }
}

#[inline]
fn match_varlen_size(n: i32) -> i32 {
    if n < MATCH_RUN_LEN {
        0
    } else if n < MATCH_RUN_LEN + 15 {
        4
    } else if n + MIN_MATCH < 256 {
        4 + 8
    } else {
        4 + 24
    }
}

#[inline]
fn offset_size(off: i32) -> i32 {
    if off <= 32 {
        4
    } else if off <= 512 {
        8
    } else if off <= 8192 + 512 {
        12
    } else {
        16
    }
}

// =====================================================================================
// Suffix array (prefix doubling).
// =====================================================================================

fn build_suffix_array(data: &[u8]) -> Vec<i32> {
    let n = data.len();
    let mut sa: Vec<i32> = (0..n as i32).collect();
    if n <= 1 {
        return sa;
    }

    // rank[i] in 0..n (compressed). Start from byte values, then compress to dense ranks.
    let mut rank: Vec<i32> = vec![0; n];
    let mut tmp = vec![0i32; n];

    // Initial sort by single byte and compress to dense ranks.
    sa.sort_by_key(|&i| data[i as usize]);
    rank[sa[0] as usize] = 0;
    for i in 1..n {
        let prev = data[sa[i - 1] as usize];
        let cur = data[sa[i] as usize];
        rank[sa[i] as usize] = rank[sa[i - 1] as usize] + if cur != prev { 1 } else { 0 };
    }

    let mut k = 1usize;
    while k < n {
        // key2[i] = rank[i+k]+1 (0 if out of range); ranks are in 0..n so key in 0..=n.
        let key2 = |i: usize| -> i32 {
            if i + k < n {
                rank[i + k] + 1
            } else {
                0
            }
        };
        let buckets = n + 2;
        // Counting sort by second key into tmp (stable over current sa order).
        let mut cnt = vec![0i32; buckets];
        for i in 0..n {
            cnt[key2(sa[i] as usize) as usize] += 1;
        }
        for i in 1..buckets {
            cnt[i] += cnt[i - 1];
        }
        for i in (0..n).rev() {
            let kk = key2(sa[i] as usize) as usize;
            cnt[kk] -= 1;
            tmp[cnt[kk] as usize] = sa[i];
        }
        // Counting sort by first key (rank[i]+1), stable on tmp order.
        let mut cnt2 = vec![0i32; buckets];
        for i in 0..n {
            cnt2[(rank[tmp[i] as usize] + 1) as usize] += 1;
        }
        for i in 1..buckets {
            cnt2[i] += cnt2[i - 1];
        }
        for i in (0..n).rev() {
            let kk = (rank[tmp[i] as usize] + 1) as usize;
            cnt2[kk] -= 1;
            sa[cnt2[kk] as usize] = tmp[i];
        }
        // Recompute compressed ranks.
        let mut newrank = vec![0i32; n];
        newrank[sa[0] as usize] = 0;
        for i in 1..n {
            let a = sa[i - 1] as usize;
            let b = sa[i] as usize;
            let mut same = rank[a] == rank[b];
            if same {
                let an = if a + k < n { rank[a + k] } else { -1 };
                let bn = if b + k < n { rank[b + k] } else { -1 };
                same = an == bn;
            }
            newrank[b] = newrank[a] + if same { 0 } else { 1 };
        }
        rank.copy_from_slice(&newrank);
        if rank[sa[n - 1] as usize] == (n as i32) - 1 {
            break;
        }
        k <<= 1;
    }
    sa
}

// =====================================================================================
// Match finder: suffix array with LCP intervals, plus per-position match enumeration
// (format version 2, window < 64 KB).
// =====================================================================================

struct MatchFinder {
    intervals: Vec<u32>,
    pos_data: Vec<u32>,
    rle_len: Vec<i32>,
}

#[inline]
fn index_tag(n: u32) -> i32 {
    (((n as u64).wrapping_mul(11400714819323198485u64)) >> (64u64 - TAG_BITS as u64)) as i32
}

impl MatchFinder {
    fn build(data: &[u8], min_match: i32) -> MatchFinder {
        let n = data.len();
        let mut intervals = vec![0u32; n.max(1)];
        let mut pos_data = vec![0u32; n.max(1)];
        let mut open_intervals = vec![0u32; (1usize << (LCP_BITS - 1)) + 1];

        if n == 0 {
            return MatchFinder {
                intervals,
                pos_data,
                rle_len: Vec::new(),
            };
        }

        let sa = build_suffix_array(data);
        for i in 0..n {
            intervals[i] = sa[i] as u32;
        }

        // PLCP via the Phi array (Karkkainen). pos_data doubles as scratch here.
        let plcp = &mut pos_data;
        let mut phi = vec![-1i32; n];
        phi[intervals[0] as usize] = -1;
        for i in 1..n {
            phi[intervals[i] as usize] = intervals[i - 1] as i32;
        }
        let mut cur_len = 0usize;
        for i in 0..n {
            if phi[i] == -1 {
                plcp[i] = 0;
                cur_len = 0;
                continue;
            }
            let p = phi[i] as usize;
            let max_len = if i > p { n - i } else { n - p };
            while cur_len < max_len && data[i + cur_len] == data[p + cur_len] {
                cur_len += 1;
            }
            plcp[i] = cur_len as u32;
            if cur_len > 0 {
                cur_len -= 1;
            }
        }

        // Pack each entry as position plus tagged LCP length.
        intervals[0] &= POS_MASK;
        for i in 1..n {
            let index = (intervals[i] & POS_MASK) as usize;
            let mut len = plcp[index] as i32;
            if len < min_match {
                len = 0;
            }
            if len > LCP_MAX {
                len = LCP_MAX;
            }
            let tagged_len = if len != 0 {
                (len << TAG_BITS) | (index_tag(index as u32) & ((1 << TAG_BITS) - 1))
            } else {
                0
            };
            intervals[i] = (index as u32) | ((tagged_len as u32) << LCP_SHIFT);
        }

        // Build the LCP-interval tree. Reset pos_data, which held PLCP scratch above.
        let mut pos_data = vec![0u32; n];
        let sa_and_lcp = &mut intervals;
        let mut next_interval_idx: u32;
        let mut top = 0usize;
        let mut prev_pos = sa_and_lcp[0] & POS_MASK;
        open_intervals[0] = 0;
        sa_and_lcp[0] = 0;
        next_interval_idx = 1;

        for r in 1..n {
            let next_pos = sa_and_lcp[r] & POS_MASK;
            let next_lcp = sa_and_lcp[r] & LCP_MASK;
            let top_lcp = open_intervals[top] & LCP_MASK;

            if next_lcp == top_lcp {
                pos_data[prev_pos as usize] = open_intervals[top];
            } else if next_lcp > top_lcp {
                top += 1;
                open_intervals[top] = next_lcp | next_interval_idx;
                next_interval_idx += 1;
                pos_data[prev_pos as usize] = open_intervals[top];
            } else {
                pos_data[prev_pos as usize] = open_intervals[top];
                loop {
                    let closed_interval_idx = (open_intervals[top] & POS_MASK) as usize;
                    top -= 1;
                    let superinterval_lcp = open_intervals[top] & LCP_MASK;
                    if next_lcp == superinterval_lcp {
                        sa_and_lcp[closed_interval_idx] = open_intervals[top];
                        break;
                    } else if next_lcp > superinterval_lcp {
                        top += 1;
                        open_intervals[top] = next_lcp | next_interval_idx;
                        next_interval_idx += 1;
                        sa_and_lcp[closed_interval_idx] = open_intervals[top];
                        break;
                    } else {
                        sa_and_lcp[closed_interval_idx] = open_intervals[top];
                    }
                }
            }
            prev_pos = next_pos;
        }
        pos_data[prev_pos as usize] = open_intervals[top];
        while top > 0 {
            let idx = (open_intervals[top] & POS_MASK) as usize;
            sa_and_lcp[idx] = open_intervals[top - 1];
            top -= 1;
        }

        // Run-length table: remaining run of the same byte starting at each position.
        let mut rle_len = vec![0i32; n];
        let mut i = 0usize;
        while i < n {
            let start = i;
            let c = data[start];
            i += 1;
            while i < n && data[i] == c {
                i += 1;
            }
            let mut j = start;
            while j < i {
                rle_len[j] = (i - j) as i32;
                j += 1;
            }
        }

        MatchFinder {
            intervals,
            pos_data,
            rle_len,
        }
    }

    /// Enumerate matches at `offset` (format version 2, window < 64 KB). Writes up to
    /// `max_matches` matches into `out` and returns the count.
    fn find_matches_at(
        &mut self,
        offset: usize,
        out: &mut [Match],
        max_matches: usize,
        in_window_size: usize,
    ) -> usize {
        let intervals = &mut self.intervals;
        let pos_data = &mut self.pos_data;

        let mut r#ref = pos_data[offset];
        pos_data[offset] = 0;

        let mut super_ref;
        // Ascend until visited/root/child-of-root.
        loop {
            super_ref = intervals[(r#ref & POS_MASK) as usize];
            if super_ref & LCP_MASK == 0 {
                break;
            }
            intervals[(r#ref & POS_MASK) as usize] = (offset as u32) | VISITED_FLAG;
            r#ref = super_ref;
        }

        if super_ref == 0 {
            if r#ref != 0 {
                intervals[(r#ref & POS_MASK) as usize] = (offset as u32) | VISITED_FLAG;
            }
            return 0;
        }

        let mut match_pos = (super_ref & EXCL_VISITED_MASK) as usize;
        let mut nmatch = 0usize;
        let mut prev_offset: u32 = 0;
        let small = in_window_size < 65536;

        // First (closest) match.
        if small && nmatch < max_matches {
            let match_offset = (offset - match_pos) as u32;
            if match_offset <= MAX_OFFSET as u32 {
                out[nmatch].length = (r#ref >> (LCP_SHIFT + TAG_BITS)) as u16;
                out[nmatch].offset = match_offset as u16;
                nmatch += 1;
                prev_offset = match_offset;
            }
        }

        loop {
            super_ref = pos_data[match_pos];
            if super_ref > r#ref {
                match_pos =
                    (intervals[(super_ref & POS_MASK) as usize] & EXCL_VISITED_MASK) as usize;
                if small && nmatch < max_matches {
                    let match_offset = (offset - match_pos) as u32;
                    if match_offset <= MAX_OFFSET as u32 {
                        out[nmatch].length = ((r#ref >> (LCP_SHIFT + TAG_BITS)) as u16) | 0x8000;
                        out[nmatch].offset = match_offset as u16;
                        nmatch += 1;
                        prev_offset = match_offset;
                    }
                }
            }

            loop {
                super_ref = pos_data[match_pos];
                if super_ref > r#ref {
                    match_pos =
                        (intervals[(super_ref & POS_MASK) as usize] & EXCL_VISITED_MASK) as usize;
                } else {
                    break;
                }
            }
            intervals[(r#ref & POS_MASK) as usize] = (offset as u32) | VISITED_FLAG;
            pos_data[match_pos] = r#ref;

            if nmatch < max_matches {
                let match_offset = (offset - match_pos) as u32;
                if match_offset <= MAX_OFFSET as u32 && match_offset != prev_offset {
                    out[nmatch].length = (r#ref >> (LCP_SHIFT + TAG_BITS)) as u16;
                    out[nmatch].offset = match_offset as u16;
                    nmatch += 1;
                    prev_offset = match_offset;
                }
            }

            if super_ref == 0 {
                break;
            }
            r#ref = super_ref;
            match_pos = (intervals[(r#ref & POS_MASK) as usize] & EXCL_VISITED_MASK) as usize;

            if small && nmatch < max_matches {
                let match_offset = (offset - match_pos) as u32;
                if match_offset <= MAX_OFFSET as u32 {
                    let match_len = (r#ref >> (LCP_SHIFT + TAG_BITS)) as u16;
                    if match_len > 2 {
                        out[nmatch].length = match_len | 0x8000;
                        out[nmatch].offset = match_offset as u16;
                        nmatch += 1;
                        prev_offset = match_offset;
                    }
                }
            }
        }

        nmatch
    }
}

// =====================================================================================
// Forward-arrivals optimal parse with rep-offset state.
// =====================================================================================

#[derive(Clone, Copy)]
struct Arrival {
    cost: i32,
    rep_offset: i32,
    from_slot: i32,
    from_pos: i32,
    rep_len: i32,
    match_len: i32,
    num_literals: i32,
    rep_pos: i32,
    score: i32,
}

impl Default for Arrival {
    fn default() -> Self {
        Arrival {
            cost: INITIAL_COST,
            rep_offset: 0,
            from_slot: 0,
            from_pos: 0,
            rep_len: 0,
            match_len: 0,
            num_literals: 0,
            rep_pos: 0,
            score: 0,
        }
    }
}

/// Whole-block compressor state.
struct Compressor {
    /// Match candidates, indexed `pos * nmatches + m`.
    matches: Vec<Match>,
    best_match: Vec<Match>,
    arrival: Vec<Arrival>,
    /// Scratch for forward-rep insertion: one Match per position.
    visited: Vec<Match>,
    min_match_size: i32,
    /// Per-position match-table stride. 64 reproduces native LZSA2 exactly (the anchor); a larger
    /// value feeds the richer, uncapped candidate set into the same parse.
    nmatches: usize,
}

impl Compressor {
    fn with_capacity(n: usize, nmatches: usize) -> Compressor {
        Compressor {
            matches: vec![Match::default(); n * nmatches],
            best_match: vec![Match::default(); n.max(1)],
            arrival: vec![Arrival::default(); (n + 1) * NARRIVALS_PER_POSITION_V2_MAX],
            visited: vec![Match::default(); n.max(1)],
            min_match_size: MIN_MATCH,
            nmatches,
        }
    }
}

/// Recursively insert forward rep-match candidates seeded from the arrivals at position `i`.
fn insert_forward_match(
    c: &mut Compressor,
    data: &[u8],
    mf_rle: &[i32],
    i: usize,
    match_offset: i32,
    end_offset: usize,
    depth: i32,
) {
    let arr_base = i * NARRIVALS_PER_POSITION_V2_MAX;
    let n_big = NARRIVALS_PER_POSITION_V2_BIG;

    for j in 0..n_big {
        if c.arrival[arr_base + j].from_slot == 0 {
            break;
        }
        let rep_offset = c.arrival[arr_base + j].rep_offset;
        if match_offset == rep_offset {
            continue;
        }
        let rep_len = c.arrival[arr_base + j].rep_len;
        let rep_pos = c.arrival[arr_base + j].rep_pos;

        if rep_pos >= 0 && (rep_pos + rep_len) <= end_offset as i32 {
            let rp = rep_pos as usize;
            if c.visited[rp].offset as i32 != match_offset
                || (c.visited[rp].length as i32) > rep_len
            {
                c.visited[rp].length = 0;
                c.visited[rp].offset = match_offset as u16;

                let nmatches = c.nmatches;
                let fwd_base = rep_pos as usize * nmatches;
                if c.matches[fwd_base + nmatches - 1].length == 0 {
                    if rep_pos >= match_offset {
                        // Require the first two bytes at this offset to match.
                        let s = rep_pos as usize;
                        if s >= 2
                            && data[s] == data[s - match_offset as usize]
                            && data[s + 1] == data[s + 1 - match_offset as usize]
                        {
                            let len0 = mf_rle[s - match_offset as usize];
                            let len1 = mf_rle[s];
                            let min_len = len0.min(len1);

                            let extra_ok = if min_len >= rep_len {
                                true
                            } else {
                                // Compare the bytes beyond the shared run, up to rep_len.
                                let a = s + min_len as usize;
                                let cnt = (rep_len - min_len) as usize;
                                (0..cnt).all(|t| data[a + t] == data[a + t - match_offset as usize])
                            };

                            if extra_ok {
                                // Find the slot already holding this offset, or the first empty one.
                                let mut rslot = 0usize;
                                while c.matches[fwd_base + rslot].length != 0 {
                                    if c.matches[fwd_base + rslot].offset as i32 == match_offset {
                                        break;
                                    }
                                    rslot += 1;
                                }
                                if c.matches[fwd_base + rslot].length == 0 {
                                    if rep_len >= MIN_MATCH {
                                        if rep_offset != 0 {
                                            let mut max_rep_len = end_offset as i32 - rep_pos;
                                            if max_rep_len > LCP_MAX {
                                                max_rep_len = LCP_MAX;
                                            }
                                            let cur_rep_len =
                                                if min_len > rep_len { min_len } else { rep_len };
                                            let window_max = s + max_rep_len as usize;
                                            let mut p = s + cur_rep_len as usize;
                                            if p > window_max {
                                                p = window_max;
                                            }
                                            while p < window_max
                                                && data[p] == data[p - match_offset as usize]
                                            {
                                                p += 1;
                                            }
                                            c.matches[fwd_base + rslot].length = (p - s) as u16;
                                            c.matches[fwd_base + rslot].offset =
                                                match_offset as u16;

                                            if depth < 9 {
                                                insert_forward_match(
                                                    c,
                                                    data,
                                                    mf_rle,
                                                    rep_pos as usize,
                                                    match_offset,
                                                    end_offset,
                                                    depth + 1,
                                                );
                                            }
                                        }
                                    }
                                }
                            } else {
                                c.visited[rp].length = rep_len as u16;
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Forward-arrivals optimal parse over the match candidates.
fn optimize_forward(
    c: &mut Compressor,
    data: &[u8],
    mf_rle: &[i32],
    end_offset: usize,
    reduce: bool,
    insert_forward_reps: bool,
    n_arrivals: usize,
) {
    let mode_switch_penalty = MODESWITCH_PENALTY;
    let min_match_size = c.min_match_size;
    let disable_score = if reduce { 0 } else { 2 * MAX_BLOCK as i32 };
    let n_max_rep_inserted_len = if reduce { LEAVE_ALONE_MATCH_SIZE } else { 0 };
    let n_leave_alone = LEAVE_ALONE_MATCH_SIZE;

    let stride = NARRIVALS_PER_POSITION_V2_MAX;

    // Clear all arrival rows for positions 0..=end_offset.
    for p in 0..=end_offset {
        for j in 0..stride {
            c.arrival[p * stride + j] = Arrival::default();
        }
    }
    c.arrival[0].cost = 0;
    c.arrival[0].from_slot = -1;

    if insert_forward_reps {
        for v in c.visited.iter_mut() {
            *v = Match::default();
        }
    }

    for i in 0..end_offset {
        let cur_base = i * stride;
        let dest_lit_base = (i + 1) * stride;

        // ---- literal step: relax i -> i+1 ----
        let mut num_arrivals_for_pos = 0usize;
        for j in 0..n_arrivals {
            if c.arrival[cur_base + j].from_slot == 0 {
                break;
            }
            num_arrivals_for_pos = j + 1;
            let prev_cost = c.arrival[cur_base + j].cost;
            let mut coding_choice_cost = prev_cost + 8;
            let score = c.arrival[cur_base + j].score + 1 - disable_score;
            let num_literals = c.arrival[cur_base + j].num_literals + 1;
            let rep_offset = c.arrival[cur_base + j].rep_offset;
            let rep_len = c.arrival[cur_base + j].rep_len;
            let rep_pos = c.arrival[cur_base + j].rep_pos;

            match num_literals {
                1 => coding_choice_cost += mode_switch_penalty,
                x if x == LITERALS_RUN_LEN => coding_choice_cost += 4,
                x if x == LITERALS_RUN_LEN + 15 => coding_choice_cost += 8,
                256 => coding_choice_cost += 16,
                _ => {}
            }

            let last = n_arrivals - 1;
            if coding_choice_cost < c.arrival[dest_lit_base + last].cost
                || (coding_choice_cost == c.arrival[dest_lit_base + last].cost
                    && score < c.arrival[dest_lit_base + last].score
                    && rep_offset != c.arrival[dest_lit_base + last].rep_offset)
            {
                let mut exists = false;
                let mut nn = 0usize;
                while c.arrival[dest_lit_base + nn].cost < coding_choice_cost {
                    if c.arrival[dest_lit_base + nn].rep_offset == rep_offset {
                        exists = true;
                        break;
                    }
                    nn += 1;
                }
                if !exists {
                    while nn < n_arrivals
                        && c.arrival[dest_lit_base + nn].cost == coding_choice_cost
                        && score >= c.arrival[dest_lit_base + nn].score
                    {
                        if c.arrival[dest_lit_base + nn].rep_offset == rep_offset {
                            exists = true;
                            break;
                        }
                        nn += 1;
                    }
                    if !exists && nn < n_arrivals {
                        let mut z = nn;
                        while z < n_arrivals - 1
                            && c.arrival[dest_lit_base + z].cost == coding_choice_cost
                        {
                            if c.arrival[dest_lit_base + z].rep_offset == rep_offset {
                                exists = true;
                                break;
                            }
                            z += 1;
                        }
                        if !exists {
                            while z < n_arrivals - 1 && c.arrival[dest_lit_base + z].from_slot != 0
                            {
                                if c.arrival[dest_lit_base + z].rep_offset == rep_offset {
                                    break;
                                }
                                z += 1;
                            }
                            // Shift the arrivals up to make room at slot nn.
                            let cnt = z - nn;
                            for t in (0..cnt).rev() {
                                c.arrival[dest_lit_base + nn + 1 + t] =
                                    c.arrival[dest_lit_base + nn + t];
                            }
                            let d = &mut c.arrival[dest_lit_base + nn];
                            d.cost = coding_choice_cost;
                            d.rep_offset = rep_offset;
                            d.from_slot = (j + 1) as i32;
                            d.from_pos = i as i32;
                            d.rep_len = rep_len;
                            d.match_len = 0;
                            d.num_literals = num_literals;
                            d.rep_pos = rep_pos;
                            d.score = score + disable_score;
                        }
                    }
                }
            }
        }

        // ---- gather rep-match arrivals at position i ----
        let mut rep_idx_and_len: Vec<i32> = Vec::with_capacity(num_arrivals_for_pos * 2 + 1);
        let mut max_overall_rep_len = 0i32;
        let min_overall_rep_len_init = 0i32;

        if i + MIN_MATCH as usize <= end_offset {
            let mut max_rep_len_for_pos = (end_offset - i) as i32;
            if max_rep_len_for_pos > LCP_MAX {
                max_rep_len_for_pos = LCP_MAX;
            }
            let window_max = i + max_rep_len_for_pos as usize;

            for j in 0..num_arrivals_for_pos {
                let rep_offset = c.arrival[cur_base + j].rep_offset;
                if i >= rep_offset as usize && rep_offset != 0 {
                    let ro = rep_offset as usize;
                    if data[i] == data[i - ro] && data[i + 1] == data[i + 1 - ro] {
                        let len0 = mf_rle[i - ro];
                        let len1 = mf_rle[i];
                        let mut min_len = len0.min(len1);
                        if min_len > max_rep_len_for_pos {
                            min_len = max_rep_len_for_pos;
                        }
                        let mut p = i + min_len as usize;
                        while p < window_max && data[p] == data[p - ro] {
                            p += 1;
                        }
                        let cur_rep_len = (p - i) as i32;
                        if max_overall_rep_len < cur_rep_len {
                            max_overall_rep_len = cur_rep_len;
                        }
                        rep_idx_and_len.push(j as i32);
                        rep_idx_and_len.push(cur_rep_len);
                    }
                }
            }
        }
        rep_idx_and_len.push(-1);

        // rep-slot and rep-len handled masks
        let mask_bytes = ((LCP_MAX + 1) / 8) as usize;
        let mut rep_slot_handled_mask = if !reduce {
            vec![0u8; n_arrivals * mask_bytes]
        } else {
            Vec::new()
        };
        let mut rep_len_handled_mask = vec![0u8; mask_bytes];

        let mut min_overall_rep_len = min_overall_rep_len_init;

        // ---- match steps ----
        let nmatches = c.nmatches;
        let mut m = 0usize;
        while m < nmatches {
            let raw = c.matches[i * nmatches + m];
            if raw.length == 0 {
                break;
            }
            let mut match_len = (raw.length & 0x7fff) as i32;
            let match_offset = raw.offset as i32;
            if (i as i32 + match_len) > end_offset as i32 {
                match_len = end_offset as i32 - i as i32;
            }

            if insert_forward_reps {
                insert_forward_match(c, data, mf_rle, i, match_offset, end_offset, 0);
            }

            // find first arrival with rep_offset != match_offset (for non-rep coding)
            let mut non_rep_idx: i32 = -1;
            let mut no_rep_off_cost = 0i32;
            let mut no_rep_score = 0i32;
            for j in 0..num_arrivals_for_pos {
                if match_offset != c.arrival[cur_base + j].rep_offset {
                    let prev_cost = c.arrival[cur_base + j].cost;
                    let score_penalty = 3 + (raw.length >> 15) as i32;
                    no_rep_off_cost = prev_cost;
                    if c.arrival[cur_base + j].num_literals == 0 {
                        no_rep_off_cost += mode_switch_penalty;
                    }
                    no_rep_off_cost += offset_size(match_offset);
                    no_rep_score = c.arrival[cur_base + j].score + score_penalty - disable_score;
                    non_rep_idx = j as i32;
                    break;
                }
            }

            let starting_match_len;
            let mut match_len_cost;
            if match_len >= n_leave_alone {
                starting_match_len = match_len;
                match_len_cost = 4 + 24 + 8;
            } else {
                starting_match_len = min_match_size;
                match_len_cost = 8;
            }

            let mut k = starting_match_len;
            while k <= match_len {
                if k == MATCH_RUN_LEN + MIN_MATCH {
                    match_len_cost = 4 + 8;
                } else if k == MATCH_RUN_LEN + 15 + MIN_MATCH {
                    match_len_cost = 4 + 8 + 8;
                } else if k == 256 {
                    match_len_cost = 4 + 24 + 8;
                }

                let dest_base = (i + k as usize) * stride;

                // ---- insert non-repmatch candidate ----
                if non_rep_idx >= 0 {
                    let coding_choice_cost = match_len_cost + no_rep_off_cost;
                    let last = n_arrivals - 1;
                    let secondlast = n_arrivals - 2;
                    if coding_choice_cost < c.arrival[dest_base + secondlast].cost
                        || (coding_choice_cost == c.arrival[dest_base + secondlast].cost
                            && no_rep_score < c.arrival[dest_base + secondlast].score
                            && (coding_choice_cost != c.arrival[dest_base + last].cost
                                || match_offset != c.arrival[dest_base + last].rep_offset))
                    {
                        let mut exists = false;
                        let mut nn = 0usize;
                        while c.arrival[dest_base + nn].cost < coding_choice_cost {
                            if c.arrival[dest_base + nn].rep_offset == match_offset {
                                exists = true;
                                break;
                            }
                            nn += 1;
                        }
                        if !exists {
                            while nn < n_arrivals
                                && c.arrival[dest_base + nn].cost == coding_choice_cost
                                && no_rep_score >= c.arrival[dest_base + nn].score
                            {
                                if c.arrival[dest_base + nn].rep_offset == match_offset {
                                    exists = true;
                                    break;
                                }
                                nn += 1;
                            }
                            if !exists && nn < n_arrivals - 1 {
                                let mut z = nn;
                                while z < n_arrivals - 1
                                    && c.arrival[dest_base + z].cost == coding_choice_cost
                                {
                                    if c.arrival[dest_base + z].rep_offset == match_offset {
                                        if !insert_forward_reps
                                            || c.arrival[dest_base + last].from_slot != 0
                                            || c.arrival[dest_base + z].rep_pos >= i as i32
                                        {
                                            exists = true;
                                        }
                                        break;
                                    }
                                    z += 1;
                                }
                                if !exists {
                                    while z < n_arrivals - 1
                                        && c.arrival[dest_base + z].from_slot != 0
                                    {
                                        if c.arrival[dest_base + z].rep_offset == match_offset {
                                            break;
                                        }
                                        z += 1;
                                    }
                                    let cnt = z - nn;
                                    for t in (0..cnt).rev() {
                                        c.arrival[dest_base + nn + 1 + t] =
                                            c.arrival[dest_base + nn + t];
                                    }
                                    let d = &mut c.arrival[dest_base + nn];
                                    d.cost = coding_choice_cost;
                                    d.rep_offset = match_offset;
                                    d.from_slot = non_rep_idx + 1;
                                    d.from_pos = i as i32;
                                    d.rep_len = k;
                                    d.match_len = k;
                                    d.num_literals = 0;
                                    d.rep_pos = i as i32;
                                    d.score = no_rep_score + disable_score;
                                    rep_len_handled_mask[(k >> 3) as usize] &=
                                        !(((1 ^ (reduce as i32)) as u8) << (k & 7));
                                }
                            }
                        }
                    }
                }

                // ---- insert repmatch candidates ----
                if k > min_overall_rep_len
                    && k <= max_overall_rep_len
                    && (rep_len_handled_mask[(k >> 3) as usize] & (1 << (k & 7))) == 0
                {
                    rep_len_handled_mask[(k >> 3) as usize] |= 1 << (k & 7);

                    let mut cur = 0usize;
                    loop {
                        let j = rep_idx_and_len[cur];
                        if j < 0 {
                            break;
                        }
                        let rep_avail_len = rep_idx_and_len[cur + 1];
                        if rep_avail_len >= k {
                            let j = j as usize;
                            let mask_offset = (j << 7) + (k >> 3) as usize;
                            let slot_handled = if reduce {
                                false
                            } else {
                                (rep_slot_handled_mask[mask_offset] & (1 << (k & 7))) != 0
                            };
                            if !slot_handled {
                                let score = c.arrival[cur_base + j].score + 2 - disable_score;
                                let rep_offset = c.arrival[cur_base + j].rep_offset;
                                let last = n_arrivals - 1;

                                if rep_offset != c.arrival[dest_base + last].rep_offset {
                                    let prev_cost = c.arrival[cur_base + j].cost;
                                    let rep_coding_choice_cost = prev_cost + match_len_cost;

                                    if rep_coding_choice_cost < c.arrival[dest_base + last].cost
                                        || (rep_coding_choice_cost
                                            == c.arrival[dest_base + last].cost
                                            && score < c.arrival[dest_base + last].score)
                                    {
                                        let mut exists = false;
                                        let mut nn = 0usize;
                                        while c.arrival[dest_base + nn].cost
                                            < rep_coding_choice_cost
                                        {
                                            if c.arrival[dest_base + nn].rep_offset == rep_offset {
                                                exists = true;
                                                if !reduce {
                                                    rep_slot_handled_mask[mask_offset] |=
                                                        1 << (k & 7);
                                                }
                                                break;
                                            }
                                            nn += 1;
                                        }
                                        if !exists {
                                            while nn < n_arrivals
                                                && c.arrival[dest_base + nn].cost
                                                    == rep_coding_choice_cost
                                                && score >= c.arrival[dest_base + nn].score
                                            {
                                                if c.arrival[dest_base + nn].rep_offset
                                                    == rep_offset
                                                {
                                                    exists = true;
                                                    break;
                                                }
                                                nn += 1;
                                            }
                                            if !exists && nn < n_arrivals {
                                                let mut z = nn;
                                                while z < n_arrivals - 1
                                                    && c.arrival[dest_base + z].cost
                                                        == rep_coding_choice_cost
                                                {
                                                    if c.arrival[dest_base + z].rep_offset
                                                        == rep_offset
                                                    {
                                                        exists = true;
                                                        break;
                                                    }
                                                    z += 1;
                                                }
                                                if !exists {
                                                    while z < n_arrivals - 1
                                                        && c.arrival[dest_base + z].from_slot != 0
                                                    {
                                                        if c.arrival[dest_base + z].rep_offset
                                                            == rep_offset
                                                        {
                                                            break;
                                                        }
                                                        z += 1;
                                                    }
                                                    let cnt = z - nn;
                                                    for t in (0..cnt).rev() {
                                                        c.arrival[dest_base + nn + 1 + t] =
                                                            c.arrival[dest_base + nn + t];
                                                    }
                                                    let d = &mut c.arrival[dest_base + nn];
                                                    d.cost = rep_coding_choice_cost;
                                                    d.rep_offset = rep_offset;
                                                    d.from_slot = (j + 1) as i32;
                                                    d.from_pos = i as i32;
                                                    d.rep_len = k;
                                                    d.match_len = k;
                                                    d.num_literals = 0;
                                                    d.rep_pos = i as i32;
                                                    d.score = score + disable_score;
                                                    rep_len_handled_mask[(k >> 3) as usize] &=
                                                        !(((1 ^ (reduce as i32)) as u8) << (k & 7));
                                                }
                                            }
                                        }
                                    } else {
                                        break;
                                    }
                                }
                            }
                        }
                        cur += 2;
                    }

                    if k < n_max_rep_inserted_len {
                        min_overall_rep_len = k;
                    }
                }

                k += 1;
            }

            if match_len >= LCP_MAX {
                let next_below = m + 1 >= nmatches
                    || ((c.matches[i * nmatches + m + 1].length & 0x7fff) as i32) < LCP_MAX;
                if next_below {
                    break;
                }
            }
            m += 1;
        }
    }

    // ---- back-trace best_match (only when not inserting forward reps, i.e. final pass) ----
    if !insert_forward_reps {
        let stride = NARRIVALS_PER_POSITION_V2_MAX;
        let mut idx = end_offset * stride; // slot 0 of terminal
                                           // emulate: end_arrival = &arrival[endOffset<<shift]; follow from_slot/from_pos
        let mut cur = c.arrival[idx];
        while cur.from_slot > 0 && (cur.from_pos as usize) < end_offset {
            let fp = cur.from_pos as usize;
            c.best_match[fp].length = cur.match_len as u16;
            c.best_match[fp].offset = if cur.match_len != 0 {
                cur.rep_offset as u16
            } else {
                0
            };
            idx = fp * stride + (cur.from_slot as usize - 1);
            cur = c.arrival[idx];
        }
    }
}

// =====================================================================================
// Command-count reduction.
// =====================================================================================

fn optimize_command_count(c: &mut Compressor, data: &[u8], end_offset: usize) -> bool {
    let bm = &mut c.best_match;
    let mut i = 0usize;
    let mut num_literals = 0i32;
    let mut prev_rep_match_offset = 0i32;
    let mut rep_match_offset = 0i32;
    let mut rep_match_len = 0i32;
    let mut rep_index = 0i32;
    let mut did_reduce = false;

    while i < end_offset {
        let m_len = bm[i].length as i32;

        // Merge a single-literal gap into a following match by extending it backward by 1.
        if m_len == 0
            && (i + 1) < end_offset
            && (bm[i + 1].length as i32) >= MIN_MATCH
            && (bm[i + 1].length as i32) < MAX_VARLEN
            && bm[i + 1].offset != 0
            && i as i32 >= bm[i + 1].offset as i32
            && (i as i32 + bm[i + 1].length as i32 + 1) <= end_offset as i32
        {
            let off = bm[i + 1].offset as usize;
            let ln = bm[i + 1].length as usize;
            // Check the match still holds when extended back over the literal.
            let eq = (0..ln + 1).all(|t| data[i - off + t] == data[i + t]);
            if eq {
                let cur_len_size = match_varlen_size(bm[i + 1].length as i32 - MIN_MATCH);
                let reduced_len_size = match_varlen_size(bm[i + 1].length as i32 + 1 - MIN_MATCH);
                if (reduced_len_size - cur_len_size) <= 8 {
                    bm[i].length = bm[i + 1].length + 1;
                    bm[i].offset = bm[i + 1].offset;
                    bm[i + 1].length = 0;
                    bm[i + 1].offset = 0;
                    did_reduce = true;
                    continue;
                }
            }
        }

        if (bm[i].length as i32) >= MIN_MATCH {
            let p_len = bm[i].length as i32;
            let p_off = bm[i].offset as i32;

            if (i + p_len as usize) < end_offset {
                let mut next_index = i + p_len as usize;
                let mut next_literals = 0i32;
                while next_index < end_offset && (bm[next_index].length as i32) < MIN_MATCH {
                    next_literals += 1;
                    next_index += 1;
                }

                if next_index < end_offset {
                    let next_off = bm[next_index].offset as i32;

                    // Try to turn this match into a rep of the previous offset.
                    if rep_match_offset != 0
                        && p_off != rep_match_offset
                        && (next_off != p_off || offset_size(p_off) > offset_size(next_off))
                    {
                        if i as i32 >= rep_match_offset
                            && (0..p_len as usize)
                                .all(|t| data[i - rep_match_offset as usize + t] == data[i + t])
                        {
                            bm[i].offset = rep_match_offset as u16;
                            did_reduce = true;
                        }
                    }

                    let p_off = bm[i].offset as i32; // may have changed above

                    // Try to gain a match forward as well.
                    if next_off != 0 && p_off != next_off {
                        if i as i32 >= next_off && (i as i32 + p_len) <= end_offset as i32 {
                            let mut max_len = 0i32;
                            while max_len < p_len
                                && data[i + max_len as usize - next_off as usize]
                                    == data[i + max_len as usize]
                            {
                                max_len += 1;
                            }
                            if max_len >= p_len {
                                bm[i].offset = next_off as u16;
                                did_reduce = true;
                            } else if max_len >= 2 && p_off != rep_match_offset {
                                let mut partial_before = match_varlen_size(p_len - MIN_MATCH);
                                partial_before += offset_size(p_off);
                                partial_before += literals_varlen_size(next_literals);
                                partial_before += offset_size(next_off);

                                let mut partial_after = match_varlen_size(max_len - MIN_MATCH);
                                partial_after +=
                                    literals_varlen_size(next_literals + (p_len - max_len))
                                        + ((p_len - max_len) << 3);
                                if rep_match_offset != next_off {
                                    partial_after += offset_size(next_off);
                                }

                                if partial_after < partial_before {
                                    let match_len_old = p_len;
                                    bm[i].length = max_len as u16;
                                    bm[i].offset = next_off as u16;
                                    for j in max_len..match_len_old {
                                        bm[i + j as usize].length = 0;
                                    }
                                    did_reduce = true;
                                }
                            }
                        }
                    }

                    let p_len = bm[i].length as i32;
                    let p_off = bm[i].offset as i32;

                    if p_len < 9 {
                        let mut cur_command_size = 8
                            + literals_varlen_size(num_literals)
                            + match_varlen_size(p_len - MIN_MATCH);
                        if p_off != rep_match_offset {
                            cur_command_size += offset_size(p_off);
                        }

                        let mut next_command_size = 8
                            + literals_varlen_size(next_literals)
                            + match_varlen_size(bm[next_index].length as i32 - MIN_MATCH);
                        if next_off != p_off {
                            next_command_size += offset_size(next_off);
                        }

                        let original_combined = cur_command_size + next_command_size;

                        let mut reduced_command_size = (p_len << 3)
                            + 8
                            + literals_varlen_size(num_literals + p_len + next_literals)
                            + match_varlen_size(bm[next_index].length as i32 - MIN_MATCH);
                        if next_off != rep_match_offset {
                            reduced_command_size += offset_size(next_off);
                        }

                        let mut replace_rep_offset = false;
                        if rep_match_offset != 0
                            && rep_match_offset != prev_rep_match_offset
                            && rep_match_len >= MIN_MATCH
                            && rep_match_offset != next_off
                            && rep_index >= next_off
                            && (rep_index - next_off + rep_match_len) <= end_offset as i32
                            && (0..rep_match_len as usize).all(|t| {
                                data[(rep_index - rep_match_offset) as usize + t]
                                    == data[(rep_index - next_off) as usize + t]
                            })
                        {
                            replace_rep_offset = true;
                            reduced_command_size -= offset_size(rep_match_offset);
                        }

                        if original_combined >= reduced_command_size {
                            let match_len_old = p_len;
                            for j in 0..match_len_old {
                                bm[i + j as usize].length = 0;
                            }
                            did_reduce = true;
                            if replace_rep_offset {
                                bm[next_index].offset = next_off as u16;
                                rep_match_offset = next_off;
                            }
                            continue;
                        }
                    }
                }
            }

            // Join two adjacent matches (with the second reachable at its own offset).
            let p_len = bm[i].length as i32;
            let p_off = bm[i].offset as i32;
            if (i + p_len as usize) < end_offset
                && p_off != 0
                && p_len >= MIN_MATCH
                && bm[i + p_len as usize].offset != 0
                && (bm[i + p_len as usize].length as i32) >= MIN_MATCH
                && (p_len + bm[i + p_len as usize].length as i32) <= MAX_VARLEN
                && (i as i32 + p_len) >= p_off
                && (i as i32 + p_len) >= bm[i + p_len as usize].offset as i32
                && (i as i32 + p_len + bm[i + p_len as usize].length as i32) <= end_offset as i32
            {
                let n2_off = bm[i + p_len as usize].offset as i32;
                let n2_len = bm[i + p_len as usize].length as i32;
                // Check the second match's bytes also match at the first match's offset.
                let eq = (0..n2_len as usize).all(|t| {
                    data[(i as i32 - p_off + p_len) as usize + t]
                        == data[(i as i32 + p_len - n2_off) as usize + t]
                });
                if eq {
                    let mut next_index = i + p_len as usize;
                    while next_index < end_offset && (bm[next_index].length as i32) < MIN_MATCH {
                        next_index += 1;
                    }

                    let mut cur_partial_size = match_varlen_size(p_len - MIN_MATCH);
                    cur_partial_size += 8 + match_varlen_size(n2_len - MIN_MATCH);
                    if n2_off != p_off {
                        cur_partial_size += offset_size(n2_off);
                    }

                    let mut reduced_partial_size = match_varlen_size(p_len + n2_len - MIN_MATCH);

                    if next_index < end_offset {
                        let next_off = bm[next_index].offset as i32;
                        if next_off != n2_off {
                            cur_partial_size += offset_size(next_off);
                        }
                        if next_off != p_off {
                            reduced_partial_size += offset_size(next_off);
                        }
                    }

                    if cur_partial_size >= reduced_partial_size {
                        let match_len_old = p_len;
                        bm[i].length += bm[i + match_len_old as usize].length;
                        bm[i + match_len_old as usize].length = 0;
                        bm[i + match_len_old as usize].offset = 0;
                        did_reduce = true;
                        continue;
                    }
                }
            }

            prev_rep_match_offset = rep_match_offset;
            rep_match_len = bm[i].length as i32;
            rep_match_offset = bm[i].offset as i32;
            rep_index = i as i32;

            i += bm[i].length as usize;
            num_literals = 0;
        } else {
            num_literals += 1;
            i += 1;
        }
    }

    did_reduce
}

// =====================================================================================
// Supplement-matches passes (window < 64 KB).
// =====================================================================================

fn supplement_matches(c: &mut Compressor, data: &[u8], rle: &[i32], end_offset: usize) {
    let n = end_offset;
    if n < 2 {
        return;
    }

    // Per-pass fill caps. At the native stride (64) these are exactly 15/46/63, reproducing LZSA;
    // a larger stride scales them up so the richer table can hold the extra candidates.
    let nmatches = c.nmatches;
    let cap1 = nmatches - 49; // 15 at stride 64
    let cap2 = nmatches - 18; // 46 at stride 64
    let cap3 = nmatches - 1; //  63 at stride 64

    // first_offset_for_byte[65536], next_offset_for_pos[n]
    let mut first_offset_for_byte = vec![-1i32; 65536];
    let mut next_offset_for_pos = vec![-1i32; n];

    for pos in 0..n.saturating_sub(1) {
        let key = (data[pos] as usize) | ((data[pos + 1] as usize) << 8);
        next_offset_for_pos[pos] = first_offset_for_byte[key];
        first_offset_for_byte[key] = pos as i32;
    }

    // pass 1: fill up to cap1 entries, up to 12 inserted, max len 16
    for pos in 1..n.saturating_sub(1) {
        let max_match_len = if pos + 16 < n { 16 } else { n - pos } as i32;
        let mbase = pos * nmatches;
        let mut m = 0usize;
        while m < cap1 && c.matches[mbase + m].length != 0 {
            m += 1;
        }
        let mut inserted = 0;
        let mut match_pos = next_offset_for_pos[pos];
        while m < cap1 && match_pos >= 0 {
            let mp = match_pos as usize;
            let match_offset = (pos - mp) as i32;
            let mut already = false;
            for e in 0..m {
                if c.matches[mbase + e].offset as i32 == match_offset {
                    already = true;
                    break;
                }
            }
            if !already {
                let mut ml = 2i32;
                while ml < max_match_len && data[pos + ml as usize] == data[mp + ml as usize] {
                    ml += 1;
                }
                c.matches[mbase + m].length = ml as u16;
                c.matches[mbase + m].offset = match_offset as u16;
                m += 1;
                inserted += 1;
                if inserted >= 12 {
                    break;
                }
            }
            match_pos = next_offset_for_pos[mp];
        }
    }

    // pass 2: when match[0] < 5, look 2..(2+1+2) forward; tag with 0x8000; <=3 inserted
    let mut offset_cache = vec![-1i32; 2048];
    for pos in 1..n.saturating_sub(1) {
        let mbase = pos * nmatches;
        if (c.matches[mbase].length as i32) < 5 {
            let max_match_len = if pos + 16 < n { 16 } else { n - pos } as i32;
            let mut m = 0usize;
            let mut inserted = 0;
            let mut max_forward_pos = pos + 2 + 1 + 2;
            if max_forward_pos > n - 2 {
                max_forward_pos = n - 2;
            }
            while m < cap2 && c.matches[mbase + m].length != 0 {
                offset_cache[(c.matches[mbase + m].offset as usize) & 2047] = pos as i32;
                m += 1;
            }
            let mut match_pos = next_offset_for_pos[pos];
            while m < cap2 && match_pos >= 0 {
                let mp = match_pos as usize;
                let match_offset = (pos - mp) as i32;
                if match_offset <= MAX_OFFSET {
                    let mut already = false;
                    if offset_cache[(match_offset as usize) & 2047] == pos as i32 {
                        for e in 0..m {
                            if c.matches[mbase + e].offset as i32 == match_offset {
                                already = true;
                                break;
                            }
                        }
                    }
                    if !already {
                        let mut forward_pos = pos + 2;
                        if forward_pos >= match_offset as usize {
                            let mut got = false;
                            while forward_pos < max_forward_pos {
                                if data[forward_pos] == data[forward_pos - match_offset as usize]
                                    && data[forward_pos + 1]
                                        == data[forward_pos + 1 - match_offset as usize]
                                {
                                    got = true;
                                    break;
                                }
                                forward_pos += 1;
                            }
                            if got {
                                let mut ml = 2i32;
                                while ml < max_match_len
                                    && data[pos + ml as usize] == data[mp + ml as usize]
                                {
                                    ml += 1;
                                }
                                c.matches[mbase + m].length = (ml as u16) | 0x8000;
                                c.matches[mbase + m].offset = match_offset as u16;
                                m += 1;
                                insert_forward_match(
                                    c,
                                    data,
                                    rle,
                                    pos,
                                    match_offset,
                                    end_offset,
                                    8,
                                );
                                inserted += 1;
                                if inserted >= 3 {
                                    break;
                                }
                            }
                        }
                    }
                    match_pos = next_offset_for_pos[mp];
                } else {
                    break;
                }
            }
        }
    }

    // pass 3: when match[0] < 8, look 2..(2+1+6) forward; no tag; <=12 inserted
    for pos in 1..n.saturating_sub(1) {
        let mbase = pos * nmatches;
        if (c.matches[mbase].length as i32) < 8 {
            let max_match_len = if pos + 16 < n { 16 } else { n - pos } as i32;
            let mut m = 0usize;
            let mut inserted = 0;
            let mut max_forward_pos = pos + 2 + 1 + 6;
            if max_forward_pos > n - 2 {
                max_forward_pos = n - 2;
            }
            while m < cap3 && c.matches[mbase + m].length != 0 {
                offset_cache[(c.matches[mbase + m].offset as usize) & 2047] = pos as i32;
                m += 1;
            }
            let mut match_pos = next_offset_for_pos[pos];
            while m < cap3 && match_pos >= 0 {
                let mp = match_pos as usize;
                let match_offset = (pos - mp) as i32;
                if match_offset <= MAX_OFFSET {
                    let mut already = false;
                    if offset_cache[(match_offset as usize) & 2047] == pos as i32 {
                        for e in 0..m {
                            if c.matches[mbase + e].offset as i32 == match_offset {
                                already = true;
                                break;
                            }
                        }
                    }
                    if !already {
                        let mut forward_pos = pos + 2;
                        if forward_pos >= match_offset as usize {
                            let mut got = false;
                            while forward_pos < max_forward_pos {
                                if data[forward_pos] == data[forward_pos - match_offset as usize]
                                    && data[forward_pos + 1]
                                        == data[forward_pos + 1 - match_offset as usize]
                                {
                                    got = true;
                                    break;
                                }
                                forward_pos += 1;
                            }
                            if got {
                                let mut ml = 2i32;
                                while ml < max_match_len
                                    && data[pos + ml as usize] == data[mp + ml as usize]
                                {
                                    ml += 1;
                                }
                                c.matches[mbase + m].length = ml as u16;
                                c.matches[mbase + m].offset = match_offset as u16;
                                m += 1;
                                insert_forward_match(
                                    c,
                                    data,
                                    rle,
                                    pos,
                                    match_offset,
                                    end_offset,
                                    8,
                                );
                                inserted += 1;
                                if inserted >= 12 {
                                    break;
                                }
                            }
                        }
                    }
                    match_pos = next_offset_for_pos[mp];
                } else {
                    break;
                }
            }
        }
    }
}

// =====================================================================================
// Nibble emission: encode best_match[] into the raw block, with the end-of-data marker.
// =====================================================================================

struct Writer {
    out: Vec<u8>,
    nibble_pos: isize,
}

impl Writer {
    fn new() -> Self {
        Writer {
            out: Vec::new(),
            nibble_pos: -1,
        }
    }

    #[inline]
    fn nibble(&mut self, v: u8) {
        if self.nibble_pos == -1 {
            self.nibble_pos = self.out.len() as isize;
            self.out.push((v & 0x0f) << 4);
        } else {
            self.out[self.nibble_pos as usize] |= v & 0x0f;
            self.nibble_pos = -1;
        }
    }

    #[inline]
    fn byte(&mut self, b: u8) {
        self.out.push(b);
    }

    #[inline]
    fn bytes(&mut self, b: &[u8]) {
        self.out.extend_from_slice(b);
    }

    fn literals_varlen(&mut self, n: i32) {
        if n >= LITERALS_RUN_LEN {
            if n < LITERALS_RUN_LEN + 15 {
                self.nibble((n - LITERALS_RUN_LEN) as u8);
            } else {
                self.nibble(15);
                if n < 256 {
                    self.byte((n - 18) as u8);
                } else {
                    self.byte(239);
                    self.byte((n & 0xff) as u8);
                    self.byte(((n >> 8) & 0xff) as u8);
                }
            }
        }
    }

    fn match_varlen(&mut self, enc: i32) {
        if enc >= MATCH_RUN_LEN {
            if enc < MATCH_RUN_LEN + 15 {
                self.nibble((enc - MATCH_RUN_LEN) as u8);
            } else {
                self.nibble(15);
                let actual = enc + MIN_MATCH;
                if actual < 256 {
                    self.byte((actual - 24) as u8);
                } else {
                    self.byte(233);
                    self.byte((actual & 0xff) as u8);
                    self.byte(((actual >> 8) & 0xff) as u8);
                }
            }
        }
    }
}

/// Encode the parsed block into a raw LZSA2 block. Returns the encoded bytes.
fn write_block(c: &Compressor, data: &[u8], end_offset: usize) -> Vec<u8> {
    let bm = &c.best_match;
    let mut w = Writer::new();
    let mut num_literals = 0i32;
    let mut in_first_literal_offset = 0usize;
    let mut rep_match_offset = 0i32;

    let mut i = 0usize;
    while i < end_offset {
        let m_len = bm[i].length as i32;
        if m_len >= MIN_MATCH {
            let match_len = m_len;
            let match_offset = bm[i].offset as i32;
            let enc_len = match_len - MIN_MATCH;
            let token_lit = if num_literals >= LITERALS_RUN_LEN {
                LITERALS_RUN_LEN
            } else {
                num_literals
            };
            let token_mlen = if enc_len >= MATCH_RUN_LEN {
                MATCH_RUN_LEN
            } else {
                enc_len
            };

            let token_offset_mode: i32;
            if match_offset == rep_match_offset {
                token_offset_mode = 0xe0;
            } else if match_offset <= 32 {
                token_offset_mode = (((-match_offset) & 0x01) << 5) ^ 0x20;
            } else if match_offset <= 512 {
                token_offset_mode = 0x40 | ((((-match_offset) & 0x100) >> 3) ^ 0x20);
            } else if match_offset <= 8192 + 512 {
                token_offset_mode = 0x80 | ((((-(match_offset - 512)) & 0x0100) >> 3) ^ 0x20);
            } else {
                token_offset_mode = 0xc0;
            }

            let token = token_offset_mode | (token_lit << 3) | token_mlen;
            w.byte(token as u8);
            w.literals_varlen(num_literals);
            if num_literals != 0 {
                w.bytes(
                    &data[in_first_literal_offset..in_first_literal_offset + num_literals as usize],
                );
                num_literals = 0;
            }

            match token_offset_mode {
                0x00 | 0x20 => {
                    w.nibble((((-match_offset) & 0x1e) >> 1) as u8);
                }
                0x40 | 0x60 => {
                    w.byte(((-match_offset) & 0xff) as u8);
                }
                0x80 | 0xa0 => {
                    w.nibble((((-(match_offset - 512)) >> 9) & 0x0f) as u8);
                    w.byte(((-(match_offset - 512)) & 0xff) as u8);
                }
                0xc0 => {
                    w.byte(((-match_offset) >> 8) as u8);
                    w.byte(((-match_offset) & 0xff) as u8);
                }
                _ => {}
            }

            rep_match_offset = match_offset;
            w.match_varlen(enc_len);

            i += match_len as usize;
        } else {
            if num_literals == 0 {
                in_first_literal_offset = i;
            }
            num_literals += 1;
            i += 1;
        }
    }

    // terminal literals-only command
    let token_lit = if num_literals >= LITERALS_RUN_LEN {
        LITERALS_RUN_LEN
    } else {
        num_literals
    };
    w.byte(((token_lit << 3) | 0xe7) as u8);
    w.literals_varlen(num_literals);
    if num_literals != 0 {
        w.bytes(&data[in_first_literal_offset..in_first_literal_offset + num_literals as usize]);
    }

    // EOD marker
    w.nibble(15);
    w.byte(232);

    if w.nibble_pos != -1 {
        w.nibble(0);
    }

    w.out
}

// =====================================================================================
// Public entry.
// =====================================================================================

/// Inject the exact Pareto front (for each length, the smallest offset that achieves it) into the
/// match table at every position, filling empty trailing slots. The suffix-array tree walk in
/// `find_matches_at` keeps only a bounded selection of offsets; the Pareto front guarantees that
/// the *cheapest* offset for each reachable length is present, which is what the cost-driven parse
/// most wants. Offsets already present at a position are skipped (no duplicates). O(n * window).
///
/// `full = false`: inject only the Pareto front. `full = true`: inject *every* distinct offset that
/// achieves a match of length >= MIN_MATCH (longest first), maximising the rep-offset choices the
/// parse can establish. Both fill only the empty trailing slots, capped at the table stride.
fn enrich_pareto(c: &mut Compressor, data: &[u8], end_offset: usize, full: bool) {
    let n = end_offset;
    let nmatches = c.nmatches;
    // best_off_for_len[L] = smallest offset achieving length L at the current position.
    let max_len = LCP_MAX as usize;
    let mut best_off_for_len: Vec<u32> = vec![0; max_len + 2];
    for i in 1..n {
        let max_possible = max_len.min(n - i);
        if max_possible < MIN_MATCH as usize {
            continue;
        }
        let mbase = i * nmatches;
        // Already-full row: nothing to add.
        if c.matches[mbase + nmatches - 1].length != 0 {
            continue;
        }
        let hi_off = i;
        let mut max_found_len = 0usize;
        // For the full mode, collect distinct (offset, length) with length >= MIN_MATCH, bounded so
        // degenerate (repetitive) input cannot blow up: at most `nmatches` recorded entries and a
        // capped examined-offset budget. We only need enough to fill the free slots anyway.
        let full_cap = nmatches;
        let mut examined = 0usize;
        let examine_budget = 8192usize;
        // Cap the full-mode per-offset length scan: long matches at any offset are already captured by
        // the tree walk and the Pareto path, so the full set only needs to surface alternative
        // *offsets*. The cap keeps degenerate input tractable.
        let full_len_cap = max_possible.min(273);
        let mut all_entries: Vec<(u32, u32)> = Vec::new();
        for d in 1..=hi_off {
            let src = i - d;
            let scan_len = if full { full_len_cap } else { max_possible };
            if !full {
                // Pareto fast path: an offset only helps if it extends the current best length.
                if max_found_len >= max_possible {
                    break;
                }
                if data[src + max_found_len] != data[i + max_found_len] {
                    continue;
                }
            } else {
                if all_entries.len() >= full_cap || examined >= examine_budget {
                    break;
                }
                let cmp1 = 1.min(max_possible - 1);
                if data[src] != data[i] || data[src + cmp1] != data[i + cmp1] {
                    continue;
                }
                examined += 1;
            }
            let mut l = 0usize;
            while l < scan_len && data[src + l] == data[i + l] {
                l += 1;
            }
            if l >= MIN_MATCH as usize {
                if full {
                    all_entries.push((d as u32, l as u32));
                }
                let start_len = max_found_len.max(MIN_MATCH as usize - 1) + 1;
                for ll in start_len..=l {
                    best_off_for_len[ll] = d as u32;
                }
                if l > max_found_len {
                    max_found_len = l;
                }
            }
        }
        if max_found_len < MIN_MATCH as usize {
            continue;
        }
        // Find the first free slot.
        let mut free = 0usize;
        while free < nmatches && c.matches[mbase + free].length != 0 {
            free += 1;
        }
        // Build the candidate list to inject.
        let mut entries: Vec<(u32, u32)> = if full {
            all_entries
        } else {
            // Pareto front (increasing length, smallest offset each), runs of equal offset collapsed.
            let mut v: Vec<(u32, u32)> = Vec::new();
            let mut ll = MIN_MATCH as usize;
            while ll <= max_found_len {
                let off = best_off_for_len[ll];
                if off != 0 {
                    let next_off = if ll < max_found_len {
                        best_off_for_len[ll + 1]
                    } else {
                        0
                    };
                    if off != next_off {
                        v.push((off, ll as u32));
                    }
                }
                ll += 1;
            }
            v
        };
        // Longest first so the most valuable matches land if space runs out.
        entries.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        for (off, len) in entries {
            if free >= nmatches {
                break;
            }
            // Skip offsets already present (the tree walk may have emitted them, tagged or not).
            let mut dup = false;
            for s in 0..free {
                if c.matches[mbase + s].offset as i32 == off as i32 {
                    dup = true;
                    break;
                }
            }
            if dup {
                continue;
            }
            // Tag as non-closest (0x8000) so the parse charges it the same score penalty as the
            // tree-walk's far matches; the length is masked off before use anyway.
            c.matches[mbase + free].length = (len as u16) | 0x8000;
            c.matches[mbase + free].offset = off as u16;
            free += 1;
        }
    }
}

/// Run the full LZSA2 parse pipeline with a given per-position match-table stride. With
/// `nmatches == 64` and `enrich == None` this is byte-for-byte the native LZSA2 result (the
/// anchor). A larger stride plus `enrich = Some(full)` feeds the richer candidate set into the
/// same parse (`full = false` Pareto front, `full = true` all distinct offsets).
fn compress_lzsa2_pipeline(input: &[u8], nmatches: usize, enrich: Option<bool>) -> Vec<u8> {
    let n = input.len();
    let end_offset = n;

    // 1) Build match finder structures and find matches (up to `nmatches` per position).
    let mut mf = MatchFinder::build(input, MIN_MATCH);
    let mut c = Compressor::with_capacity(n, nmatches);

    {
        let mut scratch = vec![Match::default(); nmatches];
        for i in 0..n {
            let cnt = mf.find_matches_at(i, &mut scratch, nmatches, n);
            let base = i * nmatches;
            for m in 0..nmatches {
                c.matches[base + m] = if m < cnt {
                    scratch[m]
                } else {
                    Match::default()
                };
            }
        }
    }

    let rle = mf.rle_len.clone();

    // Richer candidates: inject extra offsets into the spare slots before the parse.
    if let Some(full) = enrich {
        enrich_pareto(&mut c, input, end_offset, full);
    }

    // 2) First optimize pass (no reduce, insert forward reps, 32 arrivals).
    optimize_forward(
        &mut c,
        input,
        &rle,
        end_offset,
        false,
        true,
        NARRIVALS_PER_POSITION_V2_BIG,
    );

    // 3) Supplement matches (window < 64 KB).
    supplement_matches(&mut c, input, &rle, end_offset);

    // 4) Second optimize pass (reduce, use existing forward reps, 64 arrivals).
    optimize_forward(
        &mut c,
        input,
        &rle,
        end_offset,
        true,
        false,
        NARRIVALS_PER_POSITION_V2_MAX,
    );

    // 5) Command-count reduction (<= 20 passes).
    let mut passes = 0;
    loop {
        let did = optimize_command_count(&mut c, input, end_offset);
        passes += 1;
        if !did || passes >= 20 {
            break;
        }
    }

    if std::env::var("LZSA2_DEBUG").is_ok() {
        let mut cmds = 0;
        let mut reps = 0;
        let mut i = 0;
        let mut rep = 0i32;
        while i < end_offset {
            let l = c.best_match[i].length as i32;
            if l >= MIN_MATCH {
                cmds += 1;
                if c.best_match[i].offset as i32 == rep {
                    reps += 1;
                }
                rep = c.best_match[i].offset as i32;
                i += l as usize;
            } else {
                i += 1;
            }
        }
        eprintln!(
            "[lzsa2] nmatches={} enrich={:?} match_commands={} rep_matches={}",
            nmatches, enrich, cmds, reps
        );
    }

    // 6) Emit.
    write_block(&c, input, end_offset)
}

/// The byte-identical empty-input LZSA2 raw block (shared by the anchor and best-of paths).
fn lzsa2_empty_block() -> Vec<u8> {
    let mut w = Writer::new();
    w.byte(0xe7);
    w.nibble(15);
    w.byte(232);
    if w.nibble_pos != -1 {
        w.nibble(0);
    }
    w.out
}

/// Tier-1 anchor: byte-identical to `lzsa -f2 -r` - the stride-64 pipeline, no rich enrichment.
/// Returns an empty vector if `input` exceeds 64 KB (the raw-block cap), matching [`compress_lzsa2`].
pub fn compress_lzsa2_anchor(input: &[u8]) -> Vec<u8> {
    if input.len() > MAX_BLOCK {
        return Vec::new();
    }
    if input.is_empty() {
        return lzsa2_empty_block();
    }
    compress_lzsa2_pipeline(input, NMATCHES_PER_INDEX_V2, None)
}

/// Tier-1 anchor, backward layout: `reverse(forward_anchor(reverse(input)))`.
pub fn compress_lzsa2_anchor_backward(input: &[u8]) -> Vec<u8> {
    let mut rev_in = input.to_vec();
    rev_in.reverse();
    let mut out = compress_lzsa2_anchor(&rev_in);
    out.reverse();
    out
}

/// Compress `input` into an LZSA2 raw block (`lzsa -f 2 -r`). Returns an empty vector if `input`
/// exceeds 64 KB, the raw-block cap.
///
/// Best-of: the native-faithful anchor (64-candidate cap) plus richer-candidate variants that feed
/// an uncapped Pareto-enriched match set into the same optimal parse. The smaller wins, so the
/// result is never larger than native LZSA2.
pub fn compress_lzsa2(input: &[u8]) -> Vec<u8> {
    if input.len() > MAX_BLOCK {
        return Vec::new();
    }

    if input.is_empty() {
        return lzsa2_empty_block();
    }

    // Anchor: exact native LZSA2 (no-regression floor).
    let anchor = compress_lzsa2_pipeline(input, NMATCHES_PER_INDEX_V2, None);

    // Allow pinning to the anchor only (for measuring native parity).
    if std::env::var("LZSA2_ANCHOR_ONLY").is_ok() {
        return anchor;
    }

    // Richer variants: each runs the identical parse over an enriched, larger match table.
    // `Some(false)` = Pareto front (cheapest offset per length); `Some(true)` = all distinct offsets
    // (maximal rep-offset choices). The smallest of all (anchor included) wins, so the output is
    // never larger than native. Stride 128 is well above the threshold at which the corpus wins
    // appear (96) while keeping peak memory modest under parallel use.
    let configs: &[(usize, bool)] = &[(128, false), (128, true)];

    // Optional single-config pin for measurement: LZSA2_PIN="stride,full" (full in {0,1}).
    if let Ok(s) = std::env::var("LZSA2_PIN") {
        let parts: Vec<&str> = s.split(',').collect();
        if parts.len() == 2 {
            let stride: usize = parts[0].parse().unwrap_or(256);
            let full = parts[1].trim() == "1";
            let cand = compress_lzsa2_pipeline(input, stride, Some(full));
            return if cand.len() < anchor.len() {
                cand
            } else {
                anchor
            };
        }
    }

    let cands: Vec<Vec<u8>> = std::thread::scope(|s| {
        let handles: Vec<_> = configs
            .iter()
            .map(|&(stride, full)| {
                s.spawn(move || compress_lzsa2_pipeline(input, stride, Some(full)))
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("lzsa2 rich variant panicked"))
            .collect()
    });

    let mut best = anchor;
    for cand in cands {
        if cand.len() < best.len() {
            best = cand;
        }
    }
    best
}

/// Compress `input` into an LZSA2 backward raw block (`lzsa -f2 -r -b`).
///
/// The backward block is `reverse(forward_encode(reverse(input)))`. The forward writer always
/// flushes its final half-nibble to a whole byte, so the output is byte-aligned and the
/// byte-reversal is well defined.
///
/// Panics if `input.len() > 64 KB` (the LZSA2 raw block cap), via `compress_lzsa2`.
pub fn compress_lzsa2_backward(input: &[u8]) -> Vec<u8> {
    let mut rev_in = input.to_vec();
    rev_in.reverse();
    let mut out = compress_lzsa2(&rev_in);
    out.reverse();
    out
}

/// Two compression tiers. Level 1 = the stride-64 anchor (byte-identical to `lzsa -f2 -r`, fast);
/// level 2 = the rich best-of (smallest, never larger than level 1).
pub const MAX_LEVEL: u8 = 2;

/// Compress `input` into an LZSA2 raw block. `level` is NORMALIZED into `1..=MAX_LEVEL`.
///   level 1 = the stride-64 anchor (byte-identical to `lzsa -f2 -r`, fast).
///   level 2 = the rich best-of (smallest, never larger than level 1).
/// `backward` selects the backward block layout. Returns an empty vector if `input` > 64 KB.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    if level >= 2 {
        if backward {
            compress_lzsa2_backward(input)
        } else {
            compress_lzsa2(input)
        }
    } else if backward {
        compress_lzsa2_anchor_backward(input)
    } else {
        compress_lzsa2_anchor(input)
    }
}

/// Native API: run the LZSA2 pipeline at the given match-table stride `nmatches`. The stride-64
/// anchor is always taken into the best-of, so the output is never larger than native `lzsa -f2 -r`.
/// `backward` selects the backward block layout. Returns an empty vector if `input` > 64 KB.
pub fn compress_native(input: &[u8], nmatches: usize, backward: bool) -> Vec<u8> {
    if backward {
        let mut rev_in = input.to_vec();
        rev_in.reverse();
        let mut out = compress_lzsa2_native_forward(&rev_in, nmatches);
        out.reverse();
        out
    } else {
        compress_lzsa2_native_forward(input, nmatches)
    }
}

/// Forward `compress_native`: best-of(stride-64 anchor, pipeline at stride `nmatches`). Honours the
/// 64 KB raw-block cap and the empty-input special case, like [`compress_lzsa2`].
fn compress_lzsa2_native_forward(input: &[u8], nmatches: usize) -> Vec<u8> {
    if input.len() > MAX_BLOCK {
        return Vec::new();
    }
    if input.is_empty() {
        return lzsa2_empty_block();
    }
    let anchor = compress_lzsa2_pipeline(input, NMATCHES_PER_INDEX_V2, None);
    let cand = compress_lzsa2_pipeline(input, nmatches, None);
    if cand.len() < anchor.len() {
        cand
    } else {
        anchor
    }
}

/// Decode an LZSA2 raw block. For `backward`, decode the byte-reversed stream and reverse the
/// result, matching `backward = reverse(forward_encode(reverse(input)))`.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let mut rev = input.to_vec();
        rev.reverse();
        let mut out = decode_raw(&rev);
        out.reverse();
        out
    } else {
        decode_raw(input)
    }
}

/// Decode an LZSA2 raw block.
pub fn decode_raw(block: &[u8]) -> Vec<u8> {
    decode_raw_with_gap(block).0
}

/// Decode an LZSA2 raw block and also return the in-place safety gap (bytes) the
/// stream needs: `max(output_produced - input_consumed)` over the decode, minus
/// its final value. Any in-place layout (forward top-aligned or backward) must
/// keep the write head at least this many bytes clear of the read head, or it
/// will clobber unread compressed bytes - an incompressible literal run decoded
/// LATE makes the running compression peak above its final value and the fixed
/// margin is no longer enough. The gap grows during a match (output advances, the
/// read head does not) and peaks at the match's end, which is the state observed
/// at the next loop iteration's top. See [`max_gap_forward`] / [`max_gap_backward`].
fn decode_raw_with_gap(block: &[u8]) -> (Vec<u8>, i32) {
    let mut out: Vec<u8> = Vec::new();
    let mut p = 0usize;
    let end = block.len();
    let mut cur_nibbles = 0i32;
    let mut nibbles = 0u8;
    let mut match_offset: i32 = 0;

    let get_nibble = |p: &mut usize, cur: &mut i32, nib: &mut u8| -> Option<u32> {
        *cur ^= 1;
        if *cur != 0 {
            if *p < end {
                *nib = block[*p];
                *p += 1;
                Some(((*nib & 0xf0) >> 4) as u32)
            } else {
                None
            }
        } else {
            Some((*nib & 0x0f) as u32)
        }
    };

    let build_len = |p: &mut usize, cur: &mut i32, nib: &mut u8, base: u32| -> Option<u32> {
        let mut len = base;
        let v = get_nibble(p, cur, nib)?;
        len += v;
        if v == 15 {
            if *p < end {
                len += block[*p] as u32;
                *p += 1;
                if len == 257 {
                    if *p + 1 < end {
                        len = block[*p] as u32;
                        *p += 1;
                        len |= (block[*p] as u32) << 8;
                        *p += 1;
                    } else {
                        return None;
                    }
                } else if len == 256 {
                    len = 0;
                }
            } else {
                return None;
            }
        }
        Some(len)
    };

    // Peak of (produced - consumed) at a token boundary. `p` is the read head's
    // byte position (a nibble may be half-consumed, but that constant offset
    // cancels in `max_gap - final_gap`, so a consistent byte position suffices).
    let mut max_gap = 0i32;

    while p < end {
        let gap = out.len() as i32 - p as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let token = block[p];
        p += 1;
        let mut nliterals = ((token & 0x18) >> 3) as u32;
        if nliterals == LITERALS_RUN_LEN as u32 {
            nliterals = build_len(
                &mut p,
                &mut cur_nibbles,
                &mut nibbles,
                LITERALS_RUN_LEN as u32,
            )
            .unwrap();
        }
        if nliterals != 0 {
            out.extend_from_slice(&block[p..p + nliterals as usize]);
            p += nliterals as usize;
        }

        if p < end {
            let offset_mode = token & 0xc0;
            match offset_mode {
                0x00 => {
                    let v = get_nibble(&mut p, &mut cur_nibbles, &mut nibbles).unwrap();
                    let mut o = (v << 1) as i32;
                    o |= ((token & 0x20) >> 5) as i32;
                    o ^= 0x1e;
                    o += 1;
                    match_offset = o;
                }
                0x40 => {
                    let mut o = block[p] as i32;
                    p += 1;
                    o |= ((token & 0x20) as i32) << 3;
                    o ^= 0x0ff;
                    o += 1;
                    match_offset = o;
                }
                0x80 => {
                    let v = get_nibble(&mut p, &mut cur_nibbles, &mut nibbles).unwrap();
                    let mut o = block[p] as i32;
                    p += 1;
                    o |= (v as i32) << 9;
                    o |= ((token & 0x20) as i32) << 3;
                    o ^= 0x1eff;
                    o += 512 + 1;
                    match_offset = o;
                }
                _ => {
                    if (token & 0x20) == 0 {
                        let mut o = (block[p] as i32) << 8;
                        p += 1;
                        o |= block[p] as i32;
                        p += 1;
                        o ^= 0xffff;
                        o += 1;
                        match_offset = o;
                    }
                }
            }

            let mut mlen = (token & 0x07) as u32;
            mlen += MIN_MATCH as u32;
            if mlen == (MATCH_RUN_LEN + MIN_MATCH) as u32 {
                mlen = build_len(&mut p, &mut cur_nibbles, &mut nibbles, mlen).unwrap();
                if mlen == 0 {
                    break;
                }
            }
            let src = out.len() as i32 - match_offset;
            assert!(src >= 0, "bad match offset");
            for k in 0..mlen as usize {
                let b = out[src as usize + k];
                out.push(b);
            }
        }
    }

    // The read head consumes the whole `block.len()`-byte block; use it (not `p`,
    // which stops at the EOD token) so the final gap is the true end state.
    let final_gap = out.len() as i32 - block.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD LZSA2 stream: the top-aligned
/// packed block must start at least this many bytes above the output end, or the
/// decoder's write head overtakes unread compressed data. See
/// [`decode_raw_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        decode_raw_with_gap(stream).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD LZSA2 stream: the packed block
/// must sit at least this many bytes below the span start. The backward layout is
/// `reverse(forward_encode(reverse(input)))`, so the 6502 backward decoder reads
/// the stored stream from its END - exactly a forward decode of the reversed
/// stream, so the gap sequence matches. See [`decode_raw_with_gap`].
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        let rev: Vec<u8> = stream.iter().rev().copied().collect();
        decode_raw_with_gap(&rev).1.max(0) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(data: &[u8]) {
        // Forward roundtrip through the uniform API.
        let block = compress(data, MAX_LEVEL, false);
        let dec = decompress(&block, false);
        assert_eq!(dec, data, "roundtrip mismatch (len {})", data.len());

        // Backward roundtrip through the uniform API.
        let bwd = compress(data, MAX_LEVEL, true);
        let dec_b = decompress(&bwd, true);
        assert_eq!(
            dec_b,
            data,
            "lzsa2 backward roundtrip mismatch (len {})",
            data.len()
        );
    }

    #[test]
    fn empties_and_tiny() {
        rt(&[]);
        rt(&[0]);
        rt(&[1, 2, 3]);
        rt(&[5, 5, 5, 5, 5, 5, 5, 5]);
        rt(b"abcabcabcabcabc");
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        // Unlike the bit-oriented formats (apultra/bb2/zx02, whose literals cost
        // >8 bits and expand incompressible data by ~12%), LZSA2 - like its
        // sibling LZSA1 - stores literals raw at 8 bits inside arbitrarily long
        // runs whose length code is a few bytes. Incompressible data therefore
        // barely expands (8192 -> ~8198), so the in-place gap-above-final is just
        // the handful of bytes of trailing stream overhead, never the >32 seen in
        // the bit formats. We assert the machinery's real invariants: the gap is
        // exposed and non-zero, stays small, and compressible data fits the
        // default 32-byte margin.
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();

        let fc = compress_lzsa2_anchor(&noise);
        let bc = compress_lzsa2_anchor_backward(&noise);
        let gf = max_gap_forward(&fc);
        let gb = max_gap_backward(&bc);
        // The instrumentation exposes a real peak above the final decode state
        // (the trailing stream overhead), so the gap is strictly positive…
        assert!(
            gf > 0 && gb > 0,
            "gap must expose a non-zero peak (fwd {gf}, bwd {gb})"
        );
        // …but LZSA2's efficient literal coding keeps it far below a KB for 8 KB in.
        assert!(
            gf < 1024 && gb < 1024,
            "LZSA2 gap must stay small (fwd {gf}, bwd {gb})"
        );

        // Highly compressible data barely expands: the default 32-byte margin is
        // enough, both layouts.
        let zeros = vec![0u8; 8192];
        assert!(
            max_gap_forward(&compress_lzsa2_anchor(&zeros)) <= 32,
            "compressible forward data should fit within the default margin"
        );
        assert!(
            max_gap_backward(&compress_lzsa2_anchor_backward(&zeros)) <= 32,
            "compressible backward data should fit within the default margin"
        );

        // The gap wrapper must not alter decode output: byte-identical to the
        // public decoder and to the original input.
        assert_eq!(decode_raw(&fc), decode_raw_with_gap(&fc).0);
        assert_eq!(decode_raw(&fc), noise);
    }

    #[test]
    fn text() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut v = Vec::new();
        for _ in 0..300 {
            v.extend_from_slice(base);
        }
        rt(&v);
    }

    #[test]
    fn runs_and_reps() {
        let mut v = Vec::new();
        for i in 0..5000u32 {
            v.push((i % 17) as u8);
        }
        for _ in 0..1000 {
            v.push(0xAA);
        }
        rt(&v);
    }

    #[test]
    fn pseudo_random() {
        let mut state = 12345u32;
        let v: Vec<u8> = (0..20000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                (state >> 24) as u8
            })
            .collect();
        rt(&v);
    }
}
