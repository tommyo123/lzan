//! Match finders producing, for each position, the Pareto front of matches: increasing
//! length, each at the smallest offset that achieves it.
//!
//! Candidates are stored in CSR form: `matches_for(i)` returns a slice ordered by strictly
//! increasing length.

#[derive(Clone, Copy, Debug)]
pub struct Candidate {
    pub offset: u32, // distance back, >= 1
    pub length: u32,
}

pub struct MatchSet {
    cands: Vec<Candidate>,
    starts: Vec<u32>, // len n+1; matches for i are cands[starts[i]..starts[i+1]]
    /// The `max_offset` window this set was built with. Parse stages that
    /// synthesize candidates beyond this set (supplemental-offset seeding) must
    /// cap their offsets here, or a window-restricted format (min-eof's
    /// EOF_MAX_OFFSET) can be handed an offset its encoding cannot represent.
    window: u32,
}

impl MatchSet {
    #[inline]
    pub fn matches_for(&self, i: usize) -> &[Candidate] {
        let a = self.starts[i] as usize;
        let b = self.starts[i + 1] as usize;
        &self.cands[a..b]
    }

    /// The `max_offset` window this set was built with.
    #[inline]
    pub fn window(&self) -> u32 {
        self.window
    }

    /// Raw CSR candidate array (for finder-equivalence checks).
    #[inline]
    pub fn cands_slice(&self) -> &[Candidate] {
        &self.cands
    }

    /// Raw CSR row-start array (for finder-equivalence checks).
    #[inline]
    pub fn starts_slice(&self) -> &[u32] {
        &self.starts
    }
}

const HASH_BITS: u32 = 17;
const HASH_SIZE: usize = 1 << HASH_BITS;
const NONE: u32 = u32::MAX;

#[inline]
fn hash3(data: &[u8], i: usize) -> usize {
    // 3-byte hash, to also find short (length 3-4) matches. Length-2 rep matches are found
    // directly in the parser.
    let v = (data[i] as u32) | ((data[i + 1] as u32) << 8) | ((data[i + 2] as u32) << 16);
    (v.wrapping_mul(2654435761) >> (32 - HASH_BITS)) as usize
}

/// Find Pareto matches at every position via hash chains.
///
/// - `min_match`: shortest match to report (>= 2).
/// - `max_offset`: window size (max distance back).
/// - `max_len`: cap on match length recorded.
/// - `max_chain`: chain-walk depth (higher = better ratio, slower).
pub fn find_matches(
    data: &[u8],
    min_match: usize,
    max_offset: usize,
    max_len: usize,
    max_chain: usize,
) -> MatchSet {
    let n = data.len();
    let mut starts = vec![0u32; n + 1];
    let mut cands: Vec<Candidate> = Vec::with_capacity(n); // rough

    if n < 3 {
        // Nothing to match; all positions have empty candidate lists.
        for i in 0..=n {
            starts[i] = 0;
        }
        return MatchSet {
            cands,
            starts,
            window: max_offset as u32,
        };
    }

    let mut head = vec![NONE; HASH_SIZE];
    let mut prev = vec![NONE; n];
    // Direct 2-byte table for the nearest length-2+ match, catching short matches the 3-byte
    // hash misses. Only the nearest (smallest-offset) one is recorded.
    let mut head2 = vec![NONE; 1 << 16];

    for i in 0..n {
        starts[i] = cands.len() as u32;

        let max_possible = max_len.min(n - i);
        let mut best_len = if min_match > 0 { min_match - 1 } else { 0 };

        if i + 2 <= n {
            let h2 = (data[i] as usize) | ((data[i + 1] as usize) << 8);
            let c2 = head2[h2];
            if c2 != NONE {
                let curu = c2 as usize;
                let dist = i - curu;
                if dist <= max_offset
                    && best_len < max_possible
                    && data[curu + best_len] == data[i + best_len]
                {
                    let mut l = 0usize;
                    while l < max_possible && data[curu + l] == data[i + l] {
                        l += 1;
                    }
                    if l > best_len {
                        cands.push(Candidate {
                            offset: dist as u32,
                            length: l as u32,
                        });
                        best_len = l;
                    }
                }
            }
            head2[h2] = i as u32;
        }

        if i + 3 <= n {
            let h = hash3(data, i);
            let mut cur = head[h];
            let mut chain = 0usize;

            while cur != NONE {
                let curu = cur as usize;
                let dist = i - curu;
                if dist > max_offset {
                    break;
                }
                // Quick reject: check the byte at best_len before computing the full length.
                if best_len < max_possible && data[curu + best_len] == data[i + best_len] {
                    let mut l = 0usize;
                    while l < max_possible && data[curu + l] == data[i + l] {
                        l += 1;
                    }
                    if l > best_len {
                        cands.push(Candidate {
                            offset: dist as u32,
                            length: l as u32,
                        });
                        best_len = l;
                        if l >= max_possible {
                            break;
                        }
                    }
                }

                cur = prev[curu];
                chain += 1;
                if chain >= max_chain {
                    break;
                }
            }

            // Insert i into the chain.
            prev[i] = head[h];
            head[h] = i as u32;
        }
    }
    starts[n] = cands.len() as u32;
    MatchSet {
        cands,
        starts,
        window: max_offset as u32,
    }
}

/// Exact (brute-force) Pareto match finder: for every position, scan all offsets in the window
/// and record, for each achievable length, the smallest offset achieving it (the true Pareto
/// front). O(n * window) with early-out; offline only. Produces complete candidates that the
/// hash chain may miss.
pub fn find_matches_exact(
    data: &[u8],
    min_match: usize,
    max_offset: usize,
    max_len: usize,
) -> MatchSet {
    let n = data.len();
    let mut starts = vec![0u32; n + 1];
    let mut cands: Vec<Candidate> = Vec::with_capacity(n * 2);
    // best_off_for_len[L] = smallest offset achieving length L at this position (scratch).
    let mut best_off_for_len: Vec<u32> = vec![0; max_len + 2];

    for i in 0..n {
        starts[i] = cands.len() as u32;
        if i == 0 {
            continue;
        }
        let max_possible = max_len.min(n - i);
        if max_possible < min_match {
            continue;
        }
        let lo_off = 1usize;
        let hi_off = max_offset.min(i);
        let mut max_found_len = 0usize;
        // Offsets are scanned ascending, so the first offset to reach a given length owns it.
        for d in lo_off..=hi_off {
            let src = i - d;
            // Quick reject: this offset can only help if it extends the current best.
            if max_found_len >= max_possible {
                break;
            }
            if data[src + max_found_len] != data[i + max_found_len] {
                continue;
            }
            let mut l = 0usize;
            while l < max_possible && data[src + l] == data[i + l] {
                l += 1;
            }
            if l >= min_match {
                // Record this (smallest) offset for each new length in (max_found_len, l].
                let start_len = max_found_len.max(min_match - 1) + 1;
                for ll in start_len..=l {
                    best_off_for_len[ll] = d as u32;
                }
                if l > max_found_len {
                    max_found_len = l;
                }
            }
        }
        // Emit Pareto candidates: increasing length, each with its smallest offset. Runs of
        // equal offset are collapsed to only the longest length, since the parser relaxes the
        // skipped intermediate lengths at the same offset anyway. This bounds the candidate
        // count on highly repetitive input (e.g. all-zeros, where every length shares offset 1).
        let mut ll = min_match;
        while ll <= max_found_len {
            let off = best_off_for_len[ll];
            if off != 0 {
                let next_off = if ll < max_found_len {
                    best_off_for_len[ll + 1]
                } else {
                    0
                };
                if off != next_off {
                    cands.push(Candidate {
                        offset: off,
                        length: ll as u32,
                    });
                }
            }
            ll += 1;
        }
    }
    starts[n] = cands.len() as u32;
    MatchSet {
        cands,
        starts,
        window: max_offset as u32,
    }
}

// ---------------------------------------------------------------------------
// Output-sensitive exact Pareto match finder.
// ---------------------------------------------------------------------------
//
// `find_matches_fast` computes exactly the same thing as `find_matches_exact`
// (the true Pareto front: for each achievable length L, the smallest offset that
// achieves it) but without enumerating offsets.
//
// Structure: suffix array + LCP array + a TOP-DOWN sweep of the LCP-interval
// tree (== suffix tree topology) carrying a text-order doubly linked list per
// node, split small-to-large.
//
// Why this and not a bottom-up / union-find ancestor walk: the quantity we need
// at an internal node `v` of string depth `l` is
//
//     pred_v(i) = max { j in subtree(v) : j < i }
//
// because { j : lcp(j,i) >= l } is exactly `subtree(v)` for the ancestor `v` of
// leaf `i` at depth `l`. A Pareto candidate exists at `v` precisely when
// `pred_v(i)` differs from `pred_child(i)`, and then `lcp(pred_v(i), i) == l`
// exactly (the predecessor left i's child subtree, so their LCA is `v`).
//
// Maintaining `pred` under a top-down split is O(1) per moved element: removing
// one element from a doubly linked list changes the predecessor of exactly one
// other element. Keeping the largest child's list in place and rebuilding only
// the smaller siblings gives O(n log n) moved elements overall, so the total
// work is output-sensitive rather than O(n * window). Crucially this does NOT
// degenerate on long runs: for all-zeros the tree is a path of depth n, but each
// node splits off a single leaf, so the whole sweep is O(n).
//
// Candidates are emitted root-to-leaf, i.e. in increasing length AND increasing
// offset, which is exactly the Pareto order the CSR output wants.

const LL_NONE: u32 = u32::MAX;

/// Suffix array by prefix doubling with radix sort. O(n log n), std-only.
fn build_sa(data: &[u8]) -> Vec<u32> {
    let n = data.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![0];
    }
    let bucket = (n + 2).max(257);
    let mut cnt: Vec<u32> = vec![0; bucket];
    let mut sa: Vec<u32> = vec![0; n];
    let mut rank: Vec<u32> = vec![0; n];

    // Initial counting sort by first byte.
    for &b in data {
        cnt[b as usize] += 1;
    }
    let mut sum = 0u32;
    for c in cnt.iter_mut().take(256) {
        let t = *c;
        *c = sum;
        sum += t;
    }
    for (i, &b) in data.iter().enumerate() {
        let s = b as usize;
        sa[cnt[s] as usize] = i as u32;
        cnt[s] += 1;
    }
    rank[sa[0] as usize] = 0;
    let mut r = 0u32;
    for i in 1..n {
        if data[sa[i] as usize] != data[sa[i - 1] as usize] {
            r += 1;
        }
        rank[sa[i] as usize] = r;
    }

    let mut tmp_sa: Vec<u32> = vec![0; n];
    let mut tmp_rank: Vec<u32> = vec![0; n];
    let mut k = 1usize;
    while (r as usize) + 1 < n {
        // Order by the second key: suffixes whose second half falls off the end
        // sort first, then the rest in current-SA order.
        let mut p = 0usize;
        for i in (n - k)..n {
            tmp_sa[p] = i as u32;
            p += 1;
        }
        for i in 0..n {
            let s = sa[i];
            if (s as usize) >= k {
                tmp_sa[p] = s - k as u32;
                p += 1;
            }
        }
        // Stable counting sort by the first key.
        let nb = (r as usize) + 2;
        for c in cnt.iter_mut().take(nb) {
            *c = 0;
        }
        for i in 0..n {
            cnt[rank[i] as usize] += 1;
        }
        let mut sum = 0u32;
        for c in cnt.iter_mut().take(nb) {
            let t = *c;
            *c = sum;
            sum += t;
        }
        for i in 0..n {
            let s = tmp_sa[i] as usize;
            let b = rank[s] as usize;
            sa[cnt[b] as usize] = s as u32;
            cnt[b] += 1;
        }
        // Re-rank.
        tmp_rank[sa[0] as usize] = 0;
        let mut nr = 0u32;
        for i in 1..n {
            let a = sa[i - 1] as usize;
            let b = sa[i] as usize;
            let a2 = if a + k < n { rank[a + k] as i64 } else { -1 };
            let b2 = if b + k < n { rank[b + k] as i64 } else { -1 };
            if rank[a] != rank[b] || a2 != b2 {
                nr += 1;
            }
            tmp_rank[b] = nr;
        }
        std::mem::swap(&mut rank, &mut tmp_rank);
        r = nr;
        k <<= 1;
        if k >= n {
            break;
        }
    }
    sa
}

/// Kasai LCP: `lcp[r] = lcp(sa[r-1], sa[r])`, `lcp[0] = 0`.
fn build_lcp(data: &[u8], sa: &[u32]) -> Vec<u32> {
    let n = data.len();
    let mut inv = vec![0u32; n];
    for (r, &s) in sa.iter().enumerate() {
        inv[s as usize] = r as u32;
    }
    let mut lcp = vec![0u32; n];
    let mut h = 0usize;
    for i in 0..n {
        let r = inv[i] as usize;
        if r > 0 {
            let j = sa[r - 1] as usize;
            while i + h < n && j + h < n && data[i + h] == data[j + h] {
                h += 1;
            }
            lcp[r] = h as u32;
            if h > 0 {
                h -= 1;
            }
        } else {
            h = 0;
        }
    }
    lcp
}

/// Sparse-table range-argmin over the LCP array.
struct RmqMin {
    log_tab: Vec<u32>,
    table: Vec<Vec<u32>>,
}

impl RmqMin {
    fn new(lcp: &[u32]) -> Self {
        let n = lcp.len();
        let mut log_tab = vec![0u32; n + 1];
        for i in 2..=n {
            log_tab[i] = log_tab[i / 2] + 1;
        }
        let mut table: Vec<Vec<u32>> = Vec::new();
        table.push((0..n as u32).collect());
        let mut k = 1usize;
        while (1usize << k) <= n {
            let span = 1usize << k;
            let half = span / 2;
            let cnt = n + 1 - span;
            let mut cur = vec![0u32; cnt];
            {
                let prev = &table[k - 1];
                for i in 0..cnt {
                    let a = prev[i];
                    let b = prev[i + half];
                    cur[i] = if lcp[b as usize] < lcp[a as usize] { b } else { a };
                }
            }
            table.push(cur);
            k += 1;
        }
        RmqMin { log_tab, table }
    }

    /// Index of a minimum of `lcp` over the inclusive range `[a, b]`.
    #[inline]
    fn argmin(&self, lcp: &[u32], a: usize, b: usize) -> u32 {
        let k = self.log_tab[b - a + 1] as usize;
        let x = self.table[k][a];
        let y = self.table[k][b + 1 - (1usize << k)];
        if lcp[y as usize] < lcp[x as usize] {
            y
        } else {
            x
        }
    }
}

/// Exact Pareto match finder, output-sensitive.
///
/// Produces bit-for-bit the same `MatchSet` as [`find_matches_exact`] for the
/// same arguments, in roughly O(n log n + output) instead of O(n * window).
///
/// - `min_match`: shortest match to report.
/// - `max_offset`: window size (max distance back).
/// - `max_len`: cap on match length recorded.
pub fn find_matches_fast(
    data: &[u8],
    min_match: usize,
    max_offset: usize,
    max_len: usize,
) -> MatchSet {
    let n = data.len();
    let mut starts = vec![0u32; n + 1];
    if n == 0 || min_match == 0 {
        return MatchSet {
            cands: Vec::new(),
            starts,
            window: max_offset as u32,
        };
    }

    let sa = build_sa(data);
    let lcp = build_lcp(data, &sa);
    let rmq = RmqMin::new(&lcp);

    // Text-order doubly linked list of the positions belonging to the LCP
    // interval currently being processed. Each position lives in exactly one
    // active list, so plain global arrays suffice.
    let mut prev_ll: Vec<u32> = vec![LL_NONE; n];
    let mut next_ll: Vec<u32> = vec![LL_NONE; n];
    for i in 0..n {
        prev_ll[i] = if i == 0 { LL_NONE } else { (i - 1) as u32 };
        next_ll[i] = if i + 1 < n { (i + 1) as u32 } else { LL_NONE };
    }

    // Emitted candidates, in DFS order (per position: increasing length).
    let mut ev_pos: Vec<u32> = Vec::new();
    let mut ev_off: Vec<u32> = Vec::new();
    let mut ev_len: Vec<u32> = Vec::new();

    let mut is_rem: Vec<bool> = vec![false; n];
    let mut old_prev: Vec<u32> = vec![LL_NONE; n];

    let mut stack: Vec<(u32, u32)> = vec![(0, (n - 1) as u32)];
    let mut minima: Vec<u32> = Vec::new();
    let mut children: Vec<(u32, u32)> = Vec::new();
    let mut rem: Vec<u32> = Vec::new();
    let mut snap_next: Vec<u32> = Vec::new();
    let mut buf: Vec<u32> = Vec::new();
    // Work stack for enumerating all minima of an LCP range in index order.
    let mut mstack: Vec<(u32, u32, bool)> = Vec::new();
    let min_match_u = min_match as u32;

    while let Some((l, r)) = stack.pop() {
        if l >= r {
            continue; // leaf: nothing left to split
        }
        let la = (l + 1) as usize;
        let rb = r as usize;
        let ell = lcp[rmq.argmin(&lcp, la, rb) as usize];

        // All split points (positions of the minimum) in increasing order.
        minima.clear();
        mstack.clear();
        mstack.push((la as u32, rb as u32, false));
        while let Some((a, b, emit)) = mstack.pop() {
            if emit {
                minima.push(a);
                continue;
            }
            if a > b {
                continue;
            }
            let m = rmq.argmin(&lcp, a as usize, b as usize);
            if lcp[m as usize] != ell {
                continue;
            }
            mstack.push((m + 1, b, false));
            mstack.push((m, m, true));
            if m > a {
                mstack.push((a, m - 1, false));
            }
        }

        children.clear();
        let mut cs = l;
        for &m in &minima {
            children.push((cs, m - 1));
            cs = m;
        }
        children.push((cs, r));

        // Keep the largest child's list in place; rebuild the rest.
        let mut keep = 0usize;
        let mut best = 0u32;
        for (t, &(a, b)) in children.iter().enumerate() {
            let sz = b - a + 1;
            if sz > best {
                best = sz;
                keep = t;
            }
        }

        rem.clear();
        for (t, &(a, b)) in children.iter().enumerate() {
            if t == keep {
                continue;
            }
            for rr in a..=b {
                rem.push(sa[rr as usize]);
            }
        }

        for &x in &rem {
            is_rem[x as usize] = true;
            old_prev[x as usize] = prev_ll[x as usize];
        }
        snap_next.clear();
        snap_next.extend(rem.iter().map(|&x| next_ll[x as usize]));

        let emit_here = ell >= min_match_u;

        // (a) Elements staying in the kept child whose predecessor is leaving:
        //     their predecessor at this node is that departing element, and this
        //     is the deepest node at which it is their predecessor.
        if emit_here {
            for (idx, &x) in rem.iter().enumerate() {
                let s = snap_next[idx];
                if s != LL_NONE && !is_rem[s as usize] {
                    ev_pos.push(s);
                    ev_off.push(s - x);
                    ev_len.push(ell);
                }
            }
        }

        // Unlink the departing elements from the parent list.
        for &x in &rem {
            let xu = x as usize;
            let p = prev_ll[xu];
            let q = next_ll[xu];
            if p != LL_NONE {
                next_ll[p as usize] = q;
            }
            if q != LL_NONE {
                prev_ll[q as usize] = p;
            }
        }

        // (b) Rebuild each non-kept child's list; a departing element whose
        //     predecessor did not come along emits here.
        for (t, &(a, b)) in children.iter().enumerate() {
            if t == keep {
                continue;
            }
            buf.clear();
            buf.extend_from_slice(&sa[a as usize..=b as usize]);
            buf.sort_unstable();
            for idx in 0..buf.len() {
                let x = buf[idx];
                let np = if idx > 0 { buf[idx - 1] } else { LL_NONE };
                let nn = if idx + 1 < buf.len() {
                    buf[idx + 1]
                } else {
                    LL_NONE
                };
                prev_ll[x as usize] = np;
                next_ll[x as usize] = nn;
                if emit_here {
                    let op = old_prev[x as usize];
                    if op != LL_NONE && op != np {
                        ev_pos.push(x);
                        ev_off.push(x - op);
                        ev_len.push(ell);
                    }
                }
            }
        }

        for &x in &rem {
            is_rem[x as usize] = false;
        }

        for &c in &children {
            stack.push(c);
        }
    }

    // Stable counting sort of the events by position: per position the DFS
    // already produced them in increasing (offset, length) order.
    let m = ev_pos.len();
    let mut fill: Vec<u32> = vec![0; n + 1];
    for &p in &ev_pos {
        fill[p as usize + 1] += 1;
    }
    for i in 0..n {
        fill[i + 1] += fill[i];
    }
    let bounds = fill.clone();
    let mut ord_off: Vec<u32> = vec![0; m];
    let mut ord_len: Vec<u32> = vec![0; m];
    for k in 0..m {
        let p = ev_pos[k] as usize;
        let t = fill[p] as usize;
        ord_off[t] = ev_off[k];
        ord_len[t] = ev_len[k];
        fill[p] += 1;
    }

    // Apply the window, the length cap and the min-match filter, collapsing runs
    // of equal (capped) length to their smallest offset.
    let mut cands: Vec<Candidate> = Vec::with_capacity(m);
    for i in 0..n {
        starts[i] = cands.len() as u32;
        let max_possible = max_len.min(n - i);
        if max_possible < min_match {
            continue;
        }
        let a = bounds[i] as usize;
        let b = bounds[i + 1] as usize;
        let mut last = 0u32;
        for k in a..b {
            let off = ord_off[k];
            if off as usize > max_offset {
                break; // offsets are increasing
            }
            let mut len = ord_len[k];
            if len as usize > max_possible {
                len = max_possible as u32;
            }
            if (len as usize) < min_match {
                continue;
            }
            if len > last {
                cands.push(Candidate {
                    offset: off,
                    length: len,
                });
                last = len;
            }
        }
    }
    starts[n] = cands.len() as u32;

    MatchSet {
        cands,
        starts,
        window: max_offset as u32,
    }
}

/// Build the complete per-position extra-offset table for the full-format DP. For every position,
/// record every distinct offset not on the Pareto front `pareto` that yields a match of length
/// >= min_match, with its maximum match length there. These are the larger offsets the Pareto
/// front hides; they let the DP establish a recurring offset early and rep-reuse it cheaply
/// (consumed via `zx_dp`'s `extra` channel).
///
/// Each position is capped to `max_per_pos` extra offsets (longest match first). O(n * window)
/// time. Offline only.
pub fn build_complete_extra(
    data: &[u8],
    pareto: &MatchSet,
    min_match: usize,
    max_offset: usize,
    max_len: usize,
    max_per_pos: usize,
) -> Vec<Vec<(u32, u32)>> {
    let n = data.len();
    let mut extra: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
    for i in 1..n {
        let max_possible = max_len.min(n - i);
        if max_possible < min_match {
            continue;
        }
        // Offsets already on the Pareto front at i are skipped (the main loop handles them).
        let pareto_offs: std::collections::HashSet<u32> =
            pareto.matches_for(i).iter().map(|c| c.offset).collect();
        let hi_off = max_offset.min(i);
        let bi = data[i];
        let mut found: Vec<(u32, u32)> = Vec::new();
        for d in 1..=hi_off {
            let src = i - d;
            if data[src] != bi {
                continue;
            }
            let mut l = 0usize;
            while l < max_possible && data[src + l] == data[i + l] {
                l += 1;
            }
            if l < min_match {
                continue;
            }
            let off = d as u32;
            if pareto_offs.contains(&off) {
                continue;
            }
            found.push((off, l as u32));
        }
        if found.is_empty() {
            continue;
        }
        if found.len() > max_per_pos {
            // Keep the longest matches; tie -> smaller offset.
            found.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
            found.truncate(max_per_pos);
        }
        extra[i] = found;
    }
    extra
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Validity: every reported match must actually match in the data, with offset in
    /// range and length within bounds, and candidates strictly increasing in length.
    fn check_valid(data: &[u8], ms: &MatchSet, min_match: usize, max_offset: usize) {
        for i in 0..data.len() {
            let mut last_len = 0u32;
            for c in ms.matches_for(i) {
                assert!(c.offset >= 1 && (c.offset as usize) <= max_offset);
                assert!((c.offset as usize) <= i, "offset points before start");
                assert!(c.length as usize >= min_match);
                assert!(c.length > last_len, "candidates not increasing in length");
                last_len = c.length;
                let src = i - c.offset as usize;
                for k in 0..c.length as usize {
                    assert_eq!(
                        data[src + k],
                        data[i + k],
                        "bad match at pos {} off {} len {} byte {}",
                        i,
                        c.offset,
                        c.length,
                        k
                    );
                }
            }
        }
    }

    #[test]
    fn empty_and_tiny() {
        for n in 0..5usize {
            let data: Vec<u8> = (0..n).map(|i| i as u8).collect();
            let ms = find_matches(&data, 2, 65536, 1024, 64);
            check_valid(&data, &ms, 2, 65536);
        }
    }

    #[test]
    fn exact_valid_and_dominates_chain() {
        // The exact finder is valid and, at every position, reaches a max length >= the chain
        // finder (it sees all offsets).
        let mut state = 99u32;
        let data: Vec<u8> = (0..8000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                (((state >> 24) as u8) % 12) as u8 // low alphabet -> many matches
            })
            .collect();
        let ex = find_matches_exact(&data, 2, 8000, 4096);
        check_valid(&data, &ex, 2, 8000);
        let ch = find_matches(&data, 2, 8000, 4096, 4096);
        for i in 0..data.len() {
            let ex_max = ex
                .matches_for(i)
                .iter()
                .map(|c| c.length)
                .max()
                .unwrap_or(0);
            let ch_max = ch
                .matches_for(i)
                .iter()
                .map(|c| c.length)
                .max()
                .unwrap_or(0);
            assert!(
                ex_max >= ch_max,
                "exact weaker than chain at {}: {} < {}",
                i,
                ex_max,
                ch_max
            );
        }
        // Exact must give the strictly smallest offset for each length it reports.
        for i in 0..data.len() {
            for c in ex.matches_for(i) {
                // No smaller offset may achieve this length.
                for d in 1..(c.offset as usize) {
                    if d > i {
                        break;
                    }
                    let src = i - d;
                    let mut l = 0usize;
                    while l < c.length as usize
                        && src + l < data.len()
                        && data[src + l] == data[i + l]
                    {
                        l += 1;
                    }
                    assert!(
                        l < c.length as usize,
                        "smaller offset {} beats {} for len {} at {}",
                        d,
                        c.offset,
                        c.length,
                        i
                    );
                }
            }
        }
    }

    #[test]
    fn repetitive() {
        let data: Vec<u8> = (0..10000).map(|i| (i % 7) as u8).collect();
        let ms = find_matches(&data, 2, 65536, 4096, 256);
        check_valid(&data, &ms, 2, 65536);
        // Should find a long match somewhere in the middle.
        let found = (0..data.len()).any(|i| ms.matches_for(i).iter().any(|c| c.length > 100));
        assert!(found, "expected long matches in repetitive data");
    }

    #[test]
    fn text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..500 {
            data.extend_from_slice(base);
        }
        let ms = find_matches(&data, 3, 65536, 4096, 256);
        check_valid(&data, &ms, 3, 65536);
    }

    #[test]
    fn random() {
        let mut state = 42u32;
        let data: Vec<u8> = (0..20000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                (state >> 24) as u8
            })
            .collect();
        let ms = find_matches(&data, 2, 65536, 1024, 128);
        check_valid(&data, &ms, 2, 65536);
    }

    /// The suffix array must equal a naive comparison sort of all suffixes.
    #[test]
    fn suffix_array_matches_naive() {
        let sizes = [0usize, 1, 2, 3, 7, 8, 15, 16, 255, 256, 257, 511, 512, 513, 1024];
        for (name, data) in crate::testcorpus::corpus(&sizes) {
            let sa = build_sa(&data);
            let mut naive: Vec<u32> = (0..data.len() as u32).collect();
            naive.sort_by(|&a, &b| data[a as usize..].cmp(&data[b as usize..]));
            assert_eq!(sa, naive, "suffix array mismatch for {name}");
            // Kasai must agree with a direct LCP computation.
            let lcp = build_lcp(&data, &sa);
            for r in 1..data.len() {
                let a = sa[r - 1] as usize;
                let b = sa[r] as usize;
                let mut h = 0usize;
                while a + h < data.len() && b + h < data.len() && data[a + h] == data[b + h] {
                    h += 1;
                }
                assert_eq!(lcp[r], h as u32, "lcp mismatch for {name} at rank {r}");
            }
        }
    }

    /// The output-sensitive finder must produce a bit-identical `MatchSet`
    /// (same `cands`, same `starts`) as the brute-force reference oracle.
    #[test]
    fn fast_matches_exact_identically() {
        let sizes = [
            0usize, 1, 2, 3, 7, 8, 15, 16, 255, 256, 257, 511, 512, 513, 1024, 4000, 9001,
        ];
        for (name, data) in crate::testcorpus::corpus(&sizes) {
            let n = data.len();
            // Cover the real encoder settings plus tighter windows / caps so the
            // max_offset and max_len paths are exercised too.
            let params: [(usize, usize, usize); 5] = [
                (3, 0xffff, 0xffffusize.min(n.max(1))),
                (3, 0xffff, 32),
                (2, 64, 1024),
                (2, 0xffff, 5),
                (4, 300, 200),
            ];
            for (min_match, max_offset, max_len) in params {
                let a = find_matches_exact(&data, min_match, max_offset, max_len);
                let b = find_matches_fast(&data, min_match, max_offset, max_len);
                assert_eq!(
                    a.starts, b.starts,
                    "starts differ for {name} mm={min_match} mo={max_offset} ml={max_len}"
                );
                assert_eq!(
                    a.cands.len(),
                    b.cands.len(),
                    "cand count differs for {name} mm={min_match} mo={max_offset} ml={max_len}"
                );
                for (k, (x, y)) in a.cands.iter().zip(b.cands.iter()).enumerate() {
                    assert!(
                        x.offset == y.offset && x.length == y.length,
                        "cand {k} differs for {name} mm={min_match} mo={max_offset} ml={max_len}: \
                         brute (off {}, len {}) vs fast (off {}, len {})",
                        x.offset,
                        x.length,
                        y.offset,
                        y.length
                    );
                }
                check_valid(&data, &b, min_match, max_offset);
            }
        }
    }

    /// Full-size cases, at the exact parameters `compress_lzsa1` uses.
    #[test]
    fn fast_matches_exact_at_full_size() {
        for (name, data) in crate::testcorpus::bench_cases() {
            let n = data.len();
            let a = find_matches_exact(&data, 3, 0xffff, 0xffffusize.min(n.max(1)));
            let b = find_matches_fast(&data, 3, 0xffff, 0xffffusize.min(n.max(1)));
            assert_eq!(a.starts, b.starts, "starts differ for {name}");
            assert_eq!(a.cands.len(), b.cands.len(), "cand count differs for {name}");
            for (k, (x, y)) in a.cands.iter().zip(b.cands.iter()).enumerate() {
                assert!(
                    x.offset == y.offset && x.length == y.length,
                    "cand {k} differs for {name}: brute (off {}, len {}) vs fast (off {}, len {})",
                    x.offset,
                    x.length,
                    y.offset,
                    y.length
                );
            }
        }
    }

    #[test]
    fn max_offset_respected() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 4) as u8).collect();
        let ms = find_matches(&data, 2, 100, 1024, 256);
        check_valid(&data, &ms, 2, 100);
    }
}
