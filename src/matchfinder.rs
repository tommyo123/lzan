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

    #[test]
    fn max_offset_respected() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 4) as u8).collect();
        let ms = find_matches(&data, 2, 100, 1024, 256);
        check_valid(&data, &ms, 2, 100);
    }
}
