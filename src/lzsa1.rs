//! LZSA1 raw-block encoder and decoder.
//!
//! Emits and decodes the LZSA1 raw-block byte stream (`lzsa -f1 -r`), forward and backward.

use crate::matchfinder::{find_matches_exact, MatchSet};

// LZSA1 format constants.
const MIN_MATCH: usize = 3; // minimum match length
const LIT_RUN: usize = 7; // literal-run token threshold
const MATCH_RUN: usize = 15; // match-run token threshold
const MIN_OFFSET: usize = 1; // smallest back-reference distance
const MAX_OFFSET: usize = 0xffff; // largest back-reference distance (raw block, <= 64 KB)
const MAX_VARLEN: usize = 0xffff; // largest representable length

/// Highest supported compression level.
pub const MAX_LEVEL: u8 = 1;

/// Compress `input` to an LZSA1 raw block. `level` is ignored (single algorithm); `backward`
/// selects the backward orientation.
pub fn compress(input: &[u8], _level: u8, backward: bool) -> Vec<u8> {
    if backward {
        compress_lzsa1_backward(input)
    } else {
        compress_lzsa1(input)
    }
}

/// Decompress an LZSA1 raw block. `backward` selects the backward orientation.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        // Backward block = reverse(forward_encode(reverse(input))); decode is the mirror.
        let mut rev: Vec<u8> = input.to_vec();
        rev.reverse();
        let mut out = decode_lzsa1_raw(&rev);
        out.reverse();
        out
    } else {
        decode_lzsa1_raw(input)
    }
}

/// Per-position match decision. `length == 0` marks a literal byte; `length >= MIN_MATCH` marks a
/// match starting here, `offset` bytes back. Positions interior to a match keep `length == 0`.
#[derive(Clone, Copy, Default)]
struct BestMatch {
    length: u32,
    offset: u32,
}

/// Compress `input` to an LZSA1 raw block decodable by `lzsa -d -f1 -r`. Returns an empty vector if
/// `input` exceeds 64 KB, the raw-block cap. Empty input yields the empty final command plus EOD.
pub fn compress_lzsa1(input: &[u8]) -> Vec<u8> {
    if input.len() > 0x10000 {
        return Vec::new();
    }
    let n = input.len();

    // Exact match front per position (smallest offset for each length).
    let ms: MatchSet = find_matches_exact(input, MIN_MATCH, MAX_OFFSET, MAX_VARLEN.min(n.max(1)));

    // Forward multi-arrival parse into a per-position best_match[] array.
    let mut best_match = forward_parse(input, &ms);

    // Token-merge pass: fold literals into matches and join matches without growing the byte cost.
    // Iterate until it stops reducing, up to 20 passes.
    let mut passes = 0;
    loop {
        let did_reduce = optimize_command_count(input, &mut best_match);
        passes += 1;
        if !did_reduce || passes >= 20 {
            break;
        }
    }

    emit_block(input, &best_match)
}

/// Compress `input` to a backward LZSA1 raw block, decodable by `lzsa -d -f1 -r -b`.
///
/// Backward output = reverse(forward_encode(reverse(input))): the input bytes are reversed, the
/// ordinary forward encoder runs, and the whole output buffer is reversed. The parse, cost model,
/// and field layout are unchanged.
pub fn compress_lzsa1_backward(input: &[u8]) -> Vec<u8> {
    let mut rev_in = input.to_vec();
    rev_in.reverse();
    let mut out = compress_lzsa1(&rev_in);
    out.reverse();
    out
}

/// One forward-parse arrival. `NARRIVALS_PER_POSITION` of these are kept per input position, sorted
/// by ascending cost.
#[derive(Clone, Copy)]
struct Arrival {
    cost: i32,         // accumulated cost in bits
    rep_offset: u32,   // last match offset on this path
    from_pos: i32,     // input position this arrival came from
    from_slot: i32,    // 1-based slot in the source position (0 = empty, -1 = origin)
    match_len: u32,    // match length used to reach here (0 = arrived via a literal)
    num_literals: u32, // pending literal run length on this path
    score: i32,        // tie-break score (fewer/cheaper commands preferred)
}

impl Default for Arrival {
    fn default() -> Self {
        Arrival {
            cost: 0x4000_0000,
            rep_offset: 0,
            from_pos: 0,
            from_slot: 0,
            match_len: 0,
            num_literals: 0,
            score: 0,
        }
    }
}

const NARRIVALS_PER_POSITION: usize = 8;
const LEAVE_ALONE_MATCH_SIZE: usize = 300;
// Mode-switch penalty is 0 (favor-ratio); score-disable is 0 for sub-64 KB inputs.

/// Forward multi-arrival parse. Produces a per-position `best_match[]` array: literal positions have
/// `length == 0`; each command's match-start position holds `(length, offset)`; positions interior
/// to a match keep `length == 0`.
///
/// Costs are in bits: literal = 8, token = 8, offset = 8/16, match varlen 0/8/16/24, plus a +8
/// literal-run boundary bump at num_literals == 7/256/512. Eight arrivals per position are kept
/// sorted by cost; ties break toward the lower score (favouring fewer commands).
fn forward_parse(input: &[u8], ms: &MatchSet) -> Vec<BestMatch> {
    let n = input.len();
    let mut best_match = vec![BestMatch::default(); n];
    if n == 0 {
        return best_match;
    }

    // arrival[pos * NARRIVALS + slot]; positions 0..=n, the last being the sink.
    let n_arr = (n + 1) * NARRIVALS_PER_POSITION;
    let mut arrival = vec![Arrival::default(); n_arr];

    // Origin: position 0, slot 0, cost 0, from_slot = -1.
    arrival[0].cost = 0;
    arrival[0].from_slot = -1;

    for i in 0..n {
        let base = i * NARRIVALS_PER_POSITION;

        // Literal extension: extend each live arrival at i by one literal into position i+1.
        let dest_lit_base = (i + 1) * NARRIVALS_PER_POSITION;
        let mut num_arrivals_for_this_pos = 0usize;
        for j in 0..NARRIVALS_PER_POSITION {
            if arrival[base + j].from_slot == 0 {
                break; // no more live arrivals at this position
            }
            num_arrivals_for_this_pos = j + 1;
            let prev_cost = arrival[base + j].cost;
            let mut coding_choice_cost = prev_cost + 8; // one literal = 8 bits
            let score = arrival[base + j].score + 1;
            let num_literals = arrival[base + j].num_literals + 1;

            // Literal-run varlen boundary: +8 bits when the run reaches 7 / 256 / 512.
            if num_literals == LIT_RUN as u32 || num_literals == 256 || num_literals == 512 {
                coding_choice_cost += 8;
            }

            let cand = Arrival {
                cost: coding_choice_cost,
                rep_offset: arrival[base + j].rep_offset,
                from_pos: i as i32,
                from_slot: (j + 1) as i32,
                match_len: 0,
                num_literals,
                score,
            };
            insert_arrival(&mut arrival, dest_lit_base, cand);
        }

        // Match extension: from the best arrival at i (slot 0), try every candidate match.
        if num_arrivals_for_this_pos != 0 {
            let cur0 = arrival[base]; // slot 0 (lowest cost)
            for c in ms.matches_for(i) {
                let cand_off = c.offset;
                let mut match_len = c.length as usize;
                let match_offset_cost = if cand_off <= 256 { 8 } else { 16 };

                if i + match_len > n {
                    match_len = n - i;
                }
                if match_len < MIN_MATCH {
                    continue;
                }

                let starting_len = if match_len >= LEAVE_ALONE_MATCH_SIZE {
                    match_len
                } else {
                    MIN_MATCH
                };

                for k in starting_len..=match_len {
                    let match_len_cost = match_varlen_size((k - MIN_MATCH) as i32);
                    let dest_base = (i + k) * NARRIVALS_PER_POSITION;
                    // token(8) + offset + match varlen.
                    let coding_choice_cost = cur0.cost + 8 + match_offset_cost + match_len_cost;

                    // Skip if a not-more-expensive arrival of the same offset-cost class already
                    // exists at the destination.
                    let mut exists = false;
                    for slot in 0..NARRIVALS_PER_POSITION {
                        let d = &arrival[dest_base + slot];
                        if d.from_slot == 0 || d.cost > coding_choice_cost {
                            break;
                        }
                        let d_off_cost = if d.rep_offset <= 256 { 8 } else { 16 };
                        if d_off_cost == match_offset_cost {
                            exists = true;
                            break;
                        }
                    }
                    if exists {
                        continue;
                    }

                    let score = cur0.score + 5;
                    // Early-out: only attempt insertion if it can beat or tie slot 0.
                    let slot0 = &arrival[dest_base];
                    if coding_choice_cost < slot0.cost
                        || (coding_choice_cost == slot0.cost && score < slot0.score)
                    {
                        let cand = Arrival {
                            cost: coding_choice_cost,
                            rep_offset: cand_off,
                            from_pos: i as i32,
                            from_slot: 1,
                            match_len: k as u32,
                            num_literals: 0,
                            score,
                        };
                        insert_arrival(&mut arrival, dest_base, cand);
                    }
                }
            }
        }
    }

    // Backtrace from the best arrival at position n, writing best_match[] entries.
    let end_base = n * NARRIVALS_PER_POSITION;
    let mut cur = arrival[end_base]; // slot 0 at the sink
    while cur.from_slot > 0 && (cur.from_pos as usize) < n {
        let p = cur.from_pos as usize;
        best_match[p].length = cur.match_len;
        best_match[p].offset = if cur.match_len != 0 {
            cur.rep_offset
        } else {
            0
        };
        let src = p * NARRIVALS_PER_POSITION + (cur.from_slot as usize - 1);
        cur = arrival[src];
    }

    best_match
}

/// Insert `cand` into the 8-slot arrival list at `dest_base`, keeping ascending cost order (ties by
/// lower score). Finds the first slot the candidate displaces, shifts the rest down by one (dropping
/// the worst), and writes it in.
#[inline]
fn insert_arrival(arrival: &mut [Arrival], dest_base: usize, cand: Arrival) {
    for nslot in 0..NARRIVALS_PER_POSITION {
        let d = &arrival[dest_base + nslot];
        if cand.cost < d.cost || (cand.cost == d.cost && cand.score < d.score) {
            // Shift [nslot .. last-1] down to [nslot+1 .. last], dropping the last slot.
            for s in (nslot + 1..NARRIVALS_PER_POSITION).rev() {
                arrival[dest_base + s] = arrival[dest_base + s - 1];
            }
            arrival[dest_base + nslot] = cand;
            return;
        }
    }
}

/// Extra bits to encode a literals length (0/8/16/24).
#[inline]
fn lit_varlen_size(nlength: i32) -> i32 {
    if nlength < LIT_RUN as i32 {
        0
    } else if nlength < 256 {
        8
    } else if nlength < 512 {
        16
    } else {
        24
    }
}

/// Extra bits to encode an encoded match length `nlength` (= actual - MIN_MATCH) (0/8/16/24).
#[inline]
fn match_varlen_size(nlength: i32) -> i32 {
    if nlength < MATCH_RUN as i32 {
        0
    } else if nlength + (MIN_MATCH as i32) < 256 {
        8
    } else if nlength + (MIN_MATCH as i32) < 512 {
        16
    } else {
        24
    }
}

/// Token-merge pass over the per-position `best_match[]` array, minimising command count without
/// growing the encoded size. Returns `true` if anything changed.
///
/// Three transforms:
///  1. Fold a literal into the following match: a literal at `i` before a match at `i+1` whose
///     back-reference also covers `i` is absorbed by extending the match one byte left, when the
///     extra match-length cost grows by at most one byte.
///  2. Replace a short match (length <= 9) between commands by literals when token + offset + varlen
///     overhead exceeds emitting the bytes as literals folded into the surrounding runs.
///  3. Join two adjacent contiguous matches into one when the combined varlen costs no more than the
///     two separate tokens + offset + varlens.
fn optimize_command_count(input: &[u8], best_match: &mut [BestMatch]) -> bool {
    let n = input.len();
    let n_end = n as i32;
    let mut num_literals: i32 = 0;
    let mut did_reduce = false;

    let mut i: i32 = 0;
    while i < n_end {
        let iu = i as usize;
        let cur_len = best_match[iu].length as i32;

        // 1. Fold a single literal into the following match.
        if cur_len == 0
            && (i + 1) < n_end
            && best_match[iu + 1].length >= MIN_MATCH as u32
            && (best_match[iu + 1].length as usize) < MAX_VARLEN
            && best_match[iu + 1].offset != 0
            && i >= best_match[iu + 1].offset as i32
            && (i + best_match[iu + 1].length as i32 + 1) <= n_end
        {
            let nxt_off = best_match[iu + 1].offset as usize;
            let nxt_len = best_match[iu + 1].length as usize;
            // The literal at i plus the next match must be reproducible from the back-reference one
            // byte earlier.
            let src = iu - nxt_off;
            if input[src..src + nxt_len + 1] == input[iu..iu + nxt_len + 1] {
                let cur_len_size =
                    match_varlen_size(best_match[iu + 1].length as i32 - MIN_MATCH as i32);
                let reduced_len_size =
                    match_varlen_size(best_match[iu + 1].length as i32 + 1 - MIN_MATCH as i32);
                if (reduced_len_size - cur_len_size) <= 8 {
                    best_match[iu].length = best_match[iu + 1].length + 1;
                    best_match[iu].offset = best_match[iu + 1].offset;
                    best_match[iu + 1].length = 0;
                    best_match[iu + 1].offset = 0;
                    did_reduce = true;
                    continue; // re-examine i
                }
            }
        }

        if cur_len >= MIN_MATCH as i32 {
            // 2. Replace a short match (<= 9) between commands by literals.
            if cur_len <= 9 && (i + cur_len) < n_end {
                let mut next_index = i + cur_len;
                let mut next_literals = 0i32;
                while next_index < n_end
                    && best_match[next_index as usize].length < MIN_MATCH as u32
                {
                    next_literals += 1;
                    next_index += 1;
                }

                let cur_offset = best_match[iu].offset as i32;
                let lhs = 8 /* token */
                    + lit_varlen_size(num_literals)
                    + if cur_offset <= 256 { 8 } else { 16 } /* offset */
                    + match_varlen_size(cur_len - MIN_MATCH as i32)
                    + 8 /* token */
                    + lit_varlen_size(next_literals);
                let rhs = 8 /* token */
                    + (cur_len << 3)
                    + lit_varlen_size(num_literals + cur_len + next_literals);
                if lhs >= rhs {
                    for j in 0..cur_len {
                        best_match[(i + j) as usize].length = 0;
                    }
                    did_reduce = true;
                    continue; // re-examine i
                }
            }

            // 3. Join two adjacent matches.
            let cur_offset = best_match[iu].offset;
            let after = (i + cur_len) as usize;
            if (i + cur_len) < n_end
                && cur_offset != 0
                && cur_len >= MIN_MATCH as i32
                && best_match[after].offset != 0
                && best_match[after].length >= MIN_MATCH as u32
                && (cur_len + best_match[after].length as i32) <= MAX_VARLEN as i32
                && (i + cur_len) >= cur_offset as i32
                && (i + cur_len) >= best_match[after].offset as i32
                && (i + cur_len + best_match[after].length as i32) <= n_end
            {
                // The second match's bytes must also follow from the first's back-reference.
                let a = (iu as i32 - cur_offset as i32 + cur_len) as usize;
                let b = (i + cur_len - best_match[after].offset as i32) as usize;
                let next_len = best_match[after].length as usize;
                if input[a..a + next_len] == input[b..b + next_len] {
                    let mut cur_partial = match_varlen_size(cur_len - MIN_MATCH as i32);
                    cur_partial += 8 /* token */
                        + if best_match[after].offset <= 256 { 8 } else { 16 } /* offset */
                        + match_varlen_size(best_match[after].length as i32 - MIN_MATCH as i32);
                    let reduced_partial = match_varlen_size(
                        cur_len + best_match[after].length as i32 - MIN_MATCH as i32,
                    );
                    if cur_partial >= reduced_partial {
                        best_match[iu].length += best_match[after].length;
                        best_match[after].length = 0;
                        best_match[after].offset = 0;
                        did_reduce = true;
                        continue; // re-examine i
                    }
                }
            }

            i += cur_len;
            num_literals = 0;
        } else {
            num_literals += 1;
            i += 1;
        }
    }

    did_reduce
}

/// Serialise the per-position `best_match[]` array into the LZSA1 raw block. Accumulates literals
/// until a match start, emits the command, then a final literal-only command (`(LLL<<4)|0x0f`)
/// carrying trailing literals, then the EOD tail.
fn emit_block(input: &[u8], best_match: &[BestMatch]) -> Vec<u8> {
    let n = input.len();
    let mut out: Vec<u8> = Vec::with_capacity(n / 2 + 16);

    let mut num_literals = 0usize;
    let mut first_lit_offset = 0usize; // start of the pending literal run in `input`

    let mut i = 0usize;
    while i < n {
        let m = best_match[i];
        if m.length >= MIN_MATCH as u32 {
            let match_offset = m.offset as usize;
            let match_len = m.length as usize;
            let enc = match_len - MIN_MATCH; // encoded match length
            let token_lit = if num_literals >= LIT_RUN {
                LIT_RUN
            } else {
                num_literals
            };
            let token_match = if enc >= MATCH_RUN { MATCH_RUN } else { enc };
            let long_off = if match_offset <= 256 { 0x00u8 } else { 0x80u8 };

            // Raw block offsets must be in [MIN_OFFSET, MAX_OFFSET].
            debug_assert!(match_offset >= MIN_OFFSET && match_offset <= MAX_OFFSET);

            out.push(long_off | ((token_lit as u8) << 4) | (token_match as u8));
            write_lit_varlen(&mut out, num_literals);

            if num_literals != 0 {
                out.extend_from_slice(&input[first_lit_offset..first_lit_offset + num_literals]);
                num_literals = 0;
            }

            // Offset stored as negative distance: lo = (-off) & 0xff; hi = ((-off) >> 8) & 0xff.
            let neg = (match_offset as i32).wrapping_neg();
            out.push((neg & 0xff) as u8);
            if long_off != 0 {
                out.push(((neg >> 8) & 0xff) as u8);
            }
            write_match_varlen(&mut out, enc);

            i += match_len;
        } else {
            if num_literals == 0 {
                first_lit_offset = i;
            }
            num_literals += 1;
            i += 1;
        }
    }

    // Final literal-only command carrying trailing literals (raw block uses MMMM = 0x0f).
    let token_lit = if num_literals >= LIT_RUN {
        LIT_RUN
    } else {
        num_literals
    };
    out.push(((token_lit as u8) << 4) | 0x0f);
    write_lit_varlen(&mut out, num_literals);
    if num_literals != 0 {
        out.extend_from_slice(&input[first_lit_offset..first_lit_offset + num_literals]);
    }

    // Raw-block EOD marker: 00 EE 00 00.
    out.push(0x00);
    out.push(238);
    out.push(0x00);
    out.push(0x00);

    out
}

/// Write a literals-length varint.
#[inline]
fn write_lit_varlen(out: &mut Vec<u8>, nlit: usize) {
    if nlit >= LIT_RUN {
        if nlit < 256 {
            out.push((nlit - LIT_RUN) as u8);
        } else if nlit < 512 {
            out.push(250);
            out.push((nlit - 256) as u8);
        } else {
            out.push(249);
            out.push((nlit & 0xff) as u8);
            out.push(((nlit >> 8) & 0xff) as u8);
        }
    }
}

/// Write a match-length varint. `enc` is the encoded length (actual - MIN_MATCH).
#[inline]
fn write_match_varlen(out: &mut Vec<u8>, enc: usize) {
    if enc >= MATCH_RUN {
        let actual = enc + MIN_MATCH;
        if actual < 256 {
            out.push((enc - MATCH_RUN) as u8);
        } else if actual < 512 {
            out.push(239);
            out.push((actual - 256) as u8);
        } else {
            out.push(238);
            out.push((actual & 0xff) as u8);
            out.push(((actual >> 8) & 0xff) as u8);
        }
    }
}

/// LZSA1 raw-block decoder.
pub fn decode_lzsa1_raw(data: &[u8]) -> Vec<u8> {
    decode_lzsa1_raw_with_gap(data).0
}

/// Decode and also return the in-place safety gap (bytes) the stream needs:
/// `max(output_produced - input_consumed)` over the decode, minus its final
/// value. Any in-place layout (forward top-aligned or backward) must keep the
/// write head at least this many bytes clear of the read head, or it will
/// clobber unread compressed bytes - a token whose match copies output LATE (or
/// a stream that is momentarily larger than the output produced so far) makes
/// the running compression peak above its final value and the fixed margin is
/// no longer enough. See [`max_gap_forward`] / [`max_gap_backward`].
fn decode_lzsa1_raw_with_gap(data: &[u8]) -> (Vec<u8>, i32) {
    if data.is_empty() {
        return (Vec::new(), 0);
    }

    let mut out: Vec<u8> = Vec::new();
    let mut ip = 0usize;
    let end = data.len();
    // Peak of (produced - consumed) at a token boundary. The gap grows during a
    // match copy (output advances, input does not) and peaks at the match's end,
    // which is the state observed at the next loop iteration's top; a token that
    // consumes stream bytes with little output drives it back down.
    let mut max_gap = 0i32;

    while ip < end {
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let token = data[ip];
        ip += 1;
        let mut nlit = ((token & 0x70) >> 4) as usize;
        if nlit == LIT_RUN {
            let b = data[ip] as usize;
            ip += 1;
            nlit += b;
            if b == 250 {
                nlit = 256 + data[ip] as usize;
                ip += 1;
            } else if b == 249 {
                nlit = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
                ip += 2;
            }
        }
        for _ in 0..nlit {
            out.push(data[ip]);
            ip += 1;
        }
        // Raw blocks carry an explicit EOD; the match fields follow unless the stream ends here.
        if ip + 1 < end {
            let mut off = (data[ip] as i32) ^ 0xff;
            ip += 1;
            if token & 0x80 != 0 {
                off |= ((data[ip] as i32) << 8) ^ 0xff00;
                ip += 1;
            }
            off += 1; // off holds the distance D
            let mut mlen = (token & 0x0f) as usize;
            mlen += MIN_MATCH;
            if mlen == MATCH_RUN + MIN_MATCH {
                let b = data[ip] as usize;
                ip += 1;
                mlen += b;
                if b == 239 {
                    mlen = 256 + data[ip] as usize;
                    ip += 1;
                } else if b == 238 {
                    mlen = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
                    ip += 2;
                }
                if mlen == 0 {
                    break; // EOD
                }
            }
            let src = out.len() - off as usize;
            for k in 0..mlen {
                let v = out[src + k];
                out.push(v);
            }
        }
    }

    // The read head consumes the whole `data.len()`-byte block; use it (not
    // `ip`, which stops at EOD) so the final gap is the true end state.
    let final_gap = out.len() as i32 - data.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD LZSA1 raw stream: the
/// top-aligned packed block must start at least this many bytes above the output
/// end, or the decoder's write head overtakes unread compressed data. See
/// [`decode_lzsa1_raw_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        decode_lzsa1_raw_with_gap(stream).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD LZSA1 raw stream: the packed
/// block must sit at least this many bytes below the span start. A backward
/// block is `reverse(forward_encode(reverse(input)))`, and the 6502 backward
/// decoder reads the stored stream from its END - which is exactly a forward
/// decode of the reversed stream, so the gap sequence matches.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        let rev: Vec<u8> = stream.iter().rev().copied().collect();
        decode_lzsa1_raw_with_gap(&rev).1.max(0) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrips(data: &[u8]) {
        let blob = compress_lzsa1(data);
        let dec = decode_lzsa1_raw(&blob);
        assert_eq!(dec, data, "lzsa1 roundtrip len {}", data.len());

        // Backward variant via the uniform API.
        let bwd = compress(data, MAX_LEVEL, true);
        let dec_b = decompress(&bwd, true);
        assert_eq!(dec_b, data, "lzsa1 backward roundtrip len {}", data.len());

        // Forward via the uniform API.
        let fwd = compress(data, MAX_LEVEL, false);
        assert_eq!(
            decompress(&fwd, false),
            data,
            "lzsa1 uniform forward len {}",
            data.len()
        );
    }

    #[test]
    fn tiny() {
        roundtrips(&[]);
        roundtrips(&[42]);
        roundtrips(&[1, 2, 3, 4, 5]);
        roundtrips(b"abcabcabcabcabc");
        roundtrips(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    }

    #[test]
    fn repetitive() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
        roundtrips(&data);
    }

    #[test]
    fn all_zeros_long_run() {
        // Exercises the 2-byte match-length extension (238) path.
        let data = vec![0u8; 40000];
        roundtrips(&data);
    }

    #[test]
    fn long_literal_run() {
        // Exercises the literal-length extension paths (250 / 249).
        let mut state = 0x1234_5678u32;
        let data: Vec<u8> = (0..2000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                (state >> 24) as u8
            })
            .collect();
        roundtrips(&data);
    }

    #[test]
    fn text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        roundtrips(&data);
    }

    #[test]
    fn two_byte_offset() {
        // Force a far (>256) back-reference.
        let mut data: Vec<u8> = (0..1000).map(|i| (i & 0xff) as u8).collect();
        let tail: Vec<u8> = (0..400).map(|i| (i & 0xff) as u8).collect();
        data.extend_from_slice(&tail);
        roundtrips(&data);
    }

    /// Independent reference: recompute the in-place gap by sampling
    /// `produced - consumed` after EVERY output push (both literal and match
    /// copies), not just at token boundaries. If [`max_gap_forward`] ever
    /// undercounts (misses a peak), this will disagree with it. A too-small gap
    /// is unsafe, so the equality below is the load-bearing assertion.
    fn perstep_gap(data: &[u8]) -> usize {
        if data.is_empty() {
            return 0;
        }
        let mut out: Vec<u8> = Vec::new();
        let mut ip = 0usize;
        let end = data.len();
        let mut max_gap = 0i32;
        macro_rules! sample {
            () => {{
                let g = out.len() as i32 - ip as i32;
                if g > max_gap {
                    max_gap = g;
                }
            }};
        }
        while ip < end {
            sample!();
            let token = data[ip];
            ip += 1;
            let mut nlit = ((token & 0x70) >> 4) as usize;
            if nlit == LIT_RUN {
                let b = data[ip] as usize;
                ip += 1;
                nlit += b;
                if b == 250 {
                    nlit = 256 + data[ip] as usize;
                    ip += 1;
                } else if b == 249 {
                    nlit = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
                    ip += 2;
                }
            }
            for _ in 0..nlit {
                out.push(data[ip]);
                ip += 1;
                sample!();
            }
            if ip + 1 < end {
                let mut off = (data[ip] as i32) ^ 0xff;
                ip += 1;
                if token & 0x80 != 0 {
                    off |= ((data[ip] as i32) << 8) ^ 0xff00;
                    ip += 1;
                }
                off += 1;
                let mut mlen = (token & 0x0f) as usize;
                mlen += MIN_MATCH;
                if mlen == MATCH_RUN + MIN_MATCH {
                    let b = data[ip] as usize;
                    ip += 1;
                    mlen += b;
                    if b == 239 {
                        mlen = 256 + data[ip] as usize;
                        ip += 1;
                    } else if b == 238 {
                        mlen = (data[ip] as usize) | ((data[ip + 1] as usize) << 8);
                        ip += 2;
                    }
                    if mlen == 0 {
                        break;
                    }
                }
                let src = out.len() - off as usize;
                for k in 0..mlen {
                    let v = out[src + k];
                    out.push(v);
                    sample!();
                }
            }
        }
        let final_gap = out.len() as i32 - data.len() as i32;
        (max_gap - final_gap).max(0) as usize
    }

    #[test]
    fn in_place_gap_is_measured_and_bounded() {
        // Unlike bit-packed formats (apultra, bb2), LZSA1 stores literals
        // byte-aligned, so an incompressible block barely expands: its in-place
        // safety gap is only a handful of bytes (the trailing literal token +
        // the 4-byte EOD), well inside the fixed 32-byte default. The gap must
        // still be NON-ZERO - the expansion is really being measured - and the
        // token-boundary sampling in `max_gap_forward` must equal an
        // independent per-output-step recomputation (never undercount a peak).
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();

        let fwd = compress_lzsa1(&noise);
        let bwd = compress_lzsa1_backward(&noise);
        let gap_f = max_gap_forward(&fwd);
        let gap_b = max_gap_backward(&bwd);

        // Non-zero (expansion is measured) and inside the default margin.
        assert!(
            (1..=32).contains(&gap_f),
            "forward gap {gap_f} out of range"
        );
        assert!(
            (1..=32).contains(&gap_b),
            "backward gap {gap_b} out of range"
        );

        // Load-bearing safety check: the loop-top sampling must not undercount
        // relative to sampling at every output-producing step.
        assert_eq!(gap_f, perstep_gap(&fwd), "forward gap undercounts a peak");
        let bwd_rev: Vec<u8> = bwd.iter().rev().copied().collect();
        assert_eq!(
            gap_b,
            perstep_gap(&bwd_rev),
            "backward gap undercounts a peak"
        );

        // Highly compressible data also fits the default margin.
        let zeros = vec![0u8; 8192];
        assert!(max_gap_forward(&compress_lzsa1(&zeros)) <= 32);
        assert!(max_gap_backward(&compress_lzsa1_backward(&zeros)) <= 32);

        // Empty streams need no margin.
        assert_eq!(max_gap_forward(&[]), 0);
        assert_eq!(max_gap_backward(&[]), 0);

        // The instrumentation must not change decode output.
        assert_eq!(
            decode_lzsa1_raw(&fwd),
            noise,
            "with_gap wrapper altered output"
        );
    }
}
