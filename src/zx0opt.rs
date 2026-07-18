//! Exact rep0-only optimal parser (the ZX0 optimal parse). For the rep0-only grammar
//! (mode 0x01) its bit cost equals the ZX0 optimum.
//!
//! ## Approach
//!
//! ZX0 keeps a DP chain per offset (`last_literal[off]`, `last_match[off]`) and relaxes all of
//! them at every position. Its state is (position, last_offset), exact for the rep0 grammar by
//! construction. Unlike a Pareto-front match finder (smallest offset per length), this considers
//! every offset that matches at each position, including the larger offsets that code cheaply as
//! a token-only rep0.
//!
//! ## Cost model (bit-exact vs zx.rs's `encode_with`, rep_slots=1)
//!
//!   - literal run L : `1 + gamma(L) + 8L`     == flag(1) + lit_run_bits(L) + 8L
//!   - rep0 (len L)  : `1 + gamma(L)`          == flag(1) + rep_len_bits(L)   [no rep index]
//!   - new off,len   : `8 + gamma((off-1)/128+1) + gamma(len-1)`
//!                       == flag(1) + offset_cost_bits_hc(off)[=7+gamma(msb)] + newoff_len_bits(len-1)
//! `elias_gamma_bits` matches `zx::gamma_bits`, so an optimal BLOCK chain replayed as `ZxCommand`s
//! and encoded with `encode_with(_, _, 1)` yields exactly `optimal->bits` bits (ZX0's in-stream
//! EOF marker is dropped; termination is driven by orig_len).
//!
//! ## Memory
//!
//! Blocks are refcounted and recycled via a free list (`ghost_root`); see [`Pool`]. The DP only
//! points at the active frontier (`last_literal`/`last_match` per offset + `optimal` per
//! position), so live memory is bounded by O(n + window), not the O(n*window) relaxation count.
//! Time is O(n*window).
//!
//! ## Output
//!
//! Returns the optimal command list (`[literal-run][match]` units, terminal literal-only allowed),
//! ready for `zx::encode_with(input, &cmds, 1)`. The full-format DP consumes the same rep-enabling
//! offsets via `matchfinder::build_complete_extra`.

use crate::zx::ZxCommand;

/// ZX0's INITIAL_OFFSET. The fake starting block sits at index `skip-1` with this offset.
const INITIAL_OFFSET: i64 = 1;

/// ZX0's default (non-quick) offset limit.
pub const ZX0_MAX_OFFSET: usize = 32640;

#[inline]
fn elias_gamma_bits(value: i64) -> i64 {
    // 1 + 2*floor(log2(value)); value >= 1. Matches zx::gamma_bits.
    let mut bits = 1i64;
    let mut v = value;
    v >>= 1;
    while v != 0 {
        bits += 2;
        v >>= 1;
    }
    bits
}

#[inline]
fn offset_ceiling(index: i64, offset_limit: i64) -> i64 {
    if index > offset_limit {
        offset_limit
    } else if index < INITIAL_OFFSET {
        INITIAL_OFFSET
    } else {
        index
    }
}

/// One DP node, the analogue of ZX0's `BLOCK`. The chain is stored by index into a node arena
/// (`Vec<Node>`). `references` is the refcount and `ghost_chain` is the free-list link, so dead
/// nodes are recycled (a `ghost_root` pool). See [`Pool`].
//
// 24 bytes/node. The arena's high-water mark is the peak number of simultaneously-live nodes (the
// active frontier) = O(n + window).
#[derive(Clone, Copy)]
struct Node {
    bits: i32,
    index: i32, // last input position consumed by this block (0-based); fake start = skip-1
    offset: i32, // 0 => literals, else the match offset
    chain: u32, // parent node id, or NIL
    references: u32, // how many slots/chains point at this node
    ghost_chain: u32, // free-list link when this node is dead (refcount 0), else unused
}

const NIL: u32 = u32::MAX;

/// Refcounting node pool: a `ghost_root` recycler fused with the optimizer's
/// `allocate`/`assign`. Each block is refcounted; when its refcount hits 0 it goes on the
/// `ghost_root` free list and is handed back by the next `allocate`. Because the DP only points at
/// the active frontier (`last_literal`/`last_match` per offset + `optimal` per position), the
/// pool's high-water mark is bounded by O(n + window).
///
/// `allocate`/`assign` use pointers represented as arena indices (`u32`,
/// `NIL` == NULL). The "ghost" deferred-free semantics (a dead block's own `chain` ref is
/// decremented lazily on recycle, possibly cascading) are reproduced.
struct Pool {
    nodes: Vec<Node>,
    ghost_root: u32, // head of the free list (recycled dead nodes), or NIL
}

impl Pool {
    fn with_capacity(cap: usize) -> Self {
        Pool {
            nodes: Vec::with_capacity(cap),
            ghost_root: NIL,
        }
    }

    #[inline]
    fn bits(&self, id: u32) -> i64 {
        self.nodes[id as usize].bits as i64
    }
    #[inline]
    fn index(&self, id: u32) -> i64 {
        self.nodes[id as usize].index as i64
    }

    /// Allocate a node id with `references == 0` (the caller takes the
    /// reference via [`Pool::assign`]). Reuses a ghost (dead) node when one is available -
    /// decrementing that ghost's old `chain` ref, which may cascade more nodes onto the ghost list
    /// - otherwise grows the arena. `chain`'s refcount is incremented.
    #[inline]
    fn allocate(&mut self, bits: i64, index: i64, offset: i64, chain: u32) -> u32 {
        let ptr;
        if self.ghost_root != NIL {
            ptr = self.ghost_root;
            // Pop the ghost.
            self.ghost_root = self.nodes[ptr as usize].ghost_chain;
            // The recycled block's old chain loses a reference; if it drops to 0, it too becomes a ghost.
            let old_chain = self.nodes[ptr as usize].chain;
            if old_chain != NIL {
                let refs = &mut self.nodes[old_chain as usize].references;
                *refs -= 1;
                if *refs == 0 {
                    self.nodes[old_chain as usize].ghost_chain = self.ghost_root;
                    self.ghost_root = old_chain;
                }
            }
            let nd = &mut self.nodes[ptr as usize];
            nd.bits = bits as i32;
            nd.index = index as i32;
            nd.offset = offset as i32;
            nd.chain = chain;
            nd.references = 0;
        } else {
            ptr = self.nodes.len() as u32;
            self.nodes.push(Node {
                bits: bits as i32,
                index: index as i32,
                offset: offset as i32,
                chain,
                references: 0,
                ghost_chain: NIL,
            });
        }
        if chain != NIL {
            self.nodes[chain as usize].references += 1;
        }
        ptr
    }

    /// Store `chain` into the slot `*slot`, taking a reference on `chain`
    /// and releasing the slot's previous occupant (which becomes a ghost if its refcount hits 0).
    #[inline]
    fn assign(&mut self, slot: &mut u32, chain: u32) {
        self.nodes[chain as usize].references += 1;
        let old = *slot;
        if old != NIL {
            let refs = &mut self.nodes[old as usize].references;
            *refs -= 1;
            if *refs == 0 {
                self.nodes[old as usize].ghost_chain = self.ghost_root;
                self.ghost_root = old;
            }
        }
        *slot = chain;
    }
}

/// The ZX0 optimal parse. Returns the optimal command list for the
/// rep0-only grammar over `input` with the given `offset_limit` (`ZX0_MAX_OFFSET` matches the
/// original `zx0.exe`; a larger limit, e.g. 65535, exploits the wider window).
///
/// `skip` is ZX0's `skip` (0 for whole-file compression).
pub fn optimize_zx0(input: &[u8], skip: usize, offset_limit: usize) -> Vec<ZxCommand> {
    optimize_zx0_with_bits(input, skip, offset_limit).0
}

/// Same as `optimize_zx0` but also returns ZX0's internal `optimal->bits` (the head node's bit
/// cost), the value the original `zx0.exe` reports.
pub fn optimize_zx0_with_bits(
    input: &[u8],
    skip: usize,
    offset_limit: usize,
) -> (Vec<ZxCommand>, i64) {
    let n = input.len();
    if n == 0 {
        return (Vec::new(), 0);
    }
    let input_size = n as i64;
    let skip_i = skip as i64;
    let offset_limit = offset_limit as i64;
    let mut max_offset = offset_ceiling(input_size - 1, offset_limit);
    let mo = max_offset as usize;
    // The live frontier is at most one node per `optimal` slot (n) plus one per
    // `last_literal`/`last_match` offset slot (2*(mo+1)), plus a small transient. Seed the
    // capacity to that size to avoid early reallocs.
    let mut pool = Pool::with_capacity(n + 2 * (mo + 1) + 16);
    let mut last_literal = vec![NIL; mo + 1];
    let mut last_match = vec![NIL; mo + 1];
    let mut optimal = vec![NIL; n];
    let mut match_length = vec![0i64; mo + 1];
    let mut best_length = vec![0i64; n];
    if input_size > 2 {
        best_length[2] = 2;
    }
    // start with the fake block: assign(&last_match[INITIAL_OFFSET], allocate(...))
    {
        let fake = pool.allocate(-1, skip_i - 1, INITIAL_OFFSET, NIL);
        pool.assign(&mut last_match[INITIAL_OFFSET as usize], fake);
    }
    let mut best_length_size: i64;
    let mut index = skip_i;
    while index < input_size {
        best_length_size = 2;
        max_offset = offset_ceiling(index, offset_limit);
        let iu = index as usize;
        let mut offset = 1i64;
        while offset <= max_offset {
            let ou = offset as usize;
            if index != skip_i && index >= offset && input[iu] == input[(index - offset) as usize] {
                if last_literal[ou] != NIL {
                    let ll = last_literal[ou];
                    let length = index - pool.index(ll);
                    let bits = pool.bits(ll) + 1 + elias_gamma_bits(length);
                    // assign(&last_match[offset], allocate(bits, index, offset, last_literal[offset]))
                    let node = pool.allocate(bits, index, offset, ll);
                    pool.assign(&mut last_match[ou], node);
                    if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
                        // assign(&optimal[index], last_match[offset])
                        let lm = last_match[ou];
                        pool.assign(&mut optimal[iu], lm);
                    }
                }
                match_length[ou] += 1;
                if match_length[ou] > 1 {
                    if best_length_size < match_length[ou] {
                        // init from the VALUE best_length[best_length_size]
                        // (call it bl): optimal[index-bl].bits + gamma(bl-1). NOT gamma(size-1).
                        let bl = best_length[best_length_size as usize];
                        let mut bits =
                            pool.bits(optimal[(index - bl) as usize]) + elias_gamma_bits(bl - 1);
                        loop {
                            best_length_size += 1;
                            let bits2 = pool.bits(optimal[(index - best_length_size) as usize])
                                + elias_gamma_bits(best_length_size - 1);
                            if bits2 <= bits {
                                best_length[best_length_size as usize] = best_length_size;
                                bits = bits2;
                            } else {
                                best_length[best_length_size as usize] =
                                    best_length[(best_length_size - 1) as usize];
                            }
                            if best_length_size >= match_length[ou] {
                                break;
                            }
                        }
                    }
                    let length = best_length[match_length[ou] as usize];
                    let bits = pool.bits(optimal[(index - length) as usize])
                        + 8
                        + elias_gamma_bits((offset - 1) / 128 + 1)
                        + elias_gamma_bits(length - 1);
                    let lm = last_match[ou];
                    if lm == NIL || pool.index(lm) != index || pool.bits(lm) > bits {
                        let chain = optimal[(index - length) as usize];
                        // assign(&last_match[offset], allocate(bits, index, offset, optimal[index-length]))
                        let node = pool.allocate(bits, index, offset, chain);
                        pool.assign(&mut last_match[ou], node);
                        if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
                            // assign(&optimal[index], last_match[offset])
                            let lm2 = last_match[ou];
                            pool.assign(&mut optimal[iu], lm2);
                        }
                    }
                }
            } else {
                match_length[ou] = 0;
                if last_match[ou] != NIL {
                    let lm = last_match[ou];
                    let length = index - pool.index(lm);
                    let bits = pool.bits(lm) + 1 + elias_gamma_bits(length) + length * 8;
                    // assign(&last_literal[offset], allocate(bits, index, 0, last_match[offset]))
                    let node = pool.allocate(bits, index, 0, lm);
                    pool.assign(&mut last_literal[ou], node);
                    if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
                        // assign(&optimal[index], last_literal[offset])
                        let ll2 = last_literal[ou];
                        pool.assign(&mut optimal[iu], ll2);
                    }
                }
            }
            offset += 1;
        }
        index += 1;
    }
    let head = optimal[(input_size - 1) as usize];
    let head_bits = pool.bits(head);
    (chain_to_commands(input, &pool, head, skip_i), head_bits)
}

/// Convert the reversed BLOCK chain into `ZxCommand`s (forward walk over the chain,
/// emitting commands instead of bits). The chain links child->parent; reverse it, then walk forward
/// grouping `[literal-run][match]` units. A literal block (offset==0) starts/extends a pending
/// literal run; a match block (offset!=0) closes the current command with that match.
fn chain_to_commands(input: &[u8], pool: &Pool, head: u32, skip: i64) -> Vec<ZxCommand> {
    let arena = &pool.nodes;
    // Reverse the chain into forward order.
    let mut chain: Vec<u32> = Vec::new();
    let mut cur = head;
    while cur != NIL {
        chain.push(cur);
        cur = arena[cur as usize].chain;
    }
    chain.reverse();
    // chain[0] is the fake initial block (index == skip-1, offset == INITIAL_OFFSET). The real
    // blocks follow. Walk consecutive pairs: each block consumes (block.index - prev.index) bytes
    // starting just after prev.index.
    let mut cmds: Vec<ZxCommand> = Vec::new();
    let mut pending_lit_len: u32 = 0;
    let mut pending_lit_start: usize = skip.max(0) as usize;

    let mut prev_index = arena[chain[0] as usize].index; // = skip-1
    for &nid in &chain[1..] {
        let node = &arena[nid as usize];
        let length = node.index - prev_index;
        let start = (prev_index + 1) as usize;
        if node.offset == 0 {
            // literal block: extend the pending literal run.
            if pending_lit_len == 0 {
                pending_lit_start = start;
            }
            pending_lit_len += length as u32;
        } else {
            // match block: emit a command [pending literals][match].
            cmds.push(ZxCommand {
                lit_len: pending_lit_len,
                lit_start: pending_lit_start,
                match_off: node.offset as u32,
                match_len: length as u32,
                near_rep_ri: -1,
            });
            pending_lit_len = 0;
            pending_lit_start = (node.index + 1) as usize;
        }
        prev_index = node.index;
    }
    if pending_lit_len > 0 {
        cmds.push(ZxCommand {
            lit_len: pending_lit_len,
            lit_start: pending_lit_start,
            match_off: 0,
            match_len: 0,
            near_rep_ri: -1,
        });
    }
    let _ = input;
    cmds
}

/// ZX0-exact rep0-only compress (mode 0x01 blob). Pass `offset_limit = ZX0_MAX_OFFSET` to match
/// the original `zx0.exe`.
pub fn compress_zx0_exact(input: &[u8], offset_limit: usize) -> Vec<u8> {
    let cmds = optimize_zx0(input, 0, offset_limit);
    crate::zx::encode_with(input, &cmds, 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrips(data: &[u8]) {
        let blob = compress_zx0_exact(data, ZX0_MAX_OFFSET);
        let out = crate::zx::decode(&blob, data.len());
        assert_eq!(out, data, "zx0opt roundtrip len {}", data.len());
    }

    #[test]
    fn tiny_and_empty() {
        roundtrips(&[]);
        roundtrips(&[42]);
        roundtrips(&[1, 2, 3, 4, 5]);
        roundtrips(b"abcabcabcabcabc");
    }

    #[test]
    fn repetitive() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
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
    fn pseudo_random() {
        let mut state = 12345u32;
        let data: Vec<u8> = (0..6000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                ((state >> 24) % 20) as u8
            })
            .collect();
        roundtrips(&data);
    }
}
