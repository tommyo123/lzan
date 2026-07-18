//! Optimal LZ parse via forward dynamic programming with multi-arrival state.
//!
//! Each position keeps up to `ARRIVALS` candidate arrivals, each carrying its own rep0-3
//! queue and accumulated cost. Transitions:
//!   - literal:  cost += lit_cost(input[i])
//!   - match:    cost += token + offset_bytes + matchlen_ext  (explicit), token-only (rep)
//!
//! Token accounting: a command is `[literals][match]`; the single token byte is charged at the
//! match transition (the preceding literal run shares that command's token). Trailing literals
//! form a final literal-only command (its token is a small undercount; exact sizes come from
//! serialization).
//!
//! Keeping N arrivals per position lets the parser carry distinct rep queues forward and let the
//! global backtrack pick the cheapest end-to-end path. The serialize layer replays rep state
//! independently from the resulting commands, so the format is unaffected.
//!
//! The cost model is pluggable: a static byte-cost model, or an entropy-derived model filled from
//! real stream histograms for an entropy-aware re-parse.

use crate::matchfinder::MatchSet;

pub const SCALE: u32 = 16; // fixed-point: cost units are bits * 16
const INF: u32 = u32::MAX / 2;

/// Matches longer than this are only relaxed at full length (splitting a huge match is rarely
/// beneficial).
const LEAVE_ALONE: u32 = 300;

/// Cap on the length a rep-offset probe extends in the parse (short reps matter; longer repeats
/// are found by the hash chain). Bounds worst-case work in RLE regions.
const REP_MAX_LEN: usize = 64;

/// Arrivals kept per position; wider carries more distinct rep queues forward. `from_slot` is
/// packed into `FROM_SLOT_BITS` of the backtrack pointer, so ARRIVALS must be <= 2^FROM_SLOT_BITS.
const ARRIVALS: usize = 16;

pub const MIN_MATCH: u32 = 2;

#[derive(Clone, Copy)]
pub struct Command {
    pub lit_len: u32,
    pub lit_start: usize,
    pub match_off: u32, // 0 => final literal-only command
    pub match_len: u32,
}

/// Cost model in bits*SCALE units.
pub struct CostModel {
    pub lit: [u32; 256],     // per-literal-byte cost
    pub tok_explicit: u32,   // token byte cost for an explicit (non-rep) match command
    pub off_slot: [u32; 33], // cost of each offset-slot symbol (slot 0..=32 for u32 offsets)
    pub ext_byte: u32,       // cost per length-extension byte (match/lit)
}

/// Offset slot = bit-length of (off-1). slot 0 => off 1; slot k => off in [2^(k-1)+1, 2^k].
/// Extra bits emitted = slot.saturating_sub(1) (the top bit is implied).
#[inline]
pub fn offset_slot(off: u32) -> u32 {
    32 - (off - 1).leading_zeros() // bit_length(off-1); off>=1
}

impl CostModel {
    /// Static bootstrap model: slot symbol ~3 bits + (slot-1) uniform extra bits, so the parse
    /// prefers small offsets before entropy stats are known.
    pub fn static_model() -> Self {
        let mut off_slot = [0u32; 33];
        for (k, c) in off_slot.iter_mut().enumerate() {
            *c = 3 * SCALE + (k as u32).saturating_sub(1) * SCALE;
        }
        CostModel {
            lit: [8 * SCALE; 256],
            tok_explicit: 8 * SCALE,
            off_slot,
            ext_byte: 8 * SCALE,
        }
    }

    #[inline]
    fn offset_cost(&self, off: u32) -> u32 {
        let slot = offset_slot(off);
        // slot symbol cost + (slot-1) raw extra bits (1 bit each)
        self.off_slot[slot as usize] + slot.saturating_sub(1) * SCALE
    }

    #[inline]
    fn matchlen_ext_cost(&self, len: u32) -> u32 {
        // encoded = len - MIN_MATCH; ml_code = min(encoded, 7); ext when encoded >= 7 (len>=9)
        if len < 9 {
            0
        } else {
            let v = len - 9; // varint of (len-9)
            if v < 255 {
                self.ext_byte
            } else {
                5 * self.ext_byte
            }
        }
    }

    #[inline]
    pub fn match_cost(&self, off: u32, len: u32) -> u32 {
        self.tok_explicit + self.offset_cost(off) + self.matchlen_ext_cost(len)
    }

    /// Cost of a rep match: token only (no offset bytes), plus any match-length extension.
    #[inline]
    pub fn match_cost_rep(&self, len: u32) -> u32 {
        self.tok_explicit + self.matchlen_ext_cost(len)
    }
}

#[inline]
fn rep_insert(r: [u32; 4], off: u32) -> [u32; 4] {
    [off, r[0], r[1], r[2]]
}
#[inline]
fn rep_mtf(r: [u32; 4], idx: usize) -> [u32; 4] {
    let mut o = r;
    let v = o[idx];
    let mut i = idx;
    while i > 0 {
        o[i] = o[i - 1];
        i -= 1;
    }
    o[0] = v;
    o
}

/// One arrival (DP state) at a position: the cheapest way to reach it carrying a particular rep
/// queue. The backtrack pointer is (from_pos, from_slot): `from_slot` is the arrival index at
/// `from_pos` it came from, packed into the top `FROM_SLOT_BITS` of `from_packed`; the low bits
/// hold `from_pos`. `match_len == 0` => incoming step was a literal.
#[derive(Clone, Copy)]
struct Arrival {
    cost: u32,
    reps: [u32; 4],
    from_packed: u32,
    match_len: u32,
    match_off: u32,
}

const FROM_SLOT_BITS: u32 = 4; // up to 16 arrivals
const FROM_POS_MASK: u32 = (1u32 << (32 - FROM_SLOT_BITS)) - 1;

impl Arrival {
    #[inline]
    fn empty() -> Self {
        Arrival {
            cost: INF,
            reps: [1, 1, 1, 1],
            from_packed: 0,
            match_len: 0,
            match_off: 0,
        }
    }
    #[inline]
    fn pack(from_pos: u32, from_slot: u8) -> u32 {
        (from_pos & FROM_POS_MASK) | ((from_slot as u32) << (32 - FROM_SLOT_BITS))
    }
    #[inline]
    fn from_pos(&self) -> usize {
        (self.from_packed & FROM_POS_MASK) as usize
    }
    #[inline]
    fn from_slot(&self) -> usize {
        (self.from_packed >> (32 - FROM_SLOT_BITS)) as usize
    }
}

/// Bounded sorted insert into the `ARRIVALS`-wide arrival list at a destination position, deduped
/// on rep state: if an equal-or-cheaper arrival already carries the same rep queue, the candidate
/// is dropped (otherwise every slot collapses onto the same rep set). Keeps the list sorted by
/// ascending cost.
#[inline]
fn relax_into(dest: &mut [Arrival; ARRIVALS], cand: Arrival) {
    // If the candidate can't beat the worst slot of a full list, bail.
    if cand.cost >= dest[ARRIVALS - 1].cost {
        return;
    }
    // Dedupe: drop if an existing arrival with the same reps already costs <= cand.cost. (Sorted
    // by cost, so any same-rep entry before the insert point is cheaper.)
    for a in dest.iter() {
        if a.cost > cand.cost {
            break;
        }
        if a.cost < INF && a.reps == cand.reps {
            return;
        }
    }
    // Find the insertion index (first slot whose cost is > cand.cost).
    let mut idx = ARRIVALS;
    for (k, a) in dest.iter().enumerate() {
        if cand.cost < a.cost {
            idx = k;
            break;
        }
    }
    if idx >= ARRIVALS {
        return;
    }
    // Shift the tail down by one and place the candidate.
    let mut k = ARRIVALS - 1;
    while k > idx {
        dest[k] = dest[k - 1];
        k -= 1;
    }
    dest[idx] = cand;
}

/// Run the optimal parse and return the command list.
///
/// When `use_rep`, each arrival carries its own recent-offset set (rep0-3); a match whose offset
/// is one of that arrival's live reps is charged at token-only cost, so the parse prefers rep
/// matches. The global backtrack picks the cheapest end-to-end rep structure.
///
/// `codec::build_streams` replays the resulting commands with the same rep MTF rules, so the
/// emitted format is independent of how many arrivals the parser explored.
pub fn parse(input: &[u8], ms: &MatchSet, model: &CostModel, use_rep: bool) -> Vec<Command> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }
    // Backtrack pointers pack from_pos into (32 - FROM_SLOT_BITS) bits; positions must fit.
    debug_assert!(
        n as u64 <= FROM_POS_MASK as u64,
        "input too large for packed backtrack"
    );

    // arr[i] = the ARRIVALS best arrivals at position i (sorted ascending by cost).
    let mut arr = vec![[Arrival::empty(); ARRIVALS]; n + 1];
    arr[0][0] = Arrival {
        cost: 0,
        reps: [1, 1, 1, 1],
        from_packed: 0,
        match_len: 0,
        match_off: 0,
    };

    for i in 0..n {
        // Snapshot this position's arrivals (relaxing into i+l never touches arr[i]; the snapshot
        // satisfies the borrow checker and gives a stable list to iterate).
        let cur = arr[i];
        if cur[0].cost >= INF {
            continue; // position unreachable
        }

        // ---- literal transition: every live arrival at i extends to i+1 ----
        {
            let lit_add = model.lit[input[i] as usize];
            let dest = &mut arr[i + 1];
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= INF {
                    break;
                }
                let cand = Arrival {
                    cost: a.cost + lit_add,
                    reps: a.reps, // literals don't change the rep queue
                    from_packed: Arrival::pack(i as u32, slot as u8),
                    match_len: 0,
                    match_off: 0,
                };
                relax_into(dest, cand);
            }
        }

        // ---- explicit matches (Pareto candidates from the match finder) ----
        // Every live arrival drives explicit matches. The cost is rep-independent, but the
        // resulting rep queue (insert vs mtf) depends on whether `off` is already a rep of the
        // source arrival, so expanding from all arrivals makes distinct destination rep queues
        // reachable. The rep-state dedupe in `relax_into` keeps the per-position set small.
        {
            let mut prev_len = MIN_MATCH - 1;
            for c in ms.matches_for(i) {
                let lo = prev_len + 1;
                let hi = c.length;
                if hi < lo {
                    prev_len = hi;
                    continue;
                }
                let off = c.offset;
                let relax_one =
                    |dest: &mut [Arrival; ARRIVALS], a: &Arrival, src_slot: usize, l: u32| {
                        let rep_hit = if use_rep {
                            a.reps.iter().position(|&r| r == off)
                        } else {
                            None
                        };
                        let new_rep = match rep_hit {
                            Some(idx) => rep_mtf(a.reps, idx),
                            None => rep_insert(a.reps, off),
                        };
                        let add = if rep_hit.is_some() {
                            model.match_cost_rep(l)
                        } else {
                            model.match_cost(off, l)
                        };
                        relax_into(
                            dest,
                            Arrival {
                                cost: a.cost + add,
                                reps: new_rep,
                                from_packed: Arrival::pack(i as u32, src_slot as u8),
                                match_len: l,
                                match_off: off,
                            },
                        );
                    };
                if hi >= LEAVE_ALONE {
                    let mid = (LEAVE_ALONE - 1).min(hi);
                    for l in lo..=mid {
                        let dest = &mut arr[i + l as usize];
                        for (slot, a) in cur.iter().enumerate() {
                            if a.cost >= INF {
                                break;
                            }
                            relax_one(dest, a, slot, l);
                        }
                    }
                    if hi > mid {
                        let dest = &mut arr[i + hi as usize];
                        for (slot, a) in cur.iter().enumerate() {
                            if a.cost >= INF {
                                break;
                            }
                            relax_one(dest, a, slot, hi);
                        }
                    }
                } else {
                    for l in lo..=hi {
                        let dest = &mut arr[i + l as usize];
                        for (slot, a) in cur.iter().enumerate() {
                            if a.cost >= INF {
                                break;
                            }
                            relax_one(dest, a, slot, l);
                        }
                    }
                }
                prev_len = hi;
            }
        }

        // ---- rep-offset matches (token-only cost) ----
        // Probe each live rep of each arrival for a short/medium match the hash chain misses
        // (length 2-4). Different arrivals carry different rep sets, so more distinct short reps
        // are reachable than with a single rep queue per position.
        if use_rep {
            let remaining = n - i;
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= INF {
                    break;
                }
                for ridx in 0..4 {
                    let r = a.reps[ridx];
                    let ru = r as usize;
                    if ru == 0 || ru > i {
                        continue;
                    }
                    let src = i - ru;
                    let cap = remaining.min(REP_MAX_LEN);
                    let mut l = 0usize;
                    while l < cap && input[src + l] == input[i + l] {
                        l += 1;
                    }
                    if (l as u32) < MIN_MATCH {
                        continue;
                    }
                    let new_rep = rep_mtf(a.reps, ridx);
                    for ll in MIN_MATCH..=(l as u32) {
                        let dest = &mut arr[i + ll as usize];
                        relax_into(
                            dest,
                            Arrival {
                                cost: a.cost + model.match_cost_rep(ll),
                                reps: new_rep,
                                from_packed: Arrival::pack(i as u32, slot as u8),
                                match_len: ll,
                                match_off: r,
                            },
                        );
                    }
                }
            }
        }
    }

    // ---- Backtrack from the cheapest arrival at n ----
    let mut steps: Vec<(u32, u32)> = Vec::new(); // (offset, len); len==0 => literal
    let mut pos = n;
    let mut slot = 0usize; // best arrival at the end is slot 0 (sorted ascending)
    while pos > 0 {
        let a = arr[pos][slot];
        let fp = a.from_pos();
        let fs = a.from_slot();
        if a.match_len == 0 {
            steps.push((0, 0));
        } else {
            steps.push((a.match_off, a.match_len));
        }
        pos = fp;
        slot = fs;
    }
    steps.reverse();

    // ---- Forward grouping into commands ----
    let mut cmds: Vec<Command> = Vec::new();
    let mut p = 0usize;
    let mut run_start = 0usize;
    let mut run = 0u32;
    for (off, len) in steps {
        if len == 0 {
            if run == 0 {
                run_start = p;
            }
            run += 1;
            p += 1;
        } else {
            cmds.push(Command {
                lit_len: run,
                lit_start: run_start,
                match_off: off,
                match_len: len,
            });
            run = 0;
            p += len as usize;
        }
    }
    if run > 0 {
        cmds.push(Command {
            lit_len: run,
            lit_start: run_start,
            match_off: 0,
            match_len: 0,
        });
    }
    cmds
}

// ===========================================================================
// ZX gamma-cost parser (table-free Elias-gamma format with rep0-3).
//
// A separate optimal parse targeting the `zx` module grammar. It uses the ZX0/salvador cost model
// (interlaced Elias gamma) and the rep0-3 MTF extension, so the multi-arrival DP chooses literal /
// new-offset / rep_i commands to minimise the ZX bitstream size.
//
// Grammar (see zx.rs): a literal run is always followed by a match; a rep match is only legal
// right after a literal run; after a match comes either a literal run or a new-offset match.
// Costs (bits), bit-exact vs the encoder:
//   - new-offset match of (off,len>=2): 1 flag bit + offset_cost_bits(off) + gamma_bits(len-1).
//   - rep_i match (i in 0..rep_slots) of len>=1: 1 flag bit + rep_index_bits(i) [0 when
//     rep_slots==1] + gamma_bits(len).
//   - literal run of L bytes: 1 flag bit (only when it follows a match) + gamma_bits(L) + 8*L.
// The DP carries num_lits (pending run length) and last_is_match per arrival so the connecting
// flag bits and rep legality are charged exactly.
// ===========================================================================

use crate::zx::{
    after_lit_prefix_bits, after_match_prefix_bits, lit_run_bits, near_rep_delta_bits,
    newoff_len_bits, off_msb_bits, offset_cost_bits_hc, rep_index_bits, rep_len_bits, AfterLit,
    AfterMatch, ZxCommand, MAX_OFFSET as ZX_MAX_OFFSET,
};

const ZX_INF: u64 = u64::MAX / 4;
/// Arrivals (beam width) kept per position in the ZX DP.
const ZX_ARRIVALS: usize = 64;
const ZX_REP_PROBE_MAX: usize = 4096;
/// Match-length relaxation span: lengths within this of a candidate's lower bound are all relaxed;
/// beyond it only the full match length is. Bounds the O(matchlen) parse cost on highly repetitive
/// inputs while staying full-quality for typical data.
const ZX_RELAX_SPAN: u32 = 512;
/// Minimum rep-match length. Reps can be length 1 (a single-byte copy from a recent offset).
const ZX_REP_MIN_LEN: u32 = 1;
/// Max |δ| probed for the direct near-rep (rep±δ) match search. Beyond this the near-rep code
/// (prefix + sign + gamma(δ)) is no cheaper than a full new-offset.
const NEAR_REP_DELTA_MAX: i64 = 96;
/// The direct near-rep probe only runs on the top arrivals; deeper arrivals rarely yield a winning
/// near-rep and cost O(δ_max) each.
const NEAR_PROBE_ARRIVALS: usize = 16;
/// Cap on distinct rep-able offsets fed back into the re-parse. Bounds build_extra_candidates at
/// O(n * cap).
const ZX_MAX_SEED_OFFSETS: usize = 768;
/// Max seed-and-reparse rounds. The loop also breaks early once the raw parse stops changing.
const ZX_SEED_ROUNDS: usize = 2;
/// Match-length scan cap when seeding extra rep candidates. The seed only needs to establish the
/// offset (length >= MIN_MATCH); the DP's own rep probe extends the real rep length.
const ZX_SEED_PROBE_MAX: usize = 128;
/// Max extra seed candidates kept per position (the longest dominate). Bounds pass-2 DP work.
const ZX_MAX_EXTRA_PER_POS: usize = 16;
/// Below this literal-byte percentage the input is near-pure RLE: matches already cover almost
/// everything, the base parse captures the reps, and seeding adds no value.
const ZX_SEED_MIN_LIT_PCT: usize = 3;

#[derive(Clone, Copy)]
struct ZxArr {
    cost: u64,
    reps: [u32; 4],
    num_lits: u32,
    last_is_match: bool,
    from_pos: u32,
    from_slot: u16,
    step_kind: u8, // 0 = literal byte, 1 = new-offset match, 2 = rep match, 3 = near-rep match
    step_off: u32,
    step_len: u32,
    step_ri: i8, // near-rep base rep index (kind==3); -1 otherwise
    // Position where this arrival's current rep0 offset was last established (the start of the most
    // recent match on its chain). Carried unchanged across literal bytes; set to the match position
    // on every match. Salvador's `rep_pos`; the anchor `insert_forward_match` seeds offsets onto.
    rep_pos: u32,
    // Secondary objective (salvador `score`): +1 per literal, +3 per new-offset match, +2 per rep
    // match. Breaks exact bit-cost ties, biasing toward fewer / cheaper-to-decode commands.
    score: u32,
}

impl ZxArr {
    #[inline]
    fn empty() -> Self {
        ZxArr {
            cost: ZX_INF,
            reps: [1, 1, 1, 1],
            num_lits: 0,
            last_is_match: false,
            from_pos: 0,
            from_slot: 0,
            step_kind: 0,
            step_off: 0,
            step_len: 0,
            step_ri: -1,
            rep_pos: 0,
            score: 0,
        }
    }
}

/// Lexicographic (cost, score) ordering: cheaper cost wins; on equal cost, lower score wins.
#[inline]
fn zx_better(a_cost: u64, a_score: u32, b_cost: u64, b_score: u32) -> bool {
    a_cost < b_cost || (a_cost == b_cost && a_score < b_score)
}

#[inline]
fn zx_relax(dest: &mut [ZxArr; ZX_ARRIVALS], cand: ZxArr) {
    let worst = &dest[ZX_ARRIVALS - 1];
    if !zx_better(cand.cost, cand.score, worst.cost, worst.score) {
        return;
    }
    // Dedupe on full carried state: an equal-or-better (by cost,score) arrival with the same
    // (num_lits,last_is_match,reps) makes this candidate redundant.
    for a in dest.iter() {
        if zx_better(cand.cost, cand.score, a.cost, a.score) {
            break;
        }
        if a.cost < ZX_INF
            && a.num_lits == cand.num_lits
            && a.last_is_match == cand.last_is_match
            && a.reps == cand.reps
        {
            return;
        }
    }
    let mut idx = ZX_ARRIVALS;
    for (k, a) in dest.iter().enumerate() {
        if zx_better(cand.cost, cand.score, a.cost, a.score) {
            idx = k;
            break;
        }
    }
    if idx >= ZX_ARRIVALS {
        return;
    }
    let mut k = ZX_ARRIVALS - 1;
    while k > idx {
        dest[k] = dest[k - 1];
        k -= 1;
    }
    dest[idx] = cand;
}

#[inline]
fn zx_rep_insert(mut r: [u32; 4], off: u32) -> [u32; 4] {
    r[3] = r[2];
    r[2] = r[1];
    r[1] = r[0];
    r[0] = off;
    r
}
/// rep0-only insert: only reps[0] is consulted, so the tail slots are pinned to their init value.
/// This keeps the dedup key (which compares all 4 reps) collapsing on the rep offset alone, so
/// distinct tails do not fragment the beam.
#[inline]
fn zx_rep_insert1(mut r: [u32; 4], off: u32) -> [u32; 4] {
    r[0] = off;
    r[1] = 1;
    r[2] = 1;
    r[3] = 1;
    r
}
#[inline]
fn zx_rep_mtf(mut r: [u32; 4], idx: usize) -> [u32; 4] {
    let v = r[idx];
    let mut i = idx;
    while i > 0 {
        r[i] = r[i - 1];
        i -= 1;
    }
    r[0] = v;
    r
}

/// Optimal ZX parse. Returns commands consumable by
/// `zx::encode_with2(input, &cmds, rep_slots, near_rep)`.
/// `rep_slots` (1..=4): how many recent offsets are reusable (1 = pure ZX0 rep0-only).
///
/// Two-pass rep seeding: pass 1 parses with the raw match set; the offsets it used are collected
/// and, in pass 2, fed back as extra per-position candidates so the DP can establish a recurring
/// offset early and rep-reuse it cheaply later. This forward-rep seeding surfaces rep-able
/// offsets the Pareto front hides, done as an explicit re-parse.
pub fn parse_zx(input: &[u8], ms: &MatchSet, rep_slots: usize) -> Vec<ZxCommand> {
    parse_zx3(input, ms, rep_slots, false, false)
}

/// As `parse_zx`, plus optional after-literals near-rep (offset-delta) coding (needs rep_slots==4,
/// shares the rep0-3 prefix tree). Returns commands consumable by `zx::encode_with2`.
pub fn parse_zx2(input: &[u8], ms: &MatchSet, rep_slots: usize, near_rep: bool) -> Vec<ZxCommand> {
    parse_zx3(input, ms, rep_slots, near_rep, false)
}

/// As `parse_zx2`, plus optional after-MATCH near-rep coding (`am_near_rep`). Returns commands
/// consumable by `zx::encode_with3`.
///
/// The balanced entry point: multi-arrival DP + forward-rep seeding + seed-and-reparse rounds +
/// rep0 command reduction.
pub fn parse_zx3(
    input: &[u8],
    ms: &MatchSet,
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<ZxCommand> {
    parse_zx3_inner(input, ms, rep_slots, near_rep, am_near_rep, false)
}

/// Fast entry point: a single wide-beam multi-arrival DP pass with no forward-rep seeding, no
/// seed-and-reparse rounds, and no command-reduction refinement. Cheaper than `parse_zx3` (one DP
/// pass instead of a seed pass plus up to 2 reparse rounds, each with a clone+reduce), trading a
/// little ratio for encode speed.
pub fn parse_zx3_fast(
    input: &[u8],
    ms: &MatchSet,
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<ZxCommand> {
    parse_zx3_inner(input, ms, rep_slots, near_rep, am_near_rep, true)
}

/// Shared body for `parse_zx3` (balanced) and `parse_zx3_fast` (fast). When `fast` is set, the
/// refinements (forward-rep seeding, seed-and-reparse rounds, command reduction) are skipped,
/// leaving a single `zx_dp` pass over the raw Pareto match set.
fn parse_zx3_inner(
    input: &[u8],
    ms: &MatchSet,
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
    fast: bool,
) -> Vec<ZxCommand> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }
    debug_assert!((1..=4).contains(&rep_slots));
    // near-rep only makes sense with the full rep0-3 family (it shares that prefix tree).
    let near_rep = near_rep && rep_slots == 4;
    let am_near_rep = am_near_rep && rep_slots == 4;

    // The command-reduction refinement targets the rep0 grammar. It only helps rep0-only (for
    // rep0-3 the encoder already re-derives the best rep index), and is incompatible with near-rep
    // (which needs rep_slots==4), so it runs only for rep_slots==1.
    let do_reduce = !fast && rep_slots == 1 && std::env::var("ZX_NOREDUCE").is_err();
    // Finish a raw parse: return the cheaper (by predicted cost under `rep_slots`/`near_rep`) of the
    // raw parse and its rep0 command-reduction. The refinement always yields a valid command stream
    // (the encoder re-derives the rep index). `ri` (parallel near-rep index array) rides alongside
    // `best` and is consulted only at command-build time; the refinement runs only when near_rep is
    // off (ri all -1).
    let finish = |b: Vec<(u32, u32)>, ri: Vec<i8>| -> (Vec<(u32, u32)>, Vec<i8>, u64) {
        let raw_cost = eval_best_cost(&b, &ri, rep_slots, near_rep, am_near_rep);
        if !do_reduce {
            return (b, ri, raw_cost);
        }
        let mut r = b.clone();
        reduce_commands_zx(input, &mut r);
        let r_ri = vec![-1i8; r.len()];
        let r_cost = eval_best_cost(&r, &r_ri, rep_slots, near_rep, am_near_rep);
        if r_cost <= raw_cost {
            (r, r_ri, r_cost)
        } else {
            (b, ri, raw_cost)
        }
    };

    // Tuning knobs (env-overridable without recompiling).
    let max_extra_per_pos = std::env::var("ZX_EXTRAPP")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(ZX_MAX_EXTRA_PER_POS);
    let seed_probe = std::env::var("ZX_SEEDPROBE")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(ZX_SEED_PROBE_MAX);
    // Forward-rep (rep_pos chain) seeding toggle: salvador's recursive `insert_forward_match`
    // 1 (default) runs the seeding DP; 0 disables it.
    let fwd_seed_on = std::env::var("ZX_FWD")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .map(|v| v > 0)
        .unwrap_or(true);

    // ---- Forward-rep seeding (salvador two-pass) ----
    // Pass A (seeding): a narrow-beam DP that, for every match offset at every position, seeds that
    // offset backward onto the rep-establishment positions of all surviving arrivals (recursively,
    // depth 9) - salvador's `salvador_insert_forward_match`. Pass B (final): the wide-beam `zx_dp`
    // re-parses with the enriched candidate table so it can lock a recurring offset in early and
    // rep-reuse it. Gated to realistic sizes. ZX_OLDSEED=1 selects the realized-parse seeding
    // variant instead.
    let use_old_seed = std::env::var("ZX_OLDSEED").is_ok();
    let fwd_extra: Option<Vec<Vec<(u32, u32)>>> =
        if !fast && !use_old_seed && fwd_seed_on && n >= 512 && n <= 64 * 1024 {
            let fwd_cap = std::env::var("ZX_FWDCAP")
                .ok()
                .and_then(|v| v.parse::<usize>().ok())
                .unwrap_or(max_extra_per_pos);
            Some(zx_seed_forward_reps(
                input, ms, rep_slots, fwd_cap, seed_probe,
            ))
        } else {
            None
        };

    // Pass 1 (final, with forward-rep candidates if seeded): raw match set, refined.
    let (raw1, raw1_ri) = zx_dp(
        input,
        ms,
        fwd_extra.as_deref(),
        rep_slots,
        near_rep,
        am_near_rep,
    );
    let (mut best, mut best_ri, mut best_cost) = finish(raw1.clone(), raw1_ri);
    let mut prev_raw = raw1;

    // Iterate seed-and-reparse: each round collects the offsets the previous parse used and feeds
    // the recurring ones back as extra candidates so the DP can establish a recurring offset early
    // and rep-reuse it cheaply. A round can over-fit and grow, so the cheapest post-refinement
    // result across rounds is kept. Skipped above 64 KB. Stops when the raw parse stops changing.
    let seed_rounds = std::env::var("ZX_ROUNDS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(ZX_SEED_ROUNDS);
    let max_seed_offsets = std::env::var("ZX_SEEDOFF")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(ZX_MAX_SEED_OFFSETS);
    let min_seed_count = std::env::var("ZX_SEEDMINCNT")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
        .unwrap_or(2);
    // Skip seeding on highly repetitive data: if matches already cover almost everything, the base
    // parse + refinement captures the rep structure, and seeding (O(n * |used| * probe)) adds no
    // value. A small literal fraction means repetitive.
    let literal_bytes: usize = {
        let mut p = 0usize;
        let mut lits = 0usize;
        while p < prev_raw.len() {
            let (len, _) = prev_raw[p];
            if len == 0 {
                lits += 1;
                p += 1;
            } else {
                p += len as usize;
            }
        }
        lits
    };
    let too_repetitive = literal_bytes * 100 < n * ZX_SEED_MIN_LIT_PCT;
    // Lower size guard: tiny inputs have too few recurring offsets to benefit from seeding, and the
    // O(n * |used| * probe) seed builders are pure overhead on them. Seeding runs only for >= 512 B.
    if !fast && n >= 512 && n <= 64 * 1024 && seed_rounds > 0 && !too_repetitive {
        let mut prev_used: Vec<u32> = Vec::new();
        for _ in 0..seed_rounds {
            let used = collect_used_offsets(&prev_raw, max_seed_offsets, min_seed_count);
            if used.is_empty() || used == prev_used {
                break;
            }
            let mut extra = build_extra_candidates(input, &used, max_extra_per_pos, seed_probe);
            if use_old_seed {
                // Realized-parse forward-rep seeding variant.
                let fwd = build_forward_rep_candidates(input, &prev_raw, rep_slots, 9, seed_probe);
                extra = merge_extra(extra, &fwd, max_extra_per_pos);
            } else if let Some(fwd) = fwd_extra.as_ref() {
                // Layer the forward-rep candidates (constant across rounds) under the realized-parse
                // used-offset candidates, so the round-based best-of refines on top of the seeding.
                extra = merge_extra(extra, fwd, max_extra_per_pos);
            }
            let (raw, raw_ri) = zx_dp(input, ms, Some(&extra), rep_slots, near_rep, am_near_rep);
            if raw == prev_raw {
                break;
            }
            let (cand, cand_ri, cand_cost) = finish(raw.clone(), raw_ri);
            if cand_cost < best_cost {
                best_cost = cand_cost;
                best = cand;
                best_ri = cand_ri;
            }
            prev_raw = raw;
            prev_used = used;
        }
    }

    // Diagnostic (env ZX_DUMP_REPCOUNT): report rep-match vs new-offset-match counts on the final
    // parse, replaying the encoder's rep0 (rep_slots==1) / rep0-3 MTF. Off by default.
    if std::env::var("ZX_DUMP_REPCOUNT").is_ok() {
        let mut reps = [1u32, 1, 1, 1];
        let mut nrep = 0usize;
        let mut nnew = 0usize;
        let mut after_lits = false;
        let mut p = 0usize;
        while p < best.len() {
            let (len, off) = best[p];
            if len == 0 {
                after_lits = true;
                p += 1;
                continue;
            }
            let rep_idx = if after_lits {
                (0..rep_slots).find(|&r| reps[r] == off)
            } else {
                None
            };
            match rep_idx {
                Some(ridx) => {
                    nrep += 1;
                    reps = zx_rep_mtf(reps, ridx);
                }
                None => {
                    nnew += 1;
                    reps = if rep_slots == 1 {
                        zx_rep_insert1(reps, off)
                    } else {
                        zx_rep_insert(reps, off)
                    };
                }
            }
            after_lits = false;
            p += len as usize;
        }
        eprintln!(
            "ZX_REPCOUNT rep={} new={} total={}",
            nrep,
            nnew,
            nrep + nnew
        );
    }

    build_zx_commands_from_best(input, &best, &best_ri)
}

/// Full-format optimal parse with the complete candidate set (generalized to rep0-3 + near-rep).
/// `complete_extra[i]` holds every distinct non-Pareto offset that matches at i (with its max
/// length) - the larger offsets the Pareto front hides that enable cheap rep-reuse. These are fed
/// via the DP's `extra` channel so the DP can establish a recurring offset early and rep-reuse it
/// downstream. For rep0-only it reproduces ZX0's candidate space; for rep0-3 it gives the DP the
/// same rep-enabling offsets.
///
/// Best-of: the complete-candidate parse, the seeded `parse_zx3`, and (rep0-only) the ZX0-exact
/// port are all evaluated and the cheapest is kept. For rep0-only the ZX0-exact port is optimal.
pub fn parse_zx3_complete(
    input: &[u8],
    ms: &MatchSet,
    complete_extra: &[Vec<(u32, u32)>],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<ZxCommand> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }
    debug_assert!((1..=4).contains(&rep_slots));
    let near_rep = near_rep && rep_slots == 4;
    let am_near_rep = am_near_rep && rep_slots == 4;

    // Candidate A: the multi-arrival DP fed the complete extra candidate table.
    let (raw_c, ri_c) = zx_dp(
        input,
        ms,
        Some(complete_extra),
        rep_slots,
        near_rep,
        am_near_rep,
    );
    let cost_c = eval_best_cost(&raw_c, &ri_c, rep_slots, near_rep, am_near_rep);

    // Candidate B: the seeded parse (its forward-rep seeding can find structure the complete
    // table's per-position cap dropped on low-entropy data).
    let cmds_b = parse_zx3(input, ms, rep_slots, near_rep, am_near_rep);
    let (best_b, ri_b) = commands_to_best(input, &cmds_b, n);
    let cost_b = eval_best_cost(&best_b, &ri_b, rep_slots, near_rep, am_near_rep);

    // For rep0-only, also take the ZX0-exact port (optimal).
    let mut best = raw_c;
    let mut best_ri = ri_c;
    let mut best_cost = cost_c;
    if cost_b < best_cost {
        best = best_b;
        best_ri = ri_b;
        best_cost = cost_b;
    }
    if rep_slots == 1 {
        let cmds_x = crate::zx0opt::optimize_zx0(input, 0, crate::zx::MAX_OFFSET as usize);
        let (best_x, ri_x) = commands_to_best(input, &cmds_x, n);
        let cost_x = eval_best_cost(&best_x, &ri_x, rep_slots, near_rep, am_near_rep);
        if cost_x < best_cost {
            best = best_x;
            best_ri = ri_x;
            best_cost = cost_x;
        }
    }
    let _ = best_cost;
    build_zx_commands_from_best(input, &best, &best_ri)
}

/// Convert a `ZxCommand` list into the per-position (len, off) best-array + near-rep-index array
/// that `eval_best_cost` / `build_zx_commands_from_best` consume. A literal run of L bytes becomes
/// L `(0,0)` entries; a match of (off,len) becomes one `(len, off)` entry at its start position
/// (the next command starts `len` positions later, so interior slots are left `(0,0)`).
fn commands_to_best(_input: &[u8], cmds: &[ZxCommand], n: usize) -> (Vec<(u32, u32)>, Vec<i8>) {
    let mut best = vec![(0u32, 0u32); n];
    let mut ri = vec![-1i8; n];
    let mut p = 0usize;
    for c in cmds {
        for _ in 0..c.lit_len {
            if p < n {
                best[p] = (0, 0);
                ri[p] = -1;
                p += 1;
            }
        }
        if c.match_off != 0 && c.match_len > 0 {
            if p < n {
                best[p] = (c.match_len, c.match_off);
                ri[p] = c.near_rep_ri;
            }
            p += c.match_len as usize;
        }
    }
    (best, ri)
}

/// Predicted total bitstream cost (in bits) of a per-position best_match array, replaying the same
/// rep MTF rules and connecting-flag accounting the encoder uses. `rep_slots` selects rep0-only vs
/// rep0-3 index coding; `near_rep` enables the after-literals near-rep prefix tree. `ri[p] >= 0`
/// marks position `p` as a near-rep match off rep slot `ri[p]`. Bit-exact vs `zx::encode_with2`.
/// Walks commands as [literal-run][match].
fn eval_best_cost(
    best: &[(u32, u32)],
    ri: &[i8],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> u64 {
    let n = best.len();
    let mut reps = [1u32, 1, 1, 1];
    let mut bits: u64 = 0;
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;
    let mut p = 0usize;
    let mut run = 0u32;
    let flush_run = |run: u32, prev: &mut Prev, bits: &mut u64| {
        if run > 0 {
            if let Prev::Match = prev {
                // connecting flag: 0 = literals follow (1 bit in both modes - AfterMatch::Literals)
                *bits += 1;
            }
            *bits += lit_run_bits(run) as u64 + 8 * run as u64;
            *prev = Prev::Literals;
        }
    };
    while p < n {
        let (len, off) = best[p];
        if len == 0 {
            run += 1;
            p += 1;
            continue;
        }
        let near_ri = ri[p];
        // flush pending literal run
        flush_run(run, &mut prev, &mut bits);
        run = 0;
        // match: decide rep vs new-offset exactly as the encoder does
        let after_lits = matches!(prev, Prev::Literals);
        let rep_idx = if after_lits {
            (0..rep_slots).find(|&r| reps[r] == off)
        } else {
            None
        };
        if after_lits && near_rep && near_ri >= 0 {
            // after-LIT near-rep match: prefix(NearRep ri) + sign + gamma(δ) + newoff_len(len-1)
            let rj = near_ri as usize;
            let base = reps[rj];
            let delta = if off > base { off - base } else { base - off };
            bits += after_lit_prefix_bits(AfterLit::NearRep(rj, off)) as u64;
            bits += near_rep_delta_bits(delta) as u64;
            bits += newoff_len_bits(len - 1) as u64;
            reps = zx_rep_insert(reps, off);
        } else if !after_lits && am_near_rep && near_ri >= 0 {
            // after-MATCH near-rep: 11 + ri-bit + sign + gamma(δ) + newoff_len(len-1)
            let rj = near_ri as usize;
            let base = reps[rj];
            let delta = if off > base { off - base } else { base - off };
            bits += after_match_prefix_bits(AfterMatch::NearRep(rj, off)) as u64;
            bits += near_rep_delta_bits(delta) as u64;
            bits += newoff_len_bits(len - 1) as u64;
            reps = zx_rep_insert(reps, off);
        } else {
            match (after_lits, rep_idx) {
                (true, Some(ridx)) => {
                    if near_rep {
                        bits += after_lit_prefix_bits(AfterLit::ExactRep(ridx)) as u64;
                    } else {
                        bits += 1; // after-literals flag: 0 = rep
                        if rep_slots > 1 {
                            bits += rep_index_bits(ridx) as u64;
                        }
                    }
                    bits += rep_len_bits(len) as u64;
                    reps = zx_rep_mtf(reps, ridx);
                }
                _ => {
                    // plain new-offset. After a literal run: 1 bit (or after_lit_prefix in near_rep
                    // mode). After a match: 1 bit classic, or 2 bits (AfterMatch::NewOffset = `10`)
                    // in am_near_rep mode - this is the +1-bit tax on plain after-match new-offsets.
                    if after_lits {
                        if near_rep {
                            bits += after_lit_prefix_bits(AfterLit::NewOffset) as u64;
                        } else {
                            bits += 1;
                        }
                    } else if am_near_rep {
                        bits += after_match_prefix_bits(AfterMatch::NewOffset) as u64;
                    // 2
                    } else {
                        bits += 1;
                    }
                    bits += offset_cost_bits_hc(off) as u64;
                    bits += newoff_len_bits(len - 1) as u64;
                    reps = zx_rep_insert(reps, off);
                }
            }
        }
        prev = Prev::Match;
        p += len as usize;
    }
    flush_run(run, &mut prev, &mut bits);
    bits
}

/// Collect the multiset of offsets used as matches in a parse, returning the distinct offsets that
/// occur at least twice (true rep candidates), capped to the most frequent `ZX_MAX_SEED_OFFSETS`.
fn collect_used_offsets(best: &[(u32, u32)], max_seed_offsets: usize, min_count: u32) -> Vec<u32> {
    use std::collections::HashMap;
    let mut counts: HashMap<u32, u32> = HashMap::new();
    let mut p = 0usize;
    while p < best.len() {
        let (len, off) = best[p];
        if len == 0 {
            p += 1;
        } else {
            if off != 0 {
                *counts.entry(off).or_insert(0) += 1;
            }
            p += len as usize;
        }
    }
    // Offsets the parse used at least `min_count` times are the true rep-able population (an offset
    // used once is rarely worth re-establishing). Most-frequent first, capped.
    let mut v: Vec<(u32, u32)> = counts
        .into_iter()
        .filter(|&(_, c)| c >= min_count)
        .collect();
    v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    v.truncate(max_seed_offsets);
    v.into_iter().map(|(o, _)| o).collect()
}

/// Arrivals (beam width) scanned per position by the forward-rep seeding DP (40 initial arrivals
/// per position). The final parse uses the full ZX_ARRIVALS.
const ZX_SEED_ARRIVALS: usize = 40;
/// Forward-rep insertion recursion depth cap (the recursion runs while depth < 9).
const ZX_FWD_MAX_DEPTH: usize = 9;
/// Supplemental (non-Pareto) offsets gathered per position from the 2-byte hash chain to feed the
/// forward-rep seeding, recurring offsets the smallest-offset Pareto front hides.
const ZX_SUPP_OFFSETS: usize = 64;

/// Recursive forward-rep insertion.
/// When a match with offset `match_off` is available at position `i`, for every arrival reaching
/// `i` with pending literals, `match_off` is inserted as a rep-able candidate at the position where
/// that arrival's current rep offset was established (`rep_pos`), recursing backward up to depth 9.
/// This surfaces a recurring offset at the earlier positions that led to it, so the final parse can
/// establish `match_off` early and rep-reuse it.
///
/// `seed_arr` are the seeding-pass arrivals (row-major, `ZX_SEED_ARRIVALS` per position). `visited`
/// is the per-position de-dup (visited[pos] == match_off => already seeded this pass). `extra` is
/// the mutable per-position candidate table the final parse consumes. `rle_len[p]` is the run
/// length of equal bytes starting at `p`, used to skip the guaranteed-equal prefix. Grows `extra`
/// in place.
#[allow(clippy::too_many_arguments)]
fn zx_insert_forward_match(
    input: &[u8],
    ms: &MatchSet,
    seed_arr: &[ZxArr],
    visited: &mut [u32],
    extra: &mut [Vec<(u32, u32)>],
    rle_len: &[u32],
    i: usize,
    match_off: u32,
    depth: usize,
    probe_cap: usize,
) {
    let n = input.len();
    let base = i * ZX_SEED_ARRIVALS;
    // Iterate the arrivals at i, stopping at the first empty one (cost == ZX_INF).
    for j in 0..ZX_SEED_ARRIVALS {
        let a = &seed_arr[base + j];
        if a.cost >= ZX_INF {
            break;
        }
        if a.num_lits == 0 {
            continue;
        }
        let rep_off = a.reps[0]; // rep0 (the established rep offset, salvador `rep_offset`)
        if match_off == rep_off {
            continue;
        }
        let rep_pos = a.rep_pos as usize;
        // rep_pos must be a real interior position not already visited with this offset.
        if rep_pos == 0 || rep_pos >= n {
            continue;
        }
        if visited[rep_pos] == match_off {
            continue;
        }
        visited[rep_pos] = match_off;
        // The data at rep_pos must admit a match at distance match_off (first byte equal), and
        // rep_off must be live (non-init).
        if rep_pos < match_off as usize || rep_off == 0 {
            continue;
        }
        let src = rep_pos - match_off as usize;
        if input[src] != input[rep_pos] {
            continue;
        }
        // If match_off is already a candidate at rep_pos (in the real match set or the extra table),
        // only its length is extended; a brand-new insert triggers recursion.
        let mut existing_min: u32 = {
            // min(rle_len[src], rle_len[rep_pos]) - the guaranteed-equal prefix to skip.
            let l0 = rle_len[src];
            let l1 = rle_len[rep_pos];
            l0.min(l1)
        };
        let mut already = false;
        for c in ms.matches_for(rep_pos) {
            if c.offset == match_off {
                already = true;
                if c.length > existing_min {
                    existing_min = c.length;
                }
                break;
            }
        }
        if !already {
            for &(o, l) in extra[rep_pos].iter() {
                if o == match_off {
                    already = true;
                    if l > existing_min {
                        existing_min = l;
                    }
                    break;
                }
            }
        }
        // Compute the true run length of match_off at rep_pos, starting from existing_min.
        let max_rep_len = (n - rep_pos).min(probe_cap);
        let mut l = (existing_min as usize).min(max_rep_len);
        while l < max_rep_len && input[src + l] == input[rep_pos + l] {
            l += 1;
        }
        let cur_rep_len = l as u32;
        if !already {
            if cur_rep_len >= MIN_MATCH {
                extra[rep_pos].push((match_off, cur_rep_len));
                // Newly inserted: recurse backward.
                if depth < ZX_FWD_MAX_DEPTH {
                    zx_insert_forward_match(
                        input,
                        ms,
                        seed_arr,
                        visited,
                        extra,
                        rle_len,
                        rep_pos,
                        match_off,
                        depth + 1,
                        probe_cap,
                    );
                }
            }
        } else {
            // Already present: only lengthen it; no recursion.
            for e in extra[rep_pos].iter_mut() {
                if e.0 == match_off {
                    if cur_rep_len > e.1 {
                        e.1 = cur_rep_len;
                    }
                    break;
                }
            }
        }
    }
}

/// Forward-rep seeding pass (with forward-rep insertion enabled): a
/// rep0-aware multi-arrival DP that, at every position and for every match offset, calls
/// `zx_insert_forward_match` to seed rep-able offsets backward onto the positions whose arrivals
/// established the relevant rep. Returns the enriched per-position extra-candidate table the final
/// (wide-beam) `zx_dp` consumes. `rep_slots` selects rep0-only vs rep0-3 rep coding.
///
/// Seeds from all surviving arrivals' rep-establishment positions during the live DP, not just one
/// chosen parse's rep chain (cf. `build_forward_rep_candidates`).
fn zx_seed_forward_reps(
    input: &[u8],
    ms: &MatchSet,
    rep_slots: usize,
    max_extra_per_pos: usize,
    probe_cap: usize,
) -> Vec<Vec<(u32, u32)>> {
    let n = input.len();
    let mut extra: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
    if n == 0 {
        return extra;
    }

    // rle_len[p] = number of equal bytes starting at p (salvador's `rle_len`).
    let mut rle_len = vec![1u32; n];
    {
        let mut i = 0usize;
        while i < n {
            let c = input[i];
            let start = i;
            i += 1;
            while i < n && input[i] == c {
                i += 1;
            }
            let run = (i - start) as u32;
            for (k, slot) in rle_len[start..i].iter_mut().enumerate() {
                *slot = run - k as u32;
            }
        }
    }

    // visited[p] = the last match_off seeded at p in the current top-level insert call (offsets are
    // >= 1, so 0 means "unvisited").
    let mut visited = vec![0u32; n];

    // 2-byte-hash chain over the input. `prev2[p]` = the most recent earlier position sharing p's
    // first 2 bytes, or NONE. This surfaces supplemental offsets the smallest-offset Pareto match
    // front discards, providing the offset diversity the forward-rep seeding needs.
    const NONE2: u32 = u32::MAX;
    let mut prev2 = vec![NONE2; n];
    {
        let mut head = vec![NONE2; 1 << 16];
        let mut p = 0usize;
        while p + 1 < n {
            let key = (input[p] as usize) | ((input[p + 1] as usize) << 8);
            prev2[p] = head[key];
            head[key] = p as u32;
            p += 1;
        }
    }
    // How many supplemental (non-Pareto) offsets to gather per position for seeding - the most
    // recent distinct offsets from the 2-byte chain.
    let supp_max = std::env::var("ZX_SUPP")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(ZX_SUPP_OFFSETS);

    // The seeding DP: same transitions as the final DP but with a narrow (40) beam; the seeding
    // only needs each arrival's rep_offset/rep_pos.
    let bounded = n > 64 * 1024 || zx_is_repetitive(ms, n);
    let rep_probe_max = if bounded { 64 } else { ZX_REP_PROBE_MAX };
    let relax_span = if bounded { 32 } else { ZX_RELAX_SPAN };

    let mut arr = vec![ZxArr::empty(); (n + 1) * ZX_SEED_ARRIVALS];
    arr[0] = ZxArr {
        cost: 0,
        reps: [1, 1, 1, 1],
        num_lits: 0,
        last_is_match: false,
        from_pos: 0,
        from_slot: 0,
        step_kind: 0,
        step_off: 0,
        step_len: 0,
        step_ri: -1,
        rep_pos: 0,
        score: 0,
    };

    // Local relax into a flat row of ZX_SEED_ARRIVALS arrivals at position `pos`.
    #[inline]
    fn seed_relax(row: &mut [ZxArr], cand: ZxArr) {
        let worst = &row[ZX_SEED_ARRIVALS - 1];
        if !zx_better(cand.cost, cand.score, worst.cost, worst.score) {
            return;
        }
        for a in row.iter() {
            if zx_better(cand.cost, cand.score, a.cost, a.score) {
                break;
            }
            if a.cost < ZX_INF
                && a.num_lits == cand.num_lits
                && a.last_is_match == cand.last_is_match
                && a.reps == cand.reps
            {
                return;
            }
        }
        let mut idx = ZX_SEED_ARRIVALS;
        for (k, a) in row.iter().enumerate() {
            if zx_better(cand.cost, cand.score, a.cost, a.score) {
                idx = k;
                break;
            }
        }
        if idx >= ZX_SEED_ARRIVALS {
            return;
        }
        let mut k = ZX_SEED_ARRIVALS - 1;
        while k > idx {
            row[k] = row[k - 1];
            k -= 1;
        }
        row[idx] = cand;
    }

    for i in 0..n {
        // Snapshot of the current position's arrivals (so we can borrow `arr` mutably for the
        // forward positions while reading `cur`).
        let cur: [ZxArr; ZX_SEED_ARRIVALS] = {
            let mut c = [ZxArr::empty(); ZX_SEED_ARRIVALS];
            c.copy_from_slice(&arr[i * ZX_SEED_ARRIVALS..(i + 1) * ZX_SEED_ARRIVALS]);
            c
        };
        if cur[0].cost >= ZX_INF {
            continue;
        }

        // ---- literal transition ----
        {
            let dest_base = (i + 1) * ZX_SEED_ARRIVALS;
            let (_, rest) = arr.split_at_mut(dest_base);
            let dest = &mut rest[..ZX_SEED_ARRIVALS];
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                let old_lits = a.num_lits;
                let new_lits = old_lits + 1;
                let run_delta: i64 = if old_lits == 0 {
                    let flag = if a.last_is_match { 1i64 } else { 0 };
                    flag + lit_run_bits(1) as i64
                } else {
                    lit_run_bits(new_lits) as i64 - lit_run_bits(old_lits) as i64
                };
                let add = (run_delta + 8) as u64;
                seed_relax(
                    dest,
                    ZxArr {
                        cost: a.cost + add,
                        reps: a.reps,
                        num_lits: new_lits,
                        last_is_match: false,
                        from_pos: i as u32,
                        from_slot: slot as u16,
                        step_kind: 0,
                        step_off: 0,
                        step_len: 0,
                        step_ri: -1,
                        rep_pos: a.rep_pos,
                        score: a.score + 1,
                    },
                );
            }
        }

        // ---- supplemental (non-Pareto) offsets: seed each forward ----
        // Gather the most-recent distinct offsets that share i's first 2 bytes but are not on the
        // Pareto front, and seed each one forward - the recurring offsets the smallest-offset front
        // hides. Bounded to `supp_max` offsets per position. Capped to the match set's window, not
        // just the grammar max: a window-restricted format (min-eof's EOF_MAX_OFFSET) cannot encode
        // offsets past its window, and this chain is the one candidate source that bypasses the
        // window-capped match finder.
        let supp_cap = ZX_MAX_OFFSET.min(ms.window());
        if i + 1 < n && supp_max > 0 {
            let mut seeded = 0usize;
            let mut mp = prev2[i];
            while mp != NONE2 && seeded < supp_max {
                let off = i as u32 - mp;
                mp = prev2[mp as usize];
                if off == 0 || off > supp_cap {
                    continue;
                }
                // Skip offsets already on the Pareto front (handled by the explicit loop).
                if ms.matches_for(i).iter().any(|c| c.offset == off) {
                    continue;
                }
                // Must actually match here (first 2 bytes are equal by the hash; require >= MIN).
                let ou = off as usize;
                if ou > i {
                    continue;
                }
                let src = i - ou;
                if input[src] != input[i] || input[src + 1] != input[i + 1] {
                    continue;
                }
                seeded += 1;
                // Add the supplemental offset to position i's own candidate table, so pass B can
                // take it directly at i, not only at the back-seeded rep positions. Length is the
                // true run here (capped).
                let max_l = (n - i).min(probe_cap);
                let mut l = 2usize.min(max_l);
                while l < max_l && input[src + l] == input[i + l] {
                    l += 1;
                }
                if (l as u32) >= MIN_MATCH && !extra[i].iter().any(|&(o, _)| o == off) {
                    extra[i].push((off, l as u32));
                }
                zx_insert_forward_match(
                    input,
                    ms,
                    &arr,
                    &mut visited,
                    &mut extra,
                    &rle_len,
                    i,
                    off,
                    0,
                    probe_cap,
                );
            }
        }

        // ---- explicit (new-offset) matches + forward-rep seeding ----
        {
            let mut prev_len = MIN_MATCH - 1;
            for c in ms.matches_for(i) {
                let lo = prev_len + 1;
                let hi = c.length;
                if hi < lo {
                    prev_len = hi;
                    continue;
                }
                let off = c.offset;
                if off > ZX_MAX_OFFSET {
                    prev_len = hi;
                    continue;
                }

                // Forward-rep seeding: insert this offset backward onto the rep-establishment
                // positions of the arrivals reaching `i`.
                zx_insert_forward_match(
                    input,
                    ms,
                    &arr,
                    &mut visited,
                    &mut extra,
                    &rle_len,
                    i,
                    off,
                    0,
                    probe_cap,
                );

                let off_c = offset_cost_bits_hc(off) as u64;
                let new_reps = if rep_slots == 1 {
                    zx_rep_insert1(cur[0].reps, off)
                } else {
                    zx_rep_insert(cur[0].reps, off)
                };
                let _ = new_reps; // per-arrival reps recomputed below
                for (slot, a) in cur.iter().enumerate() {
                    if a.cost >= ZX_INF {
                        break;
                    }
                    let base = a.cost + 1 + off_c;
                    let nr = if rep_slots == 1 {
                        zx_rep_insert1(a.reps, off)
                    } else {
                        zx_rep_insert(a.reps, off)
                    };
                    let mut l = lo;
                    loop {
                        let dpos = (i + l as usize) * ZX_SEED_ARRIVALS;
                        let (_, rest) = arr.split_at_mut(dpos);
                        let dest = &mut rest[..ZX_SEED_ARRIVALS];
                        let add = newoff_len_bits(l - 1) as u64;
                        seed_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: nr,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 1,
                                step_off: off,
                                step_len: l,
                                step_ri: -1,
                                rep_pos: i as u32,
                                score: a.score + 3,
                            },
                        );
                        if l == hi {
                            break;
                        }
                        l = if l - lo >= relax_span { hi } else { l + 1 };
                    }
                }
                prev_len = hi;
            }
        }

        // ---- rep matches (legal right after a literal run) ----
        {
            let remaining = n - i;
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                if a.num_lits == 0 {
                    continue;
                }
                for ridx in 0..rep_slots {
                    let r = a.reps[ridx];
                    let ru = r as usize;
                    if ru == 0 || ru > i {
                        continue;
                    }
                    let src = i - ru;
                    let cap = remaining.min(rep_probe_max);
                    let mut l = 0usize;
                    while l < cap && input[src + l] == input[i + l] {
                        l += 1;
                    }
                    if (l as u32) < ZX_REP_MIN_LEN {
                        continue;
                    }
                    let prefix = if rep_slots > 1 {
                        1 + rep_index_bits(ridx) as u64
                    } else {
                        1
                    };
                    let nr = zx_rep_mtf(a.reps, ridx);
                    let base = a.cost + prefix;
                    for ll in ZX_REP_MIN_LEN..=(l as u32) {
                        let dpos = (i + ll as usize) * ZX_SEED_ARRIVALS;
                        let (_, rest) = arr.split_at_mut(dpos);
                        let dest = &mut rest[..ZX_SEED_ARRIVALS];
                        let add = rep_len_bits(ll) as u64;
                        seed_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: nr,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 2,
                                step_off: r,
                                step_len: ll,
                                step_ri: -1,
                                rep_pos: i as u32,
                                score: a.score + 2,
                            },
                        );
                    }
                }
            }
        }
    }

    // Cap each position to its longest `max_extra_per_pos` candidates (bounds final-pass DP work).
    for v in extra.iter_mut() {
        if v.len() > max_extra_per_pos {
            v.sort_by(|x, y| y.1.cmp(&x.1));
            v.truncate(max_extra_per_pos);
        }
    }
    extra
}

/// Forward-rep candidate builder driven by the realized parse (cf. `zx_seed_forward_reps`, which
/// seeds from all arrivals during the live DP). Walks the chosen parse forward, replaying rep0 and
/// tracking the chain of positions at which rep0 was (re)established (`rep_chain`). When a
/// new-offset match with offset X starts at `i`, X is seeded at the most recent `fwd_depth`
/// rep-establishment positions whose data admits an X-match (first byte equal). The recorded length
/// is the true X-run length there (capped); the DP's own rep probe extends it. Output is merged
/// into the pass-2 `extra` candidates.
fn build_forward_rep_candidates(
    input: &[u8],
    best: &[(u32, u32)],
    rep_slots: usize,
    fwd_depth: usize,
    probe_cap: usize,
) -> Vec<Vec<(u32, u32)>> {
    let n = input.len();
    let mut extra: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
    if n == 0 {
        return extra;
    }
    // Replay rep0 (rep_slots==1) or rep0-3 (rep_slots==4) MTF exactly as the encoder, walking
    // commands [literal-run][match]. `rep_chain` holds positions where the rolling rep0 offset was
    // established, most-recent last; we only need the last `fwd_depth` entries.
    let mut reps = [1u32, 1, 1, 1];
    let mut rep_chain: Vec<usize> = Vec::with_capacity(fwd_depth + 1);
    let mut p = 0usize;
    // helper: scan the X-run length at position `pos` (cap-bounded).
    let run_len = |pos: usize, off: u32| -> usize {
        let ou = off as usize;
        if ou == 0 || ou > pos {
            return 0;
        }
        let src = pos - ou;
        let cap = (n - pos).min(probe_cap);
        let mut l = 0usize;
        while l < cap && input[src + l] == input[pos + l] {
            l += 1;
        }
        l
    };
    while p < n {
        let (len, off) = best[p];
        if len == 0 {
            p += 1;
            continue;
        }
        // Is this match coded as an exact rep (off already a live rep) or a new offset?
        let is_rep = (0..rep_slots).any(|r| reps[r] == off);
        if !is_rep && off != 0 {
            // New offset established at p. Seed it backward along the rep chain.
            for &rp in rep_chain.iter().rev().take(fwd_depth) {
                if rp == 0 || off as usize > rp {
                    continue;
                }
                let l = run_len(rp, off);
                if (l as u32) >= MIN_MATCH {
                    extra[rp].push((off, l as u32));
                }
            }
        }
        // Update rep state: a match (rep or new) makes `off` the new rep0 and `p` its establish pos.
        if is_rep {
            let idx = (0..rep_slots).find(|&r| reps[r] == off).unwrap();
            reps = zx_rep_mtf(reps, idx);
        } else if rep_slots == 1 {
            reps = zx_rep_insert1(reps, off);
        } else {
            reps = zx_rep_insert(reps, off);
        }
        rep_chain.push(p);
        if rep_chain.len() > fwd_depth + 1 {
            rep_chain.remove(0);
        }
        p += len as usize;
    }
    extra
}

/// Merge per-position candidate lists (used + forward-rep), deduping on offset (longest length wins)
/// and capping each position to `max_extra_per_pos` longest candidates.
fn merge_extra(
    a: Vec<Vec<(u32, u32)>>,
    b: &[Vec<(u32, u32)>],
    max_extra_per_pos: usize,
) -> Vec<Vec<(u32, u32)>> {
    let mut out = a;
    for (i, bl) in b.iter().enumerate() {
        if bl.is_empty() {
            continue;
        }
        let dst = &mut out[i];
        for &(off, len) in bl {
            if let Some(e) = dst.iter_mut().find(|(o, _)| *o == off) {
                if len > e.1 {
                    e.1 = len;
                }
            } else {
                dst.push((off, len));
            }
        }
        if dst.len() > max_extra_per_pos {
            dst.sort_by(|x, y| y.1.cmp(&x.1));
            dst.truncate(max_extra_per_pos);
        }
    }
    out
}

/// For each position, the offsets in `used` that match there with length >= MIN_MATCH (longest wins
/// per offset). Fed to pass 2 as extra new-offset candidates so the DP can lock a recurring offset
/// in early and rep-reuse it. The match-length scan is capped at `seed_probe` (the seed only needs
/// to establish the offset; the DP's own rep probe extends it). O(n * |used| * cap).
fn build_extra_candidates(
    input: &[u8],
    used: &[u32],
    max_extra_per_pos: usize,
    seed_probe: usize,
) -> Vec<Vec<(u32, u32)>> {
    let n = input.len();
    let mut extra: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n];
    let cap = seed_probe;
    for i in 1..n {
        let remaining = n - i;
        let lim = remaining.min(cap);
        if lim < MIN_MATCH as usize {
            continue;
        }
        let bi = input[i];
        for &off in used {
            let ou = off as usize;
            if ou > i {
                continue;
            }
            let src = i - ou;
            if input[src] != bi {
                continue;
            }
            let mut l = 0usize;
            while l < lim && input[src + l] == input[i + l] {
                l += 1;
            }
            if (l as u32) >= MIN_MATCH {
                extra[i].push((off, l as u32));
            }
        }
        // Keep only the longest candidates per position; the cap bounds pass-2 DP work on
        // low-entropy data where many recurring offsets match at the same position.
        if extra[i].len() > max_extra_per_pos {
            extra[i].sort_by(|a, b| b.1.cmp(&a.1));
            extra[i].truncate(max_extra_per_pos);
        }
    }
    extra
}

/// Repetitiveness probe: sample ~256 positions and average the longest match length. Normal data
/// has short matches (tens of bytes); highly repetitive data (RLE / long runs) has matches in the
/// thousands. Used to trigger the hard parse bounds.
fn zx_is_repetitive(ms: &MatchSet, n: usize) -> bool {
    if n == 0 {
        return false;
    }
    let step = (n / 256).max(1);
    let mut samples = 0u64;
    let mut total = 0u64;
    let mut i = 0;
    while i < n {
        let maxlen = ms
            .matches_for(i)
            .iter()
            .map(|c| c.length)
            .max()
            .unwrap_or(0);
        total += maxlen as u64;
        samples += 1;
        i += step;
    }
    samples > 0 && total / samples > 512
}

/// Core multi-arrival DP. Returns the per-position best_match array (pre-refinement) plus a
/// parallel near-rep index array (`ri[p] >= 0` => the match at p was coded near-rep off rep slot
/// `ri[p]`; `-1` otherwise). `extra[i]` (when Some) holds additional new-offset candidates at
/// position i. `near_rep` enables the after-literals near-rep (rep±δ) transitions (needs
/// rep_slots==4).
fn zx_dp(
    input: &[u8],
    ms: &MatchSet,
    extra: Option<&[Vec<(u32, u32)>]>,
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> (Vec<(u32, u32)>, Vec<i8>) {
    let n = input.len();

    // The multi-arrival DP is O(n * ZX_ARRIVALS * rep_probe). Bound the rep probe / length
    // relaxation / near-delta when the input is either above 64 KB or highly repetitive (long runs
    // extend the rep probe to its full cap at every position). On normal data the probe early-exits
    // long before the cap, so the bound is invisible.
    let bounded = n > 64 * 1024 || zx_is_repetitive(ms, n);
    let rep_probe_max = if bounded { 64 } else { ZX_REP_PROBE_MAX };
    let relax_span = if bounded { 32 } else { ZX_RELAX_SPAN };
    let near_delta_max = if bounded { 16 } else { NEAR_REP_DELTA_MAX };

    let mut arr = vec![[ZxArr::empty(); ZX_ARRIVALS]; n + 1];
    arr[0][0] = ZxArr {
        cost: 0,
        reps: [1, 1, 1, 1],
        num_lits: 0,
        last_is_match: false,
        from_pos: 0,
        from_slot: 0,
        step_kind: 0,
        step_off: 0,
        step_len: 0,
        step_ri: -1,
        rep_pos: 0,
        score: 0,
    };

    for i in 0..n {
        let cur = arr[i];
        if cur[0].cost >= ZX_INF {
            continue;
        }

        // ---- literal transition: extend the pending run by one byte ----
        {
            let dest = &mut arr[i + 1];
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                let old_lits = a.num_lits;
                let new_lits = old_lits + 1;
                // Lit-run length cost is interlaced Elias gamma; the run is extended one byte at a
                // time, so charge the gamma length delta + the byte (8 bits) + (on the first byte
                // of a new run) the connecting flag bit after a match. Gamma length is monotonic
                // non-decreasing, so the delta is >= 0.
                let run_delta: i64 = if old_lits == 0 {
                    let flag = if a.last_is_match { 1i64 } else { 0 };
                    flag + lit_run_bits(1) as i64
                } else {
                    lit_run_bits(new_lits) as i64 - lit_run_bits(old_lits) as i64
                };
                let add = (run_delta + 8) as u64;
                zx_relax(
                    dest,
                    ZxArr {
                        cost: a.cost + add,
                        reps: a.reps,
                        num_lits: new_lits,
                        last_is_match: false,
                        from_pos: i as u32,
                        from_slot: slot as u16,
                        step_kind: 0,
                        step_off: 0,
                        step_len: 0,
                        step_ri: -1,
                        // Literal byte: rep0 establishment position is unchanged.
                        rep_pos: a.rep_pos,
                        score: a.score + 1,
                    },
                );
            }
        }

        // ---- explicit (new-offset) matches ----
        {
            let mut prev_len = MIN_MATCH - 1;
            for c in ms.matches_for(i) {
                let lo = prev_len + 1;
                let hi = c.length;
                if hi < lo {
                    prev_len = hi;
                    continue;
                }
                let off = c.offset;
                if off > ZX_MAX_OFFSET {
                    prev_len = hi;
                    continue;
                }
                for (slot, a) in cur.iter().enumerate() {
                    if a.cost >= ZX_INF {
                        break;
                    }
                    // Plain new-offset selector: 1 bit, except in am_near_rep mode after a match
                    // where it is 2 bits (AfterMatch::NewOffset = `10`).
                    let flag = if am_near_rep && a.last_is_match {
                        2u64
                    } else {
                        1u64
                    };
                    let off_c = offset_cost_bits_hc(off) as u64;
                    let base = a.cost + flag + off_c;
                    let new_reps = if rep_slots == 1 {
                        zx_rep_insert1(a.reps, off)
                    } else {
                        zx_rep_insert(a.reps, off)
                    };
                    // Relax every length in [lo, lo+relax_span], then jump to the full length hi.
                    // Intermediate lengths of a very long match are rarely optimal, and relaxing
                    // all of them is O(matchlen) per arrival. For typical data the candidate length
                    // gap is < relax_span so this is a no-op (full quality).
                    let mut l = lo;
                    loop {
                        let dest = &mut arr[i + l as usize];
                        let add = newoff_len_bits(l - 1) as u64;
                        zx_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: new_reps,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 1,
                                step_off: off,
                                step_len: l,
                                step_ri: -1,
                                rep_pos: i as u32,
                                score: a.score + 3,
                            },
                        );
                        if l == hi {
                            break;
                        }
                        l = if l - lo >= relax_span { hi } else { l + 1 };
                    }
                }
                prev_len = hi;
            }

            // ---- extra (rep-seed) new-offset candidates from a prior parse ----
            // Recurring offsets the smallest-offset Pareto front hides. Feeding them as new-offset
            // candidates lets the DP establish the offset early so later positions can rep-reuse it.
            if let Some(ex) = extra {
                for &(off, hi) in &ex[i] {
                    if off > ZX_MAX_OFFSET {
                        continue;
                    }
                    let off_c = offset_cost_bits_hc(off) as u64;
                    for (slot, a) in cur.iter().enumerate() {
                        if a.cost >= ZX_INF {
                            break;
                        }
                        let flag = if am_near_rep && a.last_is_match {
                            2u64
                        } else {
                            1u64
                        };
                        let base = a.cost + flag + off_c;
                        let new_reps = if rep_slots == 1 {
                            zx_rep_insert1(a.reps, off)
                        } else {
                            zx_rep_insert(a.reps, off)
                        };
                        let mut l = MIN_MATCH;
                        loop {
                            let dest = &mut arr[i + l as usize];
                            let add = newoff_len_bits(l - 1) as u64;
                            zx_relax(
                                dest,
                                ZxArr {
                                    cost: base + add,
                                    reps: new_reps,
                                    num_lits: 0,
                                    last_is_match: true,
                                    from_pos: i as u32,
                                    from_slot: slot as u16,
                                    step_kind: 1,
                                    step_off: off,
                                    step_len: l,
                                    step_ri: -1,
                                    rep_pos: i as u32,
                                    score: a.score + 3,
                                },
                            );
                            if l == hi {
                                break;
                            }
                            l = if l - MIN_MATCH >= relax_span {
                                hi
                            } else {
                                l + 1
                            };
                        }
                    }
                }
            }
        }

        // ---- rep matches (only legal right after a literal run) ----
        {
            let remaining = n - i;
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                if a.num_lits == 0 {
                    continue;
                }
                for ridx in 0..rep_slots {
                    let r = a.reps[ridx];
                    let ru = r as usize;
                    if ru == 0 || ru > i {
                        continue;
                    }
                    let src = i - ru;
                    let cap = remaining.min(rep_probe_max);
                    let mut l = 0usize;
                    while l < cap && input[src + l] == input[i + l] {
                        l += 1;
                    }
                    // Reps may be as short as length 1 (a single byte copied from a recent offset),
                    // which the gamma format codes cheaply. New-offset matches still need MIN_MATCH=2.
                    if (l as u32) < ZX_REP_MIN_LEN {
                        continue;
                    }
                    // Rep cost: exact-rep prefix + gamma(len). In near-rep mode the prefix tree is
                    // the extended one (after_lit_prefix_bits); otherwise it is 1 flag bit + the
                    // rep-index code (rep0=1b, rep1=2b, rep2=3b, rep3=3b). The index code is empty
                    // in rep0-only mode.
                    let prefix = if near_rep {
                        after_lit_prefix_bits(AfterLit::ExactRep(ridx)) as u64
                    } else if rep_slots > 1 {
                        1 + rep_index_bits(ridx) as u64
                    } else {
                        1
                    };
                    let new_reps = zx_rep_mtf(a.reps, ridx);
                    let base = a.cost + prefix;
                    for ll in ZX_REP_MIN_LEN..=(l as u32) {
                        let dest = &mut arr[i + ll as usize];
                        let add = rep_len_bits(ll) as u64; // rep length stays gamma
                        zx_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: new_reps,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 2,
                                step_off: r,
                                step_len: ll,
                                step_ri: -1,
                                rep_pos: i as u32,
                                score: a.score + 2,
                            },
                        );
                    }
                }
            }
        }

        // ---- near-rep matches (after-literals only; off = rep[ri] ± δ, ri in {0,1}) ----
        // A new-offset match whose offset is within ±δ of rep0 or rep1 is coded as near_rep(ri)
        // prefix + sign + gamma(δ) + newoff_len(len-1), reusing a recent offset's magnitude without
        // a full offset code. Probes the explicit-match candidates and tests whether a near-rep
        // coding off rep0/rep1 is representable and cheaper.
        if near_rep {
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                if a.num_lits == 0 {
                    continue; // near-rep, like exact rep, is only legal right after literals
                }
                let mut prev_len = MIN_MATCH - 1;
                for c in ms.matches_for(i) {
                    let lo = prev_len + 1;
                    let hi = c.length;
                    if hi < lo {
                        prev_len = hi;
                        continue;
                    }
                    let off = c.offset;
                    prev_len = hi;
                    if off > ZX_MAX_OFFSET {
                        continue;
                    }
                    // Find the cheapest near-rep base (rep0 or rep1) for this offset.
                    let mut best_prefix = u64::MAX;
                    let mut best_ri = -1i8;
                    for ri in 0..2usize {
                        let base_off = a.reps[ri];
                        if base_off == off {
                            continue; // δ==0 is an exact rep, handled by the exact-rep path
                        }
                        let delta = if off > base_off {
                            off - base_off
                        } else {
                            base_off - off
                        };
                        let pfx = after_lit_prefix_bits(AfterLit::NearRep(ri, off)) as u64
                            + near_rep_delta_bits(delta) as u64;
                        if pfx < best_prefix {
                            best_prefix = pfx;
                            best_ri = ri as i8;
                        }
                    }
                    if best_ri < 0 {
                        continue;
                    }
                    // Only worth exploring if cheaper than a full new-offset code.
                    let full = after_lit_prefix_bits(AfterLit::NewOffset) as u64
                        + offset_cost_bits_hc(off) as u64;
                    if best_prefix >= full {
                        continue;
                    }
                    let new_reps = zx_rep_insert(a.reps, off);
                    let base = a.cost + best_prefix;
                    let mut l = lo;
                    loop {
                        let dest = &mut arr[i + l as usize];
                        let add = newoff_len_bits(l - 1) as u64; // new-offset length coding (len-1)
                        zx_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: new_reps,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 3,
                                step_off: off,
                                step_len: l,
                                step_ri: best_ri,
                                rep_pos: i as u32,
                                score: a.score + 3,
                            },
                        );
                        if l == hi {
                            break;
                        }
                        l = if l - lo >= relax_span { hi } else { l + 1 };
                    }
                }
            }

            // Direct rep±δ probe: try offsets the Pareto match-front never lists (a smaller offset
            // achieves the same length, so the front records that instead). A near-rep at rep0±δ /
            // rep1±δ can still be the cheapest coding of a real match here, so probe those offsets
            // directly and synthesize the near-rep candidate.
            let remaining = n - i;
            let next0 = input[i]; // first-byte filter for the probe
            for (slot, a) in cur.iter().enumerate().take(NEAR_PROBE_ARRIVALS) {
                if a.cost >= ZX_INF {
                    break;
                }
                if a.num_lits == 0 {
                    continue;
                }
                for ri in 0..2usize {
                    let base_off = a.reps[ri];
                    for &signed in &[1i64, -1i64] {
                        let mut delta: i64 = 1;
                        while delta <= near_delta_max {
                            let off_i = base_off as i64 + signed * delta;
                            delta += 1;
                            if off_i < 1 || off_i > ZX_MAX_OFFSET as i64 {
                                continue;
                            }
                            let off = off_i as u32;
                            let ou = off as usize;
                            if ou > i {
                                continue;
                            }
                            let src = i - ou;
                            // First-byte precheck: skip the prefix-cost math and full scan unless
                            // the match even starts here (filters ~all non-matching δ cheaply).
                            if input[src] != next0 {
                                continue;
                            }
                            let d = (off as i64 - base_off as i64).unsigned_abs() as u32;
                            let prefix = after_lit_prefix_bits(AfterLit::NearRep(ri, off)) as u64
                                + near_rep_delta_bits(d) as u64;
                            if prefix
                                >= after_lit_prefix_bits(AfterLit::NewOffset) as u64
                                    + offset_cost_bits_hc(off) as u64
                            {
                                continue; // a full new-offset would be at least as cheap
                            }
                            let cap = remaining.min(rep_probe_max);
                            let mut l = 0usize;
                            while l < cap && input[src + l] == input[i + l] {
                                l += 1;
                            }
                            if (l as u32) < MIN_MATCH {
                                continue;
                            }
                            let new_reps = zx_rep_insert(a.reps, off);
                            let base = a.cost + prefix;
                            let hi = l as u32;
                            let mut ll = MIN_MATCH;
                            loop {
                                let dest = &mut arr[i + ll as usize];
                                let add = newoff_len_bits(ll - 1) as u64;
                                zx_relax(
                                    dest,
                                    ZxArr {
                                        cost: base + add,
                                        reps: new_reps,
                                        num_lits: 0,
                                        last_is_match: true,
                                        from_pos: i as u32,
                                        from_slot: slot as u16,
                                        step_kind: 3,
                                        step_off: off,
                                        step_len: ll,
                                        step_ri: ri as i8,
                                        rep_pos: i as u32,
                                        score: a.score + 3,
                                    },
                                );
                                if ll == hi {
                                    break;
                                }
                                ll = if ll - MIN_MATCH >= relax_span {
                                    hi
                                } else {
                                    ll + 1
                                };
                            }
                        }
                    }
                }
            }
        }

        // ---- after-MATCH near-rep matches (off = rep0/rep1 ± δ, after a match) ----
        // Mirrors the after-lit near-rep but for the after-match state. The selector is
        // AfterMatch::NearRep (`11`+ri-bit = 3 bits); a plain after-match new-offset costs
        // AfterMatch::NewOffset (`10` = 2 bits) + the full offset code. Probes both the explicit
        // match-front offsets and a direct rep±δ scan (offsets the Pareto front hides).
        if am_near_rep {
            // (1) explicit match-front offsets coded as after-match near-rep
            for (slot, a) in cur.iter().enumerate() {
                if a.cost >= ZX_INF {
                    break;
                }
                if !(a.num_lits == 0 && a.last_is_match) {
                    continue; // after-match state only
                }
                let mut prev_len = MIN_MATCH - 1;
                for c in ms.matches_for(i) {
                    let lo = prev_len + 1;
                    let hi = c.length;
                    if hi < lo {
                        prev_len = hi;
                        continue;
                    }
                    let off = c.offset;
                    prev_len = hi;
                    if off > ZX_MAX_OFFSET {
                        continue;
                    }
                    let mut best_prefix = u64::MAX;
                    let mut best_ri = -1i8;
                    for ri in 0..2usize {
                        let base_off = a.reps[ri];
                        if base_off == off {
                            continue;
                        }
                        let delta = if off > base_off {
                            off - base_off
                        } else {
                            base_off - off
                        };
                        let pfx = after_match_prefix_bits(AfterMatch::NearRep(ri, off)) as u64
                            + near_rep_delta_bits(delta) as u64;
                        if pfx < best_prefix {
                            best_prefix = pfx;
                            best_ri = ri as i8;
                        }
                    }
                    if best_ri < 0 {
                        continue;
                    }
                    // Worth exploring only if cheaper than a plain after-match new-offset (`10`+off).
                    let full = after_match_prefix_bits(AfterMatch::NewOffset) as u64
                        + offset_cost_bits_hc(off) as u64;
                    if best_prefix >= full {
                        continue;
                    }
                    let new_reps = zx_rep_insert(a.reps, off);
                    let base = a.cost + best_prefix;
                    let mut l = lo;
                    loop {
                        let dest = &mut arr[i + l as usize];
                        let add = newoff_len_bits(l - 1) as u64;
                        zx_relax(
                            dest,
                            ZxArr {
                                cost: base + add,
                                reps: new_reps,
                                num_lits: 0,
                                last_is_match: true,
                                from_pos: i as u32,
                                from_slot: slot as u16,
                                step_kind: 3,
                                step_off: off,
                                step_len: l,
                                step_ri: best_ri,
                                rep_pos: i as u32,
                                score: a.score + 3,
                            },
                        );
                        if l == hi {
                            break;
                        }
                        l = if l - lo >= relax_span { hi } else { l + 1 };
                    }
                }
            }

            // (2) direct rep±δ probe for offsets the Pareto front never lists.
            let remaining = n - i;
            let next0 = input[i];
            for (slot, a) in cur.iter().enumerate().take(NEAR_PROBE_ARRIVALS) {
                if a.cost >= ZX_INF {
                    break;
                }
                if !(a.num_lits == 0 && a.last_is_match) {
                    continue;
                }
                for ri in 0..2usize {
                    let base_off = a.reps[ri];
                    for &signed in &[1i64, -1i64] {
                        let mut delta: i64 = 1;
                        while delta <= near_delta_max {
                            let off_i = base_off as i64 + signed * delta;
                            delta += 1;
                            if off_i < 1 || off_i > ZX_MAX_OFFSET as i64 {
                                continue;
                            }
                            let off = off_i as u32;
                            let ou = off as usize;
                            if ou > i {
                                continue;
                            }
                            let src = i - ou;
                            if input[src] != next0 {
                                continue;
                            }
                            let d = (off as i64 - base_off as i64).unsigned_abs() as u32;
                            let prefix = after_match_prefix_bits(AfterMatch::NearRep(ri, off))
                                as u64
                                + near_rep_delta_bits(d) as u64;
                            if prefix
                                >= after_match_prefix_bits(AfterMatch::NewOffset) as u64
                                    + offset_cost_bits_hc(off) as u64
                            {
                                continue;
                            }
                            let cap = remaining.min(rep_probe_max);
                            let mut l = 0usize;
                            while l < cap && input[src + l] == input[i + l] {
                                l += 1;
                            }
                            if (l as u32) < MIN_MATCH {
                                continue;
                            }
                            let new_reps = zx_rep_insert(a.reps, off);
                            let base = a.cost + prefix;
                            let hi = l as u32;
                            let mut ll = MIN_MATCH;
                            loop {
                                let dest = &mut arr[i + ll as usize];
                                let add = newoff_len_bits(ll - 1) as u64;
                                zx_relax(
                                    dest,
                                    ZxArr {
                                        cost: base + add,
                                        reps: new_reps,
                                        num_lits: 0,
                                        last_is_match: true,
                                        from_pos: i as u32,
                                        from_slot: slot as u16,
                                        step_kind: 3,
                                        step_off: off,
                                        step_len: ll,
                                        step_ri: ri as i8,
                                        rep_pos: i as u32,
                                        score: a.score + 3,
                                    },
                                );
                                if ll == hi {
                                    break;
                                }
                                ll = if ll - MIN_MATCH >= relax_span {
                                    hi
                                } else {
                                    ll + 1
                                };
                            }
                        }
                    }
                }
            }
        }
    }

    // ---- backtrack ----
    // (kind, off, len, near_rep_ri): near_rep_ri >= 0 only for step_kind==3 (near-rep).
    let mut steps: Vec<(u8, u32, u32, i8)> = Vec::new();
    let mut pos = n;
    let mut slot;
    {
        let mut best = 0usize;
        let mut bc = ZX_INF;
        let mut bs = u32::MAX;
        for (k, a) in arr[n].iter().enumerate() {
            if zx_better(a.cost, a.score, bc, bs) {
                bc = a.cost;
                bs = a.score;
                best = k;
            }
        }
        slot = best;
    }
    while pos > 0 {
        let a = arr[pos][slot];
        let ri = if a.step_kind == 3 { a.step_ri } else { -1 };
        steps.push((a.step_kind, a.step_off, a.step_len, ri));
        let fp = a.from_pos as usize;
        let fs = a.from_slot as usize;
        pos = fp;
        slot = fs;
    }
    steps.reverse();

    // ---- lay steps out into the per-position best_match array ----
    // best[p] = (len, off): len==0 literal at p; len>=1 a match of that length/offset starting at
    // p (interior positions stay len==0 and are never inspected by the command builder/writer).
    // best_ri[p] >= 0 marks a near-rep match (off = rep[best_ri[p]] ± δ).
    let mut best: Vec<(u32, u32)> = vec![(0u32, 0u32); n];
    let mut best_ri: Vec<i8> = vec![-1i8; n];
    {
        let mut p = 0usize;
        for (kind, off, len, ri) in steps {
            if kind == 0 {
                // literal byte
                p += 1;
            } else {
                best[p] = (len, off);
                best_ri[p] = ri;
                p += len as usize;
            }
        }
    }
    (best, best_ri)
}

/// Build the ZxCommand list from the per-position best_match array.
/// best[p] = (len, off): len==0 => literal; len>=1 => match. Interior bytes of a match have len==0
/// but are skipped because the previous match advanced `p` past them. `best_ri[p] >= 0` marks a
/// near-rep match (carried through to `ZxCommand::near_rep_ri` so the encoder replays it exactly).
fn build_zx_commands_from_best(
    input: &[u8],
    best: &[(u32, u32)],
    best_ri: &[i8],
) -> Vec<ZxCommand> {
    let n = input.len();
    let mut cmds: Vec<ZxCommand> = Vec::new();
    let mut p = 0usize;
    let mut run_start = 0usize;
    let mut run = 0u32;
    while p < n {
        let (len, off) = best[p];
        if len == 0 {
            if run == 0 {
                run_start = p;
            }
            run += 1;
            p += 1;
        } else {
            cmds.push(ZxCommand {
                lit_len: run,
                lit_start: run_start,
                match_off: off,
                match_len: len,
                near_rep_ri: best_ri[p],
            });
            run = 0;
            p += len as usize;
        }
    }
    if run > 0 {
        cmds.push(ZxCommand {
            lit_len: run,
            lit_start: run_start,
            match_off: 0,
            match_len: 0,
            near_rep_ri: -1,
        });
    }
    cmds
}

// ---------------------------------------------------------------------------
// Cost helpers for the refinement (bit-exact vs zx.rs encoder; rep0 grammar).
//
// These use the same codes as the encoder / `eval_best_cost`: all length fields and the offset MSB
// are interlaced Elias gamma; the 7-bit offset LSB is raw. (`lit_run_bits` / `newoff_len_bits` /
// `off_msb_bits` / `rep_len_bits` all resolve to `gamma_bits` in zx.rs.)
// ---------------------------------------------------------------------------

/// Literal-run length cost INCLUDING the leading flag/token bit, EXCLUDING the L data bytes.
/// salvador_get_literals_varlen_size analogue: lit_run_bits(L)+1 for L>=1, and 0 for L==0.
#[inline]
fn lit_varlen_size(len: u32) -> i64 {
    if len == 0 {
        0
    } else {
        (lit_run_bits(len) + 1) as i64
    }
}
/// New-offset match length field cost (no flag/offset). salvador_get_match_varlen_size_norep
/// analogue: gamma(len-1).
#[inline]
fn match_varlen_norep(len: u32) -> i64 {
    newoff_len_bits(len - 1) as i64
}
/// Rep match length field cost (no flag). salvador_get_match_varlen_size_rep analogue (gamma).
#[inline]
fn match_varlen_rep(len: u32) -> i64 {
    rep_len_bits(len) as i64
}
/// High-bits-of-offset size, in bits (the 7-bit low byte is added separately as `+7`): gamma(MSB).
#[inline]
fn off_hi_size(off: u32) -> i64 {
    off_msb_bits(((off - 1) >> 7) + 1) as i64
}
/// Full new-offset cost (gamma(off-MSB) + 7-bit low byte). Mirrors zx::offset_cost_bits_hc.
#[inline]
fn offset_cost(off: u32) -> i64 {
    off_hi_size(off) + 7
}

/// Command reduction for the rep0 grammar, iterated to a fixed
/// point (max 30 passes). Operates in-place on the per-position best array.
///
/// best[p] = (len, off): len==0 literal; len==1 one-byte rep (offset==current rep); len>=2 match.
/// num_literals = pending literal run length; rep_off = current rolling last-offset (init 1).
fn reduce_commands_zx(input: &[u8], best: &mut [(u32, u32)]) {
    let n = best.len();
    let end = n as i64;
    const MIN_ENC: u32 = 2; // MIN_ENCODED_MATCH_SIZE
    const MAX_VARLEN: u32 = 0xffff;

    // memcmp(a, b, len) over the input window, with signed offsets (an intermediate `i - offset`
    // may be negative before `+ length` makes it valid). Returns false if either index is out of
    // range.
    let eq = |a: i64, b: i64, len: usize| -> bool {
        if a < 0 || b < 0 {
            return false;
        }
        let (a, b) = (a as usize, b as usize);
        if a + len > input.len() || b + len > input.len() {
            return false;
        }
        input[a..a + len] == input[b..b + len]
    };

    let mut passes = 0;
    loop {
        let mut did_reduce = false;
        let mut num_literals: u32 = 0;
        let mut rep_off: u32 = 1; // initial rep offset is 1 (matches encoder/decoder)
        let mut i: i64 = 0;
        // The first command is always literals, so position 0 is a forced literal.
        if end > 0 {
            num_literals = 1;
            i = 1;
        }
        while i < end {
            let iu = i as usize;
            let (ml, mo) = best[iu];

            // ---- (R1) merge a leading literal into the following match ----
            if ml == 0
                && (i + 1) < end
                && best[(i + 1) as usize].0 >= MIN_ENC
                && best[(i + 1) as usize].0 < MAX_VARLEN
                && best[(i + 1) as usize].1 != 0
                && i >= best[(i + 1) as usize].1 as i64
                && (i + 1 + best[(i + 1) as usize].0 as i64) <= end
                && (num_literals != 0 || best[(i + 1) as usize].1 != rep_off)
                && eq(
                    i - best[(i + 1) as usize].1 as i64,
                    i,
                    best[(i + 1) as usize].0 as usize + 1,
                )
            {
                let nxt_len = best[(i + 1) as usize].0;
                let nxt_off = best[(i + 1) as usize].1;
                let cur_len_size: i64;
                if nxt_off == rep_off {
                    cur_len_size =
                        lit_varlen_size(num_literals + 1) + 8 + match_varlen_rep(nxt_len);
                } else {
                    cur_len_size = lit_varlen_size(num_literals + 1)
                        + 8
                        + off_hi_size(nxt_off)
                        + 7
                        + match_varlen_norep(nxt_len);
                }
                let reduced_len_size: i64;
                if num_literals != 0 && nxt_off == rep_off && rep_off != 0 {
                    reduced_len_size =
                        lit_varlen_size(num_literals) + match_varlen_rep(nxt_len + 1);
                } else {
                    reduced_len_size = lit_varlen_size(num_literals)
                        + off_hi_size(nxt_off)
                        + 7
                        + match_varlen_norep(nxt_len + 1);
                }
                if reduced_len_size <= cur_len_size {
                    best[iu] = (nxt_len + 1, nxt_off);
                    best[(i + 1) as usize] = (0, 0);
                    did_reduce = true;
                    continue;
                }
            }

            if ml >= MIN_ENC {
                // Examine the gap after this match: some literals, then another match.
                if (i + ml as i64) < end {
                    let mut next_index = i + ml as i64;
                    let mut next_literals: u32 = 0;
                    while next_index < end && best[next_index as usize].0 == 0 {
                        next_literals += 1;
                        next_index += 1;
                    }
                    if next_index < end {
                        let ni = next_index as usize;
                        let (nx_len, nx_off) = best[ni];
                        if nx_len >= MIN_ENC {
                            // ---- (R2) recover a missed BACKWARD rep match ----
                            if num_literals != 0
                                && rep_off != 0
                                && mo != rep_off
                                && (nx_off != mo || offset_cost(mo) > offset_cost(nx_off))
                            {
                                if i >= rep_off as i64 && (i - rep_off as i64 + ml as i64) <= end {
                                    let mut max_len: u32 = 0;
                                    while (max_len as i64) < ml as i64
                                        && input[(i - rep_off as i64 + max_len as i64) as usize]
                                            == input[(iu - mo as usize) + max_len as usize]
                                    {
                                        max_len += 1;
                                    }
                                    if max_len >= 1 {
                                        let cur_cmd = off_hi_size(mo)
                                            + 7
                                            + match_varlen_norep(ml)
                                            + lit_varlen_size(next_literals);
                                        let reduced_cmd = match_varlen_rep(max_len)
                                            + ((ml - max_len) as i64) * 8
                                            + lit_varlen_size(next_literals + (ml - max_len));
                                        if reduced_cmd < cur_cmd {
                                            best[iu] = (max_len, rep_off);
                                            for j in max_len..ml {
                                                best[iu + j as usize] = (0, 0);
                                            }
                                            did_reduce = true;
                                        }
                                    }
                                }
                            }

                            // ---- (R3) steal the next match's offset for a forward rep ----
                            let (ml2, mo2) = best[iu]; // R2 may have changed it
                            if nx_off != 0
                                && mo2 != nx_off
                                && rep_off != nx_off
                                && next_literals != 0
                            {
                                if i >= nx_off as i64
                                    && (i - nx_off as i64 + ml2 as i64) <= end
                                    && mo2 != rep_off
                                {
                                    let mut max_len: u32 = 0;
                                    while (max_len as i64) < ml2 as i64
                                        && input[(i - nx_off as i64 + max_len as i64) as usize]
                                            == input[(iu - mo2 as usize) + max_len as usize]
                                    {
                                        max_len += 1;
                                    }
                                    if max_len >= ml2 {
                                        best[iu] = (ml2, nx_off);
                                        did_reduce = true;
                                    } else if max_len >= 2 {
                                        let before = match_varlen_norep(ml2)
                                            + offset_cost(mo2)
                                            + lit_varlen_size(next_literals);
                                        let after = match_varlen_rep(max_len)
                                            + lit_varlen_size(next_literals + (ml2 - max_len))
                                            + ((ml2 - max_len) as i64) * 8;
                                        if after < before {
                                            best[iu] = (max_len, nx_off);
                                            for j in max_len..ml2 {
                                                best[iu + j as usize] = (0, 0);
                                            }
                                            did_reduce = true;
                                        }
                                    }
                                }
                            }

                            // ---- (R4) replace a short match (<9) by literals ----
                            let (ml3, mo3) = best[iu];
                            if ml3 < 9 && ml3 >= MIN_ENC {
                                let mut cur_cmd = lit_varlen_size(num_literals);
                                if mo3 == rep_off && num_literals != 0 && rep_off != 0 {
                                    cur_cmd += match_varlen_rep(ml3);
                                } else {
                                    cur_cmd += off_hi_size(mo3) + 7 + match_varlen_norep(ml3);
                                }
                                let mut next_cmd = lit_varlen_size(next_literals) + 1;
                                if mo3 != 0 && nx_off == mo3 && next_literals != 0 {
                                    next_cmd += match_varlen_rep(nx_len);
                                } else {
                                    next_cmd +=
                                        off_hi_size(nx_off) + 7 + match_varlen_norep(nx_len);
                                }
                                let original = cur_cmd + next_cmd;
                                let mut reduced = (ml3 as i64) * 8
                                    + lit_varlen_size(num_literals + ml3 + next_literals);
                                if nx_off == rep_off
                                    && (num_literals + ml3 + next_literals) != 0
                                    && rep_off != 0
                                {
                                    reduced += match_varlen_rep(nx_len);
                                } else {
                                    reduced += off_hi_size(nx_off) + 7 + match_varlen_norep(nx_len);
                                }
                                if original >= reduced {
                                    for j in 0..ml3 {
                                        best[iu + j as usize] = (0, 0);
                                    }
                                    did_reduce = true;
                                    continue;
                                }
                            }
                        }
                    }
                }

                // ---- (R5) join two adjacent matches into one ----
                let (ml4, mo4) = best[iu];
                if (i + ml4 as i64) < end && mo4 != 0 && ml4 >= MIN_ENC {
                    let j2 = (i + ml4 as i64) as usize;
                    let (j2_len, j2_off) = best[j2];
                    if j2_off != 0
                        && j2_len >= MIN_ENC
                        && (ml4 + j2_len) <= MAX_VARLEN
                        && (i + ml4 as i64) >= mo4 as i64
                        && (i + ml4 as i64) >= j2_off as i64
                        && (i + ml4 as i64 + j2_len as i64) <= end
                        && eq(
                            i - mo4 as i64 + ml4 as i64,
                            i + ml4 as i64 - j2_off as i64,
                            j2_len as usize,
                        )
                    {
                        let mut next_index = i + ml4 as i64 + j2_len as i64;
                        let mut next_literals: u32 = 0;
                        while next_index < end && best[next_index as usize].0 == 0 {
                            next_index += 1;
                            next_literals += 1;
                        }
                        // current partial size: [this match][next match][following command]
                        let mut cur_partial: i64;
                        if mo4 == rep_off && num_literals != 0 {
                            cur_partial = match_varlen_rep(ml4);
                        } else {
                            cur_partial = off_hi_size(mo4) + 7 + match_varlen_norep(ml4);
                        }
                        cur_partial += 1; // match-with-offset follows
                        cur_partial += off_hi_size(j2_off) + 7 + match_varlen_norep(j2_len);
                        if next_index < end {
                            let nni = next_index as usize;
                            let (nn_len, nn_off) = best[nni];
                            if j2_off != 0 && nn_off == j2_off && next_literals != 0 {
                                cur_partial += match_varlen_rep(nn_len);
                            } else {
                                cur_partial += off_hi_size(nn_off) + 7 + match_varlen_norep(nn_len);
                            }
                        }
                        let mut reduced_partial: i64;
                        if mo4 == rep_off && num_literals != 0 && rep_off != 0 {
                            reduced_partial = match_varlen_rep(ml4 + j2_len);
                        } else {
                            reduced_partial =
                                off_hi_size(mo4) + 7 + match_varlen_norep(ml4 + j2_len);
                        }
                        let mut cannot_reduce = false;
                        if next_index < end {
                            let nni = next_index as usize;
                            let (nn_len, nn_off) = best[nni];
                            if mo4 != 0 && nn_off == mo4 && next_literals != 0 {
                                reduced_partial += match_varlen_rep(nn_len);
                            } else if nn_len >= MIN_ENC {
                                reduced_partial +=
                                    off_hi_size(nn_off) + 7 + match_varlen_norep(nn_len);
                            } else {
                                cannot_reduce = true;
                            }
                        }
                        if cur_partial >= reduced_partial && !cannot_reduce {
                            best[iu] = (ml4 + j2_len, mo4);
                            best[j2] = (0, 0);
                            did_reduce = true;
                            continue;
                        }
                    }
                }

                // ---- (R6) dissolve [len-2 match][lits][1-byte rep][lits][match] into literals ----
                let (ml5, mo5) = best[iu];
                if num_literals != 0 && mo5 != rep_off && ml5 == MIN_ENC && rep_off != 0 {
                    if (i + MIN_ENC as i64) < end {
                        let mut next_index = i + MIN_ENC as i64;
                        let mut next_literals: u32 = 0;
                        while next_index < end && best[next_index as usize].0 == 0 {
                            next_literals += 1;
                            next_index += 1;
                        }
                        if next_index < end
                            && next_literals != 0
                            && best[next_index as usize].0 == 1
                            && best[next_index as usize].1 == mo5
                        {
                            let mut nn_index = next_index + 1;
                            let mut nn_literals: u32 = 0;
                            while nn_index < end && best[nn_index as usize].0 == 0 {
                                nn_literals += 1;
                                nn_index += 1;
                            }
                            if nn_index < end
                                && nn_literals != 0
                                && best[nn_index as usize].0 >= MIN_ENC
                                && best[nn_index as usize].1 != best[next_index as usize].1
                            {
                                let cur_cmd = lit_varlen_size(num_literals)
                                    + 1
                                    + off_hi_size(mo5)
                                    + 7
                                    + match_varlen_norep(MIN_ENC); // norep len 2 gamma
                                let cur_rep = lit_varlen_size(next_literals)
                                    + (next_literals as i64) * 8
                                    + 1
                                    + match_varlen_rep(1); // rep len 1 == gamma_bits(1) == 1
                                let reduced =
                                    lit_varlen_size(num_literals + MIN_ENC + next_literals + 1)
                                        + (MIN_ENC as i64) * 8
                                        + (next_literals as i64) * 8
                                        + 8;
                                if (cur_cmd + cur_rep) >= reduced {
                                    for j in 0..MIN_ENC {
                                        best[iu + j as usize] = (0, 0);
                                    }
                                    best[next_index as usize] = (0, 0);
                                    did_reduce = true;
                                }
                            }
                        }
                    }
                }

                let (ml_final, mo_final) = best[iu];
                rep_off = mo_final;
                i += ml_final as i64;
                num_literals = 0;
            } else if ml == 1 {
                // ---- (R7) drop a stray 1-byte rep ----
                if num_literals != 0 {
                    let mut next_index = i + 1;
                    let mut next_literals: u32 = 0;
                    while next_index < end && best[next_index as usize].0 == 0 {
                        next_literals += 1;
                        next_index += 1;
                    }
                    if rep_off != mo && (next_index < end || !did_reduce) {
                        best[iu] = (0, 0);
                        did_reduce = true;
                        continue;
                    }
                    if next_literals != 0 {
                        let cur_partial =
                            lit_varlen_size(num_literals) + 1 + 1 + lit_varlen_size(next_literals);
                        let reduced_partial = lit_varlen_size(num_literals + 1 + next_literals) + 8;
                        if cur_partial >= reduced_partial {
                            best[iu] = (0, 0);
                            did_reduce = true;
                            continue;
                        }
                    }
                }
                num_literals = 0;
                i += 1;
            } else {
                num_literals += 1;
                i += 1;
            }
        }

        passes += 1;
        if !did_reduce || passes >= 30 {
            break;
        }
    }
}
