//! Multiplication-free ZX0-derived entropy backend for LZAN's 6510 track. Uses interlaced
//! Elias-gamma universal codes (the ZX0 / salvador coding) plus offset reuse, with no head-code
//! tables, so the 6510 decoder is small and table-free.
//!
//! All length/offset fields - literal-run length, new-offset match length, rep length, and the
//! offset MSB - are plain interlaced Elias gamma. The offset is coded as
//! `gamma(((off-1)>>7)+1) + 7 raw LSB bits`.
//!
//! ## Grammar (extends ZX0 with rep0-3)
//!
//! ZX0 alternates literal runs and matches with single flag bits, forbids two adjacent literal
//! runs (after a literal run there is always a match), and has one recent offset (rep0). This
//! generalizes the single rep to a 4-entry MTF queue (rep0-3). `rep_slots` (1..=4) selects the
//! mode. `rep_slots == 1` (rep0 only, no index code) is structurally identical to ZX0 - same
//! grammar, gamma coding, offset split, backtrack trick, and (via `zx0opt`) ZX0-optimal parse - but
//! NOT byte-compatible with stock ZX0 decoders: the layout differs (plain gamma, no invert_mode;
//! orig_len-driven termination instead of an in-stream EOF; LZAN container/first-command
//! convention), so `dzx0` cannot decode an LZAN stream. For a byte-identical ZX0 v2 stream use
//! `lzan zx0c` (`zx0compat.rs`). `rep_slots == 4` adds rep1-3 behind a short prefix code (a strict
//! superset of ZX0).
//!
//! Decoder state machine:
//!
//! ```text
//! COPY_LITERALS:                                   (first command is always literals)
//!     len   = gamma()                              ; copy `len` raw bytes
//!     if read_bit() == 1: goto NEW_OFFSET          ; 1 = new-offset match
//!     else:               goto REP                 ; 0 = rep match
//! REP:                                             (only reachable right after literals)
//!     ridx  = (rep_slots>1) ? read_rep_index() : 0 ; rep0=`1`, rep1=`01`, rep2=`001`, rep3=`000`
//!     off   = reps[ridx] ; MTF(ridx)
//!     len   = gamma()                              ; rep length, MIN 1
//!     if read_bit() == 0: goto COPY_LITERALS       ; 0 = next is a literal run
//!     else:               goto NEW_OFFSET          ; 1 = next is another (new-offset) match
//! NEW_OFFSET:
//!     msb   = gamma()                              ; offset high bits ((off-1)>>7)+1; no EOF marker
//!     lsb   = read_byte() >> 1                     ; low 7 bits of (off-1)
//!     off   = ((msb-1) << 7) | lsb ; off += 1
//!     reps  = insert_front(off)
//!     len   = gamma_with_backtrack() + 1           ; first gamma control bit shares lsb bit0; MIN 2
//!     if read_bit() == 1: goto NEW_OFFSET          ; 1 = another new-offset match
//!     else:               goto COPY_LITERALS       ; 0 = next is a literal run
//! ```
//!
//! Coding notes:
//! * Lit-run / new-offset length / offset MSB / rep length all use plain interlaced Elias gamma
//!   (no tables).
//! * Rep length has MIN 1, new-offset length MIN 2. A rep can copy a single byte (gamma(1) = 1 bit
//!   for the length), often cheaper than a raw literal.
//! * Rep-index prefix code tuned to the MTF distribution: rep0=`1` (1 bit), rep1=`01`, rep2=`001`,
//!   rep3=`000`. In `rep_slots == 1` mode the index code is omitted, so rep0 costs 0 index bits.
//! * No two adjacent literal runs; a rep follows only a literal run. The parser's cost model
//!   encodes these constraints, so the parse emits only representable command sequences.
//! * Backtrack bit: the new-offset length gamma's first control bit rides bit 0 of the offset LSB
//!   byte, saving 1 bit per match.
//! * No in-stream EOF: decode is driven by `orig_len` (stored in the LZAN container header) and
//!   stops when the output length is reached. The blob's first byte stores `rep_slots` so the
//!   entropy layer is self-describing.

// ---------------------------------------------------------------------------
// Bit I/O - MSB-first, with ZX0's backtrack trick.
// ---------------------------------------------------------------------------

/// MSB-first bit writer with a backtrack capability: the next bit written after `set_backtrack()`
/// is OR-ed into bit 0 of the most-recently emitted byte instead of starting a new bit position.
/// Used so a new-offset match's length-gamma first control bit rides in bit 0 of the offset LSB
/// byte.
pub struct BitWriter {
    bytes: Vec<u8>,
    bit_mask: u8, // 0 means "no partial byte open"; else the next bit goes here
    bit_index: usize,
    backtrack: bool,
}

impl BitWriter {
    pub fn new() -> Self {
        BitWriter {
            bytes: Vec::new(),
            bit_mask: 0,
            bit_index: 0,
            backtrack: false,
        }
    }

    #[inline]
    pub fn write_byte(&mut self, v: u8) {
        self.bytes.push(v);
    }

    #[inline]
    pub fn set_backtrack(&mut self) {
        self.backtrack = true;
    }

    #[inline]
    pub fn write_bit(&mut self, value: u32) {
        if self.backtrack {
            if value != 0 {
                let last = self.bytes.len() - 1;
                self.bytes[last] |= 1;
            }
            self.backtrack = false;
        } else {
            if self.bit_mask == 0 {
                self.bit_mask = 128;
                self.bit_index = self.bytes.len();
                self.bytes.push(0);
            }
            if value != 0 {
                self.bytes[self.bit_index] |= self.bit_mask;
            }
            self.bit_mask >>= 1;
        }
    }

    /// Interlaced Elias gamma of `value` (value >= 1). Emits the high data bits with a control
    /// bit between each (0 = more, 1 = stop), MSB-first.
    #[inline]
    pub fn write_gamma(&mut self, value: u32) {
        debug_assert!(value >= 1);
        // i = highest power of two <= value
        let mut i = 1u32 << (31 - value.leading_zeros());
        i >>= 1;
        while i != 0 {
            self.write_bit(0); // control: more bits follow
            self.write_bit(value & i);
            i >>= 1;
        }
        self.write_bit(1); // control: stop
    }

    pub fn finish(self) -> Vec<u8> {
        self.bytes
    }

    #[inline]
    pub fn len_bytes(&self) -> usize {
        self.bytes.len()
    }
}

/// MSB-first bit reader, including the backtrack trick.
pub struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    bit_mask: u8,
    bit_value: u8,
    backtrack: bool,
    last_byte: u8,
}

impl<'a> BitReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        BitReader {
            data,
            pos: 0,
            bit_mask: 0,
            bit_value: 0,
            backtrack: false,
            last_byte: 0,
        }
    }

    #[inline]
    pub fn read_byte(&mut self) -> u8 {
        let b = if self.pos < self.data.len() {
            self.data[self.pos]
        } else {
            0
        };
        self.pos += 1;
        self.last_byte = b;
        b
    }

    #[inline]
    pub fn set_backtrack(&mut self) {
        self.backtrack = true;
    }

    #[inline]
    pub fn read_bit(&mut self) -> u32 {
        if self.backtrack {
            self.backtrack = false;
            return (self.last_byte & 1) as u32;
        }
        self.bit_mask >>= 1;
        if self.bit_mask == 0 {
            self.bit_mask = 128;
            self.bit_value = self.read_byte();
        }
        if self.bit_value & self.bit_mask != 0 {
            1
        } else {
            0
        }
    }

    #[inline]
    pub fn read_gamma(&mut self) -> u32 {
        let mut value = 1u32;
        while self.read_bit() == 0 {
            value = (value << 1) | self.read_bit();
        }
        value
    }
}

// ---------------------------------------------------------------------------
// Cost helpers (bits).
// ---------------------------------------------------------------------------

/// Number of bits to encode `value` (>= 1) as interlaced Elias gamma.
/// value 1 -> 1, 2..3 -> 3, 4..7 -> 5, 8..15 -> 7, ...  == 1 + 2*floor(log2(value)).
#[inline]
pub fn gamma_bits(value: u32) -> u32 {
    debug_assert!(value >= 1);
    1 + 2 * (31 - value.leading_zeros())
}

// ===========================================================================
// Lengths and the offset MSB are plain interlaced Elias gamma; the 7-bit offset LSB is raw. The
// cost helpers below name each field for the parser/stats call sites; all reduce to `gamma_bits`.
// ===========================================================================

/// Cost helpers used by the parser's cost model (must match the encoder bit-exactly).
/// All four fields are plain interlaced Elias gamma.
#[inline]
pub fn lit_run_bits(run: u32) -> u32 {
    gamma_bits(run)
}
#[inline]
pub fn newoff_len_bits(len_minus_1: u32) -> u32 {
    gamma_bits(len_minus_1)
}
/// Rep length: interlaced Elias gamma.
#[inline]
pub fn rep_len_bits(len: u32) -> u32 {
    gamma_bits(len)
}
/// Offset MSB = ((off-1)>>7)+1, coded as interlaced Elias gamma.
#[inline]
pub fn off_msb_bits(msb: u32) -> u32 {
    gamma_bits(msb)
}
/// New-offset cost: gamma(MSB) + 7-bit raw LSB. Bit 0 of the LSB byte is borrowed by the
/// backtracked length-gamma's first control bit, so the offset itself contributes only 7 LSB bits.
/// Same value as `offset_cost_bits` (distinct name for the parser/stats call sites).
#[inline]
pub fn offset_cost_bits_hc(off: u32) -> u32 {
    let msb = ((off - 1) >> 7) + 1;
    gamma_bits(msb) + 7
}

/// Max offset = full 64 KB window (the C64 maximum: all data lives in 64 KB RAM). With no EOF
/// marker (orig_len drives termination) the gamma offset-MSB may exceed 256 (off 1..=65535 ->
/// msb 1..=512) with no ambiguity. The parse stays fast on pathologically repetitive >64 KB inputs
/// because `parse_zx` bounds its rep-probe and length-relaxation and `build_zx_matches` caps
/// chain/length above 64 KB.
pub const MAX_OFFSET: u32 = 0xFFFF; // 65535

/// Size threshold above which the LZAN-container rep0-only path uses the heuristic parse instead of
/// the exact ZX0 port (`zx0opt`). A time gate, not a memory one: `zx0opt` uses ZX0's refcount/ghost
/// recycler so live memory is O(n + window), but it is O(n * window) time. rep0-only is not the
/// per-file best-of winner for large non-C64 inputs, so the heuristic parse is used above this size
/// to keep `lzan c`'s 5-way best-of fast. The byte-identical ZX0 streams (`zx0c`/`zx0cb`,
/// zx0compat.rs) are not gated - they always run the exact parse, at any size.
pub const ZX0OPT_MAX_INPUT: usize = 33 * 1024;
/// Offset limit for the exact rep0-only port: the full 65535 window (MAX_OFFSET), wider than stock
/// zx0's 32640, so rep0-only can use offsets > 32640.
pub const ZX0OPT_OFFSET_LIMIT: usize = MAX_OFFSET as usize;

/// Cost in bits of a new-offset code (gamma(MSB) + 7-bit LSB), NOT counting the length:
/// 8 if off <= 128 else 7 + gamma_bits(((off-1)>>7)+1). (7 instead of 8 because the length-gamma's
/// first control bit rides bit 0 of the LSB byte via the backtrack trick.)
#[inline]
pub fn offset_cost_bits(off: u32) -> u32 {
    if off <= 128 {
        8
    } else {
        7 + gamma_bits(((off - 1) >> 7) + 1)
    }
}

// ---------------------------------------------------------------------------
// Command model shared with the parser. A `ZxCommand` is the same shape as parse::Command.
// ---------------------------------------------------------------------------

/// One emitted command: an optional literal run followed by a match (or a final literal-only
/// command). `match_off == 0` marks the terminal literal-only command.
///
/// `near_rep_ri`: when `>= 0`, this match's offset is coded as a near-rep relative to rep slot
/// `near_rep_ri` (off = reps[ri] ± δ). `-1` means code it normally (exact rep if `match_off` is a
/// live rep, else a full new-offset).
#[derive(Clone, Copy, Debug)]
pub struct ZxCommand {
    pub lit_len: u32,
    pub lit_start: usize,
    pub match_off: u32, // 0 => final literal-only command (no match)
    pub match_len: u32,
    pub near_rep_ri: i8,
}

// ---------------------------------------------------------------------------
// rep0-3 MTF queue (shared semantics with parser & decoder).
// ---------------------------------------------------------------------------

#[inline]
fn rep_insert(reps: &mut [u32; 4], off: u32) {
    reps[3] = reps[2];
    reps[2] = reps[1];
    reps[1] = reps[0];
    reps[0] = off;
}

#[inline]
fn rep_mtf(reps: &mut [u32; 4], idx: usize) {
    let v = reps[idx];
    let mut i = idx;
    while i > 0 {
        reps[i] = reps[i - 1];
        i -= 1;
    }
    reps[0] = v;
}

/// How many rep slots the encoder/parser/decoder use. 1 == rep0 only (ZX0); 4 == rep0-3.
/// The default `encode`/`decode` use `REP_SLOTS_DEFAULT`.
pub const REP_SLOTS_DEFAULT: usize = 4;

/// Bits to code rep index `idx` (0..=3) with >1 rep slot. Prefix code tuned to the MTF
/// distribution (rep0 dominates): rep0=`1` (1b), rep1=`01` (2b), rep2=`001` (3b), rep3=`000` (3b).
#[inline]
pub fn rep_index_bits(idx: usize) -> u32 {
    match idx {
        0 => 1,
        1 => 2,
        2 => 3,
        _ => 3,
    }
}

#[inline]
fn write_rep_index(w: &mut BitWriter, idx: usize) {
    // rep0=1; rep1=01; rep2=001; rep3=000
    match idx {
        0 => w.write_bit(1),
        1 => {
            w.write_bit(0);
            w.write_bit(1);
        }
        2 => {
            w.write_bit(0);
            w.write_bit(0);
            w.write_bit(1);
        }
        _ => {
            w.write_bit(0);
            w.write_bit(0);
            w.write_bit(0);
        }
    }
}

#[inline]
fn read_rep_index(r: &mut BitReader) -> usize {
    if r.read_bit() == 1 {
        0
    } else if r.read_bit() == 1 {
        1
    } else if r.read_bit() == 1 {
        2
    } else {
        3
    }
}

// ---------------------------------------------------------------------------
// Near-rep (offset-delta) coding. A new-offset whose value is within ±δ of a recent rep can be
// coded as `near-rep(ri) + sign + gamma(δ)`, cheaper than a full offset code when δ is small.
// Lives only in the after-literals rep-family prefix tree (the after-match new-offset path is
// untouched), as a complete prefix code over 7 symbols:
//
//   new-offset : `1`        (1 bit)
//   exact rep0 : `01`       (2)
//   exact rep1 : `001`      (3)
//   near-rep0  : `0001`     (4) + sign + gamma(δ)
//   exact rep3 : `00001`    (5)            [rep3 is more frequent than rep2, so 5 not 6]
//   exact rep2 : `000001`   (6)
//   near-rep1  : `000000`   (6) + sign + gamma(δ)
//
// `δ >= 1` (δ==0 is an exact rep, coded by the exact-rep symbol). The decoder reconstructs
// off = rep[ri] ± δ. The near-rep length uses the new-offset length gamma (MIN 2); no backtrack
// (there is no offset-LSB byte to ride bit 0 of). δ stays gamma.

/// After-literals symbol kinds for the near-rep grammar.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AfterLit {
    NewOffset,
    ExactRep(usize),     // ri in 0..4
    NearRep(usize, u32), // (ri, off): near rep ri, full reconstructed offset
}

/// Bits to code the after-literals symbol *prefix* (NOT counting the new-offset body or the
/// near-rep sign+gamma). Used by the parser's cost model.
#[inline]
pub fn after_lit_prefix_bits(sym: AfterLit) -> u32 {
    match sym {
        AfterLit::NewOffset => 1,
        AfterLit::ExactRep(0) => 2,
        AfterLit::ExactRep(1) => 3,
        AfterLit::ExactRep(2) => 6,
        AfterLit::ExactRep(_) => 5, // rep3: more frequent than rep2
        AfterLit::NearRep(0, _) => 4,
        AfterLit::NearRep(_, _) => 6,
    }
}

/// Bits for the near-rep delta body: 1 sign bit + gamma(δ). `delta >= 1`.
#[inline]
pub fn near_rep_delta_bits(delta: u32) -> u32 {
    1 + gamma_bits(delta)
}

#[inline]
fn write_after_lit_prefix(w: &mut BitWriter, sym: AfterLit) {
    // Tree: 1 / 01 / 001 / 0001 / 00001 / 000001 / 000000
    //       new  r0   r1   nr0    r3       r2       nr1
    let zeros = match sym {
        AfterLit::NewOffset => return w.write_bit(1),
        AfterLit::ExactRep(0) => 1,
        AfterLit::ExactRep(1) => 2,
        AfterLit::NearRep(0, _) => 3,
        AfterLit::ExactRep(3) => 4,
        AfterLit::ExactRep(_) => 5, // rep2
        AfterLit::NearRep(_, _) => {
            // nr1: 000000 (terminal, no closing 1)
            for _ in 0..6 {
                w.write_bit(0);
            }
            return;
        }
    };
    for _ in 0..zeros {
        w.write_bit(0);
    }
    w.write_bit(1);
}

/// Read the after-literals symbol prefix (near-rep grammar). Returns the kind without the
/// near-rep body (sign+gamma read separately by the caller).
#[inline]
fn read_after_lit_prefix(r: &mut BitReader) -> AfterLit {
    if r.read_bit() == 1 {
        return AfterLit::NewOffset;
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(0);
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(1);
    }
    if r.read_bit() == 1 {
        return AfterLit::NearRep(0, 0);
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(3);
    }
    if r.read_bit() == 1 {
        AfterLit::ExactRep(2)
    } else {
        AfterLit::NearRep(1, 0)
    }
}

/// Encode a near-rep delta (sign + gamma) of `off` relative to `base` (the rep value).
#[inline]
fn write_near_rep_delta(w: &mut BitWriter, base: u32, off: u32) {
    let (sign, delta) = if off >= base {
        (0u32, off - base)
    } else {
        (1u32, base - off)
    };
    debug_assert!(delta >= 1);
    w.write_bit(sign);
    w.write_gamma(delta);
}

/// Decode a near-rep offset given the rep base.
#[inline]
fn read_near_rep_off(r: &mut BitReader, base: u32) -> u32 {
    let sign = r.read_bit();
    let delta = r.read_gamma();
    if sign == 0 {
        base + delta
    } else {
        base - delta
    }
}

// ---------------------------------------------------------------------------
// After-MATCH near-rep. The after-match connecting decision is normally a single bit (0 = next is
// a literal run, 1 = next is a new-offset match). With `am_near_rep` mode on, the "next is a match"
// branch is split so a match whose offset is rep0±δ or rep1±δ can be coded as a cheap near-rep body
// instead of a full offset code. The decoder already knows it is in the after-match state, so no
// global flag bit is needed; only the local prefix grows:
//
//   next = literals     : `0`            (1 bit)
//   next = new-offset    : `10`           (2 bits - a +1-bit cost on plain after-match new-offsets)
//   next = near-rep(ri)  : `11` + ri-bit + sign + gamma(δ)   (ri in {0,1}; then newoff_len(len-1))
//
// Best-of-per-file turns this mode on only when the near-rep savings outweigh the +1-bit cost on
// plain after-match new-offsets. The near-rep length uses the new-offset length gamma (MIN 2); no
// backtrack (no offset LSB byte).

/// After-match connecting symbol kinds (only used in `am_near_rep` mode).
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AfterMatch {
    Literals,
    NewOffset,
    NearRep(usize, u32), // (ri in {0,1}, full reconstructed off)
}

/// Bits for the after-match connecting prefix (selector only; the near-rep sign+gamma and the
/// length are counted separately), in `am_near_rep` mode.
#[inline]
pub fn after_match_prefix_bits(sym: AfterMatch) -> u32 {
    match sym {
        AfterMatch::Literals => 1,      // 0
        AfterMatch::NewOffset => 2,     // 10
        AfterMatch::NearRep(_, _) => 3, // 11 + 1 ri-bit
    }
}

#[inline]
fn write_after_match_prefix(w: &mut BitWriter, sym: AfterMatch) {
    match sym {
        AfterMatch::Literals => w.write_bit(0),
        AfterMatch::NewOffset => {
            w.write_bit(1);
            w.write_bit(0);
        }
        AfterMatch::NearRep(ri, _) => {
            w.write_bit(1);
            w.write_bit(1);
            w.write_bit(ri as u32); // ri in {0,1}: rep0=0, rep1=1
        }
    }
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// Encode a command list + input into the ZX blob. `rep_slots` in 1..=4 selects how many recent
/// offsets are reusable (1 = rep0 only). The decoder must use the same `rep_slots`.
///
/// Blob layout: `[mode:u8][bitstream...]` where mode = `rep_slots | (near_rep << 4)`, stored in the
/// first byte so the decoder is self-describing. `orig_len` is supplied separately to `decode`.
pub fn encode_with(input: &[u8], cmds: &[ZxCommand], rep_slots: usize) -> Vec<u8> {
    encode_with2(input, cmds, rep_slots, false)
}

/// Encode with optional near-rep (offset-delta) coding in the after-literals position. `near_rep`
/// is only honored with `rep_slots == 4` (it shares the rep0-3 prefix tree). The mode byte stores
/// `rep_slots | (near_rep << 4)` so the decoder is self-describing.
pub fn encode_with2(input: &[u8], cmds: &[ZxCommand], rep_slots: usize, near_rep: bool) -> Vec<u8> {
    encode_with3(input, cmds, rep_slots, near_rep, false)
}

/// As `encode_with2`, plus optional after-MATCH near-rep coding (`am_near_rep`). Honored only with
/// `rep_slots == 4`. The mode byte stores `rep_slots | (near_rep<<4) | (am_near_rep<<5)`.
pub fn encode_with3(
    input: &[u8],
    cmds: &[ZxCommand],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<u8> {
    assert!((1..=4).contains(&rep_slots));
    let near_rep = near_rep && rep_slots == 4;
    let am_near_rep = am_near_rep && rep_slots == 4;
    let mut w = BitWriter::new();
    w.write_byte(rep_slots as u8 | ((near_rep as u8) << 4) | ((am_near_rep as u8) << 5));

    // Initial last_offset is 1; all rep slots start at 1 to match the decoder.
    let mut reps = [1u32, 1, 1, 1];

    // The first command starts with a literal run (the whole file may be one literal run for
    // incompressible data). Commands are [lit_run][match]; replay them, emitting the connecting
    // flag bit from the previous unit type:
    //   - after a literal run: 1 bit (1 = new-offset match, 0 = rep match)
    //   - after a match:       1 bit (0 = next is literal run, 1 = next is new-offset match)
    // Two literal runs can't be adjacent and a rep can only follow a literal run, so the parser's
    // command stream is always representable (asserted below). Each command = (lit_run of length L,
    // then a match) except the terminal one.

    #[derive(PartialEq)]
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;

    let ncmds = cmds.len();
    for (ci, c) in cmds.iter().enumerate() {
        let is_final = c.match_off == 0;
        debug_assert!(!is_final || ci == ncmds - 1, "literal-only cmd not last");

        // ----- literal run -----
        if c.lit_len > 0 {
            // Emit the connecting flag if the previous unit was a match: 0 = literals follow.
            // (In am_near_rep mode this `0` is the AfterMatch::Literals prefix - still a single 0 bit.)
            match prev {
                Prev::Match => w.write_bit(0),
                Prev::Start => { /* first command, literals implied, no flag */ }
                Prev::Literals => unreachable!("two adjacent literal runs"),
            }
            w.write_gamma(c.lit_len);
            for k in 0..c.lit_len as usize {
                w.write_byte(input[c.lit_start + k]);
            }
            prev = Prev::Literals;
        }

        if is_final {
            // Final literal-only command: nothing more (lit_len == 0 trailing is also fine).
            break;
        }

        // ----- match -----
        let off = c.match_off;
        let len = c.match_len;

        // Decide rep vs new-offset. A rep is legal only if the previous unit was a literal run
        // (rep is reachable only from COPY_LITERALS). Otherwise use a new-offset match even if off
        // is a live rep.
        let rep_idx = if prev == Prev::Literals {
            let mut found = None;
            for r in 0..rep_slots {
                if reps[r] == off {
                    found = Some(r);
                    break;
                }
            }
            found
        } else {
            None
        };

        match prev {
            Prev::Literals => {
                if near_rep && c.near_rep_ri >= 0 {
                    // --- near-rep match: off = reps[ri] ± δ ---
                    let ri = c.near_rep_ri as usize;
                    debug_assert!(ri < 4, "near-rep base must be a live rep slot");
                    debug_assert!(reps[ri] != off, "near-rep δ must be >= 1");
                    write_after_lit_prefix(&mut w, AfterLit::NearRep(ri, off));
                    write_near_rep_delta(&mut w, reps[ri], off);
                    // length codes like a new-offset match: gamma(len-1), MIN 2, no backtrack
                    // (the near-rep body has no offset LSB byte).
                    w.write_gamma(len - 1);
                    rep_insert(&mut reps, off);
                } else if let Some(ridx) = rep_idx {
                    // --- exact rep match ---
                    if near_rep {
                        write_after_lit_prefix(&mut w, AfterLit::ExactRep(ridx));
                    } else {
                        w.write_bit(0); // after-literals flag: 0 = rep
                        if rep_slots > 1 {
                            write_rep_index(&mut w, ridx);
                        }
                    }
                    w.write_gamma(len); // rep length: gamma
                    rep_mtf(&mut reps, ridx);
                } else {
                    // --- new-offset match after literals ---
                    if near_rep {
                        write_after_lit_prefix(&mut w, AfterLit::NewOffset);
                    } else {
                        w.write_bit(1); // after-literals flag: 1 = new offset
                    }
                    emit_new_offset(&mut w, off, len);
                    rep_insert(&mut reps, off);
                }
            }
            Prev::Match => {
                if am_near_rep && c.near_rep_ri >= 0 {
                    // --- after-match NEAR-REP: 11 + ri-bit + sign + gamma(δ) + newoff_len(len-1) ---
                    let ri = c.near_rep_ri as usize;
                    debug_assert!(ri < 2, "after-match near-rep base must be rep0 or rep1");
                    debug_assert!(reps[ri] != off, "after-match near-rep δ must be >= 1");
                    write_after_match_prefix(&mut w, AfterMatch::NearRep(ri, off));
                    write_near_rep_delta(&mut w, reps[ri], off);
                    w.write_gamma(len - 1); // MIN 2, no backtrack
                    rep_insert(&mut reps, off);
                } else {
                    // --- new-offset match after a match ---
                    debug_assert!(!am_near_rep || c.near_rep_ri < 0);
                    if am_near_rep {
                        write_after_match_prefix(&mut w, AfterMatch::NewOffset);
                    // 10 (the +1-bit tax)
                    } else {
                        w.write_bit(1); // after-match flag: 1 = another new-offset match
                    }
                    emit_new_offset(&mut w, off, len);
                    rep_insert(&mut reps, off);
                }
            }
            Prev::Start => {
                // The first command is COPY_LITERALS; a match cannot be the first unit. The parser
                // always emits a leading literal run before any match, so this is unreachable.
                unreachable!("match as first unit without literals");
            }
        }
        prev = Prev::Match;
    }

    // No in-stream EOF marker: decode is driven by `orig_len` (stored in the LZAN container
    // header), so it stops when the output length is reached and never reads a trailing marker.
    let _ = prev;
    w.finish()
}

#[inline]
fn emit_new_offset(w: &mut BitWriter, off: u32, len: u32) {
    debug_assert!(off >= 1 && off <= MAX_OFFSET, "offset {} out of range", off);
    // offset MSB via interlaced Elias gamma: msb = ((off-1) >> 7) + 1.
    let msb = ((off - 1) >> 7) + 1;
    w.write_gamma(msb);
    // LSB byte = low 7 bits of (off-1) in bits 7..1; bit 0 reserved for the backtracked length
    // gamma's first control bit.
    let lsb = ((off - 1) & 0x7f) as u8;
    w.write_byte(lsb << 1);
    // new-offset length via gamma(len-1); first control bit backtracked into LSB bit 0.
    w.set_backtrack();
    w.write_gamma(len - 1);
}

/// Default encode using rep0-3.
pub fn encode(input: &[u8], cmds: &[ZxCommand]) -> Vec<u8> {
    encode_with(input, cmds, REP_SLOTS_DEFAULT)
}

// ---------------------------------------------------------------------------
// MINIMAL EOF MODE (mode byte 0x41) - rep0-only + in-stream ZX0-style EOF marker.
//
// A variant for the smallest 6510 decoder. Bit-identical to the rep0-only stream (mode 0x01)
// except it appends an in-stream EOF marker so the decoder self-terminates without a 16-bit
// `remain` counter. The marker is a new-offset whose offset-MSB gamma value is `EOF_MSB` (256).
// Real offsets are capped to `EOF_MAX_OFFSET` (32640) so a real offset MSB never reaches 256; the
// decoder recognises EOF with a single high-byte test after the offset-MSB gamma read.
//
// The marker costs gamma(256) (17 bits) + 1 LSB byte = 25 bits per file, in exchange for dropping
// the orig_len-driven `remain` clamp from the 6510 decoder.

/// EOF offset-MSB sentinel: gamma value 256. Real offsets are capped below this.
pub const EOF_MSB: u32 = 256;
/// Max real offset in minimal-EOF mode: ((32640-1)>>7)+1 == 255 < EOF_MSB, so the real msb stays a
/// single byte and the decoder's EOF test is a one-byte `val_hi != 0`.
pub const EOF_MAX_OFFSET: u32 = 32640;
/// Mode byte for the minimal-EOF stream: bit 6 set = EOF mode, low nibble = rep_slots (1).
pub const MODE_MIN_EOF: u8 = 0x41;

/// Encode the rep0-only stream with an in-stream EOF marker (mode 0x41). Decode is driven by the
/// marker, not orig_len, so the 6510 decoder needs no remain counter. `cmds` must use offsets
/// <= EOF_MAX_OFFSET (the caller's match window is capped to guarantee this).
pub fn encode_min_eof(input: &[u8], cmds: &[ZxCommand]) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.write_byte(MODE_MIN_EOF);

    let mut reps = [1u32, 1, 1, 1]; // only rep0 used
    #[derive(PartialEq)]
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;

    let ncmds = cmds.len();
    for (ci, c) in cmds.iter().enumerate() {
        let is_final = c.match_off == 0;
        debug_assert!(!is_final || ci == ncmds - 1, "literal-only cmd not last");

        if c.lit_len > 0 {
            match prev {
                Prev::Match => w.write_bit(0),
                Prev::Start => {}
                Prev::Literals => unreachable!("two adjacent literal runs"),
            }
            w.write_gamma(c.lit_len);
            for k in 0..c.lit_len as usize {
                w.write_byte(input[c.lit_start + k]);
            }
            prev = Prev::Literals;
        }
        if is_final {
            break;
        }

        let off = c.match_off;
        let len = c.match_len;
        // Hard assert (not debug_assert): an over-window offset would make the offset-MSB gamma
        // collide with the EOF marker and emit a silently corrupt stream in release builds.
        assert!(
            off <= EOF_MAX_OFFSET,
            "offset {} exceeds EOF_MAX_OFFSET",
            off
        );
        let is_rep0 = prev == Prev::Literals && reps[0] == off;

        match prev {
            Prev::Literals => {
                if is_rep0 {
                    w.write_bit(0); // after-literals flag: 0 = rep (rep0, no index)
                    w.write_gamma(len); // rep length, MIN 1
                } else {
                    w.write_bit(1); // after-literals flag: 1 = new offset
                    emit_new_offset(&mut w, off, len);
                    reps[0] = off;
                }
            }
            Prev::Match => {
                w.write_bit(1); // after-match flag: 1 = another new-offset match
                emit_new_offset(&mut w, off, len);
                reps[0] = off;
            }
            Prev::Start => unreachable!("match as first unit without literals"),
        }
        prev = Prev::Match;
    }

    // In-stream EOF marker: a new-offset whose MSB gamma == EOF_MSB (256). The decoder reads the
    // gamma, sees val_hi != 0, and stops (it never reads the trailing dummy LSB byte). The LSB byte
    // is still written so the marker is a well-formed new-offset prefix.
    if prev == Prev::Match {
        w.write_bit(1); // after-match: pretend "another new-offset" so the decoder enters newoff
    } else if prev == Prev::Literals {
        w.write_bit(1); // after-literals: 1 = new offset
    } else {
        // empty input has no commands; with orig_len==0 the decoder returns before reading anything.
    }
    w.write_gamma(EOF_MSB);
    w.write_byte(0); // dummy LSB (never read by the decoder)
    w.finish()
}

/// High-level minimal-EOF compress: rep0-only optimal parse with the window capped to
/// EOF_MAX_OFFSET, then `encode_min_eof`. Runs at `DEFAULT_EFFORT`.
pub fn compress_min_eof(input: &[u8]) -> Vec<u8> {
    compress_min_eof_e(input, DEFAULT_EFFORT)
}

/// As `compress_min_eof`, with an explicit encoder effort tier:
///   3 = optimal  - exact ZX0 port (window capped to EOF_MAX_OFFSET) when affordable (<=33 KB).
///   2 = balanced - multi-arrival rep0-only parse (`parse_zx3`).
///   1 = fast     - single-pass rep0-only parse (`parse_zx3_fast`).
/// Above the size cap the optimal tier falls back to the balanced parse.
pub fn compress_min_eof_e(input: &[u8], effort: u8) -> Vec<u8> {
    use crate::parse;
    if input.is_empty() {
        return encode_min_eof(input, &[]);
    }
    let window = (EOF_MAX_OFFSET as usize).min(input.len());
    let cmds = match effort {
        1 => {
            let ms = build_zx_matches(input, window);
            parse::parse_zx3_fast(input, &ms, 1, false, false)
        }
        2 => {
            let ms = build_zx_matches(input, window);
            parse::parse_zx3(input, &ms, 1, false, false)
        }
        _ if input.len() <= ZX0OPT_MAX_INPUT => crate::zx0opt::optimize_zx0(input, 0, window),
        _ => {
            let ms = build_zx_matches(input, window);
            parse::parse_zx3(input, &ms, 1, false, false)
        }
    };
    encode_min_eof(input, &cmds)
}

/// Backward/in-place variant of the minimal-EOF compressor, following the repo convention
/// (`encode_with3_backward`): reverse the input, run the forward pipeline, then reverse the
/// payload bytes - `[mode_byte] ++ reverse(payload)`. A descending 6510 byte reader then
/// reproduces the forward bit sequence exactly, so the decoder's bit logic is unchanged.
pub fn compress_min_eof_backward(input: &[u8]) -> Vec<u8> {
    compress_min_eof_backward_e(input, DEFAULT_EFFORT)
}

/// As `compress_min_eof_backward` with an explicit effort tier.
pub fn compress_min_eof_backward_e(input: &[u8], effort: u8) -> Vec<u8> {
    let mut rev: Vec<u8> = input.to_vec();
    rev.reverse();
    let fwd = compress_min_eof_e(&rev, effort);
    let mut out = Vec::with_capacity(fwd.len());
    out.push(fwd[0]);
    out.extend(fwd[1..].iter().rev().copied());
    out
}

/// Reference decoder for the backward minimal-EOF stream (mode byte stripped):
/// reverse the payload, decode forward, reverse the output.
pub fn decode_min_eof_backward(body: &[u8], orig_len: usize) -> Vec<u8> {
    let rev: Vec<u8> = body.iter().rev().copied().collect();
    let mut out = decode_min_eof(&rev, orig_len);
    out.reverse();
    out
}

/// Reference decoder for the minimal-EOF stream (mode-byte stripped, so `body` starts at the
/// bitstream). rep0-only, terminating on the in-stream EOF marker (offset-MSB gamma >= 256), not on
/// orig_len. `orig_len` only sizes the output buffer and handles the empty-input case.
pub fn decode_min_eof(body: &[u8], orig_len: usize) -> Vec<u8> {
    decode_min_eof_with_gap(body, orig_len).0
}

/// Like [`decode_min_eof`], plus the in-place safety gap (bytes): the peak of
/// `output_produced - input_consumed` over the decode minus its final value.
/// `r.pos` is the input byte position. See [`max_gap_min_forward`].
fn decode_min_eof_with_gap(body: &[u8], orig_len: usize) -> (Vec<u8>, i32) {
    let mut out: Vec<u8> = Vec::with_capacity(orig_len);
    if orig_len == 0 {
        return (out, 0);
    }
    let mut r = BitReader::new(body);
    let mut rep0 = 1u32;
    let mut max_gap = 0i32;

    // State machine: Literals -> (new-offset | rep0) -> after-match -> ...
    // Start in Literals (first command is always a literal run).
    'outer: loop {
        // ---- literals ----
        let len = r.read_gamma();
        for _ in 0..len {
            out.push(r.read_byte());
        }
        let g = out.len() as i32 - r.pos as i32;
        if g > max_gap {
            max_gap = g;
        }
        // after-literals flag: 1 = new offset, 0 = rep0
        let mut new_offset = r.read_bit() == 1;
        loop {
            let len;
            if new_offset {
                // ---- new offset ----
                let msb = r.read_gamma();
                if msb >= EOF_MSB {
                    break 'outer; // EOF marker
                }
                let lsb = (r.read_byte() >> 1) as u32;
                let off = (((msb - 1) << 7) | lsb) + 1;
                rep0 = off;
                r.set_backtrack();
                len = r.read_gamma() + 1; // new-offset length: gamma(len-1), MIN 2
            } else {
                // ---- rep0 ----
                len = r.read_gamma();
            }
            // copy match from rep0
            let off = rep0 as usize;
            for _ in 0..len {
                let b = out[out.len() - off];
                out.push(b);
            }
            let g = out.len() as i32 - r.pos as i32;
            if g > max_gap {
                max_gap = g;
            }
            // after-match flag: 1 = another new offset, 0 = back to literals
            if r.read_bit() == 1 {
                new_offset = true;
                continue;
            } else {
                continue 'outer;
            }
        }
    }
    out.truncate(orig_len);
    let final_gap = out.len() as i32 - body.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD lzan-min stream (mode byte
/// already stripped). The stream self-terminates on its EOF marker, so a large
/// output bound suffices.
pub fn max_gap_min_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    decode_min_eof_with_gap(stream, 0x1_0000).1.max(0) as usize
}

/// In-place safety margin (bytes) for a BACKWARD lzan-min stream - the reverse
/// of a forward pack (`compress_min_eof_backward`), decoded by the descending
/// 6502 reader as a forward decode of the reversed stream.
pub fn max_gap_min_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    decode_min_eof_with_gap(&rev, 0x1_0000).1.max(0) as usize
}

/// Default encoder effort for the no-effort `compress*` entry points. 3 = optimal.
pub const DEFAULT_EFFORT: u8 = 3;

/// High-level: match-find + optimal ZX parse + encode, for a whole input buffer. `rep_slots`
/// selects rep0 only (1) vs rep0-3 (4). Compression is offline.
///
/// Runs at `DEFAULT_EFFORT` (optimal). Use `compress_e` to pick a tier.
pub fn compress(input: &[u8], rep_slots: usize) -> Vec<u8> {
    compress_e(input, rep_slots, DEFAULT_EFFORT)
}

/// Like `compress`, with an explicit encoder effort (1=fast / 2=balanced / 3=optimal). The effort
/// only changes which parse runs, never the grammar, so the decoder is identical across tiers.
pub fn compress_e(input: &[u8], rep_slots: usize, effort: u8) -> Vec<u8> {
    // rep0-only at optimal effort uses the exact ZX0 port (zx0opt); its parse equals ZX0's optimum
    // by construction, using the full 65535 window instead of zx0's 32640 limit. For balanced/fast
    // effort, or for rep0-3, the multi-arrival parse runs (selected by effort inside `compress3_e`).
    //
    // The exact port is O(n * window) time, O(n + window) memory. It is gated here to the C64
    // payload size (<= ZX0OPT_MAX_INPUT) to keep `lzan c`'s 5-way best-of fast; above that the
    // multi-arrival parse runs instead. The byte-identical ZX0 path `zx0c`/`zx0cb` is ungated and
    // always runs the exact parse (see zx0compat.rs).
    if effort >= 3 && rep_slots == 1 && !input.is_empty() && input.len() <= ZX0OPT_MAX_INPUT {
        let window = (ZX0OPT_OFFSET_LIMIT).min(input.len());
        let cmds = crate::zx0opt::optimize_zx0(input, 0, window);
        return encode_with(input, &cmds, 1);
    }
    compress3_e(input, rep_slots, false, false, effort)
}

/// Like `compress`, but with optional near-rep (offset-delta) coding (`rep_slots==4` only).
/// Routes through the complete-candidate full-format parse (am_near_rep off). Runs at
/// `DEFAULT_EFFORT`.
pub fn compress2(input: &[u8], rep_slots: usize, near_rep: bool) -> Vec<u8> {
    compress3_e(input, rep_slots, near_rep, false, DEFAULT_EFFORT)
}

/// Like `compress2`, plus optional after-MATCH near-rep coding (`am_near_rep`). Honored only with
/// `rep_slots==4`. The two near-rep families (after-lit and after-match) can be on together;
/// best-of-per-file decides whether the mode helps. Runs at `DEFAULT_EFFORT`.
pub fn compress3(input: &[u8], rep_slots: usize, near_rep: bool, am_near_rep: bool) -> Vec<u8> {
    compress3_e(input, rep_slots, near_rep, am_near_rep, DEFAULT_EFFORT)
}

/// As `compress3`, with an explicit encoder effort tier. The full-format entry point that all ZX
/// modes (rep0-3 0x04, near-rep 0x14/0x24, after-match 0x34) flow through. The effort picks the
/// parse:
///   3 = optimal  - `parse_zx3_complete` (complete candidate set; <=33 KB), optimal for rep0-only;
///                  falls back to balanced above the size cap.
///   2 = balanced - `parse_zx3` (seeding + reparse rounds + reduce).
///   1 = fast     - `parse_zx3_fast` (single multi-arrival DP pass).
pub fn compress3_e(
    input: &[u8],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
    effort: u8,
) -> Vec<u8> {
    use crate::parse;
    if input.is_empty() {
        return encode_with3(input, &[], rep_slots, near_rep, am_near_rep);
    }
    let window = (MAX_OFFSET as usize).min(input.len());
    let ms = build_zx_matches(input, window);
    let cmds = match effort {
        // fast: one wide-beam DP pass over the raw Pareto match set.
        1 => parse::parse_zx3_fast(input, &ms, rep_slots, near_rep, am_near_rep),
        // balanced: seeded/reparsed/reduced parse.
        2 => parse::parse_zx3(input, &ms, rep_slots, near_rep, am_near_rep),
        // optimal (default / >=3): the complete candidate set when the all-offset scan is affordable
        // (<=33 KB); otherwise the balanced parse. The complete-candidate parse is best-of {complete
        // DP, heuristic, (rep0) ZX0-exact}, so it never regresses vs the balanced parse.
        _ => match build_complete_extra(input, window) {
            Some(extra) => {
                parse::parse_zx3_complete(input, &ms, &extra, rep_slots, near_rep, am_near_rep)
            }
            None => parse::parse_zx3(input, &ms, rep_slots, near_rep, am_near_rep),
        },
    };
    encode_with3(input, &cmds, rep_slots, near_rep, am_near_rep)
}

/// Like `compress3`, but also returns the parsed commands (for bit-exactness verification).
pub fn compress3_with_cmds(
    input: &[u8],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> (Vec<u8>, Vec<ZxCommand>) {
    use crate::parse;
    if input.is_empty() {
        return (
            encode_with3(input, &[], rep_slots, near_rep, am_near_rep),
            Vec::new(),
        );
    }
    let window = (MAX_OFFSET as usize).min(input.len());
    let ms = build_zx_matches(input, window);
    let cmds = parse::parse_zx3(input, &ms, rep_slots, near_rep, am_near_rep);
    (
        encode_with3(input, &cmds, rep_slots, near_rep, am_near_rep),
        cmds,
    )
}

/// Like `compress`, but also returns the parsed commands (for diagnostics).
pub fn compress_with_cmds(input: &[u8], rep_slots: usize) -> (Vec<u8>, Vec<ZxCommand>) {
    use crate::parse;
    if input.is_empty() {
        return (encode_with(input, &[], rep_slots), Vec::new());
    }
    let window = (MAX_OFFSET as usize).min(input.len());
    let ms = build_zx_matches(input, window);
    let cmds = parse::parse_zx(input, &ms, rep_slots);
    (encode_with(input, &cmds, rep_slots), cmds)
}

/// Diagnostic helper for the ZX0-exactness comparison: rep0-only parse with a custom offset window
/// (so it can match ZX0's 32640 limit), returning the bit-exact predicted payload bits (excluding
/// the mode byte and any EOF marker - comparable to ZX0's `optimal->bits`). Returns
/// (predicted_bits, blob_len_bytes, roundtrip_ok).
pub fn rep0_cost_with_window(input: &[u8], window: usize) -> (u64, usize, bool) {
    use crate::parse;
    if input.is_empty() {
        let blob = encode_with(input, &[], 1);
        return (0, blob.len(), decode(&blob, 0) == input);
    }
    let window = window.min(input.len());
    let ms = build_zx_matches(input, window);
    let cmds = parse::parse_zx(input, &ms, 1);
    let bits = predicted_payload_bits(input, &cmds, 1, false, false);
    let blob = encode_with(input, &cmds, 1);
    let ok = decode(&blob, input.len()) == input;
    (bits, blob.len(), ok)
}

/// Cap on extra (non-Pareto) offsets per position fed to the complete-candidate full-format parse.
/// Larger = more rep-enabling offsets visible, at the cost of pass-B DP time.
const COMPLETE_EXTRA_PER_POS: usize = 64;

/// Build the complete (all-distinct-offset) extra candidate table for the full-format DP, sized to
/// the C64 target (<=33 KB → brute-force; larger → skip, the heuristic seed covers it).
fn build_complete_extra(input: &[u8], window: usize) -> Option<Vec<Vec<(u32, u32)>>> {
    use crate::matchfinder;
    use crate::parse;
    if input.len() > 33000 {
        return None; // the all-offset scan is affordable only at C64 sizes
    }
    let mm = parse::MIN_MATCH as usize;
    let ms = matchfinder::find_matches_exact(input, mm, window, 1 << 17);
    Some(matchfinder::build_complete_extra(
        input,
        &ms,
        mm,
        window,
        1 << 17,
        COMPLETE_EXTRA_PER_POS,
    ))
}

/// Full-format compress with the complete candidate set. Falls back to the heuristic `compress3`
/// above the size cap. `rep_slots`/`near_rep`/`am_near_rep` select the grammar. The parse is
/// best-of {complete-candidate DP, heuristic-seeded, (rep0) ZX0-exact}.
pub fn compress_complete(
    input: &[u8],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<u8> {
    // The "complete" entry point is the optimal tier.
    compress3_e(input, rep_slots, near_rep, am_near_rep, 3)
}

/// ZX0-exact rep0-only cost: parse via the `zx0opt` port, encode rep0-only, and return
/// (predicted_bits, blob_len, roundtrip_ok). With `offset_limit == 32640` this equals stock zx0's
/// `optimal->bits` byte-for-byte.
pub fn rep0_zx0exact_cost(input: &[u8], offset_limit: usize) -> (u64, usize, bool) {
    if input.is_empty() {
        let blob = encode_with(input, &[], 1);
        return (0, blob.len(), decode(&blob, 0) == input);
    }
    let cmds = crate::zx0opt::optimize_zx0(input, 0, offset_limit);
    let bits = predicted_payload_bits(input, &cmds, 1, false, false);
    let blob = encode_with(input, &cmds, 1);
    let ok = decode(&blob, input.len()) == input;
    (bits, blob.len(), ok)
}

/// Build the match candidate set for the ZX parse. The match extension is O(run), so the match
/// finder scales with input size:
///   - <=33 KB  : exact brute-force Pareto front (best ratio, the C64 target).
///   - 33-64 KB : deep hash chain, generous match length.
///   - >64 KB   : hard caps - bounded so the O(run) extension stays fast on pathologically
///                repetitive inputs; the parse's LEAVE_ALONE + rep-probe cap bound the rest.
fn build_zx_matches(input: &[u8], window: usize) -> crate::matchfinder::MatchSet {
    use crate::matchfinder;
    use crate::parse;
    let mm = parse::MIN_MATCH as usize;
    if input.len() <= 33000 {
        matchfinder::find_matches_exact(input, mm, window, 1 << 17)
    } else if input.len() <= 64 * 1024 {
        matchfinder::find_matches(input, mm, window, 1 << 16, 8192)
    } else {
        matchfinder::find_matches(input, mm, window, 1024, 4096)
    }
}

/// Diagnostic: per-category bit tallies from re-walking the command stream.
#[derive(Default, Debug, Clone, Copy)]
pub struct ZxStats {
    pub n_lit_runs: u64,
    pub lit_bytes: u64,
    pub lit_frame_bits: u64, // gamma(run) bits only
    pub n_newoff: u64,
    pub newoff_off_bits: u64,
    pub newoff_len_bits: u64,
    pub rep_counts: [u64; 4],
    pub rep_index_bits: u64,
    pub rep_len_bits: u64,
    pub flag_bits: u64,
    pub total_bits: u64,
}

/// Histograms of the gamma-coded values, for measuring the field distributions. Indexed by the
/// value being gamma-coded (clamped into the array).
#[derive(Clone)]
pub struct ZxHist {
    /// literal run length L (gamma(L)), L>=1
    pub lit_run: Vec<u64>,
    /// new-offset match length encoded as (len-1), len>=2 so value>=1 -> gamma(len-1)
    pub newoff_len: Vec<u64>,
    /// rep match length L (gamma(L)), L>=1
    pub rep_len: Vec<u64>,
    /// new-offset MSB value = ((off-1)>>7)+1, value>=1 -> gamma(msb)
    pub off_msb: Vec<u64>,
    /// rep index 0..3
    pub rep_idx: [u64; 4],
    /// raw 7-bit offset LSB value (low 7 bits of off-1), 0..127
    pub off_lsb: [u64; 128],
    /// near offsets (off_msb==1, i.e. off in 1..=128): the actual offset value 1..128
    pub near_off: [u64; 129],
    /// full offset value histogram (clamped to 65536)
    pub off_full: Vec<u64>,
}

impl Default for ZxHist {
    fn default() -> Self {
        ZxHist {
            lit_run: vec![0; 1 << 16],
            newoff_len: vec![0; 1 << 16],
            rep_len: vec![0; 1 << 16],
            off_msb: vec![0; 1 << 10],
            rep_idx: [0; 4],
            off_lsb: [0; 128],
            near_off: [0; 129],
            off_full: vec![0; 1 << 16],
        }
    }
}

impl ZxHist {
    pub fn merge(&mut self, o: &ZxHist) {
        for i in 0..self.lit_run.len() {
            self.lit_run[i] += o.lit_run[i];
        }
        for i in 0..self.newoff_len.len() {
            self.newoff_len[i] += o.newoff_len[i];
        }
        for i in 0..self.rep_len.len() {
            self.rep_len[i] += o.rep_len[i];
        }
        for i in 0..self.off_msb.len() {
            self.off_msb[i] += o.off_msb[i];
        }
        for i in 0..4 {
            self.rep_idx[i] += o.rep_idx[i];
        }
        for i in 0..128 {
            self.off_lsb[i] += o.off_lsb[i];
        }
        for i in 0..129 {
            self.near_off[i] += o.near_off[i];
        }
        for i in 0..self.off_full.len() {
            self.off_full[i] += o.off_full[i];
        }
    }
}

/// Collect raw value histograms (same walk as `stats`) for distribution / entropy analysis.
pub fn hist(_input: &[u8], cmds: &[ZxCommand], rep_slots: usize) -> ZxHist {
    let mut h = ZxHist::default();
    let mut reps = [1u32, 1, 1, 1];
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;
    for c in cmds.iter() {
        let is_final = c.match_off == 0;
        if c.lit_len > 0 {
            let v = (c.lit_len as usize).min(h.lit_run.len() - 1);
            h.lit_run[v] += 1;
            prev = Prev::Literals;
        }
        if is_final {
            break;
        }
        let off = c.match_off;
        let len = c.match_len;
        let rep_idx = if let Prev::Literals = prev {
            let mut found = None;
            for r in 0..rep_slots {
                if reps[r] == off {
                    found = Some(r);
                    break;
                }
            }
            found
        } else {
            None
        };
        match (&prev, rep_idx) {
            (Prev::Literals, Some(ridx)) => {
                h.rep_idx[ridx] += 1;
                let v = (len as usize).min(h.rep_len.len() - 1);
                h.rep_len[v] += 1;
                rep_mtf(&mut reps, ridx);
            }
            _ => {
                let msb = (((off - 1) >> 7) + 1) as usize;
                let mcap = h.off_msb.len() - 1;
                h.off_msb[msb.min(mcap)] += 1;
                h.off_lsb[((off - 1) & 0x7f) as usize] += 1;
                if off <= 128 {
                    h.near_off[off as usize] += 1;
                }
                let fcap = h.off_full.len() - 1;
                h.off_full[(off as usize).min(fcap)] += 1;
                let v = ((len - 1) as usize).min(h.newoff_len.len() - 1);
                h.newoff_len[v] += 1;
                rep_insert(&mut reps, off);
            }
        }
        prev = Prev::Match;
    }
    h
}

pub fn stats(_input: &[u8], cmds: &[ZxCommand], rep_slots: usize) -> ZxStats {
    let mut st = ZxStats::default();
    let mut reps = [1u32, 1, 1, 1];
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;
    let ncmds = cmds.len();
    for (ci, c) in cmds.iter().enumerate() {
        let is_final = c.match_off == 0;
        let _ = (ci, ncmds);
        if c.lit_len > 0 {
            if let Prev::Match = prev {
                st.flag_bits += 1;
            }
            st.n_lit_runs += 1;
            st.lit_bytes += c.lit_len as u64;
            st.lit_frame_bits += lit_run_bits(c.lit_len) as u64;
            prev = Prev::Literals;
        }
        if is_final {
            break;
        }
        let off = c.match_off;
        let len = c.match_len;
        let rep_idx = if let Prev::Literals = prev {
            let mut found = None;
            for r in 0..rep_slots {
                if reps[r] == off {
                    found = Some(r);
                    break;
                }
            }
            found
        } else {
            None
        };
        match (&prev, rep_idx) {
            (Prev::Literals, Some(ridx)) => {
                st.flag_bits += 1;
                if rep_slots > 1 {
                    st.rep_index_bits += rep_index_bits(ridx) as u64;
                }
                st.rep_len_bits += rep_len_bits(len) as u64;
                st.rep_counts[ridx] += 1;
                rep_mtf(&mut reps, ridx);
            }
            _ => {
                st.flag_bits += 1;
                st.n_newoff += 1;
                st.newoff_off_bits += offset_cost_bits_hc(off) as u64;
                st.newoff_len_bits += newoff_len_bits(len - 1) as u64;
                rep_insert(&mut reps, off);
            }
        }
        prev = Prev::Match;
    }
    st.total_bits = st.lit_bytes * 8
        + st.lit_frame_bits
        + st.newoff_off_bits
        + st.newoff_len_bits
        + st.rep_index_bits
        + st.rep_len_bits
        + st.flag_bits;
    st
}

/// Bit-exact predicted payload size (in bits, excluding the mode byte) of `cmds` under the given
/// modes; replays the same grammar/codes as `encode_with3`. Used to check the parser's cost model
/// against the encoder (the blob payload is `ceil(this/8)` bytes + 1 mode byte).
pub fn predicted_payload_bits(
    input: &[u8],
    cmds: &[ZxCommand],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> u64 {
    let near_rep = near_rep && rep_slots == 4;
    let am_near_rep = am_near_rep && rep_slots == 4;
    let mut reps = [1u32, 1, 1, 1];
    let mut bits: u64 = 0;
    #[derive(PartialEq)]
    enum Prev {
        Start,
        Literals,
        Match,
    }
    let mut prev = Prev::Start;
    let _ = input;
    for c in cmds {
        let is_final = c.match_off == 0;
        if c.lit_len > 0 {
            if prev == Prev::Match {
                bits += 1; // AfterMatch::Literals (`0`) - 1 bit in both modes
            }
            bits += lit_run_bits(c.lit_len) as u64 + 8 * c.lit_len as u64;
            prev = Prev::Literals;
        }
        if is_final {
            break;
        }
        let off = c.match_off;
        let len = c.match_len;
        let after_lits = prev == Prev::Literals;
        let rep_idx = if after_lits {
            (0..rep_slots).find(|&r| reps[r] == off)
        } else {
            None
        };
        if after_lits && near_rep && c.near_rep_ri >= 0 {
            let rj = c.near_rep_ri as usize;
            let base = reps[rj];
            let delta = if off > base { off - base } else { base - off };
            bits += after_lit_prefix_bits(AfterLit::NearRep(rj, off)) as u64
                + near_rep_delta_bits(delta) as u64
                + newoff_len_bits(len - 1) as u64;
            rep_insert(&mut reps, off);
        } else if !after_lits && am_near_rep && c.near_rep_ri >= 0 {
            let rj = c.near_rep_ri as usize;
            let base = reps[rj];
            let delta = if off > base { off - base } else { base - off };
            bits += after_match_prefix_bits(AfterMatch::NearRep(rj, off)) as u64
                + near_rep_delta_bits(delta) as u64
                + newoff_len_bits(len - 1) as u64;
            rep_insert(&mut reps, off);
        } else {
            match (after_lits, rep_idx) {
                (true, Some(ridx)) => {
                    if near_rep {
                        bits += after_lit_prefix_bits(AfterLit::ExactRep(ridx)) as u64;
                    } else {
                        bits += 1;
                        if rep_slots > 1 {
                            bits += rep_index_bits(ridx) as u64;
                        }
                    }
                    bits += rep_len_bits(len) as u64;
                    rep_mtf(&mut reps, ridx);
                }
                _ => {
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
                    bits += offset_cost_bits_hc(off) as u64 + newoff_len_bits(len - 1) as u64;
                    rep_insert(&mut reps, off);
                }
            }
        }
        prev = Prev::Match;
    }
    bits
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// Decoder state machine states (see `decode`).
enum St {
    Literals,
    AfterLitMatch, // read the after-literals symbol (new-offset / rep_i / near-rep_i)
    Rep(usize),
    NearRep(usize),
    NewOffset,
    AfterMatchNearRep(usize), // am_near_rep: off = reps[ri] ± δ, ri in {0,1}
}

/// Decode a ZX blob back to `orig_len` bytes. The blob's first byte encodes
/// `rep_slots | (near_rep << 4) | (am_near_rep << 5)`.
pub fn decode(blob: &[u8], orig_len: usize) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::with_capacity(orig_len);
    if orig_len == 0 {
        return out;
    }
    let mode = blob[0];
    if mode == MODE_MIN_EOF {
        return decode_min_eof(&blob[1..], orig_len);
    }
    let rep_slots = (mode & 0x0f) as usize;
    let near_rep = (mode & 0x10) != 0;
    let am_near_rep = (mode & 0x20) != 0;
    debug_assert!((1..=4).contains(&rep_slots));
    let mut r = BitReader::new(&blob[1..]);

    let mut reps = [1u32, 1, 1, 1];

    // Enum-loop state machine. `Rep`/`NewOffset` come from the after-literals symbol or the
    // after-match flag. `NearRep(ri)` (near_rep mode) copies reps[ri] ± δ. After any match the next
    // unit is literals or a new-offset (never a rep), so the rep/near-rep symbols are only read
    // immediately after a literal run. In `am_near_rep` mode the after-match decision also admits
    // `AfterMatchNearRep(ri)` (off = rep0/rep1 ± δ).
    let mut state = St::Literals;

    loop {
        match state {
            St::Literals => {
                let len = r.read_gamma();
                for _ in 0..len {
                    let b = r.read_byte();
                    out.push(b);
                    if out.len() == orig_len {
                        return out;
                    }
                }
                state = St::AfterLitMatch;
            }
            St::AfterLitMatch => {
                // Decode the after-literals symbol. Without near_rep this is the 1-bit flag
                // (1 = new-offset, 0 = rep) followed by the rep-index code.
                if near_rep {
                    match read_after_lit_prefix(&mut r) {
                        AfterLit::NewOffset => state = St::NewOffset,
                        AfterLit::ExactRep(ri) => state = St::Rep(ri),
                        AfterLit::NearRep(ri, _) => state = St::NearRep(ri),
                    }
                } else if r.read_bit() == 1 {
                    state = St::NewOffset;
                } else {
                    let ridx = if rep_slots > 1 {
                        read_rep_index(&mut r)
                    } else {
                        0
                    };
                    state = St::Rep(ridx);
                }
            }
            St::Rep(ridx) => {
                let off = reps[ridx];
                rep_mtf(&mut reps, ridx);
                let len = r.read_gamma(); // rep length: gamma
                copy_match(&mut out, off, len, orig_len);
                if out.len() >= orig_len {
                    return out;
                }
                state = read_after_match_state(&mut r, am_near_rep);
            }
            St::NearRep(ri) => {
                let off = read_near_rep_off(&mut r, reps[ri]);
                rep_insert(&mut reps, off);
                let len = r.read_gamma() + 1; // new-offset-style length (MIN 2), no backtrack
                copy_match(&mut out, off, len, orig_len);
                if out.len() >= orig_len {
                    return out;
                }
                state = read_after_match_state(&mut r, am_near_rep);
            }
            St::AfterMatchNearRep(ri) => {
                // 11 + ri-bit already consumed; read sign+gamma(δ) then gamma(len-1) (MIN 2, no backtrack).
                let off = read_near_rep_off(&mut r, reps[ri]);
                rep_insert(&mut reps, off);
                let len = r.read_gamma() + 1;
                copy_match(&mut out, off, len, orig_len);
                if out.len() >= orig_len {
                    return out;
                }
                state = read_after_match_state(&mut r, am_near_rep);
            }
            St::NewOffset => {
                let msb = r.read_gamma(); // offset MSB via interlaced Elias gamma
                                          // No EOF check: termination is driven by orig_len (checked after each copy), so
                                          // msb may be large, allowing offsets up to the full 64 KB window.
                let lsb = (r.read_byte() >> 1) as u32;
                let off = ((msb - 1) << 7) | lsb;
                let off = off + 1;
                rep_insert_n(&mut reps, off, rep_slots);
                r.set_backtrack();
                let len = r.read_gamma() + 1;
                copy_match(&mut out, off, len, orig_len);
                if out.len() >= orig_len {
                    return out;
                }
                state = read_after_match_state(&mut r, am_near_rep);
            }
        }
    }
}

/// Read the after-match connecting decision and return the next decoder state. Classic mode: a
/// single bit (1 = new-offset, 0 = literals). `am_near_rep` mode: `0`=literals, `10`=new-offset,
/// `11`+ri-bit = after-match near-rep off rep0/rep1.
#[inline]
fn read_after_match_state(r: &mut BitReader, am_near_rep: bool) -> St {
    if !am_near_rep {
        return if r.read_bit() == 1 {
            St::NewOffset
        } else {
            St::Literals
        };
    }
    if r.read_bit() == 0 {
        St::Literals // 0
    } else if r.read_bit() == 0 {
        St::NewOffset // 10
    } else {
        let ri = r.read_bit() as usize; // 11 + ri-bit (0 => rep0, 1 => rep1)
        St::AfterMatchNearRep(ri)
    }
}

#[inline]
fn rep_insert_n(reps: &mut [u32; 4], off: u32, _rep_slots: usize) {
    // Insert is identical regardless of rep_slots: the full 4-wide window always shifts; the tail
    // slots are never selected when rep_slots < 4 (the encoder never emits a rep index >= rep_slots).
    rep_insert(reps, off);
}

#[inline]
fn copy_match(out: &mut Vec<u8>, off: u32, len: u32, orig_len: usize) {
    let off = off as usize;
    let mut remaining = len as usize;
    while remaining > 0 && out.len() < orig_len {
        let src = out.len() - off;
        let b = out[src];
        out.push(b);
        remaining -= 1;
    }
}

// ===========================================================================
// BACKWARD (reverse / in-place) variant - for C64 in-place decompression.
// ===========================================================================
//
// A stream that decompresses backward in memory: the unpacker reads the compressed bytes from the
// end toward the start and writes the output from its end toward the start. The compressed and
// decompressed buffers then overlap and are consumed from opposite ends, so the pack can be done in
// place (as zx0 -b / lzsa -b / exomizer -b do).
//
// ## Mechanism
//
// The grammar, parse, gamma codes, rep0-3 + near-rep extensions, offset split and backtrack trick
// are all reused unchanged. The backward variant is purely a transform of input order + stream byte
// order:
//
//   ENCODE  (`encode_with3_backward` / `compress_backward`):
//     1. Reverse the input: ri[k] = input[n-1-k].
//     2. Run the forward parse + `encode_with3` on `ri` -> a forward blob F = [mode_byte][payload].
//        (Reversing the data flips every match offset; the parser is identical, so the command
//        stream / ratio match the forward variant on reversed data.)
//     3. The backward blob is [mode_byte] ++ reverse(payload). The mode byte stays at the front;
//        the payload bytes are reversed so a reader walking from the end sees forward-of-F order.
//
//   DECODE  (`decode_backward`):
//     - A `BackwardBitReader` walks the reversed payload from its end toward the start. Each byte it
//       fetches is payload[hi-1], payload[hi-2], ... == F.payload[0], [1], ... - the byte sequence a
//       forward `BitReader` would see. Bits are consumed MSB-first, and the backtrack trick reads
//       bit 0 of the most-recent byte, identical to the forward reader. So the same state machine
//       produces the same byte sequence the forward decoder would on F: `ri` (reversed input).
//     - Produced byte k is placed at output index (orig_len-1-k), written from the end backward, so
//       the returned Vec is the original `input`. Match copies reference `out[cur-off]` in the
//       reversed-output coordinate space, mirroring the forward copy.
//
// The only differences from the forward path are input reversal and payload byte reversal, so the
// backward size equals the forward size on the reversed data, and the forward path is untouched.

/// MSB-first bit reader that consumes the buffer from the end toward the start. Walking the
/// byte-reversed payload this way reproduces, byte-for-byte and bit-for-bit, the read sequence a
/// forward `BitReader` performs on the original payload, including the backtrack trick (bit 0 of
/// the most-recently fetched byte). Lets the backward decoder run the same state machine, fed bytes
/// from the opposite end.
pub struct BackwardBitReader<'a> {
    data: &'a [u8],
    pos: usize, // index ONE PAST the next byte to fetch; we fetch data[pos-1] then decrement
    bit_mask: u8,
    bit_value: u8,
    backtrack: bool,
    last_byte: u8,
}

impl<'a> BackwardBitReader<'a> {
    /// Start at the end of `data` (the high address); reads walk toward index 0.
    pub fn new(data: &'a [u8]) -> Self {
        BackwardBitReader {
            data,
            pos: data.len(),
            bit_mask: 0,
            bit_value: 0,
            backtrack: false,
            last_byte: 0,
        }
    }

    #[inline]
    pub fn read_byte(&mut self) -> u8 {
        let b = if self.pos > 0 {
            self.data[self.pos - 1]
        } else {
            0
        };
        // Saturating decrement: past the start, keep returning 0 (mirrors the forward reader past
        // the end). Only reached on never-read trailing padding.
        if self.pos > 0 {
            self.pos -= 1;
        }
        self.last_byte = b;
        b
    }

    #[inline]
    pub fn set_backtrack(&mut self) {
        self.backtrack = true;
    }

    #[inline]
    pub fn read_bit(&mut self) -> u32 {
        if self.backtrack {
            self.backtrack = false;
            return (self.last_byte & 1) as u32;
        }
        self.bit_mask >>= 1;
        if self.bit_mask == 0 {
            self.bit_mask = 128;
            self.bit_value = self.read_byte();
        }
        if self.bit_value & self.bit_mask != 0 {
            1
        } else {
            0
        }
    }

    #[inline]
    pub fn read_gamma(&mut self) -> u32 {
        let mut value = 1u32;
        while self.read_bit() == 0 {
            value = (value << 1) | self.read_bit();
        }
        value
    }
}

/// Backward-stream rep-index reader (mirrors `read_rep_index`, but on a `BackwardBitReader`).
#[inline]
fn read_rep_index_b(r: &mut BackwardBitReader) -> usize {
    if r.read_bit() == 1 {
        0
    } else if r.read_bit() == 1 {
        1
    } else if r.read_bit() == 1 {
        2
    } else {
        3
    }
}

/// Backward-stream after-literals prefix reader (mirrors `read_after_lit_prefix`).
#[inline]
fn read_after_lit_prefix_b(r: &mut BackwardBitReader) -> AfterLit {
    if r.read_bit() == 1 {
        return AfterLit::NewOffset;
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(0);
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(1);
    }
    if r.read_bit() == 1 {
        return AfterLit::NearRep(0, 0);
    }
    if r.read_bit() == 1 {
        return AfterLit::ExactRep(3);
    }
    if r.read_bit() == 1 {
        AfterLit::ExactRep(2)
    } else {
        AfterLit::NearRep(1, 0)
    }
}

/// Backward-stream near-rep offset reader (mirrors `read_near_rep_off`).
#[inline]
fn read_near_rep_off_b(r: &mut BackwardBitReader, base: u32) -> u32 {
    let sign = r.read_bit();
    let delta = r.read_gamma();
    if sign == 0 {
        base + delta
    } else {
        base - delta
    }
}

/// Encode the backward blob from a parsed command stream over the reversed input. The caller
/// reverses and parses the input; this reuses the forward `encode_with3`, then reverses the payload
/// bytes (keeping the mode byte at the front). Backward and forward share the same emitter, so the
/// ratio is identical on reversed data.
pub fn encode_with3_backward(
    rev_input: &[u8],
    rev_cmds: &[ZxCommand],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
) -> Vec<u8> {
    let fwd = encode_with3(rev_input, rev_cmds, rep_slots, near_rep, am_near_rep);
    // fwd = [mode_byte][payload]; backward = [mode_byte] ++ reverse(payload).
    let mut out = Vec::with_capacity(fwd.len());
    out.push(fwd[0]);
    out.extend(fwd[1..].iter().rev().copied());
    out
}

/// High-level backward compress: reverse the input, run the same parse + emission, then reverse the
/// payload bytes. `rep_slots`/`near_rep`/`am_near_rep`/`effort` select the grammar + parse tier like
/// `compress3_e`. The result decodes with `decode_backward`.
pub fn compress_backward(
    input: &[u8],
    rep_slots: usize,
    near_rep: bool,
    am_near_rep: bool,
    effort: u8,
) -> Vec<u8> {
    use crate::parse;
    assert!((1..=4).contains(&rep_slots));
    let near_rep = near_rep && rep_slots == 4;
    let am_near_rep = am_near_rep && rep_slots == 4;
    if input.is_empty() {
        return encode_with3_backward(input, &[], rep_slots, near_rep, am_near_rep);
    }
    // Reverse the input; the forward pipeline runs on the reversed buffer.
    let mut rev: Vec<u8> = input.to_vec();
    rev.reverse();

    let window = (MAX_OFFSET as usize).min(rev.len());
    let ms = build_zx_matches(&rev, window);
    let cmds = match effort {
        1 => parse::parse_zx3_fast(&rev, &ms, rep_slots, near_rep, am_near_rep),
        2 => parse::parse_zx3(&rev, &ms, rep_slots, near_rep, am_near_rep),
        _ => match build_complete_extra(&rev, window) {
            Some(extra) => {
                parse::parse_zx3_complete(&rev, &ms, &extra, rep_slots, near_rep, am_near_rep)
            }
            None => parse::parse_zx3(&rev, &ms, rep_slots, near_rep, am_near_rep),
        },
    };
    encode_with3_backward(&rev, &cmds, rep_slots, near_rep, am_near_rep)
}

/// Backward best-of: run the same five grammar variants the forward `zx_best_of` does, through the
/// backward pipeline, and keep the smallest. Returns the backward blob (decodable by
/// `decode_backward`). `effort` is threaded through every variant.
pub fn compress_backward_best_of(input: &[u8], effort: u8) -> Vec<u8> {
    let v0 = compress_backward(input, 1, false, false, effort);
    let v1 = compress_backward(input, 4, false, false, effort);
    let v2 = compress_backward(input, 4, true, false, effort);
    let v3 = compress_backward(input, 4, false, true, effort);
    let v4 = compress_backward(input, 4, true, true, effort);
    [v0, v1, v2, v3, v4]
        .into_iter()
        .min_by_key(|b| b.len())
        .unwrap()
}

/// Decode a backward blob back to `orig_len` bytes. The blob is `[mode_byte] ++ reverse(payload)`
/// where `payload` is a forward payload of the reversed input. Walk the reversed payload from its
/// end with a `BackwardBitReader` (reproducing the forward read sequence) and run the same state
/// machine as `decode`, writing each produced byte from the output end backward. The returned Vec is
/// the original input.
///
/// Models the C64 in-place unpacker: it consumes compressed bytes from high address to low and
/// writes plaintext from high address to low, so the two buffers can overlap and be packed in place.
pub fn decode_backward(blob: &[u8], orig_len: usize) -> Vec<u8> {
    let mut out: Vec<u8> = vec![0u8; orig_len];
    if orig_len == 0 {
        return out;
    }
    let mode = blob[0];
    let rep_slots = (mode & 0x0f) as usize;
    let near_rep = (mode & 0x10) != 0;
    let am_near_rep = (mode & 0x20) != 0;
    debug_assert!((1..=4).contains(&rep_slots));
    let mut r = BackwardBitReader::new(&blob[1..]);

    let mut reps = [1u32, 1, 1, 1];

    // `cur` = number of output bytes produced so far (in reversed-input coordinates). The byte
    // produced at logical position `cur` is stored at the mirrored output index orig_len-1-cur, so
    // `out` reads as the original input. Match copies reference `cur-off` in the same reversed
    // coordinate space the forward decoder uses, so the copy logic is identical.
    let mut cur: usize = 0;

    // Reversed-coordinate push: place produced byte at out[orig_len-1-cur].
    macro_rules! push_b {
        ($b:expr) => {{
            out[orig_len - 1 - cur] = $b;
            cur += 1;
        }};
    }

    let mut state = St::Literals;
    loop {
        match state {
            St::Literals => {
                let len = r.read_gamma();
                for _ in 0..len {
                    let b = r.read_byte();
                    push_b!(b);
                    if cur == orig_len {
                        return out;
                    }
                }
                state = St::AfterLitMatch;
            }
            St::AfterLitMatch => {
                if near_rep {
                    match read_after_lit_prefix_b(&mut r) {
                        AfterLit::NewOffset => state = St::NewOffset,
                        AfterLit::ExactRep(ri) => state = St::Rep(ri),
                        AfterLit::NearRep(ri, _) => state = St::NearRep(ri),
                    }
                } else if r.read_bit() == 1 {
                    state = St::NewOffset;
                } else {
                    let ridx = if rep_slots > 1 {
                        read_rep_index_b(&mut r)
                    } else {
                        0
                    };
                    state = St::Rep(ridx);
                }
            }
            St::Rep(ridx) => {
                let off = reps[ridx];
                rep_mtf(&mut reps, ridx);
                let len = r.read_gamma();
                copy_match_b(&mut out, orig_len, &mut cur, off, len);
                if cur >= orig_len {
                    return out;
                }
                state = read_after_match_state_b(&mut r, am_near_rep);
            }
            St::NearRep(ri) => {
                let off = read_near_rep_off_b(&mut r, reps[ri]);
                rep_insert(&mut reps, off);
                let len = r.read_gamma() + 1;
                copy_match_b(&mut out, orig_len, &mut cur, off, len);
                if cur >= orig_len {
                    return out;
                }
                state = read_after_match_state_b(&mut r, am_near_rep);
            }
            St::AfterMatchNearRep(ri) => {
                let off = read_near_rep_off_b(&mut r, reps[ri]);
                rep_insert(&mut reps, off);
                let len = r.read_gamma() + 1;
                copy_match_b(&mut out, orig_len, &mut cur, off, len);
                if cur >= orig_len {
                    return out;
                }
                state = read_after_match_state_b(&mut r, am_near_rep);
            }
            St::NewOffset => {
                let msb = r.read_gamma();
                let lsb = (r.read_byte() >> 1) as u32;
                let off = ((msb - 1) << 7) | lsb;
                let off = off + 1;
                rep_insert(&mut reps, off);
                r.set_backtrack();
                let len = r.read_gamma() + 1;
                copy_match_b(&mut out, orig_len, &mut cur, off, len);
                if cur >= orig_len {
                    return out;
                }
                state = read_after_match_state_b(&mut r, am_near_rep);
            }
        }
    }
}

/// Backward-stream after-match decision (mirrors `read_after_match_state`).
#[inline]
fn read_after_match_state_b(r: &mut BackwardBitReader, am_near_rep: bool) -> St {
    if !am_near_rep {
        return if r.read_bit() == 1 {
            St::NewOffset
        } else {
            St::Literals
        };
    }
    if r.read_bit() == 0 {
        St::Literals
    } else if r.read_bit() == 0 {
        St::NewOffset
    } else {
        let ri = r.read_bit() as usize;
        St::AfterMatchNearRep(ri)
    }
}

/// Backward match copy in reversed-output coordinates. `cur` is the produced-byte count; places
/// bytes at out[orig_len-1-cur] and sources from out[orig_len-1-(cur-off)] (the byte produced `off`
/// steps ago), mirroring the forward `copy_match`'s out[len-off] reference.
#[inline]
fn copy_match_b(out: &mut [u8], orig_len: usize, cur: &mut usize, off: u32, len: u32) {
    let off = off as usize;
    let mut remaining = len as usize;
    while remaining > 0 && *cur < orig_len {
        let src_logical = *cur - off; // byte produced `off` steps ago (forward: out[len-off])
        let b = out[orig_len - 1 - src_logical];
        out[orig_len - 1 - *cur] = b;
        *cur += 1;
        remaining -= 1;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Trivial greedy parser to produce a representable command stream for tests (the real
    /// parser lives in parse.rs; here we just need *some* valid command list to roundtrip).
    fn greedy_cmds(input: &[u8], rep_slots: usize) -> Vec<ZxCommand> {
        let n = input.len();
        let mut cmds = Vec::new();
        let mut i = 0usize;
        let mut lit_start = 0usize;
        let mut lit_len = 0u32;
        let mut reps = [1u32, 1, 1, 1];
        let mut prev_was_match = false;
        // first unit must be literals; force at least position 0 to be a literal
        while i < n {
            // try to find a match (naive O(n^2)-ish but capped) at i, distance <= MAX_OFFSET
            let mut best_len = 0u32;
            let mut best_off = 0u32;
            let max_back = i.min(MAX_OFFSET as usize);
            // rep candidates first (only if we'll have a preceding literal OR prev_was_match)
            for d in 1..=max_back {
                let off = d as u32;
                let src = i - d;
                let mut l = 0usize;
                while i + l < n && input[src + l] == input[i + l] && l < 4000 {
                    l += 1;
                }
                if (l as u32) > best_len {
                    best_len = l as u32;
                    best_off = off;
                }
            }
            // Decide: a match needs min length 1; but for the grammar we need the first unit to
            // be literals. We require: a match must be preceded by a literal run unless the
            // previous unit was a match. Also reps only legal after literals.
            let can_match = best_len >= 1 && (i > 0);
            // require a leading literal for the very first command
            if can_match && (lit_len > 0 || prev_was_match) && best_len >= 2 {
                // flush as a command [lit_len][match]
                // update reps
                let off = best_off;
                // check rep legality handled by encoder; just push command
                cmds.push(ZxCommand {
                    lit_len,
                    lit_start,
                    match_off: off,
                    match_len: best_len,
                    near_rep_ri: -1,
                });
                // update reps for next decision (mirror encoder, approximately)
                let mut hit = None;
                if lit_len > 0 {
                    for r in 0..rep_slots {
                        if reps[r] == off {
                            hit = Some(r);
                            break;
                        }
                    }
                }
                match hit {
                    Some(ri) => rep_mtf(&mut reps, ri),
                    None => rep_insert(&mut reps, off),
                }
                i += best_len as usize;
                lit_start = i;
                lit_len = 0;
                prev_was_match = true;
            } else {
                // literal
                if lit_len == 0 {
                    lit_start = i;
                }
                lit_len += 1;
                i += 1;
                prev_was_match = false;
            }
        }
        // trailing literals -> final literal-only command
        if lit_len > 0 {
            cmds.push(ZxCommand {
                lit_len,
                lit_start,
                match_off: 0,
                match_len: 0,
                near_rep_ri: -1,
            });
        } else if cmds.is_empty() {
            // empty input
        } else {
            // last command ended on a match; orig_len-driven termination handles it.
        }
        cmds
    }

    fn rt(input: &[u8], rep_slots: usize) {
        let cmds = greedy_cmds(input, rep_slots);
        let blob = encode_with(input, &cmds, rep_slots);
        let out = decode(&blob, input.len());
        assert_eq!(
            out,
            input,
            "roundtrip mismatch len {} rep_slots {}",
            input.len(),
            rep_slots
        );
    }

    #[test]
    fn gamma_bits_matches_writer() {
        for v in 1..2000u32 {
            let mut w = BitWriter::new();
            // Encode just this gamma, then re-read it.
            w.write_gamma(v);
            let blob = w.finish();
            let mut r = BitReader::new(&blob);
            assert_eq!(r.read_gamma(), v, "gamma roundtrip {}", v);
        }
    }

    #[test]
    fn gamma_bit_count() {
        // value -> expected bits
        let cases = [
            (1u32, 1u32),
            (2, 3),
            (3, 3),
            (4, 5),
            (7, 5),
            (8, 7),
            (15, 7),
            (16, 9),
        ];
        for (v, b) in cases {
            assert_eq!(gamma_bits(v), b, "gamma_bits({})", v);
        }
    }

    #[test]
    fn roundtrip_empty_and_tiny() {
        for slots in [1usize, 2, 3, 4] {
            for n in 0..40usize {
                let v: Vec<u8> = (0..n).map(|i| (i * 37 % 101) as u8).collect();
                rt(&v, slots);
            }
        }
    }

    #[test]
    fn roundtrip_repetitive() {
        for slots in [1usize, 4] {
            let v: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
            rt(&v, slots);
        }
    }

    #[test]
    fn roundtrip_single_byte() {
        for slots in [1usize, 4] {
            rt(&[0xABu8; 3000], slots);
        }
    }

    #[test]
    fn roundtrip_text() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..300 {
            data.extend_from_slice(base);
        }
        for slots in [1usize, 4] {
            rt(&data, slots);
        }
    }

    #[test]
    fn roundtrip_random() {
        let mut state = 7u32;
        let data: Vec<u8> = (0..20000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                (state >> 24) as u8
            })
            .collect();
        for slots in [1usize, 4] {
            rt(&data, slots);
        }
    }

    /// Roundtrip the real pipeline (match-find + parse_zx + encode + decode) on diverse and
    /// adversarial inputs, for both rep0-only and rep0-3. Exercises length-1 reps, the rep-index
    /// code, slot-0/offset edges, and overlap copies.
    #[test]
    fn roundtrip_real_pipeline() {
        let mut state: u64 = 0xDEADBEEF12345678;
        let mut rng = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        // deterministic corner cases
        let mut cases: Vec<Vec<u8>> = Vec::new();
        for n in 0..16usize {
            cases.push((0..n).map(|i| i as u8).collect());
            cases.push(vec![(n & 0xff) as u8; n]);
        }
        for &period in &[1usize, 128, 257] {
            cases.push((0..128).map(|i| (i % period) as u8).collect());
        }
        for &off in &[1usize, 127, 128, 129, 256] {
            let mut v: Vec<u8> = (0..off).map(|i| (i * 131 % 251) as u8).collect();
            let pre = v.clone();
            for k in 0..128 {
                v.push(pre[k % pre.len().max(1)]);
            }
            cases.push(v);
        }
        // random structured
        for _ in 0..4 {
            let n = (rng() % 128) as usize + 1;
            let mode = rng() % 4;
            let mut v = Vec::with_capacity(n);
            match mode {
                0 => {
                    let alpha = (rng() % 6) as u8 + 1;
                    for _ in 0..n {
                        v.push((rng() % alpha as u64) as u8);
                    }
                }
                1 => {
                    for _ in 0..n {
                        v.push((rng() >> 24) as u8);
                    }
                }
                2 => {
                    while v.len() < n {
                        if v.is_empty() || rng() % 2 == 0 {
                            let run = (rng() % 40) as usize + 1;
                            let b = (rng() >> 16) as u8;
                            for _ in 0..run {
                                v.push(b);
                            }
                        } else {
                            let back = 1 + (rng() as usize % v.len());
                            let len = (rng() % 30) as usize + 1;
                            let src = v.len() - back;
                            for k in 0..len {
                                v.push(v[src + (k % back)]);
                            }
                        }
                    }
                    v.truncate(n);
                }
                _ => {
                    let words: [&[u8]; 3] = [b"the ", b"quick ", b"fox "];
                    while v.len() < n {
                        v.extend_from_slice(words[(rng() % 3) as usize]);
                    }
                    v.truncate(n);
                }
            }
            cases.push(v);
        }
        for data in &cases {
            for slots in [1usize, 4] {
                let blob = compress(data, slots);
                let out = decode(&blob, data.len());
                assert_eq!(
                    &out,
                    data,
                    "real-pipeline roundtrip len {} slots {}",
                    data.len(),
                    slots
                );
            }
            // Near-rep path (rep0-3 + offset-delta coding) must also roundtrip exactly.
            let blob_nr = compress2(data, 4, true);
            let out_nr = decode(&blob_nr, data.len());
            assert_eq!(&out_nr, data, "near-rep roundtrip len {}", data.len());
            // After-match near-rep, both am-only and am + after-lit near-rep.
            for &(nr, am) in &[(false, true), (true, true)] {
                let blob_am = compress3(data, 4, nr, am);
                let out_am = decode(&blob_am, data.len());
                assert_eq!(
                    &out_am,
                    data,
                    "am-near-rep roundtrip len {} nr={} am={}",
                    data.len(),
                    nr,
                    am
                );
            }
        }
    }

    #[test]
    fn roundtrip_structured() {
        let mut data = Vec::new();
        for blk in 0..200u32 {
            for i in 0..80u32 {
                data.push(((blk.wrapping_mul(7) + i) & 0xff) as u8);
            }
            data.extend_from_slice(&[0u8; 40]);
        }
        for slots in [1usize, 4] {
            rt(&data, slots);
        }
    }

    /// Backward roundtrip on the same diverse + adversarial inputs as `roundtrip_real_pipeline`,
    /// across all grammar variants. `decode_backward(compress_backward(x)) == x`.
    #[test]
    fn roundtrip_backward_pipeline() {
        let mut state: u64 = 0x0BADC0DE_F00DBABE;
        let mut rng = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut cases: Vec<Vec<u8>> = Vec::new();
        // edge cases
        cases.push(Vec::new()); // empty
        cases.push(vec![0x42]); // 1 byte
        for n in 0..16usize {
            cases.push((0..n).map(|i| i as u8).collect());
            cases.push(vec![(n & 0xff) as u8; n]); // repetitive
        }
        for &period in &[1usize, 128, 257] {
            cases.push((0..128).map(|i| (i % period) as u8).collect());
        }
        for &off in &[1usize, 127, 128, 129, 256] {
            let mut v: Vec<u8> = (0..off).map(|i| (i * 131 % 251) as u8).collect();
            let pre = v.clone();
            for k in 0..128 {
                v.push(pre[k % pre.len().max(1)]);
            }
            cases.push(v);
        }
        // incompressible
        for _ in 0..4 {
            let n = (rng() % 128) as usize + 1;
            cases.push((0..n).map(|_| (rng() >> 24) as u8).collect());
        }
        // structured random
        for _ in 0..4 {
            let n = (rng() % 128) as usize + 1;
            let mut v = Vec::with_capacity(n);
            while v.len() < n {
                if v.is_empty() || rng() % 3 == 0 {
                    let run = (rng() % 40) as usize + 1;
                    let b = (rng() >> 16) as u8;
                    for _ in 0..run {
                        v.push(b);
                    }
                } else {
                    let back = 1 + (rng() as usize % v.len());
                    let len = (rng() % 30) as usize + 1;
                    let src = v.len() - back;
                    for k in 0..len {
                        v.push(v[src + (k % back)]);
                    }
                }
            }
            v.truncate(n);
            cases.push(v);
        }

        let variants: [(usize, bool, bool); 5] = [
            (1, false, false),
            (4, false, false),
            (4, true, false),
            (4, false, true),
            (4, true, true),
        ];
        for data in &cases {
            for &(slots, nr, am) in &variants {
                let blob = compress_backward(data, slots, nr, am, 3);
                let out = decode_backward(&blob, data.len());
                assert_eq!(
                    &out,
                    data,
                    "backward roundtrip len {} slots {} nr {} am {}",
                    data.len(),
                    slots,
                    nr,
                    am
                );
            }
            // best-of backward
            let bob = compress_backward_best_of(data, 3);
            assert_eq!(
                &decode_backward(&bob, data.len()),
                data,
                "backward best-of roundtrip len {}",
                data.len()
            );
        }
    }

    /// The backward blob must be the same size as the forward blob on the reversed input (same
    /// grammar and parse, only input/byte order differs). Compared against `compress3_e` on the
    /// reversed input for each variant.
    #[test]
    fn backward_size_equals_forward_on_reversed() {
        let mut data = Vec::new();
        let base = b"the quick brown fox jumps over the lazy dog. abracadabra. ";
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        let mut rev = data.clone();
        rev.reverse();
        for &(slots, nr, am) in &[(1usize, false, false), (4, false, false), (4, true, true)] {
            let bwd = compress_backward(&data, slots, nr, am, 3);
            let fwd_on_rev = compress3_e(&rev, slots, nr, am, 3);
            assert_eq!(
                bwd.len(),
                fwd_on_rev.len(),
                "backward size must equal forward-on-reversed size (slots {} nr {} am {})",
                slots,
                nr,
                am
            );
        }
    }
}
