//! ZX02: a ZX0 optimal LZ variant tuned for a 6502 decoder (no hardware multiply, 8-bit
//! arithmetic). See THIRD_PARTY.md for attribution and license.
//!
//! ZX02 reuses ZX0's optimal parser but changes the cost model and bitstream so the decoder needs
//! no hardware multiply and only 8-bit arithmetic:
//!   - Elias-gamma values are capped at 256 (`0x100`); a match/offset length of 256 wraps to 0.
//!   - Single-byte matches are allowed (match length encoded as `gamma(len-1)`, so `len==1`
//!     costs `gamma(0)` which `elias_gamma_bits_1` maps to `gamma(256)`).
//!   - Offsets are stored "positive minus one": MSB as `gamma((off-1)/128+1)`, LSB as a raw byte
//!     `((off-1)%128)<<1` whose low bit is recycled as the next stream bit (the "extra bit").
//!   - The stream ends with an Elias-gamma 256 (a 0-bit terminated end marker), no separate EOF.
//!
//! This is the default ZX02 configuration only (`elias_short_code = 0`, `elias_ending_bit = 0`,
//! `zx1_mode = 0`, `initial_offset = 1`, `skip = 0`, `offset_limit = MAX_OFFSET_ZX02`), which is
//! what `zx02` / `zx02 -b` emit. Backward mode reverses the input before parsing and reverses the
//! output afterwards, exactly like the reference (`-b`).
//!
//! Uniform API (mirrors `zx0compat`):
//!   - [`MAX_LEVEL`] = 1 (single optimal algorithm; `level` is ignored)
//!   - [`compress`] / [`decompress`] take a `backward: bool`
//!   - [`compress_zx02`] / [`compress_zx02_backward`] are the direct entry points

/// ZX02's default offset limit (`MAX_OFFSET_ZX02`).
pub const MAX_OFFSET_ZX02: usize = 32640;

/// ZX02's `INITIAL_OFFSET` (first match defaults to offset 1).
const INITIAL_OFFSET: i64 = 1;

/// Single optimal algorithm; `level` is ignored.
pub const MAX_LEVEL: u8 = 1;

// ---------------------------------------------------------------------------------------------
// Cost model
// ---------------------------------------------------------------------------------------------

/// Elias-gamma bit cost (default config: `elias_short_code = 0`).
/// Returns `1<<20` (a "really big number") for out-of-range values so the parser never picks them.
#[inline]
fn elias_gamma_bits(value: i64) -> i64 {
    if value < 1 || value > 0x100 {
        return 1 << 20;
    }
    let mut bits = 1i64;
    let mut v = value;
    v >>= 1;
    while v != 0 {
        bits += 2;
        v >>= 1;
    }
    bits
}

/// Elias-gamma bit cost of `value-1`. Encodes `value-1` with the 8-bit-cap convention:
/// `value == 1` maps to `gamma(256)` (the wrap), `value > 256` is rejected.
#[inline]
fn elias_gamma_bits_1(value: i64) -> i64 {
    if value == 1 {
        elias_gamma_bits(256)
    } else if value > 256 {
        1 << 20
    } else {
        elias_gamma_bits(value - 1)
    }
}

/// Offset field bit cost (default config: `zx1_mode = 0`).
#[inline]
fn offset_bits(value: i64) -> i64 {
    8 + elias_gamma_bits(value / 128 + 1)
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

// ---------------------------------------------------------------------------------------------
// Optimal parser with a refcounted block pool
// ---------------------------------------------------------------------------------------------

/// One DP node, the analogue of the reference `BLOCK`. The chain link is an arena index.
#[derive(Clone, Copy)]
struct Node {
    bits: i32,
    index: i32, // last input position consumed by this block (0-based); fake start = skip-1
    offset: i32, // 0 => literals, else the match offset
    chain: u32, // parent node id, or NIL
    references: u32,
    ghost_chain: u32,
}

const NIL: u32 = u32::MAX;

/// Refcounting node pool: a `ghost_root` recycler fused with `allocate` /
/// `assign`. Pointers are arena indices (`u32`, `NIL` == NULL). High-water mark is bounded by the
/// active frontier O(n + window).
struct Pool {
    nodes: Vec<Node>,
    ghost_root: u32,
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

    /// Allocate a pool node (`references == 0`).
    #[inline]
    fn allocate(&mut self, bits: i64, index: i64, offset: i64, chain: u32) -> u32 {
        let ptr;
        if self.ghost_root != NIL {
            ptr = self.ghost_root;
            self.ghost_root = self.nodes[ptr as usize].ghost_chain;
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

    /// Assign `chain` into a slot, updating refcounts.
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

/// The optimal parser. Returns the head of the optimal block chain (`optimal[n-1]`),
/// in forward (un-reversed) order as a `Vec<Node>` snapshot, plus that head's `bits`.
///
/// `skip` is the reference `skip` (0 for whole-file compression).
fn optimize(input: &[u8], skip: usize, offset_limit: usize) -> (Vec<OptBlock>, i64) {
    let n = input.len();
    debug_assert!(n > 0);
    let input_size = n as i64;
    let skip_i = skip as i64;
    let offset_limit = offset_limit as i64;

    // initial_offset clamp: `if (initial_offset >= input_size)`.
    // With INITIAL_OFFSET == 1 this only triggers for input_size == 1, where it becomes 0.
    let initial_offset = if INITIAL_OFFSET >= input_size {
        input_size - 1
    } else {
        INITIAL_OFFSET
    };

    let mut max_offset = offset_ceiling(input_size - 1, offset_limit);
    let mo = max_offset.max(initial_offset) as usize;

    let mut pool = Pool::with_capacity(n + 2 * (mo + 1) + 16);
    let mut last_literal = vec![NIL; mo + 1];
    let mut last_match = vec![NIL; mo + 1];
    let mut optimal = vec![NIL; n];
    let mut match_length = vec![0i64; mo + 1];
    let mut best_length = vec![0i64; n];
    if input_size > 1 {
        best_length[1] = 1;
    }

    // start with the fake block: assign(&last_match[initial_offset], allocate(-1, skip-1, initial_offset, NULL))
    {
        let fake = pool.allocate(-1, skip_i - 1, initial_offset, NIL);
        pool.assign(&mut last_match[initial_offset as usize], fake);
    }

    let mut best_length_size: i64;
    let mut index = skip_i;
    while index < input_size {
        best_length_size = 1;
        max_offset = offset_ceiling(index, offset_limit);
        let iu = index as usize;
        let mut offset = 1i64;
        while offset <= max_offset {
            let ou = offset as usize;
            if index != skip_i && index >= offset && input[iu] == input[(index - offset) as usize] {
                // copy from last offset, only if code at this offset was a literal
                if last_literal[ou] != NIL {
                    let ll = last_literal[ou];
                    let length = index - pool.index(ll);
                    let bits = pool.bits(ll) + 1 + elias_gamma_bits(length);
                    let node = pool.allocate(bits, index, offset, ll);
                    pool.assign(&mut last_match[ou], node);
                    if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
                        let lm = last_match[ou];
                        pool.assign(&mut optimal[iu], lm);
                    }
                }
                // copy from new offset
                match_length[ou] += 1;
                if best_length_size < match_length[ou] {
                    let bl = best_length[best_length_size as usize];
                    let mut bits =
                        pool.bits(optimal[(index - bl) as usize]) + elias_gamma_bits_1(bl);
                    loop {
                        best_length_size += 1;
                        let bits2 = pool.bits(optimal[(index - best_length_size) as usize])
                            + elias_gamma_bits_1(best_length_size);
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
                    + offset_bits(offset)
                    + elias_gamma_bits_1(length);
                let lm = last_match[ou];
                if lm == NIL || pool.index(lm) != index || pool.bits(lm) > bits {
                    let chain = optimal[(index - length) as usize];
                    let node = pool.allocate(bits, index, offset, chain);
                    pool.assign(&mut last_match[ou], node);
                    if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
                        let lm2 = last_match[ou];
                        pool.assign(&mut optimal[iu], lm2);
                    }
                }
            } else {
                // copy literals
                match_length[ou] = 0;
                if last_match[ou] != NIL {
                    let lm = last_match[ou];
                    let length = index - pool.index(lm);
                    let bits = pool.bits(lm) + 1 + elias_gamma_bits(length) + length * 8;
                    let node = pool.allocate(bits, index, 0, lm);
                    pool.assign(&mut last_literal[ou], node);
                    if optimal[iu] == NIL || pool.bits(optimal[iu]) > bits {
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

    // Snapshot the chain (child->parent) into a forward-order Vec.
    let mut chain: Vec<OptBlock> = Vec::new();
    let mut cur = head;
    while cur != NIL {
        let nd = &pool.nodes[cur as usize];
        chain.push(OptBlock {
            index: nd.index as i64,
            offset: nd.offset as i64,
        });
        cur = nd.chain;
    }
    chain.reverse();
    (chain, head_bits)
}

/// A snapshot of an optimal block in forward order (the forward walk order).
struct OptBlock {
    index: i64,
    offset: i64,
}

// ---------------------------------------------------------------------------------------------
// Bitstream writer
// ---------------------------------------------------------------------------------------------

struct Writer<'a> {
    out: Vec<u8>,
    input: &'a [u8],
    input_index: usize,
    bit_index: usize,
    bit_mask: u32,
    backtrack: bool,
}

impl<'a> Writer<'a> {
    fn new(input: &'a [u8], skip: usize) -> Self {
        Writer {
            out: Vec::new(),
            input,
            input_index: skip,
            bit_index: 0,
            bit_mask: 0,
            // backtrack initializes to TRUE.
            backtrack: true,
        }
    }

    #[inline]
    fn write_byte(&mut self, value: u8) {
        self.out.push(value);
    }

    /// Write one bit to the stream.
    #[inline]
    fn write_bit(&mut self, value: bool) {
        if self.backtrack {
            if value {
                let last = self.out.len() - 1;
                self.out[last] |= 1;
            }
            self.backtrack = false;
        } else {
            if self.bit_mask == 0 {
                self.bit_mask = 128;
                self.bit_index = self.out.len();
                self.out.push(0);
            }
            if value {
                self.out[self.bit_index] |= self.bit_mask as u8;
            }
            self.bit_mask >>= 1;
        }
    }

    /// Write an interlaced Elias-gamma value (default config:
    /// `elias_ending_bit = 0`, `elias_short_code = 0`).
    fn write_interlaced_elias_gamma(&mut self, value: i64) {
        let value = if value == 0 { 0x100 } else { value };
        // for (i = 2; i <= value; i <<= 1); i >>= 1;
        let mut i: i64 = 2;
        while i <= value {
            i <<= 1;
        }
        i >>= 1;
        // while (i >>= 1) { write_bit(!ending); write_bit(value & i); }
        loop {
            i >>= 1;
            if i == 0 {
                break;
            }
            self.write_bit(true); // !elias_ending_bit == !0 == 1
            self.write_bit((value & i) != 0);
        }
        // if (!elias_short_code || value != 0x100) write_bit(!!elias_ending_bit);
        self.write_bit(false); // !!elias_ending_bit == !!0 == 0
    }
}

// ---------------------------------------------------------------------------------------------
// Public compress
// ---------------------------------------------------------------------------------------------

/// Compress `input` to a raw ZX02 stream, byte-identical to `zx02 input out.zx02`.
pub fn compress_zx02(input: &[u8]) -> Vec<u8> {
    compress_forward(input)
}

/// Compress `input` to a raw backward ZX02 stream, byte-identical to `zx02 -b input out.zx02`.
pub fn compress_zx02_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut rev = input.to_vec();
    rev.reverse();
    let mut out = compress_forward(&rev);
    out.reverse();
    out
}

/// Uniform API entry: `level` is ignored (single optimal algorithm).
pub fn compress(input: &[u8], _level: u8, backward: bool) -> Vec<u8> {
    if backward {
        compress_zx02_backward(input)
    } else {
        compress_zx02(input)
    }
}

/// Forward compression core (no input/output reversal). Runs the optimal parse and bitstream write with
/// the default ZX02 configuration.
fn compress_forward(input: &[u8]) -> Vec<u8> {
    // The reference rejects empty input; we return an empty stream so the uniform API is total.
    if input.is_empty() {
        return Vec::new();
    }

    // The reference parser requires `input_size > 1` (the fake block sits at index `initial_offset`
    // which collapses to 0 for a single byte, and the reference `zx02` actually segfaults on a
    // 1-byte file). Emit the one valid encoding by hand: a single literal byte then the end marker.
    if input.len() == 1 {
        let mut w = Writer::new(input, 0);
        w.write_bit(false); // literal flag
        w.write_interlaced_elias_gamma(1); // length 1
        w.write_byte(input[0]);
        w.write_bit(true); // end marker flag (new offset)
        w.write_interlaced_elias_gamma(256); // gamma(256) -> end
        return w.out;
    }

    let skip = 0usize;
    let (chain, _head_bits) = optimize(input, skip, MAX_OFFSET_ZX02);

    let mut w = Writer::new(input, skip);

    // Walk consecutive blocks. chain[0] is the fake initial block (index == skip-1).
    let mut last_offset: i64 = INITIAL_OFFSET;
    let mut last_literal = false;

    let mut prev_index = chain[0].index; // = skip - 1
    for blk in &chain[1..] {
        let length = blk.index - prev_index;
        if blk.offset == 0 {
            // copy literals
            w.write_bit(false);
            w.write_interlaced_elias_gamma(length);
            for _ in 0..length {
                let b = w.input[w.input_index];
                w.write_byte(b);
                w.input_index += 1;
            }
            last_literal = true;
        } else if blk.offset == last_offset && last_literal {
            // copy from last offset (rep)
            w.write_bit(false);
            w.write_interlaced_elias_gamma(length);
            w.input_index += length as usize;
            last_literal = false;
        } else {
            // copy from new offset
            w.write_bit(true);
            // MSB
            w.write_interlaced_elias_gamma((blk.offset - 1) / 128 + 1);
            // LSB
            w.write_byte((((blk.offset - 1) % 128) << 1) as u8);
            // length: recycle the LSB's low bit as the first bit of the gamma (backtrack)
            w.backtrack = true;
            w.write_interlaced_elias_gamma(length - 1);
            w.input_index += length as usize;
            last_offset = blk.offset;
            last_literal = false;
        }
        prev_index = blk.index;
    }

    // end marker: write_bit(1); write_interlaced_elias_gamma(256)
    w.write_bit(true);
    w.write_interlaced_elias_gamma(256);

    w.out
}

// ---------------------------------------------------------------------------------------------
// Decoder, pure Rust, forward + backward
// ---------------------------------------------------------------------------------------------

/// Decoder state, reduced to the default config
/// (`elias_end = 0`, `elias_short = 0`, `zx1_mode = 0`).
struct Decoder<'a> {
    input: &'a [u8],
    ipos: isize, // signed: forward starts at -1
    bitr: u8,    // bit reserve, u8 so shifts/masks wrap at 8 bits
    extra_bit: u32,
    offset: u32,
    output: Vec<u8>,
    err: bool,
}

impl<'a> Decoder<'a> {
    fn new(input: &'a [u8]) -> Self {
        Decoder {
            input,
            ipos: -1,
            bitr: 0x80,
            extra_bit: 0,
            offset: 0,
            output: Vec::new(),
            err: false,
        }
    }

    /// Fetch the next stream byte (single-buffer case). Forward: `ipos` starts at -1, `iend` is
    /// `len-1`; EOF when `ipos == iend`, else pre-increment and return. We always decode forward
    /// (backward streams are reversed before decoding), so the forward branch is all we need.
    #[inline]
    fn get_byte(&mut self) -> Option<u8> {
        let iend = self.input.len() as isize - 1;
        if self.ipos == iend {
            return None;
        }
        self.ipos += 1;
        Some(self.input[self.ipos as usize])
    }

    /// Read one bit from the stream.
    #[inline]
    fn get_bit(&mut self) -> u32 {
        if self.extra_bit != 0 {
            let bit = self.extra_bit & 1;
            self.extra_bit = 0;
            bit
        } else if self.bitr == 0x80 {
            match self.get_byte() {
                None => {
                    self.err = true;
                    0
                }
                Some(c) => {
                    let bit = ((c & 0x80) != 0) as u32;
                    // uint8_t: (c << 1) | 1 truncates to 8 bits.
                    self.bitr = (c << 1) | 1;
                    bit
                }
            }
        } else {
            let bit = ((self.bitr & 0x80) != 0) as u32;
            // uint8_t: left shift wraps at 8 bits.
            self.bitr <<= 1;
            bit
        }
    }

    /// Read an interlaced Elias-gamma value (default config: `elias_end = 0`, `elias_short = 0`).
    #[inline]
    fn get_elias(&mut self) -> u32 {
        let mut ret: u32 = 1;
        for _ in 0..=8 {
            let b = self.get_bit();
            if b == 0 {
                // b == elias_end (0)
                return ret;
            }
            ret = (ret << 1) | self.get_bit();
            if ret > 0x100 {
                self.err = true;
                return 0;
            }
        }
        self.err = true;
        0
    }

    #[inline]
    fn put_byte(&mut self, b: u8) {
        self.output.push(b);
    }

    fn decode_literal(&mut self) {
        let len = self.get_elias();
        if self.err {
            return;
        }
        for _ in 0..len {
            match self.get_byte() {
                None => {
                    self.err = true;
                    return;
                }
                Some(c) => self.put_byte(c),
            }
        }
    }

    /// Decode a match. `pos = opos - offset - 1`, copying `len` bytes (with 8-bit
    /// length wrap), forward. (Backward output is handled by reversing the whole output at the end.)
    fn decode_match(&mut self, len_add: u32) {
        let mut len = self.get_elias().wrapping_add(len_add);
        if self.err {
            return;
        }
        if len > 0x100 {
            len &= 0xFF;
        }
        // opos == output.len(); pos = opos - offset - 1.
        let opos = self.output.len();
        let off = self.offset as usize;
        if opos < off + 1 {
            self.err = true;
            return;
        }
        let mut pos = opos - off - 1;
        for _ in 0..len {
            let b = self.output[pos];
            self.put_byte(b);
            pos += 1;
        }
    }

    /// Decode an offset (default config: `zx1_mode = 0`). Returns true to end.
    fn decode_offset(&mut self) -> bool {
        let msb = self.get_elias();
        if self.err {
            return true;
        }
        if (msb & 0xFF) == 0 {
            return true; // end marker
        }
        let msb = msb - 1;
        let off = match self.get_byte() {
            None => {
                self.err = true;
                return true;
            }
            Some(b) => b as u32,
        };
        // last bit in offset LSB is used as next bit to be read:
        self.extra_bit = 2 | (off & 1);
        self.offset = (msb << 7) | (off >> 1);
        false
    }

    /// The decode loop, instrumented to also compute the in-place
    /// safety gap. Returns `max(output_produced - input_consumed)` over the decode
    /// (measured at every token boundary). The public wrappers subtract the final
    /// gap; see [`decompress_zx02_with_gap`]. The gap tracking has no effect on the
    /// decoded output - callers that don't need it discard the return value.
    fn decode_loop(&mut self) -> i32 {
        let mut state = 0; // 0 = LITERAL, 1 = repeated offset, 2 = new offset
                           // Peak of (produced - consumed). It grows during a match (output advances,
                           // input barely moves) and peaks at the match's end, which is the state
                           // observed at the next loop iteration's top.
        let mut max_gap = 0i32;
        loop {
            // `ipos` is the index of the last byte read (-1 before any read), so the
            // number of bytes consumed is `ipos + 1` (matching `r.pos` in the sibling
            // decoders). Any constant offset cancels against the final gap anyway.
            let gap = self.output.len() as i32 - (self.ipos + 1) as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            match state {
                0 => {
                    self.decode_literal();
                    if self.err {
                        return max_gap;
                    }
                    state = if self.get_bit() != 0 { 2 } else { 1 };
                }
                2 => {
                    if self.decode_offset() {
                        return max_gap;
                    }
                    self.decode_match(1);
                    if self.err {
                        return max_gap;
                    }
                    state = if self.get_bit() != 0 { 2 } else { 0 };
                }
                _ => {
                    self.decode_match(0);
                    if self.err {
                        return max_gap;
                    }
                    state = if self.get_bit() != 0 { 2 } else { 0 };
                }
            }
        }
    }
}

/// Decompress a forward ZX02 stream (byte-identical to `dzx02`). For backward streams use
/// [`decompress`] with `backward = true`.
pub fn decompress_zx02(input: &[u8]) -> Vec<u8> {
    decompress_zx02_with_gap(input).0
}

/// Decompress a forward ZX02 stream and also return the in-place safety gap
/// (bytes): `max(output_produced - input_consumed)` over the decode, minus its
/// final value. Any in-place layout (forward top-aligned or backward) must keep
/// the write head at least this many bytes clear of the read head, or an
/// incompressible run decoded LATE (whose compressed stream is momentarily larger
/// than the output it has produced) lets the write head clobber unread compressed
/// bytes. See [`max_gap_forward`] / [`max_gap_backward`].
fn decompress_zx02_with_gap(input: &[u8]) -> (Vec<u8>, i32) {
    if input.is_empty() {
        return (Vec::new(), 0);
    }
    let mut d = Decoder::new(input);
    let max_gap = d.decode_loop();
    // The read head consumes the whole `input.len()`-byte block; use it (not the
    // reader position, which stops at the end marker) so the final gap is the true
    // end state.
    let final_gap = d.output.len() as i32 - input.len() as i32;
    (d.output, (max_gap - final_gap).max(0))
}

/// Decompress a backward ZX02 stream (byte-identical decode of `zx02 -b` output). The reference
/// reverses the input buffer logically (reading from the end) and writes output from the end; we
/// reverse the input, decode forward, then reverse the output, which yields the same plaintext.
pub fn decompress_zx02_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut rev = input.to_vec();
    rev.reverse();
    let mut d = Decoder::new(&rev);
    d.decode_loop();
    let mut out = d.output;
    out.reverse();
    out
}

/// Uniform API entry.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        decompress_zx02_backward(input)
    } else {
        decompress_zx02(input)
    }
}

/// In-place safety margin (bytes) for a FORWARD ZX02 stream: the top-aligned
/// packed block must start at least this many bytes above the output end, or the
/// decoder's write head overtakes unread compressed data. See
/// [`decompress_zx02_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        decompress_zx02_with_gap(stream).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD (`zx02 -b`) stream: the packed
/// block must sit at least this many bytes below the span start. The backward
/// stream is `reverse(compress_zx02(reverse(input)))` (see
/// [`compress_zx02_backward`]), so the 6502 backward decoder reading it from the
/// END is exactly a forward decode of the reversed stream - the gap sequence
/// matches.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    decompress_zx02_with_gap(&rev).1.max(0) as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rt(data: &[u8]) {
        // forward
        let c = compress_zx02(data);
        let d = decompress_zx02(&c);
        assert_eq!(d, data, "zx02 forward roundtrip len {}", data.len());
        // backward
        let cb = compress_zx02_backward(data);
        let db = decompress_zx02_backward(&cb);
        assert_eq!(db, data, "zx02 backward roundtrip len {}", data.len());
    }

    #[test]
    fn tiny_and_empty() {
        rt(&[]);
        rt(&[42]);
        rt(&[1, 2, 3, 4, 5]);
        rt(b"abcabcabcabcabc");
        rt(b"abcabcabcabcabcabcdefdefdefdef abcabc");
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        // Incompressible data expands the stream, so an in-place layout needs a
        // margin far larger than the fixed 32-byte default.
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        assert!(
            max_gap_forward(&compress_zx02(&noise)) > 32,
            "incompressible forward gap must exceed the fixed 32-byte margin"
        );
        assert!(
            max_gap_backward(&compress_zx02_backward(&noise)) > 32,
            "incompressible backward gap must exceed the fixed 32-byte margin"
        );
        // Highly compressible data barely expands: the default margin is fine.
        assert!(
            max_gap_backward(&compress_zx02_backward(&vec![0u8; 8192])) <= 32,
            "compressible data should fit within the default margin"
        );
    }

    #[test]
    fn repetitive() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
        rt(&data);
    }

    #[test]
    fn text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        rt(&data);
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
        rt(&data);
    }

    #[test]
    fn all_byte_values() {
        let data: Vec<u8> = (0..=255u8).cycle().take(4096).collect();
        rt(&data);
    }
}
