//! ByteBoozer2 cruncher and decruncher.
//!
//! Implements the ByteBoozer2 (`b2`) format: a cost-based optimal (shortest-path) parser plus a
//! pure-Rust decoder for the crunched bitstream.
//!
//! ## Format (the crunched bitstream)
//!
//! ByteBoozer2 is a bit-oriented LZ with a literal/match copy bit, an Elias-gamma length, a
//! 2-bit "number of offset bits" selector, and an offset field whose width depends on the match
//! length (length 1 matches use a "short" offset table, longer matches use a "long" table). Bytes
//! embedded in the stream (whole offset bytes and literal bytes) are interleaved between the bit
//! groups; the bit reader consumes a fresh byte from the stream whenever its 8-bit accumulator
//! drains. See [`Bb2Writer`] / [`decode_forward`] for the exact bit/byte ordering.
//!
//! ## The b2 container (non-executable `.b2`)
//!
//! `b2 <file.prg>` reads a C64 `.prg` (2-byte little-endian load address + data), strips the load
//! address, crunches the remaining bytes, and writes:
//!
//! ```text
//!   [0..2]  computed start (load) address of the crunched file
//!   [2..4]  original load address (the decrunch-to address) = input[0..2]
//!   [4..]   the crunched bitstream
//! ```
//!
//! [`compress_bb2`] returns ONLY the crunched bitstream (`obuf`, the `[4..]` body). The b2 CLI
//! header is reconstructed by [`b2_container`] for byte-identity checks against `b2.exe`.

/// Highest level accepted by [`compress`]. ByteBoozer2 has a single optimal (cost-based)
/// algorithm, so every level produces the same output.
pub const MAX_LEVEL: u8 = 1;

// --- Offset-table parameters -----------------------------------------------------------------

const NUM_BITS_SHORT_0: u32 = 3;
const NUM_BITS_SHORT_1: u32 = 6;
const NUM_BITS_SHORT_2: u32 = 8;
const NUM_BITS_SHORT_3: u32 = 10;
const NUM_BITS_LONG_0: u32 = 4;
const NUM_BITS_LONG_1: u32 = 7;
const NUM_BITS_LONG_2: u32 = 10;
const NUM_BITS_LONG_3: u32 = 13;

const LEN_SHORT_0: u32 = 1 << NUM_BITS_SHORT_0;
const LEN_SHORT_1: u32 = 1 << NUM_BITS_SHORT_1;
const LEN_SHORT_2: u32 = 1 << NUM_BITS_SHORT_2;
const LEN_SHORT_3: u32 = 1 << NUM_BITS_SHORT_3;
const LEN_LONG_0: u32 = 1 << NUM_BITS_LONG_0;
const LEN_LONG_1: u32 = 1 << NUM_BITS_LONG_1;
const LEN_LONG_2: u32 = 1 << NUM_BITS_LONG_2;
const LEN_LONG_3: u32 = 1 << NUM_BITS_LONG_3;

const MAX_OFFSET: u32 = LEN_LONG_3;
const MAX_OFFSET_SHORT: u32 = LEN_SHORT_3;

#[inline]
fn cond_short_0(o: u32) -> bool {
    o < LEN_SHORT_0
}
#[inline]
fn cond_short_1(o: u32) -> bool {
    o >= LEN_SHORT_0 && o < LEN_SHORT_1
}
#[inline]
fn cond_short_2(o: u32) -> bool {
    o >= LEN_SHORT_1 && o < LEN_SHORT_2
}
#[inline]
fn cond_short_3(o: u32) -> bool {
    o >= LEN_SHORT_2 && o < LEN_SHORT_3
}
#[inline]
fn cond_long_0(o: u32) -> bool {
    o < LEN_LONG_0
}
#[inline]
fn cond_long_1(o: u32) -> bool {
    o >= LEN_LONG_0 && o < LEN_LONG_1
}
#[inline]
fn cond_long_2(o: u32) -> bool {
    o >= LEN_LONG_1 && o < LEN_LONG_2
}
#[inline]
fn cond_long_3(o: u32) -> bool {
    o >= LEN_LONG_2 && o < LEN_LONG_3
}

// --- Public uniform API ----------------------------------------------------------------------

/// Compress `input` into a ByteBoozer2 crunched bitstream. `level` is ignored (one optimal
/// algorithm). `backward` selects the backward (reverse/in-place) variant.
///
/// The returned bytes are the crunched body only (the `obuf` that b2 stores at `[4..]` of its
/// `.b2` file); they are decoded by [`decompress`] with the same `backward` flag.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let _ = level;
    if backward {
        compress_bb2_backward(input)
    } else {
        compress_bb2(input)
    }
}

/// Decompress a ByteBoozer2 crunched bitstream produced by [`compress`]. `backward` selects the
/// backward decoder; otherwise the forward decoder is used.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        decode_backward(input)
    } else {
        decode_forward(input)
    }
}

/// Forward ByteBoozer2 crunch. Returns the crunched bitstream body (b2's `obuf`). Empty input
/// yields an empty `Vec`.
pub fn compress_bb2(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut c = Cruncher::new(input);
    c.setup_help_structures();
    c.find_matches();
    c.write_output()
}

/// Backward ByteBoozer2 crunch, via the standard `reverse(forward(reverse(input)))` construction.
///
/// The body is built by crunching the reversed input forward, then reversing the crunched bytes so
/// a backward decoder reads them tail-first. [`decode_backward`] is the exact inverse. Empty input
/// yields an empty `Vec`.
pub fn compress_bb2_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut rev: Vec<u8> = input.to_vec();
    rev.reverse();
    let mut body = compress_bb2(&rev);
    body.reverse();
    body
}

// --- Cost functions --------------------------------------------------------------------------

/// Bit cost of an Elias-gamma-coded length.
fn cost_of_length(len: u32) -> u32 {
    match len {
        1 => 1,
        2..=3 => 3,
        4..=7 => 5,
        8..=15 => 7,
        16..=31 => 9,
        32..=63 => 11,
        64..=127 => 13,
        128..=255 => 14,
        _ => 10000,
    }
}

/// Bit cost of the offset field for an offset/length pair. `offset`
/// here is the already-decremented (offset-1) value, and `len` is (match_len-1).
fn cost_of_offset(offset: u32, len: u32) -> u32 {
    if len == 1 {
        if cond_short_0(offset) {
            return NUM_BITS_SHORT_0;
        }
        if cond_short_1(offset) {
            return NUM_BITS_SHORT_1;
        }
        if cond_short_2(offset) {
            return NUM_BITS_SHORT_2;
        }
        if cond_short_3(offset) {
            return NUM_BITS_SHORT_3;
        }
    } else {
        if cond_long_0(offset) {
            return NUM_BITS_LONG_0;
        }
        if cond_long_1(offset) {
            return NUM_BITS_LONG_1;
        }
        if cond_long_2(offset) {
            return NUM_BITS_LONG_2;
        }
        if cond_long_3(offset) {
            return NUM_BITS_LONG_3;
        }
    }
    10000
}

/// Total bit cost of emitting a match of `len`/`offset`.
fn calculate_cost_of_match(len: u32, offset: u32) -> u32 {
    let mut cost = 1; // Copy-bit
    cost += cost_of_length(len - 1);
    cost += 2; // NumOffsetBits
    cost += cost_of_offset(offset - 1, len - 1);
    cost
}

/// Cost of extending a literal run by one more byte.
fn calculate_cost_of_literal(old_cost: u32, lit_len: u32) -> u32 {
    let mut new_cost = old_cost + 8;
    match lit_len {
        1 | 128 => new_cost += 1,
        2 | 4 | 8 | 16 | 32 | 64 => new_cost += 2,
        _ => {}
    }
    new_cost
}

// --- Cruncher state --------------------------------------------------------------------------

#[derive(Clone, Copy, Default)]
struct Node {
    cost: u32,
    next: u32,
    lit_len: u32,
    offset: u32,
}

#[derive(Clone, Copy, Default)]
struct RleInfo {
    value: u8,
    value_after: u8,
    length: u32,
}

struct Cruncher<'a> {
    ibuf: &'a [u8],
    ibuf_size: usize,
    context: Vec<Node>,
    link: Vec<u32>,
    rle_info: Vec<RleInfo>,
    first: Vec<u32>, // 65536
    last: Vec<u32>,  // 65536
}

impl<'a> Cruncher<'a> {
    fn new(input: &'a [u8]) -> Self {
        let n = input.len();
        Cruncher {
            ibuf: input,
            ibuf_size: n,
            context: vec![Node::default(); n],
            link: vec![0u32; n],
            rle_info: vec![RleInfo::default(); n],
            first: vec![0u32; 65536],
            last: vec![0u32; 65536],
        }
    }

    /// RLE table + a linked list over digram positions.
    fn setup_help_structures(&mut self) {
        let ibuf = self.ibuf;
        let n = self.ibuf_size;

        // Setup RLE-info (scan from the end).
        let mut get: isize = n as isize - 1;
        while get > 0 {
            let cur = ibuf[get as usize];
            if cur == ibuf[(get - 1) as usize] {
                let mut len: isize = 2;
                while (get >= len) && (cur == ibuf[(get - len) as usize]) {
                    len += 1;
                }
                self.rle_info[get as usize].length = len as u32;
                if get >= len {
                    self.rle_info[get as usize].value_after = ibuf[(get - len) as usize];
                } else {
                    self.rle_info[get as usize].value_after = cur; // avoid ibuf[-1]
                }
                get -= len;
            } else {
                get -= 1;
            }
        }

        // Linked list. first[]/last[] already zeroed by `new`.
        let mut get: isize = n as isize - 1;
        let mut cur: u32 = ibuf[get as usize] as u32;
        while get > 0 {
            cur = ((cur << 8) | ibuf[(get - 1) as usize] as u32) & 65535;
            let c = cur as usize;
            if self.first[c] == 0 {
                self.first[c] = get as u32;
                self.last[c] = get as u32;
            } else {
                self.link[self.last[c] as usize] = get as u32;
                self.last[c] = get as u32;
            }

            if self.rle_info[get as usize].length == 0 {
                get -= 1;
            } else {
                get -= (self.rle_info[get as usize].length - 1) as isize;
            }
        }
    }

    /// Backward shortest-path parse populating `context[]`.
    fn find_matches(&mut self) {
        let ibuf = self.ibuf;
        let n = self.ibuf_size as isize;

        // matches[len] -> (length, offset); index by length, as in the C array of 256 entries.
        let mut match_len = [0u32; 256];
        let mut match_off = [0u32; 256];

        let mut last_cost: u32 = 0;
        // (last.next / last.litLen are tracked but last.next is unused in the loop body)
        let mut last_lit_len: u32 = 0;

        let mut get: isize = n - 1;
        let mut cur: u32 = ibuf[get as usize] as u32;

        // C loop is `while (get >= 0)` over a uint that wraps; we use a signed index and the same
        // termination (process index 0 then stop).
        loop {
            // Clear matches for current position.
            for i in 0..256 {
                match_len[i] = 0;
                match_off[i] = 0;
            }

            cur = (cur << 8) & 65535; // Table65536 lookup
            if get > 0 {
                cur |= ibuf[(get - 1) as usize] as u32;
            }
            let c = cur as usize;
            // scn = first[cur]; scn = link[scn];
            let mut scn: isize = self.first[c] as isize;
            scn = self.link[scn as usize] as isize;

            let mut longest_match: u32 = 0;

            if self.rle_info[get as usize].length == 0 {
                // No RLE-match here.
                while ((get - scn) as u32 <= MAX_OFFSET) && (scn > 0) && (longest_match < 255) {
                    // Match of length >= 2.
                    let mut len: u32 = 2;
                    while (len < 255)
                        && (scn >= len as isize)
                        && (ibuf[(scn - len as isize) as usize]
                            == ibuf[(get - len as isize) as usize])
                    {
                        len += 1;
                    }

                    let offset = (get - scn) as u32;

                    if len > longest_match {
                        longest_match = len;
                        let mut l = len;
                        while l >= 2 && match_len[l as usize] == 0 {
                            if (l > 2) || (l == 2 && offset <= MAX_OFFSET_SHORT) {
                                match_len[l as usize] = l;
                                match_off[l as usize] = (get - scn) as u32;
                            }
                            l -= 1;
                        }
                    }

                    scn = self.link[scn as usize] as isize;
                }

                // Waste first entry.
                self.first[c] = self.link[self.first[c] as usize];
            } else {
                // RLE-match here.
                let rle_len = self.rle_info[get as usize].length;
                let rle_val_after = self.rle_info[get as usize].value_after;

                // First match with self-RLE, always one byte shorter than the RLE itself.
                let mut len = rle_len - 1;
                if len > 1 {
                    if len > 255 {
                        len = 255;
                    }
                    longest_match = len;
                    let mut l = len;
                    while l >= 2 {
                        match_len[l as usize] = l;
                        match_off[l as usize] = 1;
                        l -= 1;
                    }
                }

                // Search for more RLE-matches.
                while ((get - scn) as u32 <= MAX_OFFSET) && (scn > 0) && (longest_match < 255) {
                    // Check for longer matches with same value and after.
                    if (self.rle_info[scn as usize].length > longest_match)
                        && (rle_len > longest_match)
                    {
                        let offset = (get - scn) as u32;
                        let mut l = self.rle_info[scn as usize].length;
                        if l > rle_len {
                            l = rle_len;
                        }
                        if (l > 2) || (l == 2 && offset <= MAX_OFFSET_SHORT) {
                            match_len[l as usize] = l;
                            match_off[l as usize] = offset;
                            longest_match = l;
                        }
                    }

                    // Check for matches beyond the RLE.
                    if (self.rle_info[scn as usize].length >= rle_len)
                        && (self.rle_info[scn as usize].value_after == rle_val_after)
                    {
                        let mut l = rle_len;
                        let offset =
                            (get - scn) as u32 + (self.rle_info[scn as usize].length - rle_len);

                        if offset <= MAX_OFFSET {
                            while (l < 255)
                                && (get >= (offset + l) as isize)
                                && (ibuf[(get - (offset + l) as isize) as usize]
                                    == ibuf[(get - l as isize) as usize])
                            {
                                l += 1;
                            }
                            if l > longest_match {
                                longest_match = l;
                                let mut ll = l;
                                while ll >= 2 && match_len[ll as usize] == 0 {
                                    if (ll > 2) || (ll == 2 && offset <= MAX_OFFSET_SHORT) {
                                        match_len[ll as usize] = ll;
                                        match_off[ll as usize] = offset;
                                    }
                                    ll -= 1;
                                }
                            }
                        }
                    }

                    scn = self.link[scn as usize] as isize;
                }

                if self.rle_info[get as usize].length > 2 {
                    // Expand RLE to next position.
                    let g = get as usize;
                    self.rle_info[g - 1].length = self.rle_info[g].length - 1;
                    self.rle_info[g - 1].value = self.rle_info[g].value;
                    self.rle_info[g - 1].value_after = self.rle_info[g].value_after;
                } else {
                    // End of RLE, advance link.
                    self.first[c] = self.link[self.first[c] as usize];
                }
            }

            // Visit all nodes reached by the matches (highest length first).
            let mut i = 255usize;
            while i > 0 {
                let len = match_len[i];
                let offset = match_off[i];
                if len != 0 {
                    let target_i = (get - len as isize + 1) as usize;
                    let mut current_cost = last_cost;
                    current_cost += calculate_cost_of_match(len, offset);

                    let target = &mut self.context[target_i];
                    if target.cost == 0 || target.cost > current_cost {
                        target.cost = current_cost;
                        target.next = (get + 1) as u32;
                        target.lit_len = 0;
                        target.offset = offset;
                    }
                }
                i -= 1;
            }

            // Cost of this node if using one more literal.
            let lit_len = last_lit_len + 1;
            let lit_cost = calculate_cost_of_literal(last_cost, lit_len);

            let this = &mut self.context[get as usize];
            if this.cost == 0 || this.cost >= lit_cost {
                this.cost = lit_cost;
                this.next = (get + 1) as u32;
                this.lit_len = lit_len;
            }

            last_cost = self.context[get as usize].cost;
            last_lit_len = self.context[get as usize].lit_len;

            if get == 0 {
                break;
            }
            get -= 1;
        }
    }

    /// Serialise `context[]` into the crunched bitstream.
    fn write_output(&self) -> Vec<u8> {
        let mut w = Bb2Writer::new();
        let mut needs_copy_bit = true;

        let mut i: usize = 0;
        while i < self.ibuf_size {
            let link = self.context[i].next as usize;
            let lit_len = self.context[i].lit_len;
            let offset = self.context[i].offset;

            if lit_len == 0 {
                // Match.
                let len = (link - i) as u32;
                if needs_copy_bit {
                    w.w_bit(1);
                }
                w.w_length(len - 1);
                w.w_offset(offset - 1, len - 1);
                i = link;
                needs_copy_bit = true;
            } else {
                // Literal run.
                needs_copy_bit = false;
                let mut remaining = lit_len;
                while remaining > 0 {
                    let len = if remaining < 255 { remaining } else { 255 };
                    w.w_bit(0);
                    w.w_length(len);
                    w.w_bytes(self.ibuf, i, len as usize);
                    if remaining == 255 {
                        needs_copy_bit = true;
                    }
                    remaining -= len;
                    i += len as usize;
                }
            }
        }

        if needs_copy_bit {
            w.w_bit(1);
        }
        w.w_length(0xff);
        w.w_flush();

        w.into_bytes()
    }
}

// --- Bitstream writer ------------------------------------------------------------------------

/// ByteBoozer2 bit/byte writer. Bits accumulate MSB-first into a
/// "current byte" whose slot in the output is reserved when the byte begins; whole bytes
/// (`w_byte`) are appended directly and may be interleaved between bit groups.
struct Bb2Writer {
    obuf: Vec<u8>,
    cur_byte: u8,
    cur_cnt: u8,
    cur_index: usize,
}

impl Bb2Writer {
    fn new() -> Self {
        // C: put=0; curByte=0; curCnt=8; curIndex=put; put++.
        // The reserved slot for the first bit-byte is obuf[0].
        let mut obuf = Vec::new();
        obuf.push(0u8); // reserve index 0 for the first bit accumulator byte
        Bb2Writer {
            obuf,
            cur_byte: 0,
            cur_cnt: 8,
            cur_index: 0,
        }
    }

    #[inline]
    fn w_bit(&mut self, bit: u32) {
        if self.cur_cnt == 0 {
            self.obuf[self.cur_index] = self.cur_byte;
            self.cur_index = self.obuf.len();
            self.cur_cnt = 8;
            self.cur_byte = 0;
            self.obuf.push(0u8); // put++
        }
        self.cur_byte <<= 1;
        self.cur_byte |= (bit & 1) as u8;
        self.cur_cnt -= 1;
    }

    fn w_flush(&mut self) {
        while self.cur_cnt != 0 {
            self.cur_byte <<= 1;
            self.cur_cnt -= 1;
        }
        self.obuf[self.cur_index] = self.cur_byte;
    }

    #[inline]
    fn w_byte(&mut self, b: u8) {
        self.obuf.push(b);
    }

    fn w_bytes(&mut self, ibuf: &[u8], get: usize, len: usize) {
        for k in 0..len {
            self.w_byte(ibuf[get + k]);
        }
    }

    /// Elias-gamma-style length.
    fn w_length(&mut self, len: u32) {
        let mut bit: u32 = 0x80;
        while (len & bit) == 0 {
            bit >>= 1;
        }
        while bit > 1 {
            self.w_bit(1);
            bit >>= 1;
            self.w_bit(if (len & bit) == 0 { 0 } else { 1 });
        }
        if len < 0x80 {
            self.w_bit(0);
        }
    }

    /// Offset field. `offset`/`len` are already decremented (offset-1,
    /// match_len-1) exactly as the caller passes them.
    fn w_offset(&mut self, offset: u32, len: u32) {
        let mut i: u32 = 0;
        let mut n: u32 = 0;

        if len == 1 {
            if cond_short_0(offset) {
                i = 0;
                n = NUM_BITS_SHORT_0;
            }
            if cond_short_1(offset) {
                i = 1;
                n = NUM_BITS_SHORT_1;
            }
            if cond_short_2(offset) {
                i = 2;
                n = NUM_BITS_SHORT_2;
            }
            if cond_short_3(offset) {
                i = 3;
                n = NUM_BITS_SHORT_3;
            }
        } else {
            if cond_long_0(offset) {
                i = 0;
                n = NUM_BITS_LONG_0;
            }
            if cond_long_1(offset) {
                i = 1;
                n = NUM_BITS_LONG_1;
            }
            if cond_long_2(offset) {
                i = 2;
                n = NUM_BITS_LONG_2;
            }
            if cond_long_3(offset) {
                i = 3;
                n = NUM_BITS_LONG_3;
            }
        }

        // First write the 2-bit "number of bits" selector.
        self.w_bit(if (i & 2) == 0 { 0 } else { 1 });
        self.w_bit(if (i & 1) == 0 { 0 } else { 1 });

        if n >= 8 {
            // Offset is 2 bytes.
            let mut b: u32 = 1 << n;
            while b > 0x100 {
                b >>= 1;
                self.w_bit(if (b & offset) == 0 { 0 } else { 1 });
            }
            // Whole low byte, inverted.
            self.w_byte(((offset & 255) ^ 255) as u8);
            // offset >>= 8; (no further use)
        } else {
            // Offset is 1 byte.
            let mut b: u32 = 1 << n;
            while b > 1 {
                b >>= 1;
                self.w_bit(if (b & offset) == 0 { 1 } else { 0 }); // inverted
            }
        }
    }

    fn into_bytes(self) -> Vec<u8> {
        self.obuf
    }

    /// Number of output bytes written so far (the C `put` counter). Equal to `obuf.len()` because
    /// each reserved bit-byte and each whole byte advances `put` in lockstep with our `Vec`.
    fn put(&self) -> usize {
        self.obuf.len()
    }
}

// --- Decoder ---------------------------------------------------------------------------------

/// Per-`Tab`-entry decode mask. Index 0..3 = short offsets (match length 1),
/// index 4..7 = long offsets, selected by `[short_or_long, i_hi, i_lo]`. The byte is the bit
/// pattern fed into the `M_1` "get bits < 8" loop; see `decode_offset`.
const TAB: [u8; 8] = [
    // Short offsets
    0b1101_1111, // 3
    0b1111_1011, // 6
    0b0000_0000, // 8
    0b1000_0000, // 10
    // Long offsets
    0b1110_1111, // 4
    0b1111_1101, // 7
    0b1000_0000, // 10
    0b1111_0000, // 13
];

/// ByteBoozer2 bit/byte reader. Bits are read MSB-first from each stream byte; whole bytes
/// (literal bytes, the inverted offset low byte) are read directly. This reproduces the 6502
/// decruncher's self-timing `GetNextBit` exactly: the encoder packs 8 bits MSB-first per reserved
/// byte, so "read 8 bits MSB-first, then fetch a new byte" yields the identical bit/byte ordering.
struct Bb2Reader<'a> {
    data: &'a [u8],
    pos: usize,
    acc: u8,
    cnt: u8,
}

impl<'a> Bb2Reader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Bb2Reader {
            data,
            pos: 0,
            acc: 0,
            cnt: 0,
        }
    }

    #[inline]
    fn read_byte(&mut self) -> u8 {
        let b = self.data[self.pos];
        self.pos += 1;
        b
    }

    #[inline]
    fn read_bit(&mut self) -> u32 {
        if self.cnt == 0 {
            self.acc = self.read_byte();
            self.cnt = 8;
        }
        let bit = (self.acc >> 7) & 1;
        self.acc <<= 1;
        self.cnt -= 1;
        bit as u32
    }

    /// `GetLen` macro: gamma length. `A=1`; while the continuation bit is 1, `A=(A<<1)|nextbit`,
    /// stopping once bit 7 of `A` is set (the 6502 `bpl` falls through). Max value 0xff.
    fn read_len(&mut self) -> u32 {
        let mut a: u32 = 1;
        loop {
            if self.read_bit() == 0 {
                break;
            }
            a = (a << 1) | self.read_bit();
            if a & 0x80 != 0 {
                break;
            }
        }
        a
    }
}

/// Decode a match's back-distance, emulating the 6502 `Match`/`M_1`/`M8`/`MShort` path.
/// Returns the actual offset (distance back from the current output position).
///
/// `mlen` is the gamma length value (`match_len - 1`). The asm builds a signed 16-bit displacement
/// `(Y<<8 | A)` that is added to `put` to form the copy source; the offset (positive distance back)
/// is therefore `-(disp) mod 65536`.
///
/// Steps:
///   * Read the 2 selector bits; index `Tab` by `[mlen>=2, sel_hi, sel_lo]` (low 3 bits).
///   * `lda Tab,y; beq M8`: if the entry is 0, skip the bit loop and go straight to M8.
///   * `M_1`: `GetNextBit; rol A; bcs M_1` - rotate stream bits into A while the table sentinel
///     rotates out via carry. `bmi MShort` (A bit 7 set after the loop) selects the 1-byte short
///     case (`Y = $ff`, no extra byte). Otherwise (`M8`) `A = A ^ $ff` becomes the high byte `Y`,
///     and a full low byte is read from the stream.
fn decode_offset(r: &mut Bb2Reader, mlen: u32) -> u32 {
    // Y index = (mlen>=2)<<2 | sel_hi<<1 | sel_lo
    let mut idx: u32 = if mlen >= 2 { 1 } else { 0 };
    idx = (idx << 1) | r.read_bit();
    idx = (idx << 1) | r.read_bit();

    let mut a: u32 = TAB[idx as usize] as u32;

    // `beq M8`: table entry 0 jumps past the M_1 loop straight into M8 (with A == 0).
    if a != 0 {
        // M_1: GetNextBit; rol A; bcs M_1
        loop {
            let bit = r.read_bit();
            let carry_out = (a >> 7) & 1;
            a = ((a << 1) | bit) & 0xff;
            if carry_out == 0 {
                break;
            }
        }
        // bmi MShort
        if a & 0x80 != 0 {
            // MShort: Y = $ff (high = -1), A holds the low byte directly.
            let y = 0xffu32;
            return offset_from_disp(a, y);
        }
    }

    // M8: eor #$ff; tay (A becomes the high byte Y); read full low byte from stream.
    let y = a ^ 0xff;
    let lo = r.read_byte() as u32;
    offset_from_disp(lo, y)
}

/// Convert the asm's signed 16-bit displacement `(y<<8 | a)` (added to `put` to locate the copy
/// source) into a positive back-distance (offset). Since the source is always before `put`, the
/// displacement is negative in two's complement and the offset is its 16-bit negation.
#[inline]
fn offset_from_disp(a: u32, y: u32) -> u32 {
    let disp = ((y << 8) | a) & 0xffff;
    (0x1_0000 - disp) & 0xffff
}

/// Forward ByteBoozer2 decoder. Decodes the crunched bitstream body produced by [`compress_bb2`]
/// back to the original bytes.
///
/// Control flow:
///   * `DLoop`: read a copy bit. 1 -> Match, 0 -> Literal.
///   * Literal: read the length gamma, copy that many literal bytes. If the length was 255 (the
///     `iny; beq DLoop` case), loop back to `DLoop` (read a copy bit); otherwise **fall through to
///     Match without reading a copy bit** (the encoder cleared `needCopyBit` for sub-255 runs).
///   * Match: read the length gamma; `0xff` is EOF. Otherwise decode the offset and copy.
pub fn decode_forward(data: &[u8]) -> Vec<u8> {
    decode_forward_with_gap(data).0
}

/// Decode and also return the in-place safety gap (bytes): `max(produced -
/// consumed)` over the decode minus its final value - the same quantity
/// `compute_margin` derives at pack time (`maxDiff - (i - put)`). Any in-place
/// layout must keep the write head this many bytes clear of the read head, or an
/// incompressible run decoded LATE (whose stream is momentarily larger than the
/// output it has produced) lets the write head clobber unread compressed bytes.
/// See [`max_gap_forward`] / [`max_gap_backward`].
fn decode_forward_with_gap(data: &[u8]) -> (Vec<u8>, i32) {
    if data.is_empty() {
        return (Vec::new(), 0);
    }
    let mut r = Bb2Reader::new(data);
    let mut out: Vec<u8> = Vec::new();
    // Peak of (produced - consumed) at a token boundary. It spikes at the end of
    // a match (output jumps, input barely moves), which is the state observed at
    // the next loop iteration's top.
    let mut max_gap = 0i32;

    'dloop: loop {
        let gap = out.len() as i32 - r.pos as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        // DLoop: read the copy bit. 0 -> Literal (one or more sub-255 runs that each fall through
        // to a following Match), 1 -> Match.
        if r.read_bit() == 0 {
            // Literal run.
            let len = r.read_len() as usize;
            for _ in 0..len {
                let b = r.read_byte();
                out.push(b);
            }
            if len == 255 {
                // `iny; beq DLoop`: a full 255-run loops back to read a fresh copy bit.
                continue 'dloop;
            }
            // Sub-255 run falls through to Match with no copy bit.
        }

        // Match (reached via copy bit 1, or the literal fall-through).
        let mlen = r.read_len();
        if mlen == 0xff {
            break; // EOF
        }
        let offset = decode_offset(&mut r, mlen) as usize;
        let len = (mlen + 1) as usize;
        let base = out.len();
        for k in 0..len {
            let v = out[base - offset + k];
            out.push(v);
        }
    }

    // The read head consumes the whole block; use `data.len()` (not `r.pos`,
    // which stops at EOF) for the true final gap.
    let final_gap = out.len() as i32 - data.len() as i32;
    (out, (max_gap - final_gap).max(0))
}

/// In-place safety margin (bytes) for a FORWARD ByteBoozer2 stream: the
/// top-aligned packed block must start at least this many bytes above the output
/// end. See [`decode_forward_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    decode_forward_with_gap(stream).1.max(0) as usize
}

/// In-place safety margin (bytes) for a BACKWARD ByteBoozer2 stream: the packed
/// block must sit at least this many bytes below the span start. The 6502
/// backward decoder reads the stored stream reversed, so the gap sequence is a
/// forward decode of the reversed stream.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    decode_forward_with_gap(&rev).1.max(0) as usize
}

/// Backward ByteBoozer2 decoder. Inverse of [`compress_bb2_backward`]: reverse the stream, decode
/// forward, then reverse the output.
pub fn decode_backward(stream: &[u8]) -> Vec<u8> {
    if stream.is_empty() {
        return Vec::new();
    }
    let data: Vec<u8> = stream.iter().rev().copied().collect();
    let mut out = decode_forward(&data);
    out.reverse();
    out
}

// --- b2 container reconstruction (for byte-identity vs b2.exe) --------------------------------

/// Reconstruct b2's non-executable `.b2` container from a `.prg` `input` (2-byte load address +
/// data), byte-identical to `b2 <file.prg>`:
///
/// ```text
///   [0..2]  computed start (load) address
///   [2..4]  original load address (decrunch-to)
///   [4..]   crunched bitstream
/// ```
///
/// Returns `None` if `input` has no payload after the load address.
pub fn b2_container(input: &[u8]) -> Option<Vec<u8>> {
    if input.len() < 2 {
        return None;
    }
    let load_addr = (input[0] as u32) | ((input[1] as u32) << 8);
    let data = &input[2..];
    if data.is_empty() {
        return None;
    }

    let mut c = Cruncher::new(data);
    c.setup_help_structures();
    c.find_matches();
    let margin = c.compute_margin();
    let body = c.write_output();

    let ibuf_size = data.len();
    let pack_len = body.len();

    // Non-executable start address (isExecutable=false, isRelocated=false):
    //   startAddress = loadAddr + (ibufSize - packLen - 2 + margin)
    let start_address =
        (load_addr as i64 + (ibuf_size as i64 - pack_len as i64 - 2 + margin)) & 0xffff;

    let mut out = Vec::with_capacity(4 + pack_len);
    out.push((start_address & 0xff) as u8);
    out.push(((start_address >> 8) & 0xff) as u8);
    out.push(input[0]); // depack-to (original load address)
    out.push(input[1]);
    out.extend_from_slice(&body);
    Some(out)
}

impl<'a> Cruncher<'a> {
    /// Replicates `writeOutput`'s `margin = maxDiff - (i - put)` bookkeeping (used only for the
    /// container start address) by re-walking the parse and tracking the `put` byte counter.
    fn compute_margin(&self) -> i64 {
        let mut w = Bb2Writer::new();
        let mut needs_copy_bit = true;
        let mut max_diff: i64 = 0;

        let mut i: usize = 0;
        while i < self.ibuf_size {
            let link = self.context[i].next as usize;
            let lit_len = self.context[i].lit_len;
            let offset = self.context[i].offset;

            if lit_len == 0 {
                let len = (link - i) as u32;
                if needs_copy_bit {
                    w.w_bit(1);
                }
                w.w_length(len - 1);
                w.w_offset(offset - 1, len - 1);
                i = link;
                needs_copy_bit = true;
            } else {
                needs_copy_bit = false;
                let mut remaining = lit_len;
                while remaining > 0 {
                    let len = if remaining < 255 { remaining } else { 255 };
                    w.w_bit(0);
                    w.w_length(len);
                    w.w_bytes(self.ibuf, i, len as usize);
                    if remaining == 255 {
                        needs_copy_bit = true;
                    }
                    remaining -= len;
                    i += len as usize;
                }
            }

            let put = w.put();
            let diff = i as i64 - put as i64;
            if diff > max_diff {
                max_diff = diff;
            }
        }

        if needs_copy_bit {
            w.w_bit(1);
        }
        w.w_length(0xff);
        w.w_flush();

        let put = w.put();
        max_diff - (i as i64 - put as i64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrips(data: &[u8]) {
        let blob = compress_bb2(data);
        let out = decode_forward(&blob);
        assert_eq!(out, data, "bb2 forward roundtrip len {}", data.len());
    }

    fn roundtrips_backward(data: &[u8]) {
        let blob = compress_bb2_backward(data);
        let out = decode_backward(&blob);
        assert_eq!(out, data, "bb2 backward roundtrip len {}", data.len());
    }

    #[test]
    fn tiny() {
        roundtrips(&[42]);
        roundtrips(&[1, 2, 3, 4, 5]);
        roundtrips(b"abcabcabcabcabc");
        roundtrips(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        roundtrips_backward(&[42]);
        roundtrips_backward(&[1, 2, 3, 4, 5]);
        roundtrips_backward(b"abcabcabcabcabc");
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
            max_gap_forward(&compress_bb2(&noise)) > 32,
            "incompressible forward gap must exceed the fixed 32-byte margin"
        );
        assert!(
            max_gap_backward(&compress_bb2_backward(&noise)) > 32,
            "incompressible backward gap must exceed the fixed 32-byte margin"
        );
        // Highly compressible data barely expands: the default margin is fine.
        assert!(
            max_gap_backward(&compress_bb2_backward(&vec![0u8; 8192])) <= 32,
            "compressible data should fit within the default margin"
        );
    }

    #[test]
    fn empty() {
        assert!(compress_bb2(&[]).is_empty());
        assert!(decode_forward(&[]).is_empty());
        assert!(compress_bb2_backward(&[]).is_empty());
        assert!(decode_backward(&[]).is_empty());
    }

    #[test]
    fn repetitive() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
        roundtrips(&data);
        roundtrips_backward(&data);
    }

    #[test]
    fn text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        roundtrips(&data);
        roundtrips_backward(&data);
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
        roundtrips_backward(&data);
    }

    #[test]
    fn long_rle() {
        let data = vec![0x55u8; 70000];
        roundtrips(&data);
        roundtrips_backward(&data);
    }
}
