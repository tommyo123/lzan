//! Shrinkler: optimal LZ cruncher and its range-coded context-model decoder (top ratio).
//!
//! A pure, std-only Rust implementation of the Shrinkler format:
//!
//!   * The range coder (`RangeCoder`) and its decoder (`RangeDecoder` + `LZDecoder`).
//!   * The LZ token encoding (`LZEncoder`) with the adaptive context model.
//!   * The local-optimal, rep-aware parser (`LZParser`) with the `CuckooHash`,
//!     `Heap` and `RefEdge` machinery, so the parse and hence the compressed
//!     bytes are reproducible bit-for-bit.
//!   * The SA-IS suffix array + `MatchFinder`.
//!   * The multi-iteration `packData` driver with `CountingCoder` /
//!     `SizeMeasuringCoder` probability refinement.
//!
//! Output is produced in **no-parity mode** (`Shrinkler -d -b`), i.e. an 8-bit
//! data stream with a single literal/kind context per bit position (no parity
//! split). This is the stream the 6502 `unshrinkler` decruncher (PARITY=0)
//! decodes, and what the in-crate [`decompress`] decodes.
//!
//! `compress` is a 2-tier level system: **level 1** is the FAST anchor - a single
//! stock `-9` parse, byte-identical to the reference cruncher's `-9 -d -b`
//! no-parity output on the corpus - and **level 2** is the multi-config best-of
//! (always `<=` level 1). A native API ([`compress_native`]) exposes the real
//! `pack_data` knobs ([`PackParams`], starting from [`params_no_parity`]).
//! `decompress` is level-independent, a pure-Rust range/LZ decoder.

#![allow(clippy::needless_range_loop)]

// ============================================================================
// Coder model constants
// ============================================================================

const BIT_PRECISION: i32 = 6;
const ADJUST_SHIFT: u32 = 4;

const NUM_SINGLE_CONTEXTS: i32 = 1;
const NUM_CONTEXT_GROUPS: i32 = 4;
const CONTEXT_GROUP_SIZE: i32 = 256;

const CONTEXT_KIND: i32 = 0;
const CONTEXT_REPEATED: i32 = -1;

const CONTEXT_GROUP_OFFSET: i32 = 2;
const CONTEXT_GROUP_LENGTH: i32 = 3;

const KIND_LIT: i32 = 0;
const KIND_REF: i32 = 1;

const NUM_CONTEXTS: i32 = NUM_SINGLE_CONTEXTS + NUM_CONTEXT_GROUPS * CONTEXT_GROUP_SIZE;
const NUMBER_CONTEXT_OFFSET: i32 = NUM_SINGLE_CONTEXTS + CONTEXT_GROUP_OFFSET * CONTEXT_GROUP_SIZE;
const NUM_NUMBER_CONTEXTS: i32 = 2;

// The reference allocates NUM_CONTEXTS + NUM_RELOC_CONTEXTS contexts for the
// data-mode range coder. Reloc contexts are never used in data mode, but the
// allocation size is part of the (irrelevant-to-the-stream) bookkeeping. We
// allocate the same total so context indices line up identically.
const NUM_RELOC_CONTEXTS: i32 = 256;
const TOTAL_CONTEXTS: usize = (NUM_CONTEXTS + NUM_RELOC_CONTEXTS) as usize;

// ============================================================================
// Coder trait: code() one bit, returns size in fractional bits; encodeNumber()
// ============================================================================

/// Abstract entropy coder, mirroring `Coder` in the reference.
trait Coder {
    fn code(&mut self, context: i32, bit: i32) -> i32;
}

/// Variable-length number encoding (number >= 2).
/// Implemented as a free function taking a `&mut dyn Coder` plus an optional
/// size cache (`NumberCache`) used by the `SizeMeasuringCoder`.
fn encode_number(coder: &mut dyn Coder, base_context: i32, number: i32) -> i32 {
    debug_assert!(number >= 2);
    let mut size = 0;
    let mut i = 0;
    while (4 << i) <= number {
        let context = base_context + (i * 2 + 2);
        size += coder.code(context, 1);
        i += 1;
    }
    let context = base_context + (i * 2 + 2);
    size += coder.code(context, 0);

    while i >= 0 {
        let bit = (number >> i) & 1;
        let context = base_context + (i * 2 + 1);
        size += coder.code(context, bit);
        i -= 1;
    }
    size
}

// ============================================================================
// RangeCoder (encoder)
// ============================================================================

fn build_sizetable() -> [i32; 128] {
    let mut t = [0i32; 128];
    for i in 0..128 {
        t[i] = (0.5 + (8.0 - ((128 + i) as f64).ln() / 2.0_f64.ln()) * (1 << BIT_PRECISION) as f64)
            .floor() as i32;
    }
    t
}

struct RangeCoder {
    contexts: Vec<u16>,
    out: Vec<u8>,
    dest_bit: i32,
    intervalsize: u32,
    intervalmin: u32,
    sizetable: [i32; 128],
}

impl RangeCoder {
    fn new(n_contexts: usize) -> RangeCoder {
        RangeCoder {
            contexts: vec![0x8000; n_contexts],
            out: Vec::new(),
            dest_bit: -1,
            intervalsize: 0x8000,
            intervalmin: 0,
            sizetable: build_sizetable(),
        }
    }

    fn add_bit(&mut self) {
        let mut pos = self.dest_bit;
        loop {
            pos -= 1;
            if pos < 0 {
                return;
            }
            let bytepos = (pos >> 3) as usize;
            let bitmask = 0x80u8 >> (pos & 7);
            while bytepos >= self.out.len() {
                self.out.push(0);
            }
            self.out[bytepos] ^= bitmask;
            if (self.out[bytepos] & bitmask) != 0 {
                break;
            }
        }
    }

    fn reset(&mut self) {
        for c in self.contexts.iter_mut() {
            *c = 0x8000;
        }
    }

    fn finish(&mut self) {
        let intervalmax = self.intervalmin.wrapping_add(self.intervalsize) as i64;
        let mut final_min: i64 = 0;
        let mut final_size: i64 = 0x10000;
        while final_min < self.intervalmin as i64 || final_min + final_size >= intervalmax {
            if final_min + final_size < intervalmax {
                self.add_bit();
                final_min += final_size;
            }
            self.dest_bit += 1;
            final_size >>= 1;
        }
        while ((self.dest_bit - 1) >> 3) as i64 >= self.out.len() as i64 {
            self.out.push(0);
        }
    }
}

impl Coder for RangeCoder {
    fn code(&mut self, context_index: i32, bit: i32) -> i32 {
        let st = |sz: u32, sizetable: &[i32; 128]| sizetable[((sz - 0x8000) >> 8) as usize];
        let size_before = (self.dest_bit << BIT_PRECISION) + st(self.intervalsize, &self.sizetable);
        let prob = self.contexts[context_index as usize] as u32;
        let threshold = (self.intervalsize * prob) >> 16;
        let new_prob;
        if bit == 0 {
            self.intervalmin = self.intervalmin.wrapping_add(threshold);
            if self.intervalmin & 0x10000 != 0 {
                self.add_bit();
            }
            self.intervalsize -= threshold;
            new_prob = prob - (prob >> ADJUST_SHIFT);
        } else {
            self.intervalsize = threshold;
            new_prob = prob + (0xffff >> ADJUST_SHIFT) - (prob >> ADJUST_SHIFT);
        }
        self.contexts[context_index as usize] = new_prob as u16;
        while self.intervalsize < 0x8000 {
            self.dest_bit += 1;
            self.intervalsize <<= 1;
            self.intervalmin <<= 1;
            if self.intervalmin & 0x10000 != 0 {
                self.add_bit();
            }
        }
        self.intervalmin &= 0xffff;
        let size_after = (self.dest_bit << BIT_PRECISION) + st(self.intervalsize, &self.sizetable);
        size_after - size_before
    }
}

// ============================================================================
// CountingCoder + SizeMeasuringCoder - probability refinement across iterations
// ============================================================================

#[derive(Clone, Copy)]
struct ContextCounts {
    counts: [i32; 2],
}

struct CountingCoder {
    context_counts: Vec<ContextCounts>,
}

impl CountingCoder {
    fn new(n_contexts: usize) -> CountingCoder {
        CountingCoder {
            context_counts: vec![ContextCounts { counts: [0, 0] }; n_contexts],
        }
    }

    fn merge(old_counts: &CountingCoder, new_counts: &CountingCoder) -> CountingCoder {
        let mut cc = Vec::with_capacity(old_counts.context_counts.len());
        for i in 0..old_counts.context_counts.len() {
            let o = old_counts.context_counts[i];
            let n = new_counts.context_counts[i];
            cc.push(ContextCounts {
                counts: [
                    (o.counts[0] * 3 + n.counts[0]) / 4,
                    (o.counts[1] * 3 + n.counts[1]) / 4,
                ],
            });
        }
        CountingCoder { context_counts: cc }
    }
}

impl Coder for CountingCoder {
    fn code(&mut self, context_index: i32, bit: i32) -> i32 {
        self.context_counts[context_index as usize].counts[bit as usize] += 1;
        0
    }
}

#[derive(Clone, Copy)]
struct ContextSizes {
    sizes: [u16; 2],
}

struct SizeMeasuringCoder {
    context_sizes: Vec<ContextSizes>,
    // Number-size cache. Indexed by context group.
    cache: Vec<Vec<u16>>,
    has_cache: bool,
    number_context_offset: i32,
    n_number_contexts: i32,
}

impl SizeMeasuringCoder {
    const MIN_SIZE: i32 = 2;
    const MAX_SIZE: i32 = 12 << BIT_PRECISION;

    fn size_for_count(count: i32, total: i32) -> i32 {
        let mut size = (0.5
            + (total as f64 / count as f64).ln() / 2.0_f64.ln() * (1 << BIT_PRECISION) as f64)
            .floor() as i32;
        if size < Self::MIN_SIZE {
            size = Self::MIN_SIZE;
        }
        if size > Self::MAX_SIZE {
            size = Self::MAX_SIZE;
        }
        size
    }

    fn from_counting(counting: &CountingCoder) -> SizeMeasuringCoder {
        let mut context_sizes = Vec::with_capacity(counting.context_counts.len());
        for c in &counting.context_counts {
            let count0 = 1 + c.counts[0];
            let count1 = 1 + c.counts[1];
            let sum = count0 + count1;
            context_sizes.push(ContextSizes {
                sizes: [
                    Self::size_for_count(count0, sum) as u16,
                    Self::size_for_count(count1, sum) as u16,
                ],
            });
        }
        SizeMeasuringCoder {
            context_sizes,
            cache: Vec::new(),
            has_cache: false,
            number_context_offset: 0,
            n_number_contexts: 0,
        }
    }

    /// Set the number-context range for the coder.
    fn set_number_contexts(
        &mut self,
        number_context_offset: i32,
        n_number_contexts: i32,
        max_number: i32,
    ) {
        self.number_context_offset = number_context_offset;
        self.n_number_contexts = n_number_contexts;
        self.cache.clear();
        for context_index in 0..n_number_contexts {
            let base_context = number_context_offset + (context_index << 8);
            let mut c: Vec<u16> = vec![0; 4];
            c[2] = (self.code(base_context + 2, 0) + self.code(base_context + 1, 0)) as u16;
            c[3] = (self.code(base_context + 2, 0) + self.code(base_context + 1, 1)) as u16;
            let mut prev_base = 2usize;
            'data_bits: for data_bits in 2..30i32 {
                let base = c.len();
                let base_sizedif = -self.code(base_context + data_bits * 2 - 2, 0)
                    + self.code(base_context + data_bits * 2 - 2, 1)
                    + self.code(base_context + data_bits * 2, 0);
                for msb in 0..=1 {
                    let sizedif = base_sizedif + self.code(base_context + data_bits * 2 - 1, msb);
                    for tail in 0..(1i32 << (data_bits - 1)) {
                        let size = c[prev_base + tail as usize] as i32 + sizedif;
                        c.push(size as u16);
                        if c.len() > max_number as usize {
                            break 'data_bits;
                        }
                    }
                }
                prev_base = base;
            }
            self.cache.push(c);
        }
        self.has_cache = true;
    }

    fn encode_number_cached(&mut self, base_context: i32, number: i32) -> i32 {
        if self.has_cache {
            let context_index = ((base_context - self.number_context_offset) >> 8) as usize;
            let cache_for_context = &self.cache[context_index];
            if (number as usize) < cache_for_context.len() {
                return cache_for_context[number as usize] as i32;
            }
        }
        encode_number(self, base_context, number)
    }
}

impl Coder for SizeMeasuringCoder {
    fn code(&mut self, context_index: i32, bit: i32) -> i32 {
        self.context_sizes[context_index as usize].sizes[bit as usize] as i32
    }
}

// ============================================================================
// LZEncoder: token format + context model
// ============================================================================

#[derive(Clone, Copy, Default)]
struct LZState {
    after_first: bool,
    prev_was_ref: bool,
    parity: i32, // full position; parity_mask isolates the low bit
    last_offset: i32,
}

/// Encoder bound to a coder (which may be a measurer or the final range coder).
/// Generic over the underlying coder; uses a `parity_mask` of 0 (no-parity) or 1.
struct LZEncoder<'a> {
    coder: LZCoderRef<'a>,
    parity_mask: i32,
}

/// The encoder needs to drive either a RangeCoder, a CountingCoder, or a
/// SizeMeasuringCoder (the latter using the number cache). We dispatch by enum
/// to keep the cache path available for the size measurer.
enum LZCoderRef<'a> {
    Range(&'a mut RangeCoder),
    Counting(&'a mut CountingCoder),
    Measuring(&'a mut SizeMeasuringCoder),
}

impl<'a> LZEncoder<'a> {
    fn new(coder: LZCoderRef<'a>, parity_context: bool) -> LZEncoder<'a> {
        LZEncoder {
            coder,
            parity_mask: if parity_context { 1 } else { 0 },
        }
    }

    fn code(&mut self, context: i32, bit: i32) -> i32 {
        let ctx = NUM_SINGLE_CONTEXTS + context;
        match &mut self.coder {
            LZCoderRef::Range(c) => c.code(ctx, bit),
            LZCoderRef::Counting(c) => c.code(ctx, bit),
            LZCoderRef::Measuring(c) => c.code(ctx, bit),
        }
    }

    fn encode_number(&mut self, context_group: i32, number: i32) -> i32 {
        let base = NUM_SINGLE_CONTEXTS + (context_group << 8);
        match &mut self.coder {
            LZCoderRef::Range(c) => encode_number(*c, base, number),
            LZCoderRef::Counting(c) => encode_number(*c, base, number),
            LZCoderRef::Measuring(c) => c.encode_number_cached(base, number),
        }
    }

    fn set_initial_state(state: &mut LZState) {
        state.after_first = false;
        state.prev_was_ref = false;
        state.parity = 0;
        state.last_offset = 0;
    }

    fn construct_state(state: &mut LZState, pos: i32, prev_was_ref: bool, last_offset: i32) {
        state.after_first = pos > 0;
        state.prev_was_ref = prev_was_ref;
        state.parity = pos;
        state.last_offset = last_offset;
    }

    fn encode_literal(
        &mut self,
        value: u8,
        state_before: &LZState,
        state_after: &mut LZState,
    ) -> i32 {
        let parity_offset = (state_before.parity & self.parity_mask) << 8;
        let mut size = 0;
        if state_before.after_first {
            size += self.code(CONTEXT_KIND + parity_offset, KIND_LIT);
        }
        let mut context = 1i32;
        for i in (0..8).rev() {
            let bit = ((value >> i) & 1) as i32;
            size += self.code(parity_offset | context, bit);
            context = (context << 1) | bit;
        }
        state_after.after_first = true;
        state_after.prev_was_ref = false;
        state_after.parity = state_before.parity + 1;
        state_after.last_offset = state_before.last_offset;
        size
    }

    fn encode_reference(
        &mut self,
        offset: i32,
        length: i32,
        state_before: &LZState,
        state_after: &mut LZState,
    ) -> i32 {
        debug_assert!(offset >= 1);
        debug_assert!(length >= 2);
        debug_assert!(state_before.after_first);
        let parity_offset = (state_before.parity & self.parity_mask) << 8;
        let mut size = self.code(CONTEXT_KIND + parity_offset, KIND_REF);
        let rep_offset = offset == state_before.last_offset;
        if !state_before.prev_was_ref {
            size += self.code(CONTEXT_REPEATED, rep_offset as i32);
        } else {
            debug_assert!(!rep_offset);
        }
        if !rep_offset {
            size += self.encode_number(CONTEXT_GROUP_OFFSET, offset + 2);
        }
        size += self.encode_number(CONTEXT_GROUP_LENGTH, length);

        state_after.after_first = true;
        state_after.prev_was_ref = true;
        state_after.parity = state_before.parity + length;
        state_after.last_offset = offset;
        size
    }

    fn finish(&mut self, state_before: &LZState) -> i32 {
        let parity_offset = (state_before.parity & self.parity_mask) << 8;
        let mut size = self.code(CONTEXT_KIND + parity_offset, KIND_REF);
        if !state_before.prev_was_ref {
            size += self.code(CONTEXT_REPEATED, 0);
        }
        size += self.encode_number(CONTEXT_GROUP_OFFSET, 2);
        size
    }
}

// ============================================================================
// Suffix array (SA-IS)
// ============================================================================

const UNINITIALIZED: i32 = -1;

#[inline]
fn is_lms(stype: &[bool], i: i32) -> bool {
    i > 0 && stype[i as usize] && !stype[(i - 1) as usize]
}

fn induce(
    data: &[i32],
    suffix_array: &mut [i32],
    length: usize,
    alphabet_size: usize,
    stype: &[bool],
    buckets: &[i32],
    bucket_index: &mut [i32],
) {
    for b in 0..alphabet_size {
        bucket_index[b] = buckets[b];
    }
    for s in 0..length {
        let index = suffix_array[s];
        if index > 0 && !stype[(index - 1) as usize] {
            let sym = data[(index - 1) as usize] as usize;
            suffix_array[bucket_index[sym] as usize] = index - 1;
            bucket_index[sym] += 1;
        }
    }
    for b in 0..alphabet_size {
        bucket_index[b] = buckets[b + 1];
    }
    for s in (0..length).rev() {
        let index = suffix_array[s];
        if index > 0 && stype[(index - 1) as usize] {
            let sym = data[(index - 1) as usize] as usize;
            bucket_index[sym] -= 1;
            suffix_array[bucket_index[sym] as usize] = index - 1;
        }
    }
}

fn substrings_equal(data: &[i32], mut i1: i32, mut i2: i32, stype: &[bool]) -> bool {
    loop {
        let a = data[i1 as usize];
        let b = data[i2 as usize];
        i1 += 1;
        i2 += 1;
        if a != b {
            return false;
        }
        if is_lms(stype, i1) && is_lms(stype, i2) {
            return true;
        }
    }
}

fn compute_suffix_array(
    data: &[i32],
    suffix_array: &mut [i32],
    length: usize,
    alphabet_size: usize,
) {
    debug_assert!(length >= 1);
    if length == 1 {
        suffix_array[0] = 0;
        return;
    }

    let mut stype = vec![false; length];
    let mut buckets = vec![0i32; alphabet_size + 1];
    let mut bucket_index = vec![0i32; alphabet_size];

    stype[length - 1] = true;
    buckets[data[length - 1] as usize] = 1;
    let mut is_s = true;
    let mut lms_count = 0;
    for i in (0..length - 1).rev() {
        buckets[data[i] as usize] += 1;
        if data[i] > data[i + 1] {
            if is_s {
                lms_count += 1;
            }
            is_s = false;
        } else if data[i] < data[i + 1] {
            is_s = true;
        }
        stype[i] = is_s;
    }

    let mut l = 0;
    for b in 0..=alphabet_size {
        let l_next = l + buckets[b];
        buckets[b] = l;
        l = l_next;
    }

    for x in suffix_array[0..length].iter_mut() {
        *x = UNINITIALIZED;
    }
    for b in 0..alphabet_size {
        bucket_index[b] = buckets[b + 1];
    }
    for i in (1..length).rev() {
        if is_lms(&stype, i as i32) {
            let sym = data[i] as usize;
            bucket_index[sym] -= 1;
            suffix_array[bucket_index[sym] as usize] = i as i32;
        }
    }

    induce(
        data,
        suffix_array,
        length,
        alphabet_size,
        &stype,
        &buckets,
        &mut bucket_index,
    );

    let mut j = 0usize;
    for s in 0..length {
        let index = suffix_array[s];
        if index != UNINITIALIZED && is_lms(&stype, index) {
            suffix_array[j] = index;
            j += 1;
        }
    }

    // Name LMS strings using the second half of the suffix array.
    let half = length / 2;
    let sub_capacity = length - half;
    for x in suffix_array[half..length].iter_mut() {
        *x = UNINITIALIZED;
    }
    let mut name = 0i32;
    let mut prev_index = UNINITIALIZED;
    for s in 0..lms_count {
        let index = suffix_array[s];
        if prev_index != UNINITIALIZED && !substrings_equal(data, prev_index, index, &stype) {
            name += 1;
        }
        suffix_array[half + (index / 2) as usize] = name;
        prev_index = index;
    }
    let new_alphabet_size = (name + 1) as usize;

    if new_alphabet_size != lms_count {
        // Compact named LMS symbols into sub_data region.
        let mut jj = 0usize;
        for i in 0..sub_capacity {
            let nm = suffix_array[half + i];
            if nm != UNINITIALIZED {
                suffix_array[half + jj] = nm;
                jj += 1;
            }
        }
        // Recurse on the named symbols (sub_data = suffix_array[half..]).
        // We need disjoint slices for sub_data and the output region.
        // The reference passes &suffix_array[length/2] as sub_data and
        // suffix_array (whole) as the output of computeSuffixArray. Since the
        // recursion only touches [0, lms_count) of the output and reads
        // sub_data[0, lms_count), and lms_count <= length/2, the two regions
        // can overlap. We replicate by copying sub_data out, recursing into a
        // temp output, then copying back.
        let mut sub_data: Vec<i32> = suffix_array[half..half + lms_count].to_vec();
        let mut sub_out = vec![0i32; lms_count];
        compute_suffix_array(&sub_data, &mut sub_out, lms_count, new_alphabet_size);

        // Map named LMS symbol indices to LMS string indices in input string.
        let mut k = 0usize;
        for i in 1..length {
            if is_lms(&stype, i as i32) {
                sub_data[k] = i as i32;
                k += 1;
            }
        }
        for s in 0..lms_count {
            suffix_array[s] = sub_data[sub_out[s] as usize];
        }
    }
    // else: suffix_array[0..lms_count] already holds the sorted LMS indices.

    // Put LMS suffixes in sorted order at the ends of buckets.
    let mut jb = length as i32;
    let mut s: i32 = lms_count as i32 - 1;
    for b in (0..alphabet_size as i32).rev() {
        while s >= 0 && data[suffix_array[s as usize] as usize] == b {
            jb -= 1;
            suffix_array[jb as usize] = suffix_array[s as usize];
            s -= 1;
        }
        while jb > buckets[b as usize] {
            jb -= 1;
            suffix_array[jb as usize] = UNINITIALIZED;
        }
    }

    induce(
        data,
        suffix_array,
        length,
        alphabet_size,
        &stype,
        &buckets,
        &mut bucket_index,
    );
}

// ============================================================================
// MatchFinder (binary-heap min-queue for match_buffer)
// ============================================================================

/// Min-heap of i32 (std::priority_queue<int, vector, greater> => smallest on top).
struct MinHeap {
    data: Vec<i32>,
}

impl MinHeap {
    fn new() -> MinHeap {
        MinHeap { data: Vec::new() }
    }
    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    fn len(&self) -> usize {
        self.data.len()
    }
    fn top(&self) -> i32 {
        self.data[0]
    }
    fn push(&mut self, v: i32) {
        self.data.push(v);
        let mut i = self.data.len() - 1;
        while i > 0 {
            let p = (i - 1) / 2;
            if self.data[p] <= self.data[i] {
                break;
            }
            self.data.swap(p, i);
            i = p;
        }
    }
    fn pop(&mut self) {
        let n = self.data.len();
        self.data.swap(0, n - 1);
        self.data.pop();
        let n = self.data.len();
        let mut i = 0;
        loop {
            let l = i * 2 + 1;
            let r = i * 2 + 2;
            let mut m = i;
            if l < n && self.data[l] < self.data[m] {
                m = l;
            }
            if r < n && self.data[r] < self.data[m] {
                m = r;
            }
            if m == i {
                break;
            }
            self.data.swap(i, m);
            i = m;
        }
    }
}

struct MatchFinder<'a> {
    data: &'a [u8],
    length: i32,
    min_length: i32,
    match_patience: i32,
    max_same_length: i32,

    suffix_array: Vec<i32>,
    rev_suffix_array: Vec<i32>,
    longest_common_prefix: Vec<i32>,

    current_pos: i32,
    min_pos: i32,

    left_index: i32,
    left_length: i32,
    right_index: i32,
    right_length: i32,
    current_length: i32,

    match_buffer: MinHeap,
}

impl<'a> MatchFinder<'a> {
    fn new(
        data: &'a [u8],
        length: i32,
        min_length: i32,
        match_patience: i32,
        max_same_length: i32,
    ) -> MatchFinder<'a> {
        let mut mf = MatchFinder {
            data,
            length,
            min_length,
            match_patience,
            max_same_length,
            suffix_array: Vec::new(),
            rev_suffix_array: Vec::new(),
            longest_common_prefix: Vec::new(),
            current_pos: 0,
            min_pos: 0,
            left_index: 0,
            left_length: 0,
            right_index: 0,
            right_length: 0,
            current_length: 0,
            match_buffer: MinHeap::new(),
        };
        mf.make_suffix_array();
        mf
    }

    fn make_suffix_array(&mut self) {
        let length = self.length as usize;
        // rev_suffix_array temporarily holds the integer string + sentinel.
        self.rev_suffix_array.resize(length + 1, 0);
        for i in 0..length {
            self.rev_suffix_array[i] = self.data[i] as i32 + 1;
        }
        self.rev_suffix_array[length] = 0;

        self.suffix_array.resize(length + 1, 0);
        let rev = self.rev_suffix_array.clone();
        compute_suffix_array(&rev, &mut self.suffix_array, length + 1, 257);

        for i in 0..=length {
            self.rev_suffix_array[self.suffix_array[i] as usize] = i as i32;
        }

        self.longest_common_prefix.resize(length + 1, 0);
        self.longest_common_prefix[0] = 0;
        self.longest_common_prefix[length] = 0;
        let mut h = 0i32;
        for i in 0..length {
            let r = self.rev_suffix_array[i] as usize;
            if r < length {
                let j = self.suffix_array[r + 1];
                let m = length as i32 - std::cmp::max(i as i32, j);
                while h < m && self.data[i + h as usize] == self.data[(j + h) as usize] {
                    h += 1;
                }
                self.longest_common_prefix[r] = h;
                if h > 0 {
                    h -= 1;
                }
            }
        }
    }

    fn extend_left(&mut self) {
        let mut iter = 0;
        while self.left_length >= self.min_length {
            self.left_index -= 1;
            self.left_length = std::cmp::min(
                self.left_length,
                self.longest_common_prefix[self.left_index as usize],
            );
            let pos = self.suffix_array[self.left_index as usize];
            if pos < self.current_pos && pos >= self.min_pos {
                break;
            }
            iter += 1;
            if iter > self.match_patience {
                self.left_length = 0;
                break;
            }
        }
    }

    fn extend_right(&mut self) {
        let mut iter = 0;
        loop {
            self.right_length = std::cmp::min(
                self.right_length,
                self.longest_common_prefix[self.right_index as usize],
            );
            if self.right_length < self.min_length {
                break;
            }
            self.right_index += 1;
            let pos = self.suffix_array[self.right_index as usize];
            if pos < self.current_pos && pos >= self.min_pos {
                break;
            }
            iter += 1;
            if iter > self.match_patience {
                self.right_length = 0;
                break;
            }
        }
    }

    fn next_length(&self) -> i32 {
        std::cmp::max(self.left_length, self.right_length)
    }

    fn begin_matching(&mut self, pos: i32) {
        self.current_pos = pos;
        self.min_pos = 0;
        self.left_index = self.rev_suffix_array[pos as usize];
        self.left_length = self.length - pos;
        self.extend_left();
        self.right_index = self.rev_suffix_array[pos as usize];
        self.right_length = self.length - pos;
        self.extend_right();
    }

    fn next_match(&mut self) -> Option<(i32, i32)> {
        if self.match_buffer.is_empty() {
            self.current_length = self.next_length();
            if self.current_length < self.min_length {
                return None;
            }
            let mut new_min_pos = self.min_pos;
            loop {
                let match_pos;
                if self.left_length > self.right_length {
                    match_pos = self.suffix_array[self.left_index as usize];
                    self.extend_left();
                } else {
                    match_pos = self.suffix_array[self.right_index as usize];
                    self.extend_right();
                }
                new_min_pos = std::cmp::max(new_min_pos, match_pos);
                if (self.match_buffer.len() as i32) < self.max_same_length {
                    self.match_buffer.push(match_pos);
                } else if match_pos > self.match_buffer.top() {
                    self.match_buffer.pop();
                    self.match_buffer.push(match_pos);
                    self.min_pos = self.match_buffer.top();
                } else {
                    // top unchanged; min_pos already reflects it
                    self.min_pos = self.match_buffer.top();
                }
                if self.next_length() != self.current_length {
                    break;
                }
            }
            self.min_pos = new_min_pos;
        }

        let match_length = self.current_length;
        let match_pos = self.match_buffer.top();
        self.match_buffer.pop();
        Some((match_pos, match_length))
    }

    fn reset(&mut self) {}
}

// ============================================================================
// CuckooHash: replica for deterministic iteration order
// ============================================================================

const CUCKOO_UNUSED: i32 = i32::MIN; // 0x80000000
const HASH1_MUL: u32 = 0xF230D3A1;
const HASH2_MUL: u32 = 0x8084027F;
const INITIAL_SIZE_LOG: u32 = 2;

/// Cuckoo hash from i32 key to a value. Value type `V` defaults to a "null"
/// produced by `Default`. Iteration order matches the reference's array order.
struct CuckooHash<V: Copy + Default> {
    element_array: Vec<(i32, V)>, // None == array empty
    allocated: bool,
    n_elements: u32,
    hash_shift: u32,
}

impl<V: Copy + Default> CuckooHash<V> {
    fn new() -> CuckooHash<V> {
        CuckooHash {
            element_array: Vec::new(),
            allocated: false,
            n_elements: 0,
            hash_shift: 32 - INITIAL_SIZE_LOG,
        }
    }

    fn array_size(&self) -> usize {
        1usize << (32 - self.hash_shift)
    }

    fn ensure_array(&mut self) {
        if !self.allocated {
            let size = self.array_size();
            self.element_array = vec![(CUCKOO_UNUSED, V::default()); size];
            self.allocated = true;
        }
    }

    fn hashes(&self, key: i32) -> (u32, u32) {
        let f = ((key as u32) << 1).wrapping_add(1);
        let hash1 = f.wrapping_mul(HASH1_MUL) >> self.hash_shift;
        let hash2 = f.wrapping_mul(HASH2_MUL) >> self.hash_shift;
        (hash1, hash2)
    }

    fn rehash(&mut self) {
        let old_size = self.array_size();
        self.ensure_array();
        let old_array = std::mem::take(&mut self.element_array);
        self.n_elements = 0;
        self.hash_shift -= 1;
        self.allocated = false;
        self.ensure_array();
        for i in 0..old_size {
            if old_array[i].0 != CUCKOO_UNUSED {
                let key = old_array[i].0;
                let val = old_array[i].1;
                *self.get_mut_or_insert(key) = val;
            }
        }
    }

    /// `insert(hash, key, value, n)` - cuckoo displacement loop.
    fn insert_loop(&mut self, mut hash: u32, mut key: i32, mut value: V, mut n: i32) {
        self.ensure_array();
        while self.element_array[hash as usize].0 != CUCKOO_UNUSED {
            n -= 1;
            if n < 0 {
                self.rehash();
                *self.get_mut_or_insert(key) = value;
                return;
            }
            std::mem::swap(&mut key, &mut self.element_array[hash as usize].0);
            std::mem::swap(&mut value, &mut self.element_array[hash as usize].1);
            let (h1, h2) = self.hashes(key);
            hash ^= h1 ^ h2;
        }
        self.element_array[hash as usize] = (key, value);
        self.n_elements += 1;
    }

    fn clear(&mut self) {
        self.element_array = Vec::new();
        self.allocated = false;
        self.n_elements = 0;
        self.hash_shift = 32 - INITIAL_SIZE_LOG;
    }

    fn size(&self) -> i32 {
        self.n_elements as i32
    }

    fn is_empty(&self) -> bool {
        self.n_elements == 0
    }

    fn count(&self, key: i32) -> i32 {
        if self.is_empty() {
            return 0;
        }
        let (h1, h2) = self.hashes(key);
        if self.element_array[h1 as usize].0 == key || self.element_array[h2 as usize].0 == key {
            1
        } else {
            0
        }
    }

    fn erase(&mut self, key: i32) {
        let (h1, h2) = self.hashes(key);
        self.ensure_array();
        let hash = if self.element_array[h1 as usize].0 == key {
            h1
        } else if self.element_array[h2 as usize].0 == key {
            h2
        } else {
            return;
        };
        self.element_array[hash as usize] = (CUCKOO_UNUSED, V::default());
        self.n_elements -= 1;
    }

    /// `operator[]` returning a mutable reference, inserting a default if absent.
    fn get_mut_or_insert(&mut self, key: i32) -> &mut V {
        let (h1, h2) = self.hashes(key);
        self.ensure_array();
        if self.element_array[h1 as usize].0 == key {
            return &mut self.element_array[h1 as usize].1;
        }
        if self.element_array[h2 as usize].0 == key {
            return &mut self.element_array[h2 as usize].1;
        }
        if self.element_array[h1 as usize].0 == CUCKOO_UNUSED {
            self.element_array[h1 as usize] = (key, V::default());
            self.n_elements += 1;
            return &mut self.element_array[h1 as usize].1;
        }
        if self.element_array[h2 as usize].0 == CUCKOO_UNUSED {
            self.element_array[h2 as usize] = (key, V::default());
            self.n_elements += 1;
            return &mut self.element_array[h2 as usize].1;
        }
        let n = self.n_elements as i32;
        self.insert_loop(h1, key, V::default(), n);
        // After possible rehash, look up again.
        self.get_mut_or_insert(key)
    }

    /// Read-only lookup (caller ensures key present).
    fn get(&self, key: i32) -> V {
        let (h1, h2) = self.hashes(key);
        if self.element_array[h1 as usize].0 == key {
            self.element_array[h1 as usize].1
        } else {
            self.element_array[h2 as usize].1
        }
    }

    /// Iterate (key,value) pairs in array order - matches reference iterator.
    fn iter_entries(&self) -> Vec<(i32, V)> {
        if !self.allocated {
            return Vec::new();
        }
        let mut v = Vec::with_capacity(self.n_elements as usize);
        for e in &self.element_array {
            if e.0 != CUCKOO_UNUSED {
                v.push(*e);
            }
        }
        v
    }
}

// ============================================================================
// RefEdge + Heap + LZParser
// ============================================================================

#[derive(Clone, Copy, Default)]
struct EdgeId(u32);

const NULL_EDGE: EdgeId = EdgeId(u32::MAX);

impl EdgeId {
    fn is_null(self) -> bool {
        self.0 == u32::MAX
    }
}

struct RefEdge {
    pos: i32,
    offset: i32,
    length: i32,
    total_size: i32,
    refcount: i32,
    source: EdgeId,
    heap_index: usize,
    free_next: EdgeId, // freelist link when destroyed
    alive: bool,
}

impl RefEdge {
    fn target(&self) -> i32 {
        self.pos + self.length
    }
}

/// Arena/factory for RefEdge with recycling, mirroring RefEdgeFactory.
struct EdgeFactory {
    edges: Vec<RefEdge>,
    free_head: EdgeId,
    edge_capacity: i32,
    edge_count: i32,
}

impl EdgeFactory {
    fn new(edge_capacity: i32) -> EdgeFactory {
        EdgeFactory {
            edges: Vec::new(),
            free_head: NULL_EDGE,
            edge_capacity,
            edge_count: 0,
        }
    }

    fn reset(&mut self) {
        // edge_count must be 0 at this point in the reference.
        self.edge_count = 0;
    }

    fn create(
        &mut self,
        pos: i32,
        offset: i32,
        length: i32,
        total_size: i32,
        source: EdgeId,
    ) -> EdgeId {
        self.edge_count += 1;
        let id = if !self.free_head.is_null() {
            let id = self.free_head;
            self.free_head = self.edges[id.0 as usize].free_next;
            let e = &mut self.edges[id.0 as usize];
            e.pos = pos;
            e.offset = offset;
            e.length = length;
            e.total_size = total_size;
            e.source = source;
            e.refcount = 1;
            e.heap_index = 0;
            e.alive = true;
            id
        } else {
            let id = EdgeId(self.edges.len() as u32);
            self.edges.push(RefEdge {
                pos,
                offset,
                length,
                total_size,
                refcount: 1,
                source,
                heap_index: 0,
                free_next: NULL_EDGE,
                alive: true,
            });
            id
        };
        if !source.is_null() {
            self.edges[source.0 as usize].refcount += 1;
        }
        id
    }

    fn destroy(&mut self, id: EdgeId) {
        let e = &mut self.edges[id.0 as usize];
        e.alive = false;
        e.free_next = self.free_head;
        self.free_head = id;
        self.edge_count -= 1;
    }

    fn full(&self) -> bool {
        self.edge_count >= self.edge_capacity
    }

    #[inline]
    fn e(&self, id: EdgeId) -> &RefEdge {
        &self.edges[id.0 as usize]
    }
}

/// Max-heap of EdgeId keyed by total_size, with removal support.
struct EdgeHeap {
    elements: Vec<EdgeId>,
}

impl EdgeHeap {
    fn new() -> EdgeHeap {
        EdgeHeap {
            elements: Vec::new(),
        }
    }

    fn clear(&mut self) {
        self.elements.clear();
    }

    fn size(&self) -> usize {
        self.elements.len()
    }

    fn swap_elems(&mut self, factory: &mut EdgeFactory, i1: usize, i2: usize) {
        let t1 = self.elements[i1];
        let t2 = self.elements[i2];
        self.elements[i1] = t2;
        self.elements[i2] = t1;
        factory.edges[t2.0 as usize].heap_index = i1;
        factory.edges[t1.0 as usize].heap_index = i2;
    }

    /// compare(e1,e2) == e1.total_size < e2.total_size
    fn less(&self, factory: &EdgeFactory, a: EdgeId, b: EdgeId) -> bool {
        factory.e(a).total_size < factory.e(b).total_size
    }

    fn up(&mut self, factory: &mut EdgeFactory, mut i: usize) {
        while i > 0 {
            let pi = (i - 1) / 2;
            if !self.less(factory, self.elements[pi], self.elements[i]) {
                return;
            }
            self.swap_elems(factory, i, pi);
            i = pi;
        }
    }

    fn down(&mut self, factory: &mut EdgeFactory, mut i: usize) {
        while i * 2 + 1 < self.elements.len() {
            let ci1 = i * 2 + 1;
            let ci2 = i * 2 + 2;
            let ci = if ci2 < self.elements.len()
                && self.less(factory, self.elements[ci1], self.elements[ci2])
            {
                ci2
            } else {
                ci1
            };
            if !self.less(factory, self.elements[i], self.elements[ci]) {
                return;
            }
            self.swap_elems(factory, i, ci);
            i = ci;
        }
    }

    fn insert(&mut self, factory: &mut EdgeFactory, t: EdgeId) {
        self.elements.push(t);
        let idx = self.elements.len() - 1;
        factory.edges[t.0 as usize].heap_index = idx;
        self.up(factory, idx);
    }

    fn remove_index(&mut self, factory: &mut EdgeFactory, i: usize) -> EdgeId {
        let removed = self.elements[i];
        let last = self.elements[self.elements.len() - 1];
        self.elements[i] = last;
        self.elements.pop();
        if i < self.elements.len() {
            factory.edges[last.0 as usize].heap_index = i;
            self.down(factory, i);
        }
        removed
    }

    fn contains(&self, factory: &EdgeFactory, t: EdgeId) -> bool {
        let hi = factory.e(t).heap_index;
        hi < self.elements.len() && self.elements[hi].0 == t.0
    }

    fn remove(&mut self, factory: &mut EdgeFactory, t: EdgeId) {
        if self.contains(factory, t) {
            let hi = factory.e(t).heap_index;
            self.remove_index(factory, hi);
        }
    }

    fn remove_largest(&mut self, factory: &mut EdgeFactory) -> EdgeId {
        self.remove_index(factory, 0)
    }
}

struct LZResultEdge {
    pos: i32,
    offset: i32,
    length: i32,
}

struct LZParseResult {
    edges: Vec<LZResultEdge>,
    data: Vec<u8>,
    data_length: i32,
    zero_padding: i32,
}

impl LZParseResult {
    fn new() -> LZParseResult {
        LZParseResult {
            edges: Vec::new(),
            data: Vec::new(),
            data_length: 0,
            zero_padding: 0,
        }
    }

    /// Replay the parse through a (final) encoder. Mirrors LZParseResult::encode.
    fn encode(&self, encoder: &mut LZEncoder) -> u64 {
        let mut size: u64 = 0;
        let mut pos = 0i32;
        let mut state = LZState::default();
        LZEncoder::set_initial_state(&mut state);
        for i in (0..self.edges.len()).rev() {
            let edge = &self.edges[i];
            while pos < edge.pos {
                let s = state;
                size += encoder.encode_literal(self.data[pos as usize], &s, &mut state) as u64;
                pos += 1;
            }
            let s = state;
            size += encoder.encode_reference(edge.offset, edge.length, &s, &mut state) as u64;
            pos += edge.length;
        }
        while pos < self.data_length {
            let s = state;
            size += encoder.encode_literal(self.data[pos as usize], &s, &mut state) as u64;
            pos += 1;
        }
        if self.zero_padding > 0 {
            let s = state;
            size += encoder.encode_literal(0, &s, &mut state) as u64;
            if self.zero_padding == 2 {
                let s = state;
                size += encoder.encode_literal(0, &s, &mut state) as u64;
            } else if self.zero_padding > 1 {
                let s = state;
                size += encoder.encode_reference(1, self.zero_padding - 1, &s, &mut state) as u64;
            }
        }
        let s = state;
        size += encoder.finish(&s) as u64;
        size
    }
}

struct LZParser<'a> {
    data: &'a [u8],
    data_length: i32,
    zero_padding: i32,
    length_margin: i32,
    skip_length: i32,

    literal_size: Vec<i32>,
    edges_to_pos: Vec<CuckooHash<EdgeId>>,
    best: EdgeId,
    best_for_offset: CuckooHash<EdgeId>,
    root_edges: EdgeHeap,
    initial_best: EdgeId,
}

impl<'a> LZParser<'a> {
    fn new(
        data: &'a [u8],
        data_length: i32,
        zero_padding: i32,
        length_margin: i32,
        skip_length: i32,
    ) -> LZParser<'a> {
        let mut edges_to_pos = Vec::with_capacity((data_length + 1) as usize);
        for _ in 0..=data_length {
            edges_to_pos.push(CuckooHash::new());
        }
        LZParser {
            data,
            data_length,
            zero_padding,
            length_margin,
            skip_length,
            literal_size: Vec::new(),
            edges_to_pos,
            best: NULL_EDGE,
            best_for_offset: CuckooHash::new(),
            root_edges: EdgeHeap::new(),
            initial_best: NULL_EDGE,
        }
    }

    fn remove_root(&mut self, factory: &mut EdgeFactory, edge: EdgeId) {
        self.root_edges.remove(factory, edge);
    }

    fn release_edge(&mut self, factory: &mut EdgeFactory, mut edge: EdgeId) {
        while !edge.is_null() {
            let source = factory.e(edge).source;
            factory.edges[edge.0 as usize].refcount -= 1;
            if factory.e(edge).refcount == 0 {
                factory.destroy(edge);
            } else {
                return;
            }
            edge = source;
        }
    }

    fn clean_worst_edge(&mut self, factory: &mut EdgeFactory, pos: i32, exclude: EdgeId) -> bool {
        if self.root_edges.size() == 0 {
            return false;
        }
        let worst_edge = self.root_edges.remove_largest(factory);
        if worst_edge.0 == self.best.0 || worst_edge.0 == exclude.0 {
            return true;
        }
        let target = factory.e(worst_edge).target();
        let offset = factory.e(worst_edge).offset;
        let use_pos_container = target > pos;
        let container_size = if use_pos_container {
            self.edges_to_pos[target as usize].size()
        } else {
            self.best_for_offset.size()
        };
        let container_count = if use_pos_container {
            self.edges_to_pos[target as usize].count(offset)
        } else {
            self.best_for_offset.count(offset)
        };
        if container_size > 1 && container_count > 0 {
            if use_pos_container {
                self.edges_to_pos[target as usize].erase(offset);
            } else {
                self.best_for_offset.erase(offset);
            }
            self.release_edge(factory, worst_edge);
        }
        true
    }

    /// put_by_offset into `which` container (true = edges_to_pos[target], false = best_for_offset).
    fn put_by_offset(
        &mut self,
        factory: &mut EdgeFactory,
        which_pos: i32, // >=0 -> edges_to_pos[which_pos]; -1 -> best_for_offset
        edge: EdgeId,
    ) {
        let offset = factory.e(edge).offset;
        let total_size = factory.e(edge).total_size;
        let present = if which_pos >= 0 {
            self.edges_to_pos[which_pos as usize].count(offset) != 0
        } else {
            self.best_for_offset.count(offset) != 0
        };
        if !present {
            if which_pos >= 0 {
                *self.edges_to_pos[which_pos as usize].get_mut_or_insert(offset) = edge;
            } else {
                *self.best_for_offset.get_mut_or_insert(offset) = edge;
            }
            self.root_edges.insert(factory, edge);
        } else {
            let old_edge = if which_pos >= 0 {
                self.edges_to_pos[which_pos as usize].get(offset)
            } else {
                self.best_for_offset.get(offset)
            };
            if total_size < factory.e(old_edge).total_size {
                self.remove_root(factory, old_edge);
                self.release_edge(factory, old_edge);
                if which_pos >= 0 {
                    *self.edges_to_pos[which_pos as usize].get_mut_or_insert(offset) = edge;
                } else {
                    *self.best_for_offset.get_mut_or_insert(offset) = edge;
                }
                self.root_edges.insert(factory, edge);
            } else {
                self.release_edge(factory, edge);
            }
        }
    }

    fn new_edge(
        &mut self,
        encoder: &mut LZEncoder,
        factory: &mut EdgeFactory,
        source: EdgeId,
        pos: i32,
        offset: i32,
        length: i32,
    ) {
        if !source.is_null() {
            let s = factory.e(source);
            if offset == s.offset && pos == s.target() {
                return;
            }
        }
        let prev_target = if !source.is_null() {
            factory.e(source).target()
        } else {
            0
        };
        let src_offset = if !source.is_null() {
            factory.e(source).offset
        } else {
            0
        };
        let src_total = if !source.is_null() {
            factory.e(source).total_size
        } else {
            self.literal_size[self.data_length as usize]
        };
        let new_target = pos + length;

        let mut state_before = LZState::default();
        let mut state_after = LZState::default();
        LZEncoder::construct_state(&mut state_before, pos, pos == prev_target, src_offset);
        let size_before = src_total
            - (self.literal_size[self.data_length as usize] - self.literal_size[pos as usize]);
        let edge_size = encoder.encode_reference(offset, length, &state_before, &mut state_after);
        let size_after =
            self.literal_size[self.data_length as usize] - self.literal_size[new_target as usize];

        while factory.full() {
            if !self.clean_worst_edge(factory, pos, source) {
                break;
            }
        }
        let new_edge = factory.create(
            pos,
            offset,
            length,
            size_before + edge_size + size_after,
            source,
        );
        self.put_by_offset(factory, new_target, new_edge);
    }

    fn parse(
        &mut self,
        encoder: &mut LZEncoder,
        finder: &mut MatchFinder,
        factory: &mut EdgeFactory,
    ) -> LZParseResult {
        // Reset state
        self.best_for_offset.clear();
        self.root_edges.clear();
        factory.reset();

        // Accumulate literal sizes.
        self.literal_size.clear();
        self.literal_size.resize((self.data_length + 1) as usize, 0);
        let mut size = 0;
        let mut literal_state = LZState::default();
        LZEncoder::set_initial_state(&mut literal_state);
        for i in 0..self.data_length {
            self.literal_size[i as usize] = size;
            let s = literal_state;
            size += encoder.encode_literal(self.data[i as usize], &s, &mut literal_state);
        }
        self.literal_size[self.data_length as usize] = size;

        // Parse
        let initial_best = factory.create(
            0,
            0,
            0,
            self.literal_size[self.data_length as usize],
            NULL_EDGE,
        );
        self.initial_best = initial_best;
        self.best = initial_best;

        let mut pos = 1;
        while pos <= self.data_length {
            // Assimilate edges ending here.
            let entries = self.edges_to_pos[pos as usize].iter_entries();
            for (_offset, edge) in entries {
                if factory.e(edge).total_size < factory.e(self.best).total_size {
                    self.best = edge;
                }
                self.remove_root(factory, edge);
                self.put_by_offset(factory, -1, edge);
            }
            self.edges_to_pos[pos as usize].clear();

            // Add new edges according to matches.
            finder.begin_matching(pos);
            let mut max_match_length = 0;
            while let Some((match_pos, mut match_length)) = finder.next_match() {
                let offset = pos - match_pos;
                if match_length > self.data_length - pos {
                    match_length = self.data_length - pos;
                }
                let mut min_length = match_length - self.length_margin;
                if min_length < 2 {
                    min_length = 2;
                }
                for length in min_length..=match_length {
                    let best = self.best;
                    self.new_edge(encoder, factory, best, pos, offset, length);
                    let best_offset = factory.e(self.best).offset;
                    if best_offset != offset && self.best_for_offset.count(offset) != 0 {
                        let bfo = self.best_for_offset.get(offset);
                        self.new_edge(encoder, factory, bfo, pos, offset, length);
                    }
                }
                max_match_length = std::cmp::max(max_match_length, match_length);
            }

            // If we have a very long match, skip ahead.
            if max_match_length >= self.skip_length
                && !self.edges_to_pos[(pos + max_match_length) as usize].is_empty()
            {
                self.root_edges.clear();
                let bfo_entries = self.best_for_offset.iter_entries();
                for (_o, e) in bfo_entries {
                    self.release_edge(factory, e);
                }
                self.best_for_offset.clear();
                let target_pos = pos + max_match_length;
                while pos < target_pos - 1 {
                    pos += 1;
                    let entries = self.edges_to_pos[pos as usize].iter_entries();
                    for (_o, e) in entries {
                        self.release_edge(factory, e);
                    }
                    self.edges_to_pos[pos as usize].clear();
                }
                self.best = self.initial_best;
            }

            pos += 1;
        }

        // Clean unused paths.
        self.root_edges.clear();
        let bfo_entries = self.best_for_offset.iter_entries();
        for (_o, edge) in bfo_entries {
            if edge.0 != self.best.0 {
                self.release_edge(factory, edge);
            }
        }

        // Find best path.
        let mut result = LZParseResult::new();
        result.data = self.data.to_vec();
        result.data_length = self.data_length;
        result.zero_padding = self.zero_padding;
        let mut edge = self.best;
        while factory.e(edge).length > 0 {
            let e = factory.e(edge);
            result.edges.push(LZResultEdge {
                pos: e.pos,
                offset: e.offset,
                length: e.length,
            });
            edge = e.source;
        }
        // releaseEdge(edge); releaseEdge(best) - bookkeeping only.

        result
    }
}

// ============================================================================
// Pack: multi-iteration driver
// ============================================================================

/// Parse/encode knobs for the multi-iteration `pack_data` driver. These are the
/// real Shrinkler search controls; exposed publicly so callers of
/// [`compress_native`] can drive `pack_data` with their own configuration,
/// starting from the `-9` preset returned by [`params_no_parity`].
pub struct PackParams {
    pub parity_context: bool,
    pub iterations: i32,
    pub length_margin: i32,
    pub skip_length: i32,
    pub match_patience: i32,
    pub max_same_length: i32,
}

fn pack_data(data: &[u8], params: &PackParams, references: i32) -> Vec<u8> {
    let data_length = data.len() as i32;
    let zero_padding = 0;

    let mut finder = MatchFinder::new(
        data,
        data_length,
        2,
        params.match_patience,
        params.max_same_length,
    );
    let mut parser = LZParser::new(
        data,
        data_length,
        zero_padding,
        params.length_margin,
        params.skip_length,
    );
    let mut factory = EdgeFactory::new(references);

    let mut best_size: u64 = 1u64 << (32 + 3 + BIT_PRECISION);
    let mut best_result = 0usize;
    // results[0], results[1]
    let mut results: [LZParseResult; 2] = [LZParseResult::new(), LZParseResult::new()];

    let mut counting_coder = CountingCoder::new(NUM_CONTEXTS as usize);

    for _i in 0..params.iterations {
        // Parse data into LZ symbols using a size-measuring coder.
        let target = 1 - best_result;

        let mut measurer = SizeMeasuringCoder::from_counting(&counting_coder);
        measurer.set_number_contexts(NUMBER_CONTEXT_OFFSET, NUM_NUMBER_CONTEXTS, data_length);
        finder.reset();
        let result = {
            let mut enc =
                LZEncoder::new(LZCoderRef::Measuring(&mut measurer), params.parity_context);
            parser.parse(&mut enc, &mut finder, &mut factory)
        };
        results[target] = result;

        // Encode result using adaptive range coding (to measure real size).
        let real_size = {
            let mut range_coder = RangeCoder::new(NUM_CONTEXTS as usize);
            let rs = {
                let mut enc =
                    LZEncoder::new(LZCoderRef::Range(&mut range_coder), params.parity_context);
                results[target].encode(&mut enc)
            };
            range_coder.finish();
            rs
        };

        if real_size < best_size {
            best_result = 1 - best_result;
            best_size = real_size;
        }

        // Count symbol frequencies into the existing (old) counting coder, then
        // mix with a freshly-zeroed `new` coder. This follows the reference
        // exactly: `result.encode(counting_coder)` mutates the old coder, and the
        // merge `(old*3 + new)/4` combines it with the zero `new_counting_coder`
        // (which is never encoded into), decaying old counts toward the new pass.
        let new_counting_coder = CountingCoder::new(NUM_CONTEXTS as usize);
        {
            let mut enc = LZEncoder::new(
                LZCoderRef::Counting(&mut counting_coder),
                params.parity_context,
            );
            results[target].encode(&mut enc);
        }
        counting_coder = CountingCoder::merge(&counting_coder, &new_counting_coder);
    }

    // Encode best result to output with the final range coder.
    let mut range_coder = RangeCoder::new(TOTAL_CONTEXTS);
    range_coder.reset();
    {
        let mut enc = LZEncoder::new(LZCoderRef::Range(&mut range_coder), params.parity_context);
        results[best_result].encode(&mut enc);
    }
    range_coder.finish();
    range_coder.out
}

// ============================================================================
// Pure-Rust decoder
// ============================================================================

struct RangeDecoder<'a> {
    contexts: Vec<u16>,
    data: &'a [u8],
    bit_index: i64,
    intervalsize: u32,
    intervalvalue: u32,
    uncertainty: u32,
}

impl<'a> RangeDecoder<'a> {
    fn new(n_contexts: usize, data: &'a [u8]) -> RangeDecoder<'a> {
        RangeDecoder {
            contexts: vec![0x8000; n_contexts],
            data,
            bit_index: 0,
            intervalsize: 1,
            intervalvalue: 0,
            uncertainty: 1,
        }
    }

    fn get_bit(&mut self) -> u32 {
        let byte_index = (self.bit_index >> 3) as usize;
        let bit_in_byte = (!self.bit_index) & 7;
        let total_bits = self.data.len() as i64 * 8;
        if self.bit_index >= total_bits {
            self.bit_index += 1;
            self.uncertainty <<= 1;
            return 0;
        }
        self.bit_index += 1;
        ((self.data[byte_index] >> bit_in_byte) & 1) as u32
    }

    fn decode(&mut self, context_index: i32) -> i32 {
        let prob = self.contexts[context_index as usize] as u32;
        while self.intervalsize < 0x8000 {
            self.intervalsize <<= 1;
            self.intervalvalue = (self.intervalvalue << 1) | self.get_bit();
        }
        let bit;
        let new_prob;
        let threshold = (self.intervalsize.wrapping_mul(prob)) >> 16;
        if self.intervalvalue >= threshold {
            bit = 0;
            self.intervalvalue -= threshold;
            self.intervalsize -= threshold;
            new_prob = prob - (prob >> ADJUST_SHIFT);
        } else {
            bit = 1;
            self.intervalsize = threshold;
            new_prob = prob + (0xffff >> ADJUST_SHIFT) - (prob >> ADJUST_SHIFT);
        }
        self.contexts[context_index as usize] = new_prob as u16;
        bit
    }

    fn decode_number(&mut self, base_context: i32) -> i32 {
        let mut i = 0;
        loop {
            let context = base_context + (i * 2 + 2);
            if self.decode(context) == 0 {
                break;
            }
            i += 1;
        }
        let mut number = 1;
        while i >= 0 {
            let context = base_context + (i * 2 + 1);
            let bit = self.decode(context);
            number = (number << 1) | bit;
            i -= 1;
        }
        number
    }
}

/// LZ decoder - drives the range decoder, mirroring LZDecoder::decode.
fn lz_decode(decoder: &mut RangeDecoder, parity_context: bool) -> Vec<u8> {
    let parity_mask = if parity_context { 1 } else { 0 };
    let mut out: Vec<u8> = Vec::new();
    let mut reference = false;
    let mut prev_was_ref = false;
    let mut pos = 0i32;
    let mut offset = 0i32;

    let dec = |d: &mut RangeDecoder, context: i32| d.decode(NUM_SINGLE_CONTEXTS + context);
    let dec_num =
        |d: &mut RangeDecoder, group: i32| d.decode_number(NUM_SINGLE_CONTEXTS + (group << 8));

    loop {
        if reference {
            let mut repeated = false;
            if !prev_was_ref {
                repeated = dec(decoder, CONTEXT_REPEATED) != 0;
            }
            if !repeated {
                offset = dec_num(decoder, CONTEXT_GROUP_OFFSET) - 2;
                if offset == 0 {
                    break;
                }
            }
            let length = dec_num(decoder, CONTEXT_GROUP_LENGTH);
            for _ in 0..length {
                let b = out[(out.len() as i32 - offset) as usize];
                out.push(b);
            }
            pos += length;
            prev_was_ref = true;
        } else {
            let parity = pos & parity_mask;
            let mut context = 1i32;
            for _ in 0..8 {
                let bit = dec(decoder, (parity << 8) | context);
                context = (context << 1) | bit;
            }
            out.push(context as u8);
            pos += 1;
            prev_was_ref = false;
        }
        let parity = pos & parity_mask;
        reference = dec(decoder, CONTEXT_KIND + (parity << 8)) != 0;
    }
    out
}

// ============================================================================
// Public API
// ============================================================================

/// Two-tier level system. **level 1 = fastest** (the single stock `-9` anchor
/// parse - one `pack_data` config, byte-identical to native `Shrinkler -9 -d -b`),
/// **level 2 = absolute best** (the multi-config best-of, which is `<=` level 1
/// on every input). `decompress` is level-independent (one stream format).
pub const MAX_LEVEL: u8 = 2;

/// Default reference-buffer budget (Shrinkler `-r`, preset default 100000).
const REFERENCES: i32 = 100_000;

/// Map our single public level to Shrinkler preset-9 parameters in no-parity
/// mode (`Shrinkler -d -b -9`). This is the maximum-ratio stock configuration,
/// and serves as the **no-regression anchor** of the best-of below: its output
/// is byte-identical to the reference cruncher's `-9 -d -b` data stream, so the
/// best-of can never produce a result larger than native `-9`.
///
/// Public so callers can use it as the starting point for [`compress_native`]:
/// take this `-9` preset, tweak the knobs, and drive `pack_data` directly.
pub fn params_no_parity() -> PackParams {
    let preset = 9;
    PackParams {
        parity_context: false,
        iterations: preset,
        length_margin: preset,
        skip_length: 1000 * preset,
        match_patience: 100 * preset,
        max_same_length: 10 * preset,
    }
}

/// A single best-of candidate: the `PackParams` to drive `pack_data` plus the
/// reference-edge budget (`-r`) to use for it.
struct Candidate {
    params: PackParams,
    references: i32,
}

/// Build the best-of candidate set. The FIRST entry is always the exact stock
/// `-9` anchor (so the result is `<=` native `-9` by construction); the rest are
/// strictly stronger parses (larger margins / more patience / more edges / more
/// iterations) that can find a smaller encoding on some files. None of them can
/// make the result larger, because `compress_forward` keeps the smallest valid
/// (round-trip-verified) candidate and breaks ties toward the earlier (anchor)
/// config.
///
/// Every candidate keeps `parity_context = false`, so each emitted stream is a
/// valid no-parity Shrinkler stream (decodable by `unshrinkler PARITY=0` and by
/// the in-crate [`decompress`]). Only the *parse* differs between candidates;
/// the bitstream grammar is identical.
///
/// Knob meanings:
///   * `length_margin`  - consider matches down to `found_len - length_margin`.
///   * `match_patience` - match-finder perseverance (`-e` effort).
///   * `max_same_length`- how many equal-length matches to keep (`-a`).
///   * `skip_length`    - force-and-skip threshold for very long matches (`-s`).
///   * `iterations`     - model-refinement passes (driver keeps the best one).
///   * `references`     - reference-edge buffer size (`-r`).
///
/// The aggressive `match_patience` knob is the dominant cost driver, so it is
/// scaled down for large inputs (`data_length`) to keep the offline encode time
/// bounded. This never affects correctness or the no-regression guarantee (the
/// anchor is always present and unscaled); it only trims how hard the *extra*
/// candidates search on big files, where the per-byte gains are smallest.
fn best_of_candidates(data_length: i32) -> Vec<Candidate> {
    // Anchor: exact stock -9. Always present and never scaled.
    let mut cands = vec![Candidate {
        params: params_no_parity(),
        references: REFERENCES,
    }];

    // Patience cap by input size: full search on small inputs, gentler on large
    // ones (where pat=3000 over hundreds of KB is very slow for sub-promille gain).
    //   < 128 KB : 3000   (geo-class binaries; ~30s/config)
    //   < 256 KB : 2000
    //   < 512 KB : 1200
    //   >=512 KB :  900   (= -9 default; rely on iterations + max_same_length)
    let pat_hi = if data_length < 128 * 1024 {
        3000
    } else if data_length < 256 * 1024 {
        2000
    } else if data_length < 512 * 1024 {
        1200
    } else {
        900
    };

    // Iteration cap by input size: the extra model-refinement passes past the -9
    // default (9) give vanishing per-byte gains on big files but cost a full parse
    // each, so taper them down. The anchor (9 iterations) is unaffected.
    //   < 256 KB : 16
    //   < 512 KB : 13
    //   >=512 KB : 11
    let iter_hi = if data_length < 256 * 1024 {
        16
    } else if data_length < 512 * 1024 {
        13
    } else {
        11
    };

    // Helper to push a variant.
    let mut add = |length_margin: i32,
                   max_same_length: i32,
                   match_patience: i32,
                   skip_length: i32,
                   iterations: i32,
                   references: i32| {
        cands.push(Candidate {
            params: PackParams {
                parity_context: false,
                iterations,
                length_margin,
                skip_length,
                match_patience,
                max_same_length,
            },
            references,
        });
    };

    // The candidate set was tuned empirically over the corpus. No single stronger
    // config dominates the anchor on every file (the parse is a heuristic against
    // an adaptive model, so a "stronger" parse can occasionally cost a byte or
    // two); these complementary configs each win on some files, and the best-of
    // keeps the smallest. Arguments: (length_margin, max_same_length,
    // match_patience, skip_length, iterations, references).

    // More model-refinement passes - biggest win on text-like data (the driver
    // keeps the best pass, so extra iterations never hurt that config).
    add(9, 90, 900, 9000, iter_hi, REFERENCES);
    // Keep many more equal-length matches + more match-finder patience - biggest
    // win on binary/structured data.
    add(9, 500, pat_hi, 9000, 9, REFERENCES);
    // Wider short-match consideration (length_margin) with moderate same-length.
    add(30, 300, 900, 9000, 9, REFERENCES);
    // Combined strong parse: more passes + more matches kept + more patience.
    // (The default 100000-edge buffer is already ample on the corpus - raising
    // `-r` to 1e6 produced byte-identical results in tuning - so we keep it.)
    add(9, 500, pat_hi, 9000, iter_hi, REFERENCES);

    cands
}

/// Compress `input` as a forward Shrinkler no-parity stream.
///
/// Runs a best-of over several `PackParams` configurations (see
/// [`best_of_candidates`]) and emits the SMALLEST result that round-trips
/// losslessly. The candidate set includes the exact stock `-9` params as the
/// no-regression anchor, so the output is always `<=` native `-9`. Ties are
/// broken toward the earlier (anchor) candidate, so when no stronger config
/// wins the output is byte-identical to native `-9`.
///
/// Empty input is a degenerate case: the Shrinkler bitstream always begins with
/// a literal, so a stream that decodes to *nothing* is not representable - the
/// reference decoder itself asserts on the pure end-marker stream. We therefore
/// map empty input to an empty stream and back, keeping the uniform API lossless
/// without affecting byte-identity (the corpus contains no empty files).
fn compress_forward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }

    let candidates = best_of_candidates(input.len() as i32);

    // Run every candidate (they share nothing mutable) on scratch threads. Each
    // produces an independent valid no-parity stream; we keep only the smallest
    // that decodes back to `input` exactly. The anchor (index 0) is guaranteed
    // to be valid, so a valid result always exists.
    let results: Vec<(usize, Option<Vec<u8>>)> = std::thread::scope(|s| {
        let handles: Vec<_> = candidates
            .iter()
            .enumerate()
            .map(|(idx, c)| {
                s.spawn(move || {
                    let packed = pack_data(input, &c.params, c.references);
                    // Verify the candidate decodes losslessly before trusting it.
                    let ok = decompress_forward(&packed) == input;
                    (idx, if ok { Some(packed) } else { None })
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("shrinkler candidate thread panicked"))
            .collect()
    });

    // Pick the smallest valid candidate; ties resolve to the lowest index, so
    // the anchor (-9) wins ties and the result is byte-identical to native -9
    // whenever no stronger config produces a strictly smaller stream.
    let mut best: Option<(usize, Vec<u8>)> = None;
    for (idx, maybe) in results {
        if let Some(packed) = maybe {
            match &best {
                None => best = Some((idx, packed)),
                Some((_, cur)) => {
                    if packed.len() < cur.len() {
                        best = Some((idx, packed));
                    }
                }
            }
        }
    }
    best.expect("anchor candidate must always be valid").1
}

/// The level-1 anchor: a single `pack_data` run with the stock `-9` no-parity
/// preset (`params_no_parity`). This is the FAST tier - one parse, no best-of -
/// and its output is byte-identical to native `Shrinkler -9 -d -b`. Empty input
/// maps to an empty stream (see [`compress_forward`] for why).
fn compress_forward_anchor(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    pack_data(input, &params_no_parity(), REFERENCES)
}

/// Decompress a forward Shrinkler no-parity stream (pure Rust).
fn decompress_forward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut decoder = RangeDecoder::new(TOTAL_CONTEXTS, input);
    lz_decode(&mut decoder, false)
}

/// Compress `input` at `level` (clamped to 1..=[`MAX_LEVEL`]).
///
/// * **level 1** - FAST anchor: a single stock `-9` no-parity parse
///   ([`compress_forward_anchor`]), byte-identical to native `Shrinkler -9 -d -b`.
/// * **level 2** - BEST: the multi-config best-of ([`compress_forward`]), which
///   is `<=` level 1 on every input (the `-9` anchor is one of its candidates).
///
/// When `backward` is true, produce a reverse stream (reverse input, compress
/// forward, reverse output) decodable by [`decompress`] with `backward = true`.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let level = level.clamp(1, MAX_LEVEL);
    let pack_one = |data: &[u8]| -> Vec<u8> {
        if level == 1 {
            compress_forward_anchor(data)
        } else {
            compress_forward(data)
        }
    };
    if backward {
        let mut data = input.to_vec();
        data.reverse();
        let mut packed = pack_one(&data);
        packed.reverse();
        packed
    } else {
        pack_one(input)
    }
}

/// Native API: drive `pack_data` directly with the caller's [`PackParams`], in
/// no-parity mode (the only mode the in-crate [`decompress`] and the
/// `unshrinkler PARITY=0` decode). Start from [`params_no_parity`] (the `-9`
/// preset) and tweak the knobs (`iterations`, `length_margin`, `match_patience`,
/// `max_same_length`, `skip_length`); `parity_context` is forced to `false` so
/// the emitted stream is always a valid no-parity Shrinkler stream.
///
/// When `backward` is true, produce a reverse stream (reverse input, compress
/// forward, reverse output). Empty input maps to an empty stream.
pub fn compress_native(input: &[u8], params: &PackParams, backward: bool) -> Vec<u8> {
    let pack_one = |data: &[u8]| -> Vec<u8> {
        if data.is_empty() {
            return Vec::new();
        }
        // Preserve no-parity regardless of the caller's `parity_context`.
        let p = PackParams {
            parity_context: false,
            iterations: params.iterations,
            length_margin: params.length_margin,
            skip_length: params.skip_length,
            match_patience: params.match_patience,
            max_same_length: params.max_same_length,
        };
        pack_data(data, &p, REFERENCES)
    };
    if backward {
        let mut data = input.to_vec();
        data.reverse();
        let mut packed = pack_one(&data);
        packed.reverse();
        packed
    } else {
        pack_one(input)
    }
}

/// Decompress a Shrinkler stream. When `backward` is true, decode a reverse
/// stream (reverse input, decompress forward, reverse output).
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        let mut data = input.to_vec();
        data.reverse();
        let mut out = decompress_forward(&data);
        out.reverse();
        out
    } else {
        decompress_forward(input)
    }
}

/// Forward Shrinkler no-parity compression (decodes with `unshrinkler PARITY=0`).
pub fn compress_shrinkler(input: &[u8]) -> Vec<u8> {
    compress_forward(input)
}

/// Backward Shrinkler compression: reverse input, compress forward, reverse output.
pub fn compress_shrinkler_backward(input: &[u8]) -> Vec<u8> {
    compress(input, MAX_LEVEL, true)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(input: &[u8]) {
        // Both levels roundtrip both directions.
        for level in 1..=MAX_LEVEL {
            let packed = compress(input, level, false);
            assert_eq!(
                decompress(&packed, false),
                input,
                "forward roundtrip L{level}"
            );

            let packed_rev = compress(input, level, true);
            assert_eq!(
                decompress(&packed_rev, true),
                input,
                "backward roundtrip L{level}"
            );
        }

        // Level 1 == the stock -9 anchor (fast tier).
        assert_eq!(
            compress(input, 1, false),
            compress_forward_anchor(input),
            "level 1 == -9 anchor"
        );
        // Level 2 == the current best-of (== compress_shrinkler).
        let best = compress(input, 2, false);
        assert_eq!(compress_shrinkler(input), best, "level 2 == best-of");
        // Best-of (level 2) never larger than the anchor (level 1).
        assert!(
            best.len() <= compress(input, 1, false).len(),
            "level 2 <= level 1"
        );
        // Clamp: 0 -> 1, 255 -> MAX_LEVEL.
        assert_eq!(compress(input, 0, false), compress(input, 1, false));
        assert_eq!(
            compress(input, 255, false),
            compress(input, MAX_LEVEL, false)
        );

        assert_eq!(decompress(&compress_shrinkler_backward(input), true), input);

        // Native API: the -9 preset drives pack_data to the same bytes as the anchor.
        assert_eq!(
            compress_native(input, &params_no_parity(), false),
            compress(input, 1, false),
            "native(-9 preset) == level 1 anchor"
        );
        assert_eq!(
            decompress(&compress_native(input, &params_no_parity(), true), true),
            input
        );
    }

    #[test]
    fn roundtrip_basic() {
        roundtrip(b"");
        roundtrip(b"a");
        roundtrip(b"hello, hello, hello, world!");
        roundtrip(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        roundtrip(&[0u8; 256]);
        let mut v = Vec::new();
        for i in 0..1000u32 {
            v.push((i.wrapping_mul(2654435761) >> 24) as u8);
        }
        roundtrip(&v);
    }

    #[test]
    fn roundtrip_abracadabra() {
        roundtrip(b"abracadabra abracadabra abracadabra");
    }
}
