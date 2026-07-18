//! upkr-format compressor and decompressor (forward and reverse).
//!
//! Implements the upkr container: rANS bit coder, adaptive bit-probability contexts, the LZ
//! literal/match/rep-offset/length grammar, an optimal multi-arrival parser, and a suffix-array
//! match finder for the upkr format. See THIRD_PARTY.md for attribution and license.

use std::collections::{HashMap, HashSet};
use std::mem;
use std::rc::Rc;

// =====================================================================================
// Config: the upkr format variant. Only Config::default() is used.
// =====================================================================================

/// The upkr format variant. The public API uses [`Config::default`].
#[derive(Debug, Clone)]
pub struct Config {
    pub use_bitstream: bool,
    pub parity_contexts: usize,
    pub invert_bit_encoding: bool,
    pub is_match_bit: bool,
    pub new_offset_bit: bool,
    pub continue_value_bit: bool,
    pub bitstream_is_big_endian: bool,
    pub simplified_prob_update: bool,
    pub no_repeated_offsets: bool,
    pub eof_in_length: bool,
    pub max_offset: usize,
    pub max_length: usize,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            use_bitstream: false,
            parity_contexts: 1,
            invert_bit_encoding: false,
            is_match_bit: true,
            new_offset_bit: true,
            continue_value_bit: true,
            bitstream_is_big_endian: false,
            simplified_prob_update: false,
            no_repeated_offsets: false,
            eof_in_length: false,
            max_offset: usize::MAX,
            max_length: usize::MAX,
        }
    }
}

impl Config {
    fn min_length(&self) -> usize {
        if self.eof_in_length {
            2
        } else {
            1
        }
    }
}

// =====================================================================================
// rANS bit coder and fractional cost counter.
// =====================================================================================

const PROB_BITS: u32 = 8;
const ONE_PROB: u32 = 1 << PROB_BITS;

trait EntropyCoder {
    fn encode_bit(&mut self, bit: bool, prob: u16);

    fn encode_with_context(&mut self, bit: bool, context: &mut Context) {
        self.encode_bit(bit, context.prob());
        context.update(bit);
    }
}

struct RansCoder {
    bits: Vec<u16>,
    use_bitstream: bool,
    bitstream_is_big_endian: bool,
    invert_bit_encoding: bool,
}

impl EntropyCoder for RansCoder {
    fn encode_bit(&mut self, bit: bool, prob: u16) {
        assert!(prob < 32768);
        self.bits
            .push(prob | (((bit ^ self.invert_bit_encoding) as u16) << 15));
    }
}

impl RansCoder {
    fn new(config: &Config) -> RansCoder {
        RansCoder {
            bits: Vec::new(),
            use_bitstream: config.use_bitstream,
            bitstream_is_big_endian: config.bitstream_is_big_endian,
            invert_bit_encoding: config.invert_bit_encoding,
        }
    }

    fn finish(self) -> Vec<u8> {
        let mut buffer = vec![];
        let l_bits: u32 = if self.use_bitstream { 15 } else { 12 };
        let mut state = 1 << l_bits;

        let mut byte = 0u8;
        let mut bit = if self.bitstream_is_big_endian { 0 } else { 8 };
        let mut flush_state: Box<dyn FnMut(&mut u32)> = if self.use_bitstream {
            if self.bitstream_is_big_endian {
                Box::new(|state: &mut u32| {
                    byte |= ((*state & 1) as u8) << bit;
                    bit += 1;
                    if bit == 8 {
                        buffer.push(byte);
                        byte = 0;
                        bit = 0;
                    }
                    *state >>= 1;
                })
            } else {
                Box::new(|state: &mut u32| {
                    bit -= 1;
                    byte |= ((*state & 1) as u8) << bit;
                    if bit == 0 {
                        buffer.push(byte);
                        byte = 0;
                        bit = 8;
                    }
                    *state >>= 1;
                })
            }
        } else {
            Box::new(|state: &mut u32| {
                buffer.push(*state as u8);
                *state >>= 8;
            })
        };

        let num_flush_bits = if self.use_bitstream { 1 } else { 8 };
        let max_state_factor: u32 = 1 << (l_bits + num_flush_bits - PROB_BITS);
        for step in self.bits.into_iter().rev() {
            let prob = step as u32 & 32767;
            let (start, prob) = if step & 32768 != 0 {
                (0, prob)
            } else {
                (prob, ONE_PROB - prob)
            };
            let max_state = max_state_factor * prob;
            while state >= max_state {
                flush_state(&mut state);
            }
            state = ((state / prob) << PROB_BITS) + (state % prob) + start;
        }

        while state > 0 {
            flush_state(&mut state);
        }

        drop(flush_state);

        if self.use_bitstream && byte != 0 {
            buffer.push(byte);
        }

        buffer.reverse();
        buffer
    }
}

struct CostCounter {
    cost: f64,
    log2_table: Vec<f64>,
    invert_bit_encoding: bool,
}

impl CostCounter {
    fn new(config: &Config) -> CostCounter {
        let log2_table = (0..ONE_PROB)
            .map(|prob| {
                let inv_prob = ONE_PROB as f64 / prob as f64;
                inv_prob.log2()
            })
            .collect();
        CostCounter {
            cost: 0.0,
            log2_table,
            invert_bit_encoding: config.invert_bit_encoding,
        }
    }

    fn cost(&self) -> f64 {
        self.cost
    }

    fn reset(&mut self) {
        self.cost = 0.0;
    }
}

impl EntropyCoder for CostCounter {
    fn encode_bit(&mut self, bit: bool, prob: u16) {
        let prob = if bit ^ self.invert_bit_encoding {
            prob as u32
        } else {
            ONE_PROB - prob as u32
        };
        self.cost += self.log2_table[prob as usize];
    }
}

// =====================================================================================
// Adaptive bit-probability contexts.
// =====================================================================================

const INIT_PROB: u16 = 1 << (PROB_BITS - 1);
const UPDATE_RATE: u32 = 4;
const UPDATE_ADD: u32 = 8;

#[derive(Clone)]
struct ContextState {
    contexts: Vec<u8>,
    invert_bit_encoding: bool,
    simplified_prob_update: bool,
}

struct Context<'a> {
    state: &'a mut ContextState,
    index: usize,
}

impl ContextState {
    fn new(size: usize, config: &Config) -> ContextState {
        ContextState {
            contexts: vec![INIT_PROB as u8; size],
            invert_bit_encoding: config.invert_bit_encoding,
            simplified_prob_update: config.simplified_prob_update,
        }
    }

    fn context_mut(&mut self, index: usize) -> Context<'_> {
        Context { state: self, index }
    }
}

impl<'a> Context<'a> {
    fn prob(&self) -> u16 {
        self.state.contexts[self.index] as u16
    }

    fn update(&mut self, bit: bool) {
        let old = self.state.contexts[self.index];

        self.state.contexts[self.index] = if self.state.simplified_prob_update {
            let offset = if bit ^ self.state.invert_bit_encoding {
                ONE_PROB as i32 >> UPDATE_RATE
            } else {
                0
            };
            (offset + old as i32 - ((old as i32 + UPDATE_ADD as i32) >> UPDATE_RATE)) as u8
        } else if bit ^ self.state.invert_bit_encoding {
            old + ((ONE_PROB - old as u32 + UPDATE_ADD) >> UPDATE_RATE) as u8
        } else {
            old - ((old as u32 + UPDATE_ADD) >> UPDATE_RATE) as u8
        };
    }
}

// =====================================================================================
// LZ op and EOF grammar (literal, match, rep-offset, length codes).
// =====================================================================================

#[derive(Copy, Clone, Debug)]
enum Op {
    Literal(u8),
    Match { offset: u32, len: u32 },
}

impl Op {
    fn encode(&self, coder: &mut dyn EntropyCoder, state: &mut CoderState, config: &Config) {
        let literal_base = state.pos % state.parity_contexts * 256;
        match *self {
            Op::Literal(lit) => {
                encode_bit(coder, state, literal_base, !config.is_match_bit);
                let mut context_index = 1;
                for i in (0..8).rev() {
                    let bit = (lit >> i) & 1 != 0;
                    encode_bit(coder, state, literal_base + context_index, bit);
                    context_index = (context_index << 1) | bit as usize;
                }
                state.prev_was_match = false;
                state.pos += 1;
            }
            Op::Match { offset, len } => {
                encode_bit(coder, state, literal_base, config.is_match_bit);
                let mut new_offset = true;
                if !state.prev_was_match && !config.no_repeated_offsets {
                    new_offset = offset != state.last_offset;
                    encode_bit(
                        coder,
                        state,
                        256 * state.parity_contexts,
                        new_offset == config.new_offset_bit,
                    );
                }
                assert!(offset as usize <= config.max_offset);
                if new_offset {
                    encode_length(
                        coder,
                        state,
                        256 * state.parity_contexts + 1,
                        offset + if config.eof_in_length { 0 } else { 1 },
                        config,
                    );
                    state.last_offset = offset;
                }
                assert!(len as usize >= config.min_length() && len as usize <= config.max_length);
                encode_length(coder, state, 256 * state.parity_contexts + 65, len, config);
                state.prev_was_match = true;
                state.pos += len as usize;
            }
        }
    }
}

fn encode_eof(coder: &mut dyn EntropyCoder, state: &mut CoderState, config: &Config) {
    encode_bit(
        coder,
        state,
        state.pos % state.parity_contexts * 256,
        config.is_match_bit,
    );
    if !state.prev_was_match && !config.no_repeated_offsets {
        encode_bit(
            coder,
            state,
            256 * state.parity_contexts,
            config.new_offset_bit ^ config.eof_in_length,
        );
    }
    if !config.eof_in_length || state.prev_was_match || config.no_repeated_offsets {
        encode_length(coder, state, 256 * state.parity_contexts + 1, 1, config);
    }
    if config.eof_in_length {
        encode_length(coder, state, 256 * state.parity_contexts + 65, 1, config);
    }
}

fn encode_bit(
    coder: &mut dyn EntropyCoder,
    state: &mut CoderState,
    context_index: usize,
    bit: bool,
) {
    coder.encode_with_context(bit, &mut state.contexts.context_mut(context_index));
}

fn encode_length(
    coder: &mut dyn EntropyCoder,
    state: &mut CoderState,
    context_start: usize,
    mut value: u32,
    config: &Config,
) {
    assert!(value >= 1);

    let mut context_index = context_start;
    while value >= 2 {
        encode_bit(coder, state, context_index, config.continue_value_bit);
        encode_bit(coder, state, context_index + 1, value & 1 != 0);
        context_index += 2;
        value >>= 1;
    }
    encode_bit(coder, state, context_index, !config.continue_value_bit);
}

#[derive(Clone)]
struct CoderState {
    contexts: ContextState,
    last_offset: u32,
    prev_was_match: bool,
    pos: usize,
    parity_contexts: usize,
}

impl CoderState {
    fn new(config: &Config) -> CoderState {
        CoderState {
            contexts: ContextState::new((1 + 255) * config.parity_contexts + 1 + 64 + 64, config),
            last_offset: 0,
            prev_was_match: false,
            pos: 0,
            parity_contexts: config.parity_contexts,
        }
    }

    fn last_offset(&self) -> u32 {
        self.last_offset
    }
}

// =====================================================================================
// Suffix-array + LCP match finder. The suffix array is built with a prefix-doubling
// (O(n log n)) construction.
// =====================================================================================

/// Build the suffix array of `data`. SA[i] is the start index of the i-th smallest suffix.
fn build_suffix_array(data: &[u8]) -> Vec<i32> {
    let n = data.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![0];
    }

    // rank[i] = current rank of suffix starting at i, compressed to [0, n-1] so the radix
    // sort's bucket count (n+2) always covers the key range.
    let mut sa: Vec<i32> = (0..n as i32).collect();
    let mut rank: Vec<i32> = {
        let mut present = [false; 256];
        for &b in data {
            present[b as usize] = true;
        }
        let mut byte_rank = [0i32; 256];
        let mut r = 0i32;
        for (b, slot) in byte_rank.iter_mut().enumerate() {
            if present[b] {
                *slot = r;
                r += 1;
            }
        }
        data.iter().map(|&b| byte_rank[b as usize]).collect()
    };
    let mut tmp: Vec<i32> = vec![0; n];

    let mut k = 1usize;
    loop {
        // Comparator key for suffix i at this k: (rank[i], rank[i+k] or -1).
        let key = |i: usize| -> (i32, i32) {
            let second = if i + k < n { rank[i + k] } else { -1 };
            (rank[i], second)
        };

        // Sort suffixes by (rank[i], rank[i+k]). Use a radix sort on the two keys for speed.
        radix_sort_pairs(&mut sa, &rank, k, n);

        // Recompute ranks.
        tmp[sa[0] as usize] = 0;
        let mut r = 0i32;
        for w in 1..n {
            let a = sa[w - 1] as usize;
            let b = sa[w] as usize;
            if key(a) != key(b) {
                r += 1;
            }
            tmp[b] = r;
        }
        rank.copy_from_slice(&tmp);

        if (r as usize) == n - 1 {
            break; // all ranks distinct -> SA complete
        }
        k <<= 1;
        if k >= n {
            break;
        }
    }
    sa
}

/// Stable LSD radix sort of `sa` by the pair (rank[i], rank[i+k]), least significant key first.
fn radix_sort_pairs(sa: &mut Vec<i32>, rank: &[i32], k: usize, n: usize) {
    // Keys are in range [-1, n-1]; shift by +1 so they fall in [0, n].
    let buckets = n + 1;
    let mut count = vec![0u32; buckets + 1];
    let mut output = vec![0i32; n];

    // Pass 1: by second key (rank[i+k], with -1 meaning 0).
    for c in count.iter_mut() {
        *c = 0;
    }
    let second_key = |i: usize| -> usize {
        if i + k < n {
            (rank[i + k] + 1) as usize
        } else {
            0
        }
    };
    for &s in sa.iter() {
        count[second_key(s as usize)] += 1;
    }
    let mut sum = 0u32;
    for c in count.iter_mut() {
        let t = *c;
        *c = sum;
        sum += t;
    }
    for &s in sa.iter() {
        let key = second_key(s as usize);
        output[count[key] as usize] = s;
        count[key] += 1;
    }

    // Pass 2: by first key (rank[i]).
    for c in count.iter_mut() {
        *c = 0;
    }
    for &s in output.iter() {
        count[(rank[s as usize] + 1) as usize] += 1;
    }
    let mut sum = 0u32;
    for c in count.iter_mut() {
        let t = *c;
        *c = sum;
        sum += t;
    }
    for &s in output.iter() {
        let key = (rank[s as usize] + 1) as usize;
        sa[count[key] as usize] = s;
        count[key] += 1;
    }
}

struct MatchFinder {
    suffixes: Vec<i32>,
    rev_suffixes: Vec<u32>,
    lcp: Vec<u32>,

    max_queue_size: usize,
    max_matches_per_length: usize,
    patience: usize,
    max_length_diff: usize,

    queue: std::collections::BinaryHeap<usize>,
}

impl MatchFinder {
    fn new(data: &[u8]) -> MatchFinder {
        let suffixes = build_suffix_array(data);

        let mut rev_suffixes = vec![0u32; data.len()];
        for (suffix_index, index) in suffixes.iter().enumerate() {
            rev_suffixes[*index as usize] = suffix_index as u32;
        }

        // LCP array via a Kasai-style walk.
        let mut lcp = vec![0u32; data.len()];
        let mut length = 0usize;
        for suffix_index in &rev_suffixes {
            if *suffix_index as usize + 1 < suffixes.len() {
                let i = suffixes[*suffix_index as usize] as usize;
                let j = suffixes[*suffix_index as usize + 1] as usize;
                while i + length < data.len()
                    && j + length < data.len()
                    && data[i + length] == data[j + length]
                {
                    length += 1;
                }
                lcp[*suffix_index as usize] = length as u32;
            }
            length = length.saturating_sub(1);
        }

        MatchFinder {
            suffixes,
            rev_suffixes,
            lcp,
            max_queue_size: 100,
            max_matches_per_length: 5,
            patience: 100,
            max_length_diff: 2,
            queue: std::collections::BinaryHeap::new(),
        }
    }

    fn with_max_queue_size(mut self, v: usize) -> MatchFinder {
        self.max_queue_size = v;
        self
    }
    fn with_patience(mut self, v: usize) -> MatchFinder {
        self.patience = v;
        self
    }
    fn with_max_matches_per_length(mut self, v: usize) -> MatchFinder {
        self.max_matches_per_length = v;
        self
    }
    fn with_max_length_diff(mut self, v: usize) -> MatchFinder {
        self.max_length_diff = v;
        self
    }

    fn matches(&mut self, pos: usize) -> Matches<'_> {
        let index = self.rev_suffixes[pos] as usize;
        self.queue.clear();
        let mut matches = Matches {
            finder: self,
            pos_range: 0..pos,
            left_index: index,
            left_length: usize::MAX,
            right_index: index,
            right_length: usize::MAX,
            current_length: usize::MAX,
            matches_left: 0,
            max_length: 0,
        };

        matches.move_left();
        matches.move_right();

        matches
    }
}

struct Matches<'a> {
    finder: &'a mut MatchFinder,
    pos_range: std::ops::Range<usize>,
    left_index: usize,
    left_length: usize,
    right_index: usize,
    right_length: usize,
    current_length: usize,
    matches_left: usize,
    max_length: usize,
}

#[derive(Debug)]
struct Match {
    pos: usize,
    length: usize,
}

impl<'a> Iterator for Matches<'a> {
    type Item = Match;

    fn next(&mut self) -> Option<Match> {
        if self.finder.queue.is_empty() || self.matches_left == 0 {
            self.finder.queue.clear();
            self.current_length = self
                .current_length
                .saturating_sub(1)
                .min(self.left_length.max(self.right_length));
            self.max_length = self.max_length.max(self.current_length);
            if self.current_length < 2
                || self.current_length + self.finder.max_length_diff < self.max_length
            {
                return None;
            }
            while self.finder.queue.len() < self.finder.max_queue_size
                && (self.left_length == self.current_length
                    || self.right_length == self.current_length)
            {
                if self.left_length == self.current_length {
                    self.finder
                        .queue
                        .push(self.finder.suffixes[self.left_index] as usize);
                    self.move_left();
                }
                if self.right_length == self.current_length {
                    self.finder
                        .queue
                        .push(self.finder.suffixes[self.right_index] as usize);
                    self.move_right();
                }
            }
            self.matches_left = self.finder.max_matches_per_length;
        }

        self.matches_left = self.matches_left.saturating_sub(1);

        self.finder.queue.pop().map(|pos| Match {
            pos,
            length: self.current_length,
        })
    }
}

impl<'a> Matches<'a> {
    fn move_left(&mut self) {
        let mut patience = self.finder.patience;
        while self.left_length > 0 && patience > 0 && self.left_index > 0 {
            self.left_index -= 1;
            self.left_length = self
                .left_length
                .min(self.finder.lcp[self.left_index] as usize);
            if self
                .pos_range
                .contains(&(self.finder.suffixes[self.left_index] as usize))
            {
                return;
            }
            patience -= 1;
        }
        self.left_length = 0;
    }

    fn move_right(&mut self) {
        let mut patience = self.finder.patience;
        while self.right_length > 0
            && patience > 0
            && self.right_index + 1 < self.finder.suffixes.len()
        {
            self.right_index += 1;
            self.right_length = self
                .right_length
                .min(self.finder.lcp[self.right_index - 1] as usize);
            if self
                .pos_range
                .contains(&(self.finder.suffixes[self.right_index] as usize))
            {
                return;
            }
            patience -= 1;
        }
        self.right_length = 0;
    }
}

// =====================================================================================
// Optimal multi-arrival parser.
// =====================================================================================

struct Parse {
    prev: Option<Rc<Parse>>,
    op: Op,
}

impl Drop for Parse {
    // Free the chain iteratively. A recursive drop of a long `prev` list would overflow the
    // stack, so detach and drop each uniquely owned link in a loop.
    fn drop(&mut self) {
        let mut link = self.prev.take();
        while let Some(rc) = link {
            match Rc::try_unwrap(rc) {
                Ok(mut node) => link = node.prev.take(),
                Err(_) => break,
            }
        }
    }
}

struct Arrival {
    parse: Option<Rc<Parse>>,
    state: CoderState,
    cost: f64,
}

type Arrivals = HashMap<usize, Vec<Arrival>>;

struct ParseConfig {
    max_arrivals: usize,
    max_cost_delta: f64,
    max_offset_cost_delta: f64,
    num_near_matches: usize,
    greedy_size: usize,
    max_queue_size: usize,
    patience: usize,
    max_matches_per_length: usize,
    max_length_diff: usize,
}

impl ParseConfig {
    fn from_level(level: u8) -> ParseConfig {
        let max_arrivals = match level {
            0..=1 => 0,
            2 => 2,
            3 => 4,
            4 => 8,
            5 => 16,
            6 => 32,
            7 => 64,
            8 => 96,
            _ => 128,
        };
        let (max_cost_delta, max_offset_cost_delta) = match level {
            0..=4 => (16.0, 0.0),
            5..=8 => (16.0, 4.0),
            _ => (16.0, 8.0),
        };
        let num_near_matches = level.saturating_sub(1) as usize;
        let greedy_size = 4 + level as usize * level as usize * 3;
        let max_length_diff = match level {
            0..=1 => 0,
            2..=3 => 1,
            4..=5 => 2,
            6..=7 => 3,
            _ => 4,
        };
        ParseConfig {
            max_arrivals,
            max_cost_delta,
            max_offset_cost_delta,
            num_near_matches,
            greedy_size,
            max_queue_size: level as usize * 100,
            patience: level as usize * 100,
            max_matches_per_length: level as usize,
            max_length_diff,
        }
    }
}

fn pack(data: &[u8], level: u8, config: &Config) -> Vec<u8> {
    let mut parse = parse(data, ParseConfig::from_level(level), config);
    let mut ops = vec![];
    while let Some(link) = parse {
        ops.push(link.op);
        parse = link.prev.clone();
    }
    let mut state = CoderState::new(config);
    let mut coder = RansCoder::new(config);
    for op in ops.into_iter().rev() {
        op.encode(&mut coder, &mut state, config);
    }
    encode_eof(&mut coder, &mut state, config);
    coder.finish()
}

fn parse(data: &[u8], config: ParseConfig, encoding_config: &Config) -> Option<Rc<Parse>> {
    let mut match_finder = MatchFinder::new(data)
        .with_max_queue_size(config.max_queue_size)
        .with_patience(config.patience)
        .with_max_matches_per_length(config.max_matches_per_length)
        .with_max_length_diff(config.max_length_diff);
    let mut near_matches = [usize::MAX; 1024];
    let mut last_seen = [usize::MAX; 256];

    let max_arrivals = config.max_arrivals;

    let mut arrivals: Arrivals = HashMap::new();

    fn sort_arrivals(vec: &mut Vec<Arrival>, max_arrivals: usize) {
        if max_arrivals == 0 {
            return;
        }
        vec.sort_by(|a, b| {
            a.cost
                .partial_cmp(&b.cost)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut seen_offsets = HashSet::new();
        let mut remaining = Vec::new();
        for arr in mem::take(vec) {
            if seen_offsets.insert(arr.state.last_offset()) {
                if vec.len() < max_arrivals {
                    vec.push(arr);
                }
            } else {
                remaining.push(arr);
            }
        }
        for arr in remaining {
            if vec.len() >= max_arrivals {
                break;
            }
            vec.push(arr);
        }
    }

    fn add_arrival(arrivals: &mut Arrivals, pos: usize, arrival: Arrival, max_arrivals: usize) {
        let vec = arrivals.entry(pos).or_default();
        if max_arrivals == 0 {
            if vec.is_empty() {
                vec.push(arrival);
            } else if vec[0].cost > arrival.cost {
                vec[0] = arrival;
            }
            return;
        }
        vec.push(arrival);
        if vec.len() > max_arrivals * 2 {
            sort_arrivals(vec, max_arrivals);
        }
    }

    fn add_match(
        arrivals: &mut Arrivals,
        cost_counter: &mut CostCounter,
        pos: usize,
        offset: usize,
        mut length: usize,
        arrival: &Arrival,
        max_arrivals: usize,
        config: &Config,
    ) {
        if length < config.min_length() {
            return;
        }
        length = length.min(config.max_length);
        cost_counter.reset();
        let mut state = arrival.state.clone();
        let op = Op::Match {
            offset: offset as u32,
            len: length as u32,
        };
        op.encode(cost_counter, &mut state, config);
        add_arrival(
            arrivals,
            pos + length,
            Arrival {
                parse: Some(Rc::new(Parse {
                    prev: arrival.parse.clone(),
                    op,
                })),
                state,
                cost: arrival.cost + cost_counter.cost(),
            },
            max_arrivals,
        );
    }

    add_arrival(
        &mut arrivals,
        0,
        Arrival {
            parse: None,
            state: CoderState::new(encoding_config),
            cost: 0.0,
        },
        max_arrivals,
    );

    let cost_counter = &mut CostCounter::new(encoding_config);
    let mut best_per_offset = HashMap::new();
    for pos in 0..data.len() {
        let match_length = |offset: usize| {
            data[pos..]
                .iter()
                .zip(data[(pos - offset)..].iter())
                .take_while(|(a, b)| a == b)
                .count()
        };

        let here_arrivals = if let Some(mut arr) = arrivals.remove(&pos) {
            sort_arrivals(&mut arr, max_arrivals);
            arr
        } else {
            continue;
        };
        best_per_offset.clear();
        let mut best_cost = f64::MAX;
        for arrival in &here_arrivals {
            best_cost = best_cost.min(arrival.cost);
            let per_offset = best_per_offset
                .entry(arrival.state.last_offset())
                .or_insert(f64::MAX);
            *per_offset = per_offset.min(arrival.cost);
        }

        'arrival_loop: for arrival in here_arrivals {
            if arrival.cost
                > (best_cost + config.max_cost_delta).min(
                    *best_per_offset.get(&arrival.state.last_offset()).unwrap()
                        + config.max_offset_cost_delta,
                )
            {
                continue;
            }
            let mut found_last_offset = false;
            let mut closest_match = None;
            for m in match_finder.matches(pos) {
                closest_match = Some(closest_match.unwrap_or(0).max(m.pos));
                let offset = pos - m.pos;
                if offset <= encoding_config.max_offset {
                    found_last_offset |= offset as u32 == arrival.state.last_offset();
                    add_match(
                        &mut arrivals,
                        cost_counter,
                        pos,
                        offset,
                        m.length,
                        &arrival,
                        max_arrivals,
                        encoding_config,
                    );
                    if m.length >= config.greedy_size {
                        break 'arrival_loop;
                    }
                }
            }

            let mut near_matches_left = config.num_near_matches;
            let mut match_pos = last_seen[data[pos] as usize];
            while near_matches_left > 0
                && match_pos != usize::MAX
                && closest_match.iter().all(|p| *p < match_pos)
            {
                let offset = pos - match_pos;
                if offset > encoding_config.max_offset {
                    break;
                }
                let length = match_length(offset);
                assert!(length > 0);
                add_match(
                    &mut arrivals,
                    cost_counter,
                    pos,
                    offset,
                    length,
                    &arrival,
                    max_arrivals,
                    encoding_config,
                );
                found_last_offset |= offset as u32 == arrival.state.last_offset();
                if offset < near_matches.len() {
                    match_pos = near_matches[match_pos % near_matches.len()];
                }
                near_matches_left -= 1;
            }

            if !found_last_offset && arrival.state.last_offset() > 0 {
                let offset = arrival.state.last_offset() as usize;
                let length = match_length(offset);
                if length > 0 {
                    add_match(
                        &mut arrivals,
                        cost_counter,
                        pos,
                        offset,
                        length,
                        &arrival,
                        max_arrivals,
                        encoding_config,
                    );
                }
            }

            cost_counter.reset();
            let mut state = arrival.state;
            let op = Op::Literal(data[pos]);
            op.encode(cost_counter, &mut state, encoding_config);
            add_arrival(
                &mut arrivals,
                pos + 1,
                Arrival {
                    parse: Some(Rc::new(Parse {
                        prev: arrival.parse,
                        op,
                    })),
                    state,
                    cost: arrival.cost + cost_counter.cost(),
                },
                max_arrivals,
            );
        }
        near_matches[pos % near_matches.len()] = last_seen[data[pos] as usize];
        last_seen[data[pos] as usize] = pos;
    }
    arrivals.remove(&data.len()).unwrap()[0].parse.clone()
}

// =====================================================================================
// rANS decoder.
// =====================================================================================

const PROB_MASK: u32 = ONE_PROB - 1;

#[derive(Clone)]
struct RansDecoder<'a> {
    data: &'a [u8],
    pos: usize,
    state: u32,
    use_bitstream: bool,
    byte: u8,
    bits_left: u8,
    invert_bit_encoding: bool,
    bitstream_is_big_endian: bool,
}

impl<'a> RansDecoder<'a> {
    fn new(data: &'a [u8], config: &Config) -> Option<RansDecoder<'a>> {
        let mut decoder = RansDecoder {
            data,
            pos: 0,
            state: 0,
            use_bitstream: config.use_bitstream,
            byte: 0,
            bits_left: 0,
            invert_bit_encoding: config.invert_bit_encoding,
            bitstream_is_big_endian: config.bitstream_is_big_endian,
        };
        decoder.refill()?;
        Some(decoder)
    }

    /// Pull bits/bytes from the input until the state is back above the normalization bound.
    fn refill(&mut self) -> Option<()> {
        if self.use_bitstream {
            while self.state < 32768 {
                if self.bits_left == 0 {
                    if self.pos >= self.data.len() {
                        return None;
                    }
                    self.byte = self.data[self.pos];
                    self.pos += 1;
                    self.bits_left = 8;
                }
                if self.bitstream_is_big_endian {
                    self.state = (self.state << 1) | (self.byte >> 7) as u32;
                    self.byte <<= 1;
                } else {
                    self.state = (self.state << 1) | (self.byte & 1) as u32;
                    self.byte >>= 1;
                }
                self.bits_left -= 1;
            }
        } else {
            while self.state < 4096 {
                if self.pos >= self.data.len() {
                    return None;
                }
                self.state = (self.state << 8) | self.data[self.pos] as u32;
                self.pos += 1;
            }
        }
        Some(())
    }

    fn decode_bit(&mut self, prob: u16) -> Option<bool> {
        self.refill()?;
        let prob = prob as u32;
        let bit = (self.state & PROB_MASK) < prob;
        let (start, prob) = if bit {
            (0, prob)
        } else {
            (prob, ONE_PROB - prob)
        };
        self.state = prob * (self.state >> PROB_BITS) + (self.state & PROB_MASK) - start;
        Some(bit ^ self.invert_bit_encoding)
    }

    fn decode_with_context(&mut self, context: &mut Context) -> Option<bool> {
        let bit = self.decode_bit(context.prob())?;
        context.update(bit);
        Some(bit)
    }
}

/// Decode a length code: a unary-terminated sequence of value bits, low bit first.
fn decode_length(
    decoder: &mut RansDecoder,
    contexts: &mut ContextState,
    mut context_index: usize,
    config: &Config,
) -> Option<usize> {
    let mut length = 0usize;
    let mut bit_pos = 0u32;
    while decoder.decode_with_context(&mut contexts.context_mut(context_index))?
        == config.continue_value_bit
    {
        length |= (decoder.decode_with_context(&mut contexts.context_mut(context_index + 1))?
            as usize)
            << bit_pos;
        bit_pos += 1;
        if bit_pos >= 32 {
            return None;
        }
        context_index += 2;
    }
    Some(length | (1 << bit_pos))
}

/// Decode an upkr stream into the original bytes. `max_size` bounds the output length.
pub fn unpack(packed_data: &[u8], config: &Config, max_size: usize) -> Option<Vec<u8>> {
    unpack_with_gap(packed_data, config, max_size).map(|(v, _)| v)
}

/// Like [`unpack`], but also returns the in-place safety gap (bytes): the peak
/// of `output_produced - input_consumed` over the decode minus its final value.
/// `input_consumed` is the rANS decoder's byte position. See [`max_gap_backward`].
fn unpack_with_gap(packed_data: &[u8], config: &Config, max_size: usize) -> Option<(Vec<u8>, i32)> {
    let mut decoder = RansDecoder::new(packed_data, config)?;
    let mut contexts = ContextState::new((1 + 255) * config.parity_contexts + 1 + 64 + 64, config);
    let mut result: Vec<u8> = Vec::new();
    let mut offset = usize::MAX;
    let mut position = 0usize;
    let mut prev_was_match = false;
    // Peak of (produced - consumed) at a token boundary.
    let mut max_gap = 0i32;

    loop {
        let gap = position as i32 - decoder.pos as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let literal_base = position % config.parity_contexts * 256;
        if decoder.decode_with_context(&mut contexts.context_mut(literal_base))?
            == config.is_match_bit
        {
            if config.no_repeated_offsets
                || prev_was_match
                || decoder
                    .decode_with_context(&mut contexts.context_mut(256 * config.parity_contexts))?
                    == config.new_offset_bit
            {
                offset = decode_length(
                    &mut decoder,
                    &mut contexts,
                    256 * config.parity_contexts + 1,
                    config,
                )? - if config.eof_in_length { 0 } else { 1 };
                if offset == 0 {
                    break;
                }
            }
            let length = decode_length(
                &mut decoder,
                &mut contexts,
                256 * config.parity_contexts + 65,
                config,
            )?;
            if config.eof_in_length && length == 1 {
                break;
            }
            if offset > position {
                return None;
            }
            for _ in 0..length {
                if result.len() >= max_size {
                    break;
                }
                result.push(result[result.len() - offset]);
            }
            position += length;
            prev_was_match = true;
        } else {
            let mut context_index = 1;
            let mut byte = 0u8;
            for i in (0..8).rev() {
                let bit = decoder
                    .decode_with_context(&mut contexts.context_mut(literal_base + context_index))?;
                context_index = (context_index << 1) | bit as usize;
                byte |= (bit as u8) << i;
            }
            if result.len() < max_size {
                result.push(byte);
            }
            position += 1;
            prev_was_match = false;
        }
    }

    if position > max_size {
        return None;
    }
    let final_gap = position as i32 - packed_data.len() as i32;
    Some((result, (max_gap - final_gap).max(0)))
}

/// In-place safety margin (bytes) for a FORWARD 6502 upkr stream - the peak by
/// which the compressed stream is momentarily larger than the output produced.
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    unpack_with_gap(stream, &config_6502(), usize::MAX)
        .map(|(_, g)| g.max(0) as usize)
        .unwrap_or(0)
}

/// In-place safety margin (bytes) for a BACKWARD 6502 upkr stream. The backward
/// stream is the reverse of a forward pack (`compress_upkr_6502_backward`), and
/// the descending 6502 reader reproduces the forward byte sequence, so the gap
/// is a forward decode of the reversed stream.
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        return 0;
    }
    let rev: Vec<u8> = stream.iter().rev().copied().collect();
    unpack_with_gap(&rev, &config_6502(), usize::MAX)
        .map(|(_, g)| g.max(0) as usize)
        .unwrap_or(0)
}

// =====================================================================================
// Public API
// =====================================================================================

/// Highest normalized compression level for the uniform-tier API. Small by
/// design (1 = fastest, [`MAX_LEVEL`] = absolute best/smallest). The three tiers
/// map onto upkr's native 0..=9 levels via [`native_for_level`].
pub const MAX_LEVEL: u8 = 3;

/// Highest native upkr level (max compression). Exposed through
/// [`compress_native`] and used by [`compress_upkr`]/[`compress_upkr_reverse`].
pub const MAX_NATIVE_LEVEL: u8 = 9;

/// Native upkr level driven by [`compress_upkr`] and [`compress_upkr_reverse`]
/// (= max compression = best).
const LEVEL: u8 = MAX_NATIVE_LEVEL;

/// Map a normalized tier (1..=[`MAX_LEVEL`]) onto upkr's native 0..=9 level.
/// 1 → native 1 (fastest), 2 → native 6, 3 → native 9 (best). Higher tier ⇒
/// higher native level ⇒ size ≤ the lower tier.
fn native_for_level(level: u8) -> u8 {
    match level.clamp(1, MAX_LEVEL) {
        1 => 1,
        2 => 6,
        _ => 9,
    }
}

/// Compress `input` in the forward upkr format. Decodes with `upkr.exe -u`.
pub fn compress_upkr(input: &[u8]) -> Vec<u8> {
    let config = Config::default();
    if input.is_empty() {
        // Empty input encodes as just the EOF token.
        let mut state = CoderState::new(&config);
        let mut coder = RansCoder::new(&config);
        encode_eof(&mut coder, &mut state, &config);
        return coder.finish();
    }
    pack(input, LEVEL, &config)
}

/// Compress `input` in the reverse upkr format: reverse the input, pack, reverse the output.
/// Decodes with `upkr.exe -u -r`.
pub fn compress_upkr_reverse(input: &[u8]) -> Vec<u8> {
    let mut data = input.to_vec();
    data.reverse();
    let mut packed = compress_upkr(&data);
    packed.reverse();
    packed
}

/// Build the upkr [`Config`] read by the upkr 6502 decruncher: the variant
/// produced by
/// `upkr -9 --big-endian-bitstream --invert-new-offset-bit
/// --invert-continue-value-bit --simplified-prob-update`.
///
/// This is the default config with five field values changed:
/// `--big-endian-bitstream` turns the bitstream on (`use_bitstream=true`, the
/// 15-bit-`L` rANS path) AND makes it big-endian (`bitstream_is_big_endian=true`);
/// `--invert-new-offset-bit` flips `new_offset_bit` to `false`;
/// `--invert-continue-value-bit` flips `continue_value_bit` to `false`;
/// `--simplified-prob-update` sets `simplified_prob_update=true`.
fn config_6502() -> Config {
    Config {
        use_bitstream: true,
        bitstream_is_big_endian: true,
        new_offset_bit: false,
        continue_value_bit: false,
        simplified_prob_update: true,
        ..Config::default()
    }
}

/// Compress `input` into the upkr stream variant the pfusik 6502 decruncher
/// reads (see [`config_6502`]). Forward stream, native level 9 (`-9`). Decodes
/// with the upkr 6502 decruncher; verified bit-for-bit by
/// `unpack(.., &config_6502(), ..)` round trip.
pub fn compress_upkr_6502(input: &[u8]) -> Vec<u8> {
    let config = config_6502();
    if input.is_empty() {
        let mut state = CoderState::new(&config);
        let mut coder = RansCoder::new(&config);
        encode_eof(&mut coder, &mut state, &config);
        return coder.finish();
    }
    pack(input, LEVEL, &config)
}

/// Backward/in-place variant of the 6502 stream (repo convention: reverse the
/// input, pack with [`config_6502`], reverse the output). A descending 6510
/// byte reader then reproduces the forward byte sequence exactly, so the
/// backward decruncher's rANS/bit logic is identical to the forward one.
/// Decoded by `decrunchers/upkr-pfusik-backward.s`.
pub fn compress_upkr_6502_backward(input: &[u8]) -> Vec<u8> {
    let mut data = input.to_vec();
    data.reverse();
    let mut packed = compress_upkr_6502(&data);
    packed.reverse();
    packed
}

/// Reference decoder for the backward 6502 stream: reverse the stream, unpack
/// with the 6502 config, reverse the output.
pub fn decompress_upkr_6502_backward(input: &[u8], max_size: usize) -> Option<Vec<u8>> {
    let rev: Vec<u8> = input.iter().rev().copied().collect();
    let mut out = unpack(&rev, &config_6502(), max_size)?;
    out.reverse();
    Some(out)
}

/// Compress `input` at a normalized tier `level` (clamped to 1..=[`MAX_LEVEL`]).
/// 1 = fastest, [`MAX_LEVEL`] = absolute best (smallest); higher tier ⇒ size ≤
/// lower tier. The tier is mapped onto upkr's native 0..=9 level (1 → 1, 2 → 6,
/// 3 → 9) and driven through [`compress_native`]. When `backward` is true,
/// produce a reverse stream (reverse input, pack, reverse output).
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    compress_native(input, native_for_level(level), backward)
}

/// Compress `input` at the exact upkr native level `native_level` (0..=9, where
/// 9 = max compression = best). This is the algorithm's real knob, exposed
/// directly; [`compress`] is the normalized-tier wrapper over it. When
/// `backward` is true, produce a reverse stream (reverse input, pack, reverse
/// output).
pub fn compress_native(input: &[u8], native_level: u8, backward: bool) -> Vec<u8> {
    let native_level = native_level.min(MAX_NATIVE_LEVEL);
    let config = Config::default();
    let pack_one = |data: &[u8]| -> Vec<u8> {
        if data.is_empty() {
            let mut state = CoderState::new(&config);
            let mut coder = RansCoder::new(&config);
            encode_eof(&mut coder, &mut state, &config);
            return coder.finish();
        }
        pack(data, native_level, &config)
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

/// Decompress an upkr stream. When `backward` is true, decode a reverse stream (reverse input,
/// unpack, reverse output).
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    let config = Config::default();
    if backward {
        let mut data = input.to_vec();
        data.reverse();
        let mut out = unpack(&data, &config, usize::MAX).expect("invalid upkr stream");
        out.reverse();
        out
    } else {
        unpack(input, &config, usize::MAX).expect("invalid upkr stream")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(input: &[u8]) {
        for level in 1..=MAX_LEVEL {
            let packed = compress(input, level, false);
            assert_eq!(decompress(&packed, false), input, "forward level {level}");

            let packed_rev = compress(input, level, true);
            assert_eq!(
                decompress(&packed_rev, true),
                input,
                "reverse level {level}"
            );
        }
        // Every native level round-trips both directions (the stream format is
        // level-independent).
        for nat in 0..=MAX_NATIVE_LEVEL {
            assert_eq!(
                decompress(&compress_native(input, nat, false), false),
                input
            );
            assert_eq!(decompress(&compress_native(input, nat, true), true), input);
        }
        // Tier 3 (the absolute best) == native 9 == the level-9 helpers, byte for
        // byte, both directions.
        assert_eq!(
            compress(input, MAX_LEVEL, false),
            compress_native(input, 9, false)
        );
        assert_eq!(
            compress(input, MAX_LEVEL, true),
            compress_native(input, 9, true)
        );
        assert_eq!(compress_upkr(input), compress_native(input, 9, false));
        assert_eq!(
            compress_upkr_reverse(input),
            compress_native(input, 9, true)
        );
        assert_eq!(compress_upkr(input), compress(input, MAX_LEVEL, false));
        assert_eq!(
            compress_upkr_reverse(input),
            compress(input, MAX_LEVEL, true)
        );
        // Monotone non-increasing size across tiers (best tier is smallest).
        // Checked on non-trivial inputs; on tiny inputs all tiers collapse to the
        // same few bytes and the relation is uninteresting.
        if input.len() >= 64 {
            let s1 = compress(input, 1, false).len();
            let s2 = compress(input, 2, false).len();
            let s3 = compress(input, 3, false).len();
            assert!(s1 >= s2, "tier1 {s1} < tier2 {s2}");
            assert!(s2 >= s3, "tier2 {s2} < tier3 {s3}");
        }
    }

    #[test]
    fn roundtrip_basic() {
        roundtrip(&[]);
        roundtrip(&[0]);
        roundtrip(&[42]);
        roundtrip(&[1, 2, 3, 4, 5]);
        roundtrip(b"abcabcabcabc abracadabra ");
        roundtrip(&[7u8; 4096]);
    }

    #[test]
    fn in_place_gap_reflects_expansion() {
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..8192)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        // Incompressible data overruns the fixed 32-byte in-place margin.
        assert!(max_gap_forward(&compress_upkr_6502(&noise)) > 32);
        assert!(max_gap_backward(&compress_upkr_6502_backward(&noise)) > 32);
        // Highly compressible data stays within it.
        assert!(max_gap_backward(&compress_upkr_6502_backward(&vec![0u8; 8192])) <= 32);
    }

    #[test]
    fn roundtrip_repetitive() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut v = Vec::new();
        for _ in 0..500 {
            v.extend_from_slice(base);
        }
        roundtrip(&v);
    }

    #[test]
    fn roundtrip_6502_config() {
        // The pfusik-6502 Config variant must round-trip through lzan's own
        // unpack: encode with compress_upkr_6502, decode with config_6502().
        let cfg = config_6502();
        let check = |input: &[u8]| {
            let packed = compress_upkr_6502(input);
            let out = unpack(&packed, &cfg, usize::MAX)
                .expect("compress_upkr_6502 output must unpack under config_6502");
            assert_eq!(out, input, "6502-config round-trip mismatch");
        };
        check(&[]);
        check(&[0]);
        check(&[42]);
        check(b"abcabcabcabc abracadabra ");
        check(&[7u8; 4096]);
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut v = Vec::new();
        for _ in 0..200 {
            v.extend_from_slice(base);
        }
        check(&v);
        // Pseudorandom (literal-heavy) input.
        let mut r = Vec::with_capacity(4096);
        let mut x = 0x1234_5678u32;
        for _ in 0..4096 {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            r.push((x >> 16) as u8);
        }
        check(&r);
    }

    #[test]
    fn roundtrip_pseudorandom() {
        let mut v = Vec::with_capacity(8192);
        let mut x = 0x1234_5678u32;
        for _ in 0..8192 {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            v.push((x >> 16) as u8);
        }
        roundtrip(&v);
    }
}
