//! ZX0 v2 bitstream encoder and decoder.
//!
//! Serialises the ZX0-optimal parse from [`crate::zx0opt::optimize_zx0`] into the ZX0 v2 byte
//! stream read by the stock `dzx0` and 6502 ZX0 decoders, in both forward and backward layouts.

use crate::zx::ZxCommand;
use crate::zx0opt::{optimize_zx0, ZX0_MAX_OFFSET};

/// Implicit last offset before any match is seen.
const INITIAL_OFFSET: u32 = 1;

/// Highest level accepted by [`compress`]. ZX0 has a single optimal algorithm, so all levels
/// produce the same output.
pub const MAX_LEVEL: u8 = 1;

/// Compress `input` into a ZX0 v2 stream. `level` is ignored (one optimal algorithm). `backward`
/// selects the backward (in-place / reverse-unpack) layout; otherwise the forward layout is used.
pub fn compress(input: &[u8], level: u8, backward: bool) -> Vec<u8> {
    let _ = level;
    if backward {
        compress_zx0_compatible_backward(input)
    } else {
        compress_zx0_compatible(input)
    }
}

/// Decompress a ZX0 v2 stream. `backward` selects the backward decoder; otherwise the forward
/// decoder is used.
pub fn decompress(input: &[u8], backward: bool) -> Vec<u8> {
    if backward {
        dzx0_decode_backward(input)
    } else {
        dzx0_decode(input)
    }
}

/// Bit-level writer for the ZX0 v2 stream, including the `backtrack` step that steers a bit into
/// bit 0 of the previously written byte.
struct Zx0Writer {
    out: Vec<u8>,
    bit_index: usize, // index of the byte currently collecting bits
    bit_mask: u8,     // current bit position within that byte (0 => none open)
    backtrack: bool,
}

impl Zx0Writer {
    fn new() -> Self {
        Zx0Writer {
            out: Vec::new(),
            bit_index: 0,
            bit_mask: 0,
            // Start with backtrack set so the leading literal indicator bit is dropped: the stream
            // begins implicitly in COPY_LITERALS.
            backtrack: true,
        }
    }

    #[inline]
    fn write_byte(&mut self, value: u8) {
        self.out.push(value);
    }

    /// With `backtrack` set, OR the bit into bit 0 of the last emitted byte; otherwise pack it
    /// MSB-first into a fresh or continuing bit-accumulator byte.
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
                self.write_byte(0);
            }
            if value {
                self.out[self.bit_index] |= self.bit_mask;
            }
            self.bit_mask >>= 1;
        }
    }

    /// Write `value` as an interlaced Elias-gamma code. The continuation flag bit is `backwards`
    /// and the terminator is `!backwards` (forward: 0…0,1; backward: 1…1,0). `invert` flips each
    /// data bit.
    fn write_interlaced_elias_gamma(&mut self, value: u32, backwards: bool, invert: bool) {
        // Highest power of two <= value.
        let mut i: u32 = 1;
        while (i << 1) <= value {
            i <<= 1;
        }
        // Emit each data bit below the MSB, interlaced with a continuation flag.
        i >>= 1;
        while i != 0 {
            self.write_bit(backwards);
            let bit = (value & i) != 0;
            self.write_bit(if invert { !bit } else { bit });
            i >>= 1;
        }
        self.write_bit(!backwards);
    }
}

/// Emit a ZX0 v2 stream for the given optimal-parse `commands`. The mode flags are:
///
/// * `backwards` - forward (`false`) uses gamma continuation flag `0` / terminator `1` and the
///   offset-LSB complement `(127 - (off-1)%128) << 1`. Backward (`true`) flips the flag/terminator
///   to `1` / `0` and writes the raw low 7 bits `((off-1)%128) << 1`.
/// * `invert` - when `true`, inverts the offset-MSB and EOF gamma data bits only (length gammas
///   are never inverted).
///
/// Each command is an optional literal run followed by an optional `(offset, length)` match. For
/// backward mode `input` is the already-reversed buffer and the returned bytes are in build order
/// for the caller to reverse.
fn emit_zx0_v2(input: &[u8], commands: &[ZxCommand], backwards: bool, invert: bool) -> Vec<u8> {
    let mut w = Zx0Writer::new();
    let mut last_offset: u32 = INITIAL_OFFSET;

    for cmd in commands {
        // Literal run.
        if cmd.lit_len > 0 {
            // Copy-literals indicator.
            w.write_bit(false);
            // Length gamma (never inverted).
            w.write_interlaced_elias_gamma(cmd.lit_len, backwards, false);
            // The literal bytes.
            let start = cmd.lit_start;
            for k in 0..cmd.lit_len as usize {
                w.write_byte(input[start + k]);
            }
        }

        // Match.
        if cmd.match_off != 0 {
            let offset = cmd.match_off;
            let length = cmd.match_len;
            if offset == last_offset {
                // Copy from last offset: indicator 0, then length gamma.
                w.write_bit(false);
                w.write_interlaced_elias_gamma(length, backwards, false);
            } else {
                // Copy from new offset: indicator 1.
                w.write_bit(true);
                // Offset MSB gamma.
                w.write_interlaced_elias_gamma((offset - 1) / 128 + 1, backwards, invert);
                // Offset LSB byte; bit 0 carries the first length bit via backtrack.
                let lsb = if backwards {
                    ((offset - 1) % 128) << 1
                } else {
                    (127 - (offset - 1) % 128) << 1
                };
                w.write_byte(lsb as u8);
                // Length-1 gamma, first bit backtracked into the LSB byte.
                w.backtrack = true;
                w.write_interlaced_elias_gamma(length - 1, backwards, false);
                last_offset = offset;
            }
        }
    }

    // End marker: new-offset indicator + gamma(256).
    w.write_bit(true);
    w.write_interlaced_elias_gamma(256, backwards, invert);

    w.out
}

/// Compress `input` into a forward ZX0 v2 stream that `dzx0` (and 6502 ZX0 decoders) decode back
/// to `input`. The parse uses ZX0's default window (`ZX0_MAX_OFFSET`) so no offset exceeds the ZX0
/// window. Empty input yields an empty `Vec`.
pub fn compress_zx0_compatible(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    // Forward: backwards = false, invert = true.
    let commands = optimize_zx0(input, 0, ZX0_MAX_OFFSET);
    emit_zx0_v2(input, &commands, false, true)
}

/// Compress `input` into a backward ZX0 v2 stream (the in-place / reverse-unpack layout). Backward
/// mode reverses the input before parsing, emits with `backwards = true` / `invert = false`, and
/// reverses the output so a backward decoder reads it tail-first. Empty input yields an empty
/// `Vec`.
pub fn compress_zx0_compatible_backward(input: &[u8]) -> Vec<u8> {
    if input.is_empty() {
        return Vec::new();
    }
    // Reverse the input, parse, emit backward, then reverse the output.
    let mut rev: Vec<u8> = input.to_vec();
    rev.reverse();
    let commands = optimize_zx0(&rev, 0, ZX0_MAX_OFFSET);
    let mut out = emit_zx0_v2(&rev, &commands, true, false);
    out.reverse();
    out
}

/// ZX0 v2 forward decoder. Reads the interlaced-gamma stream byte-forward, MSB-first, with the
/// backtrack step and offset-MSB / EOF gamma inversion.
pub fn dzx0_decode(data: &[u8]) -> Vec<u8> {
    dzx0_decode_with_gap(data).0
}

/// Decode a forward ZX0 v2 stream and also return the in-place safety gap (bytes):
/// `max(output_produced - input_consumed)` over the decode, minus its final value.
/// build_sfx uses this to size the in-place decrunch margin so the write head never
/// clobbers unread compressed bytes. See [`max_gap_forward`] / [`max_gap_backward`].
///
/// Unlike the single-token-per-iteration sibling decoders (apultra, bb2), ZX0's outer
/// loop can emit several tokens per pass (a literal run, a repeat-offset match, then a
/// run of new-offset matches), and a pure-literal stream produces its whole output inside
/// one iteration. Sampling only at the loop top would miss those peaks, so the gap is
/// measured after every output-producing copy. Within a copy the read head is fixed while
/// the write head advances, so the local peak is at the copy's end - sampling there
/// captures every peak. (ZX0 stores literals verbatim, so incompressible data barely
/// expands and its forward gap stays near zero; the format still needs the true peak
/// wherever a late match spikes it.)
fn dzx0_decode_with_gap(data: &[u8]) -> (Vec<u8>, i32) {
    if data.is_empty() {
        return (Vec::new(), 0);
    }
    let mut out: Vec<u8> = Vec::new();
    let mut ip = 0usize;
    let mut bit_mask = 0u8;
    let mut bit_value = 0u8;
    let mut backtrack = false;
    let mut last_byte = 0u8;
    let classic = false;
    // Peak of (produced - consumed), sampled after every copy; see the doc comment.
    let mut max_gap = 0i32;

    let read_byte = |ip: &mut usize, last_byte: &mut u8| -> u8 {
        let b = data[*ip];
        *ip += 1;
        *last_byte = b;
        b
    };
    let read_bit = |ip: &mut usize,
                    bit_mask: &mut u8,
                    bit_value: &mut u8,
                    backtrack: &mut bool,
                    last_byte: &mut u8|
     -> u8 {
        if *backtrack {
            *backtrack = false;
            return *last_byte & 1;
        }
        *bit_mask >>= 1;
        if *bit_mask == 0 {
            *bit_mask = 128;
            let b = data[*ip];
            *ip += 1;
            *last_byte = b;
            *bit_value = b;
        }
        if *bit_value & *bit_mask != 0 {
            1
        } else {
            0
        }
    };

    let read_gamma = |ip: &mut usize,
                      bit_mask: &mut u8,
                      bit_value: &mut u8,
                      backtrack: &mut bool,
                      last_byte: &mut u8,
                      inverted: u8|
     -> i32 {
        let mut value: i32 = 1;
        while read_bit(ip, bit_mask, bit_value, backtrack, last_byte) == 0 {
            let b = read_bit(ip, bit_mask, bit_value, backtrack, last_byte);
            value = (value << 1) | ((b ^ inverted) as i32);
        }
        value
    };

    let mut last_offset: i32 = INITIAL_OFFSET as i32;
    loop {
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        // Copy literals.
        let length = read_gamma(
            &mut ip,
            &mut bit_mask,
            &mut bit_value,
            &mut backtrack,
            &mut last_byte,
            0,
        );
        for _ in 0..length {
            let b = read_byte(&mut ip, &mut last_byte);
            out.push(b);
        }
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let mut goto_new_offset = read_bit(
            &mut ip,
            &mut bit_mask,
            &mut bit_value,
            &mut backtrack,
            &mut last_byte,
        ) == 1;

        if !goto_new_offset {
            // Copy from last offset.
            let length = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
                0,
            );
            let off = last_offset as usize;
            for _ in 0..length {
                let v = out[out.len() - off];
                out.push(v);
            }
            let gap = out.len() as i32 - ip as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            let go = read_bit(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            ) == 1;
            if !go {
                continue;
            }
            goto_new_offset = true;
        }

        // Copy from new offset (loop while the indicator selects a new offset).
        while goto_new_offset {
            let hi = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
                if classic { 0 } else { 1 },
            );
            if hi == 256 {
                // The read head consumes the whole `data.len()`-byte block; use it (not
                // `ip`, which stops at the end marker) so the final gap is the true end
                // state.
                let final_gap = out.len() as i32 - data.len() as i32;
                return (out, (max_gap - final_gap).max(0)); // EOF
            }
            let lo = read_byte(&mut ip, &mut last_byte) as i32;
            last_offset = hi * 128 - (lo >> 1);
            backtrack = true;
            let length = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
                0,
            ) + 1;
            let off = last_offset as usize;
            for _ in 0..length {
                let v = out[out.len() - off];
                out.push(v);
            }
            let gap = out.len() as i32 - ip as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            goto_new_offset = read_bit(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            ) == 1;
        }
    }
}

/// ZX0 v2 backward decoder (the in-place / reverse-unpack layout). Reverses the stream, decodes
/// reading bytes forward with the gamma polarity flipped (continuation flag `1`, terminator `0`,
/// no data-bit inversion) and the new-offset LSB read raw, then reverses the output.
pub fn dzx0_decode_backward(stream: &[u8]) -> Vec<u8> {
    dzx0_decode_backward_with_gap(stream).0
}

/// Decode a backward ZX0 v2 stream and also return the in-place safety gap (bytes):
/// `max(output_produced - input_consumed)` over the decode, minus its final value. See
/// [`max_gap_backward`].
///
/// ZX0's backward layout is NOT a simple reversal of the forward layout: the encoder
/// flips the Elias-gamma flag/terminator polarity and writes the offset LSB raw (see
/// [`compress_zx0_compatible_backward`] vs [`compress_zx0_compatible`]), and this decoder
/// reads it with the matching flipped polarity. A forward decode of the reversed stream
/// would therefore mis-parse it, so the backward gap must be measured on this path. The
/// gap is measured after every output-producing copy, exactly as in the forward decoder.
fn dzx0_decode_backward_with_gap(stream: &[u8]) -> (Vec<u8>, i32) {
    if stream.is_empty() {
        return (Vec::new(), 0);
    }
    // Undo the encoder's output reverse so bytes read in build order.
    let data: Vec<u8> = stream.iter().rev().copied().collect();

    let mut out: Vec<u8> = Vec::new();
    let mut ip = 0usize;
    let mut bit_mask = 0u8;
    let mut bit_value = 0u8;
    let mut backtrack = false;
    let mut last_byte = 0u8;
    // Peak of (produced - consumed), sampled after every copy; see the forward decoder.
    let mut max_gap = 0i32;

    let read_byte = |ip: &mut usize, last_byte: &mut u8| -> u8 {
        let b = data[*ip];
        *ip += 1;
        *last_byte = b;
        b
    };
    let read_bit = |ip: &mut usize,
                    bit_mask: &mut u8,
                    bit_value: &mut u8,
                    backtrack: &mut bool,
                    last_byte: &mut u8|
     -> u8 {
        if *backtrack {
            *backtrack = false;
            return *last_byte & 1;
        }
        *bit_mask >>= 1;
        if *bit_mask == 0 {
            *bit_mask = 128;
            let b = data[*ip];
            *ip += 1;
            *last_byte = b;
            *bit_value = b;
        }
        if *bit_value & *bit_mask != 0 {
            1
        } else {
            0
        }
    };

    // Backward gamma: loop while the continuation flag is 1 (terminator 0); data bits not inverted.
    let read_gamma = |ip: &mut usize,
                      bit_mask: &mut u8,
                      bit_value: &mut u8,
                      backtrack: &mut bool,
                      last_byte: &mut u8|
     -> i32 {
        let mut value: i32 = 1;
        while read_bit(ip, bit_mask, bit_value, backtrack, last_byte) == 1 {
            let b = read_bit(ip, bit_mask, bit_value, backtrack, last_byte);
            value = (value << 1) | (b as i32);
        }
        value
    };

    let mut last_offset: i32 = INITIAL_OFFSET as i32;
    loop {
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        // Copy literals.
        let length = read_gamma(
            &mut ip,
            &mut bit_mask,
            &mut bit_value,
            &mut backtrack,
            &mut last_byte,
        );
        for _ in 0..length {
            let b = read_byte(&mut ip, &mut last_byte);
            out.push(b);
        }
        let gap = out.len() as i32 - ip as i32;
        if gap > max_gap {
            max_gap = gap;
        }
        let mut goto_new_offset = read_bit(
            &mut ip,
            &mut bit_mask,
            &mut bit_value,
            &mut backtrack,
            &mut last_byte,
        ) == 1;

        if !goto_new_offset {
            // Copy from last offset.
            let length = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            );
            let off = last_offset as usize;
            for _ in 0..length {
                let v = out[out.len() - off];
                out.push(v);
            }
            let gap = out.len() as i32 - ip as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            let go = read_bit(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            ) == 1;
            if !go {
                continue;
            }
            goto_new_offset = true;
        }

        while goto_new_offset {
            // Copy from new offset; offset MSB gamma not inverted in backward mode.
            let hi = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            );
            if hi == 256 {
                // EOF: reverse build-order output back to original orientation. The read
                // head consumes the whole block; use `data.len()` (not `ip`) for the true
                // final gap. Reversing does not change `out.len()`, so compute it first.
                let final_gap = out.len() as i32 - data.len() as i32;
                out.reverse();
                return (out, (max_gap - final_gap).max(0));
            }
            let lo = read_byte(&mut ip, &mut last_byte) as i32;
            // Backward LSB is raw. Encoder wrote lo = ((offset-1)%128)<<1 and
            // hi = (offset-1)/128 + 1, so offset = (hi-1)*128 + (lo>>1) + 1.
            last_offset = (hi - 1) * 128 + (lo >> 1) + 1;
            backtrack = true;
            let length = read_gamma(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            ) + 1;
            let off = last_offset as usize;
            for _ in 0..length {
                let v = out[out.len() - off];
                out.push(v);
            }
            let gap = out.len() as i32 - ip as i32;
            if gap > max_gap {
                max_gap = gap;
            }
            goto_new_offset = read_bit(
                &mut ip,
                &mut bit_mask,
                &mut bit_value,
                &mut backtrack,
                &mut last_byte,
            ) == 1;
        }
    }
}

/// In-place safety margin (bytes) for a FORWARD ZX0 v2 stream: the top-aligned packed
/// block must start at least this many bytes above the output end, or the decoder's write
/// head overtakes unread compressed data. See [`dzx0_decode_with_gap`].
pub fn max_gap_forward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        dzx0_decode_with_gap(stream).1.max(0) as usize
    }
}

/// In-place safety margin (bytes) for a BACKWARD ZX0 v2 stream: the packed block must sit
/// at least this many bytes below the span start. ZX0's backward layout flips the gamma
/// polarity and offset-LSB encoding relative to the forward stream, so (unlike apultra /
/// bb2 / zx02) it is not a plain reversal of a forward stream and the gap is measured with
/// the dedicated backward decoder. See [`dzx0_decode_backward_with_gap`].
pub fn max_gap_backward(stream: &[u8]) -> usize {
    if stream.is_empty() {
        0
    } else {
        dzx0_decode_backward_with_gap(stream).1.max(0) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zx0opt::optimize_zx0_with_bits;

    /// The in-place safety gap tracks running expansion. ZX0 stores literals verbatim
    /// (one flag + one length gamma per run, then raw bytes), so incompressible data
    /// expands far more slowly than apultra's 9-bit-per-literal streams: the running gap
    /// grows with the density of the small matches the optimal parse still finds in random
    /// data (~1.6 bytes per KiB here). A 16 KiB incompressible sample therefore clears the
    /// fixed 32-byte default margin (an 8 KiB one lands at ~27, just under it), while a
    /// highly compressible stream barely moves the gap.
    #[test]
    fn in_place_gap_reflects_expansion() {
        // Incompressible data: the running gap exceeds the fixed 32-byte default margin.
        let mut s: u32 = 0x1234_5678;
        let noise: Vec<u8> = (0..16384)
            .map(|_| {
                s = s.wrapping_mul(1664525).wrapping_add(1013904223);
                (s >> 24) as u8
            })
            .collect();
        assert!(
            max_gap_forward(&compress_zx0_compatible(&noise)) > 32,
            "incompressible forward gap must exceed the fixed 32-byte margin"
        );
        assert!(
            max_gap_backward(&compress_zx0_compatible_backward(&noise)) > 32,
            "incompressible backward gap must exceed the fixed 32-byte margin"
        );
        // Highly compressible data barely expands: the default margin is fine.
        let zeros = vec![0u8; 8192];
        assert!(
            max_gap_backward(&compress_zx0_compatible_backward(&zeros)) <= 32,
            "compressible data should fit within the default margin"
        );
    }

    fn roundtrips(data: &[u8]) {
        let blob = compress_zx0_compatible(data);
        let out = dzx0_decode(&blob);
        assert_eq!(out, data, "zx0compat roundtrip len {}", data.len());
    }

    #[test]
    fn tiny() {
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

    /// The emitted byte length equals ZX0's predicted `(bits + 25) / 8`.
    #[test]
    fn size_matches_predicted_bits() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        let (_cmds, bits) = optimize_zx0_with_bits(&data, 0, ZX0_MAX_OFFSET);
        let predicted = (bits + 25) / 8;
        let blob = compress_zx0_compatible(&data);
        assert_eq!(blob.len() as i64, predicted, "output size vs (bits+25)/8");
    }

    fn roundtrips_backward(data: &[u8]) {
        let blob = compress_zx0_compatible_backward(data);
        let out = dzx0_decode_backward(&blob);
        assert_eq!(out, data, "zx0compat backward roundtrip len {}", data.len());
    }

    #[test]
    fn backward_tiny() {
        roundtrips_backward(&[42]);
        roundtrips_backward(&[1, 2, 3, 4, 5]);
        roundtrips_backward(b"abcabcabcabcabc");
        roundtrips_backward(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    }

    #[test]
    fn backward_repetitive() {
        let data: Vec<u8> = (0..5000).map(|i| (i % 7) as u8).collect();
        roundtrips_backward(&data);
    }

    #[test]
    fn backward_text_like() {
        let base = b"the quick brown fox jumps over the lazy dog. ";
        let mut data = Vec::new();
        for _ in 0..200 {
            data.extend_from_slice(base);
        }
        roundtrips_backward(&data);
    }

    #[test]
    fn backward_pseudo_random() {
        let mut state = 12345u32;
        let data: Vec<u8> = (0..6000)
            .map(|_| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                ((state >> 24) % 20) as u8
            })
            .collect();
        roundtrips_backward(&data);
    }
}
