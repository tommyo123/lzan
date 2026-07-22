//! Code generation: turn a configured [`Decruncher`] into asm6502 source,
//! assembled bytes or a `.prg`.
//!
//! ## How relocation works
//!
//! asm6502 has a single origin and no `.pseudopc`, so any block that must RUN
//! at a different address than it is STORED at (a decruncher staged at `$0100`,
//! a mover parked at `$0334`) is assembled *separately* at its run address and
//! embedded in the main program as `.byte` rows; the init code copies it into
//! place before jumping to it.
//!
//! ## Two-pass fix-point
//!
//! Constants inside relocated blobs (the payload's load address, the address
//! of the code to return to) depend on the final program layout. We therefore
//! render + assemble in a loop: pass N renders with the addresses observed in
//! pass N-1 (placeholders on the first pass), until the output is stable.
//! All such constants are 16-bit (placeholders >= $0100), so instruction
//! widths - and thus the layout - cannot oscillate.

use std::collections::HashMap;

use crate::builder::{Decruncher, Done, GenError, Issue, MoveSrc, Packed, Severity};
use crate::zp_safety;
use crate::registry::{
    Direction, EofKind, Format, PayloadAbi, RoutineSpec, CONFIG_BLOCK_BEGIN, CONFIG_BLOCK_END,
};

/// A fully generated program.
#[derive(Clone, Debug)]
pub struct Built {
    /// Final asm6502 source (self-contained; assembles with plain asm6502).
    pub source: String,
    /// Assembly origin.
    pub origin: u16,
    /// Assembled bytes (origin..).
    pub bytes: Vec<u8>,
    /// Resolved symbols (labels/constants) from the final pass.
    pub symbols: HashMap<String, u16>,
    /// Non-fatal validation findings (also embedded as `; WARNING:` lines).
    pub warnings: Vec<Issue>,
    /// Relocated fragments: `(name, run_address, fragment source)`. These are
    /// assembled at their run address and embedded in `source` as `.byte`
    /// blobs; the sources are kept here for inspection.
    pub fragments: Vec<(String, u16, String)>,
}

fn hex4(v: u16) -> String {
    format!("${:04X}", v)
}
fn hex2(v: u8) -> String {
    format!("${:02X}", v)
}

/// The shared ZP-seed for a caller-seeded decoder (`;@seed: caller`): seed the
/// source pointer (`= comp_data`) at `zp_base+0..1` and the destination pointer
/// (`= out_addr`) at `zp_base+2..3`, then the decoder's own body carries no seed
/// preamble. Returns an empty string for self-seeding routines. This is the one
/// place the seed lives - it is shared across every caller-seeded decoder, so a
/// decoder body matches its upstream size. Seeding is one-time (16 bytes, off the
/// hot path), so decode speed is unchanged. `comp_data` / `out_addr` are the
/// constants emitted in `decoder_consts`, so this must follow that block.
fn caller_seed_asm(spec: &RoutineSpec, zp_base: u8) -> String {
    if !spec.caller_seeded {
        return String::new();
    }
    format!(
        "        LDA #<comp_data\n        STA {s0}\n        LDA #>comp_data\n        STA {s1}\n\
         \x20       LDA #<out_addr\n        STA {s2}\n        LDA #>out_addr\n        STA {s3}\n",
        s0 = hex2(zp_base),
        s1 = hex2(zp_base.wrapping_add(1)),
        s2 = hex2(zp_base.wrapping_add(2)),
        s3 = hex2(zp_base.wrapping_add(3)),
    )
}

// ---------------------------------------------------------------------------
// Payload compression (pack() → matching lzan encoder)
// ---------------------------------------------------------------------------

/// Compress `input` into the raw stream the (format, direction) decoder
/// expects. Returns `(stream, mode_byte)`; `mode_byte` is only Some for
/// LZAN-full. `out_end` is required by Subsizer-backward (dest end marker).
pub fn compress_for(
    format: Format,
    direction: Direction,
    input: &[u8],
    out_end: Option<u16>,
) -> Result<(Vec<u8>, Option<u8>), GenError> {
    use Direction::*;
    let stream = match (format, direction) {
        (Format::Zx02, Forward) => lzan::zx02::compress_zx02(input),
        (Format::Zx02, Backward) => lzan::zx02::compress_zx02_backward(input),
        (Format::Zx0, Forward) => lzan::zx0compat::compress_zx0_compatible(input),
        (Format::Zx0, Backward) => lzan::zx0compat::compress_zx0_compatible_backward(input),
        (Format::Lzsa1, Forward) => lzan::lzsa1::compress_lzsa1(input),
        (Format::Lzsa1, Backward) => lzan::lzsa1::compress_lzsa1_backward(input),
        (Format::Lzsa2, Forward) => lzan::lzsa2::compress_lzsa2_anchor(input),
        (Format::Lzsa2, Backward) => lzan::lzsa2::compress_lzsa2_anchor_backward(input),
        (Format::Aplib, Forward) => lzan::apultra::compress_apultra(input),
        (Format::Aplib, Backward) => lzan::apultra::compress_apultra_backward(input),
        (Format::TsCrunch, Forward) => lzan::tscrunch::compress_tscrunch(input),
        (Format::TsCrunch, Backward) => {
            // TSCrunch's "backward" is its native in-place mode: the wrapper
            // header embeds the real decrunch address, and the packed blob is
            // expected END-ALIGNED with the output end (tscrunch -p -i).
            let end = out_end.ok_or_else(|| {
                GenError::Config("tscrunch backward needs output()+output_len() before pack()".into())
            })?;
            let start = end.wrapping_sub(input.len() as u16);
            lzan::tscrunch::compress_tscrunch_best_backward_with_addr(
                input,
                [(start & 0xFF) as u8, (start >> 8) as u8],
            )
        }
        (Format::ByteBoozer2, Forward) => lzan::bb2::compress_bb2(input),
        (Format::ByteBoozer2, Backward) => lzan::bb2::compress_bb2_backward(input),
        (Format::Exomizer, Forward) => lzan::exo3::compress_exo3(input),
        (Format::Exomizer, Backward) => lzan::exo3::compress_exo3_backward(input),
        (Format::Shrinkler, Forward) => lzan::shrinkler::compress_shrinkler(input),
        (Format::Shrinkler, Backward) => lzan::shrinkler::compress_shrinkler_backward(input),
        (Format::Subsizer, Forward) => lzan::subsizer::compress_subsizer(input),
        (Format::Subsizer, Backward) => {
            let end = out_end.ok_or_else(|| {
                GenError::Config("subsizer backward needs output()+output_len() before pack()".into())
            })?;
            lzan::subsizer::compress_subsizer_marker_at(input, end)
        }
        (Format::Upkr, Forward) => lzan::upkr::compress_upkr_6502(input),
        (Format::Upkr, Backward) => lzan::upkr::compress_upkr_6502_backward(input),
        (Format::PuCrunch, Forward) => lzan::pucrunch::compress_pucrunch_6502(input),
        (Format::PuCrunch, Backward) => {
            lzan::pucrunch::compress_pucrunch_6502_backward(input)
        }
        (Format::LzanMin, Forward) => lzan::zx::compress_min_eof(input)[1..].to_vec(),
        (Format::LzanMin, Backward) => {
            lzan::zx::compress_min_eof_backward(input)[1..].to_vec()
        }
        (Format::LzanFull, Forward) => {
            let blob = lzan::zx::compress(input, 4);
            return Ok((blob[1..].to_vec(), Some(blob[0])));
        }
        (Format::LzanFull, Backward) => {
            let blob = lzan::zx::compress_backward(input, 4, true, true, 4);
            return Ok((blob[1..].to_vec(), Some(blob[0])));
        }
        (Format::Bolt, Forward) => lzan::bolt::compress_bolt(input),
        (Format::Bolt, Backward) => lzan::bolt::compress_bolt_backward(input),
    };
    Ok((stream, None))
}

// ---------------------------------------------------------------------------
// Body configuration (config-defaults replacement)
// ---------------------------------------------------------------------------

/// Replace the config-defaults block content of a routine source with the
/// resolved values (zp_base / scratch symbol). Values equal to the defaults
/// are still re-emitted - the block is authoritative after generation.
fn configured_body(d: &Decruncher, zp_base: Option<u8>, scratch: Option<u16>) -> String {
    // A tailored body_override (per-crunch decoder) replaces the static source;
    // it keeps the config-defaults markers, so the splice below is identical.
    let src: &str = d.body_override.as_deref().unwrap_or(d.spec().source);
    let begin = src.find(CONFIG_BLOCK_BEGIN).expect("registry validated markers");
    let end = src.find(CONFIG_BLOCK_END).expect("registry validated markers");
    let after_begin = begin + CONFIG_BLOCK_BEGIN.len();

    let mut block = String::new();
    if let Some(base) = zp_base {
        block.push_str(&format!("zp_base = {}\n", hex2(base)));
    }
    if let (Some(addr), Some(sc)) = (scratch, d.spec().scratch.as_ref()) {
        block.push_str(&format!("{} = {}\n", sc.symbol, hex4(addr)));
    }

    let mut out = String::new();
    out.push_str(&src[..after_begin]);
    out.push('\n');
    out.push_str(&block);
    out.push_str(&src[end..]);
    out
}

/// Emit `.byte` rows (16 per line) for a blob.
fn byte_rows(bytes: &[u8], out: &mut String) {
    for chunk in bytes.chunks(16) {
        out.push_str(".byte ");
        let row: Vec<String> = chunk.iter().map(|b| format!("${:02X}", b)).collect();
        out.push_str(&row.join(","));
        out.push('\n');
    }
}

/// Generate a forward (ascending) copy loop: `len` bytes from `src` to `dst`.
/// `src`/`dst` may be labels or numeric strings. Self-contained, no ZP.
/// Byte-exact (no overshoot), strictly ascending - safe for overlapping
/// moves with dst < src.
///
/// Two shapes, both smaller than a page-counting loop with an exact-length
/// test per iteration (the technique every native cruncher boot uses in some
/// form: make the loop bounds line up with the data instead of testing):
/// * len <= 256: single page, `CPY #<len` terminates (len 256 => #$00, the
///   INY wrap matches). 13 bytes.
/// * len > 256: BIASED window. Y starts at `256 - rem` and both base
///   addresses are biased DOWN by the same amount, so the first pass copies
///   exactly the remainder bytes and every later pass is a full page - the
///   per-iteration length test disappears. 22 bytes.
fn copy_forward(name: &str, src: &str, dst: &str, len: u16, out: &mut String) {
    if len == 0 {
        return;
    }
    if len <= 256 {
        out.push_str(&format!(
            "; copy {len} bytes {src} -> {dst} (ascending, single page)\n\
{name}:\n\
        LDY #$00\n\
{name}_ld:\n\
        LDA >{src},Y\n\
{name}_st:\n\
        STA >{dst},Y\n\
        INY\n\
        CPY #{lo}\n\
        BNE {name}_ld\n",
            lo = hex2((len & 0xFF) as u8),
        ));
        return;
    }
    let rem = len & 0xFF;
    let bias = if rem == 0 { 0 } else { 256 - rem };
    // ceil(len/256); 256 pages emits LDX #$00 (wraps to 256 via DEX).
    let pages = (len as u32).div_ceil(256) as u16;
    out.push_str(&format!(
        "; copy {len} bytes {src} -> {dst} (ascending, biased window: first pass = {rem2} bytes)\n\
{name}:\n\
        LDX #{pg}\n\
        LDY #{bi}\n\
{name}_ld:\n\
        LDA >{src}-{bias},Y\n\
{name}_st:\n\
        STA >{dst}-{bias},Y\n\
        INY\n\
        BNE {name}_ld\n\
        INC {name}_ld+2\n\
        INC {name}_st+2\n\
        DEX\n\
        BNE {name}_ld\n",
        rem2 = if rem == 0 { 256 } else { rem },
        pg = hex2((pages & 0xFF) as u8),
        bi = hex2(bias as u8),
    ));
}

/// Generate a backward (descending) copy loop: `len` bytes ending at
/// `src+len-1` copied to `dst+len-1` downwards. Safe when dst > src overlaps.
/// Byte-exact, strictly descending per byte.
///
/// Page-windowed: the base addresses point at the TOP chunk (the remainder,
/// or a full page when len is a multiple of 256); Y walks it down to 0, then
/// the window slides one page lower. Replaces the old per-byte 16-bit
/// operand decrement (45 bytes) with 23 bytes.
fn copy_backward(name: &str, src: &str, dst: &str, len: u16, out: &mut String) {
    if len == 0 {
        return;
    }
    let rem = len & 0xFF;
    let top = if rem == 0 { 256 } else { rem }; // size of the first (top) chunk
    let off = len - top; // window base offset: src+off .. src+off+top-1
    let pages = (len as u32).div_ceil(256) as u16;
    out.push_str(&format!(
        "; copy {len} bytes {src} -> {dst} (descending; overlap-safe for dst > src)\n\
{name}:\n\
        LDX #{pg}\n\
        LDY #{lo}\n\
{name}_loop:\n\
        DEY\n\
{name}_ld:\n\
        LDA >{src}+{off},Y\n\
{name}_st:\n\
        STA >{dst}+{off},Y\n\
        TYA\n\
        BNE {name}_loop\n\
        DEC {name}_ld+2\n\
        DEC {name}_st+2\n\
        DEX\n\
        BNE {name}_loop\n",
        pg = hex2((pages & 0xFF) as u8),
        lo = hex2((len & 0xFF) as u8),
    ));
}

// ---------------------------------------------------------------------------
// Planning
// ---------------------------------------------------------------------------

struct ResolvedMove {
    /// Numeric source, or None = payload label (resolved per pass).
    src_addr: Option<u16>,
    dst: u16,
    len: u16,
}

struct Plan {
    org: u16,
    zp_base: Option<u8>,
    scratch_addr: Option<u16>,
    out_addr: Option<u16>,
    out_len: Option<u16>,
    mode_byte: Option<u8>,
    /// Payload bytes to embed (after ABI prefixing), or incbin path, or none.
    payload_bytes: Option<Vec<u8>>,
    payload_incbin: Option<(String, u16)>,
    payload_len: Option<u16>,
    /// Where the DECODER reads the packed stream: numeric, or the embedded
    /// payload's load address (label).
    comp_data_addr: Option<u16>, // None = use payload label
    moves: Vec<ResolvedMove>,
    /// Payload-at-final-address layout: the stream's decode-time start. The
    /// bulk is emitted at its final addresses; only the head window is
    /// parked at the file tail and moved by a loop folded into the blob.
    in_place_start: Option<u16>,
    staged: bool,
}

impl Decruncher {
    fn plan(&self) -> Result<Plan, GenError> {
        let spec = self.spec();

        let org = self.code_addr.unwrap_or(if self.basic_stub { 0x0801 } else { 0x1000 });
        if self.basic_stub && org != 0x0801 {
            return Err(GenError::Config(format!(
                "basic_stub requires org $0801 (got {})",
                hex4(org)
            )));
        }

        // Zero page / scratch resolution.
        let zp_base = if spec.zp_len > 0 {
            let base = self.zp_base.or(spec.zp_base_default).unwrap();
            if base as u16 + spec.zp_len as u16 > 0x100 {
                return Err(GenError::Config(format!(
                    "zp_base {} + span {} overflows page 0",
                    hex2(base),
                    spec.zp_len
                )));
            }
            Some(base)
        } else {
            None
        };
        let scratch_addr = match (&spec.scratch, self.scratch_addr) {
            (Some(sc), sel) => {
                let addr = sel.unwrap_or(sc.default);
                if sc.page_aligned && addr & 0xFF != 0 {
                    return Err(GenError::Config(format!(
                        "{} must be page-aligned (got {})",
                        sc.symbol,
                        hex4(addr)
                    )));
                }
                Some(addr)
            }
            (None, Some(_)) => {
                return Err(GenError::Config(format!(
                    "{} has no scratch region to place",
                    spec
                )))
            }
            (None, None) => None,
        };

        // Payload resolution (compress FromInput now).
        let packed = self.packed.as_ref().ok_or_else(|| {
            GenError::Config("no packed data: use pack()/packed_inline()/packed_at()/packed_incbin()".into())
        })?;

        let out_end = match (self.out_addr, self.out_len) {
            (Some(a), Some(l)) => Some(a.wrapping_add(l)),
            _ => None,
        };

        let (mut payload_bytes, payload_incbin, external_addr, mode_from_pack) = match packed {
            Packed::External { addr } => (None, None, Some(*addr), None),
            Packed::Inline(stream) => (Some(stream.clone()), None, None, None),
            Packed::Incbin { path, len } => {
                // Best-effort: if the file is reachable now, a size mismatch is
                // a definite error (comp_data_len / move lengths would be wrong).
                // If unreachable, trust `len` - asm6502 resolves .incbin from
                // its own working directory, which may differ from ours.
                if let Ok(meta) = std::fs::metadata(path) {
                    let actual = meta.len();
                    if actual != *len as u64 {
                        return Err(GenError::Config(format!(
                            "packed_incbin(\"{path}\", {len}): file is {actual} bytes - pass the real length"
                        )));
                    }
                }
                (None, Some((path.clone(), *len)), None, None)
            }
            Packed::FromInput(input) => {
                let (stream, mode) =
                    compress_for(spec.format, spec.direction, input, out_end)?;
                (Some(stream), None, None, mode)
            }
        };

        // ByteBoozer2 forward reads its destination from the stream head.
        if spec.payload == PayloadAbi::DstPrefixed {
            if let Some(bytes) = payload_bytes.as_mut() {
                let out = self.out_addr.ok_or_else(|| {
                    GenError::Config(format!("{}: dst-prefixed payload needs output()", spec))
                })?;
                let mut prefixed = Vec::with_capacity(bytes.len() + 2);
                prefixed.push((out & 0xFF) as u8);
                prefixed.push((out >> 8) as u8);
                prefixed.append(bytes);
                *bytes = prefixed;
            }
            // External/incbin blobs must already carry the prefix - documented.
        }

        let payload_len = payload_bytes
            .as_ref()
            .map(|b| b.len() as u16)
            .or(payload_incbin.as_ref().map(|(_, l)| *l))
            .or(self.packed_len_override);

        let mode_byte = self.mode_byte.or(mode_from_pack);

        // Resolve moves.
        let mut moves = Vec::new();
        let mut payload_final: Option<u16> = external_addr; // where decoder reads from
        let mut payload_moved = false;
        for mv in &self.moves {
            let len = match (mv.len, mv.src) {
                (Some(l), _) => l,
                (None, MoveSrc::Payload) => payload_len.ok_or_else(|| {
                    GenError::Config("move of packed data needs a known packed length".into())
                })?,
                (None, MoveSrc::Addr(_)) => {
                    return Err(GenError::Config("move_data needs a length".into()))
                }
            };
            let dst = if mv.top_align {
                mv.dst.wrapping_sub(len).wrapping_add(1)
            } else {
                mv.dst
            };
            let src_addr = match mv.src {
                MoveSrc::Addr(a) => Some(a),
                MoveSrc::Payload => {
                    payload_moved = true;
                    payload_final = Some(dst);
                    external_addr // None when payload embedded → label
                }
            };
            moves.push(ResolvedMove { src_addr, dst, len });
        }

        // Where does the decoder read the stream?
        let comp_data_addr = if let Some(fs) = self.in_place_start {
            // Payload-at-final-address: the decoder reads at the declared
            // final position; the bulk already loads there, the head is
            // moved by the folded loop.
            if payload_moved {
                return Err(GenError::Config(
                    "payload_in_place() and move_packed_to()/move_packed_to_top() are mutually \
                     exclusive"
                        .into(),
                ));
            }
            if self.stage_at.is_none() {
                return Err(GenError::Config(
                    "payload_in_place() requires stage_decruncher_at() (the head move \
                     overwrites the boot code)"
                        .into(),
                ));
            }
            if payload_bytes.is_none() {
                return Err(GenError::Config(
                    "payload_in_place() needs an inline payload (pack()/packed_inline())".into(),
                ));
            }
            Some(fs)
        } else if payload_moved {
            payload_final // numeric: the (last) move destination
        } else {
            external_addr // numeric for External, None (= label) for embedded
        };

        Ok(Plan {
            org,
            zp_base,
            scratch_addr,
            out_addr: self.out_addr,
            out_len: self.out_len,
            in_place_start: self.in_place_start,
            mode_byte,
            payload_bytes,
            payload_incbin,
            payload_len,
            comp_data_addr,
            moves,
            staged: self.stage_at.is_some(),
        })
    }

    // -----------------------------------------------------------------------
    // Validation
    // -----------------------------------------------------------------------

    fn validate_plan(&self, plan: &Plan, program_end: u32, blob_regions: &[(u16, u16, &str)]) -> Vec<Issue> {
        let spec = self.spec();
        let mut issues = Vec::new();
        let err = |issues: &mut Vec<Issue>, msg: String| {
            issues.push(Issue { severity: Severity::Error, msg })
        };
        let warn = |issues: &mut Vec<Issue>, msg: String| {
            issues.push(Issue { severity: Severity::Warning, msg })
        };

        let overlaps = |a0: u32, a1: u32, b0: u32, b1: u32| a0 < b1 && b0 < a1;
        let prog = (plan.org as u32, program_end);

        // Required constants.
        if spec.needs.out_addr && plan.out_addr.is_none() {
            err(&mut issues, format!("{spec}: output() is required"));
        }
        if spec.needs.out_len && plan.out_len.is_none() {
            err(&mut issues, format!("{spec}: output_len() is required (no in-stream EOF / backward seed)"));
        }
        if spec.needs.comp_data_len && plan.payload_len.is_none() {
            err(&mut issues, format!("{spec}: packed length unknown - use packed_len() or an inline payload"));
        }
        if spec.needs.zx_mode && plan.mode_byte.is_none() {
            err(&mut issues, format!("{spec}: mode_byte() is required (or use pack())"));
        }
        if spec.eof == EofKind::Length && plan.out_len.is_none() {
            err(&mut issues, format!("{spec}: format has no in-stream EOF - output_len() (stop condition) required"));
        }

        // Zero page sanity.
        if let Some(base) = plan.zp_base {
            if base <= 0x01 {
                err(&mut issues, format!(
                    "zp_base {} overlaps the 6510 CPU port at $00/$01",
                    hex2(base)
                ));
            }
            // `Rts`/`RunBasic` return to BASIC, so the span must not damage it.
            // `Jmp` hands the machine to the payload, so it is not checked.
            let preserve = matches!(self.done, Done::Rts | Done::RunBasic);
            let span_end = (base as u16 + spec.zp_len as u16 - 1) as u8;
            for r in zp_safety::regions_hit(base, spec.zp_len) {
                // The `RunBasic` tail writes VARTAB and runs CLR, so it puts
                // $2D-$36 back itself.
                let class = if self.done == Done::RunBasic && r.lo >= 0x2D && r.hi <= 0x36 {
                    zp_safety::ZpClass::Deferred
                } else {
                    r.class
                };
                let alt = zp_safety::first_safe_base(spec.zp_len, 0x02)
                    .map(|b| format!(
                        "; {} bytes fit at {} - set zero_page({})",
                        spec.zp_len, hex2(b), hex2(b)
                    ))
                    .unwrap_or_else(|| format!(
                        "; no {}-byte window in page zero is free when BASIC and the KERNAL \
                         must survive (the widest is $07-$2A, 36 bytes)",
                        spec.zp_len
                    ));
                let where_ = format!(
                    "zp span {}..{} covers {}..{}, {}",
                    hex2(base), hex2(span_end), hex2(r.lo), hex2(r.hi), r.name
                );
                match (class, preserve) {
                    (zp_safety::ZpClass::Persistent, true) => err(&mut issues, format!(
                        "{where_} - nothing rebuilds this without a reset{alt}"
                    )),
                    (zp_safety::ZpClass::Persistent, false) => warn(&mut issues, format!(
                        "{where_} - nothing rebuilds this without a reset, so BASIC and the \
                         KERNAL are unusable after the decrunch{alt}"
                    )),
                    (zp_safety::ZpClass::Deferred, true) => warn(&mut issues, format!(
                        "{where_} - re-derived in normal operation, but the damage is visible \
                         until then{alt}"
                    )),
                    _ => {}
                }
            }
        }

        // Re-entering BASIC needs the BASIC ROM banked in and the decrunched
        // program where TXTTAB points.
        if self.done == Done::RunBasic {
            if let Some((val, restore)) = self.all_ram {
                let effective = restore.unwrap_or(val);
                // CLR goes through the KERNAL (CLALL), which needs IO as well
                // as BASIC and KERNAL ROM: LORAM, HIRAM and CHAREN all set.
                if effective & 0x07 != 0x07 {
                    err(&mut issues, format!(
                        "run_basic_when_done() needs BASIC, KERNAL and IO banked in, but $01 is \
                         left at {} - use all_ram_with({}, Some($37))",
                        hex2(effective), hex2(val)
                    ));
                }
            }
            if let Some(out) = plan.out_addr {
                if out != 0x0801 {
                    warn(&mut issues, format!(
                        "run_basic_when_done() relinks from TXTTAB ($2B/$2C); output {} is not \
                         the default BASIC start $0801",
                        hex4(out)
                    ));
                }
            }
        }

        // Returning to BASIC or the KERNAL with the ROMs still banked out.
        if let Some((val, None)) = self.all_ram {
            if matches!(self.done, Done::Rts | Done::RunBasic) {
                warn(&mut issues, format!(
                    "all_ram() leaves $01 at {} and interrupts masked, but control returns to \
                     BASIC/KERNAL - use all_ram_with({}, Some($37))",
                    hex2(val), hex2(val)
                ));
            }
        }

        // Output region checks. Some of these need only the START address, so
        // they run whenever `output()` was set - even for a forward stream-EOF
        // format whose decompressed length is unknown (`packed_inline` /
        // `packed_at` without `output_len`). Only the full-extent checks are
        // gated on a known `out_len`; when the length is unknown we warn that
        // the far end cannot be verified.
        if let Some(out) = plan.out_addr {
            let start = out as u32;
            // Start-only: the output start sits in ZP / the CPU stack.
            if start < 0x0200 {
                err(&mut issues, format!(
                    "output {} overlaps zero page / CPU stack (below $0200)",
                    hex4(out)
                ));
            }
            // Start-only: the output start lands inside the program image.
            if start >= prog.0 && start < prog.1 && !plan.staged {
                err(&mut issues, format!(
                    "output {} starts inside the program image {}..{} - use stage_decruncher_at() (e.g. $0100)",
                    hex4(out), hex4(plan.org), hex4((program_end - 1) as u16)
                ));
            }
            // Start-only: the output start lands inside a relocated blob.
            for &(b0, b1, name) in blob_regions {
                if start >= b0 as u32 && start < b1 as u32 {
                    err(&mut issues, format!(
                        "output {} starts inside the relocated {name} at {}..{}",
                        hex4(out), hex4(b0), hex4(b1 - 1)
                    ));
                }
            }

            match plan.out_len {
                Some(len) => {
                    let o = (start, start + len as u32);
                    if o.1 > 0x1_0000 {
                        err(&mut issues, format!("output {}+{} runs past $FFFF", hex4(out), len));
                    }
                    if o.1 == 0x1_0000 {
                        warn(&mut issues, "output ends exactly at $FFFF - several formats (Exomizer) cannot reference a match ending at $FFFF; consider stopping at $FFFE and copying the last byte(s) manually".into());
                    }
                    if overlaps(o.0, o.1, prog.0, prog.1) && !plan.staged {
                        err(&mut issues, format!(
                            "output {}..{} overwrites the program image {}..{} - use stage_decruncher_at() (e.g. $0100)",
                            hex4(out), hex4((o.1 - 1) as u16), hex4(plan.org), hex4((program_end - 1) as u16)
                        ));
                    }
                    for &(b0, b1, name) in blob_regions {
                        if overlaps(o.0, o.1, b0 as u32, b1 as u32) {
                            err(&mut issues, format!(
                                "output {}..{} overwrites the relocated {name} at {}..{}",
                                hex4(out), hex4((o.1 - 1) as u16), hex4(b0), hex4(b1 - 1)
                            ));
                        }
                    }
                    if o.1 > 0xD000 && self.all_ram.is_none() {
                        warn(&mut issues, "output reaches above $D000 (IO/ROM) - add all_ram() so writes land in RAM".into());
                    }
                }
                None => {
                    warn(&mut issues, format!(
                        "output_len() unknown for {} - cannot verify the decompressed region does not overwrite the program / stack / relocated blobs at its far end",
                        hex4(out)
                    ));
                }
            }
        }

        // Move destination checks.
        for (i, mv) in plan.moves.iter().enumerate() {
            let m = (mv.dst as u32, mv.dst as u32 + mv.len as u32);
            if m.1 > 0x1_0000 {
                err(&mut issues, format!("move #{i}: {}+{} runs past $FFFF", hex4(mv.dst), mv.len));
            }
            // A move onto the program image needs copy code that survives the
            // overwrite - a relocated mover or the moves folded into the
            // staged blob - AND a staged decruncher (so control does not
            // `JMP after_moves` into the just-overwritten inline decoder).
            let survives = self.mover_at.is_some() || self.fold_mover;
            if overlaps(m.0, m.1, prog.0, prog.1) && (!survives || !plan.staged) {
                err(&mut issues, format!(
                    "move #{i} destination {}..{} overwrites the program image - use mover_at() \
                     (e.g. $0334) or fold_mover_into_stage(), AND stage_decruncher_at() (e.g. $0100)",
                    hex4(mv.dst), hex4((m.1 - 1) as u16)
                ));
            }
            if m.1 > 0xD000 && self.all_ram.is_none() {
                warn(&mut issues, format!(
                    "move #{i} writes above $D000 (IO/ROM) - add all_ram() so writes land in RAM"
                ));
            }
        }

        // Staged blob checks. Test whether the blob OVERLAPS the stack working
        // region (page 1), not whether it is contained in it - a blob that
        // starts at $0100 and spills past $0200 still clobbers the whole stack
        // page and must be rejected, not silently skipped.
        for &(b0, b1, name) in blob_regions {
            if (b0 as u32) < 0x0200 && (b1 as u32) > 0x0100 {
                let headroom = 0x01F4u32.saturating_sub(b1 as u32);
                if b1 as u32 > 0x01F4 {
                    err(&mut issues, format!(
                        "{name} at {}..{} leaves no CPU stack headroom (keep below $01F4)",
                        hex4(b0), hex4((b1 - 1) as u16)
                    ));
                } else if headroom < 0x10 {
                    warn(&mut issues, format!(
                        "{name} ends at {} - only {headroom} bytes of stack left above it",
                        hex4((b1 - 1) as u16)
                    ));
                }
            }
            if overlaps(b0 as u32, b1 as u32, prog.0, prog.1) {
                err(&mut issues, format!(
                    "{name} run region {}..{} overlaps the program image",
                    hex4(b0), hex4(b1 - 1)
                ));
            }
        }

        // Scratch region. The decoder writes this buffer throughout the
        // decrunch, so it gets the same placement checks as the output.
        if let (Some(sc), Some(addr)) = (spec.scratch.as_ref(), plan.scratch_addr) {
            let s = (addr as u32, addr as u32 + sc.len as u32);
            let name = format!("scratch region {} {}..{}", sc.symbol, hex4(addr), hex4((s.1 - 1) as u16));
            if s.1 > 0x1_0000 {
                err(&mut issues, format!("{name} runs past $FFFF"));
            }
            if s.0 < 0x0200 {
                err(&mut issues, format!("{name} overlaps zero page / CPU stack (below $0200)"));
            }
            if let (Some(out), Some(len)) = (plan.out_addr, plan.out_len) {
                if overlaps(s.0, s.1, out as u32, out as u32 + len as u32) {
                    err(&mut issues, format!("{name} overlaps the output"));
                }
            }
            // As for the output: once the decruncher is staged elsewhere the
            // program image is dead and may be written over.
            if overlaps(s.0, s.1, prog.0, prog.1) && !plan.staged {
                err(&mut issues, format!(
                    "{name} overlaps the program image {}..{} - use stage_decruncher_at() \
                     (e.g. $0100)",
                    hex4(plan.org), hex4((program_end - 1) as u16)
                ));
            }
            for &(b0, b1, bname) in blob_regions {
                if overlaps(s.0, s.1, b0 as u32, b1 as u32) {
                    err(&mut issues, format!(
                        "{name} overlaps the relocated {bname} at {}..{}",
                        hex4(b0), hex4(b1 - 1)
                    ));
                }
            }
            for (i, mv) in plan.moves.iter().enumerate() {
                if overlaps(s.0, s.1, mv.dst as u32, mv.dst as u32 + mv.len as u32) {
                    err(&mut issues, format!(
                        "{name} overlaps the destination of move #{i} at {}..{}",
                        hex4(mv.dst), hex4((mv.dst as u32 + mv.len as u32 - 1) as u16)
                    ));
                }
            }
            if s.1 > 0xD000 && self.all_ram.is_none() {
                warn(&mut issues, format!(
                    "{name} reaches above $D000 (IO/ROM) - add all_ram() so writes land in RAM"
                ));
            }
        }

        issues
    }

    // -----------------------------------------------------------------------
    // Rendering
    // -----------------------------------------------------------------------

    /// Render the full program source for one pass. `env` carries addresses
    /// discovered in the previous pass (payload label, resume label). The
    /// last tuple element is the payload-at-final-address expectation
    /// (`payload_data` / `payload_head` label addresses) the assembled
    /// program must satisfy - checked in [`Decruncher::build`].
    #[allow(clippy::type_complexity)]
    fn render(
        &self,
        plan: &Plan,
        env: &HashMap<&'static str, u16>,
    ) -> Result<
        (
            String,
            Vec<(u16, u16, &'static str)>,
            Vec<(String, u16, String)>,
            Option<(u16, u16)>,
        ),
        GenError,
    > {
        let spec = self.spec();
        let payload_label_addr = *env.get("payload_data").unwrap_or(&0xC000);
        let comp_data_num = plan.comp_data_addr.unwrap_or(payload_label_addr);

        // ---- shared constant header for decoder fragments -----------------
        let mut decoder_consts = String::new();
        decoder_consts.push_str(&format!("comp_data = {}\n", hex4(comp_data_num)));
        if let Some(out) = plan.out_addr {
            decoder_consts.push_str(&format!("out_addr = {}\n", hex4(out)));
        }
        if spec.needs.out_len || spec.eof == EofKind::Length {
            if let Some(l) = plan.out_len {
                decoder_consts.push_str(&format!("out_len = {}\n", l));
            }
        }
        if spec.needs.comp_data_len {
            if let Some(l) = plan.payload_len {
                decoder_consts.push_str(&format!("comp_data_len = {}\n", l));
            }
        }
        if spec.needs.zx_mode {
            if let Some(m) = plan.mode_byte {
                decoder_consts.push_str(&format!("zx_mode = {}\n", m));
            }
        }

        let body = configured_body(self, plan.zp_base, plan.scratch_addr);

        // ---- decode tail (epilogue) ----------------------------------------
        let mut tail = String::new();
        for c in &self.custom_post {
            tail.push_str(c);
            tail.push('\n');
        }
        if let Some((_, Some(restore))) = self.all_ram {
            tail.push_str(&format!("        LDA #{}\n        STA $01\n        CLI\n", hex2(restore)));
        }
        match self.done {
            Done::Rts => tail.push_str("        RTS\n"),
            Done::Jmp(a) => tail.push_str(&format!("        JMP {}\n", hex4(a))),
            Done::RunBasic => tail.push_str(
                "        JSR $A533       ; relink the program's next-line pointers from TXTTAB\n\
                 \x20       LDA $22         ; the relink stops AT the two-byte end-of-program link\n\
                 \x20       CLC\n\
                 \x20       ADC #$02        ; VARTAB is the byte after that link\n\
                 \x20       STA $2D\n\
                 \x20       LDA $23\n\
                 \x20       ADC #$00\n\
                 \x20       STA $2E\n\
                 \x20       JSR $A659       ; CLR\n\
                 \x20       JMP $A7AE       ; BASIC interpreter loop\n",
            ),
        }

        // ---- blobs ----------------------------------------------------------
        // Each relocated piece: (name, run_at, fragment source). Assembled
        // separately; embedded as .byte rows; copied into place by init.
        let mut blobs: Vec<(&'static str, u16, String)> = Vec::new();

        // Folded mover: the moves become the FRONT of the staged blob and fall
        // through into the decoder entry - they survive destinations that
        // overwrite the program image (like a relocated mover) without a
        // second blob, install loop and JMP chain. An explicit mover_at()
        // override wins over folding.
        let mover_folded =
            self.fold_mover && self.mover_at.is_none() && self.stage_at.is_some() && !plan.moves.is_empty();

        // Payload-at-final-address bookkeeping, fixed while building the
        // staged blob (the head move lives at the blob front):
        // (head_len, bulk_len, boot_len). Used by the emission below.
        let mut in_place_split: Option<(u32, u32, u32)> = None;

        if let Some(stage_at) = self.stage_at {
            // The decoder core: wrapper (JSR entry / done tail) + body. The
            // consts are pure `symbol = value` lines (zero assembled bytes),
            // so measuring consts+core gives the core's size.
            let seed = caller_seed_asm(spec, plan.zp_base.or(spec.zp_base_default).unwrap_or(0));
            let core = format!(
                "staged_decrunch:\n{seed}        JSR {entry}\n{tail}{body}\n",
                seed = seed,
                entry = spec.entry,
                tail = tail,
                body = body,
            );
            let mut moves_frag = String::new();
            if let Some(fs) = plan.in_place_start {
                if fs as u32 >= plan.org as u32 {
                    return Err(GenError::Config(format!(
                        "payload_in_place: final start {} must lie below the org {}",
                        hex4(fs),
                        hex4(plan.org)
                    )));
                }
                // Measure the core, then iterate the head-move / install-loop
                // sizes to a fixed point (a copy loop is 13 bytes up to one
                // page, 22 above) and derive the head/bulk split: the bulk
                // loads at its final addresses (file offset boot_len == final
                // address fs + head_len), only the head window at the file
                // tail is moved down by the loop at the blob front.
                let core_len = {
                    let src = format!("*={}\n{decoder_consts}{core}", hex4(stage_at));
                    let mut asm = asm6502::Assembler6502::new();
                    asm.set_origin(stage_at);
                    asm.assemble_bytes(&src)
                        .map_err(|e| GenError::Asm(format!("staged core fragment: {e:?}")))?
                        .len() as u32
                };
                let clen = plan
                    .payload_bytes
                    .as_ref()
                    .expect("payload_in_place requires an inline payload (checked in plan)")
                    .len() as u32;
                let stub_len: u32 = if self.basic_stub { 12 } else { 0 };
                let init_len: u32 = if self.all_ram.is_some() { 5 } else { 0 };
                let pre_len: u32 = if self.custom_pre.is_empty() {
                    0
                } else {
                    let src =
                        format!("*={}\n{}\n", hex4(plan.org), self.custom_pre.join("\n"));
                    let mut asm = asm6502::Assembler6502::new();
                    asm.set_origin(plan.org);
                    asm.assemble_bytes(&src)
                        .map_err(|e| GenError::Asm(format!("custom_pre fragment: {e:?}")))?
                        .len() as u32
                };
                let loop_size =
                    |len: u32| if len == 0 { 0 } else if len <= 256 { 13 } else { 22 };
                let mut hm: u32 = 13;
                let (head_len, bulk_len, boot_len) = loop {
                    let blob_len = hm + core_len;
                    let boot = stub_len + init_len + pre_len + loop_size(blob_len) + 3;
                    let head = (boot + (plan.org as u32 - fs as u32)).min(clen);
                    let hm2 = loop_size(head);
                    if hm2 == hm {
                        break (head, clen - head, boot);
                    }
                    hm = hm2;
                };
                let head_image = plan.org as u32 + boot_len + bulk_len;
                copy_forward(
                    "move_head",
                    &hex4(head_image as u16),
                    &hex4(fs),
                    head_len as u16,
                    &mut moves_frag,
                );
                in_place_split = Some((head_len, bulk_len, boot_len));
            } else if mover_folded {
                // Folded moves first - boot JMPs to the blob start, so the
                // moves run, then execution falls into the decoder wrapper.
                for (i, mv) in plan.moves.iter().enumerate() {
                    // Numeric source only: the blob assembles separately and
                    // cannot see the main program's labels.
                    let resolved_src = mv.src_addr.unwrap_or(payload_label_addr);
                    render_move(&mut moves_frag, i, mv, &hex4(resolved_src), resolved_src);
                }
            }
            // With no folded moves the run address = the JMP target.
            let frag = format!("{decoder_consts}{moves_frag}{core}");
            blobs.push(("staged_decruncher", stage_at, frag));
        }

        let mover_relocated = self.mover_at.is_some() && !plan.moves.is_empty();
        if let Some(mover_at) = self.mover_at {
            if !plan.moves.is_empty() {
                // All moves run from the relocated mover, then control goes to
                // the decoder (staged) or back to the main program (resume).
                let mut frag = String::new();
                frag.push_str("relocated_mover:\n");
                for (i, mv) in plan.moves.iter().enumerate() {
                    let resolved_src = mv.src_addr.unwrap_or(payload_label_addr);
                    let src = mv.src_addr.map(hex4).unwrap_or_else(|| hex4(payload_label_addr));
                    render_move(&mut frag, i, mv, &src, resolved_src);
                }
                let next = if plan.staged {
                    hex4(self.stage_at.unwrap())
                } else {
                    hex4(*env.get("after_moves").unwrap_or(&0xC000))
                };
                frag.push_str(&format!("        JMP {}\n", next));
                blobs.push(("relocated_mover", mover_at, frag));
            }
        }

        // Assemble blobs to learn their sizes/regions.
        let mut blob_bytes: Vec<(usize, Vec<u8>)> = Vec::new();
        let mut blob_regions: Vec<(u16, u16, &'static str)> = Vec::new();
        for (i, (name, run_at, frag)) in blobs.iter().enumerate() {
            let src = format!("*={}\n{}", hex4(*run_at), frag);
            let mut asm = asm6502::Assembler6502::new();
            asm.set_origin(*run_at);
            let bytes = asm
                .assemble_bytes(&src)
                .map_err(|e| GenError::Asm(format!("{name} fragment: {e:?}")))?;
            blob_regions.push((*run_at, *run_at + bytes.len() as u16, name));
            blob_bytes.push((i, bytes));
        }

        // ---- main program ----------------------------------------------------
        let mut s = String::new();
        s.push_str(&format!(
            "; ============================================================\n\
             ; generated by lzan-c64: {spec}\n\
             ; org {org}{zp}{scr}\n\
             ; ============================================================\n",
            spec = spec,
            org = hex4(plan.org),
            zp = plan
                .zp_base
                // Widen before the subtraction: a span ending exactly at $FF
                // (e.g. aplib $F7+9, exomizer $F7+9) would overflow u8 here.
                .map(|b| format!("  zp {}-{}", hex2(b), hex2((b as u16 + spec.zp_len as u16 - 1) as u8)))
                .unwrap_or_default(),
            scr = plan
                .scratch_addr
                .zip(spec.scratch.as_ref())
                .map(|(a, sc)| format!("  {} {}(+{})", sc.symbol, hex4(a), sc.len))
                .unwrap_or_default(),
        ));
        s.push_str(&format!("*={}\n", hex4(plan.org)));

        if self.basic_stub {
            let sys_target = plan.org + 12; // stub is always 12 bytes below
            s.push_str(&format!(
                "; BASIC stub: 0 SYS{sys_target} (load-address bytes $01 $08 are added by prg())\n"
            ));
            let digits: Vec<u8> = sys_target.to_string().into_bytes();
            let mut stub: Vec<u8> = Vec::new();
            // next-line pointer, line number 0, $9E (SYS), digits, EOL, end.
            let line_len = 2 + 2 + 1 + digits.len() + 1;
            let next = plan.org as usize + line_len;
            stub.push((next & 0xFF) as u8);
            stub.push((next >> 8) as u8);
            stub.push(0x00);
            stub.push(0x00);
            stub.push(0x9E);
            stub.extend_from_slice(&digits);
            stub.push(0x00);
            stub.push(0x00);
            stub.push(0x00);
            debug_assert_eq!(stub.len(), 12, "SYS stub must stay 12 bytes for 4-digit targets");
            byte_rows(&stub, &mut s);
        }

        s.push_str("start:\n");
        if let Some((val, _)) = self.all_ram {
            s.push_str(&format!("        SEI\n        LDA #{}\n        STA $01\n", hex2(val)));
        }
        for c in &self.custom_pre {
            s.push_str(c);
            s.push('\n');
        }

        // Copy every relocated blob into place (program image still intact).
        for (i, (name, run_at, _)) in blobs.iter().enumerate() {
            let len = blob_bytes[i].1.len() as u16;
            copy_forward(
                &format!("install_{name}"),
                &format!("blob_{name}"),
                &hex4(*run_at),
                len,
                &mut s,
            );
        }

        // Moves: folded into the staged blob, via the relocated mover, or
        // inline here.
        if mover_folded {
            // Nothing here - the JMP into the staged blob below runs them.
            s.push_str("after_moves:\n");
        } else if mover_relocated {
            s.push_str(&format!("        JMP {}\n", hex4(self.mover_at.unwrap())));
            s.push_str("after_moves:\n");
        } else {
            for (i, mv) in plan.moves.iter().enumerate() {
                let resolved_src = mv.src_addr.unwrap_or(payload_label_addr);
                let src = mv
                    .src_addr
                    .map(hex4)
                    .unwrap_or_else(|| "payload_data".to_string());
                render_move(&mut s, i, mv, &src, resolved_src);
            }
            s.push_str("after_moves:\n");
        }

        // Decode: staged (JMP into blob) or inline (JSR + tail + body).
        if let Some(stage_at) = self.stage_at {
            s.push_str(&format!("        JMP {}\n", hex4(stage_at)));
        } else {
            s.push_str(&decoder_consts);
            s.push_str(&caller_seed_asm(spec, plan.zp_base.or(spec.zp_base_default).unwrap_or(0)));
            s.push_str(&format!("        JSR {}\n", spec.entry));
            s.push_str(&tail);
            s.push_str(&body);
            s.push('\n');
        }

        // Embedded blobs and payload. The payload-at-final-address layout
        // puts the payload FIRST: its bulk must land at file offset boot_len,
        // which by construction equals its decode-time address; the head
        // window and the blob images follow. The classic layout keeps blobs
        // first and the payload at the file end.
        let mut in_place_expect: Option<(u16, u16)> = None;
        if let Some((head_len, bulk_len, boot_len)) = in_place_split {
            let bytes = plan
                .payload_bytes
                .as_ref()
                .expect("payload_in_place requires an inline payload (checked in plan)");
            s.push_str("; stream bulk, loaded directly at its decode-time address\n");
            s.push_str("payload_data:\n");
            byte_rows(&bytes[head_len as usize..], &mut s);
            s.push_str("; stream head window, moved down by the staged blob's front loop\n");
            s.push_str("payload_head:\n");
            byte_rows(&bytes[..head_len as usize], &mut s);
            for (i, (name, _, _)) in blobs.iter().enumerate() {
                s.push_str(&format!("blob_{name}:\n"));
                byte_rows(&blob_bytes[i].1, &mut s);
            }
            in_place_expect = Some((
                (plan.org as u32 + boot_len) as u16,
                (plan.org as u32 + boot_len + bulk_len) as u16,
            ));
        } else {
            for (i, (name, _, _)) in blobs.iter().enumerate() {
                s.push_str(&format!("blob_{name}:\n"));
                byte_rows(&blob_bytes[i].1, &mut s);
            }
            if let Some(bytes) = &plan.payload_bytes {
                s.push_str("payload_data:\n");
                byte_rows(bytes, &mut s);
            } else if let Some((path, _)) = &plan.payload_incbin {
                s.push_str("payload_data:\n");
                s.push_str(&format!(".incbin \"{path}\"\n"));
            }
        }

        let fragments = blobs
            .into_iter()
            .map(|(name, run_at, frag)| (name.to_string(), run_at, frag))
            .collect();
        Ok((s, blob_regions, fragments, in_place_expect))
    }

    /// Full pipeline: plan → two-pass render/assemble → validate.
    pub fn build(&self) -> Result<Built, GenError> {
        let plan = self.plan()?;

        let mut env: HashMap<&'static str, u16> = HashMap::new();
        #[allow(clippy::type_complexity)]
        let mut last: Option<(
            String,
            Vec<u8>,
            HashMap<String, u16>,
            Vec<(u16, u16, &'static str)>,
            Vec<(String, u16, String)>,
        )> = None;

        for _pass in 0..4 {
            let (source, blob_regions, fragments, in_place_expect) = self.render(&plan, &env)?;
            let mut asm = asm6502::Assembler6502::new();
            asm.set_origin(plan.org);
            let (bytes, symbols) = asm
                .assemble_with_symbols(&source)
                .map_err(|e| GenError::Asm(format!("main program: {e:?}")))?;

            // Payload-at-final-address invariant: the stream bulk must have
            // landed exactly at its decode-time address (file offset ==
            // boot size) and the head window where the blob's front loop
            // reads it. A mismatch means the boot-size arithmetic drifted
            // from the emitted code - fail loudly, never emit a broken SFX.
            if let Some((bulk_at, head_at)) = in_place_expect {
                let got_bulk = symbols.get("payload_data").copied();
                let got_head = symbols.get("payload_head").copied();
                if got_bulk != Some(bulk_at) || got_head != Some(head_at) {
                    return Err(GenError::Asm(format!(
                        "payload_in_place layout drift: bulk at {:?} (expected {}), head at \
                         {:?} (expected {})",
                        got_bulk.map(hex4),
                        hex4(bulk_at),
                        got_head.map(hex4),
                        hex4(head_at)
                    )));
                }
            }

            let mut next_env = env.clone();
            if let Some(&p) = symbols.get("payload_data") {
                next_env.insert("payload_data", p);
            }
            if let Some(&a) = symbols.get("after_moves") {
                next_env.insert("after_moves", a);
            }

            let stable = next_env == env;
            env = next_env;
            let done = stable;
            last = Some((source, bytes, symbols, blob_regions, fragments));
            if done {
                break;
            }
        }

        let (source, bytes, symbols, blob_regions, fragments) =
            last.expect("at least one render pass ran");
        let program_end = plan.org as u32 + bytes.len() as u32;
        if program_end > 0x1_0000 {
            return Err(GenError::Config(format!(
                "program image {}+{} bytes runs past $FFFF",
                hex4(plan.org),
                bytes.len()
            )));
        }

        let issues = self.validate_plan(&plan, program_end, &blob_regions);
        let errors: Vec<Issue> = issues
            .iter()
            .filter(|i| i.severity == Severity::Error)
            .cloned()
            .collect();
        if !errors.is_empty() {
            return Err(GenError::Validation(errors));
        }
        let warnings: Vec<Issue> = issues
            .into_iter()
            .filter(|i| i.severity == Severity::Warning)
            .collect();

        // Prepend warnings as comments so the generated file is self-auditing.
        let source = if warnings.is_empty() {
            source
        } else {
            let mut w = String::new();
            for i in &warnings {
                w.push_str(&format!("; WARNING: {}\n", i.msg));
            }
            format!("{w}{source}")
        };

        Ok(Built { source, origin: plan.org, bytes, symbols, warnings, fragments })
    }

    // -----------------------------------------------------------------------
    // Public emitters
    // -----------------------------------------------------------------------

    /// Validation only (plan + one render pass); returns every issue found.
    pub fn validate(&self) -> Vec<Issue> {
        match self.plan() {
            Err(GenError::Config(msg)) => vec![Issue { severity: Severity::Error, msg }],
            Err(e) => vec![Issue { severity: Severity::Error, msg: e.to_string() }],
            Ok(plan) => match self.render(&plan, &HashMap::new()) {
                Err(e) => vec![Issue { severity: Severity::Error, msg: e.to_string() }],
                Ok((source, blob_regions, _fragments, _in_place_expect)) => {
                    let mut asm = asm6502::Assembler6502::new();
                    asm.set_origin(plan.org);
                    match asm.assemble_bytes(&source) {
                        Err(e) => vec![Issue {
                            severity: Severity::Error,
                            msg: format!("assembly: {e:?}"),
                        }],
                        Ok(bytes) => self.validate_plan(
                            &plan,
                            plan.org as u32 + bytes.len() as u32,
                            &blob_regions,
                        ),
                    }
                }
            },
        }
    }

    /// The complete program as asm6502 source ("straight from RAM").
    pub fn program_source(&self) -> Result<String, GenError> {
        Ok(self.build()?.source)
    }

    /// Write the program source to a file.
    pub fn write_source(&self, path: &str) -> Result<(), GenError> {
        std::fs::write(path, self.program_source()?)?;
        Ok(())
    }

    /// Assemble the program (via the vendored asm6502).
    pub fn assemble(&self) -> Result<Built, GenError> {
        self.build()
    }

    /// A C64 `.prg`: 2-byte little-endian load address + assembled bytes.
    pub fn prg(&self) -> Result<Vec<u8>, GenError> {
        let built = self.build()?;
        let mut prg = Vec::with_capacity(built.bytes.len() + 2);
        prg.push((built.origin & 0xFF) as u8);
        prg.push((built.origin >> 8) as u8);
        prg.extend_from_slice(&built.bytes);
        Ok(prg)
    }

    /// Write a `.prg` file.
    pub fn write_prg(&self, path: &str) -> Result<(), GenError> {
        std::fs::write(path, self.prg()?)?;
        Ok(())
    }

    /// Just the routine: resolved constants + configured body, no org, no
    /// modules. Self-contained - for an embedded `pack()` / `packed_inline()`
    /// payload it appends the compressed bytes under a `comp_data:` label so
    /// the fragment assembles as-is; for `packed_at()` it binds `comp_data` to
    /// that address. For embedding into your own program.
    pub fn routine_source(&self) -> Result<String, GenError> {
        let plan = self.plan()?;
        let spec = self.spec();
        let mut s = String::new();
        if let Some(a) = self.code_addr {
            s.push_str(&format!("*={}\n", hex4(a)));
        }
        // Embedded payload with no fixed address: bind comp_data to a label and
        // emit the bytes at the end (below). Otherwise bind to the numeric addr.
        let embed_payload = plan.comp_data_addr.is_none()
            && (plan.payload_bytes.is_some() || plan.payload_incbin.is_some());
        if let Some(addr) = plan.comp_data_addr {
            s.push_str(&format!("comp_data = {}\n", hex4(addr)));
        } else if !embed_payload {
            return Err(GenError::Config(
                "routine_source(): no compressed data placement - use packed_at() for an external \
                 address, an embedded pack()/packed_inline()/packed_incbin(), or program_source()/prg()"
                    .into(),
            ));
        }
        if let Some(out) = plan.out_addr {
            s.push_str(&format!("out_addr = {}\n", hex4(out)));
        }
        if (spec.needs.out_len || spec.eof == EofKind::Length) && plan.out_len.is_some() {
            s.push_str(&format!("out_len = {}\n", plan.out_len.unwrap()));
        }
        if spec.needs.comp_data_len && plan.payload_len.is_some() {
            s.push_str(&format!("comp_data_len = {}\n", plan.payload_len.unwrap()));
        }
        if spec.needs.zx_mode && plan.mode_byte.is_some() {
            s.push_str(&format!("zx_mode = {}\n", plan.mode_byte.unwrap()));
        }
        s.push_str(&configured_body(self, plan.zp_base, plan.scratch_addr));
        if embed_payload {
            s.push_str("comp_data:\n");
            if let Some(bytes) = &plan.payload_bytes {
                byte_rows(bytes, &mut s);
            } else if let Some((path, _)) = &plan.payload_incbin {
                s.push_str(&format!(".incbin \"{path}\"\n"));
            }
        }
        Ok(s)
    }

    /// The routine body only, with zero-page / scratch overrides applied and
    /// NO constants emitted - the caller defines `comp_data`, `out_addr` etc.
    /// (This is the flavour the decrunch-test harness consumes.)
    pub fn routine_source_bare(&self) -> Result<String, GenError> {
        let spec = self.spec();
        let zp_base = if spec.zp_len > 0 {
            let base = self.zp_base.or(spec.zp_base_default).unwrap();
            if base as u16 + spec.zp_len as u16 > 0x100 {
                return Err(GenError::Config(format!(
                    "zp_base {} + span {} overflows page 0",
                    hex2(base),
                    spec.zp_len
                )));
            }
            Some(base)
        } else {
            None
        };
        let scratch = match (&spec.scratch, self.scratch_addr) {
            (Some(sc), sel) => Some(sel.unwrap_or(sc.default)),
            (None, _) => None,
        };
        Ok(configured_body(self, zp_base, scratch))
    }
}

/// Emit one move (auto-direction: descending when dst > src and ranges
/// overlap, ascending otherwise). `resolved_src` is the numeric source address
/// (the payload's known load address for `MoveSrc::Payload`, or the explicit
/// `move_data` source) used solely to pick the copy direction; `src` is the
/// asm operand string (a numeric literal or the `payload_data` label).
fn render_move(out: &mut String, idx: usize, mv: &ResolvedMove, src: &str, resolved_src: u16) {
    let name = format!("move_{idx}");
    let s0 = resolved_src as u32;
    let d0 = mv.dst as u32;
    // Descending copy required when dst > src and the ranges overlap.
    let overlap_desc = d0 > s0 && d0 < s0 + mv.len as u32;
    if overlap_desc {
        copy_backward(&name, src, &hex4(mv.dst), mv.len, out);
    } else {
        copy_forward(&name, src, &hex4(mv.dst), mv.len, out);
    }
}
