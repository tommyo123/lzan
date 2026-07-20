//! The decrunch framework builder: configure a routine (zero page, addresses,
//! payload) plus optional modules (BASIC stub, all-RAM banking, data movers,
//! decruncher staging), then emit asm6502 source, assembled bytes or a `.prg`.

use crate::registry::{find_routine, Direction, Format, RoutineSpec, Variant};

/// What happens after decrunching completes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Done {
    /// Plain `RTS` (default) - returns to whoever called the program.
    Rts,
    /// `JMP` to an entry point (self-extracting program style).
    Jmp(u16),
}

/// Where the compressed payload comes from.
#[derive(Clone, Debug)]
pub enum Packed {
    /// Already in C64 memory at this address (loader placed it there).
    External { addr: u16 },
    /// A pre-compressed raw stream, embedded in the program as `.byte` rows.
    Inline(Vec<u8>),
    /// Compress this input with the routine's matching lzan encoder at emit
    /// time, then embed like `Inline`.
    FromInput(Vec<u8>),
    /// Embed a file on disk via `.incbin` (asm6502 reads it at assembly time).
    Incbin { path: String, len: u16 },
}

/// A "move data" step executed before decrunching.
#[derive(Clone, Debug)]
pub struct MoveData {
    /// Source: either an absolute address or the embedded payload.
    pub src: MoveSrc,
    /// Length in bytes; `None` = length of the embedded payload.
    pub len: Option<u16>,
    /// Destination address: start address, or (when `top_align`) the address
    /// the block's LAST byte should land on.
    pub dst: u16,
    /// Interpret `dst` as the end address (`move_packed_to_top`).
    pub top_align: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveSrc {
    Addr(u16),
    /// The embedded payload (wherever it lands in the program image).
    Payload,
}

/// Validation issue severity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Blocks emission.
    Error,
    /// Emitted as a `; WARNING:` comment in the generated source.
    Warning,
}

#[derive(Clone, Debug)]
pub struct Issue {
    pub severity: Severity,
    pub msg: String,
}

/// Generation error.
#[derive(Debug)]
pub enum GenError {
    /// No routine registered for the requested (format, direction, variant).
    NoSuchRoutine(String),
    /// Missing or invalid configuration.
    Config(String),
    /// Hard validation errors (each also listed individually).
    Validation(Vec<Issue>),
    /// asm6502 failed to assemble generated source (a generator bug).
    Asm(String),
    Io(std::io::Error),
}

impl std::fmt::Display for GenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GenError::NoSuchRoutine(s) => write!(f, "no such routine: {s}"),
            GenError::Config(s) => write!(f, "config error: {s}"),
            GenError::Validation(v) => {
                write!(f, "validation failed:")?;
                for i in v.iter().filter(|i| i.severity == Severity::Error) {
                    write!(f, "\n  - {}", i.msg)?;
                }
                Ok(())
            }
            GenError::Asm(s) => write!(f, "internal assembly error: {s}"),
            GenError::Io(e) => write!(f, "io error: {e}"),
        }
    }
}

impl std::error::Error for GenError {}

impl From<std::io::Error> for GenError {
    fn from(e: std::io::Error) -> Self {
        GenError::Io(e)
    }
}

/// The framework class. Build with [`Decruncher::new`], chain configuration,
/// then emit with [`Decruncher::program_source`] / [`Decruncher::assemble`] /
/// [`Decruncher::prg`] (or grab just the routine via
/// [`Decruncher::routine_source`]).
#[derive(Clone, Debug)]
pub struct Decruncher {
    pub(crate) spec: &'static RoutineSpec,
    pub(crate) code_addr: Option<u16>,
    pub(crate) zp_base: Option<u8>,
    pub(crate) scratch_addr: Option<u16>,
    pub(crate) packed: Option<Packed>,
    pub(crate) packed_len_override: Option<u16>,
    pub(crate) out_addr: Option<u16>,
    pub(crate) out_len: Option<u16>,
    pub(crate) mode_byte: Option<u8>,
    pub(crate) basic_stub: bool,
    pub(crate) all_ram: Option<(u8, Option<u8>)>, // (value, restore)
    pub(crate) moves: Vec<MoveData>,
    pub(crate) mover_at: Option<u16>,
    pub(crate) fold_mover: bool,
    pub(crate) in_place_start: Option<u16>,
    pub(crate) stage_at: Option<u16>,
    pub(crate) custom_pre: Vec<String>,
    pub(crate) custom_post: Vec<String>,
    pub(crate) done: Done,
    /// A fully-composed decoder body that replaces the routine's static source
    /// for this build (per-crunch tailored decoder; see [`Decruncher::body_override`]).
    pub(crate) body_override: Option<String>,
}

impl Decruncher {
    /// Standard variant of a format/direction.
    pub fn new(format: Format, direction: Direction) -> Result<Self, GenError> {
        Self::with_variant(format, direction, Variant::Standard)
    }

    /// Specific variant (`OptSize` / `OptSpeed` / `AltExtreme`).
    pub fn with_variant(
        format: Format,
        direction: Direction,
        variant: Variant,
    ) -> Result<Self, GenError> {
        let spec = find_routine(format, direction, variant).ok_or_else(|| {
            GenError::NoSuchRoutine(format!(
                "{}/{}/{}",
                format.as_str(),
                direction.as_str(),
                variant.as_str()
            ))
        })?;
        Ok(Decruncher {
            spec,
            code_addr: None,
            zp_base: None,
            scratch_addr: None,
            packed: None,
            packed_len_override: None,
            out_addr: None,
            out_len: None,
            mode_byte: None,
            basic_stub: false,
            all_ram: None,
            moves: Vec::new(),
            mover_at: None,
            fold_mover: false,
            in_place_start: None,
            stage_at: None,
            custom_pre: Vec::new(),
            custom_post: Vec::new(),
            done: Done::Rts,
            body_override: None,
        })
    }

    /// Prefer the FASTEST decoder for this format/direction.
    ///
    /// By default the framework selects the balanced `Standard` decoder (tuned
    /// for a size/speed compromise). Calling this swaps to the dedicated
    /// `OptSpeed` variant when one is registered for the current
    /// (format, direction); when none exists it leaves the balanced decoder in
    /// place (a safe no-op). The decoded stream is identical - only the decoder
    /// body changes - so the packed payload is unaffected.
    ///
    /// Idempotent, and order-independent w.r.t. the other builder options as long
    /// as it is called before `.prg()` / `.program_source()`.
    pub fn priority_speed(mut self) -> Self {
        if let Some(sp) = find_routine(self.spec.format, self.spec.direction, Variant::OptSpeed) {
            self.spec = sp;
        }
        self
    }

    /// The underlying routine's metadata.
    pub fn spec(&self) -> &'static RoutineSpec {
        self.spec
    }

    // ---- core configuration -------------------------------------------------

    /// Origin the program assembles at. Defaults to `$0801` when the BASIC
    /// stub module is enabled, `$1000` otherwise.
    pub fn code_address(mut self, addr: u16) -> Self {
        self.code_addr = Some(addr);
        self
    }

    /// Base of the zero-page span the routine may use (`spec().zp_len` bytes
    /// from here). Defaults to the routine's own default base.
    pub fn zero_page(mut self, base: u8) -> Self {
        self.zp_base = Some(base);
        self
    }

    /// Convenience: give an inclusive ZP range (e.g. `$FA..=$FE`) and get an
    /// error immediately if the routine needs more bytes than the range holds.
    pub fn zero_page_range(self, lo: u8, hi: u8) -> Result<Self, GenError> {
        let have = (hi as u16).saturating_sub(lo as u16) + 1;
        if (self.spec.zp_len as u16) > have {
            return Err(GenError::Config(format!(
                "{} needs {} zero-page bytes; ${lo:02X}-${hi:02X} only holds {have}",
                self.spec, self.spec.zp_len
            )));
        }
        Ok(self.zero_page(lo))
    }

    /// Base address for the routine's scratch region (Subsizer code-length
    /// table, Shrinkler/upkr probability RAM). Only meaningful when
    /// `spec().scratch` is `Some`.
    pub fn scratch_address(mut self, addr: u16) -> Self {
        self.scratch_addr = Some(addr);
        self
    }

    // ---- payload ------------------------------------------------------------

    /// Compressed data already sits in memory at `addr` (external loader).
    pub fn packed_at(mut self, addr: u16) -> Self {
        self.packed = Some(Packed::External { addr });
        self
    }

    /// Byte length of the packed stream. Required for `packed_at` when the
    /// routine (or a move) needs it; inferred automatically for inline data.
    pub fn packed_len(mut self, len: u16) -> Self {
        self.packed_len_override = Some(len);
        self
    }

    /// Embed a pre-compressed raw stream in the program.
    pub fn packed_inline(mut self, stream: Vec<u8>) -> Self {
        self.packed = Some(Packed::Inline(stream));
        self
    }

    /// Embed a compressed stream from a file on disk (`.incbin`). `len` must be
    /// the file's byte length - it drives `comp_data_len` and payload-move
    /// lengths. When the file is reachable at build time (resolved relative to
    /// the process working directory) its real size is checked against `len`
    /// and a mismatch is a `GenError::Config`; if it is not reachable then
    /// (because asm6502 resolves `.incbin` from its own working directory) the
    /// supplied `len` is trusted as-is.
    pub fn packed_incbin(mut self, path: &str, len: u16) -> Self {
        self.packed = Some(Packed::Incbin { path: path.replace('\\', "/"), len });
        self
    }

    /// Compress `input` with the routine's matching lzan encoder at emit time
    /// and embed the stream. Also derives `out_len` and (for LZAN-full) the
    /// mode byte automatically.
    pub fn pack(mut self, input: &[u8]) -> Self {
        if self.out_len.is_none() {
            self.out_len = Some(input.len() as u16);
        }
        self.packed = Some(Packed::FromInput(input.to_vec()));
        self
    }

    // ---- output -------------------------------------------------------------

    /// Where the decompressed data goes.
    pub fn output(mut self, addr: u16) -> Self {
        self.out_addr = Some(addr);
        self
    }

    /// Decompressed length. Required for backward routines and formats
    /// without an in-stream EOF; derived automatically by `pack()`.
    pub fn output_len(mut self, len: u16) -> Self {
        self.out_len = Some(len);
        self
    }

    /// LZAN-full stream mode byte (`zx_mode`). Set automatically by `pack()`;
    /// required when embedding a pre-compressed LZAN-full stream.
    pub fn mode_byte(mut self, mode: u8) -> Self {
        self.mode_byte = Some(mode);
        self
    }

    // ---- modules ------------------------------------------------------------

    /// Prepend the BASIC autostart stub (`0 SYS...`) and force org `$0801`.
    /// Together with [`Decruncher::prg`] this yields a self-extracting `.prg`
    /// (the two `$01 $08` load-address bytes are added by `prg()`).
    pub fn basic_stub(mut self) -> Self {
        self.basic_stub = true;
        self
    }

    /// `SEI` + store `$34` to `$01`: bank out BASIC/KERNAL/IO so all 64 KB of
    /// RAM is visible during decrunch.
    pub fn all_ram(self) -> Self {
        self.all_ram_with(0x34, None)
    }

    /// `SEI` + store `value` to `$01`; optionally restore another value (and
    /// `CLI`) after decrunching.
    pub fn all_ram_with(mut self, value: u8, restore: Option<u8>) -> Self {
        self.all_ram = Some((value, restore));
        self
    }

    /// Move `len` bytes from `src` to `dst` before decrunching. Copy direction
    /// (ascending/descending) is chosen automatically so overlapping moves are
    /// safe.
    pub fn move_data(mut self, src: u16, len: u16, dst: u16) -> Self {
        self.moves.push(MoveData { src: MoveSrc::Addr(src), len: Some(len), dst, top_align: false });
        self
    }

    /// Move the packed data (inline payload or `packed_at` block) so it STARTS
    /// at `dst`; the decoder then reads it from there.
    pub fn move_packed_to(mut self, dst: u16) -> Self {
        self.moves.push(MoveData { src: MoveSrc::Payload, len: None, dst, top_align: false });
        self
    }

    /// Move the packed data so its LAST byte lands at `top` (e.g. `$FFFF`),
    /// the classic layout for forward in-place decrunching from the top of
    /// memory. Requires the packed length to be known (inline payload, or
    /// `packed_len`).
    pub fn move_packed_to_top(mut self, top: u16) -> Self {
        self.moves.push(MoveData { src: MoveSrc::Payload, len: None, dst: top, top_align: true });
        self
    }

    /// Relocate the move routine itself to `addr` (e.g. `$0334`, the tape
    /// buffer) and run it from there - required when a move's destination
    /// overlaps the program image that contains the mover.
    pub fn mover_at(mut self, addr: u16) -> Self {
        self.mover_at = Some(addr);
        self
    }

    /// Fold the move routine into the FRONT of the staged decruncher blob:
    /// the moves run from staged memory (surviving a destination that
    /// overlaps the program image, like a relocated mover) and fall straight
    /// through into the decoder entry - one blob, one install loop, no
    /// separate mover placement. Requires `stage_decruncher_at()`. Ignored
    /// when `mover_at()` is also set (the explicit override wins).
    pub fn fold_mover_into_stage(mut self) -> Self {
        self.fold_mover = true;
        self
    }

    /// Payload-at-final-address layout: the packed stream's decode-time
    /// position is `[final_start, final_start + len)`, and the file is laid
    /// out so the stream BULK loads directly at those addresses - only the
    /// head window (the stream bytes whose final address lies below the end
    /// of the boot code) is parked at the file tail and moved down by a
    /// small loop folded into the staged blob. Replaces `move_packed_to()`
    /// for the backward/margin layout: identical decode-time geometry, no
    /// whole-stream copy at boot. Requires `stage_decruncher_at()` and an
    /// inline payload; mutually exclusive with payload moves.
    pub fn payload_in_place(mut self, final_start: u16) -> Self {
        self.in_place_start = Some(final_start);
        self
    }

    /// Copy the decruncher to `addr` (e.g. `$0100`, the stack page) and run it
    /// from there - required when the output region overwrites the program
    /// image (decrunching over the program's own load address).
    pub fn stage_decruncher_at(mut self, addr: u16) -> Self {
        self.stage_at = Some(addr);
        self
    }

    /// Insert a custom asm6502 fragment before the decrunch call.
    pub fn custom_pre(mut self, asm: &str) -> Self {
        self.custom_pre.push(asm.to_string());
        self
    }

    /// Insert a custom asm6502 fragment after decrunching, before done.
    pub fn custom_post(mut self, asm: &str) -> Self {
        self.custom_post.push(asm.to_string());
        self
    }

    /// Jump here when decrunching is done (instead of the default `RTS`).
    pub fn jmp_when_done(mut self, addr: u16) -> Self {
        self.done = Done::Jmp(addr);
        self
    }

    // ---- per-crunch tailored decoder ---------------------------------------

    /// Install a fully-composed decoder body that replaces the routine's static
    /// `source` for this build. This is the single injection point for the
    /// per-crunch tailored-decoder path ([`crate::tailored_body`]): the caller
    /// measures the compressed stream's feature traits, composes a
    /// trait-tailored body from the routine source (dead sections removed), and
    /// installs it here. The override MUST keep the config-defaults block
    /// markers intact - zero-page / scratch overrides are still spliced into it
    /// exactly as for the static source - and it must assemble to a body no
    /// larger than the static one. Passing the unmodified static source is a
    /// valid no-op.
    pub fn body_override(mut self, source: String) -> Self {
        self.body_override = Some(source);
        self
    }

    /// Explicitly choose the decoder-tailoring mode for `stream` - the exact
    /// compressed stream this build will embed. [`crate::DecoderTailoring::Standard`]
    /// is a no-op; [`crate::DecoderTailoring::Tailored`] measures `stream`'s
    /// feature traits and installs a tailored body via
    /// [`Decruncher::body_override`] when the format supports it (Exomizer
    /// today) and the stream leaves some gated feature unused. Returns `self`
    /// unchanged when no tailoring applies. The library forces this choice to
    /// be explicit; a host wanting "smallest of standard/tailored" builds both
    /// and compares.
    pub fn with_tailoring(self, mode: crate::DecoderTailoring, stream: &[u8]) -> Self {
        match mode {
            crate::DecoderTailoring::Standard => self,
            crate::DecoderTailoring::Tailored => {
                match crate::tailored_body(self.spec.format, self.spec.direction, stream) {
                    Some(src) => self.body_override(src),
                    None => self,
                }
            }
        }
    }
}
