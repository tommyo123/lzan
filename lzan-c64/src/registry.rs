//! Registry over the curated decrunch models in the repo-root `decrunchers/`
//! directory. Every `.s` file carries a machine-readable `;@key: value` header
//! plus a `; ---- config-defaults ----` block; this module embeds the sources
//! via `include_str!` and parses those headers into [`RoutineSpec`]s.

use std::fmt;
use std::sync::OnceLock;

/// Compression format of a routine (one per stream format lzan can encode).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Format {
    Zx02,
    Zx0,
    Lzsa1,
    Lzsa2,
    Aplib,
    TsCrunch,
    ByteBoozer2,
    Exomizer,
    Shrinkler,
    Subsizer,
    Upkr,
    PuCrunch,
    LzanMin,
    LzanFull,
}

impl Format {
    pub fn parse(s: &str) -> Option<Format> {
        Some(match s {
            "zx02" => Format::Zx02,
            "zx0" => Format::Zx0,
            "lzsa1" => Format::Lzsa1,
            "lzsa2" => Format::Lzsa2,
            "aplib" => Format::Aplib,
            "tscrunch" => Format::TsCrunch,
            "byteboozer2" => Format::ByteBoozer2,
            "exomizer" => Format::Exomizer,
            "shrinkler" => Format::Shrinkler,
            "subsizer" => Format::Subsizer,
            "upkr" => Format::Upkr,
            "pucrunch" => Format::PuCrunch,
            "lzan-min" => Format::LzanMin,
            "lzan-full" => Format::LzanFull,
            _ => return None,
        })
    }
    pub fn as_str(&self) -> &'static str {
        match self {
            Format::Zx02 => "zx02",
            Format::Zx0 => "zx0",
            Format::Lzsa1 => "lzsa1",
            Format::Lzsa2 => "lzsa2",
            Format::Aplib => "aplib",
            Format::TsCrunch => "tscrunch",
            Format::ByteBoozer2 => "byteboozer2",
            Format::Exomizer => "exomizer",
            Format::Shrinkler => "shrinkler",
            Format::Subsizer => "subsizer",
            Format::Upkr => "upkr",
            Format::PuCrunch => "pucrunch",
            Format::LzanMin => "lzan-min",
            Format::LzanFull => "lzan-full",
        }
    }
}

/// Decode direction. Backward = in-place capable (src/dst seeded at LAST byte,
/// both pointers walk down); requires `comp_data_len` + `out_len`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    Forward,
    Backward,
}

impl Direction {
    pub fn parse(s: &str) -> Option<Direction> {
        match s {
            "forward" => Some(Direction::Forward),
            "backward" => Some(Direction::Backward),
            _ => None,
        }
    }
    pub fn as_str(&self) -> &'static str {
        match self {
            Direction::Forward => "forward",
            Direction::Backward => "backward",
        }
    }
}

/// Routine variant: the size-tuned baseline, or an alternative trade-off.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Variant {
    /// The recommended baseline decoder.
    Standard,
    /// Smaller but slower alternative.
    OptSize,
    /// Bigger but faster alternative.
    OptSpeed,
    /// Alternative port of the same format (TSCrunch "extreme" BeebAsm port).
    AltExtreme,
    /// Legal-opcode-only alternative to a Standard variant that uses
    /// undocumented (illegal) opcodes - for CPUs/emulators without them.
    /// Decodes the identical stream; only the decoder body differs.
    Legal,
    /// Extra-small body meant to be staged into the stack page ($0100 slot):
    /// single-shot boot assumptions and cycle-for-byte trades that only pay
    /// off when the smaller body lets the whole staged blob fit where a
    /// bigger Standard body cannot. Callers select it explicitly when their
    /// placement allows (see LazyCruncherWorkshop's auto placement);
    /// otherwise the Standard variant remains the baseline.
    ZpStack,
}

impl Variant {
    pub fn parse(s: &str) -> Option<Variant> {
        match s {
            "standard" => Some(Variant::Standard),
            "opt-size" => Some(Variant::OptSize),
            "opt-speed" => Some(Variant::OptSpeed),
            "alt-extreme" => Some(Variant::AltExtreme),
            "legal" => Some(Variant::Legal),
            "zp-stack" => Some(Variant::ZpStack),
            _ => None,
        }
    }
    pub fn as_str(&self) -> &'static str {
        match self {
            Variant::Standard => "standard",
            Variant::OptSize => "opt-size",
            Variant::OptSpeed => "opt-speed",
            Variant::AltExtreme => "alt-extreme",
            Variant::Legal => "legal",
            Variant::ZpStack => "zp-stack",
        }
    }
}

/// How the compressed payload relates to the output address.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PayloadAbi {
    /// Decoder finds the output address via the `out_addr` symbol.
    Raw,
    /// The payload must be prefixed with a 2-byte (lo,hi) destination address
    /// (ByteBoozer2 forward reads its dst from the stream head).
    DstPrefixed,
    /// The ENCODER embeds the destination inside the stream itself
    /// (Subsizer backward: `compress_subsizer_marker_at(input, dest_end)`).
    DstInStream,
}

/// How the decoder knows when to stop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EofKind {
    /// In-stream end marker; no length needed.
    Stream,
    /// Needs an explicit length (`out_len` / `comp_data_len` constants).
    Length,
}

/// External symbols a routine's body references but does not define.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Needs {
    pub comp_data: bool,
    pub out_addr: bool,
    pub comp_data_len: bool,
    pub out_len: bool,
    pub zx_mode: bool,
}

/// A non-zero-page scratch region the routine needs (probability tables,
/// code-length tables), exposed as an overridable symbol in the
/// config-defaults block.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScratchSpec {
    pub symbol: String,
    pub len: u16,
    pub page_aligned: bool,
    pub default: u16,
}

/// One curated decruncher: parsed `;@` header + embedded source.
#[derive(Clone, Debug)]
pub struct RoutineSpec {
    pub format: Format,
    pub direction: Direction,
    pub variant: Variant,
    /// File name inside `decrunchers/` (e.g. `zx02-small-dmsc.s`).
    pub file: &'static str,
    /// Full asm6502 source text (with config-defaults block intact).
    pub source: &'static str,
    /// Entry label (`JSR` target that seeds pointers and decodes).
    pub entry: String,
    /// The lzan encoder producing this routine's stream (informational).
    pub encoder: String,
    /// vfy harness key (informational / test plumbing).
    pub vfy_key: String,
    pub payload: PayloadAbi,
    pub eof: EofKind,
    pub needs: Needs,
    /// Contiguous zero-page bytes spanned from `zp_base` (0 = uses no ZP).
    pub zp_len: u8,
    /// Default `zp_base` from the config-defaults block (None if zp_len = 0).
    pub zp_base_default: Option<u8>,
    pub scratch: Option<ScratchSpec>,
    /// Uses undocumented (illegal) opcodes.
    pub illegal: bool,
    /// Uses self-modifying code (cannot run from ROM; must be assembled for
    /// the address it runs at).
    pub smc: bool,
    /// Expected assembled body size in bytes (informational).
    pub code_bytes: u16,
}

impl fmt::Display for RoutineSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{} ({}, {} B)",
            self.format.as_str(),
            self.direction.as_str(),
            self.variant.as_str(),
            self.file,
            self.code_bytes
        )
    }
}

/// Marker lines delimiting the overridable config block in every source.
pub const CONFIG_BLOCK_BEGIN: &str = "; ---- config-defaults ----";
pub const CONFIG_BLOCK_END: &str = "; ---- end config-defaults ----";

macro_rules! sources {
    ($($file:literal),+ $(,)?) => {
        &[$(($file, include_str!(concat!("../../decrunchers/", $file)))),+]
    };
}

/// All embedded sources: (file name, source text).
static SOURCES: &[(&str, &str)] = sources![
    "zx02-small-dmsc.s",
    "zx02-small-dmsc-backward.s",
    "zx0-negativecharge-acorn.s",
    "zx0-negativecharge-acorn-backward.s",
    "lzsa1-marty-small.s",
    "lzsa1-marty-small-backward.s",
    "lzsa1-marty-small-legal.s",
    "lzsa1-marty-small-legal-backward.s",
    "lzsa2-marty-small.s",
    "lzsa2-marty-small-backward.s",
    "lzsa2-marty-small-legal.s",
    "lzsa2-marty-small-legal-backward.s",
    "aplib-apultra-brandwood-6502.s",
    "aplib-apultra-brandwood-6502-opt-size.s",
    "aplib-apultra-marty-backward.s",
    "tscrunch-savon.s",
    "tscrunch-savon-opt-size.s",
    "tscrunch-savon-backward.s",
    "tscrunch-savon-legal.s",
    "tscrunch-savon-legal-backward.s",
    "tscrunch-negativecharge-beebasm-extreme.s",
    "byteboozer2-difraia.s",
    "byteboozer2-difraia-opt-size.s",
    "byteboozer2-difraia-backward.s",
    "exomizer-lind-mem-forward.s",
    "exomizer-lind-mem-backward.s",
    "shrinkler-atari8xxl-unshrinkler.s",
    "shrinkler-atari8xxl-unshrinkler-opt-size.s",
    "shrinkler-atari8xxl-unshrinkler-backward.s",
    "subsizer-tlr-standalone.s",
    "subsizer-tlr-standalone-opt-size.s",
    "subsizer-tlr-standalone-forward.s",
    "upkr-pfusik.s",
    "upkr-pfusik-backward.s",
    "upkr-pfusik-legal.s",
    "upkr-pfusik-legal-backward.s",
    "pucrunch-lzan.s",
    "pucrunch-lzan-backward.s",
    "pucrunch-lzan-zpstack.s",
    "pucrunch-lzan-legal.s",
    "pucrunch-lzan-legal-backward.s",
    "lzan-decoder-min.s",
    "lzan-decoder-min-backward.s",
    "lzan-decoder-min-opt-speed.s",
    "lzan-decoder-full.s",
    "lzan-decoder-full-opt-speed.s",
    "lzan-decoder-full-backward.s",
];

fn parse_hex_or_dec(s: &str) -> Option<u32> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix('$') {
        u32::from_str_radix(h, 16).ok()
    } else if let Some(h) = s.strip_prefix("0x") {
        u32::from_str_radix(h, 16).ok()
    } else {
        s.parse().ok()
    }
}

/// Extract `;@key: value` from the header. Keys may appear once each.
fn meta_value<'a>(src: &'a str, key: &str) -> Option<&'a str> {
    let tag = format!(";@{}:", key);
    src.lines()
        .find_map(|l| l.trim().strip_prefix(tag.as_str()))
        .map(|v| v.trim())
}

/// Find `name = value` inside the config-defaults block.
fn config_default(src: &str, name: &str) -> Option<u32> {
    let begin = src.find(CONFIG_BLOCK_BEGIN)?;
    let end = src.find(CONFIG_BLOCK_END)?;
    for line in src[begin..end].lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix(name) {
            let rest = rest.trim_start();
            if let Some(v) = rest.strip_prefix('=') {
                // Strip a trailing comment if present.
                let v = v.split(';').next().unwrap_or("");
                return parse_hex_or_dec(v);
            }
        }
    }
    None
}

fn parse_spec(file: &'static str, source: &'static str) -> Result<RoutineSpec, String> {
    let req = |key: &str| -> Result<&str, String> {
        meta_value(source, key).ok_or_else(|| format!("{file}: missing ;@{key}:"))
    };

    let format = Format::parse(req("format")?)
        .ok_or_else(|| format!("{file}: bad ;@format:"))?;
    let direction = Direction::parse(req("direction")?)
        .ok_or_else(|| format!("{file}: bad ;@direction:"))?;
    let variant = Variant::parse(req("variant")?)
        .ok_or_else(|| format!("{file}: bad ;@variant:"))?;
    let entry = req("entry")?.to_string();
    let vfy_key = req("vfy-key")?.to_string();
    let encoder = req("encoder")?.to_string();

    let payload = match req("payload")? {
        "raw" => PayloadAbi::Raw,
        "dst-prefixed" => PayloadAbi::DstPrefixed,
        "dst-in-stream" => PayloadAbi::DstInStream,
        other => return Err(format!("{file}: bad ;@payload: {other}")),
    };
    let eof = match req("eof")? {
        "stream" => EofKind::Stream,
        "length" => EofKind::Length,
        other => return Err(format!("{file}: bad ;@eof: {other}")),
    };

    let mut needs = Needs::default();
    for item in req("needs")?.split(',').map(|s| s.trim()) {
        match item {
            "comp_data" => needs.comp_data = true,
            "out_addr" => needs.out_addr = true,
            "comp_data_len" => needs.comp_data_len = true,
            "out_len" => needs.out_len = true,
            "zx_mode" => needs.zx_mode = true,
            "" => {}
            other => return Err(format!("{file}: bad ;@needs item: {other}")),
        }
    }

    let zp_len: u8 = req("zp-len")?
        .parse()
        .map_err(|_| format!("{file}: bad ;@zp-len:"))?;
    let zp_base_default = if zp_len > 0 {
        let v = config_default(source, "zp_base")
            .ok_or_else(|| format!("{file}: zp-len > 0 but no zp_base in config-defaults"))?;
        if v > 0xFF || v as u16 + zp_len as u16 > 0x100 {
            return Err(format!("{file}: default zp_base ${v:02X} + span {zp_len} overflows page 0"));
        }
        Some(v as u8)
    } else {
        None
    };

    let scratch = match req("scratch")? {
        "none" => None,
        desc => {
            // Format: symbol=<name>,len=<n>,align=<none|page>
            let mut symbol = None;
            let mut len = None;
            let mut align_page = false;
            for part in desc.split(',').map(|s| s.trim()) {
                if let Some(v) = part.strip_prefix("symbol=") {
                    symbol = Some(v.to_string());
                } else if let Some(v) = part.strip_prefix("len=") {
                    len = parse_hex_or_dec(v);
                } else if let Some(v) = part.strip_prefix("align=") {
                    align_page = v == "page";
                }
            }
            let symbol = symbol.ok_or_else(|| format!("{file}: ;@scratch missing symbol="))?;
            let len = len.ok_or_else(|| format!("{file}: ;@scratch missing len="))? as u16;
            let default = config_default(source, &symbol)
                .ok_or_else(|| format!("{file}: scratch symbol {symbol} not in config-defaults"))?
                as u16;
            Some(ScratchSpec { symbol, len, page_aligned: align_page, default })
        }
    };

    let illegal = req("illegal")? == "yes";
    let smc = req("smc")? == "yes";
    let code_bytes: u16 = req("code-bytes")?
        .parse()
        .map_err(|_| format!("{file}: bad ;@code-bytes:"))?;

    if !source.contains(CONFIG_BLOCK_BEGIN) || !source.contains(CONFIG_BLOCK_END) {
        return Err(format!("{file}: missing config-defaults block markers"));
    }

    Ok(RoutineSpec {
        format,
        direction,
        variant,
        file,
        source,
        entry,
        encoder,
        vfy_key,
        payload,
        eof,
        needs,
        zp_len,
        zp_base_default,
        scratch,
        illegal,
        smc,
        code_bytes,
    })
}

/// All parsed routines. Panics with a descriptive message if any embedded
/// source has a malformed header (covered by the registry unit tests).
pub fn all_routines() -> &'static [RoutineSpec] {
    static REG: OnceLock<Vec<RoutineSpec>> = OnceLock::new();
    REG.get_or_init(|| {
        SOURCES
            .iter()
            .map(|&(file, source)| match parse_spec(file, source) {
                Ok(s) => s,
                Err(e) => panic!("registry: {e}"),
            })
            .collect()
    })
}

/// Look up a routine by (format, direction, variant).
pub fn find_routine(format: Format, direction: Direction, variant: Variant) -> Option<&'static RoutineSpec> {
    all_routines()
        .iter()
        .find(|r| r.format == format && r.direction == direction && r.variant == variant)
}

/// Pick a routine for `(format, direction)` honoring an illegal-opcode policy.
///
/// * `allow_illegal == true` → the `Standard` variant (the size/speed baseline,
///   which may use undocumented opcodes).
/// * `allow_illegal == false` → a routine with `illegal == false`: the
///   `Standard` variant when it is already legal (most formats), otherwise the
///   dedicated `Legal` variant, otherwise any legal variant. `None` if the
///   format/direction has no legal decoder at all.
///
/// The compressed stream is identical across variants, so switching only swaps
/// the embedded decoder - the packed payload does not change.
pub fn pick_routine(
    format: Format,
    direction: Direction,
    allow_illegal: bool,
) -> Option<&'static RoutineSpec> {
    if allow_illegal {
        return find_routine(format, direction, Variant::Standard);
    }
    if let Some(std) = find_routine(format, direction, Variant::Standard) {
        if !std.illegal {
            return Some(std);
        }
    }
    find_routine(format, direction, Variant::Legal).or_else(|| {
        all_routines()
            .iter()
            .find(|r| r.format == format && r.direction == direction && !r.illegal)
    })
}

/// The stack-page-resident extra-small variant, honoring the illegal-opcode
/// policy. `None` when the format/direction has no such variant (or it uses
/// illegal opcodes under a legal-only policy) - callers then stay on
/// [`pick_routine`]'s baseline. The compressed stream is identical, so a
/// caller may choose per build based purely on whether the smaller body fits
/// its placement.
pub fn pick_zp_stack_routine(
    format: Format,
    direction: Direction,
    allow_illegal: bool,
) -> Option<&'static RoutineSpec> {
    find_routine(format, direction, Variant::ZpStack).filter(|r| allow_illegal || !r.illegal)
}
