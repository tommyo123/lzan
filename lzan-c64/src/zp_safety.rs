//! C64 zero-page ownership, used to check a decruncher's zero-page span
//! against state that BASIC or the KERNAL must keep when control returns to
//! them. A location matters not by how high it sits but by whether anything
//! rebuilds it after the decrunch (see [`ZpClass`]).

/// How a zero-page location behaves after a decruncher has overwritten it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZpClass {
    /// Unused, or re-derived before its next read.
    Safe,
    /// Rebuilt in normal operation, but not immediately: the damage is
    /// observable until whatever owns the location next recomputes it.
    Deferred,
    /// Not rebuilt without a reset.
    Persistent,
}

/// One contiguous run of zero page with a single owner.
#[derive(Clone, Copy, Debug)]
pub struct ZpRegion {
    pub lo: u8,
    pub hi: u8,
    pub class: ZpClass,
    pub name: &'static str,
}

use ZpClass::{Deferred, Persistent, Safe};

/// Page zero from `$02` up. `$00/$01` are the 6510 port and are rejected by a
/// dedicated check, so they are not described here.
static REGIONS: &[ZpRegion] = &[
    ZpRegion { lo: 0x02, hi: 0x02, class: Safe, name: "unused by BASIC and the KERNAL" },
    ZpRegion { lo: 0x03, hi: 0x06, class: Deferred, name: "ADRAY1/ADRAY2 conversion vectors, set at cold start and read only by machine code" },
    ZpRegion { lo: 0x07, hi: 0x2A, class: Safe, name: "BASIC per-statement temporaries and work pointers" },
    ZpRegion { lo: 0x2B, hi: 0x2C, class: Persistent, name: "TXTTAB, start of BASIC text" },
    ZpRegion { lo: 0x2D, hi: 0x2E, class: Persistent, name: "VARTAB, end of program / start of variables" },
    ZpRegion { lo: 0x2F, hi: 0x36, class: Persistent, name: "ARYTAB/STREND/FRETOP/FRESPC, rebuilt only by CLR" },
    ZpRegion { lo: 0x37, hi: 0x38, class: Persistent, name: "MEMSIZ, top of BASIC RAM" },
    ZpRegion { lo: 0x39, hi: 0x44, class: Deferred, name: "CURLIN/OLDLIN/OLDTXT/DATLIN/DATPTR, the CONT and RESTORE state" },
    ZpRegion { lo: 0x45, hi: 0x53, class: Safe, name: "BASIC pointer temporaries, re-derived per statement" },
    ZpRegion { lo: 0x54, hi: 0x54, class: Persistent, name: "the JMP opcode of JMPER, stored once at BASIC cold start" },
    ZpRegion { lo: 0x55, hi: 0x72, class: Safe, name: "JMPER's operand and the floating-point accumulators, rewritten per use" },
    ZpRegion { lo: 0x73, hi: 0x8A, class: Persistent, name: "CHRGET/CHRGOT, the RAM copy of BASIC's token fetch routine" },
    ZpRegion { lo: 0x8B, hi: 0x8F, class: Deferred, name: "RNDX, the RND seed" },
    ZpRegion { lo: 0x90, hi: 0x9F, class: Deferred, name: "KERNAL status and serial state" },
    ZpRegion { lo: 0xA0, hi: 0xA2, class: Deferred, name: "TIME, the jiffy clock" },
    ZpRegion { lo: 0xA3, hi: 0xC4, class: Deferred, name: "KERNAL tape/serial and file-name state" },
    ZpRegion { lo: 0xC5, hi: 0xD8, class: Deferred, name: "screen editor and keyboard-queue state" },
    ZpRegion { lo: 0xD9, hi: 0xF2, class: Deferred, name: "LDTB1, the screen line-link table" },
    ZpRegion { lo: 0xF3, hi: 0xF6, class: Deferred, name: "USER and KEYTAB, re-derived by the editor and the keyboard scan" },
    ZpRegion { lo: 0xF7, hi: 0xFA, class: Safe, name: "RS-232 buffer pointers, unused unless RS-232 is opened" },
    ZpRegion { lo: 0xFB, hi: 0xFE, class: Safe, name: "the four unused bytes" },
    ZpRegion { lo: 0xFF, hi: 0xFF, class: Safe, name: "head of BASIC's FP-to-string work area, rewritten before it is read" },
];

/// The regions a `len`-byte span at `base` intersects, excluding `Safe` ones
/// and anything below `$02`. `Persistent` regions come first.
pub fn regions_hit(base: u8, len: u8) -> Vec<&'static ZpRegion> {
    if len == 0 {
        return Vec::new();
    }
    let lo = base as u16;
    let hi = lo + len as u16 - 1;
    let mut hits: Vec<&'static ZpRegion> = REGIONS
        .iter()
        .filter(|r| r.class != Safe && lo <= r.hi as u16 && r.lo as u16 <= hi)
        .collect();
    hits.sort_by_key(|r| match r.class {
        Persistent => 0,
        Deferred => 1,
        Safe => 2,
    });
    hits
}

/// The lowest base at which `len` bytes fit entirely in `Safe` regions, at or
/// above `from`. Used to name a concrete alternative in diagnostics.
pub fn first_safe_base(len: u8, from: u8) -> Option<u8> {
    if len == 0 {
        return Some(from);
    }
    (from as u16..=0x100u16.saturating_sub(len as u16))
        .map(|b| b as u8)
        .find(|&b| regions_hit(b, len).is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn regions_tile_page_zero_from_02() {
        let mut next = 0x02u16;
        for r in REGIONS {
            assert_eq!(r.lo as u16, next, "gap or overlap before {}", r.name);
            assert!(r.hi >= r.lo);
            next = r.hi as u16 + 1;
        }
        assert_eq!(next, 0x100, "regions must reach $FF");
    }

    #[test]
    fn chrget_span_is_persistent() {
        // $80-$89 covers ten bytes of CHRGET.
        let hits = regions_hit(0x80, 10);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].class, Persistent);
        assert!(hits[0].name.contains("CHRGET"));
    }

    #[test]
    fn upkr_span_is_deferred_only() {
        // $F1-$FF: line-link tail, USER/KEYTAB, then unused bytes.
        let hits = regions_hit(0xF1, 15);
        assert!(!hits.is_empty());
        assert!(hits.iter().all(|r| r.class == Deferred));
    }

    #[test]
    fn safe_windows_report_clean() {
        for (base, len) in [(0xFBu8, 4u8), (0xF7, 8), (0x45, 10), (0x07, 36), (0x02, 1)] {
            assert!(regions_hit(base, len).is_empty(), "expected clean: {base:02X}+{len}");
        }
    }

    #[test]
    fn jmper_opcode_is_not_swallowed_by_the_temporaries() {
        // $54 holds the JMP opcode of BASIC's function-dispatch instruction and
        // is written once at cold start, so a span crossing it is not safe even
        // though everything around it is.
        let hits = regions_hit(0x4C, 19);
        assert_eq!(hits.len(), 1, "expected the JMPER byte to be reported");
        assert_eq!(hits[0].class, Persistent);
        assert!(regions_hit(0x45, 15).is_empty(), "$45-$53 stays clean");
        assert!(regions_hit(0x55, 30).is_empty(), "$55-$72 stays clean");
    }

    #[test]
    fn first_safe_base_finds_windows() {
        assert_eq!(first_safe_base(4, 0x02), Some(0x07));
        assert_eq!(first_safe_base(10, 0x40), Some(0x45));
        // $07-$2A (36 bytes) is the widest run with no owner that keeps state.
        assert_eq!(first_safe_base(36, 0x02), Some(0x07));
        assert_eq!(first_safe_base(37, 0x02), None);
    }
}
