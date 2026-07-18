//! 6502 opcode tables and initialization

use std::collections::HashMap;

pub struct OpcodeTables {
    /// Base opcodes (implied/accumulator and the default immediate form).
    pub opcodes: HashMap<&'static str, u8>,
    /// Immediate-mode opcodes for mnemonics whose `#imm` encoding must NOT
    /// collide with an implied/accumulator opcode of the same mnemonic. The
    /// immediate handler consults this table first and falls back to
    /// `opcodes`. The only mnemonic that genuinely needs this split is the
    /// illegal multi-byte `NOP` (implied $EA *and* immediate $80), but the
    /// mechanism is general.
    pub immediate_opcodes: HashMap<&'static str, u8>,
    /// Extended opcodes by mnemonic -> addressing mode -> opcode
    pub extended_opcodes: HashMap<&'static str, HashMap<&'static str, u8>>,
}

impl OpcodeTables {
    pub fn new() -> Self {
        let mut tables = Self {
            opcodes: HashMap::new(),
            immediate_opcodes: HashMap::new(),
            extended_opcodes: HashMap::new(),
        };
        tables.init_opcodes();
        tables.init_address_modes();
        tables
    }

    fn init_opcodes(&mut self) {
        self.opcodes = HashMap::from([
            ("LDA", 0xA9), ("LDX", 0xA2), ("LDY", 0xA0),
            ("STA", 0x8D), ("STX", 0x8E), ("STY", 0x8C),
            ("ADC", 0x69), ("SBC", 0xE9),
            ("AND", 0x29), ("ORA", 0x09), ("EOR", 0x49),
            ("CMP", 0xC9), ("CPX", 0xE0), ("CPY", 0xC0),
            ("INC", 0xE6), ("INX", 0xE8), ("INY", 0xC8),
            ("DEC", 0xC6), ("DEX", 0xCA), ("DEY", 0x88),
            ("ASL", 0x0A), ("LSR", 0x4A), ("ROL", 0x2A), ("ROR", 0x6A),
            ("JMP", 0x4C), ("JSR", 0x20), ("RTS", 0x60), ("RTI", 0x40),
            ("BCC", 0x90), ("BCS", 0xB0), ("BEQ", 0xF0), ("BMI", 0x30),
            ("BNE", 0xD0), ("BPL", 0x10), ("BVC", 0x50), ("BVS", 0x70),
            ("CLC", 0x18), ("SEC", 0x38), ("CLD", 0xD8), ("SED", 0xF8),
            ("CLI", 0x58), ("SEI", 0x78), ("CLV", 0xB8),
            ("TAX", 0xAA), ("TXA", 0x8A), ("TAY", 0xA8), ("TYA", 0x98),
            ("TSX", 0xBA), ("TXS", 0x9A),
            ("PHA", 0x48), ("PLA", 0x68), ("PHP", 0x08), ("PLP", 0x28),
            ("BIT", 0x24), ("NOP", 0xEA), ("BRK", 0x00),

            // ===== Illegal/undocumented immediate-only opcodes =====
            // (canonical mnemonic -> #imm opcode; aliases added below)
            ("ANC", 0x0B), // AND #imm then copy bit7 -> C
            ("ALR", 0x4B), // AND #imm then LSR A      (alias: ASR)
            ("ARR", 0x6B), // AND #imm then ROR A
            ("AXS", 0xCB), // (A & X) - #imm -> X      (alias: SBX)
            ("LAX", 0xAB), // LAX #imm (a.k.a. ATX/LXA/OAL) -- UNSTABLE: result
                           //          depends on a CPU-internal magic constant
                           //          (commonly $EE/$EF/$FF); $00 imm is reliable.

            // Immediate-only aliases mapping to the same opcodes.
            ("ASR", 0x4B), // == ALR
            ("SBX", 0xCB), // == AXS

            // Implied illegal NOPs (single-byte). Canonical NOP ($EA) is above;
            // these are the spare implied no-ops the NMOS core also treats as NOP.
            // They are NOT given distinct mnemonics (all disassemble as NOP), so
            // there is no separate entry here -- the parser only ever emits the
            // mnemonic "NOP" with no operand, which already resolves to $EA.
        ]);

        // Immediate-mode encodings that must coexist with an implied opcode of
        // the same mnemonic. Currently only NOP: implied $EA vs immediate $80.
        // (0x80/0x82/0x89/0xC2/0xE2 are all 2-byte NOP #imm; $80 is canonical.)
        self.immediate_opcodes = HashMap::from([
            ("NOP", 0x80),
        ]);
    }

    fn init_address_modes(&mut self) {
        use std::iter::FromIterator;

        let lda: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xA5), ("zeropage,X", 0xB5),
            ("absolute", 0xAD), ("absolute,X", 0xBD), ("absolute,Y", 0xB9),
            ("indirect,X", 0xA1), ("indirect,Y", 0xB1),
        ]);
        let ldx: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xA6), ("zeropage,Y", 0xB6),
            ("absolute", 0xAE), ("absolute,Y", 0xBE),
        ]);
        let ldy: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xA4), ("zeropage,X", 0xB4),
            ("absolute", 0xAC), ("absolute,X", 0xBC),
        ]);
        let sta: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x85), ("zeropage,X", 0x95),
            ("absolute", 0x8D), ("absolute,X", 0x9D), ("absolute,Y", 0x99),
            ("indirect,X", 0x81), ("indirect,Y", 0x91),
        ]);
        let stx: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x86), ("zeropage,Y", 0x96), ("absolute", 0x8E),
        ]);
        let sty: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x84), ("zeropage,X", 0x94), ("absolute", 0x8C),
        ]);
        let adc: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x65), ("zeropage,X", 0x75),
            ("absolute", 0x6D), ("absolute,X", 0x7D), ("absolute,Y", 0x79),
            ("indirect,X", 0x61), ("indirect,Y", 0x71),
        ]);
        let sbc: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xE5), ("zeropage,X", 0xF5),
            ("absolute", 0xED), ("absolute,X", 0xFD), ("absolute,Y", 0xF9),
            ("indirect,X", 0xE1), ("indirect,Y", 0xF1),
        ]);
        let and_: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x25), ("zeropage,X", 0x35),
            ("absolute", 0x2D), ("absolute,X", 0x3D), ("absolute,Y", 0x39),
            ("indirect,X", 0x21), ("indirect,Y", 0x31),
        ]);
        let ora: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x05), ("zeropage,X", 0x15),
            ("absolute", 0x0D), ("absolute,X", 0x1D), ("absolute,Y", 0x19),
            ("indirect,X", 0x01), ("indirect,Y", 0x11),
        ]);
        let eor: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x45), ("zeropage,X", 0x55),
            ("absolute", 0x4D), ("absolute,X", 0x5D), ("absolute,Y", 0x59),
            ("indirect,X", 0x41), ("indirect,Y", 0x51),
        ]);
        let cmp: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xC5), ("zeropage,X", 0xD5),
            ("absolute", 0xCD), ("absolute,X", 0xDD), ("absolute,Y", 0xD9),
            ("indirect,X", 0xC1), ("indirect,Y", 0xD1),
        ]);
        let cpx: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xE4), ("absolute", 0xEC),
        ]);
        let cpy: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xC4), ("absolute", 0xCC),
        ]);
        let bit: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x24), ("absolute", 0x2C),
        ]);
        let asl: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x06), ("zeropage,X", 0x16), ("absolute", 0x0E), ("absolute,X", 0x1E),
        ]);
        let lsr: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x46), ("zeropage,X", 0x56), ("absolute", 0x4E), ("absolute,X", 0x5E),
        ]);
        let rol: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x26), ("zeropage,X", 0x36), ("absolute", 0x2E), ("absolute,X", 0x3E),
        ]);
        let ror: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x66), ("zeropage,X", 0x76), ("absolute", 0x6E), ("absolute,X", 0x7E),
        ]);
        let dec: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xC6), ("zeropage,X", 0xD6), ("absolute", 0xCE), ("absolute,X", 0xDE),
        ]);
        let inc: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xE6), ("zeropage,X", 0xF6), ("absolute", 0xEE), ("absolute,X", 0xFE),
        ]);
        let jsr: HashMap<&'static str, u8> = HashMap::from_iter([
            ("absolute", 0x20),
        ]);

        // ===== Illegal/undocumented opcodes (NMOS 6510 standard encodings) =====
        // Undocumented memory-mode opcodes.
        //
        // Deliberately NOT assembled (highly unstable — result depends on the
        // target address high byte and/or RDY/page-cross timing, and no
        // decruncher uses them): SHA/AHX ($9F/$93), SHX ($9E), SHY ($9C),
        // TAS/SHS ($9B), LAS/LAR ($BB), XAA/ANE ($8B). The KIL/JAM/HLT jam
        // opcodes are also omitted. LAX #imm ($AB, a.k.a. LXA/ATX) is included
        // (in `opcodes`) per request, flagged unstable there.

        // LAX = LDA+LDX (load into both A and X). #imm form ($AB) is in
        // `immediate`/`opcodes`; the memory forms live here.
        let lax: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xA7), ("zeropage,Y", 0xB7),
            ("absolute", 0xAF), ("absolute,Y", 0xBF),
            ("indirect,X", 0xA3), ("indirect,Y", 0xB3),
        ]);
        // SAX = store (A & X). No flags. No abs,X/abs,Y/(zp),Y forms exist.
        let sax: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x87), ("zeropage,Y", 0x97),
            ("absolute", 0x8F), ("indirect,X", 0x83),
        ]);
        // DCP (a.k.a. DCM) = DEC then CMP.
        let dcp: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xC7), ("zeropage,X", 0xD7),
            ("absolute", 0xCF), ("absolute,X", 0xDF), ("absolute,Y", 0xDB),
            ("indirect,X", 0xC3), ("indirect,Y", 0xD3),
        ]);
        // ISC (a.k.a. ISB / INS) = INC then SBC.
        let isc: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0xE7), ("zeropage,X", 0xF7),
            ("absolute", 0xEF), ("absolute,X", 0xFF), ("absolute,Y", 0xFB),
            ("indirect,X", 0xE3), ("indirect,Y", 0xF3),
        ]);
        // SLO (a.k.a. ASO) = ASL then ORA.
        let slo: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x07), ("zeropage,X", 0x17),
            ("absolute", 0x0F), ("absolute,X", 0x1F), ("absolute,Y", 0x1B),
            ("indirect,X", 0x03), ("indirect,Y", 0x13),
        ]);
        // RLA (a.k.a. RLN) = ROL then AND.
        let rla: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x27), ("zeropage,X", 0x37),
            ("absolute", 0x2F), ("absolute,X", 0x3F), ("absolute,Y", 0x3B),
            ("indirect,X", 0x23), ("indirect,Y", 0x33),
        ]);
        // SRE (a.k.a. LSE) = LSR then EOR.
        let sre: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x47), ("zeropage,X", 0x57),
            ("absolute", 0x4F), ("absolute,X", 0x5F), ("absolute,Y", 0x5B),
            ("indirect,X", 0x43), ("indirect,Y", 0x53),
        ]);
        // RRA (a.k.a. RRD) = ROR then ADC.
        let rra: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x67), ("zeropage,X", 0x77),
            ("absolute", 0x6F), ("absolute,X", 0x7F), ("absolute,Y", 0x7B),
            ("indirect,X", 0x63), ("indirect,Y", 0x73),
        ]);
        // Undocumented multi-byte NOPs (DOP/TOP family). Read operand, discard.
        // The implied + #imm forms are in `opcodes`/`immediate_opcodes`.
        let nop: HashMap<&'static str, u8> = HashMap::from_iter([
            ("zeropage", 0x04), ("zeropage,X", 0x14),
            ("absolute", 0x0C), ("absolute,X", 0x1C),
        ]);

        // Aliases that map to the same opcode tables (cloned).
        let dcm = dcp.clone(); // DCP alias
        let isb = isc.clone(); // ISC alias
        let ins = isc.clone(); // ISC alias

        self.extended_opcodes = HashMap::from([
            ("LDA", lda), ("LDX", ldx), ("LDY", ldy),
            ("STA", sta), ("STX", stx), ("STY", sty),
            ("ADC", adc), ("SBC", sbc),
            ("AND", and_), ("ORA", ora), ("EOR", eor),
            ("CMP", cmp), ("CPX", cpx), ("CPY", cpy),
            ("BIT", bit),
            ("ASL", asl), ("LSR", lsr), ("ROL", rol), ("ROR", ror),
            ("DEC", dec), ("INC", inc),
            ("JSR", jsr),

            // Illegal/undocumented memory-mode opcodes.
            ("LAX", lax), ("SAX", sax),
            ("DCP", dcp), ("DCM", dcm),
            ("ISC", isc), ("ISB", isb), ("INS", ins),
            ("SLO", slo), ("RLA", rla), ("SRE", sre), ("RRA", rra),
            ("NOP", nop),
        ]);
    }
}

impl Default for OpcodeTables {
    fn default() -> Self {
        Self::new()
    }
}
