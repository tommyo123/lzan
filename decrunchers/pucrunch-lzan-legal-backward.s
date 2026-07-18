; ===========================================================================
; PuCrunch 6502 decruncher - BACKWARD / in-place, lzan container. CLEAN-ROOM:
; written from the pucrunch token grammar (no original pucrunch assembly
; consulted). Mirrors pucrunch-lzan.s with only the
; direction aspects flipped, the same recipe as the collection's other
; -backward decoders:
;
;   1. src reads DOWN  : the refill decrements the input operand pair.
;   2. dst writes DOWN : putch decrements its store operand pair.
;   3. match source    : the back-reference lies at HIGHER addresses, so
;                        src = OUT + dist (forward computes OUT - dist).
;
; The encoder (lzan::pucrunch::compress_pucrunch_6502_backward) compresses the
; reversed input and reverses the bitstream bytes, so this descending reader
; sees the exact forward byte sequence. The 19-byte parameter block sits at
; the TOP of the container (read first) and is copied VERBATIM into zero page
; by one ascending loop - the zp layout below matches the block byte for byte:
;
;   comp_data+len-1   startEsc          (len = comp_data_len)
;   comp_data+len-2   escBits
;   comp_data+len-3   8-escBits
;   comp_data+len-4   extraLZPosBits
;   comp_data+len-20+r  RLE rank r (1..15, ascending in memory)
;   comp_data+len-20  first bitstream byte, then descending
;
; LEGAL variant: the same size-optimised structure as pucrunch-lzan-backward.s
; with its two undocumented opcodes expanded to documented equivalents:
;   * SBX #$FF (LZ copy-count setup) -> TAX / INX - same size, same registers
;     and flags where read, +2 cycles per LZ token.
;   * DCP (the descending 16-bit pointer steps in putch and the LZ copy loop)
;     -> the classic test-then-decrement pair (LDA lo / BNE + / DEC hi /
;     +: DEC lo). Unlike DCP this proves no carry, so putch keeps its own
;     PLA/RTS tail and the literal path returns to main by JMP instead of a
;     flag branch (+3 bytes total vs the standard variant).
; For CPUs/emulators without NMOS illegal opcodes.
;
; ONE-SHOT: the input/output SMC operands assemble to their start values
; (comp_data+len-20 / out_addr+out_len-1), and the bit buffer byte assembles
; to its empty-sentinel value $80; none are re-seeded at entry.
; RTS at EOF; on exit the output pointer is at out_addr-1.
; ===========================================================================
;@format: pucrunch
;@direction: backward
;@variant: legal
;@entry: full_decomp
;@vfy-key: pucrunch-lzan-legal-backward
;@encoder: lzan::pucrunch::compress_pucrunch_6502_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 19
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 251

; ---- config-defaults ----
zp_base = $E0
; ---- end config-defaults ----

rtab   = zp_base+0   ; 15 bytes: RLE rank table (ranks 1..15), copied out of
                     ; the packed block at init - in-place layouts overwrite
                     ; the block while ranked lookups still need it
exb    = zp_base+15  ; extraLZPosBits
litb   = zp_base+16  ; literal rest width (8-escBits)
escb   = zp_base+17  ; escape width (escBits)
esc    = zp_base+18  ; current escape selector (right-aligned)

full_decomp:
        LDX #18              ; one loop moves the whole 19-byte parameter
tcp:
        LDA comp_data+comp_data_len-19,X
        STA rtab,X           ; block: ranks, widths and startEsc land on the
        DEX                  ; matching zp variables in a single pass
        BPL tcp
        ; fall through into the main token loop (bit buffer is seeded to $80
        ; at assembly time - see ONE-SHOT above)

; --- main dispatch -----------------------------------------------------------
main:
        JSR read_esc         ; A = selector
        CMP esc
        BNE lit              ; ordinary literal: finish the byte
        JSR getval           ; a = first control value
        CMP #1
        BNE lznorm           ; a >= 2: normal LZ / EOF
        JSR getbit
        BCC lz2              ; 0   : LZ length 2
        JSR getbit
        BCC newesc           ; 10  : escape change + literal
        ; 11  : RLE - fall through

; --- RLE (len 2..32256) ------------------------------------------------------
rle:
        LDY #1               ; high loop count for the short form
        JSR getval
        CMP #$80
        BCC rs               ; short: A = len-1 (1..127)
        ; long: A = 128 + (nlo >> 1); one more raw bit completes nlo
        ASL                  ; A = (nlo & $FE); the 128 falls off the top
        JSR getbit
        ADC #0               ; ... and the raw bit lands in bit 0
        PHA
        JSR getval           ; (n >> 8) + 1
        TAY
        PLA
rs:
        STA rx+1             ; nlo, parked in the loop-count operand (SMC)
        ; byte code: gamma < 16 = table rank, else hi nibble | 4 raw bits
        JSR getval
        CMP #16              ; C=1 for the unranked path, C=0 for ranked
        BCS unrk
        TAX
        LDA rtab-1,X         ; rank table (zp copy)
        BCC remit            ; always (C=0 from the BCS above)
unrk:
        LDX #4
        JSR gbits            ; A = (code << 4) | bits; the top bit of code
remit:                       ; falls off the 8-bit ROL, leaving (code-16)<<4
rx:
        LDX #0               ; (SMC: nlo)
        INX                  ; nlo+1 first pass, then 256 per extra Y pass
rloop:
        JSR putch            ; putch preserves A (the run byte)
        DEX
        BNE rloop
        DEY
        BNE rloop
mainc:
        BEQ main             ; always (also the LZ loop's relay to main)

; --- escape change + literal (old escape prefixes the byte) -------------------
newesc:
        LDY esc
        JSR read_esc
        STA esc
        TYA                  ; A = old escape = the literal's selector bits
; --- ordinary literal: A = selector, read the remaining 8-escBits ------------
lit:
        LDX litb
        JSR gbits
        JSR putch
        JMP main             ; legal putch proves no flag - JMP, not a branch

; --- LZ length 2 (distance 1..256) --------------------------------------------
; A = 1 here (the dispatched gamma value) = len-1 for the 2-byte form, so the
; normal-LZ tail computes the right count from the same stacked value.
lz2:
        PHA                  ; len-1 = 1
        LSR                  ; A = 0 (d.hi), C = 1
        PHA
        BCS lzlow            ; always

; --- normal LZ (len 3..256) / EOF ---------------------------------------------
lznorm:
        PHA                  ; a = len-1
        JSR getval           ; b = high position group + 1, or the sentinel
        CMP #$FF
        BEQ pr               ; EOF: drop len-1 and return (shared PLA/RTS)
        SBC #0               ; C=0 here (A < $FF), so this is A-1
        LDX exb
        JSR gbits            ; A = (b-1) << extra | middle bits = d.hi
        PHA
lzlow:
        LDA #1               ; guard bit: eight ROLs push it out, so C=1 after
        LDX #8
        JSR gbits            ; A = encoded low = ~(d.lo), C = 1
        ; BACKWARD: src = OUT + dist = OUT + d + 1 (the match lies ABOVE the
        ; write head). ~(d.lo) ^ $FF = d.lo; the guard-bit carry feeds the +1
        ; into the ADC and its carry chains into OUT.hi + d.hi.
        EOR #$FF
        ADC putch+1
        STA lzl+1
        PLA                  ; d.hi (PLA leaves C alone)
        ADC putch+2
        STA lzl+2
        PLA
        TAX                  ; legal expansion of SBX #$FF:
        INX                  ; X = A + 1 = the copy length; 256 wraps X to 0,
                             ; which the DEX loop turns into 256 iterations
lzloop:
lzl:
        LDA $AAAA            ; SMC: match source, stepping DOWN with the output
        JSR putch
        LDA lzl+1            ; classic borrow test (legal expansion of DCP)
        BNE lz1
        DEC lzl+2
lz1:
        DEC lzl+1
        DEX
        BNE lzloop
        BEQ mainc            ; always (relay: main is too far for one hop)

; --- output one byte, descending (A preserved) ---------------------------------
putch:
        STA out_addr+out_len-1 ; SMC operand = output position
        PHA
        LDA putch+1          ; classic borrow test (legal expansion of DCP)
        BNE pc1
        DEC putch+2
pc1:
        DEC putch+1
        PLA
        RTS

; --- bit primitives -------------------------------------------------------------
; getbit: next stream bit into C. The buffer byte holds remaining bits with a
; 1-sentinel at the bottom; shifting out the sentinel (result 0) triggers a
; refill, whose ROL pushes the new byte's top bit to C and re-plants the
; sentinel from the carry the exhausted buffer left behind.
getbit:
        ASL bitstr
        BEQ refill
        RTS
refill:
        PHA
inp:
        LDA comp_data+comp_data_len-20 ; SMC operand = next stream byte (down)
        ROL                  ; C is 1 here (the sentinel just shifted out)
        STA bitstr
        LDA inp+1
        BNE rf1
        DEC inp+2
rf1:
        DEC inp+1
pr:
        PLA                  ; shared tail: refill exit and EOF
        RTS

; getval: bounded gamma, result 1..255 in A (maxGamma = 7). Unary ones count
; the exponent in X, then the suffix bits rotate into A on top of the implicit
; leading 1 - the suffix read IS gbits. Every call site reaches here with
; X = $FF (each is preceded by a gbits exit, and getbit preserves X), so INX
; zeroes the unary counter.
getval:
        LDA #1
        INX
gv_u:
        JSR getbit
        BCC gbits            ; 0-terminated: X = suffix length
        INX
        CPX #7
        BNE gv_u             ; at maxGamma the terminator is implicit
        BEQ gbits            ; always (X = 7 suffix bits)

; read_esc: A = the next escBits stream bits (0 allowed); falls into gbits
read_esc:
        LDA #0
        LDX escb

; gbits: rotate X more stream bits into A from the right (X may be 0).
; Exits with X = $FF and N=1/Z=0 (from the final DEX).
gbits:
        DEX
        BMI gb_rts
gb_l:
        JSR getbit
        ROL
        DEX
        BPL gb_l
gb_rts:
        RTS

bitstr:
        .byte $80            ; sentinel bit buffer, assembled empty (ONE-SHOT)
