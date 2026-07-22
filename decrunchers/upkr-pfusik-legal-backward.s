; ===========================================================================
; Legal-only variant of upkr-pfusik-backward.s (no undocumented opcodes): the
; one `LAX (u_probs),Y` illegal load (in the hot u_getBit prob decoder) is
; expanded to `LDA (u_probs),Y` / `TAX` (identical A, X, Z and N). Decodes the
; same stream (lzan::upkr::compress_upkr_6502_backward). Use when the decruncher
; must avoid NMOS illegal opcodes.
; Upstream: unupkr.asx (c) 2024 Piotr Fusik, zlib.
;
; BACKWARD / in-place variant. Mirrors the forward decoder with only the
; direction aspects flipped:
;
;   1. src reads DOWN  : u_fetchBit PRE-decrements u_src, then loads; the seed
;                        is therefore comp_data+comp_data_len (one PAST the
;                        last byte). LDA/DEC preserve the guard carry, so no
;                        byte needs to be parked across the pointer step.
;   2. dst writes DOWN : u_store walks u_dest downward; the page borrow also
;                        decrements u_prev+1 so the (u_prev),Y = dest+offset
;                        invariant survives page crossings (literals included -
;                        u_sameOffset reuses u_prev across matches).
;   3. match source    : the back-reference is at a HIGHER address, so
;                        u_prev = (E-1) and effective src = u_prev + dest_lo
;                        = dest + offset (forward computes dest - offset).
;
; NOTE the explicit SEC before falling into u_sameOffset: u_getLen's ROR chain
; needs C=1 as its completion sentinel. The forward decoder gets that carry
; structurally from its `ADC u_dest+1`; the backward offset math adds small
; positive values, so the carry must be set explicitly.
;
; E (encoded offset+1) is always >= 1 for encoder-produced streams (matches
; encode offset+1 >= 2, EOF encodes 1), so the E==0 case needs no check:
; the SBC borrow cannot happen and C stays set into the u_prev math.
;
; full_decomp seeds u_bitBuf/u_src/u_dest/u_state/u_probs in one loop from
; u_seedtab (the zp layout puts those nine bytes contiguously at zp_base).
; Termination is upkr's in-stream EOF offset marker, as in the forward decoder.
; ===========================================================================
;@format: upkr
;@direction: backward
;@variant: legal
;@entry: full_decomp
;@vfy-key: upkr-pfusik-legal-backward
;@encoder: lzan::upkr::compress_upkr_6502_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 15
;@scratch: symbol=probs_ram,len=319,align=page
;@illegal: no
;@smc: no
;@code-bytes: 236

; ---- config-defaults ----
zp_base = $F1
probs_ram = $C000
; ---- end config-defaults ----

; ---- zero page layout (15 bytes at zp_base) ----
; The first 9 bytes are seeded from u_seedtab (order matters! u_bitBuf's $80
; is loaded LAST so the probs fill below can reuse it; u_probs is seeded whole
; and kept as an invariant: the match path INCs the high byte and DECs it back
; after the length decode, so u_loop needs no reload).
u_bitBuf    = zp_base+0
u_src       = zp_base+1       ; word (reads DOWN, pre-decrement)
u_dest      = zp_base+3       ; word (writes DOWN)
u_state     = zp_base+5       ; word
u_probs     = zp_base+7       ; word (ptr into probs buffer)
u_prev      = zp_base+9       ; word (match-source base: dest + offset - dest_lo)
u_len       = zp_base+11      ; word
u_wasLiteral = zp_base+13
u_prob      = zp_base+14      ; (unupkr_mul==0 only)

; probs_ram (config-defaults above): page-aligned, >=319 bytes of scratch RAM.
; MUST stay page-aligned: u_seedtab seeds the pointer low byte (<probs_ram==0).

OFFSET_BITS = 16
LENGTH_BITS = 16
OFFSET_PROBS = OFFSET_BITS*2-1     ; 31
LENGTH_PROBS = LENGTH_BITS*2-1     ; 31
PROBS_LEN = 1+255+1+OFFSET_PROBS+LENGTH_PROBS   ; 319

; ---------------------------------------------------------------------------
full_decomp:
        LDX #8
u_seed:
        LDA u_seedtab,X
        STA u_bitBuf,X       ; bitBuf=$80 (last), src, dest, state=0, probs
        DEX
        BPL u_seed
        ; fall through into the probs fill with A = u_seedtab[0] = $80

; ---- init: fill probs[] with $80 ----
        LDX #160
u_init:
        STA probs_ram-1,X
        STA probs_ram+PROBS_LEN/2-1,X   ; PROBS_LEN/2 = 159
        DEX
        BNE u_init
        BEQ u_loop           ; X=0 set Z in the DEX loop

; ---------------------------------------------------------------------------
u_unpackCopy:
        INC u_probs+1
        LSR u_wasLiteral
        BCC u_getOffset
        DEY
        JSR u_getBit
        BCS u_sameOffset     ; --invert-new-offset-bit
u_getOffset:
        SEC
        JSR u_getLen         ; u_len = E (encoded offset+1); C=1, X=0 on exit
        ; BACKWARD: u_prev = E - 1 (the positive offset); effective source =
        ; u_prev + dest_lo = dest + offset. Forward computes 1 - E + dest_hi<<8
        ; (dest - offset); EOF is E == 1 in both (E == 0 never occurs, see top).
        ; u_len is zeroed on the fly (X=0) so the next u_getLen shift-in
        ; sentinel is not tripped early by leftover set bits.
        LDA u_len
        SBC #1               ; C=1: A = E_lo - 1
        STA u_prev
        STX u_len
        LDA u_len+1
        SBC #0               ; A = (E-1)_hi; C stays 1 (E >= 1)
        STX u_len+1
        TAY                  ; Y = (E-1)_hi (reloaded at u_sameOffset)
        ORA u_prev
        BEQ u_eof            ; E == 1 -> offset 0 -> EOF
        TYA
        CLC
        ADC u_dest+1         ; u_prev+1 = dest_hi + (E-1)_hi
        STA u_prev+1
        SEC                  ; u_getLen sentinel (see header note)
u_sameOffset:
        LDY #1+OFFSET_PROBS
        JSR u_getLen         ; C=1
        BEQ u_noinc
        INC u_len+1
u_noinc:
        DEC u_probs+1        ; restore the u_probs = probs_ram invariant
u_copy:
        LDY u_dest
        LDA (u_prev),Y
u_store:
        STA (u_dest,X)       ; X=0
        ; walk DOWN: borrow into the high bytes BEFORE decrementing dest_lo;
        ; u_prev+1 follows so (u_prev),Y keeps tracking dest + offset.
        LDA u_dest
        BNE u_samePage
        DEC u_dest+1
        DEC u_prev+1
u_samePage:
        DEC u_dest
        DEC u_len
        BNE u_copy
        DEC u_len+1
        BNE u_copy

u_loop:
        LDY #0
        JSR u_getBit         ; u_probs = probs_ram here (init seed + DEC above)
        BCS u_unpackCopy

        ; After LDY #0 + JSR u_getBit, getBit's `TYA/INY` left Y=1, so len=1.
        STY u_len            ; Y=1
        STY u_len+1
        STY u_wasLiteral
u_getLiteral:
        JSR u_getBit
        ROL
        TAY
        BCC u_getLiteral
        BCS u_store          ; jmp

; ---------------------------------------------------------------------------
u_fetchLen:
        JSR u_getBit
u_getLen:
        ROR u_len+1
        ROR u_len
        JSR u_getBit
        BCC u_fetchLen
        ; --invert-continue-value-bit
u_padLen:
        ROR u_len+1
        ROR u_len
        BCC u_padLen
u_eof:
        RTS

; ---------------------------------------------------------------------------
u_fetchBit:
        ; --big-endian-bitstream; src steps DOWN, pre-decrement (u_src points
        ; one past the next byte; LDA/DEC preserve the guard carry).
        ASL u_bitBuf
        BNE u_rolState
        LDA u_src
        BNE u_fb_dec
        DEC u_src+1
u_fb_dec:
        DEC u_src
        LDA (u_src,X)        ; X=0
        ROL                  ; C=1
        STA u_bitBuf
u_rolState:
        ROL u_state
        ROL u_state+1
u_getBit:
        ; -b
        LDA u_state+1
        BPL u_fetchBit

        LDA (u_probs),Y      ; legal expansion of LAX (u_probs),Y:
        TAX                  ; A = X = prob
        EOR #$ff
        DEX
        CPX u_state
        BCS u_skip_tax
        TAX
u_skip_tax:
        STX u_prob           ; (unupkr_mul==0)
        PHP

        ; --simplified-prob-update
        ROR
        LSR
        LSR
        LSR
        ADC #$f0
        CLC
        ADC (u_probs),Y
        STA (u_probs),Y

        ; slow multiplication (unupkr_mul==0)
        LDA #0
        LDX #8
u_mul:
        ASL u_state
        ROL
        ROL u_state+1
        BCC u_mulNot
        ADC u_prob           ; C=1
        BCC u_mul_skipinc
        INC u_state+1
u_mul_skipinc:
u_mulNot:
        DEX
        BNE u_mul
        PLP
        BCS u_bit1b
        SEC
        ADC u_prob
        BCS u_mul_skipdec
        DEC u_state+1
u_mul_skipdec:
        CLC
u_bit1b:
        STA u_state

        TYA
        INY
        RTS

; ---------------------------------------------------------------------------
; Seeds for zp_base+0..+8, stored by the u_seed loop (see zp layout above).
u_seedtab:
        .byte $80                            ; u_bitBuf (guard bit only; doubles as fill value)
        .byte <(comp_data + comp_data_len)   ; u_src (one past the last byte)
        .byte >(comp_data + comp_data_len)
        .byte <(out_addr + out_len - 1)      ; u_dest (the last output byte)
        .byte >(out_addr + out_len - 1)
        .byte 0, 0                           ; u_state
        .byte 0, >probs_ram                  ; u_probs (page-aligned)
