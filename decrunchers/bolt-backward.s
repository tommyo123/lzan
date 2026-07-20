; ===========================================================================
; BoltLZ 6510 decruncher - backward / in-place, standard, in asm6502 syntax.
; lzan's own format. Decodes a backward BoltLZ block == reverse(compress_bolt(
; reverse(input))) == lzan::bolt::compress_bolt_backward. In-place capable: both
; pointers walk DOWN from the top, so packed and unpacked data may overlap.
;
; Mirror of the forward decoder: src/dst are seeded at the LAST byte and
; DECREMENT; the token/offset fields are read the same order (the stream is
; byte-reversed, so reading it top-down yields the forward stream); a match's
; source lies at dst + d (a HIGHER address, already written), and the copy runs
; DESCENDING (Y from L-1 down to 0), which is the overlap/RLE-safe direction for
; a downward-writing decoder. The negated offset makes mptr = base_dst - NEG
; (== base_dst + d). No bit reader, no undocumented opcodes.
;
; Self-seeds the end pointers from comp_data/out_addr + comp_data_len/out_len.
; Entry = full_decomp; in-stream $00 token -> RTS.
; ===========================================================================
;@format: bolt
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: bolt-backward
;@encoder: lzan::bolt::compress_bolt_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 9
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 174

; ---- config-defaults ----
zp_base = $F7
; ---- end config-defaults ----

src  = zp_base+0  ; 2 bytes: compressed read pointer (decrementing from the top)
dst  = zp_base+2  ; 2 bytes: output write pointer (decrementing from the top)
mptr = zp_base+4  ; 2 bytes: match source base
neg  = zp_base+6  ; 2 bytes: the token's negated offset
cm1  = zp_base+8  ; 1 byte : run length minus 1 (loop index / base offset)

full_decomp:
        LDA #<(comp_data + comp_data_len - 1)   ; src = last packed byte
        STA src
        LDA #>(comp_data + comp_data_len - 1)
        STA src+1
        LDA #<(out_addr + out_len - 1)          ; dst = last output byte
        STA dst
        LDA #>(out_addr + out_len - 1)
        STA dst+1

bb_next:
        LDY #0
        LDA (src),Y             ; token
        BEQ bb_done             ; $00 -> end of stream
        TAX                     ; X = token (save it before the decrement clobbers A)
        ; consume the token byte: src -= 1
        LDA src
        BNE bb_t1
        DEC src+1
bb_t1:
        DEC src
        TXA                     ; A = token again, re-sets N (bit7)
        BMI bb_match            ; $80..$FF -> match

; --- literal run: X = N (1..127) ---
        DEX                     ; X = N-1
        STX cm1
        ; base_src = src - (N-1)
        LDA src
        SEC
        SBC cm1
        STA src
        BCS bb_l1
        DEC src+1
bb_l1:
        ; base_dst = dst - (N-1)
        LDA dst
        SEC
        SBC cm1
        STA dst
        BCS bb_l2
        DEC dst+1
bb_l2:
        LDY cm1                 ; Y = N-1
bb_lcopy:
        LDA (src),Y             ; descending copy from the packed literals
        STA (dst),Y
        DEY
        BPL bb_lcopy
        ; advance below the run: src -= 1, dst -= 1
        LDA src
        BNE bb_l3
        DEC src+1
bb_l3:
        DEC src
        LDA dst
        BNE bb_l4
        DEC dst+1
bb_l4:
        DEC dst
        JMP bb_next

; --- match: X = token ($80..$FF) ---
bb_match:
        TXA
        AND #$7F
        CLC
        ADC #3                  ; A = L (3..130)
        SEC
        SBC #1
        STA cm1                 ; cm1 = L-1
        ; read NEG_lo (src), src -= 1
        LDY #0
        LDA (src),Y
        STA neg
        LDA src
        BNE bb_m1
        DEC src+1
bb_m1:
        DEC src
        ; read NEG_hi (src), src -= 1
        LDA (src),Y
        STA neg+1
        LDA src
        BNE bb_m2
        DEC src+1
bb_m2:
        DEC src
        ; base_dst = dst - (L-1)
        LDA dst
        SEC
        SBC cm1
        STA dst
        BCS bb_m3
        DEC dst+1
bb_m3:
        ; mptr = base_dst - NEG   (== base_dst + d, the match source base)
        LDA dst
        SEC
        SBC neg
        STA mptr
        LDA dst+1
        SBC neg+1
        STA mptr+1
        LDY cm1                 ; Y = L-1  (up to 129 - can exceed 127)
bb_mcopy:
        LDA (mptr),Y            ; DESCENDING copy: RLE / overlap safe backward
        STA (dst),Y
        DEY
        CPY #$FF                ; stop after Y wraps past 0 (BPL would fail for L>128)
        BNE bb_mcopy
        ; advance below the run: dst -= 1
        LDA dst
        BNE bb_m4
        DEC dst+1
bb_m4:
        DEC dst
        JMP bb_next

bb_done:
        RTS
