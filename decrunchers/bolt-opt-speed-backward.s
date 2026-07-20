; ===========================================================================
; BoltLZ 6510 decruncher - SPEED-optimized, backward / in-place, asm6502 syntax.
; lzan's own format. Decodes the IDENTICAL backward stream as bolt-backward.s
; (compress_bolt_backward): both pointers walk DOWN from the top, so packed and
; unpacked data may overlap. This variant trades size for speed exactly like the
; forward bolt-opt-speed.s: the copy loops move 2 bytes/iteration (parity split),
; with the PAIR count held in X so a full L=130 match copies cleanly - the
; single-byte DEY/BPL loop of the standard decoder cannot, since Y would start
; with bit 7 set. Descending copy (top index DOWN to 0) is the overlap/RLE-safe
; direction for a downward-writing decoder. No bit reader, no undocumented opcodes.
;
; Self-seeds the end pointers from comp_data/out_addr + comp_data_len/out_len.
; Entry = full_decomp; in-stream $00 token -> RTS. Selected by the priority-speed
; flag together with a backward layout.
; ===========================================================================
;@format: bolt
;@direction: backward
;@variant: opt-speed
;@entry: full_decomp
;@vfy-key: bolt-opt-speed-backward
;@encoder: lzan::bolt::compress_bolt_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 10
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 219

; ---- config-defaults ----
zp_base = $F6
; ---- end config-defaults ----

src  = zp_base+0  ; 2 bytes: compressed read pointer (decrementing from the top)
dst  = zp_base+2  ; 2 bytes: output write pointer (decrementing from the top)
mptr = zp_base+4  ; 2 bytes: match source base
neg  = zp_base+6  ; 2 bytes: the token's negated offset
cm1  = zp_base+8  ; 1 byte : run length minus 1 (top copy index / base offset)
cnt  = zp_base+9  ; 1 byte : run length (for the pair/odd split)

full_decomp:
        LDA #<(comp_data + comp_data_len - 1)   ; src = last packed byte
        STA src
        LDA #>(comp_data + comp_data_len - 1)
        STA src+1
        LDA #<(out_addr + out_len - 1)          ; dst = last output byte
        STA dst
        LDA #>(out_addr + out_len - 1)
        STA dst+1

bx_next:
        LDY #0
        LDA (src),Y             ; token
        BEQ bx_done             ; $00 -> end of stream
        TAX                     ; X = token (save before the decrement clobbers A)
        LDA src                 ; src -= 1 (consume token)
        BNE bx_t1
        DEC src+1
bx_t1:
        DEC src
        TXA                     ; A = token again, re-sets N (bit7)
        BMI bx_match            ; $80..$FF -> match

; --- literal run: X = N (1..127) ---
        STX cnt                 ; cnt = N
        DEX
        STX cm1                 ; cm1 = N-1
        LDA src                 ; base_src = src - (N-1)
        SEC
        SBC cm1
        STA src
        BCS bx_l1
        DEC src+1
bx_l1:
        LDA dst                 ; base_dst = dst - (N-1)
        SEC
        SBC cm1
        STA dst
        BCS bx_l2
        DEC dst+1
bx_l2:
        LDY cm1                 ; Y = N-1  (top index of the run)
        LDA cnt
        LSR                     ; A = N/2 pairs, C = N&1 (odd)
        BEQ bx_lit1             ; N == 1 -> a single byte
        TAX                     ; X = pairs (>= 1)
        BCC bx_lit_e            ; even: no leading (top) byte
        LDA (src),Y             ; odd top byte
        STA (dst),Y
        DEY
bx_lit_e:
        LDA (src),Y             ; pair loop: two bytes/iteration, descending
        STA (dst),Y
        DEY
        LDA (src),Y
        STA (dst),Y
        DEY
        DEX
        BNE bx_lit_e
        JMP bx_ladv
bx_lit1:
        LDA (src),Y             ; Y = 0
        STA (dst),Y
bx_ladv:
        LDA src                 ; src -= 1 (below the run)
        BNE bx_l3
        DEC src+1
bx_l3:
        DEC src
        LDA dst                 ; dst -= 1 (below the run)
        BNE bx_l4
        DEC dst+1
bx_l4:
        DEC dst
        JMP bx_next

; --- match: X = token ($80..$FF) ---
bx_match:
        TXA
        AND #$7F
        CLC
        ADC #3                  ; A = L (3..130)
        STA cnt                 ; cnt = L
        SEC
        SBC #1
        STA cm1                 ; cm1 = L-1
        LDY #0
        LDA (src),Y             ; NEG_lo
        STA neg
        LDA src                 ; src -= 1
        BNE bx_m1
        DEC src+1
bx_m1:
        DEC src
        LDA (src),Y             ; NEG_hi
        STA neg+1
        LDA src                 ; src -= 1
        BNE bx_m2
        DEC src+1
bx_m2:
        DEC src
        LDA dst                 ; base_dst = dst - (L-1)
        SEC
        SBC cm1
        STA dst
        BCS bx_m3
        DEC dst+1
bx_m3:
        LDA dst                 ; mptr = base_dst - NEG (== base_dst + d)
        SEC
        SBC neg
        STA mptr
        LDA dst+1
        SBC neg+1
        STA mptr+1
        LDY cm1                 ; Y = L-1  (top index; L>=3 so pairs (L/2) >= 1)
        LDA cnt
        LSR                     ; A = L/2 pairs, C = L&1 (odd)
        TAX                     ; X = pairs (>= 1)
        BCC bx_mat_e            ; even: no top byte
        LDA (mptr),Y            ; odd top byte (DESCENDING, overlap/RLE safe)
        STA (dst),Y
        DEY
bx_mat_e:
        LDA (mptr),Y            ; pair loop: two bytes/iteration, descending
        STA (dst),Y
        DEY
        LDA (mptr),Y
        STA (dst),Y
        DEY
        DEX
        BNE bx_mat_e
        LDA dst                 ; dst -= 1 (below the run)
        BNE bx_m4
        DEC dst+1
bx_m4:
        DEC dst
        JMP bx_next

bx_done:
        RTS
