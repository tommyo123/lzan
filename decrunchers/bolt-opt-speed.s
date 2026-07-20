; ===========================================================================
; BoltLZ 6510 decruncher - SPEED-optimized (opt-speed), forward, in asm6502.
; lzan's own format; decodes the IDENTICAL stream as bolt.s (the balanced 97 B
; standard decoder). This variant trades size for speed: the per-command pointer
; advances are inlined (no JSR/RTS, so the standard decoder's ~24 cycles/command
; of call overhead is gone). It is the faster decoder the priority-speed flag
; selects. Same sign-bit dispatch, no bit reader, no undocumented opcodes,
; ascending (mptr),Y match copy (RLE/overlap safe).
;
; CALLER-SEEDED (`;@seed: caller`): the caller seeds src (= comp_data) at
; zp_base+0 and dst (= out_addr) at zp_base+2 before entry. Entry = full_decomp.
; ===========================================================================
;@format: bolt
;@direction: forward
;@variant: opt-speed
;@entry: full_decomp
;@vfy-key: bolt-opt-speed
;@encoder: lzan::bolt::compress_bolt
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 6
;@scratch: none
;@illegal: no
;@smc: no
;@seed: caller
;@code-bytes: 147

; ---- config-defaults ----
zp_base = $FA
; ---- end config-defaults ----

src  = zp_base+0  ; 2 bytes: compressed source pointer (caller-seeded = comp_data)
dst  = zp_base+2  ; 2 bytes: output pointer (caller-seeded = out_addr)
mptr = zp_base+4  ; 2 bytes: match source pointer

full_decomp:
bs_next:
        LDY #0
        LDA (src),Y             ; token
        BEQ bs_done             ; $00 -> end of stream
        INC src                 ; consume the token byte
        BNE bs_tok
        INC src+1
bs_tok:
        TAX                     ; X = token; re-sets N (bit7) and Z
        BMI bs_match            ; $80..$FF -> match

; --- literal run: X = N (1..127), Y = 0; copied 2 bytes/iteration ---
        TXA
        LSR                     ; A = N/2 pairs, C = N&1 (odd)
        BEQ bs_lit1             ; N == 1 -> a single byte
        TAX                     ; X = pairs
        BCC bs_lit_e            ; even count: no leading byte
        LDA (src),Y             ; odd leading byte
        STA (dst),Y
        INY
bs_lit_e:
        LDA (src),Y             ; pair loop: two bytes per iteration
        STA (dst),Y
        INY
        LDA (src),Y
        STA (dst),Y
        INY
        DEX
        BNE bs_lit_e
        BEQ bs_ladv             ; always
bs_lit1:
        LDA (src),Y
        STA (dst),Y
        INY
bs_ladv:
        TYA                     ; A = N; src += N (inline)
        CLC
        ADC src
        STA src
        BCC bs_ladv2
        INC src+1
bs_ladv2:
        TYA                     ; dst += N (inline)
        CLC
        ADC dst
        STA dst
        BCC bs_next
        INC dst+1
        JMP bs_next

; --- match: X = token ($80..$FF), Y = 0 ---
bs_match:
        TXA
        AND #$7F                ; M = token & $7F
        CLC
        ADC #3                  ; L = M + 3  (3..130)
        TAX                     ; X = L
        LDA (src),Y             ; NEG_lo (Y=0)
        CLC
        ADC dst
        STA mptr                ; mptr_lo = dst_lo + NEG_lo
        INY                     ; Y=1
        LDA (src),Y             ; NEG_hi
        ADC dst+1               ; carry chained from the low add
        STA mptr+1              ; mptr = dst - d
        LDA src                 ; src += 2 (past the offset), inline
        CLC
        ADC #2
        STA src
        BCC bs_msrc
        INC src+1
bs_msrc:
        LDY #0                  ; L >= 3, so pairs (L/2) >= 1: no zero-count guard
        TXA
        LSR                     ; A = L/2 pairs, C = L&1 (odd)
        TAX                     ; X = pairs
        BCC bs_mat_e            ; even: no leading byte
        LDA (mptr),Y            ; odd leading byte (ASCENDING, overlap-safe)
        STA (dst),Y
        INY
bs_mat_e:
        LDA (mptr),Y            ; pair loop: two bytes per iteration
        STA (dst),Y
        INY
        LDA (mptr),Y
        STA (dst),Y
        INY
        DEX
        BNE bs_mat_e
        TYA                     ; A = L; dst += L (inline)
        CLC
        ADC dst
        STA dst
        BCC bs_next
        INC dst+1
        JMP bs_next

bs_done:
        RTS
