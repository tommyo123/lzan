; ===========================================================================
; BoltLZ 6510 decruncher - purely byte-oriented, forward, in asm6502 syntax.
; This is lzan's own format; there is no external upstream.
;
; A bolt is both fast (lightning) and small (a tiny fastener): the whole point
; of the format. There is NO bit reader anywhere - every field is whole bytes
; and dispatch is a single sign-bit test (BMI). No undocumented opcodes.
;
; Stream grammar (raw block, forward), one token byte T per command:
;   T == $00          -> end of stream (RTS).
;   T == $01..$7F     -> literal run: N = T (1..127) raw bytes follow inline.
;   T == $80..$FF     -> match: length L = (T & $7F) + 3 (3..130), then a 2-byte
;                        little-endian NEGATED offset NEG = (65536 - d) & $FFFF;
;                        the match source is mptr = dst + NEG == dst - d, a plain
;                        16-bit add (no sign fixups).
; $00 is unambiguous as EOF (literals are >=$01, matches >=$80, and literal /
; offset bytes are consumed by count). The match copy is ASCENDING so a
; self-overlap (d < L, i.e. RLE) reproduces correctly.
;
; Pointers live in zero page: src, dst, mptr. This is a CALLER-SEEDED decoder
; (`;@seed: caller`): the caller - the lzan-c64 framework's shared seed, or the
; test harness - seeds src (= comp_data) at zp_base+0 and dst (= out_addr) at
; zp_base+2 before entry, so the body carries no seed preamble of its own (which
; keeps it small). Seeding is one-time and off the hot path, so decode speed is
; unchanged. Entry = full_decomp, ends in RTS. ~18 cycles per copied byte.
; ===========================================================================
;@format: bolt
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: bolt
;@encoder: lzan::bolt::compress_bolt
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 6
;@scratch: none
;@illegal: no
;@smc: no
;@seed: caller
;@code-bytes: 97

; ---- config-defaults ----
zp_base = $FA
; ---- end config-defaults ----

src  = zp_base+0  ; 2 bytes: compressed source pointer
dst  = zp_base+2  ; 2 bytes: output pointer
mptr = zp_base+4  ; 2 bytes: match source pointer

; Entry: the caller has already seeded src (zp_base+0) = comp_data and
; dst (zp_base+2) = out_addr.
full_decomp:
bl_next:
        LDY #0                  ; Y=0 = base for token / offset / copy indexing
        LDA (src),Y             ; fetch token
        BEQ bl_done             ; $00 -> end of stream
        INC src                 ; consume the token byte (16-bit ptr bump)
        BNE bl_tok
        INC src+1
bl_tok:
        TAX                     ; X = token; re-sets N (bit7) and Z from the token
        BMI bl_match            ; $80..$FF -> match

; --- literal run: X = N in 1..127, Y = 0 ---
bl_lit:
        LDA (src),Y
        STA (dst),Y
        INY
        DEX
        BNE bl_lit
        TYA                     ; A = N (Y ended at N)
        JSR bl_addsrc           ; src += N
        TYA
        JSR bl_adddst           ; dst += N
        JMP bl_next

; --- match: X = token ($80..$FF), Y = 0 ---
bl_match:
        TXA
        AND #$7F                ; M = token & $7F  (0..127)
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
        LDA #2
        JSR bl_addsrc           ; src += 2 (past the offset)
        LDY #0
bl_mcopy:
        LDA (mptr),Y            ; ASCENDING copy: RLE / overlap safe
        STA (dst),Y
        INY
        DEX
        BNE bl_mcopy
        TYA                     ; A = L
        JSR bl_adddst           ; dst += L
        JMP bl_next

; --- 16-bit "pointer += A" helpers ---
bl_addsrc:
        CLC
        ADC src
        STA src
        BCC bl_as_rts
        INC src+1
bl_as_rts:
        RTS
bl_adddst:
        CLC
        ADC dst
        STA dst
        BCC bl_done             ; shares the end-of-stream RTS
        INC dst+1
bl_done:
        RTS
