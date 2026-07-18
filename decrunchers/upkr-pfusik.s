; ===========================================================================
; upkr 6502 decruncher, SIZE variant, in asm6502 syntax.
; Upstream: unupkr.asx (c) 2024 Piotr Fusik, zlib.
;
; Build variant: unupkr_mul = 0 (portable software 8x8 multiply, no 65C02
; opcodes, no fast square table). ?65c02 = 0.
;
; Decodes the upkr stream produced by
;   upkr -9 --big-endian-bitstream --invert-new-offset-bit \
;        --invert-continue-value-bit --simplified-prob-update
; which is exactly what lzan::upkr::compress_upkr_6502 emits.
;
; The zp registers bitBuf/src/dest/probs/state sit contiguously at zp_base+0..8
; and are seeded from the 9-byte u_inittab with one indexed copy loop. The loop
; runs X=8..0 (DEX/BPL), so it exits with A = tab[0] = $80 (the probs fill byte)
; and X = $FF; the probs fill reuses that X=$FF with two stores at probs_ram-1,X
; and probs_ram+63,X covering the 319 prob bytes. u_probs+1 is seeded by the
; table and restored with a single DEC u_probs+1 after the match-length read.
; The 319-byte probs buffer is page-aligned scratch RAM (probs_ram). full_decomp
; seeds bitBuf/src/dest/probs/state from u_inittab, then falls into the probs
; fill.
; ===========================================================================

;@format: upkr
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: upkr-pfusik
;@encoder: lzan::upkr::compress_upkr_6502
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 15
;@scratch: symbol=probs_ram,len=319,align=page
;@illegal: yes
;@smc: no
;@code-bytes: 222

; ---- config-defaults ----
zp_base = $50
probs_ram = $0400
; ---- end config-defaults ----

; ---- zero page layout (15 bytes at zp_base) ----
; zp_base+0..8 are loaded from u_inittab by the init copy loop, in this order.
u_bitBuf    = zp_base+0
u_src       = zp_base+1       ; word
u_dest      = zp_base+3       ; word
u_probs     = zp_base+5       ; word (ptr into probs buffer)
u_state     = zp_base+7       ; word
u_prev      = zp_base+9       ; word
u_len       = zp_base+11      ; word
u_wasLiteral = zp_base+13
u_prob      = zp_base+14      ; (unupkr_mul==0 only)

; probs_ram (config-defaults above): >=319 bytes of scratch RAM.  The code
; itself seeds the full pointer from u_inittab, but @scratch keeps
; align=page (ABI; guarantees <probs_ram = 0 as the table byte).

OFFSET_BITS = 16
LENGTH_BITS = 16
OFFSET_PROBS = OFFSET_BITS*2-1     ; 31
LENGTH_PROBS = LENGTH_BITS*2-1     ; 31
PROBS_LEN = 1+255+1+OFFSET_PROBS+LENGTH_PROBS   ; 319

; ---------------------------------------------------------------------------
full_decomp:
        LDX #8
u_ptrs:
        LDA u_inittab,X
        STA u_bitBuf,X
        DEX
        BPL u_ptrs
        ; A = u_inittab+0 = $80 (probs fill byte / bit-buffer seed), X = $FF

; ---- init: fill probs[0..318] with $80 ----
; X counts $FF..$01: probs_ram-1,X covers +0..+254, probs_ram+63,X covers
; +64..+318 -- together exactly the PROBS_LEN bytes (overlap is harmless).
u_init:
        STA probs_ram-1,X
        STA probs_ram+63,X
        DEX
        BNE u_init
        ; X=0 (needed: (u_src,X)/(u_dest,X) use X=0), Z=1
        BEQ u_loop

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
        JSR u_getLen
        LDA #1
        SBC u_len            ; C=1
        STA u_prev
        TXA                  ; #0  (:!?65c02)
        SBC u_len+1
        BCS u_eof
        ADC u_dest+1         ; C=0
        STA u_prev+1
        STX u_len            ; X=0  (:!?65c02)
        STX u_len+1          ; (:!?65c02)
u_sameOffset:
        LDY #1+OFFSET_PROBS
        JSR u_getLen         ; C=1
        ; seq:inc ?len+1  -> execute INC when NOT zero (skip if Z set)
        BEQ u_copy_skipinc
        INC u_len+1
u_copy_skipinc:
        DEC u_probs+1        ; back to the literal-context page for u_loop
u_copy:
        LDY u_dest
        LDA (u_prev),Y
u_store:
        STA (u_dest,X)       ; X=0  (:!?65c02)
        INC u_dest
        BNE u_samePage
        INC u_dest+1
        INC u_prev+1
u_samePage:
        DEC u_len
        BNE u_copy
        DEC u_len+1
        BNE u_copy

u_loop:
        LDY #0
        JSR u_getBit         ; u_probs -> probs_ram (init table / DEC above)
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
        ; --big-endian-bitstream
        ASL u_bitBuf
        BNE u_rolState
        LDA (u_src,X)        ; X=0  (:!?65c02)
        ; inw ?src
        INC u_src
        BNE u_inw1
        INC u_src+1
u_inw1:
        ROL                  ; C=1
        STA u_bitBuf
u_rolState:
        ROL u_state
        ROL u_state+1
u_getBit:
        ; -b
        LDA u_state+1
        BPL u_fetchBit

        LAX (u_probs),Y      ; fused LDA (u_probs),Y + TAX (A=X=prob)
        EOR #$ff
        DEX
        CPX u_state
        ; scs:tax  -> execute TAX when C clear (skip if C set)
        BCS u_skip_tax
        TAX
u_skip_tax:
        STX u_prob           ; (unupkr_mul==0)
        PHP

        ; --simplified-prob-update:  ror @ / :3 lsr @ / adc #$f0 / add:sta (probs),y
        ROR
        LSR
        LSR
        LSR
        ADC #$f0
        ; add:sta (?probs),y  -> CLC / ADC (probs),y / STA (probs),y
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
        ; scc:inc ?state+1 -> execute INC when C set (skip if C clear)
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
        ; scs:dec ?state+1 -> execute DEC when C clear (skip if C set)
        BCS u_mul_skipdec
        DEC u_state+1
u_mul_skipdec:
        CLC
u_bit1b:
        STA u_state

        TYA
        INY
        RTS

; init values for zp_base+0..8; tab[0]=$80 doubles as the probs fill byte
; (the copy loop runs X=8..0, so tab[0] is loaded last and stays in A).
u_inittab:
        .byte $80                       ; u_bitBuf (empty, marker at bit 7)
        .byte <comp_data, >comp_data    ; u_src
        .byte <out_addr, >out_addr      ; u_dest
        .byte <probs_ram, >probs_ram    ; u_probs
        .byte 0, 0                      ; u_state
