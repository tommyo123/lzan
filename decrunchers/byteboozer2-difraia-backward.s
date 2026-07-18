; ===========================================================================
; ByteBoozer2 BACKWARD / in-place decruncher, in asm6502 syntax.
; Upstream: ByteBoozer2 Decruncher.inc (c) 2018 Luigi Di Fraia (decruncher by
; HCL 2003, B2 by David Malmborg 2014), MIT.
; Byte-identical decode vs lzan::bb2::compress_bb2_backward.
;
; Structure notes:
;   * The three self-modified stream operands are collapsed to one shared
;     descending fetch subroutine GetSrc; its single DEC GS+2 replaces the
;     three DECs. The operand is seeded at assembly time (routine is one-shot
;     per assembled image; @smc yes); the `>` prefix forces the absolute,X
;     encoding so GS+2 exists.
;   * The bit reader is de-inlined: GetBit (ASL bits / refill) shares one RTS.
;     The refill re-raises the guard with SEC (the shifted-out guard bit is
;     provably 1 whenever ASL leaves bits=0), so no PHP/PLP.
;   * GetLen is a shared subroutine (both callers) and stores cnt itself.
;   * The EOF (match-len 255) exit tail-merges into the shared RTS (GbRet).
;   * Y register conventions: Y==0 is an invariant at DLoop/Literal/Match
;     (seeded once at entry, both copy loops preserve it), so the selector
;     bit-builder starts from TYA, and `bits` is seeded with STY (0 triggers
;     the first refill just like $80 since SEC re-raises the guard). The
;     offset high-byte accumulator is kept raw in Y (M8: TAY; the short path
;     re-zeroes Y right after the Tab load, where the selector index dies)
;     and added directly in Mdone.
;   * dec16(put)+DEC cnt is one shared tail (DecPut) for both copy loops;
;     after its BNE falls through Z=1, so a 2-byte BEQ re-enters DLoop.
;   * ll's ==255 continue test is INC ll / BEQ (ll is dead afterwards).
; NOTE: M8's TAY is load-bearing: M8 is entered both via BEQ (A=0 -> Y=0) and
; by fall-through from the M_1 short-offset loop, where A holds the accumulated
; high-offset bits that become the offset high byte in Y.
; ===========================================================================
;@format: byteboozer2
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: byteboozer2-backward
;@encoder: lzan::bb2::compress_bb2_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 7
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 173

; ---- config-defaults ----
zp_base = $02
; ---- end config-defaults ----

bits = zp_base          ; 1
put  = zp_base+1        ; 2
mpos = zp_base+3        ; 2
cnt  = zp_base+5        ; 1  copy byte counter
ll   = zp_base+6        ; 1  saved literal run length (==255 continue test)

full_decomp:
        LDX #0             ; GetSrc index (stream operand is assembly-seeded)
        LDY #0             ; invariant: Y=0 at DLoop/Literal/Match
        LDA #<(out_addr+out_len-1)
        STA put
        LDA #>(out_addr+out_len-1)
        STA put+1
        STY bits           ; 0: first ASL leaves Z=1 -> refill (guard is
                           ; re-raised by SEC in GetBit, so $80 is not needed)
        ; fall into DLoop

DLoop:
        JSR GetBit         ; .GetNextBit() (#0)
        BCS Match

Literal:
        JSR GetLen         ; literal run length (GetLen also stores cnt)
        STA ll
LLoop:
        JSR GetSrc         ; fetch literal byte (descending)
        STA (put),Y        ; write DOWNWARD (Y=0)
        JSR DecPut         ; dec16 put, DEC cnt
        BNE LLoop
        INC ll             ; Z=1 iff run length was 255 (ll dead after)
        BEQ DLoop          ; full 255-run -> fresh copy bit
        ; sub-255 run falls into Match (no copy bit)

Match:
        JSR GetLen         ; match length (= match_len-1); also stored to cnt
        CMP #$FF           ; length 255 -> EOF
        BEQ GbRet          ; -> shared RTS
        CMP #2             ; C = (mlen >= 2) : short vs long table
        TYA                ; A = 0 (Y invariant)
        ROL
        JSR GetBit         ; .GetNextBit() (#1)
        ROL
        JSR GetBit         ; .GetNextBit() (#2)
        ROL
        TAY
        LDA Tab,Y
        BEQ M8             ; (A=0 -> TAY re-zeroes Y below)
        LDY #0             ; selector index is dead; short offsets add high 0

        ; Get bits < 8
M_1:
        JSR GetBit         ; .GetNextBit() (#3)
        ROL
        BCS M_1
        BMI Mdone          ; short offset: A = form byte, Y = 0
M8:
        TAY                ; Y = offset high bits (0 via the BEQ entry)
        JSR GetSrc         ; fetch offset low byte (descending), fall through
Mdone:
        ; mpos = put - disp  (disp = -(Y:offset), i.e. source is put+offset)
        EOR #$FF
        SEC
        ADC put
        STA mpos
        TYA
        ADC put+1
        STA mpos+1

        INC cnt            ; cnt = mlen+1 = match length
        LDY #0
MLoop:
        LDA (mpos),Y       ; source is HIGHER (put + offset)
        STA (put),Y
        LDA mpos           ; dec16 mpos
        BNE Mmp1
        DEC mpos+1
Mmp1:
        DEC mpos
        JSR DecPut         ; dec16 put, DEC cnt
        BNE MLoop
        BEQ DLoop          ; always (Z=1 from DEC cnt)

; ---- shared "advance output": dec16 put, DEC cnt (Z = done) --------------
DecPut:
        LDA put
        BNE Dp1
        DEC put+1
Dp1:
        DEC put
        DEC cnt
        RTS

; ---- shared .GetLen() : gamma length -> A (and cnt) -----------------------
GetLen:
        LDA #1
GlLoop:
        JSR GetBit
        BCC GlEnd
        JSR GetBit
        ROL
        BPL GlLoop
GlEnd:
        STA cnt
        RTS

; ---- shared bit reader --------------------------------------------------
; GetBit: shift one bit of `bits` into carry; on empty buffer refill.
; Preserves A (gamma accumulator); returns delivered bit in carry.
; When ASL leaves bits=0 the shifted-out bit is the guard (always 1), so the
; refill re-raises it with SEC before rolling the new byte.
GetBit:
        ASL bits
        BNE GbRet
        PHA                ; save gamma accumulator
        JSR GetSrc
        SEC                ; guard bit
        ROL                ; guard in, bit7 out -> carry = delivered bit
        STA bits
        PLA                ; N/Z clobbered; only carry is contractual
GbRet:
        RTS

; ---- shared descending stream fetch -> A --------------------------------
; Operand seeded at assembly time; `>` forces absolute,X so GS+2 is real.
GetSrc:
GS:
        LDA >comp_data+comp_data_len-1,X
        DEX
        CPX #$FF
        BNE GSok
        DEC GS+2
GSok:
        RTS

Tab:
        ; Short offsets
        .byte $DF                ; 3
        .byte $FB                ; 6
        .byte $00                ; 8
        .byte $80                ; 10
        ; Long offsets
        .byte $EF                ; 4
        .byte $FD                ; 7
        .byte $80                ; 10
        .byte $F0                ; 13
