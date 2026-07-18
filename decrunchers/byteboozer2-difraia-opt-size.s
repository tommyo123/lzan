; ===========================================================================
; ByteBoozer2 standard in-memory decruncher (forward), opt-size variant, in
; asm6502 syntax.
; Upstream: ByteBoozer2 Decruncher.inc (c) 2018 Luigi Di Fraia (decruncher by
; HCL 2003, B2 by David Malmborg 2014), MIT.
;
; Smaller but slower than the standard variant: GetNextBit and GetLen are folded
; into shared subroutines GetBit / GetLen (extra JSR/RTS per bit). Decode is
; byte-identical:
;   * GetBit = "ASL bits / BEQ GetNewBits / RTS". Carry (the extracted bit) is
;     preserved across RTS; on underflow it BEQ-falls into GetNewBits whose RTS
;     returns to GetBit's caller with C = next real bit, exactly the inline
;     semantics. Y/X are only clobbered when a refill happens, same as inline.
;   * GetLen returns the gamma length in A (flags reset by the caller's STA/CMP).
; The "JMP Mdone" that skipped "LDY #$FF" is a $2C (BIT abs) that swallows the
; LDY as its operand (BIT touches N/V/Z only, not A/X/Y/C; Mdone's ADC only
; depends on carry, which BIT leaves untouched).
;
; STREAM CONVENTION: reads the 2-byte decrunch-TO address from the first two
; stream bytes, then the bitstream (harness prepends [out_addr_lo, out_addr_hi]).
; ZP: bits ($02), put ($04). Entry = full_decomp; EOF = match length $FF.
; ===========================================================================
;@format: byteboozer2
;@direction: forward
;@variant: opt-size
;@entry: full_decomp
;@vfy-key: byteboozer2-difraia
;@encoder: lzan::bb2::compress_bb2
;@payload: dst-prefixed
;@eof: stream
;@needs: comp_data
;@zp-len: 4
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 214

; ---- config-defaults ----
zp_base = $02
; ---- end config-defaults ----

; zp_base+1 is the hole put-1 (STY put-1,X index base) - keep put at +2.
bits    = zp_base       ; 1
put     = zp_base+2     ; 2

full_decomp:
        LDY #<comp_data
        LDX #>comp_data
        ; fall into Decrunch

Decrunch:
        STY Get1+1
        STY Get2+1
        STY Get3+1
        STX Get1+2
        STX Get2+2
        STX Get3+2

        LDX #0
init_loop:
        JSR GetNewBits
        STY put-1,X
        CPX #2
        BCC init_loop
        LDA #$80
        STA bits
DLoop:
        JSR GetBit              ; .GetNextBit() (#0)
        BCS Match
Literal:
        ; Literal run.. get length.
        JSR GetLen
        STA LLen+1

        LDY #0
LLoop:
Get3:
        LDA $FEED,X
        INX
        BNE Get3_no_inc
        JSR GnbInc
Get3_no_inc:
L1:
        STA (put),Y
        INY
LLen:
        CPY #0
        BNE LLoop

        CLC
        TYA
        ADC put
        STA put
        BCC lit_no_inc
        INC put+1
lit_no_inc:
        INY
        BEQ DLoop

        ; Has to continue with a match..
Match:
        ; Match.. get length.
        JSR GetLen
        STA MLen+1

        ; Length 255 -> EOF
        CMP #$FF
        BEQ End

        ; Get num bits
        CMP #2
        LDA #0
        ROL
        JSR GetBit              ; .GetNextBit() (#1)
        ROL
        JSR GetBit              ; .GetNextBit() (#2)
        ROL
        TAY
        LDA Tab,Y
        BEQ M8

        ; Get bits < 8
M_1:
        JSR GetBit              ; .GetNextBit() (#3)
        ROL
        BCS M_1
        BMI MShort
M8:
        ; Get byte
        EOR #$FF
        TAY
Get2:
        LDA $FEED,X
        INX
        BNE Get2_no_inc
        JSR GnbInc
Get2_no_inc:
        .byte $2C               ; BIT abs -> swallow next "LDY #$FF" (skip trick)
MShort:
        LDY #$FF
Mdone:
        ; clc
        ADC put
        STA MLda+1
        TYA
        ADC put+1
        STA MLda+2

        LDY #$FF
MLoop:
        INY
MLda:
        LDA $BEEF,Y
        STA (put),Y
MLen:
        CPY #0
        BNE MLoop

        ; sec
        TYA
        ADC put
        STA put
        BCC match_no_inc
        INC put+1
match_no_inc:
        JMP DLoop

End:
        RTS

; ---- shared length reader: returns gamma length in A ----
GetLen:
        LDA #1
GlLoop:
        JSR GetBit
        BCC GlEnd
        JSR GetBit
        ROL
        BPL GlLoop
GlEnd:
        RTS

; ---- shared single-bit reader: C = next bit; falls into GetNewBits on refill ----
GetBit:
        ASL bits
        BEQ GetNewBits
        RTS
GetNewBits:
Get1:
        LDY $FEED,X
        STY bits
        ROL bits
        INX
        BNE GnbEnd
GnbInc:
        INC Get1+2
        INC Get2+2
        INC Get3+2
GnbEnd:
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
