; ===========================================================================
; ByteBoozer2 standard in-memory decruncher (forward), in asm6502 syntax.
; Upstream: ByteBoozer2 Decruncher.inc (c) 2018 Luigi Di Fraia (decruncher by
; HCL 2003, B2 by David Malmborg 2014), MIT.
;
; Structure notes:
;   * GetBit = "ASL bits / BNE <rts>" falling through into the refill
;     (TAY-save of A, fetch, ROL A with the sentinel carry, STA bits).
;     Callers consume only C, which every path preserves.
;   * GetLen returns the gamma length in A; both callers STA it into their
;     CPY operand (LLen/MLen self-modifying compares).
;   * GetByte returns the next stream byte in A, preserving Y and C; used by
;     the header fetch, the literal copy loop, the offset low byte and the
;     bit refill. `.byte $2C` (BIT abs) swallows MShort's "LDY #$FF".
;   * EOF's BEQ targets GbEnd, the shared RTS.
;
; STREAM CONVENTION: the standalone decoder reads the 2-byte decrunch-TO address
; from the first two stream bytes, then the bitstream. lzan::bb2::compress_bb2
; returns only the crunched bitstream body (no dest header), so the harness
; prepends [out_addr_lo, out_addr_hi].
;
; ZP: bits ($02, 1 byte), put ($04, 2 bytes). Entry = full_decomp; EOF = a
; match length of $FF -> RTS (via the shared GbEnd return).
; ===========================================================================
;@format: byteboozer2
;@direction: forward
;@variant: standard
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
;@code-bytes: 175

; ---- config-defaults ----
zp_base = $02
; ---- end config-defaults ----

; zp_base+1 is the hole put-1 (STA put-1,X index base) - keep put at +2.
bits    = zp_base       ; 1
put     = zp_base+2     ; 2

full_decomp:
        LDX #0
init_loop:
        JSR GetByte     ; X post-incremented: X=1 stores put, X=2 stores put+1
        STA put-1,X
        CPX #2
        BCC init_loop
        LDA #$80
        STA bits
DLoop:
        JSR GetBit
        BCS Match
Literal:
        ; Literal run.. get length.
        JSR GetLen
        STA LLen+1

        LDY #0
LLoop:
        JSR GetByte
        STA (put),Y
        INY
LLen:
        CPY #0
        BNE LLoop

        CLC
        JSR AddPut
        INY
        BEQ DLoop

        ; Has to continue with a match..
Match:
        ; Match.. get length.
        JSR GetLen
        STA MLen+1

        ; Length 255 -> EOF
        CMP #$FF
        BEQ GbEnd

        ; Get num bits
        CMP #2
        LDA #0
        ROL
        JSR GetBit
        ROL
        JSR GetBit
        ROL
        TAY
        LDA Tab,Y
        BEQ M8

        ; Get bits < 8
M_1:
        JSR GetBit
        ROL
        BCS M_1
        BMI MShort
M8:
        ; Get byte
        EOR #$FF
        TAY
        JSR GetByte
        .byte $2C   ; BIT abs -> swallow the following "LDY #$FF" (skip trick)
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
        JSR AddPut
        JMP DLoop

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

AddPut:
        TYA
        ADC put
        STA put
        BCC ApEnd
        INC put+1
ApEnd:
        RTS

GetBit:
        ASL bits
        BNE GbEnd       ; C = extracted bit
        ; fall into the refill (C=1: the shifted-out sentinel)
GetNewBits:
        TAY             ; save caller's A
        JSR GetByte
        ROL             ; A = (byte<<1)|1, C = byte bit7 (first data bit)
        STA bits
        TYA
        RTS

GetByte:
Get1:
        LDA comp_data,X ; SMC: high byte INC'd on wrap (seeded at assembly)
        INX
        BNE GbEnd
        INC Get1+2
GbEnd:
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
