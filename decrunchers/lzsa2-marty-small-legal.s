; ===========================================================================
; Legal-only variant of lzsa2-marty-small.s (no undocumented opcodes): the one
; `ALR #$18` illegal op is expanded to the legal pair `AND #$18` / `LSR`
; (identical A, Z and C), costing 1 byte over the standard body. Decodes the
; same stream (lzan::lzsa2::compress_lzsa2_anchor). Use when illegal opcodes
; are unwanted.
; Upstream: decompress_small_v2.asm (c) 2019 Emmanuel Marty, zlib.
;
; Same assembly-time SMC seeding as the standard file: the GETSRC/PUTDST
; pointer operands are assembled to comp_data/out_addr and the rep-offset
; placeholder immediates to $FF, so there is no runtime init preamble.
;
; Entry = `full_decomp`; RTS at DECOMPRESSION_DONE.
; ===========================================================================
;@format: lzsa2
;@direction: forward
;@variant: legal
;@entry: full_decomp
;@vfy-key: lzsa2-marty-small-legal
;@encoder: lzan::lzsa2::compress_lzsa2_anchor(i)
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 1
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 241

; ---- config-defaults ----
zp_base = $FC
; ---- end config-defaults ----

NIBCOUNT = zp_base+0

; The rep-match offset placeholder (OFFSLO/OFFSHI, the self-modifying ADC
; operands) is assembled as $FFFF. lzsa stores match offsets as two's
; complement, so a real match always leaves the offset-high byte near $FF and
; the EOD's offset-add carries (that carry is what the EOD detection relies
; on). For an all-literals stream the EOD token is the very first command, so
; no prior match has set the offset - without this seed its placeholder would
; not carry and EOD would be missed. The offset is never used for a copy at
; EOD, so $FFFF is safe.
full_decomp:
DECOMPRESS_LZSA2:
        LDY #$00
        STY NIBCOUNT

DECODE_TOKEN:
        JSR GETSRC                      ; read token byte: XYZ|LL|MMM
        PHA                             ; preserve token on stack

        AND #$18                        ; legal expansion of ALR #$18:
        LSR                             ; (token & $18) >> 1  (BEQ still valid)
        BEQ NO_LITERALS
        LSR
        LSR
        CMP #$03                        ; LITERALS_RUN_LEN_V2?
        BCC PREPARE_COPY_LITERALS

        JSR GETNIBBLE                   ; extra literals length nibble
        ADC #$02                        ; (LITERALS_RUN_LEN_V2) minus carry
        CMP #$12                        ; LITERALS_RUN_LEN_V2 + 15 ?
        BCC PREPARE_COPY_LITERALS

        JSR GETSRC                      ; extra byte of variable literals count
        SBC #$EE                        ; overflow?

PREPARE_COPY_LITERALS:
        TAX
        BCC PREPARE_COPY_LITERALS_HIGH

        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A
        TAY

PREPARE_COPY_LITERALS_HIGH:
        TXA
        BEQ COPY_LITERALS
        INY

COPY_LITERALS:
        JSR GETPUT                      ; copy one byte of literals
        DEX
        BNE COPY_LITERALS
        DEY
        BNE COPY_LITERALS

NO_LITERALS:
        PLA                             ; retrieve token from stack
        PHA
        ASL
        BCS REPMATCH_OR_LARGE_OFFSET    ; 1YZ: rep-match or 13/16 bit offset

        ASL                             ; 0YZ: 5 or 9 bit offset
        BCS OFFSET_9_BIT

        ; 00Z: 5 bit offset
        LDX #$FF                        ; set offset bits 15-8 to 1
        JSR GETCOMBINEDBITS             ; rotate Z bit into bit 0, read nibble for bits 4-1
        ORA #$E0                        ; set bits 7-5 to 1
        BNE GOT_OFFSET_LO               ; store low byte and prepare match

OFFSET_9_BIT:                           ; 01Z: 9 bit offset
        ROL                             ; carry: Z bit; A: xxxxxxx1
        ADC #$00                        ; if Z set, add 1
        ORA #$FE                        ; set offset bits 15-9 to 1
        BNE GOT_OFFSET_HI               ; (like JMP GOT_OFFSET_HI but shorter)

REPMATCH_OR_LARGE_OFFSET:
        ASL                             ; 13 bit offset?
        BCS REPMATCH_OR_16_BIT

        ; 10Z: 13 bit offset
        JSR GETCOMBINEDBITS             ; rotate Z bit into bit 8, read nibble for bits 12-9
        ADC #$DE                        ; set bits 15-13 to 1 and subtract 2 (512)
        BNE GOT_OFFSET_HI

REPMATCH_OR_16_BIT:                     ; rep-match or 16 bit offset
        BMI REP_MATCH                   ; reuse previous offset (rep-match)

        ; 110: handle 16 bit offset
        JSR GETSRC                      ; grab high 8 bits
GOT_OFFSET_HI:
        TAX
        JSR GETSRC                      ; grab low 8 bits
GOT_OFFSET_LO:
        STA OFFSLO                      ; store low byte of match offset
        STX OFFSHI                      ; store high byte of match offset

REP_MATCH:
        ; Forward decompression - add match offset
        CLC
        LDA PUTDST+1                    ; low 8 bits
OFFSLO = *+1
        ADC #$FF                        ; seeded $FF (rep-offset placeholder)
        STA COPY_MATCH_LOOP+1           ; store back reference address
OFFSHI = *+1
        LDA #$FF                        ; high 8 bits, seeded $FF
        ADC PUTDST+2
        STA COPY_MATCH_LOOP+2           ; store high 8 bits of address

        PLA                             ; retrieve token from stack again
        AND #$07                        ; isolate match len (MMM)
        ADC #$01                        ; add MIN_MATCH_SIZE_V2 and carry
        CMP #$09                        ; MIN_MATCH_SIZE_V2 + MATCH_RUN_LEN_V2?
        BCC PREPARE_COPY_MATCH

        JSR GETNIBBLE                   ; extra match length nibble
        ADC #$08                        ; (MIN_MATCH_SIZE_V2 + MATCH_RUN_LEN_V2) minus carry
        CMP #$18
        BCC PREPARE_COPY_MATCH

        JSR GETSRC                      ; extra byte of variable match length
        SBC #$E8                        ; overflow?

PREPARE_COPY_MATCH:
        TAX
        BCC PREPARE_COPY_MATCH_Y
        BEQ DECOMPRESSION_DONE          ; if EOD code, bail

        JSR GETLARGESRC                 ; 16 bit match length: low in X, high in A
        TAY

PREPARE_COPY_MATCH_Y:
        TXA
        BEQ COPY_MATCH_LOOP
        INY

COPY_MATCH_LOOP:
        LDA $AAAA                       ; get one byte of backreference
        JSR PUTDST                      ; copy to destination

        ; Forward decompression -- put backreference bytes forward
        INC COPY_MATCH_LOOP+1
        BNE GETMATCH_DONE
        INC COPY_MATCH_LOOP+2
GETMATCH_DONE:

        DEX
        BNE COPY_MATCH_LOOP
        DEY
        BNE COPY_MATCH_LOOP
        JMP DECODE_TOKEN

GETCOMBINEDBITS:
        EOR #$80
        ASL
        PHP

        JSR GETNIBBLE                   ; get nibble into bits 0-3 (offset bits 1-4)
        PLP                             ; merge Z bit as carry (offset bit 0)
        ROL                             ; nibble -> bits 1-4; carry -> bit 0
DECOMPRESSION_DONE:
        RTS

GETNIBBLE:
NIBBLES = *+1
        LDA #$AA
        LSR NIBCOUNT
        BCS HAS_NIBBLES

        INC NIBCOUNT
        JSR GETSRC                      ; get 2 nibbles
        STA NIBBLES
        LSR
        LSR
        LSR
        LSR
        SEC

HAS_NIBBLES:
        AND #$0F                        ; isolate low 4 bits of nibble
        RTS

; --- Forward GETPUT / PUTDST / GETLARGESRC / GETSRC --------------------------
GETPUT:
        JSR GETSRC
PUTDST:
        .byte $8D                       ; STA abs - SMC operand is the word below
        .word out_addr                  ; assembled dest pointer seed
        INC PUTDST+1
        BNE PUTDST_DONE
        INC PUTDST+2
PUTDST_DONE:
        RTS

GETLARGESRC:
        JSR GETSRC                      ; grab low 8 bits
        TAX                             ; move to X
                                        ; fall through grab high 8 bits
GETSRC:
        .byte $AD                       ; LDA abs - SMC operand is the word below
        .word comp_data                 ; assembled source pointer seed
        INC GETSRC+1
        BNE GETSRC_DONE
        INC GETSRC+2
GETSRC_DONE:
        RTS
