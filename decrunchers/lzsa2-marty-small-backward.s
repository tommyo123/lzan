; ===========================================================================
; LZSA2 raw-block 6502 decruncher, backward / in-place, in asm6502 syntax.
; Upstream: decompress_small_v2.asm (c) 2019 Emmanuel Marty, zlib.
; Decodes the `lzsa -r -b -f2` stream == lzan::lzsa2::compress_lzsa2_anchor_backward.
;
; Backward in-place decoder: the source/destination pointers live as SMC
; operands seeded at assembly time (the routine is one-shot per assembled
; image): GETSRC pre-decrements from comp_data+comp_data_len (one past the
; last byte; `>` forces absolute addressing so a stream ending at $FFFF wraps
; through $0000 correctly), PUTDST post-decrements from out_addr+out_len-1
; (the last byte). A match offset is SUBTRACTED. This lets packed and
; unpacked regions overlap (write head trails read head) so a file can be
; decrunched over itself. The literals/offset/length decode (incl. the
; ALR #$18 illegal op) is direction-independent and unchanged.
;
;   * On exit PUTDST's operand = out_addr-1.
;   * The rep-match offset operands (OFFSLO/OFFSHI) need no seeding: the
;     match-source subtraction runs only after the EOD bail-out, and the
;     encoder never emits a rep-match before the first real match.
;   * `comp_data_len` and `out_len` are supplied by the harness.
; ===========================================================================
;@format: lzsa2
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzsa2-marty-backward
;@encoder: lzan::lzsa2::compress_lzsa2_anchor_backward(i)
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 1
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 238

; ---- config-defaults ----
zp_base = $FC
; ---- end config-defaults ----

NIBCOUNT = zp_base+0

full_decomp:
        LDY #$00
        STY NIBCOUNT

DECODE_TOKEN:
        JSR GETSRC                      ; read token byte: XYZ|LL|MMM
        PHA                             ; preserve token on stack

        ALR #$18                        ; (token & $18) >> 1  (illegal op; BEQ still valid)
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
        JSR PREPARE_COPY                ; count -> X (low) / Y (high, adjusted)

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
        DEX                             ; X=0 here (copy loops end X=0; the
                                        ;   first token always has literals),
                                        ;   so DEX sets offset bits 15-8 to 1
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
        SEC                             ; carry set for the ADC #$01 below
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
        BEQ DECOMPRESSION_DONE          ; A=0 only on the EOD code: every short
                                        ;   count is 2..255 and the 16-bit
                                        ;   marker leaves A=1, so Z alone
                                        ;   identifies EOD
        JSR PREPARE_COPY                ; count -> X (low) / Y (high, adjusted)

        ; Backward decompression - subtract match offset (after the EOD
        ; bail-out, so the offset operands never need a runtime seed)
        SEC
        LDA PUTDST+1                    ; low 8 bits
OFFSLO = *+1
        SBC #$AA
        STA COPY_MATCH_LOOP+1           ; store back reference address
        LDA PUTDST+2                    ; high 8 bits
OFFSHI = *+1
        SBC #$AA
        STA COPY_MATCH_LOOP+2           ; store high 8 bits of address

COPY_MATCH_LOOP:
        LDA $AAAA                       ; get one byte of backreference
        JSR PUTDST                      ; copy to destination

        ; Backward decompression -- put backreference bytes backward.
        ; DCP = DEC+CMP (illegal): Z=1 iff the low byte wrapped to $FF.
        ; A = $FF here: PUTDST always exits with A=$FF from its own DCP.
        DCP COPY_MATCH_LOOP+1
        BNE GETMATCH_DONE
        DEC COPY_MATCH_LOOP+2
GETMATCH_DONE:
        DEX
        BNE COPY_MATCH_LOOP
        DEY
        BNE COPY_MATCH_LOOP
        JMP DECODE_TOKEN

; Shared literals/match count tail. In: A = 8-bit count, or (C=1) a 16-bit
; count follows in the stream. Out: X = count low, Y = count high adjusted
; for the DEX/BNE/DEY/BNE copy loops. Preserves carry until the BCC.
PREPARE_COPY:
        TAX
        BCC PREPARE_COPY_HIGH
        JSR GETLARGESRC                 ; 16 bit count: low in X, high in A
        TAY
PREPARE_COPY_HIGH:
        TXA
        BEQ PREPARE_COPY_DONE
        INY
PREPARE_COPY_DONE:
        RTS

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

; --- Backward GETPUT / PUTDST / GETLARGESRC / GETSRC (decrementing) ----------
GETPUT:
        JSR GETSRC
PUTDST:
        STA >out_addr + out_len - 1     ; SMC operand, seeded to the LAST byte
        LDA #$FF
        DCP PUTDST+1                    ; dec low; Z=1 iff it wrapped to $FF
        BNE PUTDST_DONE
        DEC PUTDST+2
PUTDST_DONE:
        RTS

GETLARGESRC:
        JSR GETSRC                      ; grab low 8 bits
        TAX                             ; move to X
                                        ; fall through grab high 8 bits
GETSRC:                                 ; pre-decrementing read (forces C=1,
        LDA #$FF                        ;   which every SBC caller relied on)
        DCP GETSRC_LDA+1                ; dec low; Z=1 iff it wrapped to $FF
        BNE GETSRC_LDA
        DEC GETSRC_LDA+2
GETSRC_LDA:
        LDA >comp_data + comp_data_len  ; SMC operand, seeded ONE PAST the last
        RTS                             ;   byte (forced absolute: may wrap $0000)
