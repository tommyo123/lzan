; ===========================================================================
; Legal-only variant of lzsa1-marty-small-backward.s (no undocumented opcodes).
; The illegal ops of the standard body expand to legal pairs:
;   * ALR #$70 -> AND #$70 / LSR (identical A, Z, C);
;   * the three DCP pointer decrements (GETSRC / PUTDST / COPY_MATCH_LOOP)
;     -> the classic test-low / dec-high / dec-low sequences, which no longer
;     force C=1, so GOT_OFFSET keeps its SEC.
; The low offset byte is also complemented in A (TXA / EOR #$FF / ADC == the
; SBC), dropping the OFFSLO SMC temp. Decodes the same stream
; (lzan::lzsa1::compress_lzsa1_backward). Use when illegal opcodes are unwanted.
; Upstream: decompress_small_v1.asm (c) 2019 Emmanuel Marty, zlib.
;
; Backward in-place: the source/destination pointers live as SMC operands
; seeded at assembly time (the routine is one-shot per assembled image):
;   * GETSRC pre-decrements from comp_data+comp_data_len (one past the last
;     byte; `>` forces absolute addressing so a stream ending at $FFFF wraps
;     through $0000 correctly). It preserves carry, which every SBC caller
;     relies on.
;   * PUTDST post-decrements from out_addr+out_len-1 (the last byte). On
;     exit PUTDST's operand = out_addr-1.
; The match offset is SUBTRACTED (back-ref lies at higher addresses); the
; high offset byte rides the stack as its complement (EOR #$FF / PHA .. PLA /
; ADC == the SBC).
; ===========================================================================
;@format: lzsa1
;@direction: backward
;@variant: legal
;@entry: full_decomp
;@vfy-key: lzsa1-marty-legal-backward
;@encoder: lzan::lzsa1::compress_lzsa1_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 0
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 183

; ---- config-defaults ----
; ---- end config-defaults ----

full_decomp:
        LDY #$00

DECODE_TOKEN:
        JSR GETSRC                      ; read token byte: O|LLL|MMMM
        PHA                             ; preserve token on stack

        AND #$70                        ; legal expansion of ALR #$70:
        LSR                             ; (token & $70) >> 1  (BEQ still valid)
        BEQ NO_LITERALS
        LSR
        LSR
        LSR
        CMP #$07                        ; LITERALS_RUN_LEN?
        BCC PREPARE_COPY_LITERALS

        JSR GETSRC                      ; extra byte of variable literals count
        SBC #$F9                        ; (LITERALS_RUN_LEN)
        BCC PREPARE_COPY_LITERALS
        BEQ LARGE_VARLEN_LITERALS       ; adding up to zero -> 16-bit count

        JSR GETSRC                      ; single extended byte of variable literals count
        INY                             ; add 256 to literals count
        BCS PREPARE_COPY_LITERALS       ; (*like JMP but shorter: C=1 from the
                                        ;   SBC above, GETSRC preserves carry)
LARGE_VARLEN_LITERALS:
        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A
        TAY
        TXA

PREPARE_COPY_LITERALS:
        TAX
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
        BMI GET_LONG_OFFSET             ; $80: 16 bit offset

        JSR GETSRC                      ; 8 bit offset from stream in A
        TAX                             ; save for later
        LDA #$FF                        ; high 8 bits
        BNE GOT_OFFSET                  ; (*like JMP GOT_OFFSET but shorter)

SHORT_VARLEN_MATCHLEN:
        JSR GETSRC                      ; single extended byte of variable match len
        INY                             ; add 256 to match length

PREPARE_COPY_MATCH:
        TAX
PREPARE_COPY_MATCH_Y:
        TXA
        BEQ COPY_MATCH_LOOP
        INY

COPY_MATCH_LOOP:
        LDA $AAAA                       ; get one byte of backreference
        JSR PUTDST                      ; copy to destination

        ; Backward decompression -- put backreference bytes backward
        ; (legal expansion of the standard's DCP pointer decrement).
        LDA COPY_MATCH_LOOP+1
        BNE GETMATCH_DONE
        DEC COPY_MATCH_LOOP+2
GETMATCH_DONE:
        DEC COPY_MATCH_LOOP+1
        DEX
        BNE COPY_MATCH_LOOP
        DEY
        BNE COPY_MATCH_LOOP
        BEQ DECODE_TOKEN                ; (*like JMP DECODE_TOKEN but shorter)

GET_LONG_OFFSET:                        ; handle 16 bit offset:
        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A

GOT_OFFSET:
        ; Backward decompression - subtract match offset. BOTH offset bytes
        ; ride as complements (A + ~M + C == A - M - !C, i.e. SBC M): the
        ; high byte on the stack (over the token), the low byte in A via
        ; TXA / EOR -- dropping both the OFFSHI and OFFSLO SMC temps.
        EOR #$FF                        ; A = ~offhi
        PHA                             ; save on stack (over the token)
        TXA                             ; A = offlo
        EOR #$FF                        ; A = ~offlo
        SEC                             ; carry is NOT guaranteed on the legal paths
        ADC PUTDST+1                    ; dst_lo + ~offlo + 1 == dst_lo - offlo
        STA COPY_MATCH_LOOP+1           ; store back reference address
        PLA                             ; A = ~offhi again (PLA preserves carry)
        ADC PUTDST+2                    ; dst_hi + ~offhi + C == dst_hi - offhi - !C
        STA COPY_MATCH_LOOP+2           ; store high 8 bits of address
        SEC

        PLA                             ; retrieve token from stack again
        AND #$0F                        ; isolate match len (MMMM)
        ADC #$02                        ; plus carry (always set by the SEC above)
        CMP #$12                        ; MATCH_RUN_LEN?
        BCC PREPARE_COPY_MATCH

        JSR GETSRC                      ; extra byte of variable match length
        SBC #$EE                        ; add MATCH_RUN_LEN and MIN_MATCH_SIZE
        BCC PREPARE_COPY_MATCH
        BNE SHORT_VARLEN_MATCHLEN

        JSR GETLARGESRC                 ; 16 bit match length: low in X, high in A
        TAY
        BNE PREPARE_COPY_MATCH_Y        ; large match length with nonzero high byte

DECOMPRESSION_DONE:
        RTS

; --- Backward GETPUT / PUTDST / GETLARGESRC / GETSRC (decrementing) ----------
GETPUT:
        JSR GETSRC
PUTDST:
        STA >out_addr + out_len - 1     ; SMC operand, seeded to the LAST byte
        LDA PUTDST+1
        BNE PUTDST_DONE
        DEC PUTDST+2
PUTDST_DONE:
        DEC PUTDST+1
        RTS

GETLARGESRC:
        JSR GETSRC                      ; grab low 8 bits
        TAX                             ; move to X
                                        ; fall through grab high 8 bits
GETSRC:                                 ; pre-decrementing read (preserves carry)
        LDA GETSRC_LDA+1
        BNE GETSRC_DEC
        DEC GETSRC_LDA+2
GETSRC_DEC:
        DEC GETSRC_LDA+1
GETSRC_LDA:
        LDA >comp_data + comp_data_len  ; SMC operand, seeded ONE PAST the last
        RTS                             ;   byte (forced absolute: may wrap $0000)
