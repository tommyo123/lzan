; ===========================================================================
; LZSA1 raw-block 6502 decruncher, backward / in-place, in asm6502 syntax.
; Upstream: decompress_small_v1.asm (c) 2019 Emmanuel Marty, zlib.
; Decodes `lzsa -r -b -f1` == lzan::lzsa1::compress_lzsa1_backward.
;
; Backward in-place: the source/destination pointers live as SMC operands
; seeded at assembly time (the routine is one-shot per assembled image):
;   * GETSRC pre-decrements from comp_data+comp_data_len (one past the last
;     byte; `>` forces absolute addressing so a stream ending at $FFFF wraps
;     through $0000 correctly). Its DCP forces C=1 on return, which every
;     SBC caller relied on and which replaces the SEC in GOT_OFFSET (both
;     entry paths arrive straight from a GETSRC).
;   * PUTDST post-decrements from out_addr+out_len-1 (the last byte) and
;     exits with A=$FF from its own DCP; COPY_MATCH_LOOP reuses that A=$FF
;     for its own DCP pointer decrement. On exit PUTDST's operand = out_addr-1.
; The match offset is SUBTRACTED (back-ref lies at higher addresses); the
; high offset byte rides the stack as its complement (EOR #$FF / PHA .. PLA /
; ADC == the SBC). The ALR #$70 illegal op is used for the literals-length
; field.
; ===========================================================================
;@format: lzsa1
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzsa1-marty-backward
;@encoder: lzan::lzsa1::compress_lzsa1_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 0
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 178

; ---- config-defaults ----
; ---- end config-defaults ----

full_decomp:
        LDY #$00

DECODE_TOKEN:
        JSR GETSRC                      ; read token byte: O|LLL|MMMM
        PHA                             ; preserve token on stack

        ALR #$70                        ; (token & $70) >> 1  (illegal op; BEQ still valid)
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
        BCS PREPARE_COPY_LITERALS       ; (*like JMP but shorter: GETSRC forces C=1)

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
        JSR PUTDST                      ; copy to destination (exits A=$FF)

        ; Backward decompression -- put backreference bytes backward.
        ; DCP = DEC+CMP (illegal): with A=$FF (from PUTDST) Z=1 iff the low
        ; byte wrapped to $FF.
        DCP COPY_MATCH_LOOP+1
        BNE GETMATCH_DONE
        DEC COPY_MATCH_LOOP+2
GETMATCH_DONE:
        DEX
        BNE COPY_MATCH_LOOP
        DEY
        BNE COPY_MATCH_LOOP
        BEQ DECODE_TOKEN                ; (*like JMP DECODE_TOKEN but shorter)

GET_LONG_OFFSET:                        ; handle 16 bit offset:
        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A

GOT_OFFSET:
        ; Backward decompression - subtract match offset. The high offset
        ; byte rides the stack as its complement (PHA over the token slot),
        ; dropping the OFFSHI SMC temp: A + ~M + C == A - M - !C (i.e. SBC M).
        ; No SEC needed: both paths here end in a GETSRC whose DCP forced C=1.
        EOR #$FF                        ; A = ~offhi
        PHA                             ; save on stack (over the token)
        STX OFFSLO
        LDA PUTDST+1                    ; subtract dest - match offset (C=1)
OFFSLO = *+1
        SBC #$AA                        ; low 8 bits
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
