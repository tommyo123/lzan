; ===========================================================================
; LZSA1 raw-block 6502 decruncher (size-optimized), in asm6502 syntax.
; Upstream: decompress_small_v1.asm (c) 2019 Emmanuel Marty, zlib.
;
; Forward variant only. The routine is one-shot per assembled image
; (@smc: yes): source=comp_data and dest=out_addr are baked into the
; self-modifying operand bytes at assembly time (valid because the emitter
; assembles a fresh image per run and calls full_decomp exactly once).
;
; Calling convention: entry = full_decomp, RTS at DECOMPRESSION_DONE.
; comp_data and out_addr are supplied by the harness.
; ===========================================================================
;@format: lzsa1
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzsa1-marty-small
;@encoder: lzan::lzsa1::compress_lzsa1
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 0
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 165

; ---- config-defaults ----
; ---- end config-defaults ----

; --- Match copy block lives ABOVE the token loop so its exit falls through ---
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

        ; Forward decompression -- put backreference bytes forward
        INC COPY_MATCH_LOOP+1
        BNE GETMATCH_DONE
        INC COPY_MATCH_LOOP+2
GETMATCH_DONE:

        DEX
        BNE COPY_MATCH_LOOP
        DEY
        BNE COPY_MATCH_LOOP
        ; fall through into DECODE_TOKEN when the match is fully copied

; Entry point: Y is reloaded with 0 at the top of every token, so the harness
; can call straight in here with Y undefined.
full_decomp:
DECODE_TOKEN:
        LDY #$00
        JSR GETSRC                      ; read token byte: O|LLL|MMMM
        PHA                             ; preserve token on stack

        ALR #$70                        ; ILLEGAL: (token&$70)>>1; Z set iff LLL==0
        BEQ NO_LITERALS                 ; skip if no literals to copy
        LSR
        LSR
        LSR
        CMP #$07                        ; LITERALS_RUN_LEN?
        BCC PREPARE_COPY_LITERALS       ; count directly embedded in token

        JSR GETSRC                      ; extra byte of variable literals count
        SBC #$F9                        ; (LITERALS_RUN_LEN)
        BCC PREPARE_COPY_LITERALS
        BEQ LARGE_VARLEN_LITERALS       ; if adding up to zero, grab 16-bit count

        JSR GETSRC                      ; single extended byte of literals count
        INY                             ; add 256 to literals count
        BCS PREPARE_COPY_LITERALS       ; (like JMP but shorter)

LARGE_VARLEN_LITERALS:
        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A
        TAY                             ; put high 8 bits in Y
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
        PHA                             ; preserve token again
        ASL                             ; carry = token bit 7 (16-bit offset flag)
        JSR GETSRC                      ; offset low byte (GETSRC preserves carry)
        TAX                             ; save low byte for later
        LDA #$FF                        ; assume 8-bit offset: high byte = $FF
        BCC GOT_OFFSET                  ; short offset: done
        JSR GETSRC                      ; 16-bit offset: fetch real high byte

GOT_OFFSET:
        ; Forward decompression - add match offset
        PHA                             ; save offhi on stack (over the token)
        TXA

        CLC                             ; add dest + match offset
        ADC PUTDST+1                    ; low 8 bits
        STA COPY_MATCH_LOOP+1           ; store back reference address
        PLA                             ; A = offhi again (PLA preserves carry)

        ADC PUTDST+2
        STA COPY_MATCH_LOOP+2           ; store high 8 bits of address

        PLA                             ; retrieve token from stack again
        AND #$0F                        ; isolate match len (MMMM)
        ADC #$02                        ; plus carry (always set by the high ADC)
        CMP #$12                        ; MATCH_RUN_LEN?
        BCC PREPARE_COPY_MATCH          ; count directly embedded in token

        JSR GETSRC                      ; extra byte of variable match length
        SBC #$EE                        ; add MATCH_RUN_LEN + MIN_MATCH_SIZE
        BCC PREPARE_COPY_MATCH
        BNE SHORT_VARLEN_MATCHLEN

        ; Handle 16 bits match length
        JSR GETLARGESRC                 ; low 8 bits in X, high 8 bits in A
        TAY                             ; put high 8 bits in Y
        BNE PREPARE_COPY_MATCH_Y        ; if not zero high byte, continue

DECOMPRESSION_DONE:
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
