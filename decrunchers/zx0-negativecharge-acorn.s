; ===========================================================================
; Standard ZX0 (v2) 6502 decruncher, forward, in asm6502 syntax.
; Upstream: BeebAsm ZX0 decoder by NegativeCharge (port of Krzysztof "XXL"
; Dudek's standard ZX0 decoder); ZX0 format (c) Einar Saukas, BSD-3-Clause.
;
; This decodes a standard ZX0 v2 forward stream, exactly what
; lzan::zx0compat::compress_zx0_compatible emits (not a bitfire/Dali-modified
; variant).
;
; The SMC operands (lenL/lenH/offsetL/offsetH and the get_byte source) are
; seeded at assembly time, so there is no runtime init block; the assembled
; image is one-shot (a second call would see dirty state). The self-modifying
; operand labels `lenL = *-1` etc. name the operand byte of the preceding
; immediate load; the decode loop patches those bytes at runtime and their
; assembled values are the required initial state (lenL/lenH = $00,
; offsetL/offsetH = $FF). get_byte's source operand is an explicit `.byte $AD`
; (LDA abs) + `ZX0_INPUT: .word comp_data`, always 3 bytes and starting out
; pointing at the stream. ZX0_OUTPUT (output cursor) and COPY_SRC (match source)
; live in zero page.
;
; Entry = full_decomp; EOF is the standard ZX0 end marker -> RTS.
; ===========================================================================
;@format: zx0
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: zx0-negativecharge-acorn
;@encoder: lzan::zx0compat::compress_zx0_compatible(i)
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 4
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 183

; $F7-$FA: RS-232 buffer pointers, free unless RS-232 is used.
; ---- config-defaults ----
zp_base = $F7
; ---- end config-defaults ----

ZX0_OUTPUT = zp_base+0  ; 2 bytes: output cursor (ZP)
COPY_SRC   = zp_base+2  ; 2 bytes: match-copy source pointer (ZP)

full_decomp:
        LDA #<out_addr
        STA ZX0_OUTPUT
        LDA #>out_addr
        STA ZX0_OUTPUT+1
        LDA #$80                ; empty bit buffer (marker bit only)
        ; falls through into dzx0s_literals; lenL/lenH/offsetL/offsetH and
        ; the get_byte operand carry their initial values from assembly.
dzx0s_literals:
        JSR dzx0s_elias
        PHA
cop0:
        JSR get_byte
        LDY #$00
        STA (ZX0_OUTPUT),Y
        INC ZX0_OUTPUT
        BNE l0
        INC ZX0_OUTPUT+1
l0:
        LDA #$00                ; SMC: current length low (init 0)
lenL = *-1
        BNE l1
        DEC lenH
l1:
        DEC lenL
        BNE cop0
        LDA #$00                ; SMC: current length high (init 0)
lenH = *-1
        BNE cop0
        PLA
        ASL
        BCS dzx0s_new_offset
        JSR dzx0s_elias         ; returns X = lenL with Z/N still valid
        BEQ dzx0s_copy
        INC lenH
dzx0s_copy:
        ; copy X + 256*(lenH-1) bytes (callers preload X and bias lenH):
        ; X counts the partial page (X=0 -> 256), DEC lenH counts full
        ; pages; Y wraps bump both pointer high bytes. A = bit buffer.
        PHA
        LDA ZX0_OUTPUT
        CLC
        ADC #$FF                ; SMC: offset low (init $FF -> offset -1)
offsetL = *-1
        STA COPY_SRC
        LDA ZX0_OUTPUT+1
        ADC #$FF                ; SMC: offset high (init $FF)
offsetH = *-1
        STA COPY_SRC+1
        LDY #$00
copyByte:
        LDA (COPY_SRC),Y
        STA (ZX0_OUTPUT),Y
        INY
        BNE nowrap
        INC COPY_SRC+1
        INC ZX0_OUTPUT+1
nowrap:
        DEX
        BNE copyByte
        DEC lenH
        BNE copyByte
        TYA                     ; Y = count & $FF; add it to the cursor
        CLC
        ADC ZX0_OUTPUT
        STA ZX0_OUTPUT
        BCC copyDone
        INC ZX0_OUTPUT+1
copyDone:
        PLA                     ; lenH is 0 here; lenL is reseeded by elias
        ASL
        BCC dzx0s_literals
dzx0s_new_offset:
        LDX #$FE
        JSR dzx0s_elias_seed    ; returns X = lenL
        INX
        BEQ done                ; elias result 1 -> EOF marker (shared RTS)
        PHA
        TXA
        ROR                     ; C=1 here (elias stop bit), so
        STA offsetH             ; offsetH = $80 | X>>1, C = X bit 0
        JSR get_byte            ; (get_byte keeps C)
        ROR                     ; offsetL = C<<7 | byte>>1, C = byte bit 0
        STA offsetL
        LDX #$00
        STX lenH
        INX
        STX lenL
        PLA
        JSR dzx0s_elias_skip    ; C=1: return at once; C=0: keep reading
        INX                     ; match length = elias + 1: X = lenL+1 and
        INC lenH                ; one extra lenH round (X=0 rolls the +1
                                ; into a full extra page)
        BNE dzx0s_copy          ; always: len <= output size < $FF00 so
                                ; lenH+1 never wraps to 0
dzx0s_elias:
        LDX #$01
dzx0s_elias_seed:
        STX lenL                ; seed the accumulator (1, or $FE for offsets)
        BNE dzx0s_elias_loop    ; always: Z=0 from the caller's LDX #imm
                                ; (STX and JSR both leave the flags alone)
dzx0s_elias_backtrack:
        ASL
        ROL lenL
        ROL lenH
dzx0s_elias_loop:
        ASL
        BNE dzx0s_elias_skip
        JSR get_byte
        ROL                     ; C set by the ASL of the $80 marker
dzx0s_elias_skip:
        BCC dzx0s_elias_backtrack
        LDX lenL                ; return low count in X, Z/N valid past RTS
done:
        RTS
get_byte:
        .byte $AD               ; LDA abs
ZX0_INPUT:
        .word comp_data         ; SMC: stream cursor, starts at comp_data
        INC ZX0_INPUT
        BNE l5
        INC ZX0_INPUT+1
l5:
        RTS
