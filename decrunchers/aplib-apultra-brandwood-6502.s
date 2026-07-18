; ===========================================================================
; aPLib (apultra) 6502 decruncher, forward, in asm6502 syntax.
; Upstream: aplib_6502.asm (c) 2019 John Brandwood, Boost Software License 1.0.
; Standard-format build (enhanced disabled).
;
; Structure:
;   * The APL_GET_SRC expansions are factored into one `get_byte` subroutine
;     (flags on return reflect the pointer INC; C is preserved).
;   * The inline `ASL apl_bitbuf / BNE / JSR load_bit` sequences are factored
;     into one `get_bit` subroutine that preserves A/X/Y and returns the data
;     bit in C (the old `load_bit` reload is its refill path).
;   * The match-length adjustment ladder is folded into a single `ADC #1`
;     entered with C=1 for the +2 classes (offset < 128 or >= 32000) and C=0
;     for the +1 class (1280 <= offset < 32000); C comes straight from the
;     CPY #$7D class compare, so both match_plus paths merge.
;   * Pointer seeding at full_decomp is a 4-byte table copied by a loop.
;   * Standard-format only: the enhanced (>32000 offset) extension path is left
;     in (it is harmless when those offsets never occur in 16-bit space).
;
; Calling convention: apl_srcptr ($FC) = compressed source ptr, apl_dstptr
; ($FE) = output ptr; the harness pokes those. Entry = full_decomp (a thin
; alias seeding the ZP pointers, then apl_decompress).
; EOF = offset 0 in copy_normal -> finished -> RTS.
; ===========================================================================
;@format: aplib
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: aplib-apultra-brandwood
;@encoder: lzan::apultra::compress_apultra
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 9
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 225

; ---- config-defaults ----
zp_base = $F7
; ---- end config-defaults ----

apl_bitbuf = zp_base+0  ; 1 byte
apl_offset = zp_base+1  ; 1 word
apl_winptr = zp_base+3  ; 1 word
apl_srcptr = zp_base+5  ; 1 word
apl_dstptr = zp_base+7  ; 1 word
apl_length = apl_winptr

full_decomp:
        LDX #3                          ; Seed apl_srcptr/apl_dstptr (they are
init_loop:
        LDA init_tab,X                  ; adjacent in ZP) from the table.
        STA apl_srcptr,X
        DEX
        BPL init_loop
        ; fall through into apl_decompress (X is re-seeded at write_byte)

apl_decompress:
        LDY #0                          ; Initialize source index.
        LDA #$80                        ; Initialize an empty bit-buffer.
        STA apl_bitbuf

        ; 0 bbbbbbbb - One byte from compressed data, i.e. a "literal".
literal:
        JSR get_byte

write_byte:
        LDX #0                          ; LWM=0.
        STA (apl_dstptr),Y              ; Write the byte directly to the output.
        INC apl_dstptr
        BNE next_tag
        INC apl_dstptr+1

next_tag:
        JSR get_bit                     ; 0 bbbbbbbb
        BCC literal

        JSR get_bit                     ; 1 0 <offset> <length>
        BCC copy_large

        JSR get_bit                     ; 1 1 0 dddddddn
        BCC copy_normal

        ; 1 1 1 dddd - Copy 1 byte within 15 bytes (or zero).
copy_short:
        LDA #$10
nibble_loop:
        JSR get_bit
        ROL
        BCC nibble_loop
        BEQ write_byte                  ; Offset=0 means write zero.

        EOR #$FF                        ; Read the byte directly from the
        TAY                             ; destination window.
        INY
        DEC apl_dstptr+1
        LDA (apl_dstptr),Y
        INC apl_dstptr+1
        LDY #0
        BEQ write_byte

        ; 1 1 0 dddddddn - Copy 2 or 3 within 128 bytes.
copy_normal:
        JSR get_byte
        LSR
        BEQ finished                    ; Offset 0 == EOF.

        STA apl_offset                  ; Preserve offset.
        STY apl_offset+1
        TYA                             ; Y == 0.
        TAX                             ; Bits 8..15 of length.
        ADC #2                          ; Bits 0...7 of length (C from LSR).
        BNE do_match                    ; NZ from previous ADC.

        ; Subroutine: get a gamma-coded value.
get_gamma:
        LDA #1
gamma_loop:
        JSR get_bit
        ROL
        ROL apl_length+1
        JSR get_bit
        BCS gamma_loop

finished:
        RTS                             ; All decompressed!

        ; 1 0 <offset> <length> - gamma-coded LZSS pair.
copy_large:
        JSR get_gamma                   ; Bits 8..15 of offset (min 2).
        STY apl_length+1                ; Clear hi-byte of length.

        CPX #1                          ; CC if LWM==0, CS if LWM==1.
        SBC #2                          ; -3 if LWM==0, -2 if LWM==1.
        BCS normal_pair                 ; CC if LWM==0 && offset==2.

        JSR get_gamma                   ; Get length (A=lo-byte & CC).
        LDX apl_length+1
        BCC do_match                    ; Use previous Offset.

normal_pair:
        STA apl_offset+1                ; Save bits 8..15 of offset.
        JSR get_byte
        STA apl_offset                  ; Save bits 0...7 of offset.

        JSR get_gamma                   ; Get length (A=lo-byte & CC).
        LDX apl_length+1

        LDY apl_offset+1                ; If offset >= 256 ...
        BNE ge256
        LDY apl_offset                  ; If offset >= 128, no adjustment.
        BMI do_match
        SEC                             ; Offset < 128: length += 2 (C=1).
        BCS match_plus
ge256:
        CPY #$05                        ; If offset < 1280, no adjustment.
        BCC do_match
        CPY #$7D                        ; C=1 iff offset >= 32000: +2, else +1.
match_plus:
        ADC #1                          ; +1, or +2 when C was set.
        BCC do_match
        INX                             ; Length overflowed into the hi-byte.

do_match:
        EOR #$FF                        ; Negate the lo-byte of length
        TAY                             ; and check for zero.
        INY
        BEQ calc_addr
        EOR #$FF

        INX                             ; Increment # of pages to copy.

        CLC                             ; Calc destination for partial page.
        ADC apl_dstptr
        STA apl_dstptr
        BCS calc_addr
        DEC apl_dstptr+1

calc_addr:
        SEC                             ; Calc address of match.
        LDA apl_dstptr
        SBC apl_offset
        STA apl_winptr
        LDA apl_dstptr+1
        SBC apl_offset+1
        STA apl_winptr+1

copy_page:
        LDA (apl_winptr),Y
        STA (apl_dstptr),Y
        INY
        BNE copy_page
        INC apl_winptr+1
        INC apl_dstptr+1
        DEX                             ; Any full pages left to copy?
        BNE copy_page

        INX                             ; LWM=1.
        JMP next_tag

        ; Subroutine: fetch the next bit of the bitstream into C.
        ; Preserves A/X/Y; refills the bit-buffer when it runs empty.
get_bit:
        ASL apl_bitbuf
        BNE bit_done
        PHA
        JSR get_byte
        ROL
        STA apl_bitbuf
        PLA
bit_done:
        RTS

        ; Subroutine: fetch the next raw byte from the compressed source.
        ; Preserves X/Y and C; flags on return reflect the pointer INC.
get_byte:
        LDA (apl_srcptr),Y
        INC apl_srcptr
        BNE byte_done
        INC apl_srcptr+1
byte_done:
        RTS

init_tab:
        .byte <comp_data, >comp_data, <out_addr, >out_addr
