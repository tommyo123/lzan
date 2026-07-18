; ===========================================================================
; aPLib (apultra) 6502 decruncher, forward, opt-size variant, in asm6502 syntax.
; Upstream: aplib_6502.asm (c) 2019 John Brandwood, Boost Software License 1.0.
;
; Smaller but slower than the standard variant: the bit-buffer refill guard and
; APL_GET_SRC are de-inlined into shared subroutines (one JSR per byte fetch).
; Decodes the same stream as aplib-apultra-brandwood-6502.s.
; ===========================================================================
;@format: aplib
;@direction: forward
;@variant: opt-size
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
;@code-bytes: 237

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
        LDA #<comp_data
        STA apl_srcptr
        LDA #>comp_data
        STA apl_srcptr+1
        LDA #<out_addr
        STA apl_dstptr
        LDA #>out_addr
        STA apl_dstptr+1

apl_decompress:
        LDY #0
        LDA #$80
        STA apl_bitbuf

literal:
        JSR get_src                     ; de-inlined APL_GET_SRC (site 1)

write_byte:
        LDX #0
        STA (apl_dstptr),Y
        INC apl_dstptr
        BNE next_tag
        INC apl_dstptr+1

next_tag:
        JSR get_bit                     ; 0 bbbbbbbb
        BCC literal

skip1:
        JSR get_bit                     ; 1 0 <offset> <length>
        BCC copy_large

        JSR get_bit                     ; 1 1 0 dddddddn
        BCC copy_normal

copy_short:
        LDA #$10
nibble_loop:
        PHA
        JSR get_bit
        PLA
        ROL
        BCC nibble_loop
        BEQ write_byte

        EOR #$FF
        TAY
        INY
        DEC apl_dstptr+1
        LDA (apl_dstptr),Y
        INC apl_dstptr+1
        LDY #0
        BEQ write_byte

copy_normal:
        JSR get_src                     ; de-inlined APL_GET_SRC (site 2)
        LSR
        BEQ finished

        STA apl_offset
        STY apl_offset+1
        TYA
        TAX
        ADC #2
        BNE do_match

get_gamma:
        LDA #1
gamma_loop:
        PHA
        JSR get_bit
        PLA
        ROL
        ROL apl_length+1
        PHA
        JSR get_bit
        PLA
        BCS gamma_loop

finished:
        RTS

copy_large:
        JSR get_gamma
        STY apl_length+1

        CPX #1
        SBC #2
        BCS normal_pair

        JSR get_gamma
        LDX apl_length+1
        BCC do_match

normal_pair:
        STA apl_offset+1

        JSR get_src                     ; de-inlined APL_GET_SRC (site 3)
        STA apl_offset

        JSR get_gamma
        LDX apl_length+1

        LDY apl_offset+1
        BEQ lt256
        CPY #$7D
        BCS match_plus2
        CPY #$05
        BCS match_plus1
        BCC do_match
lt256:
        LDY apl_offset
        BMI do_match

        SEC

match_plus2:
        ADC #1
        BCS match_plus256

match_plus1:
        ADC #0
        BCC do_match

match_plus256:
        INX

do_match:
        EOR #$FF
        TAY
        INY
        BEQ calc_addr
        EOR #$FF

        INX

        CLC
        ADC apl_dstptr
        STA apl_dstptr
        BCS calc_addr
        DEC apl_dstptr+1

calc_addr:
        SEC
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
        DEX
        BNE copy_page

        INX
        JMP next_tag

        ; Shared "get one bit" into carry; refills the bit-buffer when empty.
        ; Fast path: ASL sets C=next bit, buffer nonzero -> RTS (C preserved).
        ; Refill path: JSR get_src (does not touch C) then ROL brings sentinel
        ; C into bit0 and bit7 (data bit) into C. Returns C = extracted bit.
get_bit:
        ASL apl_bitbuf
        BNE get_bit_ret
        JSR get_src
        ROL
        STA apl_bitbuf
get_bit_ret:
        RTS

        ; Shared APL_GET_SRC: A = [srcptr], then srcptr++. Does not touch C, X, Y
        ; (LDA/INC/BNE/INC leave C alone), matching every inline site's needs.
get_src:
        LDA (apl_srcptr),Y
        INC apl_srcptr
        BNE get_src_ret
        INC apl_srcptr+1
get_src_ret:
        RTS
