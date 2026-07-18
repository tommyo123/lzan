; ===========================================================================
; aPLib (apultra) 6502 decruncher, backward / in-place, in asm6502 syntax.
; Upstream: aplib_6502_b.asm (c) 2020 Emmanuel Marty (parts after John Brandwood
; and Peter Ferrie), zlib. For data made with `apultra -b` ==
; lzan::apultra::compress_apultra_backward.
;
; Backward in-place decoder: src and dst are the last bytes of the compressed
; block and the output buffer; both pointers decrement; a match offset is ADDED
; to dst (the back-reference lies at higher addresses). This lets the packed and
; unpacked regions overlap (write head trails the read head), so a file can be
; decrunched over itself.
;
; Structure:
;   * apl_srcptr uses PRE-decrement (seeded one past the last byte, decremented
;     before each load) so the shared get_src subroutine can test the low byte
;     in A before the fetch and thus preserve X (the 'follows literal' flag).
;     The sequence of addresses read is identical to the original.
;   * One get_src subroutine replaces the three inline APL_GET_SRC expansions
;     and the source fetch inside the bit reload; it preserves C (needed by the
;     ROL in get_bit) and X.
;   * One A-preserving get_bit subroutine (PHA/PLA around the reload) serves
;     all six bit-fetch sites (former APL_GET_BIT / APL_GET_BIT_SAVEA).
;   * One put_dst subroutine replaces both APL_PUT_DST expansions.
;   * ZP pointers + bit queue are seeded from a .byte table via a copy loop
;     (apl_bitbuf/apl_srcptr/apl_dstptr made contiguous for this).
;   * `full_decomp` seeds apl_srcptr = comp_data+comp_data_len (one PAST the
;     last byte; pre-decrement) and apl_dstptr = out_addr+out_len-1 (the last
;     byte). `comp_data_len` and `out_len` are supplied by the harness.
; ===========================================================================
;@format: aplib
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: aplib-apultra-backward
;@encoder: lzan::apultra::compress_apultra_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 10
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 216

; ---- config-defaults ----
zp_base = $F6
; ---- end config-defaults ----

apl_gamma2_hi = zp_base+0
apl_offset    = zp_base+1  ; word
apl_winptr    = zp_base+3  ; word
apl_bitbuf    = zp_base+5  ; contiguous with the pointers: seeded from init_tab
apl_srcptr    = zp_base+6  ; word (pre-decrement: points one PAST the next byte)
apl_dstptr    = zp_base+8  ; word (post-decrement: points AT the next byte)

full_decomp:
        LDX #$04                        ; seed bitbuf ($80) + srcptr + dstptr
init_loop:
        LDA init_tab,X
        STA apl_bitbuf,X
        DEX
        BPL init_loop
        LDY #$00
        ; falls into copy_literal: a stream always starts with a raw literal

copy_literal:
        JSR get_src
write_literal:
        JSR put_dst
        LDX #$00                        ; clear 'follows match' flag
next_token:
        JSR get_bit
        BCC copy_literal                ; 0 -> literal

        JSR get_bit
        BCC long_match                  ; 10x -> long 8+n match

        JSR get_bit
        BCS short_match                 ; 111 -> 4-bit offset 1-byte copy

        JSR get_src                     ; 110 -> 7-bit offset, 2/3-byte match
        LSR                             ; offset into place, len bit into carry
        BEQ done                        ; EOD
        STA apl_offset
        STY apl_offset+1
        TYA
        STY apl_gamma2_hi
        ADC #$02                        ; len = 2 or 3
        BNE got_len                     ; always

long_match:
        JSR get_gamma2                  ; gamma2 high offset bits in A
        STY apl_gamma2_hi
        CPX #$01                        ; carry if following literal
        SBC #$02
        BCS no_repmatch
        JSR get_gamma2                  ; repmatch length low in A
        BCC got_len                     ; always (gamma2 exits with C=0)

short_match:
        LDA #$10
read_short_offs:
        JSR get_bit
        ROL
        BCC read_short_offs
        BEQ write_literal               ; offset 0 -> write a 0
        TAY
        LDA (apl_dstptr),Y
        LDY #$00
        BEQ write_literal               ; always

get_gamma2:
        LDA #$01
gamma2_loop:
        JSR get_bit
        ROL
        ROL apl_gamma2_hi
        JSR get_bit
        BCS gamma2_loop
done:
        RTS

no_repmatch:
        STA apl_offset+1
        JSR get_src
        STA apl_offset
        JSR get_gamma2                  ; match length low in A
        LDX apl_offset+1
        BEQ offset_1byte
        CPX #$7d                        ; offset >= 32000 ?
        BCS offset_incby2
        CPX #$05                        ; offset >= 1280 ?
        BCS offset_incby1
        BCC got_len                     ; always
offset_1byte:
        LDX apl_offset                  ; offset < 128 ?
        BMI got_len
        SEC
offset_incby2:
        ADC #$01
        BCS len_inchi
offset_incby1:
        ADC #$00
        BCC got_len                     ; always
len_inchi:
        INC apl_gamma2_hi
got_len:
        TAX
        BEQ add_offset
        INC apl_gamma2_hi
add_offset:
        CLC                             ; back-ref = dst + offset (backward!)
        LDA apl_dstptr
        ADC apl_offset
        STA apl_winptr
        LDA apl_dstptr+1
        ADC apl_offset+1
        STA apl_winptr+1
copy_match_loop:
        LDA (apl_winptr),Y
        JSR put_dst
        LDA apl_winptr                  ; decrement back-ref address
        BNE backref_page_done
        DEC apl_winptr+1
backref_page_done:
        DEC apl_winptr
        DEX
        BNE copy_match_loop
        DEC apl_gamma2_hi
        BNE copy_match_loop
        INX                             ; set 'follows match' flag
        JMP next_token

get_bit:                                ; C = next bit; preserves A and X
        ASL apl_bitbuf
        BNE gb_have
        PHA
        JSR get_src                     ; preserves C (the roll-in guard bit)
        ROL
        STA apl_bitbuf
        PLA
gb_have:
        RTS

get_src:                                ; A = next source byte; preserves C, X
        LDA apl_srcptr
        BNE src_page_done
        DEC apl_srcptr+1
src_page_done:
        DEC apl_srcptr
        LDA (apl_srcptr),Y
        RTS

put_dst:                                ; store A at dst, step dst down
        STA (apl_dstptr),Y
        LDA apl_dstptr
        BNE dst_page_done
        DEC apl_dstptr+1
dst_page_done:
        DEC apl_dstptr
        RTS

init_tab:
        .byte $80                       ; empty bit queue + roll-in guard bit
        .byte <(comp_data + comp_data_len), >(comp_data + comp_data_len)
        .byte <(out_addr + out_len - 1), >(out_addr + out_len - 1)
