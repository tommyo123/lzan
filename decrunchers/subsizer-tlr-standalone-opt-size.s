; ===========================================================================
; Subsizer 0.6 standalone (memory-mode) 6502 decruncher, opt-size variant, in
; asm6502 syntax.
; Upstream: subsizer standalone/decrunch_normal.asm (c) Daniel Kahlin "tlr",
; BSD-style permissive license.
;
; Smaller but slower than the standard variant: entry fall-through, and all six
; bit-reader refill sites de-inlined into one get_bit subroutine. Decodes the
; same stream.
;
; This pairs with the native `subsizer -m` (memory) crunch mode ==
; lzan::subsizer::compress_subsizer_marker_at. It decodes the
; marker-bit-reservoir (BITMODE_PRESHIFT) bitstream, reads a 4-byte prologue,
; and writes its output BACKWARD from a dest pointer.
;
; The tables live in zeroed scratch RAM at $0334 (N_PARTS=16, HAVE_LONG_PARTS=1):
;        base_l/base_len = $0334  (low-byte base array)
;        base_offs_l     = $0344
;        base_h          = $0368  (= base_offs_l+52-16, overlapping, by design)
;        base_offs_h     = $0378
;        bits/bits_len   = $03AC
;        bits_offs       = $03BC
; The `tabb` bytes are precomputed offset-class selector constants:
;        %10000000 | (48>>2) = $80|12 = $8C
;        %11100000 | (0 >>4) = $E0|0  = $E0
;        %11100000 | (16>>4) = $E0|1  = $E1
;        %11100000 | (32>>4) = $E0|2  = $E2
;
; Entry `subsizer_decrunch` seeds dc_ptr to comp_data+comp_data_len (one past
; the last stream byte; dc_get_byte pre-decrements), then JSR decrunch.
; `comp_data_len` is supplied by the test as a constant.
; ===========================================================================
;@format: subsizer
;@direction: backward
;@variant: opt-size
;@entry: subsizer_decrunch
;@vfy-key: subsizer-tlr-standalone
;@encoder: lzan::subsizer::compress_subsizer_marker_at(input, out_addr + input.len())
;@payload: dst-in-stream
;@eof: stream
;@needs: comp_data,comp_data_len
;@zp-len: 8
;@scratch: symbol=table_base,len=188,align=none
;@illegal: no
;@smc: yes
;@code-bytes: 242

; ---- config-defaults ----
zp_base = $f8
table_base = $0334
; ---- end config-defaults ----

; ---- zero page (verbatim $f8.. layout from the source, now zp_base-relative) ----
len_zp    = zp_base+0
copy_zp   = zp_base+1    ; word (+1/+2)
hibits_zp = zp_base+3
buf_zp    = zp_base+4    ; +4..+7 (buf,dest lo/hi,endm) loaded via `STA buf_zp-1,X`
dest_zp   = zp_base+5    ; word (+5/+6)
endm_zp   = zp_base+7

N_PARTS   = 16
PART_MASK = $0f

; ---- table block in zeroed scratch RAM at table_base (filled by the decoder),
;      188 bytes, verbatim DASM layout (default $0334..$03EF) ----
base_l       = table_base+$00
base_len     = table_base+$00
base_offs_l  = table_base+$10
base_h       = table_base+$34
base_offs_h  = table_base+$44
bits         = table_base+$78
bits_len     = table_base+$78
bits_offs    = table_base+$88

; ---------------------------------------------------------------------------
; Entry: seed the backward stream pointer, then decrunch. The compressed
; payload is at comp_data (length comp_data_len, supplied by the harness);
; dc_get_byte walks it from the end downward.
; ---------------------------------------------------------------------------
; (entry subsizer_decrunch relocated just before `decrunch` to fall through,
;  saving the JMP)

; ---------------------------------------------------------------------------
; dc_get_byte: return the next (backward) input byte in A, preserving X,Y,C.
; Pre-decrements a 16-bit pointer then loads. (Port of the sample reader.)
; ---------------------------------------------------------------------------
dc_get_byte:
        LDA dc_ptr
        BNE dcgb_skp1
        DEC dc_ptr+1
dcgb_skp1:
        DEC dc_ptr
dc_ptr = * + 1
        LDA >$0000               ; force-absolute (3-byte) self-modifying load (DASM `lda.w`)
        RTS

; ---------------------------------------------------------------------------
; get_bit: read one bitstream bit into C, preserving A,X,Y. Shared factoring
; of the 6 hand-inlined bit-reader refill sites (opt-size: slower via JSR).
; ---------------------------------------------------------------------------
get_bit:
        ASL buf_zp
        BNE gb_skp
        PHA
        JSR dc_get_byte
        ROL
        STA buf_zp
        PLA
gb_skp:
        RTS

; ---------------------------------------------------------------------------
; Entry: seed the backward stream pointer, then fall into decrunch.
; ---------------------------------------------------------------------------
subsizer_decrunch:
        LDA #<(comp_data + comp_data_len)
        STA dc_ptr
        LDA #>(comp_data + comp_data_len)
        STA dc_ptr+1

; ---------------------------------------------------------------------------
; decrunch
; ---------------------------------------------------------------------------
decrunch:
        LDX #4
; Get dest_zp, endm_zp and buf_zp (4 prologue bytes, read backward).
dc_lp00:
        JSR dc_get_byte
        STA buf_zp-1,X
        DEX
        BNE dc_lp00
; X = 0

dc_lp01:
; get 4 bits (inline of the dcg loop already present in source)
        LDA #$E0                ; %11100000
dcg_lp1:
        JSR get_bit
        ROL
        BCS dcg_lp1
; Acc = 4 bits.

        STA bits,X

        TXA
        AND #PART_MASK
        TAY
        BEQ dc_skp01

        LDA #0
        STA hibits_zp
        LDY bits-1,X
        SEC
dc_lp02:
        ROL
        ROL hibits_zp
        DEY
        BPL dc_lp02
; C = 0
        ADC base_l-1,X
        TAY
        LDA hibits_zp
        ADC base_h-1,X

dc_skp01:
        STA base_h,X
        TYA
        STA base_l,X
        INX
        CPX #N_PARTS*4+4
        BNE dc_lp01

; perform decrunch
        LDY #0
        BEQ decrunch_entry      ; always taken

; ---------------------------------------------------------------------------
; single literal byte
; ---------------------------------------------------------------------------
dc_literal:
        LDA dest_zp
        BNE dc_skp5
        DEC dest_zp+1
dc_skp5:
        DEC dest_zp
        JSR dc_get_byte
dc_common:
        STA (dest_zp),Y
        ; fall through

decrunch_entry:
; perform actual decrunch
dc_lp1:
; get_bit (inline #1)
        JSR get_bit
        BCS dc_literal

; get length as bits/base.
        LDX #$80-N_PARTS
dc_lp2:
        INX
        BMI dc_skp0
; get_bit (inline #2)
        JSR get_bit
        BCC dc_lp2
        CLC
dc_skp0:
; C = 0, Y = 0
        TYA
        LDY bits_len-$80+N_PARTS-1,X
        BEQ dcb1_skp2
; get_bits_max8 (inline #3)
gb3_lp1:
        JSR get_bit
        ROL
        DEY
        BNE gb3_lp1
dcb1_skp2:
; C = 0
        ADC base_len-$80+N_PARTS-1,X
        STA len_zp
; C = 0

; IN: len = $01..$100 (Acc = $00..$ff); OUT: dest_zp -= len, X = len-1
        TAX
        EOR #$ff
        ADC dest_zp
        STA dest_zp
        BCS dc_skp22
        DEC dest_zp+1
dc_skp22:

; check end marker
        CPX endm_zp
        BEQ done

; Get selector bits depending on length.
        CPX #4
        BCC dc_skp2
        LDX #3
dc_skp2:

; get offset as bits/base.
        LDA tabb,X
; get_bits_max8_masked (inline #4)
gb4_lp1:
        JSR get_bit
        ROL
        BCS gb4_lp1
        TAX
; C = 0

        LDA #0
        STA hibits_zp
        LDY bits_offs,X
        BEQ dcb3_skp2
; get_bits_max16 (inline #5)
gb5_lp1:
        JSR get_bit
        ROL
        ROL hibits_zp
        DEY
        BNE gb5_lp1             ; C=0 for all Y!=0
dcb3_skp2:
; C = 0, Acc/hibits_zp + base_offs,x = offset - 1

; copy_zp = Acc/hibits_zp + base_offs,x + 1 + dest_zp = dest_zp + offset
        ADC base_offs_l,X
        BCC dcb3_skp3
        INC hibits_zp
dcb3_skp3:
        SEC
        ADC dest_zp
        STA copy_zp
        LDA hibits_zp
        ADC base_offs_h,X
; C = 0
        ADC dest_zp+1
        STA copy_zp+1

; Reverse fast copy.  IN: len_zp = $00..$ff, C = 0
copy:
        LDY len_zp
        BEQ dc_skp4
dc_lp4:
        LDA (copy_zp),Y
        STA (dest_zp),Y
        DEY
        BNE dc_lp4
dc_skp4:
        LDA (copy_zp),Y
        JMP dc_common

; exit out
done:
        RTS

; static offset selector table (HAVE_LONG_PARTS=1), constants precomputed.
tabb:
        .byte $8c                ; %10000000 | (48>>2)  -> 2 bits
        .byte $e0                ; %11100000 | (0 >>4)  -> 4 bits
        .byte $e1                ; %11100000 | (16>>4)  -> 4 bits
        .byte $e2                ; %11100000 | (32>>4)  -> 4 bits
