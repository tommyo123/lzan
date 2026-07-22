; ===========================================================================
; ZX02 6502 decruncher, BACKWARD / in-place, mirrored from zx02-small-dmsc.s.
; Upstream: zx02 6502 decoder (c) 2022 DMSC, MIT.
; Decodes lzan::zx02::compress_zx02_backward output (== `zx02 -b`).
;
; The backward ZX02 stream is the byte-reversed forward stream, and the backward
; plaintext is the byte-reversed forward plaintext. So the forward DMSC decoder
; logic is bit-for-bit identical when we:
;   1. get_byte reads DOWN  (post-decrement ZX0_src / pntr instead of increment)
;   2. put_byte writes DOWN (post-decrement ZX0_dst instead of increment)
;   3. match source = dst + offset + 1 (ADC, C=1) instead of dst - offset - 1
;      (the back-reference lies at a HIGHER address when writing downward)
; Everything else - token parse, interlaced Elias gamma, bit reader, the clever
; X=-2 (ZX0_src+2,X) aliasing that lets get_byte serve both the literal source
; and the match window (pntr) - is unchanged.
;
; Calling convention: full_decomp seeds ZX0_src = comp_data+comp_data_len-1 and
; ZX0_dst = out_addr+out_len-1 (the LAST bytes), then falls into the decoder.
; `comp_data_len` and `out_len` are injected by the harness. On exit ZX0_dst =
; out_addr-1 (output fills [out_addr, out_addr+out_len)).
; ===========================================================================
;@format: zx02
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: zx02-backward
;@encoder: lzan::zx02::compress_zx02_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 10
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 128

; $F6-$FF: only $F6 (KEYTAB high) is live KERNAL state, re-derived by the
; keyboard scan; the rest is RS-232 pointers and free bytes.
; ---- config-defaults ----
zp_base = $F6
; ---- end config-defaults ----

offset   = zp_base+0
bitr     = zp_base+2
ZX0_dst  = zp_base+3
ZX0_src  = zp_base+5
pntr     = zp_base+7
setx     = zp_base+9

; Initial values, copied to ZP at init time. Same layout as forward; only the
; dst/src pointers are seeded to the END of their regions (backward start).
zx0_ini_block:
        .byte <0, >0                 ; Initial offset - 1.
        .byte $80                    ; Initial bit reservoir. Don't ever change.
        .byte <(out_addr+out_len-1), >(out_addr+out_len-1)       ; last output byte
        .byte <(comp_data+comp_data_len-1), >(comp_data+comp_data_len-1) ; last input byte

;--------------------------------------------------
full_decomp:
        ; Get initialization block
        LDX #6
copy_init:
        LDA zx0_ini_block,X
        STA offset,X
        DEX
        BPL copy_init

        ; Init: X = -2
        DEX

; Decode literal: copy next N bytes from compressed file (downward)
decode_literal:
        LDY #1
        JSR get_elias
        JSR put_byte
        BCS dzx0s_new_offset

        ; Copy from last offset (repeat N bytes from last offset)
        INY
        JSR get_elias
dzx0s_copy:
        ; pntr = ZX0_dst + offset + 1  (backward back-reference is at HIGHER addr)
        SEC                       ; C=1 gives the +1
adc1:
        LDA ZX0_dst+2,X
        ADC offset+2,X
        STA pntr+2,X
        INX
        BNE adc1

        JSR put_byte
        BCC decode_literal

; Copy from new offset (repeat N bytes from new offset)
dzx0s_new_offset:
        ; Read elias code for high part of offset
        INY
        JSR get_elias
        BEQ exit          ; Read a 0, signals the end
        ; Decrease and divide by 2
        DEY
        TYA
        LSR
        STA offset+1

        ; Get low part of offset, a literal 7 bits
        JSR get_byte

        ; Divide by 2
        ROR
        STA offset

        ; And get the copy length.
        ; Start elias reading with the bit already in carry:
        LDY #1
        JSR elias_skip1

        INY
        BCC dzx0s_copy

; Read an elias-gamma interlaced code.
; ------------------------------------
elias_loop:
        ; Read next data bit to result
        ASL bitr
        ROL
        TAY

get_elias:
        ; Get one bit
        ASL bitr
        BNE elias_skip1

        ; Read new bit from stream
        JSR get_byte
        ROL
        STA bitr

elias_skip1:
        TYA
        BCS elias_loop
        ; Got ending bit, stop reading
        RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; get_byte - read one byte at (ZX0_src+2,X) then POST-DECREMENT that 16-bit
; pointer (X=-2 -> ZX0_src literal source; X=0 -> pntr match window). Must
; preserve A (the fetched byte) AND carry (recycled by ROL/ROR in the elias /
; offset math), so we PHA the byte, test the low pointer byte for a page
; borrow, decrement, then PLA. DEC/PHA/PLA/LDA leave carry untouched.
get_byte:
        LDA (ZX0_src+2,X)
        PHA
        LDA ZX0_src+2,X
        BNE gb_nolo
        DEC ZX0_src+3,X
gb_nolo:
        DEC ZX0_src+2,X
        PLA
get_byte_done:
exit:
        RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; put_byte - copy Y bytes downward. Source via get_byte (X=setx selects
; ZX0_src or pntr); destination is (ZX0_dst), POST-DECREMENTED each byte.
put_byte:
        STX setx
ploop:
        LDX setx
        JSR get_byte
        LDX #$FE
        STA (ZX0_dst+2,X)
        ; decrement ZX0_dst (A already stored, free to clobber)
        LDA ZX0_dst
        BNE pb_nolo
        DEC ZX0_dst+1
pb_nolo:
        DEC ZX0_dst
        DEY
        BNE ploop
        ASL bitr
        RTS