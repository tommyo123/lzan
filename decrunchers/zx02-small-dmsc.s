; ===========================================================================
; ZX02 6502 decruncher, in asm6502 syntax, for the decrunch-test harness.
; Upstream: zx02 6502 decoder (c) 2022 DMSC, MIT.
;
; This file is assembled in-place by the harness, which prepends an origin
; (*=...) line, defines `out_addr`, and appends a `comp_data:` label plus the
; ZX02 payload via .incbin.
; ===========================================================================
;@format: zx02
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: zx02-small-dmsc
;@encoder: lzan::zx02::compress_zx02
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 10
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 121

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

; Initial values for the de-compressor, copied to ZP at init time:
zx0_ini_block:
        .byte <0, >0                 ; Initial offset - 1.
        .byte $80                    ; Initial bit reservoir. Don't ever change.
        .byte <out_addr, >out_addr   ; Address to place decompressed data.
        .byte <comp_data, >comp_data ; Address of data to decompress.

;--------------------------------------------------
; Decompress ZX0 data (6502 optimized format)
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

; Decode literal: copy next N bytes from compressed file
decode_literal:
        LDY #1
        JSR get_elias
        JSR put_byte
        BCS dzx0s_new_offset

        ; Copy from last offset (repeat N bytes from last offset)
        INY
        JSR get_elias
dzx0s_copy:
        ; C=0 from get_elias
sbc1:
        LDA ZX0_dst+2,X
        SBC offset+2,X
        STA pntr+2,X
        INX
        BNE sbc1

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
get_byte:
        LDA (ZX0_src+2,X)
        INC ZX0_src+2,X
        BNE get_byte_done
        INC ZX0_src+3,X
exit:
get_byte_done:
        RTS

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
put_byte:
        STX setx
ploop:
        LDX setx
        JSR get_byte
        LDX #$FE
        STA (ZX0_dst+2,X)
        INC ZX0_dst
        BNE put_byte_skip
        INC ZX0_dst+1
put_byte_skip:
        DEY
        BNE ploop
        ASL bitr
        RTS
