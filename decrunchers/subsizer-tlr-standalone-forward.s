; ===========================================================================
; Subsizer FORWARD decoder (raw `-r` stream = lzan::subsizer::compress_subsizer
; == crunch_normal_int, BITMODE_SIDEBYTE, no preshift, no prologue).
; Upstream: subsizer format (c) Daniel Kahlin "tlr", BSD-style permissive.
;
; Written from lzan's pure-Rust subsizer reference decoder decrunch_normal_int
; (subsizer.rs), not from an external forward decoder. Reads the compressed
; stream FORWARD from comp_data (byte 0) and writes output FORWARD from
; out_addr; a match copies from (dst - offset).
;
; Stream layout (read forward):
;   byte 0            : endm (side byte, whole byte)
;   then, MSB-first bits:
;     bitsl : 16 parts x 4-bit widths  (match length,  Unary  prefix, floor 1)
;     bits2 : 16 parts x 4-bit widths  (offset len==2, Binary prefix, floor 1)
;     bits3 : 16 parts x 4-bit widths  (offset len==3, Binary prefix, floor 1)
;     bits  : 16 parts x 4-bit widths  (offset len>=4, Binary prefix, floor 1)
;     bits1 :  4 parts x 4-bit widths  (offset len==1, Binary prefix, floor 1)
;   then tokens (bits, interleaved with whole side bytes):
;     read 1 bit: 1 -> literal side byte (whole byte) -> output
;                 0 -> match: len=read_enc(bitsl); if len==endm END;
;                             offs=read_enc(bits1|bits2|bits3|bits by len);
;                             copy len bytes forward from (dst-offs).
;
; Bit IO: MSB-first, lazy one-byte reservoir with a sentinel bit (refill fires
; exactly when a new bit is requested and 8 have been served, matching the
; reference reader's `pos`/`buf` interleaving with side bytes byte-exactly).
;
; Binary offset prefixes are read with the table start folded into the shift:
; the accumulator is SEEDED with tstart>>pbits before the prefix bits are
; rolled in, so (seed<<pbits)|prefix is the final W/L/H index directly
; (bits2 seed 1, bits3 seed 2, bits seed 3, all <<4; bits1 seed 16 <<2).
;
; Calling convention: full_decomp seeds source=comp_data (first byte) and
; dest=out_addr (first byte). comp_data_len is not needed (we decode until the
; end marker). Illegal ops not required.
; ===========================================================================
;@format: subsizer
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: subsizer-forward
;@encoder: lzan::subsizer::compress_subsizer(input)
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 18
;@scratch: symbol=table_base,len=204,align=none
;@illegal: no
;@smc: yes
;@code-bytes: 276

; ---- config-defaults ----
zp_base = $02
table_base = $0334
; ---- end config-defaults ----

; ---- zero page (zp_base-relative, contiguous; lo/hi pairs must stay adjacent) ----
bitbuf   = zp_base+0
dst_lo   = zp_base+1
dst_hi   = zp_base+2
msrc_lo  = zp_base+3
msrc_hi  = zp_base+4
vlo      = zp_base+5
vhi      = zp_base+6
rlo      = zp_base+7
rhi      = zp_base+8
base_lo  = zp_base+9
base_hi  = zp_base+10
len_lo   = zp_base+11
len_hi   = zp_base+12
; +13..+16: spare (freed by size work; kept reserved so the ABI span stays 18)
endm     = zp_base+17

; ---- table block in scratch RAM (filled by the decoder). 3 arrays x 68 bytes,
; 204 bytes total from table_base (default $0334..$03FF).
; Index layout (stream order): bitsl 0..15, bits2 16..31, bits3 32..47,
; bits 48..63, bits1 64..67.
W_arr    = table_base+0   ; part widths        (+0..+67)
L_arr    = table_base+68  ; running base, low  (+68..+135)
H_arr    = table_base+136 ; running base, high (+136..+203)

; ---------------------------------------------------------------------------
; Entry: seed pointers, read endm, build the 5 tables (inline), fall into the
; token loop.
; ---------------------------------------------------------------------------
full_decomp:
        LDA #<comp_data
        STA gb_src
        LDA #>comp_data
        STA gb_src+1
        LDA #<out_addr
        STA dst_lo
        LDA #>out_addr
        STA dst_hi
        LDA #$80
        STA bitbuf            ; sentinel only -> first get_bit refills

        JSR get_byte          ; endm side byte (byte 0)
        STA endm

; build tables: read 68 4-bit part widths and precompute running bases. Base
; resets to floor (=1) at each table boundary (index & $0F == 0).
        LDY #0                ; Y = part index 0..67
bt_lp:
        TYA
        AND #$0F
        BNE bt_nores
        STA base_hi           ; A = 0 here
        LDA #1
        STA base_lo
bt_nores:
        JSR read4             ; A = width 0..15 (preserves Y)
        STA W_arr,Y
        TAX                   ; X = width
        LDA base_lo
        STA L_arr,Y
        LDA base_hi
        STA H_arr,Y
; base += 1 << width: seed C=1 into a cleared pair, ROL it width+1 times
        LDA #0
        STA vlo
        STA vhi
        SEC
        INX
bt_sh:
        ROL vlo
        ROL vhi
        DEX
        BNE bt_sh
        LDA base_lo           ; C = 0 after the last ROL (result <= $8000)
        ADC vlo
        STA base_lo
        LDA base_hi
        ADC vhi
        STA base_hi
        INY
        CPY #68
        BNE bt_lp
; fall through into the token loop (Y = 68; every path below re-seeds Y)

; ---------------------------------------------------------------------------
; token loop
; ---------------------------------------------------------------------------
token_loop:
        JSR get_bit
        BCC tk_match
; literal: whole side byte; store it, then run the copy tail once (X=1,
; len_hi=1) to advance dst
        JSR get_byte
        LDY #0
        STA (dst_lo),Y
        LDX #1
        STX len_hi
        BNE tk_cp1            ; always (LDX #1 set Z=0)

tk_match:
; length: Unary prefix (bitsl, n=16 -> cap 15), then value via read_val
        LDY #0
dl_lp:
        JSR get_bit
        BCS dl_done           ; '1' terminator -> index = Y
        INY
        CPY #15
        BNE dl_lp             ; Y == 15: truncated unary, no terminator bit
dl_done:
        JSR read_val          ; vlo/vhi = length
        LDX #4                ; default Binary prefix width
        LDA vlo
        STA len_lo
        LDY vhi
        STY len_hi
        BNE tk_long           ; len >= 256 -> bits (long); can't be endm
        CMP endm              ; A = len_lo
        BEQ tk_done           ; ---- done ----
        CMP #4
        BCS tk_long
        SBC #0                ; C=0: A = len-1 -> seed 1 (len 2) / 2 (len 3)
        BNE tk_seed
        LDX #2                ; len == 1: bits1, 2 prefix bits
        LDA #16               ; seed 16 -> (16<<2) = 64
        BNE tk_seed           ; always
tk_long:
        LDA #3                ; seed 3 -> (3<<4) = 48
tk_seed:
        JSR read_bits         ; A = (seed<<X)|prefix = table index
        TAY
        JSR read_val          ; vlo/vhi = offset

; msrc = dst - offset
        SEC
        LDA dst_lo
        SBC vlo
        STA msrc_lo
        LDA dst_hi
        SBC vhi
        STA msrc_hi

; copy len (1..) bytes forward; X = low count, len_hi(+1 if X!=0) = page count
        LDY #0
        LDX len_lo
        BEQ tk_cp
        INC len_hi
tk_cp:
        LDA (msrc_lo),Y
        STA (dst_lo),Y
        INC msrc_lo
        BNE tk_cp1
        INC msrc_hi
tk_cp1:
        INC dst_lo
        BNE tk_cp2
        INC dst_hi
tk_cp2:
        DEX
        BNE tk_cp
        DEC len_hi
        BNE tk_cp
        BEQ token_loop        ; always

; ---------------------------------------------------------------------------
; read4: read 4 bits MSB-first -> A. Preserves Y. Falls into read_bits.
; read_bits: A = seed, X = bit count (0 ok). Rolls X stream bits MSB-first
; into rlo/rhi (rlo seeded with A, rhi = 0). Returns A = rlo. Preserves Y.
; ---------------------------------------------------------------------------
read4:
        LDX #4
read_bits0:
        LDA #0
read_bits:
        STA rlo
        LDA #0
        STA rhi
        TXA
        BEQ rb_done
rb_lp:
        JSR get_bit
        ROL rlo
        ROL rhi
        DEX
        BNE rb_lp
rb_done:
        LDA rlo
tk_done:
        RTS

; ---------------------------------------------------------------------------
; read_val: Y = index into W_arr/L_arr/H_arr. vlo/vhi = base[Y] + read(width[Y]).
; ---------------------------------------------------------------------------
read_val:
        LDX W_arr,Y
        JSR read_bits0        ; rlo/rhi = raw bits, A = rlo
        CLC
        ADC L_arr,Y
        STA vlo
        LDA rhi
        ADC H_arr,Y
        STA vhi
        RTS

; ---------------------------------------------------------------------------
; get_bit: next stream bit (MSB-first) -> carry. Preserves X,Y.
; One-byte reservoir with sentinel: refills (lazily) when the sentinel falls
; out, i.e. exactly every 8th requested bit.
; ---------------------------------------------------------------------------
get_bit:
        ASL bitbuf            ; MSB -> carry
        BNE gb_rts
        JSR get_byte          ; C = 1 here: the emptying ASL shifted out the
        ROL                   ; sentinel, so A = (byte<<1)|1, C = byte bit 7
        STA bitbuf
gb_rts:
        RTS

; ---------------------------------------------------------------------------
; get_byte: return the next forward byte in A. Preserves X,Y.
; Self-modifying absolute load, post-increment.
; ---------------------------------------------------------------------------
get_byte:
gb_src = *+1
        LDA $FFFF
        INC gb_src
        BNE gb_nc
        INC gb_src+1
gb_nc:
        RTS
