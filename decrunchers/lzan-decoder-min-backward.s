; ===========================================================================
; LZAN "ZX" 6510 minimal decoder (rep0-only, mode 0x41, EOF-terminated),
; BACKWARD / in-place variant. This is lzan's own 6510 decoder; there is no
; external upstream.
;
; Same three direction aspects as the full backward decoder:
;   1. src reads DOWN  : the stream pointer steps downward.
;   2. dst writes DOWN : the copy loop walks both pointers downward.
;   3. match source    : mptr = dst + moff + 1 (back-ref is at a HIGHER
;                        address; moff is stored as off-1, the +1 rides in
;                        on the carry, which is proven set - see below).
;
; The bit reader (gbit), interlaced Elias-gamma (read_gamma/rg_entry) and the
; EOF marker test are logically identical to the forward decoder: the encoder
; emits [mode] ++ reverse(payload) (zx::compress_min_eof_backward), so a
; descending byte reader reproduces the forward bit sequence exactly.
; Termination is the in-stream EOF marker (offset-MSB gamma >= 256);
; comp_data_len / out_len are needed only to seed the END pointers.
;
; Implementation notes:
;   * PRE-decrement convention: src/dst are seeded one PAST the last byte
;     (comp_data+comp_data_len, out_addr+out_len) and every access first
;     steps the pointer down, then reads/writes through it. This makes
;     "step + load" a single shareable routine.
;   * getb: shared "16-bit decrement of the zp pointer at zp_base+X, then
;     LDA (zp_base,X)" - used by the bit refill, the offset-LSB fetch and
;     both copy-run sources. fetch = LDX #1 falling into getb.
;   * copy_run takes its SOURCE pointer as an X selector: X=1 copies the
;     literal run straight from the stream (src), X=9 copies the match
;     (mptr). No mptr<->src shuttling around the literal run. read_gamma
;     provably returns X=1 (rg_entry's INX; a gbit refill re-sets X=1),
;     so the literal call site pays no LDX.
;   * init is a 7-byte table copied by one loop (bitbuf/moff zeroed, src/dst
;     seeded); the final A=0 doubles as TAY for the permanent Y=0 invariant
;     (Y is used only by STA (dst),Y - nothing else writes Y).
;   * carry is set on every return from read_gamma/rg_entry (gamma ends on
;     a 1 control bit; the immediate-BCS path enters with C=1), so domatch
;     needs no SEC and do_rep0 falls straight in.
;   * dispatch: after-literals falls into do_rep0 on 0 / branches on 1;
;     after-match falls into st_newoffset on 1 / branches on 0. The EOF
;     exit tail-merges into read_gamma's RTS, and copy_run falls into
;     gbit (both call sites read the after-run flag right away).
; ===========================================================================
;@format: lzan-min
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzan-decoder-min-backward
;@encoder: lzan::zx::compress_min_eof_backward(i)[1..].to_vec()
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 11
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 156

; ---- config-defaults ----
zp_base = $F1
; ---- end config-defaults ----

; zp layout: offsets 0..6 are the init-table block (bitbuf, src, moff, dst);
; src sits at offset 1 so read_gamma's exit X=1 selects it for free.
bitbuf = zp_base+0  ; current bit byte (MSB first, guard-bit sentinel)
src    = zp_base+1  ; bitstream pointer (lo/hi), pre-decremented, reads DOWN
moff   = zp_base+3  ; 2 bytes: current offset (rep0), stored as off-1
dst    = zp_base+5  ; output pointer (lo/hi), pre-decremented, writes DOWN
val    = zp_base+7  ; 2 bytes: gamma result / copy length
mptr   = zp_base+9  ; 2 bytes: match copy source pointer

full_decomp:
        LDX #6                    ; copy 7-byte init block into zp_base+0..6:
fd_init:
        LDA inittab,X             ; bitbuf=0, src=comp_data+comp_data_len,
        STA zp_base,X             ; moff=0 (rep0 = off-1 = 0, i.e. off=1),
        DEX                       ; dst=out_addr+out_len
        BPL fd_init
        TAY                       ; A = inittab[0] = 0 -> Y=0 for STA (dst),Y
        ; fall into st_literals (first unit is always a literal run)

st_literals:
        JSR read_gamma            ; val = len; exits with X=1 (= src selector)
        JSR copy_run              ; copy val stream bytes -> (dst), both DOWN;
                                  ; falls into gbit: C = after-literals flag
        BCS st_newoffset          ; 1 = new offset, 0 = rep0
do_rep0:
        JSR read_gamma            ; val = len (returns C=1)
domatch:
        LDA dst                   ; BACKWARD: match source is ABOVE dst.
        ADC moff                  ; mptr = dst + moff + 1 = dst + off (C=1
        STA mptr                  ; on every path in - see header)
        LDA dst+1
        ADC moff+1
        STA mptr+1
        LDX #9                    ; mptr selector
        JSR copy_run              ; falls into gbit: C = after-match flag
after_match:
        BCC st_literals           ; 0 = literal run
        ; 1 = another new offset: fall into st_newoffset

st_newoffset:
        JSR read_gamma            ; val = offset MSB gamma
        LDA val+1
        BNE rg_done               ; msb >= 256 -> EOF (shared RTS tail)
        LDX val                   ; A = msb-1
        DEX
        TXA
        LSR                       ; A = (msb-1)>>1; carry = (msb-1)&1
        STA moff+1
        JSR fetch                 ; A = lsb byte; src stepped DOWN (C kept)
        ROR                       ; A = (off-1)_lo; C = backtracked ctrl bit
        STA moff
        JSR rg_entry              ; gamma(len-1) with ctrl bit in carry
        INC val                   ; len = (len-1)+1; 16-bit result is never
        BNE domatch               ; 0, so one of the BNEs always takes
        INC val+1
        BNE domatch

; copy_run: copy val (16-bit, >=1) bytes from the pointer selected by X
; (X=1: src, X=9: mptr) to (dst), all pointers pre-decremented DOWNWARD.
; getb preserves X, so the selector survives the loop. Y=0 throughout.
; Falls into gbit on exit: both call sites need the next stream bit (the
; after-run flag), so the run's RTS is gbit's (tail merge).
copy_run:
cr_loop:
        LDA dst                   ; dst-- (16-bit, DOWN)
        BNE cr_ds
        DEC dst+1
cr_ds:
        DEC dst
        JSR getb                  ; source--, A = source byte
        STA (dst),Y
        LDA val                   ; val-- (16-bit copy counter)
        BNE cr_vs
        DEC val+1
cr_vs:
        DEC val
        BNE cr_loop
        LDA val+1
        BNE cr_loop
        ; fall into gbit: C = the flag bit that follows the run

; gbit: next stream bit -> carry (MSB first, guard-bit sentinel refill).
gbit:
        ASL bitbuf                ; carry = next data bit
        BNE gb_have
        JSR fetch                 ; refill: A = next byte, src stepped DOWN
        SEC
        ROL                       ; C = b7; bit0 = 1 (guard)
        STA bitbuf
gb_have:
        RTS

; fetch: A = next stream byte, src stepped DOWN. Preserves carry and Y.
; getb: 16-bit decrement of the zp pointer at zp_base+X, then load through
; it. Preserves X, Y and carry.
fetch:
        LDX #1                    ; src selector; falls into getb
getb:
        LDA zp_base,X
        BNE gd_dec
        DEC zp_base+1,X
gd_dec:
        DEC zp_base,X
        LDA (zp_base,X)
        RTS

; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }.
; Returns C=1 (the terminating control bit) and X=1 on every path.
read_gamma:
        JSR gbit                  ; C = first control bit
rg_entry:
        LDX #0                    ; val = 1
        STX val+1
        INX
        STX val
        BCS rg_done
rg_data:
        JSR gbit                  ; data bit -> carry
        ROL val
        ROL val+1
        JSR gbit                  ; next control bit -> carry
        BCC rg_data
rg_done:
        RTS

inittab:
        .byte 0                                                   ; bitbuf
        .byte <(comp_data + comp_data_len), >(comp_data + comp_data_len) ; src
        .byte 0, 0                                                ; moff
        .byte <(out_addr + out_len), >(out_addr + out_len)        ; dst
