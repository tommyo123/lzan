; ===========================================================================
; LZAN "ZX" 6510 minimal decoder (rep0-only, mode 0x41, EOF-terminated), in
; asm6502 syntax for the decrunch-test harness. This is lzan's own 6510 decoder;
; there is no external upstream.
;
; Implementation notes:
;   * ZP pointer/state seeding is a 7-byte table copied by a loop (bitbuf
;     seeds to $80 = "empty" with the guard convention below).
;   * bitbuf init $80 makes the first refill arrive with carry already set,
;     so the refill's SEC is dropped (the guard bit supplies C=1 thereafter).
;   * fetch reads via LDA (src,X) with a proven X=0 invariant (the init loop
;     ends with X=0; copy_run always exits with X=0; nothing else moves X).
;   * read_gamma exits with A = val lo and Z/C set (LDA val; C=1 on every
;     exit), so the offset-MSB EOF test is one BEQ (gamma 256 wraps the lo
;     byte to 0) and msb-1 is SBC #1.
;   * Literal run: fused "mptr = src; src += len" 16-bit copy+add replaces
;     the copy plus post-copy src re-derivation.
;   * copy_run is one fused loop (page INCs on the Y wrap, X = count lo,
;     val+1 = count hi biased by one when lo != 0); clobbers val+1.
;
; It decodes the raw mode-0x41 blob produced by lzan::zx::compress_min_eof (not
; the LZAN container: the container's "LZAN"+mode+orig_len header is stripped;
; this blob starts directly at the bitstream).
;
; Entry = full_decomp; in-stream EOF marker -> finish -> RTS.
; ===========================================================================
;@format: lzan-min
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzan-decoder-min
;@encoder: lzan::zx::compress_min_eof(i)[1..].to_vec()
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 11
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 181

; ---- config-defaults ----
zp_base = $F1
; ---- end config-defaults ----

bitbuf = zp_base+0  ; current bit byte (MSB first, guard-bit sentinel); $80 = empty
moff   = zp_base+1  ; 2 bytes: current offset (rep0), stored as off-1
src    = zp_base+3  ; bitstream pointer (lo/hi)
dst    = zp_base+5  ; output pointer (lo/hi)
val    = zp_base+7  ; 2 bytes: gamma result / copy length
mptr   = zp_base+9  ; 2 bytes: copy source pointer

full_decomp:
        LDX #7                    ; seed bitbuf/moff/src/dst (adjacent in ZP);
init_loop:                        ; -1-biased bases so the loop ends with X=0
        LDA init_tab-1,X          ; (the X=0 invariant fetch/rg_entry rely on)
        STA zp_base-1,X
        DEX
        BNE init_loop
        ; fall into st_literals with X=0

st_literals:
        JSR read_gamma            ; val = len
        LDA src                   ; mptr = src, src += len (in one pass;
        STA mptr                  ;  copy_run leaves the stream ptr alone)
        CLC
        ADC val
        STA src
        LDA src+1
        STA mptr+1
        ADC val+1
        STA src+1
        JSR copy_run              ; copy val bytes (mptr)->(dst)
        JSR gbit                  ; 1 = new offset, 0 = rep0
        BCC do_rep0
        ; fall into st_newoffset

st_newoffset:
        JSR read_gamma            ; val = msb; A = msb & $FF, Z set, C = 1
        BEQ eof_rts               ; msb == 256 (EOF): lo byte wrapped to 0
        SBC #1                    ; A = msb-1 (carry was set)
        LSR                       ; A = (msb-1)>>1; carry = (msb-1)&1
        STA moff+1
        JSR fetch                 ; A = lsb byte; src advanced (carry preserved)
        ROR                       ; A = (off-1)_lo; carry = lsb&1 (1st len ctrl bit)
        STA moff
        JSR rg_entry              ; gamma(len-1) with backtracked ctrl bit in carry
        INC val                   ; len = (len-1)+1
        BNE domatch
        INC val+1
        BNE domatch

do_rep0:
        JSR read_gamma            ; val = len
domatch:
        LDA dst                   ; mptr = dst - (moff+1)
        CLC
        SBC moff
        STA mptr
        LDA dst+1
        SBC moff+1
        STA mptr+1
        JSR copy_run
after_match:
        JSR gbit
        BCS st_newoffset
        BCC st_literals           ; carry clear -> always taken

; copy_run: copy val (16-bit) bytes (mptr)->(dst); advance dst by val.
; Exits with X=0, Y=val&255, mptr/dst highs page-adjusted; clobbers val+1.
copy_run:
        LDX val                   ; X = count lo
        BEQ cr_go
        INC val+1                 ; hi+1 when lo != 0 (X-loop borrows a page)
cr_go:
        LDY #0
cr_loop:
        LDA (mptr),Y
        STA (dst),Y
        INY
        BNE cr_nohi
        INC mptr+1
        INC dst+1
cr_nohi:
        DEX
        BNE cr_loop
        DEC val+1
        BNE cr_loop
        TYA                       ; dst += Y (page INCs already applied)
        CLC
        ADC dst
        STA dst
        BCC eof_rts
        INC dst+1
eof_rts:
        RTS

; BIT READER. gbit returns the next stream bit in carry (MSB first, guard
; sentinel). Preserves X. The refill ROL needs C=1: the init value $80 shifts
; out C=1 on the first refill, every later refill shifts out the guard bit.
gbit:
        ASL bitbuf                ; carry = next data bit
        BNE gb_have
        JSR fetch                 ; refill: A = next byte (carry = 1 here)
        ROL                       ; C = b7; bit0 = 1 (guard)
        STA bitbuf
gb_have:
        RTS

; fetch: A = (src), src += 1. Preserves carry, Y, X (requires X=0).
fetch:
        LDA (src,X)
        INC src
        BNE f_rts
        INC src+1
f_rts:
        RTS

; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }.
; Returns A = val & $FF with Z/N set, C = 1 (gbit refills clobber A, so the
; value accumulates in val, not A). Requires and preserves X=0: every caller
; comes from init, copy_run (exits X=0), or a path that kept X=0.
read_gamma:
        JSR gbit                  ; C = first control bit
rg_entry:
        STX val+1                 ; val = 1 (X=0)
        LDA #1
        STA val
        BCS rg_done
rg_data:
        JSR gbit                  ; data bit -> carry
        ROL val
        ROL val+1
        JSR gbit                  ; next control bit -> carry
        BCC rg_data
rg_done:
        LDA val
        RTS

init_tab:
        .byte $80, 0, 0, <comp_data, >comp_data, <out_addr, >out_addr
