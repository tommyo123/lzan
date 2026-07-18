; ===========================================================================
; LZAN minimal rep0 decoder, speed-optimized variant.
; This is lzan's own 6510 decoder; there is no external upstream.
; Baseline: lzan-decoder-min.s. Same mode-0x41 (rep0-only, in-stream EOF)
; stream, no format change; decodes byte-identically.
;
; Speed techniques applied (output identical, just fewer cycles):
;   * Bit reader inlined at the hot gamma sites. The baseline calls `gbit` via
;     JSR (12 cyc JSR+RTS overhead per bit); here the common no-refill case is
;     just `ASL bitbuf / BNE have` inline, and only the ~1/8 refill path takes a
;     JSR to `gb_refill`. The gamma data/control loop (`rg_data`) reads two bits
;     per iteration and is the dominant cost, so inlining there is the big win.
;   * `read_gamma` / `rg_entry` keep their original entry contract (rg_entry is
;     entered carry-primed from the new-offset backtrack), so callers are
;     unchanged.
;   * `fetch` stays a subroutine (called only on refill + the offset-LSB read).
;
; Larger than the baseline (inlining trades bytes for cycles).
; ===========================================================================
;@format: lzan-min
;@direction: forward
;@variant: opt-speed
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
;@code-bytes: 220

; ---- config-defaults ----
zp_base = $F1
; ---- end config-defaults ----

bitbuf = zp_base+0  ; current bit byte (MSB first, guard-bit sentinel)
val    = zp_base+1  ; 2 bytes: gamma result / copy length
moff   = zp_base+3  ; 2 bytes: current offset (rep0), stored as off-1
mptr   = zp_base+5  ; 2 bytes: copy source pointer
src    = zp_base+7  ; bitstream pointer (lo/hi)  (default $F8, original address)
dst    = zp_base+9  ; output pointer (lo/hi)     (default $FA, original address)

full_decomp:
        LDA #<comp_data
        STA src
        LDA #>comp_data
        STA src+1
        LDA #<out_addr
        STA dst
        LDA #>out_addr
        STA dst+1
        ; fall through into decode_entry

decode_entry:
        LDA #0
        STA bitbuf                ; bitbuf=0 -> first gbit refills
        STA moff+1
        STA moff                  ; rep0 = off-1 = 0 (off=1)
        ; fall into st_literals

st_literals:
        JSR read_gamma            ; val = len
        LDA src                   ; mptr = src
        STA mptr
        LDA src+1
        STA mptr+1
cl_loop:
        JSR copy_run              ; copy val bytes (mptr)->(dst)
cl_done:
        TYA
        CLC
        ADC mptr
        STA src
        LDA mptr+1
        ADC #0
        STA src+1
        ; inline gbit: 1 = new offset, 0 = rep0
        ASL bitbuf
        BNE sl_have
        JSR gb_refill
sl_have:
        BCC do_rep0
        ; fall into st_newoffset

st_newoffset:
        JSR read_gamma            ; val = msb
        LDA val+1                 ; msb >= 256 -> EOF marker
        BNE finish
        LDX val                   ; A = msb-1
        DEX
        TXA
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
        ; fall into domatch

domatch:
        LDA dst
        CLC
        SBC moff
        STA mptr
        LDA dst+1
        SBC moff+1
        STA mptr+1
cm_loop:
        JSR copy_run
cm_done:
        ; fall into after_match

after_match:
        ; inline gbit: 1 = new offset, 0 = literals
        ASL bitbuf
        BNE am_have
        JSR gb_refill
am_have:
        BCS st_newoffset
        BCC st_literals           ; carry clear -> always taken

finish:
        RTS

; copy_run: copy val (16-bit) bytes (mptr)->(dst); advance dst by val.
copy_run:
        LDY #0
        LDX val+1
        BEQ cr_partial
cr_page:
        LDA (mptr),Y
        STA (dst),Y
        INY
        BNE cr_page
        INC mptr+1
        INC dst+1
        DEX
        BNE cr_page
cr_partial:
        LDX val
        BEQ cr_advance
cr_pl:
        LDA (mptr),Y
        STA (dst),Y
        INY
        DEX
        BNE cr_pl
cr_advance:
        TYA
        CLC
        ADC dst
        STA dst
        BCC adv1
        INC dst+1
adv1:
        RTS

; gb_refill: refill the bit reservoir and return the freshly-read MSB in carry.
; Called only when `ASL bitbuf` zeroed the buffer (~1 in 8 bit reads). Returns
; with carry = the next data bit (b7 of the new byte) and the guard reseeded.
gb_refill:
        JSR fetch                 ; A = next byte, src advanced, Y=0
        SEC
        ROL                       ; C = b7; bit0 = 1 (guard)
        STA bitbuf
        RTS

; fetch: A = (src), src += 1. Preserves carry. Leaves Y=0.
fetch:
        LDY #0
        LDA (src),Y
        INC src
        BNE adv2
        INC src+1
adv2:
        RTS

; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }.
; Bit reads INLINED (ASL bitbuf / BNE / JSR gb_refill) for speed.
read_gamma:
        ; first control bit (inline gbit)
        ASL bitbuf
        BNE rg_have0
        JSR gb_refill
rg_have0:
rg_entry:
        LDX #0                    ; val = 1
        STX val+1
        INX
        STX val
        BCS rg_done
rg_data:
        ; data bit (inline gbit)
        ASL bitbuf
        BNE rg_have1
        JSR gb_refill
rg_have1:
        ROL val
        ROL val+1
        ; next control bit (inline gbit)
        ASL bitbuf
        BNE rg_have2
        JSR gb_refill
rg_have2:
        BCC rg_data
rg_done:
        RTS
