; ===========================================================================
; LZAN "ZX" 6510 full-grammar decoder, BACKWARD / in-place variant.
; This is lzan's own 6510 decoder; there is no external upstream.
; Same stream as the forward decoder, three direction aspects flipped (as
; zx0 -b / lzsa -b / exomizer -b do):
;
;   1. src reads DOWN  : read_byte decrements src instead of incrementing.
;   2. dst writes DOWN : emit walks dst downward; a literal run consumes the
;                        stream directly through read_byte, so src needs no
;                        save/restore around it.
;   3. match source    : mptr = dst + offset (back-ref is at a HIGHER address;
;                        the offset is ADDED, not subtracted).
;
; The token parse, guard-bit reader (gbit), interlaced Elias-gamma
; (read_gamma / rg_entry), the rep0-3 MTF queue, near-rep math, offset split
; and the backtrack-via-carry trick read the stream bit-for-bit in the same
; order as the forward decoder. This works because the harness feeds the
; BACKWARD stream [mode] ++ reverse(payload): a descending byte reader
; reproduces, byte-for-byte and bit-for-bit, the exact read sequence the
; forward reader performs on the forward payload (matching src/zx.rs
; BackwardBitReader / decode_backward). So the gamma continue-flag polarity is
; UNCHANGED (still continue on 0 / BCC), unlike the official ZX0-back stream.
;
; Stream produced by: lzan::zx::compress_backward(input, 4, true, true, 4).
; That encoder ALWAYS emits mode byte $34 (rep_slots=4, near_rep on,
; am_near_rep on), so this routine is specialized to that grammar: the
; after-literals symbol is always the 7-leaf near-rep prefix tree and the
; after-match decision is always the 3-way 0 / 10 / 11+ri code. The injected
; zx_mode constant is accepted (per the ABI) but not consulted at runtime.
;
; Implementation notes:
;   - all zero-page init (remain/src/dst + bitbuf/moff/reps zero-fill) is one
;     table-driven loop; reps[] and moff hold OFFSET-1, so their init is 0 and
;     domatch folds the +1 back in via the carry, which is 1 on every path
;     into domatch (read_gamma / rg_entry always return C=1, INC/DEC/LDA/STA
;     preserve it).
;   - (v-1)<<7 for the new-offset MSB is two RORs through the carry instead of
;     a 7-step shift loop; read_byte preserves C, so the second ROR both
;     merges the LSB byte and pops the backtracked length-control bit out into
;     the carry for rg_entry (rep_insert preserves C, no PHP/PLP needed).
;   - one shared per-byte `emit` helper (dst--, remain-- + finish test, val--)
;     serves both the literal run and the match copy; when `remain` hits zero
;     it discards its own return address (PLA/PLA) and RTSes straight out of
;     full_decomp, so no saved stack pointer is needed.
;   - the rep index travels pre-doubled (0/2/4/6) in X straight into
;     load_rep_off/rmtf_loop - no zero-page slot at all; the after-literals
;     prefix table encodes rep entries as 2*ridx+1 (the guaranteed C=1 from
;     the zero-count loop makes SBC #1 restore them), $00 for new-offset and
;     $80|2*ridx for near-reps.
;   - rep_mtf/rep_insert share one loop that ends by copying moff (= reps-2)
;     into slot 0, so the store-to-front is the loop's final iteration.
;   - Y is 1 for the whole routine: (zp),Y reads/writes one ABOVE the pointer,
;     which lets read_byte decrement src BEFORE loading (no PHA/PLA to keep A
;     alive) while dst simply idles one below the next write position.
;
; full_decomp seeds src = comp_data+comp_data_len-1 and dst = out_addr+out_len-2
; (one below the last byte; Y=1 rides on top), remain = out_len (termination
; counter). comp_data_len / out_len / zx_mode are injected by the harness. On
; exit the output fills [out_addr, out_addr+out_len).
; Entry = full_decomp; termination is the `remain` counter reaching 0.
; ===========================================================================
;@format: lzan-full
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzan-full-backward
;@encoder: lzan::zx::compress_backward(input, 4, true, true, 4)[1..] (leading mode byte stripped; harness injects zx_mode + out_len + comp_data_len consts)
;@payload: raw
;@eof: length
;@needs: comp_data,out_addr,out_len,comp_data_len,zx_mode
;@zp-len: 24
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 310

; ---- config-defaults ----
zp_base = $E4
; ---- end config-defaults ----

; ---- zero page: one contiguous span, zp_base+0 .. zp_base+23 ----
; Layout is init-driven: indexes 0-5 are loaded from init_tab, 6-16 are
; zero-filled by the same preamble loop (bitbuf's guard sentinel, and
; moff/reps in offset-1 space where the initial rep offset 1 is stored as 0).
; moff MUST stay at reps-2: the rmtf_loop's final iteration copies it into
; rep slot 0. Two declared bytes (+22,+23) are spare.
remain    = zp_base+0   ; 2: bytes left to emit (init=out_len); termination counter
src       = zp_base+2   ; 2: bitstream read pointer (reads DOWN; next byte at src, pre-dec)
dst       = zp_base+4   ; 2: output write pointer (writes DOWN; next write at dst+1)
bitbuf    = zp_base+6   ; current bit buffer (shifted left, MSB-first, guard-bit sentinel)
moff      = zp_base+7   ; 2: current offset MINUS 1 (positive; = reps-2, see rmtf_loop)
; rep queue: 4 offsets (each stored as offset-1) * 2 bytes, contiguous.
reps      = zp_base+9   ; 8 bytes: reps+0/1=rep0 ... reps+6/7=rep3
val       = zp_base+17  ; 2: decoded gamma value / copy count
mptr      = zp_base+19  ; 2: match copy source pointer
; (the rep index needs no slot: it rides pre-doubled in X into load_rep_off)

full_decomp:
          LDX #16
          LDY #1                    ; Y=1 for the rest of the routine
          LDA #0
fd_zero:
          STA remain,X              ; zero bitbuf / moff / reps (indexes 16..6)
          DEX
          CPX #5
          BNE fd_zero
fd_init:
          LDA init_tab,X            ; load remain / src / dst (indexes 5..0)
          STA remain,X
          DEX
          BPL fd_init
          ; A = init_tab[0] = <out_len (the last byte stored)
          ORA remain+1              ; remain==0? -> nothing to do
          BNE st_literals
          RTS

; ===========================================================================
; LITERALS: gamma run length, then copy that many raw stream bytes DOWN.
; The stream pointer IS the literal source, so no pointer shuffling.
; ===========================================================================
st_literals:
          JSR read_gamma            ; val = run length (>=1)
lit_loop:
          JSR read_byte             ; A = next stream byte, src--
          JSR emit                  ; write it, dst--/remain--/val--; Z=run done
          BNE lit_loop

; ===========================================================================
; AFTER-LITERALS symbol (mode $34: always the near-rep prefix tree).
; Tree: 1 / 01 / 001 / 0001 / 00001 / 000001 / 000000
;       new  r0   r1   nr0    r3      r2       nr1
; X counts DOWN from 6 while control bits are 0, so al_tab is indexed by
; 6-zeros. The X=0 fallthrough (6 zeros, C=0 from the last gbit) can only hit
; the $82 near-rep entry, which never consults C; every other entry is reached
; through BCS, so the rep path's SBC #1 always sees C=1.
; ===========================================================================
st_after_lit:
          LDX #6
al_b:
          JSR gbit
          BCS al_done
          DEX
          BNE al_b
al_done:
          LDA al_tab,X              ; $00=new offset, 2*ridx+1=rep, $80|2*ri=near-rep
          BEQ st_newoffset
          BMI al_nr
          SBC #1                    ; C=1 (BCS exit)
          TAX                       ; X = 2*ridx
          ; fall into do_rep

; ===========================================================================
; REP match: moff = reps[X/2], move it to front, gamma length.
; ===========================================================================
do_rep:
          JSR load_rep_off          ; moff = reps[X/2]; X preserved
          JSR rmtf_loop             ; move-to-front with the same X
          JSR read_gamma            ; val = rep length; returns C=1
          BCS domatch               ; always taken

al_nr:                              ; near-rep after-lit symbol: A = $80|2*ri
          AND #2                    ; A = 2*ri (0 or 2)
          BPL st_ridx               ; always taken (N=0)

; ===========================================================================
; NEW OFFSET: gamma msb, then the LSB byte.
;   moff = ((msb-1) << 7) | (lsb >> 1)      (offset-1; domatch re-adds the 1)
; The <<7 is done as two RORs: hi = (msb-1)>>1, its carry-out lands in bit 7
; of the low byte via the ROR that also splits off the backtracked length
; control bit (lsb bit 0) into C for rg_entry. read_byte and rep_insert
; both preserve C.
; ===========================================================================
st_newoffset:
          JSR read_gamma            ; val = msb (>=1), C=1
          LDA val
          SBC #1                    ; 16-bit msb-1 (C=1 in)
          STA moff+1                ; park lo(msb-1)
          LDA val+1
          SBC #0                    ; A = hi(msb-1), 0 or 1
          LSR                       ; C = bit 8 of msb-1
          ROR moff+1                ; moff+1 = (msb-1)>>1 = hi of (msb-1)<<7
                                    ;   ... C = bit 0 of msb-1
          JSR read_byte             ; A = lsb byte (C preserved)
          ROR                       ; A = (msb-1&1)<<7 | lsb>>1, C = length ctrl bit
          STA moff
          JSR rep_insert            ; preserves C
          JSR rg_entry              ; gamma(len-1) with the backtracked bit primed
          ; fall through to gp1_plus1

; ===========================================================================
; gp1_plus1: val = (len-1) + 1, then domatch. C stays 1 throughout.
; ===========================================================================
gp1_plus1:
          INC val
          BNE domatch
          INC val+1
domatch:
          ; BACKWARD: match source is ABOVE dst. moff holds offset-1 and C=1
          ; on every path here, so ADC adds the missing +1 for free.
          LDA dst
          ADC moff
          STA mptr
          LDA dst+1
          ADC moff+1
          STA mptr+1
cm_loop:
          LDA (mptr),Y              ; source byte at mptr+1; then mptr--
          LDX mptr                  ; (X-flavored 16-bit dec: keeps A)
          BNE cm1
          DEC mptr+1
cm1:
          DEC mptr
          JSR emit                  ; Z = match done
          BNE cm_loop
          ; fall into after_match

; ===========================================================================
; AFTER-MATCH dispatch (mode $34): 0=literals, 10=new offset, 11+ri=near-rep.
; ===========================================================================
after_match:
          JSR gbit
          BCC st_literals
          JSR gbit
          BCC st_newoffset
          JSR gbit                  ; ri bit
          LDA #0
          ROL
          ASL                       ; A = 2*ri
st_ridx:
          TAX                       ; X = 2*ri for load_rep_off
          ; fall into do_nearrep

; ===========================================================================
; NEAR-REP match (after-lit or after-match): moff = reps[ridx] +/- gamma delta
; (sign bit first: 0 = add), insert as a fresh offset, new-offset-style length.
; Deltas are unchanged in offset-1 space. PHP/PLP carries the sign across the
; delta gamma read.
; ===========================================================================
do_nearrep:
          JSR load_rep_off          ; moff = reps[X/2] (gbit keeps X; gamma won't)
          JSR gbit
          PHP                       ; save sign bit (C)
          JSR read_gamma            ; val = delta
          PLP
          LDX #0                    ; C=0: add (mask 0)
          BCC nr_go
          LDX #$FF                  ; C=1: subtract (mask $FF, carry already 1)
nr_go:
          TXA
          EOR val
          ADC moff
          STA moff
          TXA
          EOR val+1
          ADC moff+1
          STA moff+1
          JSR rep_insert
          JSR read_gamma            ; val = len-1; returns C=1
          BCS gp1_plus1             ; always taken

finish:
          PLA                       ; drop emit's return address ...
          PLA
          RTS                       ; ... and return from full_decomp itself

; ===========================================================================
; emit: write A to dst+1 (Y=1), dst--, remain-- (jumps to finish when it hits
; 0, never returning), val--; returns Z=1 when the run (val) is exhausted.
; Preserves C (clobbers A and X; both copy loops reload X anyway).
; ===========================================================================
emit:
          STA (dst),Y               ; next write position is dst+1
          LDX #4
          JSR dec16z                ; dst--   (Z result unused)
          LDX #0
          JSR dec16z                ; remain--
          BEQ finish                ; remain==0 -> final byte just written
          LDX #17
          JSR dec16z                ; val--
          RTS                       ; Z = (val==0) for the caller's loop test

; dec16z: 16-bit decrement of the zero-page pair at remain+X; returns Z=1
; when the pair reaches 0. Preserves C.
dec16z:
          LDA remain,X
          BNE dz_lo
          DEC remain+1,X
dz_lo:
          DEC remain,X
          BNE dz_rts                ; lo != 0 -> Z=0
          LDA remain+1,X            ; Z = (hi==0)
dz_rts:
          RTS

; ===========================================================================
; helpers
; ===========================================================================
load_rep_off:                       ; in: X = 2*rep-index (preserved)
          LDA reps,X
          STA moff
          LDA reps+1,X
          STA moff+1
          RTS

; rep_insert: shift all 4 slots down and put moff in front (X=6).
; rmtf_loop (with X=2*ridx from load_rep_off): move-to-front - only slots
; 0..ridx-1 shift (X=0 = store only). The X=0 iteration reads moff via
; reps-2,X - that IS the store-to-front. Both preserve C.
rep_insert:
          LDX #6
rmtf_loop:
          LDA reps-2,X              ; X=0: reps-2 = moff
          STA reps,X
          LDA reps-1,X
          STA reps+1,X
          DEX
          DEX
          BPL rmtf_loop
          RTS

; ===========================================================================
; BIT READER (MSB-first, guard-bit sentinel). Preserves X.
; ===========================================================================
gbit:
          ASL bitbuf                ; carry = next data bit; 0 left = guard popped
          BNE gb_have
          JSR read_byte             ; refill: A = next stream byte, src--
          SEC
          ROL                       ; C = b7 (this call's bit); bit0 = guard
          STA bitbuf
gb_have:
          RTS

; read_byte: fetch next stream byte into A, src -= 1 (reads DOWN). The
; decrement comes FIRST and Y=1 points the load back at the byte, so A needs
; no saving. Preserves X (al_b's zero counter lives there across refills),
; Y and C (st_newoffset RORs through it).
read_byte:
          LDA src
          BNE rb_dec
          DEC src+1
rb_dec:
          DEC src
          LDA (src),Y
          RTS

; ===========================================================================
; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }. Result -> val.
; Continue on 0 / BCC (the backward byte reader reproduces the forward bit
; sequence, so the polarity does NOT flip). Always returns C=1.
; rg_entry: same, with the first control bit pre-primed in C.
; ===========================================================================
read_gamma:
          JSR gbit                  ; C = first control bit
rg_entry:
          LDA #0                    ; val = 1 (C untouched; Y is the constant 1)
          STA val+1
          STY val
          BCS rg_done
rg_data:
          JSR gbit                  ; data bit -> carry
          ROL val
          ROL val+1
          JSR gbit                  ; next control bit -> carry
          BCC rg_data
rg_done:
          RTS

; after-literals symbol table, indexed by 6 MINUS the leading-zero count:
; $00 = new offset, odd = 2*ridx+1 (rep), $80|2*ri = near-rep.
al_tab:
          .byte $82,$05,$07,$80,$03,$01,$00

; preamble init values for zp indexes 0..5 (remain, src, dst).
init_tab:
          .byte <out_len, >out_len
          .byte <(comp_data + comp_data_len - 1), >(comp_data + comp_data_len - 1)
          .byte <(out_addr + out_len - 2), >(out_addr + out_len - 2)
