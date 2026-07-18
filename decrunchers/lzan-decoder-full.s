; ===========================================================================
; LZAN "ZX" 6510 full-grammar decoder, in asm6502 syntax for the decrunch-test
; harness. This is lzan's own 6510 decoder; there is no external upstream.
;
; Covers the full grammar: rep0-3 (4-entry MTF offset queue) + after-literals
; near-rep + after-match near-rep. The mode byte
; (rep_slots | near_rep<<4 | am_near_rep<<5) is baked at assemble time from the
; injected `zx_mode` constant: the two near-rep mode bits load as immediates
; and the rep-index unary limit is a CPX immediate, so no runtime mode byte is
; kept (the assembled size is the same for every mode).
;
; Implementation notes:
;   * The `full_decomp` preamble seeds remain=out_len, src=comp_data,
;     dst=out_addr, bitbuf=$80 (bare guard: first gbit refills with C=1) from a
;     7-byte table and zero-fills moff+reps[] (OFFSET-1 space: initial rep
;     offset 1 is stored as 0). The test/bench prepends `zx_mode = <byte>` and
;     `out_len = <len>` constants to this body.
;   * A shared per-byte `emit` helper (dst++, mptr++, remain++, val--) with a
;     self-popping PLA/PLA finish handles both literal and match output;
;     `remain` is stored negated so its tick is INC with a free Z test.
;   * moff/reps live in OFFSET-1 space (domatch folds the -1 in via CLC+SBC);
;     the rep index rides pre-doubled in X (no zp slot); the after-literals
;     unary index read is inlined with its CPX limit baked from zx_mode;
;     read_byte runs with a global Y=0.
;
; Entry = full_decomp; termination is the `remain` counter (orig_len).
; ===========================================================================
;@format: lzan-full
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: lzan-decoder-full
;@encoder: lzan::zx::compress(input, 4)[1..] (leading mode byte stripped; harness injects zx_mode + out_len consts)
;@payload: raw
;@eof: length
;@needs: comp_data,out_addr,out_len,zx_mode
;@zp-len: 24
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 315

; ---- config-defaults ----
zp_base = $E4
; ---- end config-defaults ----

; ---- zero page: one contiguous span, zp_base+0 .. zp_base+23 ----
; Layout is init-driven: indexes 0-6 are loaded from init_tab, 7-16 are
; zero-filled by the same preamble loop (moff and reps[] hold OFFSET-1, so
; the initial rep offset 1 is stored as 0).
; moff MUST stay at reps-2: the rmtf_loop's final iteration copies it into
; rep slot 0. Three declared bytes (+21..+23) are spare.
remain    = zp_base+0   ; 2: NEGATED bytes left to emit (init=-out_len); counts UP
                        ;    so the per-byte tick is INC+INC with a free Z test
src       = zp_base+2   ; 2: bitstream read pointer
dst       = zp_base+4   ; 2: output write pointer
bitbuf    = zp_base+6   ; current bit buffer (shifted left, MSB-first, guard-bit sentinel)
moff      = zp_base+7   ; 2: current offset MINUS 1 (= reps-2, see rmtf_loop)
; rep queue: 4 offsets (each stored as offset-1) * 2 bytes, contiguous.
reps      = zp_base+9   ; 8 bytes: reps+0/1=rep0 ... reps+6/7=rep3
val       = zp_base+17  ; 2: decoded gamma value / copy count
mptr      = zp_base+19  ; 2: match copy source pointer
; (the rep index needs no slot: it rides pre-doubled in X into load_rep_off)

full_decomp:
          LDX #16
          LDY #0                    ; Y = 0 for the rest of the routine
fd_zero:
          STY remain,X              ; zero moff + reps[] (indexes 16..7)
          DEX
          CPX #6
          BNE fd_zero
fd_init:
          LDA init_tab,X            ; remain / src / dst / bitbuf (indexes 6..0)
          STA remain,X
          DEX
          BPL fd_init
          ; last table load (X=0) left A = <(-out_len)
          ORA remain+1              ; remain==0? -> nothing to do
          BNE st_literals
          RTS

; ===========================================================================
; LITERALS: gamma run length, then copy that many raw stream bytes. The
; stream pointer IS the literal source, so no pointer shuffling.
; ===========================================================================
st_literals:
          JSR read_gamma            ; val = run length (>=1)
lit_loop:
          JSR read_byte             ; A = next stream byte, src++
          JSR emit                  ; write it, dst++/remain--/val--; Z=run done
          BNE lit_loop
          ; fall through to st_after_lit

; ===========================================================================
; AFTER-LITERALS symbol. Both grammars open with `1` = new offset, so the
; first bit is read BEFORE the mode dispatch; the near_rep mode bit is an
; assemble-time immediate: nonzero selects the rest of the 7-leaf prefix
; tree, zero the plain unary rep index.
; ===========================================================================
st_after_lit:
          JSR gbit                  ; shared first bit: 1 = new offset
          BCS st_newoffset
          LDA #zx_mode/16-(zx_mode/32)*2   ; near_rep mode bit (baked)
          BNE al_nr
          LDX #0                    ; unary rep index, pre-doubled in X
ru_b:
          CPX #(zx_mode-(zx_mode/16)*16-1)*2 ; 2*(rep_slots-1), baked
          BCS do_rep                ; index maxed: no more bits
          JSR gbit
          BCS do_rep
          INX
          INX
          BNE ru_b                  ; always (X = 2,4,6: nonzero)

; Tree: 1 / 01 / 001 / 0001 / 00001 / 000001 / 000000
;       new  r0   r1   nr0    r3      r2       nr1
; The leading `1` (new offset) was consumed above, so X counts DOWN from 5
; while control bits are 0: al_tab is indexed by 6-zeros as before, minus the
; dropped new-offset leaf. Entries are 4*ridx (rep) / 4*ri+1 (near-rep): one
; LSR yields X=2*index with the near-rep flag in C (LSR sets C itself, so the
; X=0 fallthrough's stale carry is harmless).
al_nr:
          LDX #5
al_b:
          JSR gbit
          BCS al_done
          DEX
          BNE al_b
al_done:
          LDA al_tab,X              ; 4*ridx=rep, 4*ri+1=near-rep
          LSR                       ; A = 2*index; C = near-rep flag
          TAX
          BCS do_nearrep
          ; fall into do_rep

; ===========================================================================
; REP match: moff = reps[X/2], move it to front, gamma length.
; ===========================================================================
do_rep:
          JSR load_rep_off          ; moff = reps[X/2]; X preserved
          JSR rmtf_loop             ; move-to-front with the same X
          JSR read_gamma            ; val = rep length; returns C=1
          BCS domatch               ; always taken

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
ins_len:                            ; shared tail: insert offset, gamma(len-1)
          JSR rep_insert            ; preserves C (the primed length control bit)
          JSR rg_entry              ; val = gamma(len-1)
          ; fall through to gp1_plus1

; ===========================================================================
; gp1_plus1: val = (len-1) + 1, then domatch.
; ===========================================================================
gp1_plus1:
          INC val
          BNE domatch
          INC val+1
domatch:
          CLC                       ; moff holds offset-1: SBC with C=0 takes
          LDA dst                   ;   the extra 1 off for free
          SBC moff
          STA mptr
          LDA dst+1
          SBC moff+1
          STA mptr+1
cm_loop:
          LDA (mptr),Y              ; (emit advances mptr)
          JSR emit                  ; Z = match done
          BNE cm_loop
          ; fall into after_match

; ===========================================================================
; AFTER-MATCH dispatch. The am_near_rep mode bit is an assemble-time
; immediate: plain mode short-circuits to new-offset after the first 1 bit.
; ===========================================================================
after_match:
          JSR gbit
          BCC st_literals           ; 0 -> literals
          LDA #zx_mode/32           ; am_near_rep mode bit (baked)
          BEQ st_newoffset
          JSR gbit
          BCC st_newoffset
          JSR gbit                  ; ri bit
          TYA                       ; A = 0 (Y is the constant 0; C preserved)
          ROL                       ; A = ri, C = 0
          ASL                       ; A = 2*ri
st_ridx:
          TAX                       ; X = 2*ri for load_rep_off
          ; fall into do_nearrep

; ===========================================================================
; NEAR-REP match (after-lit or after-match): moff = reps[ridx] +/- gamma delta
; (sign bit first: 0 = add), insert as a fresh offset, new-offset-style length.
; Deltas are unchanged in offset-1 space. PHP/PLP carries the sign across the
; delta gamma read; the sign bit doubles as the add/sub carry-in.
; ===========================================================================
do_nearrep:
          JSR load_rep_off          ; moff = reps[X/2] (gbit keeps X; gamma won't)
          JSR gbit                  ; C = sign bit (1 = subtract)
          PHP
          JSR read_gamma            ; val = delta; leaves X=1 (rg_entry's seed)
          DEX                       ; add: mask $00 (carry-in 0 = the sign bit)
          PLP                       ; C = sign
          BCC nr_go
          LDX #$FF                  ; sub: mask $FF (carry-in 1 = the sign bit)
nr_go:
          TXA
          EOR val
          ADC moff
          STA moff
          TXA
          EOR val+1
          ADC moff+1
          STA moff+1
          JSR gbit                  ; C = first length-gamma control bit
          BNE ins_len               ; always (gbit returns Z=0)

; ===========================================================================
; emit: write A to (dst), dst++, mptr++ (a don't-care during literal runs:
; mptr is scratch until domatch rewrites it), remain++ (negated count: jumps
; to finish when it hits 0, never returning), val--; returns Z=1 when the run
; (val) is exhausted. Clobbers A only.
; ===========================================================================
emit:
          STA (dst),Y               ; Y = 0
          INC dst
          BNE em0
          INC dst+1
em0:
          INC mptr                  ; mptr++ (match copy source)
          BNE em1
          INC mptr+1
em1:
          INC remain                ; remain++ (it is stored negated)
          BNE em2                   ; lo != 0: not done (hi untouched: no carry)
          INC remain+1
          BEQ finish                ; remain==0 -> final byte just written
em2:
          LDA val                   ; val-- with Z = (val==0)
          BNE em3
          DEC val+1
em3:
          DEC val
          BNE em4                   ; lo != 0 -> Z=0
          LDA val+1                 ; Z = (hi==0)
em4:
          RTS

; ===========================================================================
; helpers
; ===========================================================================
finish:
          PLA                       ; drop emit's return address, then fall into
          PLA                       ;   load_rep_off: zp-only, ends in RTS ->
                                    ;   returns from full_decomp itself
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
          ASL bitbuf                ; carry = next data bit; bitbuf hits 0 when the guard pops out
          BNE gb_have
          JSR read_byte             ; refill: A = next stream byte, src advanced
          ROL                       ; C=1 here (the guard bit ASL'd out): C = b7 (this
          STA bitbuf                ;   call's bit) and bit0 = 1 (fresh guard sentinel)
gb_have:
          RTS                       ; always returns Z=0 (bitbuf keeps the guard bit)

read_byte:                          ; preserves X, Y and C
          LDA (src),Y               ; Y = 0
          INC src
          BNE rb_ret
          INC src+1
rb_ret:
          RTS

; ===========================================================================
; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }. Result -> val.
; rg_entry: same, with the first control bit pre-primed in C.
; Always returns C=1.
; ===========================================================================
read_gamma:
          JSR gbit                  ; C = first control bit (from the stream)
rg_entry:
          STY val+1                 ; val = 1 (Y = 0)
          LDX #1
          STX val
          BCS rg_done
rg_data:
          JSR gbit                  ; data bit -> carry
          ROL val
          ROL val+1
          JSR gbit                  ; next control bit -> carry
          BCC rg_data
rg_done:
          RTS                       ; always returns C=1

; after-literals symbol table, indexed by 6 MINUS the leading-zero count
; (the zero-count includes the already-consumed leading flag bit):
; 4*ridx = rep, 4*ri+1 = near-rep (LSR-decoded).
al_tab:
          .byte $05,$08,$0C,$01,$04,$00

; preamble seed for zp indexes 0..6: -out_len (remain counts up to 0),
; comp_data, out_addr, bitbuf=$80 (bare guard bit). moff/reps[] are
; zero-filled (offset-1 space).
init_tab:
          .byte <(0-out_len),>(0-out_len),<comp_data,>comp_data,<out_addr,>out_addr,$80
