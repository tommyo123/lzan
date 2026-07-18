; ===========================================================================
; LZAN "ZX" 6510 full-grammar decoder, speed-optimized variant.
; This is lzan's own 6510 decoder; there is no external upstream.
; Baseline: lzan-decoder-full.s. Same stream (mode read at runtime), no format
; change; decodes byte-identically on modes 0x04/0x14/0x34.
;
; Speed technique: the bit reader is inlined at the hot gamma loop (read_gamma /
; rg_data, two bits per gamma iteration, the dominant cost) and at the
; after_match dispatch. The common no-refill case is `ASL bitbuf / BNE have`
; inline; only the ~1/8 refill path JSRs `gb_refill`. Cold-path bit reads
; (read_rep_index, the near-rep prefix loops) keep the `gbit` subroutine to
; avoid bloating rarely-taken code. Output is identical, just fewer JSR/RTS
; round-trips.
;
; Larger than the baseline (inlining trades bytes for cycles).
; ===========================================================================
;@format: lzan-full
;@direction: forward
;@variant: opt-speed
;@entry: full_decomp
;@vfy-key: lzan-decoder-full
;@encoder: lzan::zx::compress(input, 4)[1..] (leading mode byte stripped; harness injects zx_mode + out_len consts)
;@payload: raw
;@eof: length
;@needs: comp_data,out_addr,out_len,zx_mode
;@zp-len: 27
;@scratch: none
;@illegal: no
;@smc: no
;@code-bytes: 555

; ---- config-defaults ----
zp_base = $E1
; ---- end config-defaults ----

; ---- zero page: one contiguous span, zp_base+0 .. zp_base+26 ----
; (this variant uses `orig` and `sign`, so its span is 3 bytes wider than the
;  baseline; src/dst keep their original $F8/$FA addresses under the default
;  zp_base.)
orig      = zp_base+0   ; 2: original length (input)
remain    = zp_base+2   ; 2: bytes left to emit (init=orig); termination counter
mode_byte = zp_base+4
bitbuf    = zp_base+5   ; current bit buffer (shifted left, MSB-first, guard-bit sentinel)
val       = zp_base+6   ; 2: decoded gamma value
moff      = zp_base+8   ; 2: current offset
sign      = zp_base+10
mptr      = zp_base+11  ; 2: copy source pointer (literals: =src; match: =dst-moff)
ridx      = zp_base+13
entry_sp  = zp_base+14
; rep queue: 4 offsets * 2 bytes, contiguous. reps+0/1=rep0 ... reps+6/7=rep3
; (rmtf_loop indexes reps-2,X / reps-1,X across the whole block - keep intact)
reps      = zp_base+15  ; 8 bytes
src       = zp_base+23  ; 2: bitstream read pointer
dst       = zp_base+25  ; 2: output write pointer

full_decomp:
          LDA #<comp_data
          STA src
          LDA #>comp_data
          STA src+1
          LDA #<out_addr
          STA dst
          LDA #>out_addr
          STA dst+1
          LDA #<out_len
          STA orig
          LDA #>out_len
          STA orig+1
          LDA #zx_mode
          STA mode_byte
          ; fall into decode_entry

; ===========================================================================
decode_entry:
          TSX
          STX entry_sp
          LDA #0
          STA bitbuf                ; bitbuf=0 -> first gbit refills (guard-bit sentinel)
          LDA orig
          STA remain
          LDA orig+1
          STA remain+1
          ORA remain                ; remain==0? -> nothing to do (SP still clean -> rts)
          BNE de_go
          RTS
de_go:
          ; init reps[] = {1,1,1,1}: lo byte (even idx)=1, hi byte (odd idx)=0.
          LDX #7
          LDA #0
ir_l:
          STA reps,X
          EOR #1
          DEX
          BPL ir_l
          ; fall into st_literals

; ===========================================================================
; LITERALS: gamma run length, then copy that many raw bitstream bytes.
; ===========================================================================
st_literals:
          JSR read_gamma            ; val = run length
          LDA src
          STA mptr
          LDA src+1
          STA mptr+1
cl_loop:
          JSR copy_run              ; copies val bytes (mptr)->(dst), clamps to remain, advances dst
          TYA
          CLC
          ADC mptr
          STA src
          LDA mptr+1
          ADC #0
          STA src+1
cl_done:
          ; fall through to st_after_lit

; ===========================================================================
; AFTER-LITERALS symbol -> sets ridx and jumps into a match producer.
; ===========================================================================
st_after_lit:
          LDA mode_byte
          AND #$10                  ; near_rep?
          BNE al_nr
          ; classic after-literals: 1 bit (inline gbit). Hot - after every literal run.
          ASL bitbuf
          BNE al_have
          JSR gb_refill
al_have:
          BCS st_newoffset
          LDX #0
          STX ridx
          LDA mode_byte
          AND #$0F
          CMP #2
          BCC do_rep                ; rep_slots<2 -> rep0
          JSR read_rep_index
          JMP do_rep
al_nr:
          LDX #0
al_nr_b:
          JSR gbit
          BCS al_nr_done
          INX
          CPX #6
          BCC al_nr_b
al_nr_done:
          LDA al_tab,X
          CMP #$FF
          BEQ al_no                 ; NewOffset
          TAY
          AND #3
          STA ridx
          TYA
          AND #$40
          BNE do_nearrep
          JMP do_rep
al_no:
          JMP st_newoffset
al_tab:
          .byte $FF,$00,$01,$40,$03,$02,$41

; ===========================================================================
; Match producers.
; ===========================================================================
do_rep:
          JSR load_rep_off
          JSR rep_mtf
          JSR read_gamma            ; val = rep length
          JMP domatch

do_nearrep:
st_am_nearrep:
          JSR load_rep_off
          JSR read_near_rep_off
          JSR rep_insert
          JMP gamma_p1_match

; ===========================================================================
; NEW OFFSET
; ===========================================================================
st_newoffset:
          JSR read_gamma            ; val = msb
          LDA val
          SEC
          SBC #1
          STA moff
          LDA val+1
          SBC #0
          STA moff+1
          LDX #7
no_sh:
          ASL moff
          ROL moff+1
          DEX
          BNE no_sh
          JSR read_byte             ; A=lsb byte
          LSR                       ; lsb>>1 ; carry = lsb&1 = backtracked first length ctrl bit
          ROR sign                  ; stash carry into sign bit7 (survives rep_insert)
          ORA moff
          STA moff
          INC moff                  ; off += 1
          BNE no_nc
          INC moff+1
no_nc:
          JSR rep_insert
          ASL sign                  ; carry = backtracked first length control bit
          JSR rg_entry              ; gamma(len-1) with that bit primed; result in val
          JMP gp1_plus1             ; -> mlen = val + 1, domatch

; ===========================================================================
; gamma_p1_match: mlen = read_gamma()+1, then domatch.
; ===========================================================================
gamma_p1_match:
          JSR read_gamma
gp1_plus1:
          INC val                   ; val = len = (len-1) + 1
          BNE domatch
          INC val+1
domatch:
          LDA dst
          SEC
          SBC moff
          STA mptr
          LDA dst+1
          SBC moff+1
          STA mptr+1
cm_loop:
          JSR copy_run
cm_done:
          ; fall into after_match

; ===========================================================================
; AFTER-MATCH dispatch.
; ===========================================================================
after_match:
          LDA mode_byte
          AND #$20                  ; am_near_rep?
          BNE am_disp
          ; classic after-match: 1 bit (inline gbit). Hot - runs after every match.
          ASL bitbuf
          BNE am_have
          JSR gb_refill
am_have:
          BCC am_lit                ; 0 -> literals
          BCS am_newoff             ; 1 -> new offset (carry set: always)
am_disp:
          JSR gbit
          BCC am_lit
          JSR gbit
          BCC am_newoff
          JSR gbit                  ; ri bit -> ridx (0 or 1)
          LDA #0
          ROL                       ; A = carry
          STA ridx
          JMP st_am_nearrep
am_newoff:
          JMP st_newoffset
am_lit:
          JMP st_literals

finish:
          LDX entry_sp
          TXS
          RTS

; ===========================================================================
; copy_run: shared copy routine. Count in `val`, source in `mptr`.
; ===========================================================================
copy_run:
          LDA val+1
          CMP remain+1
          BCC cr_ok
          BNE cr_clamp
          LDA val
          CMP remain
          BCC cr_ok
cr_clamp:
          LDA remain
          STA val
          LDA remain+1
          STA val+1
cr_ok:
          LDA remain
          SEC
          SBC val
          STA remain
          LDA remain+1
          SBC val+1
          STA remain+1
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
          BCC cr_nodhi
          INC dst+1
cr_nodhi:
          LDA remain                ; remain==0 -> final run -> finish
          ORA remain+1
          BNE cr_ret
          JMP finish
cr_ret:
          RTS

; ===========================================================================
; helpers
; ===========================================================================
load_rep_off:
          LDA ridx
          ASL
          TAX
          LDA reps,X
          STA moff
          LDA reps+1,X
          STA moff+1
          RTS

rep_insert:
          LDX #6                    ; shift all 4 slots
          BNE rmtf_loop             ; (always: x=6)
rep_mtf:
          LDA ridx
          BEQ mtf_done              ; ridx==0 -> no-op
          ASL
          TAX                       ; x = ridx*2
rmtf_loop:
          LDA reps-2,X
          STA reps,X
          LDA reps-1,X
          STA reps+1,X
          DEX
          DEX
          BNE rmtf_loop
          LDA moff
          STA reps+0
          LDA moff+1
          STA reps+1
mtf_done:
          RTS

read_near_rep_off:
          JSR gbit
          LDA #0
          ROL                       ; A = sign bit (1 = subtract)
          STA sign
          JSR read_gamma            ; val = delta
          LDX #0
          CLC                       ; add: mask 0, carry 0
          LDA sign
          BEQ nr_go
          LDX #$FF                  ; sub: mask $ff, carry 1
          SEC
nr_go:
          TXA
          EOR val
          ADC moff
          STA moff
          TXA
          EOR val+1
          ADC moff+1
          STA moff+1
          RTS

read_rep_index:
          LDX #0
rri_b:
          JSR gbit
          BCS rri_done
          INX
          CPX #3
          BCC rri_b
rri_done:
          STX ridx
          RTS

; ===========================================================================
; BIT READER.
; ===========================================================================
gbit:
          ASL bitbuf                ; carry = next data bit; bitbuf hits 0 when the guard pops out
          BNE gb_have
          JSR gb_refill
gb_have:
          RTS

; gb_refill: reservoir empty -> read next stream byte, return its b7 in carry,
; reseed the guard. Shared by the inline sites and the gbit subroutine.
gb_refill:
          JSR read_byte             ; A = next stream byte, src advanced, Y=0
          SEC
          ROL                       ; C = b7 (this call's bit); bit0 = 1 (guard sentinel)
          STA bitbuf
          RTS

read_byte:
          LDY #0
          LDA (src),Y
          INC src
          BNE rb_ret
          INC src+1
rb_ret:
          RTS

; ===========================================================================
; read_gamma: value=1; while ctrl==0 { value=(value<<1)|data }. Result -> val.
; Bit reads INLINED (ASL bitbuf / BNE / JSR gb_refill) - the hot path.
; ===========================================================================
read_gamma:
          ; first control bit (inline gbit)
          ASL bitbuf
          BNE rg_have0
          JSR gb_refill
rg_have0:
rg_entry:
          LDX #0                    ; val = 1, val+1 = 0
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
