; ===========================================================================
; Standard ZX0 (v2) 6502 decruncher, BACKWARD / in-place, in asm6502 syntax.
; Upstream: BeebAsm ZX0 decoder by NegativeCharge (port of Krzysztof "XXL"
; Dudek's standard ZX0 decoder); ZX0 format (c) Einar Saukas, BSD-3-Clause.
;
; Mirrors the forward decoder zx0-negativecharge-acorn.s, converted to the
; backward stream layout that lzan::zx0compat::compress_zx0_compatible_backward
; emits and dzx0_decode_backward decodes (the official ZX0 -b /
; dzx0_standard_back form).
;
; BACKWARD in-place: src and dst are the LAST bytes of the compressed block and
; the output buffer; both pointers DECREMENT; a match back-reference lies at a
; HIGHER address so match_source = dst + offset (ADD). This lets packed and
; unpacked regions overlap (write head trails read head).
;
; The three direction flips vs the forward form:
;   1. get_byte reads DOWN (ZX0_INPUT self-mod operand decrements).
;   2. output writes DOWN (ZX0_OUTPUT decrements); match copy walks DOWN.
;   3. match_source = dst + offset (offset stored POSITIVE, added).
; Plus the two backward-stream semantic differences (both present in the
; official ZX0 back variant):
;   * interlaced-gamma continue flag is 1 (terminator 0) -> BCS, not BCC.
;   * offset MSB gamma is NOT inverted (read with the same reader as lengths),
;     and offset = (hi-1)*128 + (lo>>1) + 1, stored positive.
;
; Layout notes:
;   * ZX0_INPUT (the get_byte self-mod operand) is seeded at assembly time to
;     comp_data+comp_data_len (one past the last byte; get_byte pre-decrements),
;     legal because @smc is yes and the routine is one-shot per image.
;   * last_offset is stored as offset-1 (initial offset 1 -> 0, so the whole
;     ZP state inits to zero); the +1 comes back for free via SEC in the
;     16-bit COPY_SRC = dst + offset add.
;   * Y is 0 for the routine's whole lifetime (nothing ever writes it).
;   * both copy loops share one two-entry pointer/length down-count subroutine
;     (dec_src_out_len / dec_out_len) that returns Z=1 when len hits 0.
;   * lenL/lenH are provably 0 at every gamma start, so the elias entry
;     INC lenL / BNE is an always-taken branch into the loop.
;
; full_decomp seeds ZX0_OUTPUT = out_addr+out_len-1 (the last output byte);
; comp_data_len / out_len are injected by the harness. On exit ZX0_OUTPUT =
; out_addr-1. Entry = full_decomp; EOF is the standard ZX0 end marker
; (hi==256) -> RTS.
; ===========================================================================
;@format: zx0
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: zx0-backward
;@encoder: lzan::zx0compat::compress_zx0_compatible_backward(i)
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 8
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 170

; $F7-$FE: RS-232 buffer pointers (free unless RS-232 is used) + the four
; free bytes.
; ---- config-defaults ----
zp_base = $F7
; ---- end config-defaults ----

ZX0_OUTPUT = zp_base+0  ; 2 bytes: output cursor (ZP), writes DOWN
COPY_SRC   = zp_base+2  ; 2 bytes: match-copy source pointer (ZP), reads DOWN
offsetL    = zp_base+4  ; last match offset MINUS ONE (SEC in the add restores it)
offsetH    = zp_base+5
lenL       = zp_base+6  ; 16-bit gamma value / copy down-counter
lenH       = zp_base+7

full_decomp:
        LDA #<(out_addr + out_len - 1)
        STA ZX0_OUTPUT
        LDA #>(out_addr + out_len - 1)
        STA ZX0_OUTPUT+1
        LDY #$00                        ; Y = 0 (index for (ZP),Y) - stays 0 forever
        STY offsetL                     ; last_offset-1 = 0 (INITIAL_OFFSET 1)
        STY offsetH
        STY lenL
        STY lenH
        LDA #$80                        ; empty bit buffer (sentinel primed)
dzx0sb_literals:
        JSR dzx0sb_elias                ; literal length -> lenL:lenH
        PHA                             ; save bit buffer across literal copy
cop0b:
        JSR get_byte                    ; read literal byte (DOWN), returns in A
        STA (ZX0_OUTPUT),Y              ; write DOWN (Y=0)
        JSR dec_out_len                 ; dst walks DOWN; len down-count, Z on 0
        BNE cop0b
        PLA                             ; restore bit buffer
        ASL                             ; new-offset indicator (polarity unchanged)
        BCS dzx0sb_new_offset
        JSR dzx0sb_elias                ; last-offset match length
dzx0sb_copy:
        PHA                             ; save bit buffer across match copy
        LDA ZX0_OUTPUT                  ; COPY_SRC = dst + (offset-1) + 1 (SEC)
        SEC
        ADC offsetL
        STA COPY_SRC
        LDA ZX0_OUTPUT+1
        ADC offsetH
        STA COPY_SRC+1
cop1b:
        LDA (COPY_SRC),Y
        STA (ZX0_OUTPUT),Y
        JSR dec_src_out_len             ; src+dst walk DOWN; len down-count, Z on 0
        BNE cop1b
        PLA                             ; restore bit buffer
        ASL                             ; literals or new offset? (polarity unchanged)
        BCC dzx0sb_literals
dzx0sb_new_offset:
        JSR dzx0sb_elias                ; offset MSB gamma -> lenL:lenH (value>=1)
        LDX lenH                        ; hi==256 (lenH=1,lenL=0) => end marker
        BNE zx0b_rts                    ; end (bit buffer still in A, stack clean)
        PHA                             ; save bit buffer across get_byte
        DEC lenL                        ; lenL = hi-1
        LDA lenL
        LSR                             ; A = (hi-1)>>1, C = (hi-1)&1
        STA offsetH
        JSR get_byte                    ; offset LSB (read DOWN); C preserved
        ROR                             ; A = 16-bit (hi-1):lo >> 1 low byte,
        STA offsetL                     ;   C = lo&1 = first length (backtrack) bit
        LDA #$01
        STA lenL                        ; length = 1 (lenH already 0 here)
        PLA                             ; restore bit buffer (does not touch carry)
        BCC nb_nobt                     ; backtrack bit 0 -> gamma terminates
        JSR dzx0sb_elias_backtrack      ; backtrack bit 1 -> read more length bits
nb_nobt:
        INC lenL                        ; length += 1
        BNE nb_done
        INC lenH
nb_done:
        BNE dzx0sb_copy                 ; always taken (length >= 2)

dzx0sb_elias:
        INC lenL
        BNE dzx0sb_elias_loop           ; always taken (lenL was 0 -> 1)
dzx0sb_elias_backtrack:
        ASL
        ROL lenL
        ROL lenH
dzx0sb_elias_loop:
        ASL
        BNE dzx0sb_elias_skip
        JSR get_byte                    ; C is 1 here: the emptying ASL shifted
        ROL                             ; out the sentinel bit (z80 rla trick)
dzx0sb_elias_skip:
        BCS dzx0sb_elias_backtrack      ; flag 1 -> continue (FLIPPED vs forward)
zx0b_rts:
        RTS                             ; flag 0 -> done (also the EOF exit)

dec_src_out_len:                        ; match entry: src, then dst, then len
        LDA COPY_SRC
        BNE dso0
        DEC COPY_SRC+1
dso0:
        DEC COPY_SRC
dec_out_len:                            ; literal entry: dst, then len
        LDA ZX0_OUTPUT
        BNE dso1
        DEC ZX0_OUTPUT+1
dso1:
        DEC ZX0_OUTPUT
        LDA lenL                        ; 16-bit borrow-decrement counter
        BNE dso2
        DEC lenH
dso2:
        DEC lenL
        BNE dso3                        ; Z=0 -> more to copy
        LDA lenH                        ; Z = (len == 0)
dso3:
        RTS

get_byte:
        LDA ZX0_INPUT                   ; pre-decrement the self-mod operand,
        BNE gb0                         ; then fetch - operand is seeded at
        DEC ZX0_INPUT+1                 ; assembly time to one PAST the last
gb0:                                    ; compressed byte
        DEC ZX0_INPUT
        LDA comp_data + comp_data_len
ZX0_INPUT = *-2
        RTS
