; ===========================================================================
; Shrinkler 6502 BACKWARD / in-place decruncher ("unShrinkler", reverse).
; Upstream: unShrinkler (c) 2021 Krzysztof Dudek and Piotr Fusik, zlib
; (a 6502 port of Aske Simon Christensen's Shrinkler).
;
; Mirror of shrinkler-atari8xxl-unshrinkler.s (PARITY=0, FAST=0) with the three
; LZ-layer direction flips. The adaptive binary range coder core (d2/d3/srcBits
; reservoir, 16-bit software multiply, prob tables) is direction-independent and
; unchanged.
;
; Backward stream: lzan::shrinkler::compress_shrinkler_backward = reverse of a
; forward Shrinkler stream of the reversed input. Reading that stream from its
; LAST byte downward (MSB-first per byte, unchanged reservoir) reproduces the
; exact bit sequence the forward decoder consumes; writing the produced bytes
; DOWNWARD from out_addr+out_len-1 lands each at its correct in-place address.
;
; Direction flips vs the forward form (everything else identical):
;   1. readBit source fetch RETREATS: 16-bit DEC *before* each byte fetch
;      (s_src is seeded one PAST the last stream byte), so the freshly read
;      byte never has to be saved around the pointer update.
;   2. literal WRITE dest RETREATS (post-write 16-bit DEC) instead of INC.
;   3. match source = dst + offset (backref at HIGHER address); the offset
;      operand becomes (number-2) instead of (2-number), and the copy walks
;      DOWNWARD (decrement copy + dst per byte).
;
; Structure notes:
;   * all 16-bit pointer/counter decrements funnel through s_decWord
;     (word at zp offset X) / s_decWords (words at offsets X,X-2,..,2;
;     exits X=0, re-establishing the getBit X=0 invariant for free);
;   * zp reordered so s_src sits at offset 0 (readBit's X=0 call needs no
;     LDX) and s_dst/s_copy/s_number at 2/4/6 (one s_decWords call steps
;     the whole copy-loop trio);
;   * everything the preamble seeds (src, dst, d2, d3, tabs-lo) lives in
;     offsets 0..12, filled by one table-copy loop from s_initTab;
;   * dead LDY #0 before the copy loop dropped (getNumber exits Y=0), and
;     the end-marker SBC runs on the known C=0 from getNumber (no SEC).
;
; Calling convention (harness prepends `comp_data_len = N` and `out_len = M`):
;   full_decomp seeds s_src = comp_data+comp_data_len (one past the last
;   stream byte; pre-decrement fetch) and s_dst = out_addr+out_len-1 (last
;   output byte), then falls into init. On exit s_dst = out_addr-1; output
;   fills [out_addr, out_addr+out_len).
; ===========================================================================
;@format: shrinkler
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: shrinkler-backward
;@encoder: lzan::shrinkler::compress_shrinkler_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 19
;@scratch: symbol=probs_base,len=1536,align=page
;@illegal: no
;@smc: yes
;@code-bytes: 325

; ---- config-defaults ----
zp_base = $40
probs_base = $2000
; ---- end config-defaults ----

; ---- zero page layout (zp_base + 0..18, contiguous; REORDERED vs forward:
;      s_src at 0 so s_decWord with the invariant X=0 hits it directly,
;      s_dst/s_copy/s_number at 2/4/6 for the s_decWords copy-loop trio,
;      and offsets 0..12 form the s_initTab-seeded preamble span). ----
s_src     = zp_base               ; word  (compressed source)
s_dst     = zp_base+2             ; word  (output dest)
s_copy    = zp_base+4             ; word  (copy source during a match)
s_factor  = zp_base+4             ; (aliased)
s_number  = zp_base+6             ; word
s_d2      = zp_base+8             ; word  (range value)
s_d3      = zp_base+10            ; word  (range size)
s_tabs    = zp_base+12            ; word  (prob table pointer)
s_cp      = zp_base+14            ; word  (multiply result)
s_frac    = zp_base+16            ; word
s_srcBits = zp_base+18            ; bit reservoir

; ---- prob tables: page-aligned scratch RAM (probs_base, $600 bytes;
;      unchanged from forward). ----
probs       = probs_base
probsRef    = probs_base + $200
probsLength = probsRef
probsOffset = probsRef + $200

; ---------------------------------------------------------------------------
; Preamble: one loop copies s_initTab into zp offsets 0..12 - src/dst seeds
; (FLIP: last bytes of each region), d2=0, d3=1, tabs-lo=0 (<probs_base is 0,
; page-aligned). Offsets 4..7 (s_copy/s_number) are don't-care.
full_decomp:
        LDX #12
s_seed:
        LDA s_initTab,X
        STA s_src,X
        DEX
        BPL s_seed

; ---- init prob tables to $8000 and range state (verbatim from forward) ----
        LDX #>(probsOffset+$100)
        LDY #0
        TYA               ; A = 0 (first fill value)
s_initPage:
        STX s_tabs+1
s_fill:
        STA (s_tabs),Y    ; fills whole page
        INY
        BNE s_fill
        STA s_srcBits     ; eventually $80
        EOR #$80          ; alternate fill value $00 <-> $80
        DEX
        CPX #>probs_base
        BCS s_initPage
        TAX               ; #0

; ---------------------------------------------------------------------------
s_literal:
        LDY #1
s_literalBit:
        JSR s_getBit
        TYA
        ROL
        TAY
        BCC s_literalBit
        STA (s_dst,X)     ; X=0, write literal byte
        ; FLIP 2: dst RETREATS (post-write 16-bit decrement); X back to 0.
        LDX #2
        JSR s_decWords
        JSR s_getKind
        BCC s_literal

        LDA #>probsRef
        JSR s_getBitFrom
        BCC s_readOffset

s_readLength:
        LDA #>probsLength
        JSR s_getNumber
; FLIP 3a: match source = dst + offset. Operand (s_offsetL+1 / s_offsetH+1) now
; holds (number-2) = offset, so this 16-bit ADD yields s_copy = s_dst + offset.
s_offsetL:
        LDA #$ff           ; operand self-modified by s_readOffset (= offset lo)
        ADC s_dst          ; C=0 (from getNumber)
        STA s_copy
s_offsetH:
        LDA #$ff           ; operand self-modified by s_readOffset (= offset hi)
        ADC s_dst+1
        STA s_copy+1

; FLIP 3b: copy DOWNWARD. Copy s_number bytes, decrementing s_number, s_copy
; and s_dst per byte (one s_decWords call: zp offsets 6/4/2, exits X=0), so
; overlapping matches replay identically to the forward ascending copy.
; Y=0 here: getNumber always exits with Y=0.
s_copyByte:
        LDA (s_copy),Y
        STA (s_dst),Y
        LDX #6
        JSR s_decWords
        LDA s_number
        ORA s_number+1
        BNE s_copyByte

s_copyDone:
        JSR s_getKind
        BCC s_literal

s_readOffset:
        LDA #>probsOffset
        JSR s_getNumber
; FLIP 3c: operand = number-2 = offset (was 2-number forward). End marker is
; offset==0 (number==2): store operand, then finish iff (number-2)==0.
; C=0 from getNumber, so SBC #1 subtracts 2.
        LDA s_number
        SBC #1
        STA s_offsetL+1   ; self-modify the LDA #$ff (s_offsetL) operand
        LDA s_number+1
        SBC #0
        STA s_offsetH+1   ; self-modify the LDA #$ff (s_offsetH) operand
        ORA s_offsetL+1   ; A = offsetHi | offsetLo
        BNE s_readLength  ; offset != 0 -> decode length
        RTS               ; offset == 0 -> finish

; ---------------------------------------------------------------------------
s_getNumber:
        STA s_tabs+1
        LDA #1
        STA s_number
        STY s_number+1    ; #0  (Y must be 0 on entry)
s_getNumberCount:
        INY
        INY
        JSR s_getBit
        BCS s_getNumberCount

s_getNumberBit:
        DEY
        JSR s_getBit
        ROL s_number
        ROL s_number+1
        DEY
        BNE s_getNumberBit
        RTS

s_getKind:
        LDY #0
        LDA #>probs
s_getBitFrom:
        STA s_tabs+1
        BNE s_getBit      ; always (page hi byte != 0)

; ---------------------------------------------------------------------------
s_readBit:
        ASL s_d3
        ROL s_d3+1
        ASL s_srcBits
        BNE s_gotBit
        ; FLIP 1: src RETREATS. Pre-read decrement via s_decWord (X=0 -> the
        ; s_src word; preserves C = the sentinel bit from the ASL above).
        JSR s_decWord
        LDA (s_src,X)     ; X=0, A = *src
        ROL               ; C=1 (sentinel): A = (byte<<1)|1
        STA s_srcBits
s_gotBit:
        ROL s_d2
        ROL s_d2+1

s_getBit:
        LDA s_d3+1
        BPL s_readBit

        LDA (s_tabs),Y
        STA s_factor+1
        STA s_frac+1
        INC s_tabs+1
        LDA (s_tabs),Y

; ---- slow multiplication (FAST=0), verbatim ----
        STA s_factor
        LDX #4
s_computeFrac:
        LSR s_frac+1
        ROR
        DEX
        BNE s_computeFrac
        STA s_frac

        TXA               ; #0
        STA s_cp+1
        LDX #16
s_mulLoop:
        LSR s_factor+1
        ROR s_factor
        BCC s_mulNext
        CLC
        ADC s_d3
        PHA
        LDA s_cp+1
        ADC s_d3+1
        STA s_cp+1
        PLA
s_mulNext:
        ROR s_cp+1
        ROR
        DEX
        BNE s_mulLoop
        STA s_cp

        EOR #$ff
        SEC
        ADC s_d2

        TAX
        LDA s_d2+1
        SBC s_cp+1
        BCS s_zero

        LDX s_cp
        LDA s_cp+1
        BCC s_setD3       ; always

s_zero:
        STX s_d2
        STA s_d2+1
        LDA s_d3
        SBC s_cp          ; C=1
        TAX
        LDA s_d3+1
        SBC s_cp+1

s_setD3:
        STX s_d3
        STA s_d3+1
        PHP
        LDA (s_tabs),Y
        SBC s_frac
        STA (s_tabs),Y
        DEC s_tabs+1
        LDA (s_tabs),Y
        SBC s_frac+1
        PLP
        BCS s_retZero
        SBC #$ef          ; C=0
        SEC
        .byte $A2         ; LDX-imm opcode swallows the next byte (skips CLC)
s_retZero:
        CLC
        STA (s_tabs),Y
        LDX #0
        RTS

; ---------------------------------------------------------------------------
; s_decWords: 16-bit decrement of the zp words at offsets X, X-2, ..., 2;
; exits with X=0 (the getBit invariant). s_decWord: single word at offset X
; (X preserved). Both preserve C and Y; A is clobbered.
s_decWords:
        JSR s_decWord
        DEX
        DEX
        BNE s_decWords
        RTS

s_decWord:
        LDA s_src,X
        BNE s_dwLo
        DEC s_src+1,X
s_dwLo:
        DEC s_src,X
        RTS

; preamble seed values for zp offsets 0..12 (see full_decomp).
s_initTab:
        .byte <(comp_data + comp_data_len), >(comp_data + comp_data_len)
        .byte <(out_addr + out_len - 1), >(out_addr + out_len - 1)
        .byte 0,0,0,0     ; s_copy / s_number (don't care)
        .byte 0,0         ; s_d2 = 0
        .byte 1,0         ; s_d3 = 1
        .byte 0           ; s_tabs lo = <probs_base = 0
