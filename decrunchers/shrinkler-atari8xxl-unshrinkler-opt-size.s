; ===========================================================================
; Shrinkler 6502 decruncher ("unShrinkler"), opt-size variant, in asm6502.
; Upstream: unShrinkler (c) 2021 Krzysztof Dudek and Piotr Fusik, zlib
; (a 6502 port of Aske Simon Christensen's Shrinkler).
;
; Smaller than the standard variant: two provably-dead LDY #0 in the copy loops
; are dropped. Decodes the same stream.
;
; Build variant: unshrinkler_PARITY = 0, unshrinkler_FAST = 0
;   (320 B code, 1.5 KB prob data, no parity context, software 16-bit multiply).
; PARITY=0 matches lzan::shrinkler::compress_shrinkler (no-parity stream): there
; is no parity-context byte and the `?getKind`/literal contexts are taken
; straight from `>?probs` (no `(dst&1)` mixing).
;
; The prob tables live in page-aligned scratch RAM (probs_ram, $700 bytes).
; full_decomp seeds src=comp_data, dst=out_addr, then runs init.
; ===========================================================================
;@format: shrinkler
;@direction: forward
;@variant: opt-size
;@entry: full_decomp
;@vfy-key: shrinkler-atari8xxl-unshrinkler
;@encoder: lzan::shrinkler::compress_shrinkler
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 19
;@scratch: symbol=probs_base,len=1536,align=page
;@illegal: no
;@smc: yes
;@code-bytes: 338

; ---- config-defaults ----
zp_base = $40
probs_base = $2000
; ---- end config-defaults ----

; ---- zero page layout (zp_base + 0..18, contiguous) ----
s_src     = zp_base               ; word  (compressed source)
s_dst     = zp_base+2             ; word  (output dest)
s_copy    = zp_base+4             ; word  (copy source during a match)
s_factor  = zp_base+4             ; (aliased; only used in FAST)
s_tabs    = zp_base+6             ; word  (prob table pointer)
s_number  = zp_base+8             ; word
s_cp      = zp_base+10            ; word  (multiply result)
s_d2      = zp_base+12            ; word  (range value)
s_d3      = zp_base+14            ; word  (range size)
s_frac    = zp_base+16            ; word
s_srcBits = zp_base+18            ; bit reservoir

; ---- prob tables: page-aligned scratch RAM (probs_base, $600 bytes). ----
; ?probs       = data+$000  (512 B = 256 ctx x 2 bytes: kind + literal tree)
; ?probsRef    = data+$200  (= ?probsLength: repeated bit + length numbers)
; ?probsOffset = data+$400  (offset numbers)
probs       = probs_base
probsRef    = probs_base + $200
probsLength = probsRef
probsOffset = probsRef + $200

; ---------------------------------------------------------------------------
full_decomp:
        LDA #<comp_data
        STA s_src
        LDA #>comp_data
        STA s_src+1
        LDA #<out_addr
        STA s_dst
        LDA #>out_addr
        STA s_dst+1
        ; fall into init

; ---- init prob tables to $8000 and range state ----
; Original: ldx >?probsOffset+$100 ; mwy #1 ?d3 (=> ?d3=1, Y=0) ; sty ?d2 ;
;           sty ?tabs ; tya (A=0) ; then the alternating $00/$80 page fill.
        LDX #>(probsOffset+$100)
        LDY #0            ; mwy leaves Y = >1 = 0
        LDA #1
        STA s_d3
        STY s_d3+1        ; ?d3 = $0001
        STY s_d2          ; sty ?d2 (low)
        STY s_d2+1        ; clear ?d2 hi too (range value starts 0; orig relies
                          ; on cleared RAM, the emulator may not provide it)
        STY s_tabs        ; <probs_base == 0 (page-aligned) -> tabs low = 0
        TYA               ; A = 0
s_initPage:
        STX s_tabs+1
s_fill:
        STA (s_tabs),Y    ; Y enters as 0 (from prior wrap), fills whole page
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
        STA (s_dst,X)     ; X=0
        ; inw ?dst
        INC s_dst
        BNE s_dst_nohi1
        INC s_dst+1
s_dst_nohi1:
        JSR s_getKind
        BCC s_literal

        LDA #>probsRef
        JSR s_getBitFrom
        BCC s_readOffset

s_readLength:
        LDA #>probsLength
        JSR s_getNumber
s_offsetL:
        LDA #$ff           ; #$ff operand (s_offsetL+1) self-modified by readOffset
        ADC s_dst          ; C=0
        STA s_copy
s_offsetH:
        LDA #$ff           ; #$ff operand (s_offsetH+1) self-modified by readOffset
        ADC s_dst+1
        STA s_copy+1

        LDX s_number+1
        BEQ s_copyRemainder
s_copyPage:
        ; mva:rne (?copy),y (?dst),y+  -> copy a full page (Y=0 on entry from getNumber; wraps to 0 each page)
s_copyPageLoop:
        LDA (s_copy),Y
        STA (s_dst),Y
        INY
        BNE s_copyPageLoop
        INC s_copy+1
        INC s_dst+1
        DEX
        BNE s_copyPage

s_copyRemainder:
        LDX s_number
        BEQ s_copyDone
        ; Y is already 0 here (s_getNumber exits Y=0; s_getBit never touches Y)
s_copyByte:
        LDA (s_copy),Y
        STA (s_dst),Y
        INY
        DEX
        BNE s_copyByte
        TYA
        ; add ?dst -> CLC / ADC ?dst
        CLC
        ADC s_dst
        STA s_dst
        ; scc:inc ?dst+1 -> INC runs when C set
        BCC s_copyDone
        INC s_dst+1

s_copyDone:
        JSR s_getKind
        BCC s_literal

s_readOffset:
        LDA #>probsOffset
        JSR s_getNumber
        LDA #3
        SBC s_number      ; C=0
        STA s_offsetL+1   ; self-modify the `LDA #$ff` (s_offsetL) operand
        TYA               ; #0
        SBC s_number+1
        STA s_offsetH+1   ; self-modify the `LDA #$ff` (s_offsetH) operand
        BCC s_readLength
        RTS               ; finish

; ---------------------------------------------------------------------------
s_getNumber:
        STA s_tabs+1
        LDA #1
        STA s_number
        STY s_number+1    ; #0  (Y must be 0 on entry)
s_getNumberCount:
        ; :2*!unshrinkler_FAST iny  -> TWO inys (context steps base+2,4,6,...)
        INY
        INY
        JSR s_getBit
        BCS s_getNumberCount

s_getNumberBit:
        DEY               ; :!unshrinkler_FAST dey  (value bit at base+2i+1)
        JSR s_getBit
        ROL s_number
        ROL s_number+1
        DEY               ; :!unshrinkler_FAST dey  (back to even)
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
        LDA (s_src,X)     ; X=0
        ; inw ?src
        INC s_src
        BNE s_src_nohi
        INC s_src+1
s_src_nohi:
        ROL               ; C=1
        STA s_srcBits
s_gotBit:
        ROL s_d2
        ROL s_d2+1

s_getBit:
        LDA s_d3+1
        BPL s_readBit

        LDA (s_tabs),Y
        STA s_factor+1    ; mva (?tabs),y ?factor+1
        ; (FAST-only `lsr @` skipped)
        STA s_frac+1
        INC s_tabs+1
        LDA (s_tabs),Y

; ---- slow multiplication (unshrinkler_FAST=0) ----
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
        ; add ?d3 -> CLC / ADC ?d3
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
        .byte $A2         ; dta {ldx #} : LDX-imm opcode swallows the next byte
s_retZero:
        CLC
        STA (s_tabs),Y
        LDX #0            ; :!unshrinkler_FAST ldx #0
        RTS
