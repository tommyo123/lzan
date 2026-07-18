; ===========================================================================
; TSCrunch 6502 decruncher, canonical standard (balanced) decoder, forward,
; non-inplace, in asm6502 syntax.
; Upstream: TSCrunch decrunch.asm (c) 2022 Antonio Savona, Apache-2.0.
;
; Illegal opcodes kept from the canonical decoder:
;   LAX (zp),Y  - load A and X with the same byte (token fetch), and
;   ALR #imm    - AND #imm then LSR A (the RLE/LZ dispatch).
;
; Flag contracts: C=1 out of ALR for RLE/ZERORUN, C=0 for LZ; C=bit1(token) out
; of the second LSR selecting short/long LZ; A=len-1 + C=1 into updatezp_noclc
; adds len to tsput, then X = token byte count advances tsget. The LZ copy is
; one loop (CPY/BCC plus one unrolled final write) shared by LZ2, which seeds
; the length and joins the short-LZ tail at lz_lo.
;
; Token format is identical to the negativecharge-extreme decoder (both decode
; the same `tscrunch -p` / lzan::tscrunch::compress_tscrunch stream). This is
; the SAVON variant, distinct from the NegativeCharge variant.
;
; ZP: tsget(2)=src, tsput(2)=dst, tstemp(1), lzput(2). Entry = full_decomp.
; EOF = TERMINATOR token (0x20) -> done -> RTS.
; ===========================================================================
;@format: tscrunch
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: tscrunch-savon
;@encoder: lzan::tscrunch::compress_tscrunch
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 7
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 160

; ---- config-defaults ----
zp_base = $F8
; ---- end config-defaults ----

tsget  = zp_base+0  ; 2 bytes: source pointer
tsput  = zp_base+2  ; 2 bytes: dest pointer
tstemp = zp_base+4  ; 1 byte: RLE run length / LZ copy length (len-1)
lzput  = zp_base+5  ; 2 bytes: match source pointer

full_decomp:
        LDX #3                  ; seed tsget/tsput (adjacent in ZP) from the
init_loop:
        LDA init_tab,X          ; table; tsget starts at comp_data+1 (the
        STA tsget,X             ; optimal-run byte is read absolutely below)
        DEX
        BPL init_loop
        ; falls through into the token loop (Y is loaded at entry2)

entry2:
        LDY #0
        LAX (tsget),Y           ; token -> A and X

        BMI rleorlz

        CMP #$20
        BCS lz2

        ; literal, token = length 1..31
        TAY
ts_delit_loop:
        LDA (tsget),Y
        DEY
        STA (tsput),Y
        BNE ts_delit_loop

        TXA                     ; A = len (C=0 from BCS), X+1 = token+payload
        INX

updatezp_noclc:
        ADC tsput               ; tsput += A + C  (callers arrange A+C = len)
        STA tsput
        BCC putnoof
        INC tsput+1
        CLC
putnoof:
        TXA
        ADC tsget               ; tsget += X (token byte count)
        STA tsget
        BCC entry2
        INC tsget+1
        BCS entry2              ; always

rleorlz:
        ALR #$7F                ; A = (token&$7F)>>1, C = token bit0
        BCC ts_delz             ; LZ tokens have bit0 clear

        ; RLE: A = runlen-1, C=1
        BEQ optRun              ; runlen-1 == 0 -> ZERORUN

        LDX #2
        INY
        STA tstemp              ; runlen-1
        LDA (tsget),Y           ; fetch rle byte
        LDY tstemp
runStart:
        STA (tsput),Y
ts_derle_loop:
        DEY
        STA (tsput),Y
        BNE ts_derle_loop

        LDA tstemp              ; A = runlen-1, C still 1
        BCS updatezp_noclc      ; always

optRun:
        LDY comp_data           ; optimalRun-1 (first stream byte)
        STY tstemp
        LDX #1
        ; A is zero
        BNE runStart            ; always (X=1)

done:
        RTS

; LZ2: token = $21..$7E (offset 1..94), 2-byte match
lz2:
        BEQ done                ; token $20 = TERMINATOR

        LDX #1                  ; 1-byte token; copy length 2 -> tstemp = 1
        STX tstemp
        ORA #$80
        ADC tsput               ; C=1: lzput = tsput - (127-token)
        JMP lz_lo               ; join the short-LZ tail (X=1, C -> hi byte)

; LZ
ts_delz:
        LSR                     ; A = len-1 (short) / (len-1)>>1 (long)
        STA tstemp
        INY

        LDA tsput
        BCC long                ; C = token bit1: clear -> 3-byte long LZ

        SBC (tsget),Y           ; C=1: lzput = tsput - offset
        LDX #2
lz_lo:
        STA lzput
        LDA tsput+1
        SBC #$00
        ; lz MUST decrunch forward
lz_put:
        STA lzput+1
        LDY #0
ts_delz_loop:
        LDA (lzput),Y
        STA (tsput),Y
        INY
        CPY tstemp              ; len-1
        BCC ts_delz_loop
        LDA (lzput),Y           ; final byte, Y = len-1, C = 1
        STA (tsput),Y

        TYA                     ; A = len-1 (>0), C=1
        BNE updatezp_noclc      ; always

long:
        ; carry is clear and compensated for from the encoder
        ADC (tsget),Y
        STA lzput
        INY
        LAX (tsget),Y
        ORA #$80
        ADC tsput+1

        CPX #$80                ; C = length low bit
        ROL tstemp              ; tstemp = len-1
        LDX #3

        BNE lz_put              ; always

init_tab:
        .byte <(comp_data+1), >(comp_data+1), <out_addr, >out_addr
