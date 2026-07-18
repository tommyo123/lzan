; ===========================================================================
; TSCrunch 6502 decruncher, canonical standard (balanced) decoder, forward,
; non-inplace, opt-size variant, in asm6502 syntax.
; Upstream: TSCrunch decrunch.asm (c) 2022 Antonio Savona, Apache-2.0.
;
; Smaller than the standard variant: the lz2 pointer tail reuses the inc_get
; updater and the literal/RLE update falls through into putnoof.
;
; Uses the illegal opcodes
;   LAX (zp),Y  - load A and X with the same byte (token fetch), and
;   ALR #imm    - AND #imm then LSR A (the RLE/LZ dispatch).
;
; The `full_decomp` preamble seeds tsget = comp_data, tsput = out_addr, then
; JSRs tsdecrunch (so the harness needs no register/ZP setup). Self-modifying
; operand labels: optRun (the `LDY #255` whose operand byte optRun+1 holds the
; optimal run length, seeded from the first stream byte) and lzto (the `CPY #0`
; whose operand byte lzto+1 is the match length).
;
; Token format is identical to the negativecharge-extreme decoder (both decode
; the same `tscrunch -p` / lzan::tscrunch::compress_tscrunch stream). This is
; the SAVON variant, distinct from the NegativeCharge variant.
;
; ZP: tsget(2)=src, tstemp(1), tsput(2)=dst, lzput(2). Entry = full_decomp.
; EOF = TERMINATOR token (0x20) -> done -> RTS.
; ===========================================================================
;@format: tscrunch
;@direction: forward
;@variant: opt-size
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
;@code-bytes: 205

; ---- config-defaults ----
zp_base = $F8
; ---- end config-defaults ----

tsget  = zp_base+0  ; 2 bytes: source pointer
tstemp = zp_base+2  ; 1 byte
tsput  = zp_base+3  ; 2 bytes: dest pointer
lzput  = zp_base+5  ; 2 bytes: match source pointer

full_decomp:
        LDA #<comp_data
        STA tsget
        LDA #>comp_data
        STA tsget+1
        LDA #<out_addr
        STA tsput
        LDA #>out_addr
        STA tsput+1
        ; fall through into tsdecrunch

tsdecrunch:
decrunch:
        LDY #0

        LDA (tsget),Y
        STA optRun+1

inc_get:
        INC tsget
        BNE entry2
        INC tsget+1

entry2:
        LAX (tsget),Y

        BMI rleorlz

        CMP #$20
        BCS lz2

        ; literal (non-inplace)
        TAY

ts_delit_loop:
        LDA (tsget),Y
        DEY
        STA (tsput),Y

        BNE ts_delit_loop

        TXA
        INX

updatezp_noclc:
        ADC tsput
        STA tsput
        BCC putnoof
        INC tsput+1
        CLC
putnoof:
        TXA
update_getonly:
        ADC tsget
        STA tsget
        BCC entry2
        INC tsget+1
        BCS entry2

rleorlz:
        ALR #$7F
        BCC ts_delz

        ; RLE
        BEQ optRun

plain:
        LDX #2
        INY
        STA tstemp               ; number of bytes to de-rle

        LDA (tsget),Y            ; fetch rle byte
        LDY tstemp
runStart:
        STA (tsput),Y

ts_derle_loop:
        DEY
        STA (tsput),Y

        BNE ts_derle_loop

        ; update zero page with a = runlen, x = 2, y = 0
        LDA tstemp

        BCS updatezp_noclc

done:
        RTS

; LZ2
lz2:
        BEQ done

        ORA #$80
        ADC tsput
        STA lzput
        LDA tsput+1
        SBC #$00
        STA lzput+1

        ; y already zero
        LDA (lzput),Y
        STA (tsput),Y
        INY
        LDA (lzput),Y
        STA (tsput),Y

        TYA
        DEY

        ADC tsput
        STA tsput
        BCC skp2
        INC tsput+1
skp2:
        JMP inc_get

; LZ
ts_delz:
        LSR
        STA lzto+1

        INY

        LDA tsput
        BCC long

        SBC (tsget),Y
        STA lzput
        LDA tsput+1

        SBC #$00

        LDX #2
        ; lz MUST decrunch forward
lz_put:
        STA lzput+1

        LDY #0

        LDA (lzput),Y
        STA (tsput),Y

        INY
        LDA (lzput),Y
        STA (tsput),Y

ts_delz_loop:
        INY

        LDA (lzput),Y
        STA (tsput),Y

lzto:
        CPY #0
        BNE ts_delz_loop

        TYA

        ; update zero page with a = runlen, x = 2, y = 0
        LDY #0
        ; clc not needed: len-1 in A (from encoder), C = 1
        JMP updatezp_noclc

optRun:
        LDY #255
        STY tstemp

        LDX #1
        ; A is zero

        BNE runStart

long:
        ; carry is clear and compensated for from the encoder
        ADC (tsget),Y
        STA lzput
        INY
        LAX (tsget),Y
        ORA #$80
        ADC tsput+1

        CPX #$80
        ROL lzto+1
        LDX #3

        BNE lz_put
