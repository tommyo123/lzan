; ===========================================================================
; TSCrunch 6502 decruncher ("extreme", forward, non-inplace), in asm6502 syntax.
; Upstream: NegativeCharge BeebAsm/Acorn port of Antonio Savona's TSCrunch,
; Apache-2.0.
;
; Uses the illegal opcodes LAX (tsget),Y and ALR #$7F.
;
; The `full_decomp` preamble seeds tsget = comp_data, tsput = out_addr, then
; JSRs tsdecrunch. Self-modifying operand/opcode labels:
;   optRun  -> the `LDY #255` whose operand byte (optRun+1) is patched with
;              the optimal run length;
;   optOdd  -> the `BNE odd3` whose OPCODE byte is patched to $D0 (BNE) or
;              $29 (AND #imm) to flip parity handling;
;   lzto    -> the `CPY #0` whose operand byte (lzto+1) is the match length.
;
; ZP: tsget(2)=src, tstemp(1), tsput(2)=dst, lzput(2). Entry = full_decomp.
; EOF = TERMINATOR token -> done -> RTS.
; ===========================================================================
;@format: tscrunch
;@direction: forward
;@variant: opt-speed
;@entry: full_decomp
;@vfy-key: tscrunch-negativecharge-extreme
;@encoder: lzan::tscrunch::compress_tscrunch
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 7
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 261

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

        LDX #$D0                 ; bne opcode
        AND #1
        BNE skp1
        LDX #$29                 ; and-immediate opcode
skp1:
        STX optOdd

        INC tsget
        BNE entry2
        INC tsget+1

entry2:
        LAX (tsget),Y            ; A=X=mem in one op (was LDA+TAX)

        BMI rleorlz

        CMP #$20
        BCS lz2
        ; literal (non-inplace)
        TAY

        AND #1
        BNE odd1

ts_delit_loop:
        LDA (tsget),Y
        DEY
        STA (tsput),Y
odd1:
        LDA (tsget),Y
        DEY
        STA (tsput),Y

        BNE ts_delit_loop

        TXA
        INX

updatezp_noclc:
        ADC tsput
        STA tsput
        BCS updateput_hi
putnoof:
        TXA
update_getonly:
        ADC tsget
        STA tsget
        BCC entry2
        INC tsget+1
        BCS entry2

updateput_hi:
        INC tsput+1
        CLC
        BCC putnoof

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
        BCS lz2_put_hi
skp2:
        INC tsget
        BNE entry2
        INC tsget+1
        BNE entry2

lz2_put_hi:
        INC tsput+1
        BCS skp2

rleorlz:
        ALR #$7F                 ; (A & $7F) >> 1, C=bit0 of A (was AND+LSR)
        BCC ts_delz

        ; RLE
        BEQ zeroRun

plain:
        INY
        STA tstemp               ; number of bytes to de-rle

        LSR                      ; c = test parity

        LDA (tsget),Y            ; fetch rle byte
        LDY tstemp
runStart:
        STA (tsput),Y

        BCS odd
        SEC

ts_derle_loop:
        DEY
        STA (tsput),Y
odd:
        DEY
        STA (tsput),Y

        BNE ts_derle_loop

        ; update zero page with a = runlen, x = 2, y = 0
        LDA tstemp
        LDX #2
        BCS updatezp_noclc

done:
        RTS

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

        LDA lzto+1
        LSR
        BCS odd2

        LDA (lzput),Y
        STA (tsput),Y
ts_delz_loop:
        INY

odd2:
        LDA (lzput),Y
        STA (tsput),Y

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

zeroRun:
optRun:
        LDY #255
        STA (tsput),Y
optOdd:
        BNE odd3
ts_dezero_loop:
        DEY
        STA (tsput),Y
odd3:
        DEY
        STA (tsput),Y
        BNE ts_dezero_loop

        LDA optRun+1

        LDX #1
        JMP updatezp_noclc

long:
        ; carry is clear and compensated for from the encoder
        ADC (tsget),Y
        STA lzput
        INY
        LAX (tsget),Y            ; A=X=mem in one op (was LDA+TAX)
        ORA #$80
        ADC tsput+1

        CPX #$80
        ROL lzto+1
        LDX #3

        BNE lz_put
