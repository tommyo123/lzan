; ===========================================================================
; Legal-only variant of tscrunch-savon-backward.s (no undocumented opcodes):
; same structure (shared LZ copy engine, init table, stack-stashed remainder
; byte, always-taken proven-flag branches, no JMPs), with the two
; `LAX (tsget),Y` illegal loads expanded to `LDA (tsget),Y` / `TAX` and the one
; `ALR #$7F` to `AND #$7F` / `LSR` (each pair reproduces the illegal op's
; A, X, Z, N and C exactly). Decodes the same stream
; (lzan::tscrunch::compress_tscrunch_backward). For CPUs without illegal opcodes.
; Upstream: TSCrunch decrunch.asm (c) 2022 Antonio Savona, Apache-2.0.
;
; BACKWARD / IN-PLACE variant (SAVON canonical).
; Decodes the stream produced by lzan::tscrunch::compress_tscrunch_backward
; (== `tscrunch -p -i`, the in-place layout). NOTE: TSCrunch's in-place /
; "backward" format is decoded FORWARD (ascending) - it is NOT a reverse-in/
; reverse-out descending stream. The token loop is IDENTICAL to the forward
; savon opt-size baseline; the only in-place work is (a) an in-place header
; preamble that ignores the dest address embedded in the stream and uses
; out_addr instead, and (b) a literal-tail copy after the TERMINATOR.
;
; Stream layout (compress_tscrunch_backward, at comp_data):
;   comp_data+0..1  [load_to]         PRG load addr (ignored)
;   comp_data+2..3  [addr]            original dest addr (ignored; we use out_addr)
;   comp_data+4     [optRun-1]        -> optRun+1 self-mod operand
;   comp_data+5     [remainder_byte]  written just after the token body
;   comp_data+6..   <tokens...>       identical to the forward token stream
;                   [TERMINATOR=$20]
;                   <literal tail>    trailing raw bytes copied to fill out_addr
;
; Structure notes:
;   * remainder_byte (P[5]) is stashed on the STACK at entry (PHA) and pulled
;     at `done` (PLA). It must be read early: in the in-place layout (packed
;     end-aligned with the output end) the token body overwrites the packed
;     header long before the tail copy runs.
;   * pointer inits (tsget=comp_data+6, tsput=out_addr) via a 4-byte table and
;     a zp,X loop - tsget/tsput are adjacent in ZP for this.
;   * tsget is seeded directly to the first token (comp_data+6); no inc_get
;     preamble hop (lz2 no longer JMPs there either).
;   * ONE shared LZ copy engine (lz_put): a rotated loop `copy, INY, CPY, BNE`
;     plus one trailing copy. Works for LZ (run>=3, count=run-1 in tstemp) and
;     for LZ2 (falls in with tstemp=1, X=1 => token len 1). Short LZ hops over
;     lz2 with BCS (carry provably set: match source never underflows $0000).
;   * run count lives in tstemp (CPY tstemp) instead of an SMC CPY operand -
;     tstemp is free during LZ decode (only RLE/zerorun use it) - and the LZ
;     update tail reuses the RLE tail's LDA tstemp (both hold count-1).
;   * all update tails end in always-taken branches on proven flags (carry set
;     by the CPY-equal loop exit / the AND+LSR pair; Z set by DEY-to-zero) -
;     no JMPs at all.
;   * layout keeps every branch in range so the assembler's long-branch
;     relaxation never fires.
;   * `done` (TERMINATOR) is a store/advance/test loop: writes remainder_byte
;     first, then streams the raw tail; the byte is pre-loaded and the 16-bit
;     end test uses CPX so A survives. The final pass pre-loads one byte past
;     the tail - harmless, it is never stored.
;
; ZP: tsget(2)=src, tsput(2)=dst, tstemp(1), lzput(2). Entry = full_decomp.
; ===========================================================================
;@format: tscrunch
;@direction: backward
;@variant: legal
;@entry: full_decomp
;@vfy-key: tscrunch-legal-backward
;@encoder: lzan::tscrunch::compress_tscrunch_backward
;@payload: dst-in-stream
;@eof: length
;@needs: comp_data,out_addr,out_len
;@zp-len: 7
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 207

; ---- config-defaults ----
zp_base = $F8
; ---- end config-defaults ----

tsget  = zp_base+0  ; 2 bytes: source pointer   (adjacent to tsput: init loop)
tsput  = zp_base+2  ; 2 bytes: dest pointer
tstemp = zp_base+4  ; 1 byte: RLE count / LZ copy count (run-1)
lzput  = zp_base+5  ; 2 bytes: match source pointer

out_end = out_addr + out_len

; --- in-place header preamble -----------------------------------------------
full_decomp:
        LDA comp_data+4          ; P[4] = optRun-1
        STA optRun+1
        LDA comp_data+5          ; P[5] = remainder_byte: stash NOW (see notes)
        PHA

        LDX #3                   ; tsget = comp_data+6 (first token),
init_lp:
        LDA inittab,X            ; tsput = out_addr
        STA tsget,X
        DEX
        BPL init_lp

        LDY #0
        ; fall through into the token loop

; --- forward token loop (opt-size savon body) -------------------------------
entry2:
        LDA (tsget),Y            ; legal expansion of LAX (tsget),Y:
        TAX                      ; A = X = token (Z/N from token)

        BMI rleorlz

        CMP #$20
        BCS lz2

        ; literal
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
        AND #$7F                 ; legal expansion of ALR #$7F:
        LSR                      ; (A & $7F) >> 1  (C/Z as ALR)
        BCC ts_delz
        BEQ optRun

        ; RLE
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

rle_tail:
        LDA tstemp               ; RLE: count-1; LZ hops in here: tstemp = run-1
        BCS updatezp_noclc       ; always taken (carry set from AND+LSR / CPY exit)

optRun:
        LDY #255                 ; self-mod operand = optRun-1 (A is zero here)
        STY tstemp

        LDX #1

        BNE runStart             ; always (X != 0)

; --- in-place tail (TERMINATOR reached; Y=0) --------------------------------
; tsget points AT the TERMINATOR, tsput at the end of the token body. Write the
; remainder byte (pulled from the entry-time stash), then stream the raw tail
; (after TERMINATOR) until tsput == out_end. Loop invariant: A holds the byte
; to store this pass. CPX keeps A intact; back-edge is a plain branch.
done:
        PLA                      ; first byte = remainder_byte (stashed at entry)
dt_loop:
        STA (tsput),Y
        INC tsget                ; advance src (first pass: past the TERMINATOR)
        BNE ds1
        INC tsget+1
ds1:
        LDA (tsget),Y            ; pre-load next byte (kept in A across the test)
        INC tsput
        BNE ds2
        INC tsput+1
ds2:
        LDX tsput                ; tsput == out_end ?  (CPX leaves A intact)
        CPX #<out_end
        BNE dt_loop
        LDX tsput+1
        CPX #>out_end
        BNE dt_loop
        RTS

; LZ
ts_delz:
        LSR
        STA tstemp               ; copy count = run-1

        INY

        LDA tsput
        BCC long

        SBC (tsget),Y
        STA lzput
        LDA tsput+1

        SBC #$00

        LDX #2
        BCS lz_put               ; always (match source never borrows past $0000)

; LZ2 - reuses the LZ copy engine: run-1 = 1, token length = 1
lz2:
        BEQ done

        LDX #1
        STX tstemp
        ORA #$80
        ADC tsput
        STA lzput
        LDA tsput+1
        SBC #$00
        ; fall through into lz_put

        ; lz MUST decrunch forward
lz_put:
        STA lzput+1

        LDY #0
ts_delz_loop:
        LDA (lzput),Y
        STA (tsput),Y
        INY
        CPY tstemp
        BNE ts_delz_loop

        LDA (lzput),Y            ; trailing copy at Y = run-1
        STA (tsput),Y

        LDY #0
        BCS rle_tail             ; carry set from the CPY exit; A reloads as
                                 ; tstemp = run-1 there -> updatezp_noclc

long:
        ; carry is clear and compensated for from the encoder
        ADC (tsget),Y
        STA lzput
        INY
        LDA (tsget),Y            ; legal expansion of LAX (tsget),Y:
        TAX                      ; A = X = byte (CPX #$80 below uses X)
        ORA #$80
        ADC tsput+1

        CPX #$80
        ROL tstemp               ; copy count = run-1 (parity from bit7)
        LDX #3

        BNE lz_put               ; always (X != 0)

inittab:
        .word comp_data+6, out_addr
