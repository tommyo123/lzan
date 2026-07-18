; ===========================================================================
; PuCrunch 6502 decruncher - forward, lzan container. CLEAN-ROOM: written from
; the pucrunch token grammar (no original pucrunch assembly consulted), using
; this collection's house tricks: sentinel bit
; buffer, self-modified absolute input/output pointers, and a single ROL-into-A
; multi-bit reader that doubles as the gamma-suffix, literal, extra-bits and
; low-byte reader.
;
; Stream layout (lzan::pucrunch::compress_pucrunch_6502, maxGamma fixed at 7):
;   comp_data+0   startEsc (right-aligned)
;   comp_data+1   escBits
;   comp_data+2   8-escBits (precomputed literal-rest width)
;   comp_data+3   extraLZPosBits
;   comp_data+4   RLE rank table, 15 fixed slots (rank r at comp_data+3+r)
;   comp_data+19  bitstream, MSB first; ends with the EOF token
;
; Token grammar (escape selector E, width e):
;   selector != E                        -> literal (8 bits total)
;   E, gamma=1, 0, ~lo                   -> LZ len 2, dist = (~lo)+1 (<= 256)
;   E, gamma=1, 1, 0, newE, rest         -> escape change + literal (old E used)
;   E, gamma=1, 1, 1, len, bytecode      -> RLE (short/long length forms)
;   E, gamma=a>1, gamma=b<255, x, ~lo    -> LZ len a+1, dist-1 = (b-1)<<(8+x)|mid|lo
;   E, gamma=2,   gamma=255              -> EOF
;
; The 19-byte stream header is laid out exactly like the zp parameter block
; (startEsc, escBits, 8-escBits, extraLZPosBits, rank table), so init is one
; copy loop and the width loads are plain zp reads - nothing is patched at
; init. The rank table must leave the packed block anyway: in-place layouts
; overwrite comp_data during decode while ranked lookups still need it.
;
; The 8-bit low reads always leave C=0 (A starts at 0, so the last ROL shifts
; a 0 out), which the complement-add LZ address math relies on:
;   src = OUT - dist  ==  encodedLow + OUT.lo  /  OUT.hi SBC d.hi (borrow chains)
;
; Proven-state shortcuts this body leans on (all exercised by the hard gates):
;   * gbits exits with X=$FF, and every JSR getval site is downstream of such
;     an exit with X untouched in between, so getval opens with INX, not
;     LDX #0.
;   * the dispatch parks the first gamma value in Y, where every consumer
;     wants it: LZ len-1 for both forms, and the short-RLE high loop count
;     (the value is 1 on that path). getbit preserves A and Y throughout.
;   * CMP #$FF leaves C=0 for every non-EOF group value, so the b-1 step is
;     SBC #0 with no SEC.
;   * the copy length rides in Y (INY turns len-1 into len), and the LZ copy
;     indexes with X - no stack traffic needed.
;
; LEGAL variant: same body as pucrunch-lzan.s - the size pass replaced the
; historical SBX #$FF (copy-count setup) with the Y-parked length, so every
; remaining instruction is a documented 6502 opcode (the $2C byte is BIT abs,
; a plain flag-scrambling skip). Kept as its own registry entry so the
; illegal-opcode policy switch always has an explicitly legal routine to pick.
;
; ONE-SHOT: the input/output SMC operands assemble to their start values and
; are not re-seeded at entry (the SFX pipeline and the test harness both load
; a fresh image per run); the LZ-source and RLE-count operands are rewritten
; before use by every token that needs them. The zp parameter block IS
; re-seeded from the stream. RTS at EOF.
; ===========================================================================
;@format: pucrunch
;@direction: forward
;@variant: legal
;@entry: full_decomp
;@vfy-key: pucrunch-lzan-legal
;@encoder: lzan::pucrunch::compress_pucrunch_6502
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 20
;@scratch: none
;@illegal: no
;@smc: yes
;@code-bytes: 230

; ---- config-defaults ----
zp_base = $E0
; ---- end config-defaults ----

esc    = zp_base+0   ; current escape selector (right-aligned)
re_w   = zp_base+1   ; escBits
lit_w  = zp_base+2   ; 8-escBits (literal rest width)
ex_w   = zp_base+3   ; extraLZPosBits
rtab   = zp_base+4   ; 15 bytes: RLE rank table (zp copy of comp_data+4..18)
bitstr = zp_base+19  ; sentinel bit buffer ($80 = empty)

full_decomp:
        LDX #18              ; header and zp block share one layout:
tcp:
        LDA comp_data,X      ; startEsc, escBits, 8-escBits, extraLZPosBits,
        STA esc,X            ; rank table - a single 19-byte copy seeds all
        DEX                  ; per-stream parameters
        BPL tcp
        LDA #$80
        STA bitstr
        ; fall through into the main token loop

; --- main dispatch -----------------------------------------------------------
main:
        JSR read_esc         ; A = selector
        CMP esc
        BNE lit              ; ordinary literal: finish the byte
        JSR getval           ; a = first control value
        TAY                  ; every branch below wants it in Y: LZ len-1, or
        CMP #1               ; the short-RLE high loop count (a = 1 there)
        BNE lznorm           ; a >= 2: normal LZ / EOF
        JSR getbit
        BCC lz2              ; 0   : LZ length 2
        JSR getbit
        BCC newesc           ; 10  : escape change + literal
        ; 11  : RLE - fall through (Y = 1 from the dispatch TAY)

; --- RLE (len 2..32256) ------------------------------------------------------
rle:
        JSR getval
        CMP #$80
        BCC rs               ; short: A = len-1 (1..127)
        ; long: A = 128 + (nlo >> 1); one more raw bit completes nlo
        ASL                  ; A = (nlo & $FE); the 128 falls off the top
        JSR getbit
        ADC #0               ; ... and the raw bit lands in bit 0
        PHA
        JSR getval           ; (n >> 8) + 1
        TAY
        PLA
rs:
        STA rl_x+1           ; nlo (SMC: rewritten by every RLE token)
        ; byte code: gamma < 16 = table rank, else hi nibble | 4 raw bits
        JSR getval
        CMP #16              ; C=1 for the unranked path, C=0 for ranked
        BCS unrk
        TAX
        LDA rtab-1,X         ; rank table (zp copy)
        BCC remit            ; always (C=0 from the BCS above)
unrk:
        LDX #4
        JSR gbits            ; A = (code << 4) | bits; the top bit of code
remit:                       ; falls off the 8-bit ROL, leaving (code-16)<<4
rl_x:
        LDX #0               ; (rewritten: nlo)
        INX                  ; nlo+1 first pass, then 256 per extra Y pass
rloop:
        JSR putch            ; putch preserves A (the run byte)
        DEX
        BNE rloop
        DEY
        BNE rloop
tomain:
        BEQ main             ; always; doubles as the LZ loop's relay to main

; --- escape change + literal (old escape prefixes the byte) -------------------
newesc:
        LDY esc
        JSR read_esc
        STA esc
        TYA                  ; A = old escape = the literal's selector bits
; --- ordinary literal: A = selector, read the remaining 8-escBits ------------
lit:
        LDX lit_w
        JSR gbits
        JSR putch
        JMP main             ; (not a proven-flag branch: with escBits = 8 the
                             ; rest-read is empty and C is CMP esc leftovers)

; --- LZ length 2 (distance 1..256) --------------------------------------------
; Y = 1 here (the dispatched gamma value) = len-1 for the 2-byte form, so the
; shared tail computes the same count from Y either way.
lz2:
        LDA #0               ; d.hi = 0
        BEQ sethi            ; always

; --- normal LZ (len 3..256) / EOF ---------------------------------------------
; Y = len-1 from the dispatch TAY (getval/gbits/getbit all preserve Y).
lznorm:
        JSR getval           ; b = high position group + 1, or the sentinel
        CMP #$FF
        BEQ pc1              ; EOF: nothing stacked - return via putch's RTS
        SBC #0               ; b-1 (the CMP left C=0: A < $FF here)
        LDX ex_w
        JSR gbits            ; A = (b-1) << extra | middle bits = d.hi
sethi:
        STA lzld+2           ; d.hi, rewritten with src.hi just below
        JSR get8             ; A = encoded low = ~(d.lo), C = 0
        ; src = OUT - dist: OUT.lo + ~(d.lo) = OUT.lo - d.lo - 1 (mod 256) with
        ; carry = NOT borrow, so the SBC completes OUT - d - 1 = OUT - dist.
        ADC putch+1
        STA lzld+1
        LDA putch+2
        SBC lzld+2
        STA lzld+2
        INY                  ; Y = copy length; 256 wraps to 0, which the DEY
        INX                  ; loop turns into 256 iterations; X = $FF (get8
                             ; went through gbits), so INX starts the index at 0
lzloop:
lzld:
        LDA out_addr,X       ; (rewritten: LZ copy source)
        JSR putch
        INX
        DEY
        BNE lzloop
        BEQ tomain           ; always (relay: main is out of branch reach)

; --- output one byte (A, X, Y preserved) ---------------------------------------
putch:
        STA out_addr         ; SMC operand = output position
        INC putch+1
        BNE pc1
        INC putch+2
pc1:
        RTS

; --- bit primitives -------------------------------------------------------------
; get8: 8 stream bits into A (A forced to 0 first) - the LZ low-byte read.
get8:
        LDX #8
        .byte $2C            ; BIT abs (documented): skips the LDX below
; read_esc: A = the next escBits stream bits (0 allowed)
read_esc:
        LDX re_w
        LDA #0
        BEQ gbits            ; always

; getval: bounded gamma, result 1..255 in A (maxGamma = 7). Unary ones count
; the exponent in X, then the suffix bits rotate into A on top of the implicit
; leading 1 - the suffix read IS gbits. X opens at $FF at every call site (see
; the header note), so INX starts the count at 0.
getval:
        LDA #1
        INX                  ; X = 0
gv_u:
        JSR getbit
        BCC gbits            ; 0-terminated: X = suffix length
        INX
        CPX #7
        BNE gv_u             ; at maxGamma the terminator is implicit
        ; falls through into gbits (X = 7 suffix bits)

; gbits: rotate X more stream bits into A from the right (X may be 0).
; Exits with X = $FF. After a full 8-bit read that started from A = 0 the
; final ROL shifts out a 0, so C = 0 on exit (the LZ address math relies on
; this).
gbits:
        DEX
        BMI gb_rts
gb_l:
        JSR getbit
        ROL
        DEX
        BPL gb_l
gb_rts:
        RTS

; getbit: next stream bit into C (A, X, Y preserved). The buffer byte holds
; remaining bits with a 1-sentinel at the bottom; shifting out the sentinel
; (result 0) falls into the refill, whose ROL pushes the new byte's top bit to
; C and re-plants the sentinel from the carry the exhausted buffer left behind.
getbit:
        ASL bitstr
        BNE gb_rts
refill:
        PHA
inp:
        LDA comp_data+19     ; SMC operand = next stream byte
        ROL                  ; C is 1 here (the sentinel just shifted out)
        STA bitstr
        INC inp+1
        BNE rf1
        INC inp+2
rf1:
        PLA
        RTS
