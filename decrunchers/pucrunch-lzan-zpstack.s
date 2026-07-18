; ===========================================================================
; PuCrunch 6502 decruncher - forward, lzan container, EXTRA-SMALL "zp-stack"
; body. Clean-room: written from the pucrunch token grammar (no original
; pucrunch assembly consulted).
; Same stream, same entry ABI as the standard sibling (pucrunch-lzan.s);
; decodes byte-identically. This variant trades cycles and wider zero-page
; usage for bytes so the staged blob fits the $0100 stack-page slot.
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
; The 19-byte stream header is laid out exactly like the zp parameter block,
; so init is one copy loop. The rank table must leave the packed block:
; in-place layouts overwrite comp_data during decode.
;
; Differences from the standard body:
;   * REGISTER SWAP: Y (not X) is the bit-engine width/unary register, so X
;     SURVIVES every bit read (getbit preserves both; gbits/getval count in
;     Y). The dispatch parks the first gamma value in X, where it rides
;     untouched through the whole LZ field decode and becomes the copy count
;     with a single INX - the standard body's park-move-reset dance is gone.
;   * LZ source pointer in zp (`lz`): the copy runs (lz),Y with Y ascending
;     from 0 (hoisted INY turns the guaranteed Y=$FF bit-engine exit state
;     into the 0 start index), and the address math reads/writes 2-byte zp
;     operands instead of 3-byte absolute SMC operands.
;   * dispatch tests a==1 with LSR; on the ==1 path this leaves A=0, which IS
;     the LZ2 d.hi, so the length-2 form branches straight into sethi.
;   * gbits is entered at its bottom DEY so a 0 width falls straight out and
;     the loop body needs no BMI guard; getval's unary loop counts up in Y
;     against maxGamma and falls into gbits as the suffix read.
;   * long-RLE low-byte completion is a bare getbit/ROL merge; the RLE
;     bytecode does the rank-table load speculatively (LDA rtab-1,Y before
;     the ranked/unranked test - a dead read for codes >= 16, and TYA
;     rebuilds the code); the RLE low count parks in zp (`rlc`, aliasing the
;     LZ source low byte - never live at the same time).
;   * the literal tail falls through into `main` (init branches over it), and
;     the bit-buffer byte is an SMC data cell inside the body, seeded to $80
;     at assembly time (single-shot), so init is just the header copy.
;
; The 8-bit low reads always leave C=0 (A starts at 0, so the last ROL shifts
; a 0 out), which the complement-add LZ address math relies on:
;   src = OUT - dist  ==  encodedLow + OUT.lo  /  OUT.hi SBC d.hi (borrow chains)
;
; Proven-state shortcuts (all exercised by the hard gates):
;   * gbits exits with Y=$FF (N=1), and every JSR getval site is downstream
;     of such an exit with Y untouched in between, so getval's INY starts the
;     unary count at 0; the LZ copy's hoisted INY starts the index at 0.
;   * X survives read_esc in newesc (old-esc park) and survives the RLE len
;     and bytecode decodes (a=1 is the short-RLE outer count; the long form
;     overwrites X with the high count).
;   * CMP #$FF leaves C=0 for every non-EOF group value, so b-1 is SBC #0.
;   * init's copy loop exits with X=$FF (N=1), so BMI reaches main for free.
;
; ONE-SHOT: the input/output SMC operands and the bit-buffer cell assemble to
; their start values and are not re-seeded at entry (the SFX pipeline and the
; test harness both load a fresh image per run). The zp parameter block IS
; re-seeded from the stream. RTS at EOF.
; ===========================================================================
;@format: pucrunch
;@direction: forward
;@variant: zp-stack
;@entry: full_decomp
;@vfy-key: pucrunch-lzan
;@encoder: lzan::pucrunch::compress_pucrunch_6502
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 21
;@scratch: none
;@illegal: yes
;@smc: yes
;@code-bytes: 211

; ---- config-defaults ----
zp_base = $E0
; ---- end config-defaults ----

esc    = zp_base+0   ; current escape selector (right-aligned)
re_w   = zp_base+1   ; escBits
lit_w  = zp_base+2   ; 8-escBits (literal rest width)
ex_w   = zp_base+3   ; extraLZPosBits
rtab   = zp_base+4   ; 15 bytes: RLE rank table (zp copy of comp_data+4..18)
lz     = zp_base+19  ; LZ copy source pointer (lo, hi)
rlc    = zp_base+19  ; RLE low count park - aliases lz.lo: an RLE token never
                     ; touches the LZ source, an LZ token never touches rlc

full_decomp:
        LDX #18              ; header and zp block share one layout:
tcp:
        LDA comp_data,X      ; startEsc, escBits, 8-escBits, extraLZPosBits,
        STA esc,X            ; rank table - a single 19-byte copy seeds all
        DEX                  ; per-stream parameters
        BPL tcp
        BMI main             ; always (X=$FF from the copy loop)

; --- escape change + literal (old escape prefixes the byte) -------------------
newesc:
        LDX esc              ; X survives read_esc (the bit engine runs on Y)
        JSR read_esc
        STA esc
        TXA                  ; A = old escape = the literal's selector bits
; --- ordinary literal: A = selector, read the remaining 8-escBits ------------
lit:
        LDY lit_w
        JSR gbits
        JSR putch
        ; falls into the dispatch: the literal tail IS the loop edge

; --- main dispatch -----------------------------------------------------------
main:
        JSR read_esc         ; A = selector
        CMP esc
        BNE lit              ; ordinary literal: finish the byte
        JSR getval           ; a = first control value
        TAX                  ; park a in X: LZ len-1, or the short-RLE outer
        LSR                  ; count (a = 1 there). X survives all bit reads.
        BNE lznorm           ; a >= 2: normal LZ / EOF. a==1 leaves A=0 = the
        JSR getbit           ; LZ2 d.hi, so ...
        BCC sethi            ; 0   : LZ length 2 enters the shared tail direct
        JSR getbit
        BCC newesc           ; 10  : escape change + literal
        ; 11  : RLE - fall through (X = 1 from the dispatch TAX)

; --- RLE (len 2..32256) ------------------------------------------------------
rle:
        JSR getval
        CMP #$80
        BCC rs               ; short: A = len-1 (1..127), X = 1 outer pass
        ; long: A = 128 + (nlo >> 1); one more raw bit completes nlo - the
        ; single ROL merges it while the 128 falls off the top
        JSR getbit
        ROL                  ; A = nlo
        PHA
        JSR getval           ; (n >> 8) + 1
        TAX                  ; X = outer count (overwrites the parked a)
        PLA
rs:
        STA rlc              ; nlo
        ; byte code: gamma < 16 = table rank, else hi nibble | 4 raw bits.
        ; The table load runs speculatively before the range test (a dead
        ; read for code >= 16 - abs,Y may reach into free/page-1 RAM, reads
        ; are side-effect free) and TYA rebuilds the code. X stays live.
        JSR getval
        TAY
        LDA rtab-1,Y         ; rank table (zp copy; abs,Y - no zp,Y LDA)
        CPY #16
        BCC remit            ; ranked: A = run byte
        TYA
        LDY #4
        JSR gbits            ; A = (code << 4) | bits; the top bit of code
remit:                       ; falls off the 8-bit ROL, leaving (code-16)<<4
        LDY rlc
        INY                  ; nlo+1 first pass, then 256 per extra X pass
rloop:
        JSR putch            ; putch preserves A (the run byte)
        DEY
        BNE rloop
        DEX
        BNE rloop
tomain:
        BEQ main             ; always; doubles as the LZ loop's relay to main

; --- normal LZ (len 3..256) / EOF ---------------------------------------------
; X = len-1 from the dispatch TAX (the whole field decode preserves X).
lznorm:
        JSR getval           ; b = high position group + 1, or the sentinel
        CMP #$FF
        BEQ done             ; EOF: nothing stacked - return via putch's RTS
        SBC #0               ; b-1 (the CMP left C=0: A < $FF here)
        LDY ex_w
        JSR gbits            ; A = (b-1) << extra | middle bits = d.hi
; --- shared LZ tail: A = d.hi, X = len-1 ---------------------------------------
sethi:
        STA lz+1             ; park d.hi where src.hi lands anyway
        LDY #8               ; read the encoded low byte: 8 stream bits into
        JSR rd_y             ; A cleared first. A = ~(d.lo), C = 0, Y = $FF
        ; src = OUT - dist: OUT.lo + ~(d.lo) = OUT.lo - d.lo - 1 (mod 256) with
        ; carry = NOT borrow, so the SBC completes OUT - d - 1 = OUT - dist.
        ADC putch+1
        STA lz
        LDA putch+2
        SBC lz+1
        STA lz+1
        INX                  ; X = len; 256 wraps to 0 = 256 loop passes
lzloop:
        INY                  ; hoisted: Y = $FF from rd_y, so the index opens
        LDA (lz),Y           ; at 0 with no LDY #0
        JSR putch
        DEX
        BNE lzloop
        BEQ tomain           ; always (relay: main is out of branch reach)

; --- output one byte (A, X, Y preserved) ---------------------------------------
putch:
        STA out_addr         ; SMC operand = output position
        INC putch+1
        BNE done
        INC putch+2
done:
        RTS

; --- bit primitives -------------------------------------------------------------
; read_esc: A = the next escBits stream bits (0 allowed). rd_y is the
; width-in-Y entry the LZ low-byte read uses (LDY #8 / JSR rd_y).
read_esc:
        LDY re_w
rd_y:
        LDA #0
        BEQ gbits            ; always

; getval: bounded gamma, result 1..255 in A (maxGamma = 7). Unary ones count
; the exponent in Y, then the suffix bits rotate into A on top of the implicit
; leading 1 - the suffix read IS gbits. Y opens at $FF at every call site (see
; the header note), so the loop's INY starts the count at 0.
getval:
        LDA #1
gv_u:
        INY                  ; $FF -> 0 on the first pass
        CPY #7
        BEQ gbits            ; at maxGamma the terminator is implicit
        JSR getbit
        BCS gv_u             ; count the unary ones
        BCC gbits            ; 0-terminated: Y = suffix length (C = 0)

; gbits: rotate Y more stream bits into A from the right (Y may be 0).
; The loop is entered at the bottom (the DEY), so a 0 width falls straight
; out. Exits with Y = $FF. After a full 8-bit read that started from A = 0
; the final ROL shifts out a 0, so C = 0 on exit (the LZ address math relies
; on this).
gb_l:
        JSR getbit
        ROL
gbits:
        DEY
        BPL gb_l
gb_rts:
        RTS

; getbit: next stream bit into C (A, X, Y preserved). The buffer byte holds
; remaining bits with a 1-sentinel at the bottom; shifting out the sentinel
; (result 0) falls into the refill, whose ROL pushes the new byte's top bit to
; C and re-plants the sentinel from the carry the exhausted buffer left behind.
; The buffer lives in an SMC data cell below, assembled to $80 (= empty).
getbit:
        ASL bitstr
        BNE gb_rts
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
bitstr:
        .byte $80            ; bit buffer (SMC data cell), $80 = empty
