; ===========================================================================
; LZSA1 raw-block 6502 decruncher, SPEED-optimized, forward, in asm6502 syntax.
; Upstream: decompress_faster_v1.asm / lzsa1_6502.s (John Brandwood), the
; "normal" (LZSA_SMALL_SIZE=0) fast path. Same raw LZSA1 stream our encoder
; emits (lzsa -r -f1) - interchangeable with lzsa1-marty-small.s at the byte
; level, but faster: literals and matches are copied with (zp),Y page loops
; instead of a per-byte JSR GETPUT/PUTDST (~1.5-2x on match-heavy data).
;
;   LZSA1 faster decompressor  Copyright (C) John Brandwood 2021.
;   Distributed under the Boost Software License, Version 1.0.
;   (See http://www.boost.org/LICENSE_1_0.txt)
;
; This is a CALLER-SEEDED decoder (`;@seed: caller`, matching Brandwood's native
; "caller sets lzsa_srcptr/lzsa_dstptr" contract): the caller - the lzan-c64
; framework's shared seed, or the test harness - seeds src (= comp_data) at
; zp_base+0 and dst (= out_addr) at zp_base+2 before entry, so the body carries
; no seed preamble and matches the upstream 191-byte size. Only legal opcodes.
; Self-modifying: the literal page-count operand (cp_npages+1), which self-heals
; to 0 after each literal run. Entry = full_decomp; normal exit is via the
; in-stream EOF (finished) which pops back to the caller.
; ===========================================================================
;@format: lzsa1
;@direction: forward
;@variant: opt-speed
;@entry: full_decomp
;@vfy-key: lzsa1-brandwood-faster
;@encoder: lzan::lzsa1::compress_lzsa1
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 7
;@scratch: none
;@illegal: no
;@smc: yes
;@seed: caller
;@code-bytes: 191

; ---- config-defaults ----
zp_base = $F9
; ---- end config-defaults ----

lzsa_srcptr = zp_base+0  ; 2 bytes: compressed source pointer (caller-seeded = comp_data)
lzsa_dstptr = zp_base+2  ; 2 bytes: output pointer (caller-seeded = out_addr)
lzsa_winptr = zp_base+4  ; 2 bytes: match window pointer (== lzsa_offset)
lzsa_offset = lzsa_winptr
lzsa_cmdbuf = zp_base+6  ; 1 byte : the command/token byte

; Entry: the caller has already seeded src (zp_base+0) = comp_data and
; dst (zp_base+2) = out_addr.
full_decomp:
lzsa1_unpack:
        LDY #0                          ; source index / copy index
        LDX #0                          ; hi-byte of length (X=0 invariant)

cp_length:
        LDA (lzsa_srcptr),Y
        INC lzsa_srcptr
        BNE cp_skip0
        INC lzsa_srcptr+1
cp_skip0:
        STA lzsa_cmdbuf                 ; preserve token
        AND #$70                        ; literal length field
        LSR                             ; set CC before the branch
        BEQ lz_offset                   ; no literals -> straight to match
        LSR
        LSR
        LSR                             ; A = 3-bit literal length
        CMP #$07                        ; extended?
        BCC cp_got_len
        JSR get_length                  ; X=0, CS from CMP, returns CC
        STX cp_npages+1                 ; hi-byte of literal length (SMC)
cp_got_len:
        TAX                             ; lo-byte of length
cp_byte:
        LDA (lzsa_srcptr),Y
        STA (lzsa_dstptr),Y
        INC lzsa_srcptr
        BNE cp_skip1
        INC lzsa_srcptr+1
cp_skip1:
        INC lzsa_dstptr
        BNE cp_skip2
        INC lzsa_dstptr+1
cp_skip2:
        DEX
        BNE cp_byte
cp_npages:
        LDA #0                          ; full pages of literals left? (SMC operand)
        BEQ lz_offset
        DEC cp_npages+1
        BCC cp_byte                     ; always taken

lz_offset:
        LDA (lzsa_srcptr),Y             ; offset-lo
        INC lzsa_srcptr
        BNE offset_lo
        INC lzsa_srcptr+1
offset_lo:
        STA lzsa_offset
        LDA #$FF                        ; assume 8-bit offset (hi = $FF)
        BIT lzsa_cmdbuf
        BPL offset_hi
        LDA (lzsa_srcptr),Y             ; 16-bit offset: real hi byte
        INC lzsa_srcptr
        BNE offset_hi
        INC lzsa_srcptr+1
offset_hi:
        STA lzsa_offset+1
lz_length:
        LDA lzsa_cmdbuf                 ; X=0 from previous loop
        AND #$0F
        ADC #$03                        ; always CC from the copy loop
        CMP #$12                        ; extended?
        BCC got_lz_len
        JSR get_length                  ; X=0, CS from CMP, returns CC
got_lz_len:
        INX                             ; hi-byte of (length+256)
        EOR #$FF                        ; negate the lo-byte of length
        TAY
        EOR #$FF
get_lz_dst:
        ADC lzsa_dstptr                 ; address of the partial page
        STA lzsa_dstptr
        INY
        BCS get_lz_win
        BEQ get_lz_win                  ; lo-byte of length zero?
        DEC lzsa_dstptr+1
get_lz_win:
        CLC                             ; address of the match (offset negative)
        ADC lzsa_offset
        STA lzsa_winptr
        LDA lzsa_dstptr+1
        ADC lzsa_offset+1
        STA lzsa_winptr+1
lz_byte:
        LDA (lzsa_winptr),Y
        STA (lzsa_dstptr),Y
        INY
        BNE lz_byte
        INC lzsa_dstptr+1
        DEX                             ; full pages left?
        BNE lz_more
        JMP cp_length                   ; loop to the top
lz_more:
        INC lzsa_winptr+1
        BNE lz_byte                     ; always taken

; --- get 16-bit length in X:A, returns CC. X=0 expected on entry ---
get_length:
        CLC
        ADC (lzsa_srcptr),Y
        INC lzsa_srcptr
        BNE skip_inc
        INC lzsa_srcptr+1
skip_inc:
        BCC got_length                  ; no overflow -> done
        CLC                             ; MUST return CC
        TAX                             ; preserve overflow value
extra_byte:
        JSR get_byte
        PHA
        TXA                             ; overflow to 256 or 257?
        BEQ extra_word
check_length:
        PLA                             ; length-lo
        BNE got_length
        DEX                             ; one less page loop if zero
got_length:
        RTS
extra_word:
        JSR get_byte
        TAX
        BNE check_length                ; length-hi == 0 at EOF
finished:
        PLA                             ; pop the PHA'd length-lo, then
        PLA                             ; get_length's return address; RTS then
        PLA                             ; returns to the unpacker's caller
        RTS
get_byte:
        LDA (lzsa_srcptr),Y
        INC lzsa_srcptr
        BNE got_byte
        INC lzsa_srcptr+1
got_byte:
        RTS
