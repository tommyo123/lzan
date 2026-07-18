; ===========================================================================
; Exomizer (v2/v3) 6502 decruncher, backward / in-place ("mem" form), asm6502.
; Upstream: exodecrunch.s (c) 2002-2020 Magnus Lind, zlib.
;
; Mirror of exomizer-lind-mem-forward.s: table generation is identical; the
; three direction aspects are flipped to decode the backward stream that
; lzan::exo3::compress_exo3_backward emits (== `exomizer raw -d -b`).
; Build options: INLINE_GET_BITS=0, LITERAL_SEQUENCES_NOT_USED=0,
; MAX_SEQUENCE_LENGTH_256=0, EXTRA_TABLE_ENTRY_FOR_LENGTH_THREE=0,
; DONT_REUSE_OFFSET=0, DECRUNCH_FORWARDS=0, ENABLE_SPLIT_ENCODING=0.
;
; The three direction flips vs the forward form:
;   1. get_crunched_byte reads DOWN: the self-modifying LDA operand is
;      pre-decremented (decrement first, then load), so no A save is needed;
;      full_decomp seeds the operand one past the last compressed byte. The
;      low operand is tested for the borrow before decrementing. LDA/BNE/DEC/DEC
;      touch only Z/N (not C/V), so carry/overflow survive.
;   2. Match offset is ADDED to dest: writing downward, a back-reference lies
;      at a higher address, so match_source = dst + offset. The offset math
;      drops the forward CLC/EOR#$ff complement and instead ADCs tabl_lo/hi
;      then ADCs zp_dest_hi (src = dest + offset).
;   3. Copy walks DOWNWARD: dest (via Y + zp_dest_hi) and src (via Y +
;      zp_src_hi) decrement; the copy loop pre-decrements Y (tya/bne/dey) and
;      DECs the page bytes on wrap, instead of iny/inc.
;
; get_bit is one shared subroutine for the four bit fetches (get_bits inner
; loop, the tag-index gamma loop, the offset-index loop, the reuse-bit read),
; returning the bit in C with A,X,Y preserved. get_bits falls through into
; get_crunched_byte (no JMP); its early-out RTS is get_bit's RTS via an inverted
; BVC. A single literal byte is routed through the main copy loop as a length-1
; literal-sequence copy. The 156-byte decrunch_table sits at $0334 (C64 tape
; buffer).
;
; Calling convention (harness): full_decomp seeds the source self-mod operand
; to comp_data+comp_data_len (one past the last compressed byte;
; get_crunched_byte pre-decrements, so the first fetch returns the last byte)
; and zp_dest to out_addr+out_len (one past the last output byte, because the
; copy loop pre-decrements Y before each write, so the first write lands on
; out_addr+out_len-1). Then it falls into decrunch. comp_data_len and out_len
; are harness constants. On exit the output fills [out_addr, out_addr+out_len).
; ===========================================================================
;@format: exomizer
;@direction: backward
;@variant: standard
;@entry: full_decomp
;@vfy-key: exomizer-backward
;@encoder: lzan::exo3::compress_exo3_backward
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr,comp_data_len,out_len
;@zp-len: 9
;@scratch: symbol=table_ram,len=156,align=none
;@illegal: no
;@smc: yes
;@code-bytes: 291

; ---- config-defaults ----
zp_base = $f7
table_ram = $0334
; ---- end config-defaults ----

; ---- zero page layout: one contiguous 9-byte span at zp_base ----
; (Original source scattered these across $9e/$a7/$ae/$fd; compacted here.
; zp_bitbuf/zp_dest keep their original $fd/$fe/$ff at the default zp_base:
; the dest pointer pair must stay adjacent for (zp_dest_lo),Y, the src pair
; for (zp_src_lo),Y. Same layout as the forward mirror.)
zp_len_lo   = zp_base + 0
zp_len_hi   = zp_base + 1
zp_src_lo   = zp_base + 2
zp_src_hi   = zp_base + 3
zp_bits_hi  = zp_base + 4
zp_ro_state = zp_base + 5
zp_bitbuf   = zp_base + 6
zp_dest_lo  = zp_base + 7
zp_dest_hi  = zp_base + 8

; encoded_entries = 52 (EXTRA_TABLE_ENTRY_FOR_LENGTH_THREE=0)
encoded_entries = 52

; Decrunch table in scratch RAM (156 bytes), base exposed as table_ram in the
; config-defaults block above ($0334 default = C64 tape buffer, free here).
; tabl_bi/lo/hi are the three 52-entry sub-tables.
tabl_bi = table_ram
tabl_lo = table_ram + encoded_entries
tabl_hi = table_ram + encoded_entries * 2

; ---------------------------------------------------------------------------
; get_bit: shift the next stream bit out of zp_bitbuf into C, refilling the
; buffer from the stream when it runs empty. Preserves A,X,Y (and V). This is
; the shared body of the four former inline "ASL zp_bitbuf / refill" copies.
; ---------------------------------------------------------------------------
get_bit:
        ASL zp_bitbuf
        BNE gbit_rts            ; buffer not empty: C = extracted bit
        PHA
        JSR get_crunched_byte
        ROL                     ; C (marker bit) -> bit0, C = fetched bit7
        STA zp_bitbuf
        PLA
gbit_rts:
        RTS

; ---------------------------------------------------------------------------
; get_bits (INLINE_GET_BITS=0 out-of-line form) -- same semantics as forward.
; Falls through into get_crunched_byte for the ">8 bits" tail (the forward
; baseline's JMP), and exits through get_bit's RTS otherwise.
get_bits:
        ADC #$80                ; needs c=0, affects v
        ASL
        BPL gb_skip
gb_next:
        JSR get_bit
        ROL
        BMI gb_next
gb_skip:
        BVC gbit_rts
        SEC
        STA zp_bits_hi
        ; fall through into get_crunched_byte

; ---------------------------------------------------------------------------
; get_crunched_byte: return next compressed byte in A, preserving X,Y,C,V.
; BACKWARD: PRE-DECREMENT the 16-bit operand of an absolute self-modifying
; LDA (so Y is untouched), then load. The low operand is tested for the
; borrow BEFORE decrementing. LDA/BNE/DEC/DEC touch only Z/N (not C/V), so
; the caller's carry/overflow survive.
; ---------------------------------------------------------------------------
get_crunched_byte:
        LDA gcb_lda+1           ; current low byte of the operand
        BNE gcb_nodecr          ; nonzero => no borrow into high byte
        DEC gcb_lda+2
gcb_nodecr:
        DEC gcb_lda+1
gcb_lda:
        LDA $FFFF
        RTS

; ---------------------------------------------------------------------------
; full_decomp: seed pointers ONE PAST the last bytes, then decrunch. BACKWARD
; stream is read from the last byte of comp_data downward; output is written
; from the last byte of the out buffer downward. The dest low byte is parked
; on the stack until the prepare block below (table generation clobbers Y).
full_decomp:
        LDA #<(comp_data + comp_data_len)
        STA gcb_lda+1
        LDA #>(comp_data + comp_data_len)
        STA gcb_lda+2
        LDA #<(out_addr + out_len)
        PHA
        LDA #>(out_addr + out_len)
        STA zp_dest_hi
decrunch:
        ; RAW stream: only the initial bit-buffer byte is present (no 2-byte
        ; load address). Read it straight into zp_bitbuf.
        JSR get_crunched_byte
        STA zp_bitbuf

; calculate tables. IDENTICAL to forward. (X entering is dead: iteration 0
; takes the BEQ shortcut because Y=0, and LSR/TAX rewrites X before any read.)
        LDY #0
        CLC
table_gen:
        TAX
        TYA
        AND #$0f
        STA tabl_lo,Y
        BEQ shortcut            ; start a new sequence
        TXA
        ADC tabl_lo - 1,Y
        STA tabl_lo,Y
        LDA zp_len_hi
        ADC tabl_hi - 1,Y
shortcut:
        STA tabl_hi,Y
        LDA #$01
        STA zp_len_hi
        LDA #$78                ; %01111000
        JSR get_bits
        LSR
        TAX
        BEQ rolled
        PHP
rolle:
        ASL zp_len_hi
        SEC
        ROR
        DEX
        BNE rolle
        PLP
rolled:
        ROR
        STA tabl_bi,Y
        BMI no_fixup_lohi
        LDA zp_len_hi
        STX zp_len_hi
        .byte $24               ; BIT zp -- skips the next `txa` (2-byte skip)
no_fixup_lohi:
        TXA
        INY
        CPY #encoded_entries
        BNE table_gen

; prepare for main decruncher (DONT_REUSE_OFFSET=0). X = 0 here. C=1 (CPY)
; seeds zp_ro_state's top bit; SEC re-arms C=1 for the implicit first literal.
; zp_len_hi is zeroed so that first literal's copy tail exits after 1 byte
; (every later pass leaves zp_len_hi=0 behind). zp_bits_hi is zeroed by the
; copy tail's STX before the first get_bits needs it.
        ROR zp_ro_state
        SEC
        PLA                     ; parked <(out_addr+out_len)
        TAY
        STX zp_dest_lo
        STX zp_len_hi

; copy one literal byte to destination (DECRUNCH_FORWARDS=0): run the main
; copy loop as a length-1 literal-sequence copy (C=1 here on every entry:
; SEC above / the tag loop below exits with C=1).
literal_start1:
        LDX #1
        BNE copy_next           ; always

; fetch sequence length index. x must be #0 entering; on exit x = index+1,
; or 0 for a literal byte (tag bit 0). C=1 on loop exit.
next_round:
        ROR zp_ro_state
        DEX
nr_bit:
        JSR get_bit
        INX
        BCC nr_bit
        BEQ literal_start1      ; X=0 => literal byte

        CPX #$11
        BCS exit_or_lit_seq

; calculate length of sequence (zp_len) -- IDENTICAL to forward.
        LDA tabl_bi - 1,X
        JSR get_bits
        ADC tabl_lo - 1,X       ; zp_len_lo
        STA zp_len_lo
        LDA zp_bits_hi
        ADC tabl_hi - 1,X       ; c = 0 after this
        STA zp_len_hi
        LDX zp_len_lo
        LDA #0

; decide to reuse latest offset or not (DONT_REUSE_OFFSET=0) -- IDENTICAL.
        BIT zp_ro_state
        BMI test_reuse
no_reuse:
        STA zp_bits_hi
        LDA #$e1
        CPX #$03
        BCS gbnc2_next
        ; lzan extension: X (zp_len_lo) < 3 only means "length is literally 1
        ; or 2" when zp_len_hi is 0. lzan's exo3 encoder emits full 16-bit
        ; match lengths (upstream exomizer splits matches at 255, so its
        ; streams never take this branch); a length like $0102 must select
        ; the len>=3 offset table, not the len-2 table.
        LDX zp_len_hi
        BNE gbnc2_next
        LDX zp_len_lo
        LDA tabl_bit - 1,X
gbnc2_next:
        JSR get_bit
        ROL
        BCS gbnc2_next
        TAX

; calculate absolute offset (zp_src), BACKWARD form: src = dest + offset.
; No CLC/EOR#$ff complement; ADC tabl_lo/hi then ADC zp_dest_hi. get_bits
; returns carry clear, and a valid backward reference never wraps past $FFFF,
; so carry stays clear here (=> the copy loop's BCS falls through to a copy).
        LDA tabl_bi,X
        JSR get_bits
        ADC tabl_lo,X
        STA zp_src_lo
        LDA zp_bits_hi
        ADC tabl_hi,X
        ADC zp_dest_hi
        STA zp_src_hi

; prepare for copy loop
        LDX zp_len_lo

; main copy loop (DECRUNCH_FORWARDS=0, LITERAL_SEQUENCES_NOT_USED=0).
; C=0: match copy from (zp_src),Y. C=1: literal sequence from the stream.
copy_next:
        TYA
        BNE copy_skip_hi
        DEC zp_dest_hi
        DEC zp_src_hi
copy_skip_hi:
        DEY
        BCS get_literal_byte
        LDA (zp_src_lo),Y
literal_byte_gotten:
        STA (zp_dest_lo),Y
        DEX
        BNE copy_next
        LDA zp_len_hi
        STX zp_bits_hi
        BEQ next_round
copy_next_hi:
        DEC zp_len_hi
        JMP copy_next

; test for offset reuse (DONT_REUSE_OFFSET=0). A=0 here; after ROL, A = the
; bit and C=0 either way (no_reuse stores the 0; copy_next needs c=0).
test_reuse:
        BVS no_reuse
        JSR get_bit
        ROL
        BEQ no_reuse            ; bit == 0 => C=0, no reuse
        BNE copy_next           ; bit != 0 => C=0, reuse previous offset

; exit or literal sequence handling (LITERAL_SEQUENCES_NOT_USED=0).
; genbody gate `litseq`: present only when the stream uses literal SEQUENCES.
; When absent, the only path here is the end-of-stream marker (tag index $11 =>
; Z=1 from CPX #$11), so the handler collapses to a bare RTS. `get_literal_byte`
; below stays OUT of the gate - the copy loop branches to it directly.
;>>> gate litseq
exit_or_lit_seq:
        BEQ decr_exit
        JSR get_crunched_byte
        STA zp_len_hi
        JSR get_crunched_byte
        TAX
        BCS copy_next
decr_exit:
        RTS
;=== else
;g exit_or_lit_seq:
;g         RTS
;<<< gate litseq

get_literal_byte:
        JSR get_crunched_byte
        BCS literal_byte_gotten

; static table for bits+offset for lengths 1 and 2 (2 bytes)
; bits 2, 4 and offsets 48, 32 corresponding to %10001100, %11100010
tabl_bit:
        .byte $8c, $e2
