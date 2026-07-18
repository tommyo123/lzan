; ===========================================================================
; Exomizer (v2/v3) 6502 decruncher, forward ("mem" form), in asm6502 syntax.
; Upstream: exodecrunch.s (c) 2002-2020 Magnus Lind, zlib.
;
; Build options baked in: INLINE_GET_BITS=0, LITERAL_SEQUENCES_NOT_USED=0,
; MAX_SEQUENCE_LENGTH_256=0, EXTRA_TABLE_ENTRY_FOR_LENGTH_THREE=0,
; DONT_REUSE_OFFSET=0, DECRUNCH_FORWARDS=1, ENABLE_SPLIT_ENCODING=0.
;
; This decodes the forward `exomizer raw -d` stream, exactly what lzan's
; `exo3::compress_exo3` emits. The decruncher reads compressed bytes forward via
; a `get_crunched_byte` callback; we supply one that walks `comp_data` upward.
; The 156-byte decrunch_table is placed in fixed scratch RAM at table_ram.
; ===========================================================================
;@format: exomizer
;@direction: forward
;@variant: standard
;@entry: full_decomp
;@vfy-key: exomizer-lind-mem-forward
;@encoder: lzan::exo3::compress_exo3
;@payload: raw
;@eof: stream
;@needs: comp_data,out_addr
;@zp-len: 9
;@scratch: symbol=table_ram,len=156,align=none
;@illegal: no
;@smc: yes
;@code-bytes: 276

; ---- config-defaults ----
zp_base = $f7
table_ram = $0334
; ---- end config-defaults ----

; ---- zero page layout: one contiguous 9-byte span at zp_base ----
; (Original source scattered these across $9e/$a7/$ae/$fd; compacted here.
; zp_bitbuf/zp_dest keep their original $fd/$fe/$ff at the default zp_base:
; the dest pointer pair must stay adjacent for (zp_dest_lo),Y, the src pair
; for (zp_src_lo),Y.)
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
; get_bit: shift the next stream bit out of zp_bitbuf (refilling the buffer
; from the stream when it runs empty) into A via ROL. Callers keep the value
; being assembled in A; X/Y preserved, C = bit 7 shifted out of the old A.
; Shared body of the three inline bit-refill expansions.
get_bit:
        ASL zp_bitbuf
        BNE gb_ok
        PHA
        JSR get_crunched_byte
        ROL
        STA zp_bitbuf
        PLA
gb_ok:
        ROL
        RTS

; ---------------------------------------------------------------------------
; get_bits (INLINE_GET_BITS=0 out-of-line form); falls through into
; get_crunched_byte on the >=8-bit path (which also supplies the RTS).
get_bits:
        ADC #$80                ; needs c=0, affects v
        ASL
        BPL gb_skip
gb_next:
        JSR get_bit
        BMI gb_next
gb_skip:
        BVC gb_rts
        SEC
        STA zp_bits_hi
        ; fall through: read the low byte of the >=8-bit value

; ---------------------------------------------------------------------------
; get_crunched_byte: return next compressed byte in A, preserving X,Y,C,V.
; An absolute self-modifying load walks comp_data upward. The operand is
; seeded to comp_data at ASSEMBLY time (the SFX runs the routine exactly
; once), so full_decomp needs no runtime reseed. Only INC/LDA are used, so
; the caller's C/V survive; Z/N are not required to be preserved.
get_crunched_byte:
gcb_lda:
        LDA comp_data
        INC gcb_lda+1
        BNE gb_rts
        INC gcb_lda+2
gb_rts:
        RTS

; ---------------------------------------------------------------------------
; decrunch: init zp, build tables, decode.
full_decomp:
        LDA #>out_addr
        STA zp_dest_hi
decrunch:
        ; RAW stream (lzan compress_exo3): only the initial bit-buffer byte is
        ; in the stream, the 2-byte decrunch address that the `mem` format
        ; prepends is NOT present. zp_dest is seeded above / in the prepare
        ; step below.
        JSR get_crunched_byte
        STA zp_bitbuf
        LDY #0
        CLC

; calculate tables. y must be #0 when entering (A/X are don't-care: the first
; iteration takes the BEQ shortcut and the later TAX overwrites X).
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
        .byte $24               ; BIT zp - skips the next `txa` (2-byte skip)
no_fixup_lohi:
        TXA
        INY
        CPY #encoded_entries
        BNE table_gen

; prepare for main decruncher (DONT_REUSE_OFFSET=0). X = 0 here. The ROR
; consumes C=1 from the CPY loop exit, marking "previous op was a literal".
; zp_len_hi must be 0 so the first (implicit) literal exits the copy loop
; after one byte; zp_bits_hi is zeroed by the copy-loop tail before its
; first use in the length decode.
        ROR zp_ro_state
        LDY #<out_addr
        STX zp_dest_lo
        STX zp_len_hi

; copy one literal byte to destination (DECRUNCH_FORWARDS=1): route through
; the main copy loop with X=1 (one byte) and C=1 (fetch from the stream).
; Entered by falling through here (the stream's implicit first literal) and
; from the BEQ below (explicit literal tag).
lit_one:
        SEC
        INX
        BNE copy_next           ; always (X=1)

; fetch sequence length index. Entered ONLY from the copy-loop tail, so
; A = zp_len_hi = 0 and X = 0 on entry; on exit X = index+1, or 0 for a
; literal byte. A=0 is preserved around get_bit while the fetched bits are 0,
; so the loop needs no reload; A also stays 0 for the no_reuse/test_reuse
; paths below.
next_round:
        ROR zp_ro_state
        DEX
nr_loop:
        INX
        JSR get_bit
        BEQ nr_loop
        TXA
        BEQ lit_one             ; tag bit 0 => literal byte

        CPX #$11
        BCS exit_or_lit_seq

; calculate length of sequence (zp_len)
        LDA tabl_bi - 1,X
        JSR get_bits
        ADC tabl_lo - 1,X       ; zp_len_lo
        STA zp_len_lo
        LDA zp_bits_hi
        ADC tabl_hi - 1,X       ; c = 0 after this
        STA zp_len_hi
        LDX zp_len_lo
        LDA #0

; decide to reuse latest offset or not (DONT_REUSE_OFFSET=0)
        BIT zp_ro_state
        BMI test_reuse
no_reuse:
        STA zp_bits_hi
        ; lzan extension: X (zp_len_lo) < 3 only means "length is literally 1
        ; or 2" when zp_len_hi is 0. lzan's exo3 encoder emits full 16-bit
        ; match lengths (standard exomizer splits matches at 255, so its
        ; streams never take this branch); a length like $0102 must select
        ; the len>=3 offset table, not the len-2 table.
        LDA zp_len_hi
        BNE nr_big
        CPX #$03
        BCS nr_big
        LDA tabl_bit - 1,X
        .byte $2c               ; BIT abs - skips the next `lda #$e1`
nr_big:
        LDA #$e1
gbnc2_next:
        JSR get_bit
        BCS gbnc2_next
        TAX

; calculate absolute offset (zp_src), forward form
        LDA tabl_bi,X
        JSR get_bits
        CLC
        ADC tabl_lo,X
        EOR #$ff
        STA zp_src_lo
        LDA zp_bits_hi
        ADC tabl_hi,X
        EOR #$ff
        ADC zp_dest_hi
        STA zp_src_hi
        CLC

; prepare for copy loop
        LDX zp_len_lo

; main copy loop (DECRUNCH_FORWARDS=1, LITERAL_SEQUENCES_NOT_USED=0).
; C selects the source and is stable across the loop: 0 = match copy from
; (zp_src),Y, 1 = literal bytes fetched from the stream (the BIT-abs opcode
; byte skips the match load on that path).
copy_next:
        BCC copy_src
        JSR get_crunched_byte
        .byte $2c               ; BIT abs - skips the next `lda (zp_src_lo),y`
copy_src:
        LDA (zp_src_lo),Y
        STA (zp_dest_lo),Y
        INY
        BNE copy_skip_hi
        INC zp_dest_hi
        ; DONT_REUSE_OFFSET=0: keep zp_src in sync with zp_dest across the
        ; page wrap, or an offset-REUSE match right after a literal copies
        ; from one page too low.
        INC zp_src_hi
copy_skip_hi:
        DEX
        BNE copy_next
        LDA zp_len_hi
        STX zp_bits_hi
        BEQ next_round
        DEC zp_len_hi
        JMP copy_next

; test for offset reuse (DONT_REUSE_OFFSET=0). A = 0 on entry; get_bit
; returns the bit in A with C=0 either way.
test_reuse:
        BVS no_reuse
        JSR get_bit
        BEQ no_reuse            ; bit == 0 => no reuse
        BNE copy_next           ; bit != 0 => C=0, reuse previous offset

; exit or literal sequence handling (LITERAL_SEQUENCES_NOT_USED=0).
; genbody gate `litseq`: present only when the stream actually uses literal
; SEQUENCES. When absent, the only way to reach here is the end-of-stream
; marker (tag index $11 => Z=1 from the CPX #$11 above), so the whole handler
; collapses to a bare RTS. The tailored decoder still decodes the identical
; `exomizer raw` stream - the removed code is unreachable for that stream.
;>>> gate litseq
exit_or_lit_seq:
        BEQ decr_exit
        JSR get_crunched_byte
        STA zp_len_hi
        JSR get_crunched_byte
        TAX
        BCS copy_next           ; always (c=1 from the CPX above)
decr_exit:
        RTS
;=== else
;g exit_or_lit_seq:
;g         RTS
;<<< gate litseq

; static table for bits+offset for lengths 1 and 2 (2 bytes)
; bits 2, 4 and offsets 48, 32 corresponding to %10001100, %11100010
tabl_bit:
        .byte $8c, $e2
