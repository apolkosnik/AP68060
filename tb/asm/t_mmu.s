; AP68060 MMU self test (converted from the AP040 milestone E MMU test)
; assembled with vasmm68k_mot -Fbin -m68040; 68060-only opcodes as dc.w
;
; testbench protocol: $F100 fail number, $F102 result magic
;
; The 68040 mechanisms this test used to lean on do not exist on the
; MC68060 (MC68060UM 11.1.2.2): no PTEST, no MMUSR, no format $7 frame,
; no SSW and no write-back slots.  Every access error is a format $4
; frame (MC68060UM 8.4.4): SR, the PC of the faulted instruction, $4008,
; the fault address and the fault status long word (FSLW).  The handler
; checks the FSLW of every fault against what the test expects, repairs
; the descriptor and returns: the instruction restarts from its beginning
; (8.4.4.1).  A failed table search loads no ATC entry (4.3, 8.4.5 step
; four), so the handler repairs WITHOUT a PFLUSH, and it fails the test if
; the restarted access faults again on an already repaired descriptor --
; that can only come from an ATC entry the failed search should not have
; left behind.  PTEST checks became PLPAR/PLPAW checks (MC68060UM 4.7.3)
; and ATC persistence probes (a descriptor changed without a PFLUSH).
;
; memory map (all physical = logical except where noted):
;   $0000 vectors, $0400 code, $3400 ISP top, $3C00 USP top
;   $4000 root table, $4200 pointer table, $4400 page table (64 x 4K pages)
;   page  8 ($8000) write protected
;   page  9 ($9000) supervisor only
;   page 10 ($A000) invalid, fixed by the access error handler
;   page 11 ($B000) remapped to physical $C000
;   page 13 ($D000) invalid, fixed on an instruction fetch fault

FAILREG		equ	$F100
DONEREG		equ	$F102
HFAILID		equ	$3670	; the bench prints it with fail code 98

cnt_aerr	equ	$3600
expect_fa	equ	$3604
fix_addr	equ	$3608	; descriptor the handler repairs (-1: DTT0, -2: TC)
fix_val		equ	$360C
seen_desc	equ	$3610
cnt_stub	equ	$3614
uret		equ	$3618
expect_fslw	equ	$361C	; the FSLW the next fault must carry (under fslw_mask)
fslw_mask	equ	$3620
expect_pc	equ	$3624	; stacked PC the next fault must carry (0: any)
last_fslw	equ	$3628	; the frame of the last access error, as stacked
last_fa		equ	$362C
last_pc		equ	$3630
stale_ok	equ	$3634	; word: a re-fault on a repaired descriptor is expected
cnt_stale	equ	$3636	; word: such re-faults taken (the handler then PFLUSHes)
last_vec	equ	$3638	; word: format/vector word seen by h_skip
skip_len	equ	$363A	; word: bytes h_skip steps over
cnt_int2	equ	$3684	; level-2 interrupts taken (interleave sweeps)
cnt_int3	equ	$3688	; level-3 interrupts taken (spl-storm sweep)
storm_scr	equ	$368C	; scratch the storm decrements, like serintr's count
chkbuf		equ	$3690	; CHK bound operand (stale-record test)
aerr_act	equ	$3694	; nonzero while h_aerr is executing
cnt_trace	equ	$3698	; vector-9 traces taken in the T0 window
last_tpc	equ	$369C	; stacked PC of the first such trace
storm_dly	equ	$368E	; sweep delay handed to h_int3 for its own arming
IPLREG	equ	$F110
IPLDLY	equ	$F148
IPLCAP	equ	$F160	; bench IPL-injection capability (bit 0)
WBERRCTL	equ	$F146

; FSLW fields (MC68060UM figure 8-6; the SIZE coding is WinUAE's, see
; mmu_common.h: the manual's SIZE table is wrong)
F_MA		equ	$08000000	; 2nd or later access of a misaligned transfer
F_LK		equ	$02000000	; locked transfer
F_R		equ	$01000000	; RW = 10 read
F_W		equ	$00800000	; RW = 01 write (both: read-modify-write)
F_SZB		equ	$00200000	; SIZE byte
F_SZW		equ	$00400000	; SIZE word
F_SZL		equ	$00000000	; SIZE long
F_SZD		equ	$00600000	; SIZE double / MOVE16 line
F_SZM		equ	$00600000	; the SIZE field
F_TT16		equ	$00080000	; TT = 01 MOVE16
F_TTALT		equ	$00100000	; TT = 10 alternate (MOVES of FC 0/3/4/7)
F_TM0		equ	$00000000	; TM = the function code
F_TM1		equ	$00010000
F_TM2		equ	$00020000
F_TM5		equ	$00050000
F_TM6		equ	$00060000
F_IO		equ	$00008000	; instruction fetch
F_PTA		equ	$00001000	; invalid root descriptor
F_PTB		equ	$00000800	; invalid pointer descriptor
F_PF		equ	$00000200	; invalid page descriptor
F_SP		equ	$00000100	; supervisor protected
F_WP		equ	$00000080	; write protected
F_TWE		equ	$00000040	; bus error during the table search
F_TTR		equ	$00000008	; the access was mapped by a TTR

; 68060-only opcodes (vasm assembles for the 68040)
PLPAR_A0	equ	$F5C8		; plpar (a0)
PLPAW_A0	equ	$F588		; plpaw (a0)

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; arm the access error handler for the next fault: fault address, FSLW
; (all 32 bits compared), descriptor to repair and its repaired value.
; Clears expect_pc (set it after the macro when the PC is checked) and
; resets fslw_mask to all ones.
expf	macro
	bsr.w	setexp
	dc.l	\1,\2,\3,\4
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_aerr		; 2 access error
	rept	23
	dc.l	unexp		; 3-25
	endr
	dc.l	h_int2		; 26 level-2 autovector (interleave sweeps)
	dc.l	h_int3		; 27 level-3 autovector (spl-storm sweep)
	rept	5
	dc.l	unexp		; 28-32
	endr
	dc.l	h_utrap		; 33 TRAP #1
	rept	222
	dc.l	unexp		; 34-255
	endr

	org	$400
start:
	clr.w	(cnt_aerr).l
	clr.w	(cnt_stub).l
	clr.w	(stale_ok).l
	clr.w	(cnt_stale).l
	clr.w	(cnt_int2).l
	clr.w	(cnt_int3).l

;----------------------------------------------------------------- tables
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; i << 12
	addq.l	#3,d2		; resident
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop

	move.l	#$00008007,($4420).l	; page 8: write protected
	move.l	#$00009083,($4424).l	; page 9: supervisor only
	move.l	#0,($4428).l		; page 10: invalid
	move.l	#$0000C003,($442C).l	; page 11 -> physical $C000
	move.l	#0,($4434).l		; page 13: invalid

	move.l	#$00004203,($4000).l	; root entry 0 -> pointer table
	move.l	#$00004403,($4200).l	; pointer entry 0 -> page table

;----------------------------------------------- pre-place physical data
	move.l	#$11112222,($3000).l
	move.l	#$0A0A0A0A,($A000).l
	move.l	#$09090909,($9000).l
	move.l	#$AAAA1111,($5000).l
	move.l	#$BBBB2222,($E000).l
	move.w	#$5279,($D000).l	; addq.w #1,(cnt_stub).l
	move.l	#cnt_stub,($D002).l
	move.w	#$4E75,($D006).l	; rts

;----------------------------------------------------------------- enable
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc

;----------------------------------------------------------- translation
	move.l	($3000).l,d0
	chkl	d0,$11112222,1

	move.l	#$DEAD1234,($B000).l	; lands at physical $C000
	move.l	($C000).l,d0
	chkl	d0,$DEAD1234,2
	move.l	($B000).l,d0
	chkl	d0,$DEAD1234,3

;----------------------------------------------------------- history bits
	move.l	($4000).l,d0
	and.l	#8,d0
	chkl	d0,8,4			; root descriptor U set

	move.l	#$CAFE0505,($5000).l
	move.l	($4414).l,d0
	and.l	#$10,d0
	chkl	d0,$10,5		; page 5 modified

	tst.l	($6000).l
	move.l	($4418).l,d0
	and.l	#$10,d0
	chkl	d0,0,6			; page 6 read only: M clear
	move.l	($4418).l,d0
	and.l	#8,d0
	chkl	d0,8,7			; but U set

;------------------------------------------------- PTEST and MMUSR are gone
; MC68060UM 11.1.2.2.4: PTEST is unimplemented (its F-line encoding takes
; the line-F exception, vector 11) and the MMUSR does not exist: a MOVEC
; of it is an illegal instruction (vector 4).  Both frames are format $0
; with the PC of the instruction itself.
	move.l	#$FFFFFFFF,(expect_fa).l	; no access error may happen here
	move.l	#h_skip,($2C).l
	move.l	#h_skip,($10).l
	moveq	#5,d0
	movec	d0,dfc
	lea	($5000).l,a0
	move.w	#2,(skip_len).l
	clr.w	(last_vec).l
	ptestr	(a0)			; $F568: line F on the 68060
	move.w	(last_vec).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$002C,261		; format $0, vector 11
	move.w	#4,(skip_len).l
	clr.w	(last_vec).l
	moveq	#-1,d0
	movec	mmusr,d0		; MOVEC $805: illegal on the 68060
	move.w	(last_vec).l,d1
	and.l	#$FFFF,d1
	chkl	d1,$0010,262		; format $0, vector 4
	chkl	d0,$FFFFFFFF,263	; and D0 was not written
	move.l	#unexp,($2C).l
	move.l	#unexp,($10).l

;----------------------------------------------------------------- PLPA
; PLPAR/PLPAW (MC68060UM 4.7.3) replace PTEST: translate An in the DFC's
; space as a read/write would, load the ATC, return the physical address.
	moveq	#5,d0
	movec	d0,dfc
	lea	($B004).l,a0		; page 11 is remapped to physical $C000
	dc.w	PLPAR_A0
	chkl	a0,$0000C004,8		; the physical address, offset kept

	; PLPA sets the history bits as the access would (4.7.3): PLPAR on an
	; untouched page sets U only, PLPAW then sets M (table 4-1)
	move.l	($441C).l,d0
	and.l	#$18,d0
	chkl	d0,0,264		; page 7 untouched so far
	lea	($7000).l,a0
	dc.w	PLPAR_A0
	chkl	a0,$00007000,265
	move.l	($441C).l,d0
	and.l	#$18,d0
	chkl	d0,8,266		; PLPAR: U set, M clear
	dc.w	PLPAW_A0		; the read entry has M clear: PLPAW searches
	move.l	($441C).l,d0		; again to set M
	and.l	#$18,d0
	chkl	d0,$18,267		; PLPAW: U and M set
	move.l	#$00007003,($441C).l	; page 7 back to pristine
	pflusha

	; PLPA loads the ATC with its successful search (4.7.3, 11.1.2.2.4).
	; Changing the descriptor alone therefore leaves the probed mapping
	; cached: the old translation must still be used.
	pflusha
	lea	($E000).l,a0
	dc.w	PLPAR_A0
	chkl	a0,$0000E000,268
	move.l	#$00005003,($4438).l	; page 14 now points at physical $5000
	move.l	($E000).l,d0
	chkl	d0,$BBBB2222,36		; PLPA-installed identity mapping is stale

	; MMU register writes do not flush either ATC (MC68060UM 4.1.2: "the
	; TCR accesses and reset do not flush the ATCs").  An unchanged TC
	; write must therefore leave the PLPA-installed mapping stale.
	move.l	#$8000,d0
	movec	d0,tc
	move.l	($E000).l,d0
	chkl	d0,$BBBB2222,37		; TC write did not flush the ATC
	pflusha
	move.l	($E000).l,d0
	chkl	d0,$CAFE0505,60		; explicit PFLUSH exposes the remap
	move.l	#$0000E003,($4438).l
	pflusha

;----------------------------------------------- write protection fault
; A write that meets W during its table search: the search ends, the ATC
; is NOT loaded, and neither U nor M is written (MC68060UM 4.2.6.3, table
; 4-1 rows WP=1).  The handler repairs the descriptor without a PFLUSH;
; the restart searches again and succeeds.
	expf	$8000,F_W|F_SZL|F_TM5|F_WP,$4420,$8003
	move.l	#aerr_wp,(expect_pc).l	; the write itself (no WB slot: 11.1.2.2)
aerr_wp:
	move.l	#$FEED0808,($8000).l	; faults, handler unprotects, restarts
	move.l	($8000).l,d0
	chkl	d0,$FEED0808,9
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,1,10
	move.l	(seen_desc).l,d0
	and.l	#$18,d0
	chkl	d0,0,45			; the denied write left U and M clear
					; (the 68040 set U here)

;------------------------------ a W entry the ATC DID load needs a PFLUSH
; The other half of the rule: a READ of a write-protected page loads an
; ATC entry with W set (4.2.6.3).  A later write faults from that entry
; without a search, so repairing the descriptor alone does NOT help:
; the restart faults again (the handler sees its own repair already in
; place) until the entry is flushed -- MC68060UM 8.4.5, fifth step.
	move.w	(cnt_aerr).l,d5		; hand the counter back below
	move.l	#$00008007,($4420).l	; page 8 write protected again
	pflusha
	move.l	#$FFFFFFFF,(expect_fa).l	; the read itself must not fault
	tst.l	($8000).l		; a read search: loads W into the ATC
	move.l	($4420).l,d0
	chkl	d0,$0000800F,270	; the read set U (and left M clear)
	expf	$8000,F_W|F_SZL|F_TM5|F_WP,$4420,$800B
	move.l	#aerr_atcw,(expect_pc).l
	move.w	#1,(stale_ok).l
	clr.w	(cnt_stale).l
aerr_atcw:
	move.l	#$5A5A0808,($8000).l	; faults from the ATC, twice
	clr.w	(stale_ok).l
	move.l	($8000).l,d0
	chkl	d0,$5A5A0808,271	; landed once the entry was flushed
	move.w	(cnt_stale).l,d0
	and.l	#$FFFF,d0
	chkl	d0,1,272		; the repair alone did not help, once
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,2,273		; two faults: stale entry, then flushed
	move.w	d5,(cnt_aerr).l		; cnt_aerr is cumulative and asserted
					; downstream: hand it back untouched

;----------------------------------------------------- invalid data page
	expf	$A000,F_R|F_SZL|F_TM5|F_PF,$4428,$A003
	move.l	#aerr_pf,(expect_pc).l
aerr_pf:
	move.l	($A000).l,d0
	chkl	d0,$0A0A0A0A,11
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,2,12

;------------------------------------------------ instruction fetch fault
; FSLW IO (opword fetch), read, TM 6 (supervisor program), PF; the FA and
; the stacked PC are the opword's address (MC68060UM 8.4.4.2)
	expf	$D000,F_IO|F_R|F_SZL|F_TM6|F_PF,$4434,$D003
	move.l	#$D000,(expect_pc).l
	jsr	($D000).l		; target page invalid: fetch fault
	move.w	(cnt_stub).l,d0
	and.l	#$FFFF,d0
	chkl	d0,1,13
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,3,14

; an EXTENSION word in the invalid page: IO and MA both set, and the FA
; and the stacked PC are still the OPWORD's address, in the page before
; (MC68060UM 8.4.4.2: "For instruction extension word faults, this field
; points to the logical address of the instruction opword")
	move.w	(cnt_aerr).l,d5
	move.w	#$303C,($CFFE).l	; move.w #$1234,d0: opword in page 12
	move.w	#$1234,($D000).l	; its extension word in page 13
	move.w	#$4E75,($D002).l	; rts
	move.l	#0,($4434).l		; page 13 invalid again
	pflusha
	expf	$CFFE,F_IO|F_MA|F_R|F_SZL|F_TM6|F_PF,$4434,$D003
	move.l	#$CFFE,(expect_pc).l
	moveq	#0,d0
	jsr	($CFFE).l
	chkl	d0,$1234,274		; restarted from the opword
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,1,275
	move.w	d5,(cnt_aerr).l

;------------------------------------------- user access to a super page
	movea.l	#$3C00,a0
	movec	a0,usp
	move.l	#ucont,(uret).l
	expf	$9000,F_R|F_SZL|F_TM1|F_SP,$4424,$9003
	move.l	#user_code,(expect_pc).l
	move.w	#$0000,-(sp)
	pea	user_code(pc)
	move.w	#$0000,-(sp)
	rte

user_code:
	move.l	($9000).l,d0	; faults: S page from user mode
	trap	#1

ucont:
	chkl	d0,$09090909,15
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,4,16
	; a user access to a supervisor-only page still sets U (with a locked
	; read-modify-write for a read: MC68060UM 4.2.5) before the fault
	move.l	(seen_desc).l,d0
	and.l	#$18,d0
	chkl	d0,8,276

;------------------------------- RTE to user: fetch uses the user root
; a separate user root maps VA $7000 -> PA $F000 (moveq/trap) while the
; supervisor root keeps identity, where $7000 holds ILLEGAL. The first
; fetch after RTE belongs to the restored user context, so it must
; translate through URP; with a supervisor-FC fetch this executes the
; ILLEGAL instead and the unexpected-exception handler fails the test.
	lea	($4C00).l,a0
	moveq	#0,d0
	moveq	#63,d1
utloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,utloop
	move.l	#$0000F003,($4C1C).l	; user VA page 7 -> PA $F000
	move.l	#$00004A03,($4800).l
	move.l	#$00004C03,($4A00).l
	move.w	#$702A,($F000).l	; moveq #42,d0
	move.w	#$4E41,($F002).l	; trap #1
	move.w	#$4AFC,($7000).l	; illegal via the supervisor map
	move.l	#$4800,d0
	movec	d0,urp
	pflusha
	move.l	#ucont2,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($7000).l
	move.w	#$0000,-(sp)
	rte

ucont2:
	chkl	d0,42,33		; ran the user-mapped code

;--------------------- MOVE to SR supervisor->user: fetch uses the user root
; The MOVE to SR sits at VA $7000, which the supervisor root identity maps.
; Clearing S means the NEXT fetch (VA $7004) belongs to the user context and
; must translate through URP to PA $F004; the supervisor view of $7004 holds
; ILLEGAL, so a stale supervisor FC on that fetch fails the test.
	move.l	#ucont3,(uret).l
	moveq	#0,d0
	move.w	#$702B,($F004).l	; user view:  moveq #43,d0
	move.w	#$4E41,($F006).l	;             trap #1
	move.w	#$4AFC,($7004).l	; super view: illegal
	move.w	#$46FC,($7000).l	; move.w #$0000,sr
	move.w	#$0000,($7002).l
	pflusha
	jmp	($7000).l

ucont3:
	chkl	d0,43,39		; ran the user-mapped code after MOVE to SR
	move.l	#$4000,d0
	movec	d0,urp			; restore the shared root
	pflusha

;---------------------------- high VA walk: nonzero root/pointer indexes
; VA $1E0C3000: root index 15, pointer index 3, page index 3. The page
; table sits at $4D00 (bit 8 set) to prove the 256-byte table base mask.
	move.l	#$00004E03,($403C).l	; root[15] -> pointer table $4E00
	move.l	#$00004D03,($4E0C).l	; pointer[3] -> page table $4D00
	move.l	#$0000E003,($4D0C).l	; page[3] -> PA $E000
	move.l	($1E0C3000).l,d0
	chkl	d0,$BBBB2222,34
	move.l	#$FEED5678,($1E0C3004).l
	move.l	($E004).l,d0
	chkl	d0,$FEED5678,35

;------------------------- transfers that cross a page boundary
; The MMU translates one address per bus transaction, so a misaligned
; transfer spanning two pages must be split: every byte has to go through
; its own page's mapping.  Page 2 ($2000) keeps its identity mapping while
; page 3 ($3000) is remapped to physical $E000, so a long written at
; $2FFE lands two bytes in $2FFE and two bytes at the top of $E000.
	; First make the second page invalid.  Its fault must report the
	; operand's FIRST byte as the FA and the original long size, not the
	; split part, and set FSLW MA because the fault was on the second
	; access of the misaligned transfer (MC68060UM 8.4.4.2, figure 8-6).
	; The restart repeats the first, successful read (8.4.4.1).
	; Use pages 5/6 rather than 2/3: the active ISP is in page 3, so making
	; page 3 invalid would correctly double-fault while stacking the frame.
	move.w	#$1122,($5FFE).l
	move.w	#$3344,($E000).l
	expf	$5FFE,F_MA|F_R|F_SZL|F_TM5|F_PF,$4418,$E003
	move.l	#aerr_ma,(expect_pc).l
	move.l	#0,($4418).l
	pflusha
aerr_ma:
	move.l	($5FFE).l,d0
	chkl	d0,$11223344,46
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,5,47
	move.l	#$00006003,($4418).l
	pflusha

	move.l	#$0000E003,($440C).l	; page 3 -> physical $E000
	pflusha
	move.l	#$00000000,($2FFC).l
	move.l	#$00000000,($E000).l
	move.l	#$11223344,($2FFE).l	; straddles the boundary
	move.l	($2FFC).l,d0
	chkl	d0,$00001122,40		; low half of page 2 got the top bytes
	move.l	($E000).l,d0
	chkl	d0,$33440000,41		; page 3's physical target got the rest
	move.l	($2FFE).l,d0
	chkl	d0,$11223344,42		; and it reads back through both pages

	move.w	#$5566,($2FFF).l	; word crossing at an odd address
	move.b	($2FFF).l,d0
	and.l	#$FF,d0
	chkl	d0,$55,43
	move.b	($E000).l,d0
	and.l	#$FF,d0
	chkl	d0,$66,44

	move.l	#$00003003,($440C).l	; restore the identity mapping
	pflusha
	move.l	#$BBBB2222,($E000).l	; the ATC tests below still need this

;------------------------------------------------------------ TTR bypass
	move.l	#$FFFFFFFF,(expect_fa).l	; no fault may be taken here
	move.l	#0,($4414).l	; page 5 invalid
	pflusha
	move.l	#$0000C000,d0	; DTT0: base $00, mask $00, both modes
	movec	d0,dtt0
	move.l	($5000).l,d0	; transparent: no fault, no walk
	chkl	d0,$CAFE0505,17
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,5,18
	; PLPA under a TTR match answers the address itself without a search
	; (WinUAE mmu_op_real PLPA: mmu_translate only on TTR_NO_MATCH): page 5
	; is invalid, so a search would fault
	lea	($5000).l,a0
	dc.w	PLPAR_A0
	chkl	a0,$00005000,19
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,5,277		; no fault

	; PLPAW of a write-protected transparent translation is an access
	; error with FSLW TTR and WP (the 68040's PTESTW reported MMUSR B):
	; the handler clears the TTR's W and the PLPAW restarts.  The W TTR
	; matches USER accesses only (S field 00) and is probed through the
	; DFC (1): the supervisor stack the exception frame goes to lies in
	; the same 16MB block and must not be write protected itself.
	move.w	(cnt_aerr).l,d5
	expf	$5000,F_W|F_TM1|F_TTR|F_WP,-1,$0000C000
	move.l	#~F_SZM,(fslw_mask).l	; PLPA is no data transfer: the manual
					; gives it no size (WinUAE passes word)
	move.l	#aerr_plpattr,(expect_pc).l
	moveq	#1,d0
	movec	d0,dfc
	move.l	#$00008004,d0		; DTT0: E, user only, W
	movec	d0,dtt0
aerr_plpattr:
	dc.w	PLPAW_A0
	chkl	a0,$00005000,38
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,1,278

	; and a real write through the write-protected TTR (a MOVES to user
	; data space): FSLW TTR|WP, the FA the operand address, size long,
	; TM 1 (MC68060UM 8.4.5 step three)
	expf	$5004,F_W|F_SZL|F_TM1|F_TTR|F_WP,-1,$0000C000
	move.l	#aerr_ttrw,(expect_pc).l
	move.l	#$CAFE0606,d1
	move.l	#$00008004,d0
	movec	d0,dtt0
aerr_ttrw:
	moves.l	d1,($5004).l
	move.l	($5004).l,d0		; supervisor read: DTT0 both modes now
	chkl	d0,$CAFE0606,279
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,2,280

	; DFC program space selects ITT rather than DTT for PLPA (WinUAE:
	; data = (dfc & 3) != 2): only ITT0 matches, page 5 is invalid, so
	; with DFC 6 there is no search and no fault...
	move.l	#$FFFFFFFF,(expect_fa).l
	moveq	#0,d0
	movec	d0,dtt0
	move.l	#$0000C000,d0
	movec	d0,itt0
	moveq	#6,d0
	movec	d0,dfc
	dc.w	PLPAR_A0
	chkl	a0,$00005000,291	; ITT match: the address itself (was 39)
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,2,281
	; ...while with DFC 5 the ITT does not apply: the search meets the
	; invalid page and the PLPAR faults (FA = An, TM = the DFC, PF)
	moveq	#5,d0
	movec	d0,dfc
	expf	$5000,F_R|F_TM5|F_PF,$4414,$5003
	move.l	#~F_SZM,(fslw_mask).l
	move.l	#aerr_plpapf,(expect_pc).l
aerr_plpapf:
	dc.w	PLPAR_A0
	chkl	a0,$00005000,282
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,3,283
	move.w	d5,(cnt_aerr).l
	moveq	#0,d0
	movec	d0,itt0
	moveq	#0,d0
	movec	d0,dtt0
	move.l	#0,($4414).l	; page 5 invalid again, as the 68040 test had it
	pflusha

;--------------------------- chipset IO through a transparent translation
; Everything that covers RTG so far runs with the MMU off, but the MMU is
; the structural difference between this CPU and the TG68K the upstream
; core uses -- and 68040.library always enables it, mapping the low 16MB
; of IO space through a transparent translation rather than page tables.
; Reproduce that idiom exactly: DTT0 base $00, mask $00 (so only
; $00000000-$00FFFFFF matches), E=1, S=both, CM=10 cache-inhibited
; serialized, and read the RTG ID through it.  The page tables here cover
; only $0-$3FFFF, so this access reaches fastchip solely via the TTR.
	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_ttr_done
	move.l	#$0000C040,d0
	movec	d0,dtt0
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,171	; ID/VERSION through the TTR

	; and the registers must still take a write under translation
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,172

	; same again with the TTR marked cache-inhibited nonserialized, the
	; other mode 68040.library uses for IO
	move.l	#$0000C060,d0
	movec	d0,dtt0
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,173
	moveq	#0,d0
	movec	d0,dtt0

rtg_ttr_done:

	move.l	#$00005003,($4414).l
	pflusha

; ...and now the case that actually matters.  A TTR bypasses the table walk
; entirely, but SetPatch installs 68040.library, which maps IO space through
; real page tables -- and the RTG ID reads $5001 before SetPatch and $0000
; after.  So walk to it: $00B8010E splits into root index 0, pointer index
; $2E (VA[24:18]), page index 0 (VA[17:12]), and the page frame is
; identity with CM = 10, cache-inhibited serialized, the mode 68040.library
; uses for a register block.
	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_walk_done
	move.l	#$00B80043,($5800).l	; page 0 of the $B8 region, CI serialized
	move.l	#$00005803,($42B8).l	; pointer entry $2E -> that page table
	pflusha
	move.l	#$80008000,d0
	movec	d0,cacr			; SetPatch enables these too
	lea	($B8010E).l,a0
	dc.w	PLPAR_A0		; DFC is 5
	chkl	a0,$00B8010E,190	; the walk itself resolves
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,174	; ID through a real table walk

	; a write and read-back through the same translation
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,175

	; the walk must be repeatable once the ATC entry is resident, and
	; still correct after it is flushed away again
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,176
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,177

	; 68040.library chooses the cache mode in the descriptor, and it does
	; not necessarily know $B80000 is a register block: an unknown region
	; can end up copyback or writethrough rather than cache-inhibited.
	; The L1 must bypass it regardless, because $B8xxxx is outside
	; cache_win -- so every CM encoding has to read the same.
	move.l	#$00B80003,($5800).l	; CM = 00, writethrough
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,191
	move.l	#$00B80023,($5800).l	; CM = 01, copyback
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,192
	move.l	#$00B80063,($5800).l	; CM = 11, cache-inhibited nonserialized
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,193

	; and a write-back-then-read under copyback, which is the mode that
	; would post a dirty line if anything ever cached this page
	move.l	#$00B80023,($5800).l
	pflusha
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,194
	cpusha	dc
	move.l	($B80100).l,d1
	chkl	d1,$02000000,195

	; With the region UNMAPPED -- which is what an MMU setup that only
	; covers the boards it knows about leaves behind, and the MiSTer RTG
	; board is not autoconfig -- the access must take a normal access
	; fault, not quietly return data.  A faulted read is what a monitor
	; displays as $0000, which is exactly the post-SetPatch symptom.
	move.w	(cnt_aerr).l,d5		; hand the counter back below
	and.l	#$FFFF,d5
	move.l	#0,($42B8).l
	pflusha
	; the pointer descriptor is the invalid one: FSLW PTB, word read
	expf	$00B8010E,F_R|F_SZW|F_TM5|F_PTB,$42B8,$00005803
	move.w	($B8010E).l,d0		; faults, handler maps it, restarts
	and.l	#$FFFF,d0
	chkl	d0,$00005001,196	; and the restarted access reads the ID
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	sub.l	d5,d0
	chkl	d0,1,197		; exactly one access fault was taken
	move.w	d5,(cnt_aerr).l		; cnt_aerr is cumulative and asserted
					; downstream: hand it back untouched

	moveq	#0,d0
	movec	d0,cacr			; back to the uncached regime
	cinva	bc
	move.l	#0,($42B8).l		; unmap the region again
	pflusha
rtg_walk_done:
	; 8K is checked in the 8K-pages section below, where the tables that
	; map this code have been rebuilt for it -- switching TC alone would
	; reinterpret them and fault on the next instruction fetch

;--------------------------------------- ATC caching and page PFLUSH
	move.l	($5000).l,d0	; walk and cache the mapping
	chkl	d0,$CAFE0505,20
	move.l	#$0000E003,($4414).l	; remap page 5 without a flush
	move.l	($5000).l,d0
	chkl	d0,$CAFE0505,21		; stale ATC entry still used
	lea	($5000).l,a0
	pflush	(a0)
	move.l	($5000).l,d0
	chkl	d0,$BBBB2222,22		; new mapping after the flush
	move.l	#$00005003,($4414).l
	pflusha

;----------------------------------------- MOVEM restart across a fault
	; registers to memory into a page that faults mid-transfer: the
	; 68060 has no MOVEM continuation (no SSW CM, MC68060UM 8.4.4.1): the
	; frame's PC is the MOVEM and RTE re-executes all of it, repeating the
	; write to $6FFC that had already completed.  Each register is its own
	; access, so the FA is the faulting long ($7000), not the MOVEM's EA.
	expf	$7000,F_W|F_SZL|F_TM5|F_PF,$441C,$7003
	move.l	#aerr_mvw,(expect_pc).l
	move.l	#0,($441C).l		; make page 7 invalid
	pflusha
	move.l	#$AAAA0001,d1
	move.l	#$BBBB0002,d2
	move.l	#$CCCC0003,d3
	lea	($6FFC).l,a0		; last longword of valid page 6
aerr_mvw:
	movem.l	d1-d3,(a0)		; d1 at $6FFC, d2/d3 fault into page 7
	move.l	($6FFC).l,d0
	chkl	d0,$AAAA0001,24
	move.l	($7000).l,d0
	chkl	d0,$BBBB0002,25
	move.l	($7004).l,d0
	chkl	d0,$CCCC0003,26
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,6,27

	; memory to registers with a fault on the second page
	expf	$7000,F_R|F_SZL|F_TM5|F_PF,$441C,$7003
	move.l	#aerr_mvr,(expect_pc).l
	move.l	#0,($441C).l
	pflusha
	moveq	#0,d1
	moveq	#0,d2
	moveq	#0,d3
aerr_mvr:
	movem.l	(a0),d1-d3
	chkl	d1,$AAAA0001,28
	chkl	d2,$BBBB0002,29
	chkl	d3,$CCCC0003,30
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,7,31

	; the EA base register inside a LOADED list: a fault after the base
	; was loaded must still restart with the ORIGINAL base.  The loaded
	; value may only commit with the last transfer -- a core that writes
	; it mid-loop recomputes the restart EA from the loaded DATA (here a
	; non-address) and reads garbage.
	expf	$7000,F_R|F_SZL|F_TM5|F_PF,$441C,$7003
	move.l	#aerr_mvb,(expect_pc).l
	move.l	#$11112222,($6FF8).l	; a0's image: not an address
	move.l	#$33334444,($6FFC).l	; a1's image, last valid long
	move.l	#$55556666,($7000).l	; a2's image, first faulting long
	move.l	#0,($441C).l		; page 7 invalid again
	pflusha
	lea	($6FF8).l,a0
aerr_mvb:
	movem.l	(a0),a0-a2		; a0 loads first; a2's read faults
	chkl	a0,$11112222,144
	chkl	a1,$33334444,145
	chkl	a2,$55556666,146
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,8,147
	subq.w	#1,(cnt_aerr).l	; later sections count faults absolutely

	; the EA INDEX register inside a loaded list.  The 68040 saved the
	; calculated EA in its frame (SSW CM, MC68040UM 8.4.6.5) and restarted
	; MOVEM from it; the 68060 has no such field and RTE re-executes the
	; MOVEM from its first word (MC68060UM 8.4.4.1), recomputing the EA.
	; That is only correct if no register the EA depends on has been
	; committed -- the base register is held back for exactly this reason,
	; and the index register needs the same treatment.
	expf	$7000,F_R|F_SZL|F_TM5|F_PF,$441C,$7003
	move.l	#aerr_mvx,(expect_pc).l
	move.l	#$D0D00001,($6FF8).l	; d1's image: the index, loaded first
	move.l	#$D0D00002,($6FFC).l	; d2's image, last valid long
	move.l	#$D0D00003,($7000).l	; d3's image, first faulting long
	move.l	#0,($441C).l		; page 7 invalid again
	pflusha
	lea	($6FF8).l,a0
	move.w	(cnt_aerr).l,d4		; the running total is section-relative
	moveq	#0,d1			; the index starts at zero
	moveq	#0,d2
	moveq	#0,d3
aerr_mvx:
	movem.l	(0,a0,d1.l),d1-d3
	chkl	d1,$D0D00001,248
	chkl	d2,$D0D00002,249
	chkl	d3,$D0D00003,250
	move.w	(cnt_aerr).l,d0
	sub.w	d4,d0
	and.l	#$FFFF,d0
	chkl	d0,1,251		; exactly one fault, and one restart
	subq.w	#1,(cnt_aerr).l		; leave the total as the later sections expect

;----------------------------------------------------------------- 8K pages
	moveq	#0,d0
	movec	d0,tc		; MMU off while rebuilding tables
	pflusha

	; page table: 32 entries of 8K covering LA 0-$3FFFF, identity
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#31,d1
t8loop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2		; i << 13
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,t8loop

	; entry 5 (LA $A000-$BFFF) remapped to PA $C000-$DFFF
	move.l	#$0000C003,($4414).l
	; pre-place physical data through the identity map (MMU off)
	move.l	#$08081111,($C120).l	; seen through LA $A120 (LA12=0)
	move.l	#$08082222,($D120).l	; seen through LA $B120 (LA12=1)

	; the $B8 region again, now under 8K paging: root index 0, pointer
	; index $2E, and the page index is VA[17:13] which is still 0, so the
	; same descriptor serves -- the frame simply carries no bit 12
	move.w	(IPLCAP).l,d0
	btst	#4,d0
	beq	rtg_8k_skip
	move.l	#$00B80043,($5800).l
	move.l	#$00005803,($42B8).l
rtg_8k_skip:

	move.l	#$C000,d0	; E=1, P=1: 8K pages
	movec	d0,tc

	move.l	($3000).l,d0	; identity page, LA12=1 within its 8K page
	chkl	d0,$11112222,240
	move.l	($A120).l,d0
	chkl	d0,$08081111,241
	move.l	($B120).l,d0	; same 8K page, LA bit 12 set
	chkl	d0,$08082222,242

	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_8k_done
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,179	; RTG ID through an 8K page walk
rtg_8k_done:
	move.l	#0,($42B8).l
	pflusha

	lea	($A120).l,a0	; PLPA under 8K paging
	dc.w	PLPAR_A0
	chkl	a0,$0000C120,243
	lea	($B120).l,a0	; SAME 8K page as $A120 above
	dc.w	PLPAR_A0
	; PLPA returns the probed LA's PHYSICAL ADDRESS, not the page frame
	; the 68040's MMUSR reported: under 8K pages, PA bit 12 is LA bit 12
	; (MC68060UM 4.3; WinUAE mmu_translate: l->phys | (addr & mmu_pagemask)),
	; so the second probe of the same 8K page answers $D120
	chkl	a0,$0000D120,244

	; fault and restart under 8K paging
	expf	$E000,F_W|F_SZL|F_TM5|F_PF,$441C,$E003	; entry 7: LA $E000-$FFFF
	move.l	#aerr_8kw,(expect_pc).l
	move.l	#0,($441C).l
	pflusha
	move.l	#$0E0E0E0E,d1
aerr_8kw:
	move.l	d1,($E000).l	; faults, handler fixes, restart writes
	move.l	($E000).l,d0
	chkl	d0,$0E0E0E0E,245
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,8,246

;----------------------------------------------------------------- disable
	moveq	#0,d0
	movec	d0,tc
	move.l	($3000).l,d0
	chkl	d0,$11112222,23

;================ WinUAE-oracle MMU/frame audit battery (2026-08-07) ======
; Rebuild the 4K identity tables (the 8K section rewrote the page table).
	lea	($4400).l,a0
	moveq	#0,d1
	move.w	#63,d0
t48loop:
	move.l	d1,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; page number << 12
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d1
	dbra	d0,t48loop
	pflusha

;--------------------------------------- PLPA with translation disabled
; The 68040's PTEST walked the tables even with TC.E clear; PLPA does not:
; with E = 0 the default transparent translation answers and An is left
; unchanged (MC68060UM 4.1.2, 4.7.3).  Page 8's descriptor is pointed
; elsewhere so a search would be visible.
	move.l	#$FFFFFFFF,(expect_fa).l	; no fault may be taken here
	move.l	#$0000C003,($4420).l	; page 8 -> PA $C000, if anything searched
	lea	($8000).l,a0
	moveq	#5,d0
	movec	d0,dfc
	dc.w	PLPAR_A0
	chkl	a0,$00008000,48		; no search: the address itself
	dc.w	PLPAW_A0
	chkl	a0,$00008000,284

	; TCR DWO write protects the default translation: PLPAW is then an
	; access error with FSLW WP and no TTR bit (MC68060UM 8.4.5, sixth
	; step).  DTT0 maps the low 16MB (stack, variables, bench ports) so
	; the exception's own frame writes are not caught by DWO; the PLPAW
	; probes $01008000, outside it.  The handler clears DWO.
	move.w	(cnt_aerr).l,d5
	move.l	#$0000C000,d0
	movec	d0,dtt0
	lea	($01008000).l,a0
	expf	$01008000,F_W|F_TM5|F_WP,-2,0
	move.l	#~F_SZM,(fslw_mask).l
	move.l	#aerr_dwo,(expect_pc).l
	move.l	#$00000020,d0		; E = 0, DWO = 1
	movec	d0,tc
aerr_dwo:
	dc.w	PLPAW_A0
	chkl	a0,$01008000,285
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,1,286
	move.w	d5,(cnt_aerr).l
	moveq	#0,d0
	movec	d0,dtt0
	move.l	#$00008003,($4420).l	; page 8 identity again

; enable 4K translation for the fault-shape tests
	move.l	#$8000,d0
	movec	d0,tc
	pflusha

; a bus error on a descriptor fetch during PLPAR (the 68040's PTEST
; reported MMUSR B): an access error with FSLW TWE and LK (table searches
; are locked), read, TM = the DFC (MC68060UM 8.4.5 fourth step; WinUAE
; mmu_fill_atc).  The failed search loads nothing, so the handler's
; rewrite of the descriptor plus RTE is enough: the PLPAR restarts,
; searches again and answers.
	move.w	(cnt_aerr).l,d5
	move.l	#$0000800B,($4420).l	; page 8 identity, U already set
	pflusha
	lea	($8000).l,a0
	expf	$8000,F_LK|F_R|F_TM5|F_TWE,$4420,$8003
	move.l	#~F_SZM,(fslw_mask).l
	move.l	#aerr_twe,(expect_pc).l
	move.w	#1,(WBERRCTL).l
aerr_twe:
	dc.w	PLPAR_A0
	chkl	a0,$00008000,49
	move.w	(cnt_aerr).l,d0
	sub.w	d5,d0
	and.l	#$FFFF,d0
	chkl	d0,1,287
	move.w	d5,(cnt_aerr).l

; The FSLW shape tests below accept any FSLW in the handler (mask 0) and
; check the captured one explicitly, so each field has its own number.

; MOVE16 write fault: FSLW SIZE = 11 (line) with TT = 01 (MOVE16), write,
; TM 5, WP; FA the line address (WinUAE mmu_bus_error: addr &= ~15).  The
; 68040 frame had a WB2 line writeback here; the 68060 restarts the MOVE16,
; so both postincrements happen exactly once.
	move.l	#$16161616,($3100).l
	move.l	#$27272727,($3104).l
	move.l	#$38383838,($3108).l
	move.l	#$49494949,($310C).l
	move.l	#$00008007,($4420).l	; write protect page 8
	pflusha
	expf	$8000,0,$4420,$8003
	clr.l	(fslw_mask).l
	move.l	#aerr_m16,(expect_pc).l
	lea	($3100).l,a0
	lea	($8000).l,a1
aerr_m16:
	move16	(a0)+,(a1)+
	move.l	(last_fslw).l,d0
	chkl	d0,F_W|F_SZD|F_TT16|F_TM5|F_WP,50
	chkl	a0,$00003110,51		; restart: each postincrement once
	chkl	a1,$00008010,288
	move.l	($8000).l,d0
	chkl	d0,$16161616,52		; restart completed the line
	move.l	($800C).l,d0
	chkl	d0,$49494949,289

; TAS operand fault: a locked read-modify-write.  FSLW LK and RW = 11
; (MC68060UM figure 8-6: RW 11 = read-modify-write, LK = TAS), byte, WP
	move.b	#$11,($8005).l
	move.l	#$00008007,($4420).l
	pflusha
	expf	$8005,0,$4420,$8003
	clr.l	(fslw_mask).l
	move.l	#aerr_tas,(expect_pc).l
aerr_tas:
	tas	($8005).l
	move.l	(last_fslw).l,d0
	chkl	d0,F_LK|F_R|F_W|F_SZB|F_TM5|F_WP,53
	moveq	#0,d0
	move.b	($8005).l,d0
	chkl	d0,$91,54		; restart completed the set

; MOVES to FC 0: MC68060UM table 4-2 -- TT = 10 (alternate), TM = 000;
; the access error frame carries those (4.2.5: "the resulting access
; error stack frame contains the converted function code in the TM field")
	move.l	#$00008007,($4420).l
	pflusha
	expf	$8006,0,$4420,$8003
	clr.l	(fslw_mask).l
	move.l	#aerr_mvs0,(expect_pc).l
	moveq	#0,d0
	movec	d0,dfc
	move.b	#$5C,d1
aerr_mvs0:
	moves.b	d1,($8006).l
	move.l	(last_fslw).l,d0
	chkl	d0,F_W|F_SZB|F_TTALT|F_TM0|F_WP,55
	moveq	#0,d0
	move.b	($8006).l,d0
	chkl	d0,$5C,56

; MOVES to FC 2 is converted to user data space: TT = 00, TM = 001
; (MC68060UM table 4-2)
	move.l	#$00008007,($4420).l
	pflusha
	expf	$8007,0,$4420,$8003
	clr.l	(fslw_mask).l
	move.l	#aerr_mvs2,(expect_pc).l
	moveq	#2,d0
	movec	d0,dfc
	move.b	#$7B,d1
aerr_mvs2:
	moves.b	d1,($8007).l
	move.l	(last_fslw).l,d0
	chkl	d0,F_W|F_SZB|F_TM1|F_WP,57
	moveq	#5,d0
	movec	d0,dfc

; a plain write fault: the 68040 carried the write in WB3 and stacked the
; NEXT instruction's PC.  The 68060 has no write-back slots: the stacked PC
; is the faulting write itself (MC68060UM 11.1.2.2.3) and RTE performs the
; store by re-executing it.
	move.l	#$00008007,($4420).l
	pflusha
	expf	$8000,F_W|F_SZL|F_TM5|F_WP,$4420,$8003
	move.l	#aerr_pw,(expect_pc).l
aerr_pw:
	move.l	#$DEADBEE5,($8000).l
	move.l	(last_pc).l,d0
	chkl	d0,aerr_pw,58
	move.l	($8000).l,d0
	chkl	d0,$DEADBEE5,59		; the restart performed the store

;----------- walker U/M writes bypass the data cache (MC68060UM 4.2.5)
; The 68040 test required the walker's U write to invalidate a cached copy
; of the descriptor.  The 68060 does the opposite: "Table search accesses
; bypass the data cache. No allocation is done and no cache search is
; performed" (4.2.5), and the tables must live in noncachable memory
; because the processor only uses tables in external memory (8.4.5).  So
; a descriptor the CPU has already read AS DATA into the cache stays as
; it was -- the walker neither uses, updates nor invalidates the line --
; while memory gets the U bit, which a read after CINV then sees.
; The descriptor is rewritten and the ATC flushed first, so the test does
; not depend on history bits left by anything above.
	move.l	#$80008000,d0
	movec	d0,cacr			; caches on for this test only
	move.l	#$00007003,($441C).l	; page 7 identity, U and M clear
	pflusha
	move.l	($441C).l,d0		; caches the line holding the descriptor
	and.l	#$18,d0
	chkl	d0,0,148		; U and M start clear
	tst.l	($7000).l		; touch page 7: the walker sets U in memory
	move.l	($441C).l,d0		; the cached line was not touched
	and.l	#$18,d0
	chkl	d0,0,149
	cinva	dc			; drop the (clean, writethrough) line
	move.l	($441C).l,d0		; memory: the walker's U write landed
	and.l	#$18,d0
	chkl	d0,8,290
	cinva	bc
	moveq	#0,d0
	movec	d0,cacr			; back to the uncached regime

	moveq	#0,d0
	movec	d0,tc

;--------------------- relocated: 8K user-mode demand paging (see below)
; Runs LAST: it rebuilds its own tables and nothing downstream depends on
; the state it leaves.  Re-enable 8K translation with the shared root.
	move.l	#$4000,d0	; shared root: $4000 -> $4200 -> $4400
	movec	d0,urp
	movec	d0,srp
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	lea	($4400).l,a0	; rebuild the 32-entry 8K identity table
	moveq	#0,d0
	moveq	#31,d1
t8loop2:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,t8loop2
	move.l	#$C000,d0
	movec	d0,tc
	pflusha
	move.w	(cnt_aerr).l,d7	; running last: count faults relative

;--------------------- 8K user-mode demand paging: the NetBSD exec shape
; NetBSD/amiga runs the 040 with 8K pages and execs init by mapping
; nothing, letting the FIRST USER INSTRUCTION FETCH fault, and paging the
; code in from disk at whatever fault address the frame reports.  If the
; stacked FA is wrong under 8K user ifetch, UVM pages in the wrong page
; and the right one never arrives -- the exact live signature captured in
; tests/ap040/hw/netbsd (uvmexp.paging stuck at 1).  This test performs
; that sequence: user table with the code page NOT RESIDENT, RTE to user,
; ifetch faults (TM=2), the handler validates FA and maps the page, the
; restart runs the user code, and a trap returns.  Still under TC=$C000.
	lea	($4C00).l,a0	; user page table: 32 x 8K identity
	moveq	#0,d0
	moveq	#31,d1
u8loop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2		; i << 13
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,u8loop
	move.l	#$00004A03,($4800).l	; user root -> pointer -> table
	move.l	#$00004C03,($4A00).l
	; user code page: VA $6000 (8K page 3, covers $6000-$7FFF), NOT
	; resident yet; physical backing prepared at PA $E000
	move.l	#0,($4C0C).l
	move.w	#$702C,($E000).l	; moveq #44,d0
	move.w	#$4E41,($E002).l	; trap #1
	move.l	#$4800,d0
	movec	d0,urp
	pflusha
	; USER instruction fetch of the opword at VA $6000: FSLW IO, read,
	; TM 2, PF; FA and stacked PC the opword's address.  The handler maps
	; entry 3: VA $6000-$7FFF -> PA $E000
	expf	$6000,F_IO|F_R|F_SZL|F_TM2|F_PF,$4C0C,$E003
	move.l	#$6000,(expect_pc).l
	move.l	#u8cont,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($6000).l
	move.w	#$0000,-(sp)
	rte			; to user; the ifetch at $6000 faults

u8cont:
	chkl	d0,44,150	; the paged-in user code ran after restart
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,1,151	; exactly one ifetch fault

	; same shape for a user DATA fault mid-page: fault on a non-resident
	; 8K page at an offset with LA bit 12 SET, so a bit-12 confusion in
	; the fault path (FA or the repaired translation) would surface here.
	; The user code lives on a FRESH page (VA $B000, upper half of 8K
	; entry 5, identity) written
	; before it ever executes -- and executing from the UPPER 4K half
	; also proves the ifetch side of the 13-bit offset.  The faulting
	; read targets VA $9120 (bit 12 SET: offset $1120) on
	; non-resident entry 4, mapped by the handler to PA $C000, where the
	; datum sits at PA $C000 + $1120 = $D120 (13-bit page offset).
	move.w	#$2039,($B000).l	; move.l ($9120).l,d0
	move.l	#$00009120,($B002).l
	move.w	#$4E41,($B006).l	; trap #1
	cpusha	bc			; the code page must reach memory and
	cinva	ic			; no stale line may shadow it
	move.l	#0,($4C10).l		; entry 4 (VA $8000-$9FFF) not resident
	move.l	#$0BBB1234,($D120).l	; backing for VA $8120: offset $1120
	move.l	#$0000A003,($4C14).l	; entry 5 identity: the code page
					; VA $A000 -> PA $A000; code placed
					; in its SECOND half at PA $B000 so
					; the RTE target VA $B000 needs the
					; 13-bit offset to reach it
	pflusha
	expf	$9120,F_R|F_SZL|F_TM1|F_PF,$4C10,$C003	; USER data read
	move.l	#$B000,(expect_pc).l
	move.l	#u8dat,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($B000).l		; offset $1000 into the 8K page
	move.w	#$0000,-(sp)
	rte

u8dat:
	chkl	d0,$0BBB1234,152	; read through the paged-in mapping
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,2,252	; exactly one more, the data fault

	move.l	#$4400,d0	; restore the shared root
	movec	d0,urp

;--------------- write-fault discipline: the NetBSD relocation shape (154/155)
; ld.elf_so relocates libc with add.l %d1,%a0@ on copy-on-write data pages.
; The first store to such a page write-faults; the OS repairs the page and
; returns.  On the 68040 NetBSD also performed the writebacks the frame
; marked valid, and a restart model advertising one applied the add twice
; (captured live on hardware: init's ctor pointer held link VA + 2x load
; base).  The 68060 frame has no writeback slots at all: the stacked PC is
; the ADD (MC68060UM 11.1.2.2.3) and RTE re-executes it, re-reading the
; unmodified operand.  A read-modify-write meets the write protection in
; its table search (4.2.6.3), so the FSLW is RW = 11 with WP, and the
; datum must gain the addend EXACTLY ONCE.
	move.l	#$11110000,($E000).l	; datum, via the identity map
	move.l	#$0000E007,($441C).l	; 8K entry 7: LA $E000 write-protected
	pflusha
	expf	$E000,F_R|F_W|F_SZL|F_TM5|F_WP,$441C,$E003	; handler unprotects
	move.l	#aerr_rel,(expect_pc).l
	move.l	#$00220000,d1
aerr_rel:
	add.l	d1,($E000).l		; RMW: the write protection faults it
	move.l	($E000).l,d0
	chkl	d0,$11330000,155	; the addend landed exactly once
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,3,154		; exactly one write fault

	; The same family, register side effects: a faulting (An)+ / -(An)
	; store must restart with the ORIGINAL address register and leave
	; exactly one increment behind -- NetBSD's copy loops fault like
	; this on every fresh COW page.
	move.l	#$0000E007,($441C).l	; write protect the page again
	pflusha
	expf	$E000,F_W|F_SZL|F_TM5|F_WP,$441C,$E003
	move.l	#aerr_pi,(expect_pc).l
	lea	($E000).l,a0
	move.l	#$0FEE1234,d1
aerr_pi:
	move.l	d1,(a0)+		; write faults, handler unprotects
	move.l	($E000).l,d0
	chkl	d0,$0FEE1234,156	; landed at the original address
	move.l	a0,d0
	chkl	d0,$E004,157		; increment applied exactly once

	move.l	#$0000E007,($441C).l
	pflusha
	expf	$E000,F_W|F_SZL|F_TM5|F_WP,$441C,$E003
	move.l	#aerr_pd,(expect_pc).l
	lea	($E004).l,a0
	move.l	#$0FEE5678,d1
aerr_pd:
	move.l	d1,-(a0)		; predecrement flavour
	move.l	($E000).l,d0
	chkl	d0,$0FEE5678,158
	move.l	a0,d0
	chkl	d0,$E000,159
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,5,160		; the two extra write faults, once each

;----- A7 rollback must not corrupt the supervisor stack (168/169/170/253)
; The killer NetBSD bug, captured live: libc's __cerror ends with
;   MOVE.L (A7)+,(A0)     ; store the error value through the errno pointer
; run in USER mode.  When the (A0) write faults -- routine demand paging --
; the restart model (68060: MC68060UM 8.4.4.1) rolls the A7 post-increment
; back.  A7 is a
; SHADOWED register: writes land in USP, ISP or MSP according to S/M.  If
; the rollback runs after exception entry has set S, the USER stack value
; is written into the SUPERVISOR pointer; the frame is then stacked in
; user space, that write faults too, and the fault-during-exception halts
; the core.  Hardware beacon at the halt: A7=1dfff9b8 (a user stack) in
; supervisor mode, IR=209f, PC in __cerror.
; Here: user code runs exactly that instruction with a non-resident
; destination.  The handler maps the page and returns.  On a broken core
; the supervisor stack pointer is destroyed and the machine dies inside
; exception processing; on a correct one the ISP is untouched, the store
; completes on restart, and A7 (USP) advances exactly one longword.
	move.l	#$4800,d0		; user root -> pointer -> page table
	movec	d0,urp			; (the 154-160 block left URP elsewhere)
	move.l	#$0000A003,($4C14).l	; user code page identity
	move.l	#$00008003,($4C10).l	; user data page VA $8000 resident
	pflusha
	; user code at PA $B000:  move.l (a7)+,(a0) ; trap #1
	move.w	#$209F,($B000).l
	move.w	#$4E41,($B002).l	; trap #1
	cpusha	bc
	cinva	ic
	; user stack at VA $8100 holding the value to store
	move.l	#$C0DE1234,($8100).l
	move.l	#0,($4C18).l		; USER entry 6 (VA $C000) NOT resident
	pflusha
	expf	$C000,F_W|F_SZL|F_TM1|F_PF,$4C18,$C003	; user data write
	move.l	#$B000,(expect_pc).l
	move.w	(cnt_aerr).l,d6
	move.l	sp,d4			; the supervisor stack, before (the
					; 68060 has no ISP/MSP control registers:
					; MOVEC $804 is illegal, so read A7 here)
	move.l	#u7done,(uret).l
	move.l	#$8100,d0
	movec	d0,usp			; USP = the user stack
	lea	($C000).l,a0		; destination: the non-resident page
	move.w	#$0000,-(sp)
	pea	($B000).l
	move.w	#$0000,-(sp)
	rte				; to user: MOVE.L (A7)+,(A0)

u7done:
	move.l	sp,d0			; RTE to user and the trap's RTE balance
	cmp.l	d4,d0
	beq.s	u7isp
	failt	168			; ISP corrupted by the A7 rollback
u7isp:
	movec	usp,d0
	chkl	d0,$8104,169		; USP advanced exactly one longword
	move.l	($C000).l,d0		; the restarted store landed
	chkl	d0,$C0DE1234,170
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,1,253		; exactly one fault
	move.l	#$0000C003,($4C18).l
	move.l	#$4400,d0		; hand URP back as the block found it
	movec	d0,urp
	pflusha

;----- stale (An)+ record must die at non-access exception entry (254-256)
; CHK.W (A2)+,D1 with D1 negative: the EA read SUCCEEDS (a2 advances),
; then the CHK exception is taken.  The rollback records exist solely so
; the access-error path can undo the FAULTING instruction's side effects;
; only fetch_next and that consumer cleared them, so an instruction that
; raises a non-access exception after its EA carried a live record
; through exception_prefetch into its handler.  h_chk's FIRST instruction
; stores to a non-resident page; the access error's rollback then
; "restored" a2 from the stale record -- an unrelated instruction's
; register reverted to a stale value.  h_aerr repairs the page and the
; store restarts; afterwards a2 must still hold the post-increment value.
	move.l	#h_chk,($18).l		; CHK, vector 6
	move.l	#0,($C000).l		; scrub the target while still mapped
	move.w	#$7FFF,(chkbuf).l	; CHK bound operand
	move.l	#0,($4418).l		; supervisor VA $C000 not resident
	pflusha
	expf	$C000,F_W|F_SZL|F_TM5|F_PF,$4418,$C003	; h_chk's store
	move.l	#h_chk,(expect_pc).l
	move.w	(cnt_aerr).l,d6
	lea	(chkbuf).l,a2
	moveq	#-1,d1			; negative: CHK always traps
	chk.w	(a2)+,d1
	lea	(chkbuf+2).l,a0
	cmp.l	a0,a2
	beq.s	stale_rec_ok
	failt	254			; a2 reverted by the stale record
stale_rec_ok:
	move.l	($C000).l,d0
	chkl	d0,$FFFFFFFF,255	; h_chk ran and its store landed
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,1,256		; exactly one access error
	move.l	#unexp,($18).l

;----- MOVES alternate function codes select the ROOT by FC bit 2 (257-260)
; An external audit read MC68040 UM 3.2.5 as requiring FC 0/3/4/7 MOVES to
; BYPASS translation as physical accesses.  WinUAE -- the reference this
; core is validated against, and the source of the cputest corpus -- does
; not do that.  Its 68040 MOVES source read is:
;     bool super = (regs.sfc & 4) != 0;
;     res = mmu_get_user_byte(addr, super, false, sz_byte, false);
; i.e. EVERY function code is translated, and only bit 2 picks the root.
; The same code (sfc_get_*/dfc_put_* in cpummu.cpp) serves the 68060, and
; MC68060UM 4.2.5 carries the same "alternate address space ... used as a
; physical address" sentence next to table 4-2, so on the 68060 this stays
; a manual-versus-reference disagreement, pinned to the reference.  (The
; TT/TM the 68060 reports for these accesses is checked by tests 55/57.)  This core has lost that bet in
; the manual's favour before (the T0 change-of-flow list), so pin the
; behaviour with a test that can actually tell the roots apart: URP and
; SRP address DIFFERENT tables here, so a bypass -- or a super bit taken
; from anything but FC2 -- lands the store in the wrong page.
;   user  VA $C000 -> PA $C000     (URP tables)
;   super VA $C000 -> PA $E000     (SRP tables)
;   super VA $E000 -> PA $C000     (so PA $C000 is readable from here)
	move.l	#$4800,d0
	movec	d0,urp			; user root, distinct from SRP ($4000)
	move.l	#$0000C003,($4C18).l	; user   entry 6: VA $C000 -> PA $C000
	move.l	#$0000E003,($4418).l	; super  entry 6: VA $C000 -> PA $E000
	move.l	#$0000C003,($441C).l	; super  entry 7: VA $E000 -> PA $C000
	pflusha
	move.l	#0,($C000).l		; PA $C000 via super VA $E000 below
	move.l	#0,($E000).l
	pflusha

	moveq	#1,d0			; FC1: user data
	movec	d0,dfc
	move.l	#$11111111,d1
	moves.l	d1,($C000).l
	move.l	($E000).l,d0		; super VA $E000 = PA $C000
	chkl	d0,$11111111,257	; FC1 translated through URP

	moveq	#5,d0			; FC5: supervisor data
	movec	d0,dfc
	move.l	#$55555555,d1
	moves.l	d1,($C000).l
	move.l	($C000).l,d0		; super VA $C000 = PA $E000
	chkl	d0,$55555555,258	; FC5 translated through SRP

	moveq	#0,d0			; FC0: bit 2 clear -> USER, not a bypass
	movec	d0,dfc
	move.l	#$00000000,d1
	moves.l	d1,($C000).l
	move.l	($E000).l,d0
	chkl	d0,0,259		; FC0 landed in the USER page

	moveq	#4,d0			; FC4: bit 2 set -> SUPERVISOR
	movec	d0,dfc
	move.l	#$44444444,d1
	moves.l	d1,($C000).l
	move.l	($C000).l,d0
	chkl	d0,$44444444,260	; FC4 landed in the SUPERVISOR page

	moveq	#5,d0
	movec	d0,dfc			; leave DFC as the block found it
	move.l	#$0000C003,($4418).l
	move.l	#$0000E003,($441C).l
	move.l	#$4400,d0
	movec	d0,urp
	pflusha


;---------------- interrupt-vs-MMU-operation interleave sweeps (161-164)
; The live NetBSD freeze happened inside pmap_enter -- PTE rewrite,
; PFLUSH, fresh table walks -- with a VBL interrupt nested in the same
; window and every interrupt level dead afterwards.  Sweep a delayed
; level-2 request across that whole window.  PFLUSHA empties the ATC
; each round, so the interrupt's own exception entry (vector fetch,
; frame pushes) re-walks the tables while the pipeline is mid-MMU-op:
; exactly the nesting the hardware died in.  Nothing may wedge (the
; bench timeout catches a stall), every request must be taken, and the
; translated accesses must stay correct.
	move.w	(IPLCAP).l,d0	; the interleave sweeps need working IPL
	btst	#0,d0		; injection; benches without it advertise 0
	beq	imix_done	; and the sweeps are bypassed, not faked

	; The bench ports live at PA $F1xx -- inside 8K page 7, which the
	; faulting sweep leaves non-resident.  The interrupt handler must
	; acknowledge through a mapping that never disappears: alias entry 6
	; (LA $C000) onto PA $E000 so VA $D110 reaches the port at PA $F110.
	move.l	#$0000E003,($4418).l
	pflusha
	move.w	#$2000,sr	; open the mask for level 2
	move.l	#$0EE0BEEF,($E000).l
	moveq	#1,d5
imix_a:
	move.w	d5,(IPLDLY).l	; level-2 lands d5 cycles from now
	move.l	#$0000E003,($441C).l
	pflusha			; ATC empty: everything below re-walks
	move.l	($E000).l,d0
	chkl	d0,$0EE0BEEF,162	; translated read correct every round
imix_aw:
	move.w	(cnt_int2).l,d0
	cmp.w	d5,d0		; the request must be delivered before the
	bne.s	imix_aw		; next round arms a new one
	addq.w	#1,d5
	cmp.w	#64,d5
	bls.s	imix_a
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	chkl	d0,64,161	; one interrupt per round, none lost

	; Same sweep with the access FAULTING: fault entry, h_aerr repair,
	; restart, and the pending interrupt all contend in one window --
	; the trap-plus-uvm_fault-plus-interrupt shape from the live stack.
	expf	$E000,F_W|F_SZL|F_TM5|F_PF,$441C,$E003
	move.l	#aerr_swb,(expect_pc).l
	move.w	(cnt_aerr).l,d6
	moveq	#1,d5
imix_b:
	move.w	d5,(IPLDLY).l	; arm FIRST: the port page ($F1xx) is about
	move.l	#0,($441C).l	; to become non-resident, and a faulting arm
	pflusha			; would deadlock the failure reporting too
	move.l	#$0DDF0000,d1
	add.l	d5,d1
aerr_swb:
	move.l	d1,($E000).l	; faults, handler repairs, restart stores
	move.l	($E000).l,d0
	cmp.l	d1,d0
	beq.s	imix_bd
	failt	164		; restarted store lost or doubled
imix_bd:
imix_bw:
	move.w	(cnt_int2).l,d0
	sub.w	#64,d0
	cmp.w	d5,d0
	bne.s	imix_bw
	addq.w	#1,d5
	cmp.w	#64,d5
	bls.s	imix_b
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,64,163	; exactly one repair fault per round
	move.w	#$2700,sr

	; Sweep C: the spl storm.  A level-3 handler does the serintr mask
	; dance while a delayed level-2 request lands at every offset across
	; the handler's life -- entry, mid-storm between mask writes, and the
	; RTE.  The pending 2 must survive every mask transition and be taken
	; exactly once when the mask finally opens.
	move.l	#$0000E003,($441C).l	; page 7 resident again
	pflusha
	move.w	#$2000,sr
	moveq	#1,d5
imix_cc:
	move.w	#96,(storm_scr).l
	move.w	d5,(storm_dly).l
	move.w	#3,(IPLREG).l	; the level-3 device interrupts; h_int3 arms
				; the delayed ports request itself
imix_cw:
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	sub.w	#128,d0		; sweeps A+B consumed 128
	cmp.w	d5,d0		; one more level-2 per round
	bne.s	imix_cw
	addq.w	#1,d5
	cmp.w	#96,d5
	bls.s	imix_cc
	moveq	#0,d0
	move.w	(cnt_int3).l,d0
	chkl	d0,96,165	; one level-3 service per round
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	chkl	d0,224,166	; 128 + 96: every pending 2 delivered
	move.l	($3000).l,d0
	chkl	d0,$11112222,167	; the stormed touches stayed coherent
	move.w	#$2700,sr
	move.l	#$0000C003,($4418).l	; entry 6 back to identity
	pflusha
imix_done:

;------------------------------------------------- NeXTSTEP kernel shape
; NeXT Mach 3.3 runs the kernel through transparent translation windows
; and walks tables only for user space and dynamically mapped kernel
; regions (loadable kernel servers around LA $3C000000).  The observed
; panic is a supervisor instruction fetch at $3C014D9C reporting an
; invalid descriptor: root index $1E, pointer index 0, page index 10
; under 8K pages, walked through SRP while URP points at a different
; tree.  Replay exactly that: map the page, call through it, remap it
; the way pmap_enter does, and prove the used/modified writebacks land
; on their own descriptors only.
	moveq	#0,d0
	movec	d0,tc
	pflusha

	; clear root $4000, decoy user root $5000, pointer table $4600,
	; page tables $4800/$4880
	lea	($4000).l,a0
	move.w	#127,d1
nsclr1:	clr.l	(a0)+
	dbra	d1,nsclr1
	lea	($5000).l,a0
	move.w	#127,d1
nsclr2:	clr.l	(a0)+
	dbra	d1,nsclr2
	lea	($4600).l,a0
	move.w	#127,d1
nsclr3:	clr.l	(a0)+
	dbra	d1,nsclr3
	lea	($4800).l,a0
	move.w	#63,d1
nsclr4:	clr.l	(a0)+
	dbra	d1,nsclr4

	; SRP tree: root[$1E] -> pointer $4600, pointer[0] -> page table
	; $4800, page[10] -> PA $6000
	move.l	#$00004603,($4078).l
	move.l	#$00004803,($4600).l
	move.l	#$00006003,($4828).l

	; target routines at the fault's page offset: move.l #tag,d3 / rts
	move.l	#$263CC0DE,($6D9C).l
	move.l	#$00404E75,($6DA0).l
	move.l	#$263CC0DE,($8D9C).l
	move.l	#$00414E75,($8DA0).l

	; SRP the real tree, URP an empty decoy: a supervisor walk that
	; wrongly uses URP dies on an invalid root entry
	move.l	#$4000,d0
	movec	d0,srp
	move.l	#$5000,d0
	movec	d0,urp

	; kernel-style transparent windows for supervisor code and data
	move.l	#$000FA000,d0
	movec	d0,itt0
	movec	d0,dtt0

	move.l	#$C000,d0	; E=1, P=1: 8K pages
	movec	d0,tc
	pflusha

	; the exact kernel gesture: supervisor jsr into the walked region
	moveq	#0,d3
	jsr	($3C014D9C).l
	chkl	d3,$C0DE0040,200

	; the used bit came back on this walk's descriptors and nowhere else
	move.l	($4078).l,d0
	chkl	d0,$0000460B,201	; root: resident + U
	move.l	($4074).l,d0
	chkl	d0,0,202		; root neighbor untouched
	move.l	($4600).l,d0
	chkl	d0,$0000480B,203	; pointer: resident + U
	move.l	($4604).l,d0
	chkl	d0,0,204		; pointer neighbor untouched
	move.l	($4828).l,d0
	chkl	d0,$0000600B,205	; page: resident + U
	move.l	($4824).l,d0
	chkl	d0,0,206
	move.l	($482C).l,d0
	chkl	d0,0,207

	; a walked write sets M on the page descriptor and lands physically
	move.l	#$5EC70001,($3C014000).l
	move.l	($6000).l,d0
	chkl	d0,$5EC70001,208
	move.l	($4828).l,d0
	chkl	d0,$0000601B,209	; page: U + M

	; pmap_enter: retarget the pointer entry at a fresh page table,
	; flush, and the same LA must reach the new physical page
	move.l	#$00008003,($48A8).l	; new table $4880, page[10] -> $8000
	move.l	#$00004883,($4600).l
	pflusha
	moveq	#0,d3
	jsr	($3C014D9C).l
	chkl	d3,$C0DE0041,210

	move.l	($4828).l,d0
	chkl	d0,$0000601B,211	; old page descriptor left alone
	move.l	($4600).l,d0
	chkl	d0,$0000488B,212	; new pointer entry: resident + U

	; 213/214: a PAGE-CROSSING write leaves no flags pending.  A crossing
	; access is split into byte transfers (S_MWR_B) and never reaches the
	; whole-transfer acknowledge, so a deferred CCR commit that only lives
	; there outlives the instruction: the conditional below reads the flags
	; of whatever ran BEFORE the ADD.  Only reachable with translation on --
	; the split is gated on TC[15], which is why the corpus, running with
	; tc=0, cannot see this at all.
	moveq	#-1,d0			; a REGISTER op sets the CCR last, so what
	tst.l	d0			; the branch reads can only come from the MOVE
	move.l	#0,($DFFE).l		; crossing write: 8K pages here, so this
					; straddles the $E000 boundary between two
					; identity-mapped pages.  Value zero -> Z set
	beq.s	mcz_ok
	failt	213			; Z never reached the CCR: the split write's
					; deferred flags were dropped
mcz_ok:
	; and the value itself is still right across the boundary
	move.l	#$11223344,($DFFE).l
	move.l	($DFFE).l,d0
	chkl	d0,$11223344,214

	; translation off first (the code window is a TTR), then the
	; windows themselves
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	movec	d0,dtt0
	pflusha

	; leave translation off for the harness epilogue
	moveq	#0,d0
	movec	d0,tc
	pflusha

	move.w	#$600D,(DONEREG).l
	stop	#$2700

;----------------------------------------------------------------- handlers
h_chk:
	move.l	d1,($C000).l		; FIRST instruction: faults, restarts
	rte

; Access error (MC68060UM 8.4.4): format $4, vector 2.  After the movem
; below the frame sits at 12(sp): SR 12, PC 14, format/vector 18, fault
; address 20, FSLW 24.  The frame is captured, checked against what the
; test armed with expf, the descriptor (or TTR/TC) repaired, and RTE
; restarts the faulted instruction.  No PFLUSH: a failed table search
; loads no ATC entry (MC68060UM 4.3, 8.4.5 fourth step), so finding the
; descriptor ALREADY repaired means the fault came from an ATC entry --
; a test failure unless the test armed stale_ok (an entry a SUCCESSFUL
; search loaded, which does need the flush: 8.4.5 fifth step).
h_aerr:
	move.w	#1,(aerr_act).l	; a trace taken now is a leaked latch
	cmpi.w	#$4008,6(sp)	; format $4, vector 2
	bne	hf_fmt
	movem.l	d0-d1/a0,-(sp)
	move.l	14(sp),(last_pc).l	; capture for the shape tests
	move.l	20(sp),(last_fa).l
	move.l	24(sp),(last_fslw).l
	move.l	20(sp),d0	; fault address
	cmp.l	(expect_fa).l,d0
	bne	hf_fa
	move.l	24(sp),d0	; FSLW
	and.l	(fslw_mask).l,d0
	cmp.l	(expect_fslw).l,d0
	bne	hf_fslw
	move.l	(expect_pc).l,d0
	beq.s	haerr_pcok
	cmp.l	14(sp),d0	; the faulted instruction (restart model)
	bne	hf_pc
haerr_pcok:
	move.l	(fix_addr).l,d0
	cmp.l	#-1,d0
	bne.s	haerr_ntt
	move.l	(fix_val).l,d0	; a TTR fault: rewrite DTT0
	movec	d0,dtt0
	bra.s	haerr_done
haerr_ntt:
	cmp.l	#-2,d0
	bne.s	haerr_mem
	move.l	(fix_val).l,d0	; a default-translation fault: rewrite TC
	movec	d0,tc
	bra.s	haerr_done
haerr_mem:
	movea.l	d0,a0
	move.l	(a0),d1
	move.l	d1,(seen_desc).l	; descriptor before the handler fixes it
	cmp.l	(fix_val).l,d1
	bne.s	haerr_fix
	tst.w	(stale_ok).l	; already repaired: an ATC entry faulted
	beq	hf_stale
	addq.w	#1,(cnt_stale).l
	pflusha			; the entry a successful search loaded
haerr_fix:
	move.l	(fix_val).l,(a0)
haerr_done:
	addq.w	#1,(cnt_aerr).l
	movem.l	(sp)+,d0-d1/a0
	clr.w	(aerr_act).l
	rte

; hfail with the reason in HFAILID (the bench prints it): 1 not a format $4
; vector 2 frame, 2 wrong FA, 3 wrong FSLW, 4 wrong stacked PC, 5 the
; restarted access faulted again on a repaired descriptor (a failed search
; left an ATC entry behind).  last_fa/last_fslw/last_pc hold the frame.
hf_fmt:
	move.w	#1,(HFAILID).l
	bra	hfail
hf_fa:
	move.w	#2,(HFAILID).l
	bra	hfail
hf_fslw:
	move.w	#3,(HFAILID).l
	bra	hfail
hf_pc:
	move.w	#4,(HFAILID).l
	bra	hfail
hf_stale:
	move.w	#5,(HFAILID).l
	bra	hfail

; format $0 exceptions the PTEST/MMUSR checks provoke: record the
; format/vector word and step over the instruction
h_skip:
	move.w	6(sp),(last_vec).l
	move.l	d0,-(sp)
	moveq	#0,d0
	move.w	(skip_len).l,d0
	add.l	d0,6(sp)	; stacked PC (at 2(sp) before the push)
	move.l	(sp)+,d0
	rte

; arm h_aerr (the expf macro): four longs follow the BSR
setexp:
	move.l	a0,-(sp)
	movea.l	4(sp),a0
	move.l	(a0)+,(expect_fa).l
	move.l	(a0)+,(expect_fslw).l
	move.l	(a0)+,(fix_addr).l
	move.l	(a0)+,(fix_val).l
	move.l	a0,4(sp)
	move.l	#$FFFFFFFF,(fslw_mask).l
	clr.l	(expect_pc).l
	movea.l	(sp)+,a0
	rts

h_int2:
	addq.w	#1,(cnt_int2).l
	move.w	#0,($D110).l	; IPLREG through the always-resident alias:
	rte			; the direct page may be mid-repair (sweep B)

; The NetBSD serintr shape, verbatim from the live freeze: inside a
; level-3 handler, storm the SR mask up and down (splraise/splx pairs at
; $2400/$2500) around translated memory touches, exactly as serintr
; drains its ring -- while a lower-priority request stays pending the
; whole time.  Every mask write resynchronises the pipeline and refills
; the fetch stream through translation.
h_int3:
	movem.l	d0-d1,-(sp)
	move.w	#0,($D110).l	; take the level-3 device down FIRST, then
	move.w	(storm_dly).l,(IPLDLY).l ; arm the ports device: its rise can
	addq.w	#1,(cnt_int3).l	; never race the clear, and the swept delay
	pflusha			; lands it anywhere in the storm below
	moveq	#7,d1
h3storm:
	move.w	sr,d0
	move.w	#$2400,sr	; splraise, serintr-style
	tst.l	($3000).l	; translated data touch
	move.w	#$2500,sr	; deeper raise around the count update
	subq.w	#1,(storm_scr).l
	move.w	d0,sr		; splx back to the entry mask
	dbra	d1,h3storm
	movem.l	(sp)+,d0-d1
	rte

h_utrap:
	ori.w	#$2000,(sp)	; back to supervisor
	move.l	(uret).l,2(sp)
	rte

hfail:
	failt	98

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
