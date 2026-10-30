; AP68060 bus errors: what the access error reports (MC68060UM 8.4)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench's one-shot bus error ($F142 arms TEA for the next access to
; $F140) is placed under the access being checked.  Data is noncachable
; precise (TCR DCO = 10) unless a test changes it.
;   1-3   a misaligned long read at $F13E: its first bus cycle ($F13E) is
;         fine, the second ($F140) gets TEA: FSLW RE with MA (the fault is
;         on the second access of a misaligned transfer, 8.4.4.3), FA the
;         operand's first byte; restarted, it completes
;   4-6   the same for a write: WE with MA
;   7-8   an aligned read of $F140: RE without MA
;   9-11  a push that gets TEA (a dirty copyback line at $F140, CPUSHL):
;         recorded and reported as an access error with FSLW PBE (8.4.6),
;         the line gone
;   12-15 FSLW TTR (8.4.4: a fault on an access one of the four TTRs maps,
;         a bus error too): a read and a write of $F140 under DTT0 have RE
;         or WE with TTR; the TCR's default translation does not (8)
;   16-20 an instruction fetch's bus error ($F156: every fetch of a long
;         word): IO, RE, FA the instruction; with TTR under ITT0 (16-18),
;         without it otherwise (19-20)
;
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
BERRARM	equ	$F142
FTEA	equ	$F156		; every fetch of this long word: TEA (0: off)

cnt_ae	equ	$3600		; access errors
ae_fslw	equ	$3604
ae_fa	equ	$3608
ae_pc	equ	$360C

F_MA	equ	$08000000
F_R	equ	$01000000
F_W	equ	$00800000
F_PBE	equ	$00004000
F_RE	equ	$00000020
F_WE	equ	$00000010
F_TTR	equ	$00000008
F_IO	equ	$00008000

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro			; register, value, test
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_ae		; 2 access error
	rept	253
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$00000200,d0	; TCR: default data noncachable precise
	movec	d0,tc
	clr.l	(cnt_ae).l
	; the bench arms $F140 at reset: take that first fault out of the way
	tst.l	($F140).l
	clr.l	(cnt_ae).l

;------------------------------------------------ 1-3 misaligned read
	move.w	#1,(BERRARM).l
	move.l	($F13E).l,d0
	move.l	(cnt_ae).l,d1
	chkl	d1,1,1
	move.l	(ae_fslw).l,d1
	and.l	#F_MA|F_R|F_W|F_RE|F_WE,d1
	chkl	d1,F_MA|F_R|F_RE,2
	move.l	(ae_fa).l,d1
	chkl	d1,$F13E,3

;------------------------------------------------ 4-6 misaligned write
	clr.l	(cnt_ae).l
	move.w	#1,(BERRARM).l
	move.l	#$12345678,($F13E).l
	move.l	(cnt_ae).l,d1
	chkl	d1,1,4
	move.l	(ae_fslw).l,d1
	and.l	#F_MA|F_R|F_W|F_RE|F_WE,d1
	chkl	d1,F_MA|F_W|F_WE,5
	move.l	(ae_fa).l,d1
	chkl	d1,$F13E,6

;------------------------------------------------ 7-8 aligned: no MA
	clr.l	(cnt_ae).l
	move.w	#1,(BERRARM).l
	move.l	($F140).l,d0
	move.l	(cnt_ae).l,d1
	chkl	d1,1,7
	move.l	(ae_fslw).l,d1
	and.l	#F_MA|F_R|F_W|F_RE|F_WE|F_TTR,d1
	chkl	d1,F_R|F_RE,8	; (the default translation: no TTR)

;------------------------------------------------ 9-11 push bus error
	clr.l	(cnt_ae).l
	cinva	dc
	move.l	#$00000100,d0	; TCR: default data copyback
	movec	d0,tc
	move.l	#$80000000,d0	; data cache on
	movec	d0,cacr
	move.l	#$12345678,($F140).l	; a dirty line
	moveq	#0,d0		; the cache off (contents kept), DCO precise:
	movec	d0,cacr		; the arm write goes to the bus
	move.l	#$00000200,d0
	movec	d0,tc
	move.w	#1,(BERRARM).l
	lea	($F140).l,a0
	cpushl	dc,(a0)		; the push gets TEA
	nop			; the access error comes by here
	move.l	(cnt_ae).l,d1
	chkl	d1,1,9
	move.l	(ae_fslw).l,d1
	and.l	#F_PBE,d1
	chkl	d1,F_PBE,10
	move.l	(ae_fslw).l,d1
	and.l	#F_RE|F_WE,d1
	chkl	d1,0,11		; (not an operand's bus error)

;------------------------------------------------ 12-15 TTR-mapped data
	; DTT0: $00xxxxxx, both modes, noncachable precise
	move.l	#$0000C040,d0
	movec	d0,dtt0
	clr.l	(cnt_ae).l
	move.w	#1,(BERRARM).l
	move.l	($F140).l,d0
	move.l	(cnt_ae).l,d1
	chkl	d1,1,12
	move.l	(ae_fslw).l,d1
	and.l	#F_MA|F_R|F_W|F_RE|F_WE|F_TTR,d1
	chkl	d1,F_R|F_RE|F_TTR,13
	clr.l	(cnt_ae).l
	move.w	#1,(BERRARM).l
	move.w	#$1234,($F140).l
	move.l	(cnt_ae).l,d1
	chkl	d1,1,14
	move.l	(ae_fslw).l,d1
	and.l	#F_MA|F_R|F_W|F_RE|F_WE|F_TTR,d1
	chkl	d1,F_W|F_WE|F_TTR,15
	moveq	#0,d0
	movec	d0,dtt0

;------------------------------------------------ 16-20 instruction fetch
	; ITT0 the same: the fetch's bus error has TTR (the handler drops the
	; TEA, the restart fetches the instruction)
	move.l	#$0000C040,d0
	movec	d0,itt0
	clr.l	(cnt_ae).l
	move.w	#ftgt1&$FFFF,(FTEA).l
	nop			; (serializes: the target is fetched after the arm)
	jmp	(ftgt1).l
ft_back1:
	moveq	#0,d0
	movec	d0,itt0
	move.l	(cnt_ae).l,d1
	chkl	d1,1,16
	move.l	(ae_fslw).l,d1
	and.l	#F_IO|F_R|F_W|F_RE|F_WE|F_TTR,d1
	chkl	d1,F_IO|F_R|F_RE|F_TTR,17
	move.l	(ae_fa).l,d1
	chkl	d1,ftgt1,18
	; without a TTR: no TTR
	clr.l	(cnt_ae).l
	move.w	#ftgt2&$FFFF,(FTEA).l
	nop			; (serializes: the target is fetched after the arm)
	jmp	(ftgt2).l
ft_back2:
	move.l	(cnt_ae).l,d1
	chkl	d1,1,19
	move.l	(ae_fslw).l,d1
	and.l	#F_IO|F_R|F_W|F_RE|F_WE|F_TTR,d1
	chkl	d1,F_IO|F_R|F_RE,20

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

; access error: record, then restart (the one-shot TEA is spent, the
; fetches' TEA dropped)
h_ae:
	clr.w	(FTEA).l
	addq.l	#1,(cnt_ae).l
	move.l	12(sp),(ae_fslw).l
	move.l	8(sp),(ae_fa).l
	move.l	2(sp),(ae_pc).l
	cmpi.w	#$4008,6(sp)
	bne.s	h_ae_bad
	rte
h_ae_bad:
	failt	98

unexp:
	failt	98

; the fetch tests' targets, each its own long word (FTEA)
	cnop	0,16
ftgt1:	nop
	nop
	jmp	(ft_back1).l
	cnop	0,16
ftgt2:	nop
	nop
	jmp	(ft_back2).l
