; AP68060 caches: line fill errors, locked accesses, page-scope maintenance
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench's watch window ($F2D0 start, $F2D4 end) counts the processor's
; completed data bus reads ($F2D8) and writes ($F2DC, pushes included) in a
; range.  Supervisor data is noncachable through DTT1 except where a test
; maps it otherwise.
;   1-3  a data line fill whose LATER beat gets TEA ($F14C read: the burst
;        starts at $F14C and wraps to $F140, which fails): the long word the
;        load needs arrived, so no exception (MC68060UM 7.9.1, 5.7.1), the
;        load gets it, and the line is not allocated -- a second load fills
;        again (1 good beat + 4 beats in the window)
;   4    the same on the instruction side: the fetched long word arrived,
;        a later beat of its line failed: it executes, no exception
;   5-7  accesses locked through BUSCR bypass the data cache (5.2): with a
;        dirty line cached, a locked read goes to the bus (the line pushed
;        first) and a locked write reaches memory; the stale line is not
;        pushed over it afterwards
;   8-9  CPUSHP with 8K pages (TCR.P) pushes a dirty line anywhere in the
;        8K page, and none outside it
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
BERRARM	equ	$F142		; one-shot TEA on the next access to $F140
FBERR	equ	$F154		; one-shot TEA on a fetch of the long word at
WLO	equ	$F2D0
WHI	equ	$F2D4
WRD	equ	$F2D8
WWR	equ	$F2DC
cnt_ae	equ	$3600
resume	equ	$3604

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

watch	macro			; start, end: clear the counters
	move.l	#\1,(WLO).l
	move.l	#\2,(WHI).l
	clr.l	(WRD).l
	clr.l	(WWR).l
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
	dc.w	$F4D8		; cinva bc
	move.l	#$0000A040,d0	; DTT1: supervisor $00xxxxxx noncachable
	movec	d0,dtt1
	move.l	#spent,(resume).l
	tst.l	($F140).l	; the reset-armed one-shot TEA, spent here
spent:	clr.l	(cnt_ae).l
	clr.l	(resume).l	; (from here every access error is a failure)

;------------------------------------------------ 1-3 a late data fill error
	move.w	#1,(BERRARM).l
	watch	$F140,$F150
	moveq	#0,d0
	movec	d0,dtt1		; no TTR: TCR's defaults apply
	move.l	#$00000100,d0	; DCO copyback: $F14C is cachable
	movec	d0,tc
	move.l	#$80000000,d0	; data cache on
	movec	d0,cacr
	move.l	($F14C).l,d0	; fill: $F14C fine, $F140 (third beat) TEA
	move.l	($F14C).l,d1	; not allocated: a new fill
	moveq	#0,d2
	movec	d2,cacr
	movec	d2,tc
	move.l	#$0000A040,d2
	movec	d2,dtt1
	move.l	(cnt_ae).l,d2
	chkl	d2,0,1		; no exception
	move.l	(WRD).l,d2
	chkl	d2,5,2		; 1 good beat, then a whole line again
	cmp.l	d0,d1
	beq.s	ok3
	failt	3
ok3:

;------------------------------------------------ 4 a late instruction fill error
	dc.w	$F4D8		; cinva bc
	move.l	#$00008000,d0	; instruction cache on
	movec	d0,cacr
	nop
	move.w	#ifill_bad&$FFFF,(FBERR).l
	jmp	(ifill_go).l
ifill_back:
	moveq	#0,d0
	movec	d0,cacr
	move.w	#0,(FBERR).l
	move.l	(cnt_ae).l,d2
	chkl	d2,0,4

;------------------------------------------------ 5-7 BUSCR-locked accesses
	dc.w	$F4D8		; cinva bc
	move.l	#$11111111,($9000).l	; memory (noncachable here)
	move.l	#$0000A020,d0	; DTT0: supervisor $00xxxxxx copyback
	movec	d0,dtt0		; (DTT1 also matches: DTT0 wins)
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	($9000).l,d0	; allocate
	move.l	#$33333333,($9000).l	; now dirty, memory still $11111111
	move.l	#$A0000000,d0	; BUSCR L and LE
	dc.w	$4E7B,$0008	; movec d0,buscr
	move.l	($9000).l,d1	; locked read: the line pushed, then the bus
	move.l	#$22222222,($9000).l	; locked final write: the bus
	moveq	#0,d0
	dc.w	$4E7B,$0008	; movec d0,buscr: L and LE cleared
	dc.w	$F4F8		; cpusha bc: nothing dirty must remain
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0		; supervisor data noncachable again (DTT1)
	chkl	d1,$33333333,5	; the locked read saw the pushed data
	move.l	($9000).l,d0
	chkl	d0,$22222222,6	; memory has the locked write
	dc.w	$F4D8		; (the line, if any, not in the cache any more)
	move.l	($9004).l,d0	; the rest of the line as it was in memory
	chkl	d0,0,7

;------------------------------------------------ 8-9 CPUSHP with 8K pages
	dc.w	$F4D8		; cinva bc
	move.l	#$00004000,d0	; TCR: P (8K), translation off
	movec	d0,tc
	move.l	#$0000A020,d0	; supervisor data copyback
	movec	d0,dtt0
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	#$12345678,($A000).l	; a dirty line in the 8K page $A000
	moveq	#0,d0		; (the counters through DTT1: noncachable --
	movec	d0,dtt0		; the line stays dirty in the cache)
	watch	$A000,$A010
	move.l	#$0000A020,d0
	movec	d0,dtt0
	lea	($C000).l,a0
	dc.w	$F470		; cpushp dc,(a0): another 8K page
	lea	($B000).l,a0
	moveq	#0,d0
	movec	d0,dtt0
	move.l	(WWR).l,d2
	dc.w	$F470		; cpushp dc,(a0): the same 8K page
	move.l	(WWR).l,d3
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	chkl	d2,0,8		; nothing pushed for $C000
	chkl	d3,4,9		; the line pushed for $B000 (four beats)

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_ae:
	addq.l	#1,(cnt_ae).l
	tst.l	(resume).l
	beq.s	h_bad
	move.l	(resume).l,2(sp)
	rte
h_bad:
	failt	98

unexp:
	failt	98

; the instruction fill test: a line whose first fetched long word is fine
; and whose next one gets TEA
	org	$2000
	nop
	nop
	nop
	nop
ifill_go:
	bra.w	ifill_far	; $2008: the critical long word
ifill_bad:
	dc.w	$4AFC,$4AFC	; $200C: never executed, its beat gets TEA
ifill_far:
	jmp	(ifill_back).l
