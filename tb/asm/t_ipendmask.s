; AP68060: an interrupt is never taken inside a section the mask closed.
; exec keeps interrupt code out of its dispatcher's list work (TaskReady,
; TaskWait) with ORI.W #$0700,SR / MOVE.W #$2700,SR; an interrupt server
; that runs Signal() inside it Enqueue()s into a list the dispatcher is
; unlinking, and tasks are lost (under Minimig: scsi.device signalled but
; on no list, a total IDE hang).  Each round raises IPL 2 a swept number
; of clocks ($F148) around the instruction that raises the mask, so the
; request reaches the processor before it, with it, and after it.
; A request that beat the old mask may still be taken at the boundary of
; the instruction raising it (IPEND, MC68060UM 7.8.1): the handler then
; sees the new mask stacked, but the PC stacked is the instruction right
; after the raise, and nothing of the section has run.  At any later
; boundary inside the section it must not be taken.  Every request must
; be taken once the mask is lowered again.
;   1   an unexpected exception
;   2   taken with the stacked mask at 2 or above, past the raise's boundary
;   3   taken inside the critical section (its first instruction had run)
;   4   the interrupt was never taken
;   5   it was taken more than once
;   6   the program restarted
FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148		; raise IPL 2 N clocks from now
NDLY	equ	48		; delays 1..NDLY clocks
ROUNDS	equ	NDLY*12		; each delay with each of the 12 variants

failt	macro
	move.w	#\1,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
.h\@:	bra.s	.h\@
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	24
	dc.l	unexp		; vectors 2..25
	endr
	dc.l	h_irq2		; 26: level 2 autovector
	rept	229
	dc.l	unexp		; vectors 27..255
	endr

	org	$400
start:
	move.w	#$2700,sr
	tst.w	started
	bne	restarted
	move.w	#1,started
	move.l	#$0000A040,d0	; DTT1: supervisor data noncachable (the bench registers)
	movec	d0,dtt1
	move.l	#$A0808000,d0	; data, store buffer, branch, instruction caches
	movec	d0,cacr
	moveq	#0,d4		; interrupts taken
	moveq	#0,d5		; round
	moveq	#1,d6		; delay, 1..NDLY
	moveq	#0,d3		; variant, 0..11
round:
	move.w	#$2000,sr	; interrupts on
	move.l	d4,d2		; the count before this round's request
	clr.w	incrit
	; the variant: which mask-raising instruction, and how much filler
	; before it (0, 1 or 2 short instructions; with the delay sweep this
	; puts the request on every clock around it)
	move.w	d3,d0
	add.w	d0,d0
	lea	.rt(pc),a0
	move.l	(a0,d0.w*2),raisepc
	move.w	.vt(pc,d0.w),d0
	move.w	d6,(IPLDLY).l	; IPL 2 in d6 clocks
	jmp	.vt(pc,d0.w)
.vt:	dc.w	v_ori0-.vt,v_ori1-.vt,v_ori2-.vt,v_mov0-.vt,v_mov1-.vt,v_mov2-.vt
	dc.w	v_ori0n-.vt,v_ori1n-.vt,v_ori2n-.vt,v_mov0n-.vt,v_mov1n-.vt,v_mov2n-.vt
	cnop	0,4
.rt:	dc.l	r_ori,r_ori,r_ori,r_mov,r_mov,r_mov,r_orin,r_orin,r_orin,r_movn,r_movn,r_movn

crit	macro
	move.w	#1,incrit	; the section's first instruction
	bra	section
	endm

v_ori2:	addq.l	#1,d1
v_ori1:	addq.l	#1,d1
v_ori0:	ori.w	#$0700,sr
r_ori:	crit
v_mov2:	addq.l	#1,d1
v_mov1:	addq.l	#1,d1
v_mov0:	move.w	#$2700,sr
r_mov:	crit
; (the same with a NOP first: a pipeline-synchronising instruction ahead)
v_ori2n: addq.l	#1,d1
v_ori1n: addq.l	#1,d1
v_ori0n: nop
	ori.w	#$0700,sr
r_orin:	crit
v_mov2n: addq.l	#1,d1
v_mov1n: addq.l	#1,d1
v_mov0n: nop
	move.w	#$2700,sr
r_movn:	crit

; the critical section: exec's list work under the raised mask
section:
	lea	list,a0
	move.l	(a0),a1		; (a few dependent loads and stores, as an
	move.l	4(a0),a2	; Enqueue/Remove does)
	move.l	a2,(a0)
	move.l	a1,4(a0)
	move.l	(a0),a1
	move.l	4(a0),a2
	move.l	a2,(a0)
	move.l	a1,4(a0)
	moveq	#20,d0
.spin:	dbra	d0,.spin	; long enough for any request to be due
	clr.w	incrit
	andi.w	#$f8ff,sr	; mask 0: the request is taken here (or soon)
	; the request taken (bounded wait)
	move.l	#4000,d1
.w:	cmp.l	d2,d4
	bne.s	.got
	subq.l	#1,d1
	bne.s	.w
	failt	4
.got:	move.l	d2,d0
	addq.l	#1,d0
	cmp.l	d0,d4
	beq.s	.once
	failt	5
.once:	; next round: next delay; after NDLY delays, the next variant
	addq.w	#1,d6
	cmp.w	#NDLY,d6
	bls.s	.nd
	moveq	#1,d6
	addq.w	#1,d3
.nd:	addq.l	#1,d5
	cmp.l	#ROUNDS,d5
	bne	round
	move.w	#$600D,(DONEREG).l
.done:	bra.s	.done

h_irq2:
	move.w	(sp),d0		; the SR it interrupted
	and.w	#$0700,d0
	cmp.w	#$0200,d0
	bcs.s	.mok
	move.l	2(sp),d0	; masked: only at the raise's own boundary
	cmp.l	raisepc,d0
	beq.s	.mok
	failt	2
.mok:	tst.w	incrit
	beq.s	.cok
	failt	3
.cok:	move.w	#0,(IPLREG).l	; the request withdrawn
	addq.l	#1,d4
	rte

unexp:	failt	1
restarted: failt 6

	even
started: dc.w	0
incrit:	dc.w	0
	cnop	0,4
raisepc: dc.l	0
	cnop	0,4
list:	dc.l	list+4,list
