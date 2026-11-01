; AP68060: an IDE driver's traffic under interrupts at random moments.
; Under Minimig a total hang came at random times during IDE use (the
; Gayle's port: uncached, slow; its INT2 at any moment).  Each round arms
; a level 2 interrupt a pseudo-random 1-1024 clocks ahead ($F148), then
; runs one burst, picked pseudo-randomly:
;   0  128 long reads of the "data port" (uncached) into a copyback buffer
;   1   64 word reads of it
;   2  task-file byte writes and a status poll (byte reads)
;   3  128 long writes to the port from the buffer (the store buffer)
;   4  STOP #$2000 until the interrupt (exec's idle while a task waits)
; and waits (bounded) until the interrupt was taken.  The reads are
; checked against the port's contents every round.  Failures:
;   1  an unexpected exception      2  the interrupt never taken
;   3  a read copied wrong data     4  the program restarted
DONEREG	equ	$F102
TESTREG	equ	$F100
IPLREG	equ	$F110
IPLDLY	equ	$F148		; raise IPL 2 N clocks from now
PORT	equ	$4000		; the "IDE data port": uncached (DTT0), slow
TF	equ	$4400		; its task file (bytes)
BUF	equ	$50000000	; Fast RAM, copyback (DTT1)
ROUNDS	equ	1500

failt	macro
	move.w	#\1,(TESTREG).l
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
	move.l	#$0000c040,d0	; DTT0: $00xxxxxx data cache inhibited, precise
	movec	d0,dtt0
	move.l	#$5000c020,d0	; DTT1: $50xxxxxx copyback
	movec	d0,dtt1
	move.l	#$A0808000,d0	; data, store buffer, branch, instruction caches
	movec	d0,cacr
	; the port's contents: a pattern (its long word i = $A5000000 + i*$10001)
	lea	(PORT).l,a0
	move.l	#$A5000000,d0
	move.w	#127,d1
.fill:	move.l	d0,(a0)+
	add.l	#$10001,d0
	dbra	d1,.fill
	moveq	#0,d4		; the interrupts taken
	move.l	#$1234ACE1,d6	; the LFSR
	move.l	#ROUNDS,d5
	move.w	#$2000,sr	; interrupts on
round:
	bsr	lfsr
	move.l	d6,d0
	and.w	#$3FF,d0
	addq.w	#1,d0
	move.l	d4,d3		; the count before this round's interrupt
	move.w	d0,(IPLDLY).l	; IPL 2 in 1..1024 clocks
	bsr	lfsr
	move.l	d6,d0
	lsr.l	#8,d0
	and.l	#7,d0
	cmp.w	#4,d0
	bls.s	.k
	sub.w	#4,d0		; (5..7 -> 1..3)
.k:	add.w	d0,d0
	move.w	.tab(pc,d0.w),d0
	jsr	.tab(pc,d0.w)
	; the interrupt taken (bounded wait)
	move.l	#20000,d1
.w:	cmp.l	d3,d4
	bne.s	.got
	subq.l	#1,d1
	bne.s	.w
	failt	2
.got:	subq.l	#1,d5
	bne	round
	move.w	#$600D,(DONEREG).l
.done:	bra.s	.done
.tab:	dc.w	b_lread-.tab,b_wread-.tab,b_tf-.tab,b_lwrite-.tab,b_stop-.tab

; 0: 128 long reads into the buffer, checked
b_lread:
	lea	(PORT).l,a0
	lea	(BUF).l,a1
	move.w	#127,d1
.r:	move.l	(a0)+,(a1)+
	dbra	d1,.r
	lea	(BUF).l,a1
	move.l	#$A5000000,d0
	move.w	#127,d1
.c:	cmp.l	(a1)+,d0
	bne	badcopy
	add.l	#$10001,d0
	dbra	d1,.c
	rts
; 1: 64 word reads, checked (the port's first 32 long words' halves)
b_wread:
	lea	(PORT).l,a0
	lea	(BUF+$400).l,a1
	moveq	#63,d1
.r:	move.w	(a0)+,(a1)+
	dbra	d1,.r
	lea	(BUF+$400).l,a1
	lea	(PORT).l,a0
	moveq	#31,d1
.c:	move.l	(a0)+,d0
	cmp.l	(a1)+,d0
	bne	badcopy
	dbra	d1,.c
	rts
; 2: the task file's byte writes, then a status poll
b_tf:
	lea	(TF).l,a0
	move.b	#1,2(a0)	; sector count
	move.b	d6,3(a0)	; LBA 0-7
	move.b	#0,4(a0)
	move.b	#0,5(a0)
	move.b	#$E0,6(a0)	; drive/head
	move.b	#$20,7(a0)	; command: READ SECTORS
	moveq	#15,d1
.p:	tst.b	7(a0)		; status
	tst.b	1(a0)		; error
	dbra	d1,.p
	rts
; 3: 128 long writes to the port from the buffer (the same pattern back)
b_lwrite:
	lea	(PORT).l,a0
	move.l	#$A5000000,d0
	move.w	#127,d1
.w:	move.l	d0,(a0)+
	add.l	#$10001,d0
	dbra	d1,.w
	rts
; 4: wait in STOP: the mask raised first, then an interrupt requested a
; little later, which STOP's lower mask lets in.  (A request already
; pending when the mask rises is still taken at the next boundary -- the
; 68060's IPEND, MC68060UM figure 7-25 -- so the request must follow the
; raised mask; AmigaOS's idle STOP is ended by the periodic VBL anyway.)
b_stop:
	move.w	#$2700,sr
	move.w	#40,(IPLDLY).l	; IPL 2 in 40 clocks: recognised only once STOP lowers the mask
	stop	#$2000
	rts

lfsr:	lsr.l	#1,d6		; Galois LFSR, taps $80200003
	bcc.s	.z
	eor.l	#$80200003,d6
.z:	rts

h_irq2:
	move.w	#0,(IPLREG).l	; the request withdrawn
	addq.l	#1,d4
	rte

unexp:	failt	1
badcopy: failt	3
restarted: failt 4

	even
started: dc.w	0
