; AP68060: bus snooping under random traffic
; assembled with vasmm68k_mot -Fbin -m68040
;
; While the program runs, the bench's alternate master ($F2C4) issues random
; snooped transfers (tb_ap68060.sv): reads of the program's counters at
; $A000 (long and line; the bench fails if one ever goes backwards), writes
; of the shared words at $B000 (long and line), and reads of them (the bench
; fails on a value nobody wrote).  Every one has SNOOP asserted, so each
; hit invalidates the 68060's line (MC68060UM 5.6).  As the 68060 requires,
; the shared pages are writethrough (5.6: "pages shared with any other bus
; master [must] be marked cachable writethrough or cache inhibited"), which
; the MMU's page descriptors give them.  The program meanwhile:
;   - increments its 64 counters at $A000 (read-modify-write): the cached
;     copy is current or has been invalidated, never stale,
;   - reads shared words and checks each names its writer and address,
;   - writes shared words itself (writethrough: memory has them at once),
;   - increments a long word of each line of $C000-$CFFF in turn: copyback
;     pages whose lines share the sets of $A000 and $B000, so dirty lines
;     are evicted and pushed while it all runs,
;   - pushes the data cache now and then.
; At the end every counter, and every stream line, must hold exactly its
; number of increments.
;
; User data goes through the page tables (MOVES with SFC = DFC = 1);
; supervisor data is noncachable (DTT1), instruction fetches transparent
; (ITT0).
;   1   a shared word that nobody wrote
;   2   a counter that lost an increment
;   3   a stream line that lost an increment

FAILREG	equ	$F100
DONEREG	equ	$F102
TRAFFIC	equ	$F2C4
CNT	equ	$A000		; 64 counters
SHR	equ	$B000		; 64 shared words
STRM	equ	$C000		; streamed, same sets
N	equ	3000		; iterations
ROOT	equ	$10000		; root table, pointer table, one page table
PTRT	equ	$10200
PAGT	equ	$10400		; 64 x 4K pages: logical 0-$3FFFF, identity

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0	; DTT1: supervisor data noncachable
	movec	d0,dtt1
	move.l	#$0000C000,d0	; ITT0: every instruction fetch, writethrough
	movec	d0,itt0
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	lea	(CNT).l,a0		; clear counters and shared words
	move.w	#63,d1
.c1:	clr.l	(a0)+
	dbra	d1,.c1
	lea	(SHR).l,a0
	move.w	#63,d1
.c2:	clr.l	(a0)+
	dbra	d1,.c2
	lea	(STRM).l,a0		; and the stream
	move.w	#1023,d1
.c5:	clr.l	(a0)+
	dbra	d1,.c5
	; page tables: identity, copyback ($020) but for the shared pages
	; $A000 and $B000, writethrough ($000) (descriptors: 4.2.2)
	lea	(PAGT).l,a0
	moveq	#0,d1
.pg:	move.l	d1,d0
	moveq	#12,d3
	lsl.l	d3,d0
	or.w	#$0023,d0		; copyback, resident
	cmp.w	#$A,d1
	beq.s	.wt
	cmp.w	#$B,d1
	bne.s	.cb
.wt:	and.w	#$FF9F,d0		; writethrough
.cb:	move.l	d0,(a0)+
	addq.w	#1,d1
	cmp.w	#64,d1
	bne.s	.pg
	move.l	#PTRT|3,(ROOT).l	; root entry 0 -> pointer table
	move.l	#PAGT|3,(PTRT).l	; pointer entry 0 -> page table
	move.l	#ROOT,d0
	movec	d0,urp
	movec	d0,srp
	move.l	#$8000,d0		; E=1, 4K pages
	movec	d0,tc
	move.l	#$80808000,d0
	movec	d0,cacr
	move.w	#$8008,(TRAFFIC).l	; traffic on, a transfer every 8 bus clocks

	lea	(CNT).l,a1
	lea	(SHR).l,a2
	lea	(STRM).l,a3
	moveq	#0,d2			; iteration
.loop:
	; counter d2 & 63 += 1
	move.l	d2,d3
	and.w	#63,d3
	lsl.w	#2,d3
	moves.l	(a1,d3.w),d0
	addq.l	#1,d0
	moves.l	d0,(a1,d3.w)
	; read a shared word and check it: low word = (4j) ^ $5A5A or ^ $A5A5
	move.l	d2,d3
	mulu.w	#5,d3
	and.w	#63,d3
	lsl.w	#2,d3
	moves.l	(a2,d3.w),d0
	tst.l	d0
	beq.s	.okr
	move.w	d3,d4
	eor.w	#$5A5A,d4
	cmp.w	d4,d0
	beq.s	.okr
	move.w	d3,d4
	eor.w	#$A5A5,d4
	cmp.w	d4,d0
	beq.s	.okr
	move.w	#$2700,sr
	failt	1
.okr:
	; every 4th iteration write a shared word: (iteration << 16) | (4k ^ $A5A5)
	move.l	d2,d3
	and.w	#3,d3
	bne.s	.now
	move.l	d2,d3
	mulu.w	#3,d3
	and.w	#63,d3
	lsl.w	#2,d3
	move.l	d2,d0
	swap	d0
	move.w	d3,d0
	eor.w	#$A5A5,d0
	moves.l	d0,(a2,d3.w)
.now:
	; stream: one line of $C000-$CFFF per iteration, incremented (dirty)
	move.l	d2,d3
	and.w	#$FF,d3
	lsl.w	#4,d3
	moves.l	(a3,d3.w),d0
	addq.l	#1,d0
	moves.l	d0,(a3,d3.w)
	; push everything now and then
	move.l	d2,d3
	and.w	#$FF,d3
	bne.s	.nopush
	cpusha	dc
.nopush:
	addq.l	#1,d2
	cmp.l	#N,d2
	bne	.loop

	clr.w	(TRAFFIC).l		; traffic off, let the last transfer end
	move.w	#200,d1
.w:	nop
	dbra	d1,.w

	; every counter: N/64 increments, one more below N mod 64
	moveq	#0,d3
.chk:	move.l	#N/64,d1
	cmp.w	#N&63,d3
	bcc.s	.c3
	addq.l	#1,d1
.c3:	move.w	d3,d4
	lsl.w	#2,d4
	moves.l	(a1,d4.w),d0
	cmp.l	d1,d0
	beq.s	.c4
	failt	2
.c4:	addq.w	#1,d3
	cmp.w	#64,d3
	bne.s	.chk

	; every stream line: N/256 increments, one more below N mod 256
	moveq	#0,d3
.chs:	move.l	#N/256,d1
	cmp.w	#N&255,d3
	bcc.s	.s3
	addq.l	#1,d1
.s3:	move.w	d3,d4
	lsl.w	#4,d4
	moves.l	(a3,d4.w),d0
	cmp.l	d1,d0
	beq.s	.s4
	failt	3
.s4:	addq.w	#1,d3
	cmp.w	#256,d3
	bne.s	.chs

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	clr.w	(TRAFFIC).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
