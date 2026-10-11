; AP68060 front end: the fetch BTB, the D1 checks, the return stack
; assembled with vasmm68k_mot -Fbin -m68040
;
; The BTB is a prediction structure: whatever it holds, the program must
; run as written.  These tests build the situations in which an entry is
; wrong and check the architectural results:
;   1-4  a branch ending in word 0 or 1 of a long word, entered at either
;        word, taken many times (the entry is used)
;   5    a conditional branch that turns: taken 40 times, then not (D1
;        overrules the entry, the history table relearns)
;   6-9  an entry left over at an address that now holds the middle of a
;        longer instruction (code rewritten, CPUSHA as the 68040 requires):
;        the flagged word is inside an instruction, in word 0 and word 1
;  10-12 an entry whose branch now jumps elsewhere (rewritten displacement),
;        and whose word is now the first of a two-word branch
;  13    short branches back to back (the target FIFO fills up)
;  14-15 calls and returns nested deeper than the return stack
;  16    a computed JMP through a register to changing targets
;  17    BTB index aliasing: two branches 1 KB apart (the 256-entry BTB
;        indexes PC[9:2]), both taken
;  18-19 returns the return stacks cannot predict: RTS to a pushed
;        address, and a call that leaves by a jump (a stale entry)
;  20-23 the 68060's branch cache controls (MC68060UM 5.2, 5.11), counted
;        through the bench's count of the predictions D1 confirms ($F2EC):
;        CACR.EBC clear,
;        nothing is predicted; CACR.CABC (and CUBC) clear every entry, and
;        so does any CINV/CPUSH of the instruction cache; CABC and CUBC
;        read as zero

FAILREG	equ	$F100
DONEREG	equ	$F102
BCPRED	equ	$F2EC		; branch cache predictions followed (bench)
CODE	equ	$6000		; rewritten code (test 6, 7)

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80808000,d0
	movec	d0,cacr

;-------------------------------------- 1-4: branch end in word 0 / word 1
	moveq	#0,d0
	move.w	#99,d1
	cnop	0,4
.l1:	addq.l	#1,d0		; long word: addq | bra.s (ends in word 1)
	bra.s	.n1
	nop
.n1:	dbra	d1,.l1
	chkl	d0,100,1

	moveq	#0,d0
	move.w	#99,d1
	cnop	0,4
.l2:	bra.s	.n2		; ends in word 0
	nop
	nop
.n2:	addq.l	#2,d0
	dbra	d1,.l2
	chkl	d0,200,2

	moveq	#0,d0
	move.w	#99,d1
	cnop	0,4
	nop
.l3:	bra.w	.n3		; starts in word 1, ends in word 0 of the next
	nop
.n3:	addq.l	#3,d0
	dbra	d1,.l3
	chkl	d0,300,3

	moveq	#0,d0
	move.w	#99,d1
	bra.s	.e4
	cnop	0,4
.l4:	nop			; entered at word 1 below: the BTB entry for
.e4:	bra.s	.n4		; this long word ends in word 1
	nop
.n4:	addq.l	#4,d0
	dbra	d1,.l4
	chkl	d0,400,4

;------------------------------------------------ 5: a branch that turns
	moveq	#0,d0
	moveq	#0,d2
	move.w	#79,d1
.l5:	cmp.w	#40,d2
	bcc.s	.n5		; not taken 40 times, then taken (forward)
	addq.l	#1,d0
.n5:	addq.w	#1,d2
	cmp.w	#40,d2
	bcs.s	.m5		; taken 39 times, then not (forward)
	add.l	#$100,d0
.m5:	dbra	d1,.l5
	chkl	d0,$2928,5	; 40 x 1 + 41 x $100

;------------------- 6: an old entry inside a longer instruction (rewrite)
; A: a BRA.S ending in word 0 of the long word at CODE gets an entry
	lea	(CODE).l,a0
	move.w	#$6002,(a0)	; CODE:   bra.s CODE+4
	move.w	#$7001,2(a0)	;         moveq #1,d0 (skipped)
	move.w	#$7002,4(a0)	; CODE+4: moveq #2,d0
	move.w	#$4E75,6(a0)	;         rts
	cpusha	bc
	move.w	#20,d1
.l6:	moveq	#0,d0
	jsr	(CODE).l
	dbra	d1,.l6
	chkl	d0,2,6
; B: CODE is now the high immediate word of a MOVE.L starting at CODE-2:
; the flagged word is inside the instruction
	move.w	#$203C,-2(a0)	; CODE-2: move.l #$11112222,d0
	move.w	#$1111,(a0)
	move.w	#$2222,2(a0)
	move.w	#$4E75,4(a0)	; CODE+4: rts
	cpusha	bc
	moveq	#0,d0
	jsr	(CODE-2).l
	chkl	d0,$11112222,7
; C: a BRA.S ending in word 1 (CODE+2)
	move.w	#$4E71,(a0)	; CODE:   nop
	move.w	#$6002,2(a0)	; CODE+2: bra.s CODE+6
	move.w	#$7001,4(a0)	;         moveq #1,d0 (skipped)
	move.w	#$7002,6(a0)	; CODE+6: moveq #2,d0
	move.w	#$4E75,8(a0)	;         rts
	cpusha	bc
	move.w	#20,d1
.l6c:	moveq	#0,d0
	jsr	(CODE).l
	dbra	d1,.l6c
	chkl	d0,2,8
; D: CODE+2 is now the high immediate word of a MOVE.L at CODE
	move.w	#$203C,(a0)	; CODE: move.l #$33334444,d0
	move.w	#$3333,2(a0)
	move.w	#$4444,4(a0)
	move.w	#$4E75,6(a0)	;       rts
	cpusha	bc
	moveq	#0,d0
	jsr	(CODE).l
	chkl	d0,$33334444,9

;------------------------------------- 7: the same branch, a new target
	move.w	#$6004,(a0)	; CODE:   bra.s CODE+6
	move.w	#$7001,2(a0)	;         moveq #1,d0 (skipped)
	move.w	#$5880,4(a0)	; CODE+4: addq.l #4,d0
	move.w	#$5080,6(a0)	; CODE+6: addq.l #8,d0
	move.w	#$4E75,8(a0)	;         rts
	cpusha	bc
	move.w	#20,d1
.l7:	moveq	#0,d0
	jsr	(CODE).l
	dbra	d1,.l7
	chkl	d0,8,10
	move.w	#$6002,(a0)	; bra.s CODE+4: the entry still says CODE+6
	cpusha	bc
	moveq	#0,d0
	jsr	(CODE).l
	chkl	d0,12,11
	move.w	#$6000,(a0)	; bra.w CODE+4: two words, so the entry's word
	move.w	#$0002,2(a0)	; (CODE) is now inside the branch
	cpusha	bc
	moveq	#0,d0
	jsr	(CODE).l
	chkl	d0,12,12

;----------------------------- 8: one-word branches back to back (FIFO)
	moveq	#0,d0
	move.w	#49,d1
.l8:	bra.s	.a8
	nop
.a8:	bra.s	.b8
	nop
.b8:	bra.s	.c8
	nop
.c8:	bra.s	.d8
	nop
.d8:	bra.s	.e8
	nop
.e8:	bra.s	.f8
	nop
.f8:	addq.l	#1,d0
	dbra	d1,.l8
	chkl	d0,50,13

;------------------- 9: calls nested deeper than the return stack (8)
	moveq	#0,d0
	moveq	#11,d1		; depth 12
	bsr	deep
	chkl	d0,12,14
	moveq	#0,d0
	move.w	#9,d2
.l9:	moveq	#11,d1
	bsr	deep
	dbra	d2,.l9
	chkl	d0,120,15

;--------------------------------- 10: a computed JMP to changing targets
	moveq	#0,d0
	move.w	#29,d1
	lea	(.t10a).l,a1
	lea	(.t10b).l,a2
.l10:	exg	a1,a2
	jmp	(a1)
.t10a:	addq.l	#1,d0
	bra.s	.n10
.t10b:	add.l	#$10,d0
.n10:	dbra	d1,.l10
	chkl	d0,$F0+15,16	; 15 x 1 + 15 x $10

;------------------------- 11: two branches with the same BTB index
	moveq	#0,d0
	move.w	#49,d1
.l11:	jsr	(alias1).l
	jsr	(alias2).l
	dbra	d1,.l11
	chkl	d0,50*3,17

;------------- 18-19: returns the stacks cannot know (correct regardless)
	moveq	#0,d0
	move.w	#19,d1
.l18:	pea	(.t18).l		; RTS to an address the program pushed
	rts
	addq.l	#8,d0			; not reached
.t18:	addq.l	#1,d0
	dbra	d1,.l18
	chkl	d0,20,18
	moveq	#0,d0
	move.w	#19,d1
l19:	bsr	noret			; returns by a jump: a stale stack entry
b19:	addq.l	#2,d0
	bsr	ret1			; the next return finds the stale one first
	dbra	d1,l19
	chkl	d0,20*(1+2+4),19
	bra.s	done19
noret:	addq.l	#4,sp			; drop the return address
	addq.l	#1,d0
	jmp	(b19).l
ret1:	addq.l	#4,d0
	rts
done19:

;------------------------------------------- 20-23: branch cache control
	; (the bench counter is read noncachable: supervisor data by DTT1)
	move.l	#$0000A040,d0
	movec	d0,dtt1
	; 20: EBC clear: a taken branch is never predicted
	move.l	#$80008000,d0		; EDC, EIC, no EBC
	movec	d0,cacr
	clr.l	(BCPRED).l
	bsr	bc_loop
	move.l	(BCPRED).l,d0
	chkl	d0,0,20
	; ... EBC set: the loop's branch is predicted once it is known
	move.l	#$80808000,d0
	movec	d0,cacr
	bsr	bc_loop			; (learns)
	clr.l	(BCPRED).l
	bsr	bc_loop
	move.l	(BCPRED).l,d0
	cmp.l	#30,d0
	bcc.s	.ok20
	failt	20
.ok20:
	; 21: CABC: the learned entry is gone, the first pass is not predicted
	bsr	bc_loop
	move.l	#$80C08000,d0		; EDC, EBC, CABC, EIC
	movec	d0,cacr
	clr.l	(BCPRED).l
	bsr	bc_one			; one taken branch, cold
	move.l	(BCPRED).l,d0
	cmp.l	#2,d0
	bcs.s	.cold0
	failt	21
.cold0:
	bsr	bc_one			; and now learned: its BRA and RTS
	clr.l	(BCPRED).l
	bsr	bc_one
	move.l	(BCPRED).l,d0
	cmp.l	#2,d0
	bcc.s	.warm21
	failt	21
.warm21:
	; ... CUBC clears too (no supervisor/user split is kept: the entry
	; a supervisor branch made goes as well, which costs only a prediction)
	move.l	#$80A08000,d0		; EDC, EBC, CUBC, EIC
	movec	d0,cacr
	clr.l	(BCPRED).l
	bsr	bc_one
	move.l	(BCPRED).l,d0
	cmp.l	#2,d0
	bcs.s	.cold1
	failt	21
.cold1:
	; 22: CINVA of the instruction cache clears the branch cache (5.11)
	bsr	bc_one
	bsr	bc_one
	cinva	ic
	clr.l	(BCPRED).l
	bsr	bc_one
	move.l	(BCPRED).l,d0
	cmp.l	#2,d0
	bcs.s	.cold2
	failt	22
.cold2:
	; ... a data cache CINV does not
	bsr	bc_one
	cinva	dc
	clr.l	(BCPRED).l
	bsr	bc_one
	move.l	(BCPRED).l,d0
	cmp.l	#2,d0
	bcc.s	.warm22
	failt	22
.warm22:
	; 23: CABC and CUBC read as zero; EBC is kept
	move.l	#$80E08000,d0
	movec	d0,cacr
	movec	cacr,d1
	chkl	d1,$80808000,23
	bra	done23

	cnop	0,4
bc_loop:
	move.w	#39,d1
.bl:	bra.s	.bn			; a taken branch, forty times
	nop
.bn:	dbra	d1,.bl
	rts
	cnop	0,4
bc_one:
	bra.s	.b1			; one taken branch (and the return): a
	nop				; warm call retires both predicted, a cold
.b1:	rts				; one neither -- the call itself may be
					; learned by a decode run ahead (past a
					; serializing instruction) and counts at
					; most once
done23:

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99

; d1+1 levels of calls, each adds 1 to d0
deep:	addq.l	#1,d0
	subq.l	#1,d1
	bmi.s	.r
	bsr	deep
.r:	rts

; the same index, different tags (1 KB apart)
	cnop	0,1024
alias1:	bra.s	.a1
	nop
.a1:	addq.l	#1,d0
	rts
	cnop	0,1024
alias2:	bra.s	.a2
	nop
.a2:	addq.l	#2,d0
	rts
