; AP68060: a byte load's condition codes at the branch that follows it, in
; a structure-initialisation interpreter of the shape of exec's InitStruct
; (Kickstart 3.2's, found under Minimig: its BEQ on a loaded end marker of
; zero fell through, the interpreter ran past its table and copied code into
; expansion.library's base, Alert $0A000001).
;   clr.w   d0
;   move.b  (a1)+,d0        ; the command byte, 0 ends the table
;   beq.b   done
;   bpl.b   ...             ; computed dispatch, DBRA copy loops, word
;                           ; alignment of the table pointer
; The table is read with the data cache inhibited (DTT0, as Kickstart sets
; it up), the code runs from the instruction cache; 64 passes, each from a
; table and a destination at a different offset.  Each pass checks every
; field written, that nothing after the structure was touched, and that
; the interpreter stopped at the end marker (a1 just past it).
; assembled with vasmm68k_mot -Fbin -m68040

DONEREG	equ	$f102
TESTREG	equ	$f100
DEST	equ	$5000		; the structure, 64 bytes, then a guard of 64
TABS	equ	$6000		; the table, copied to a new offset each pass

ck	macro			; \1 operand, \2 expected (with #), \3 test
	cmp.l	\2,\1
	beq.s	ok\@
	move.w	#\3,(TESTREG).l
	bra.w	fail
ok\@:
	endm

	org	0
	dc.l	$3ff0,start
	rept	254
	dc.l	fail
	endr

	org	$400
start:
	move.w	#$2700,sr
	move.l	#$0000c040,d0	; DTT0: everything, cache inhibited (precise)
	movec	d0,dtt0
	move.l	#$00008000,d0	; CACR: the instruction cache only
	movec	d0,cacr
	moveq	#0,d7		; the pass
pass:
	; the table at TABS + 2*pass (word aligned, as exec's are)
	lea	(TABS).l,a1
	move.l	d7,d0
	add.l	d0,d0
	adda.l	d0,a1
	movea.l	a1,a3		; (its start)
	lea	table(pc),a0
	move.w	#tabend-table-1,d0
cpy:	move.b	(a0)+,(a1)+
	dbra	d0,cpy
	; the destination: 128 bytes of $A5, at DEST + 4*pass
	lea	(DEST).l,a2
	move.l	d7,d0
	lsl.l	#2,d0
	adda.l	d0,a2
	movea.l	a2,a0
	moveq	#31,d0
fil:	move.l	#$a5a5a5a5,(a0)+
	dbra	d0,fil
	movea.l	a3,a1
	moveq	#64,d0		; the size cleared first
	bsr.w	istruct
	; stopped at the end marker: a1 one past it
	movea.l	a3,a0
	adda.w	#tabzero-table+1,a0
	ck	a1,a0,1
	; the fields
	move.w	8(a2),d0
	ck	d0,#$09ec,2
	move.w	$a(a2),d0
	swap	d0
	move.w	$c(a2),d0
	ck	d0,#$00f84170,3
	move.b	$e(a2),d0
	and.l	#$ff,d0
	ck	d0,#$06,4
	move.w	$14(a2),d0
	and.l	#$ffff,d0
	ck	d0,#$002f,5
	move.w	$16(a2),d0
	and.l	#$ffff,d0
	ck	d0,#$0004,6
	move.l	$18(a2),d0
	ck	d0,#$00f8418e,7
	; the rest of the structure cleared, the guard after it untouched
	move.l	(a2),d0
	ck	d0,#0,8
	move.l	$1c(a2),d0
	ck	d0,#0,9
	move.l	$3c(a2),d0
	ck	d0,#0,10
	move.l	$40(a2),d0
	ck	d0,#$a5a5a5a5,11
	move.l	$78(a2),d0
	ck	d0,#$a5a5a5a5,12
	addq.l	#1,d7
	cmp.l	#64,d7
	bne.w	pass
	move.w	#$600D,(DONEREG).l
done:	bra.s	done
fail:	move.w	#$BAD0,(DONEREG).l
halt:	bra.s	halt

; istruct: a1 the table, a2 the structure, d0 its size (cleared first).
; A command byte: bits 7-6 how the destination moves (00 on, 01 the same
; value repeated, 10 a byte offset follows, 11 a 24-bit offset follows),
; bits 5-4 the size (long, word, byte), bits 3-0 the count less one.  Word
; and long data start on an even byte of the table.
istruct:
	movea.l	a2,a0
	lsr.w	#1,d0
	bra.s	clr2
clr1:	clr.w	(a0)+
clr2:	dbra	d0,clr1
	movea.l	a2,a0
next:	clr.w	d0
	move.b	(a1)+,d0	; the command
	beq.b	idone		; 0: the end
	bpl.b	disp		; 00 / 01: no offset
	bclr	#6,d0
	beq.b	off8
	subq.l	#1,a1		; 11: a 24-bit offset in the command's long
	move.l	(a1)+,d1
	and.l	#$ffffff,d1
	bra.b	offs
idone:	rts			; (forward of the BEQ, as in exec)
off8:	moveq	#0,d1		; 10: a byte offset
	move.b	(a1)+,d1
offs:	movea.l	a2,a0
	adda.l	d1,a0
disp:	move.w	d0,d1
	lsr.w	#3,d1
	and.w	#$e,d1
	move.w	itab(pc,d1.w),d1
	and.w	#$f,d0
	jmp	itab(pc,d1.w)
itab:	dc.w	c_long-itab,c_word-itab,c_byte-itab,c_bad-itab
	dc.w	r_long-itab,r_word-itab,r_byte-itab,c_bad-itab
c_long:	bsr.s	align
cl1:	move.l	(a1)+,d1
	move.l	d1,(a0)+
	dbra	d0,cl1
	bra.w	next
c_word:	bsr.s	align
cw1:	move.w	(a1)+,(a0)+
	dbra	d0,cw1
	bra.w	next
c_byte:
cb1:	move.b	(a1)+,(a0)+
	dbra	d0,cb1
	bsr.s	align
	bra.w	next
r_long:	bsr.s	align
	move.l	(a1)+,d1
rl1:	move.l	d1,(a0)+
	dbra	d0,rl1
	bra.w	next
r_word:	bsr.s	align
	move.w	(a1)+,d1
rw1:	move.w	d1,(a0)+
	dbra	d0,rw1
	bra.w	next
r_byte:	move.b	(a1)+,d1
rb1:	move.b	d1,(a0)+
	dbra	d0,rb1
	bsr.s	align
	bra.w	next
c_bad:	move.w	#99,(TESTREG).l
	bra.w	fail
align:	move.l	a1,d1		; the table pointer to an even byte
	addq.l	#1,d1
	and.b	#$fe,d1
	movea.l	d1,a1
	rts

; the structure's table: fields at 8 (2 bytes), $A (long), $E (byte),
; $14 (word), $16 (word), $18 (long); then the end; then words that are not
; commands (what would be copied if the end were missed)
	even
table:	dc.b	$a1,$08,$09,$ec		; bytes at 8: $09,$ec
	dc.b	$80,$0a,$00,$f8,$41,$70	; long at $A
	dc.b	$a0,$0e,$06,$00		; byte at $E
	dc.b	$90,$14,$00,$2f		; word at $14
	dc.b	$90,$16,$00,$04		; word at $16
	dc.b	$80,$18,$00,$f8,$41,$8e	; long at $18
tabzero:
	dc.b	$00			; the end
	dc.b	$00
	dc.w	$ffff,$003a,$0042,$0046,$0046,$0676,$0950,$0610
tabend:
