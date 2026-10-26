; AP68060: FMOVECR's <ea> field (operation word bits 5-0).  With the field 0
; FMOVECR is an unimplemented floating-point instruction (vector 11, format
; $2, the PC of the next instruction, FPIAR the instruction).  Any other
; field takes F-line (vector 11, format $0, the PC of the instruction,
; FPIAR untouched), with the FPU disabled too: the M68060SP's _fpsp_fline
; ("fmovecr instructions w/ a non-zero <ea> field ... may get flagged as
; Line F") converts that frame into the unimplemented instruction's, or
; into the FPU disabled one's when PCR.DFP is set.  GitHub issue 4.
; assembled with vasmm68k_mot -Fbin -m68040
;
;  the FPU enabled
;   1-2   F200 5C00 (FMOVECR #0,FP0): format/vector $202C, the next PC
;   3     ... FPIAR the instruction
;   4-5   F201 5C00 (<ea> field 1): $002C, the instruction's PC
;   6-7   F208 5C00 (<ea> field 8, A0): $002C, the instruction's PC
;   8-9   F23F 5C00 (<ea> field $3F): $002C, the instruction's PC
;   10    FPIAR untouched by the F-line cases
;  PCR.DFP set (the FPU disabled)
;   11-12 F200 5C00: $402C, the next PC
;   13-14 F201 5C00: $002C, the instruction's PC
;   15-16 F210 5C00 (<ea> field $10, (A0)): $002C, the instruction's PC
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102

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

; one case: operation word, format/vector, at the instruction (1) or past
; it (0), test; the instruction's address is left in a2
fcase	macro
	lea	.r\@(pc),a1		; where the handler resumes
	lea	.i\@(pc),a2		; the instruction
	moveq	#-1,d1
	moveq	#-1,d2
.i\@:	dc.w	\1,$5C00
	nop
.r\@:	chkl	d1,\2,\4
	lea	4(a2),a3
	if \3
	move.l	a2,a3
	endif
	cmp.l	a3,d2
	beq.s	.p\@
	failt	\4+1
.p\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	9
	dc.l	unexp
	endr
	dc.l	h_fline			; 11
	rept	244
	dc.l	unexp
	endr

	org	$400
start:
	lea	$8000,a0		; (the <ea> registers point at RAM)
	moveq	#0,d0
	fmove.l	d0,fpcr
	fmove.l	d0,fpsr
	fcase	$F200,$0000202C,0,1
	fmove.l	fpiar,d0
	cmp.l	a2,d0
	beq.s	.ia
	failt	3
.ia:	move.l	#$12345678,d0
	fmove.l	d0,fpiar
	fcase	$F201,$0000002C,1,4
	fcase	$F208,$0000002C,1,6
	fcase	$F23F,$0000002C,1,8
	fmove.l	fpiar,d0
	chkl	d0,$12345678,10

	; the FPU disabled
	dc.w	$4e7a,$0808		; movec pcr,d0
	move.l	d0,d6
	bset	#1,d0			; DFP
	dc.w	$4e7b,$0808		; movec d0,pcr
	fcase	$F200,$0000402C,0,11
	fcase	$F201,$0000002C,1,13
	fcase	$F210,$0000002C,1,15
	move.l	d6,d0
	dc.w	$4e7b,$0808		; movec d0,pcr
	move.w	#$600d,(DONEREG).l
.d:	bra.s	.d

; vector 11: the format/vector word into d1, the stacked PC into d2, resume
; at a1 (RTE pops the format $0, $2 or $4 frame)
h_fline:
	moveq	#0,d1
	move.w	6(sp),d1
	move.l	2(sp),d2
	move.l	a1,2(sp)
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.f:	bra.s	.f

unexp:
	move.w	#98,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.u:	bra.s	.u
