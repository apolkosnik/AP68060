; AP68060 bus snooping (MC68060UM 2.6, 5.5, 5.6, 5.8)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench's alternate bus master (tb/m68060_alt_master.sv, registers at
; $F200) takes the bus from the 68060 and runs transfers with SNOOP
; asserted or negated, so every snoop case can be checked from software.
; The 68060 only invalidates (5.6: "Unlike the MC68040, the MC68060 cannot
; source or sink cache data during alternate bus master accesses"):
;
;   a snooped transfer (SNOOP asserted, normal access) that hits a line
;   invalidates it in both caches, read or write, byte, word, long or line
;   (5.5.3, 5.6); dirty data is lost, the other master sees memory; a
;   snooped write goes to memory; SNOOP negated is not snooped; a dirty
;   victim in the push buffer, or a push queued for the bus, is snooped
;   too (5.8) and never reaches memory; a snoop has priority over the
;   processor, so the push cannot slip out between the other master's
;   transfer and the invalidation.
;
; Cached data is reached through MOVES with SFC = DFC = 1 (user data), which
; DTT0 maps copyback for user accesses only.  DTT1 makes every supervisor
; data access noncachable, so the bench registers, the stack and the
; memory prefills never touch the data cache.  Memory itself is read back
; with SC = 00 transfers of the alternate master, which the cache ignores.

FAILREG	equ	$F100
DONEREG	equ	$F102
AMX	equ	$F200		; transfer k: AMX+32k: +0 addr, +4 ctl, +8 data
AMGO	equ	$F280
AMTRIG	equ	$F284
AMDLY	equ	$F288
AMST	equ	$F28C
DUMMY	equ	$3800		; a harmless word for the warm-up "go"
STUB	equ	$3FF0		; I-cache test routine (set $3F)

RD	equ	1
SZL	equ	0<<1		; long
SZB	equ	1<<1		; byte
SZW	equ	2<<1		; word
SZLN	equ	3<<1		; line
NSN	equ	0<<3		; SNOOP negated
SNP	equ	1<<3		; SNOOP asserted

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

chkm	macro			; memory (absolute), value, test
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

; transfer k: address, control
amx	macro
	move.l	#\2,(AMX+32*\1).l
	move.w	#\3,(AMX+32*\1+4).l
	endm

; transfer k: write data (one long word, as on D31-D0)
amd	macro
	move.l	#\2,(AMX+32*\1+8).l
	endm

; run n transfers and wait for them
amgo	macro
	move.w	#\1,d0
	bsr	amrun
	endm

; cached (copyback, user space) write and read
cwr	macro
	lea	(\1).l,a0
	move.l	#\2,d0
	moves.l	d0,(a0)
	endm
crd	macro
	lea	(\1).l,a0
	moves.l	(a0),d1
	endm

; noncachable memory write (supervisor space)
mwr	macro
	move.l	#\2,(\1).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0	; DTT1: all of $00xxxxxx, supervisor, noncachable
	movec	d0,dtt1
	move.l	#$00008020,d0	; DTT0: all of $00xxxxxx, user, copyback
	movec	d0,dtt0
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	move.l	#$80808000,d0
	movec	d0,cacr
	clr.l	(AMDLY).l

;---------------------------------- snooped read of a dirty line (5.5.3)
	mwr	$8000,$AAAA0000
	cwr	$8000,$11111111		; allocate, now dirty
	amx	0,$8000,RD|SZL|SNP
	amx	1,$8000,RD|SZL|NSN
	amgo	2
	chkm	AMX+8,$AAAA0000,1	; nothing sourced: memory's (stale) data
	chkm	AMX+32+8,$AAAA0000,2	; memory was not written
	crd	$8000			; the line went: memory's data comes back
	chkl	d1,$AAAA0000,3
	cpusha	dc			; nothing dirty is left to push
	amx	0,$8000,RD|SZL|NSN
	amgo	1
	chkm	AMX+8,$AAAA0000,4

;-------------------------- snooped line read of a dirty line: four beats
	mwr	$8010,$BBBB0000
	mwr	$8014,$BBBB0001
	mwr	$8018,$BBBB0002
	mwr	$801C,$BBBB0003
	cwr	$8010,$21000000
	cwr	$8014,$21000001
	cwr	$8018,$21000002
	cwr	$801C,$21000003
	amx	0,$8018,RD|SZLN|SNP	; starts at the third long word
	amx	1,$8010,RD|SZLN|NSN
	amgo	2
	chkm	AMX+8,$BBBB0000,5	; memory's line, A3:A2 wrapped
	chkm	AMX+12,$BBBB0001,6
	chkm	AMX+16,$BBBB0002,7
	chkm	AMX+20,$BBBB0003,8
	chkm	AMX+32+8,$BBBB0000,9
	crd	$801C			; all four dirty long words were lost
	chkl	d1,$BBBB0003,10

;---------------------- snooped byte read of a dirty line: invalidates too
	mwr	$8020,$CCCC0000
	cwr	$8020,$22222222
	amx	0,$8023,RD|SZB|SNP
	amgo	1
	chkm	AMX+8,$CCCC0000,11	; the long word as memory drives D31-D0
	crd	$8020			; gone: memory's (stale) data comes back
	chkl	d1,$CCCC0000,12

;---------------------------------- snooped read of a clean line: invalidate
	mwr	$8030,$DDDD0000
	crd	$8030
	chkl	d1,$DDDD0000,13
	amx	0,$8030,RD|SZL|SNP
	amx	1,$8030,SZL|NSN		; memory changes behind the cache
	amd	1,$33333333
	amgo	2
	chkm	AMX+8,$DDDD0000,14
	crd	$8030
	chkl	d1,$33333333,15		; the line was dropped by the read

;------------------------------------------- SNOOP negated is not snooped
	amx	0,$8030,SZL|NSN
	amd	0,$34343434
	amgo	1
	crd	$8030
	chkl	d1,$33333333,16		; stale: nothing told the cache
	lea	($8030).l,a0
	cinvl	dc,(a0)
	crd	$8030
	chkl	d1,$34343434,17

;------------------- snooped long write into a dirty line: no sink (5.6)
	mwr	$8040,$EEEE0000
	mwr	$8044,$EEEE0001
	cwr	$8040,$44444444
	cwr	$8044,$44444445
	amx	0,$8044,SZL|SNP
	amd	0,$55555555
	amx	1,$8044,RD|SZL|NSN
	amgo	2
	chkm	AMX+32+8,$55555555,18	; memory took the write
	crd	$8044
	chkl	d1,$55555555,19
	crd	$8040			; the line's other dirty long word is lost
	chkl	d1,$EEEE0000,20
	cpusha	dc			; nothing dirty: memory keeps both
	amx	0,$8040,RD|SZLN|NSN
	amgo	1
	chkm	AMX+8,$EEEE0000,21
	chkm	AMX+12,$55555555,22

;------------------------------ snooped byte and word writes: lanes, memory
	cwr	$8050,$66666666		; dirty (memory holds zero)
	amx	0,$8051,SZB|SNP
	amd	0,$00A50000		; byte 1: D23-D16
	amx	1,$8052,SZW|SNP
	amd	1,$00001234		; word 1: D15-D0
	amx	2,$8050,RD|SZL|NSN
	amgo	3
	crd	$8050			; the dirty line went with the byte write
	chkl	d1,$00A51234,23
	chkm	AMX+64+8,$00A51234,24	; memory has both writes, on their lanes

;---------------------------- snooped write to a clean line: invalidate
	mwr	$8060,$12340000
	crd	$8060
	chkl	d1,$12340000,25
	amx	0,$8060,SZL|SNP
	amd	0,$77777777
	amgo	1
	crd	$8060
	chkl	d1,$77777777,26

;---------------- snooped write to a dirty line: invalidate, the dirty data
;---------------- of the rest of the line is lost (5.6)
	mwr	$8070,$56780000
	mwr	$8074,$56780001
	cwr	$8070,$88888888
	cwr	$8074,$88888889
	amx	0,$8070,SZL|SNP
	amd	0,$99999999
	amgo	1
	crd	$8070
	chkl	d1,$99999999,27
	crd	$8074
	chkl	d1,$56780001,28

;-------------------------------- snooped line write to a dirty line
	cwr	$8080,$AB000000
	amx	0,$8080,SZLN|SNP
	move.l	#$C0000000,(AMX+8).l
	move.l	#$C0000001,(AMX+12).l
	move.l	#$C0000002,(AMX+16).l
	move.l	#$C0000003,(AMX+20).l
	amgo	1
	crd	$8080
	chkl	d1,$C0000000,29
	crd	$808C
	chkl	d1,$C0000003,30

;------------------------------------ a snooped write that misses: memory
	amx	0,$8090,SZL|SNP
	amd	0,$BEEF0001
	amgo	1
	crd	$8090
	chkl	d1,$BEEF0001,31

;------------------------------------------------------- instruction cache
; (5.6: both caches are snooped; a read hit invalidates as a write does)
	mwr	STUB,$70014E75		; moveq #1,d0 ; rts
	cpusha	bc			; required before self-modified code runs
	jsr	(STUB).l
	chkl	d0,1,32
	amx	0,STUB,SZL|NSN		; not snooped
	amd	0,$70024E75
	amgo	1
	jsr	(STUB).l
	chkl	d0,1,33			; the cached copy still runs
	amx	0,STUB,SZL|SNP		; a snooped write drops the line
	amd	0,$70034E75
	amgo	1
	jsr	(STUB).l
	chkl	d0,3,34
	amx	0,STUB,SZL|NSN
	amd	0,$70044E75
	amx	1,STUB,RD|SZL|SNP	; so does a snooped read
	amgo	2
	jsr	(STUB).l
	chkl	d0,4,35
	amx	0,STUB,SZL|NSN
	amd	0,$70054E75
	amgo	1
	jsr	(STUB).l
	chkl	d0,4,36			; refetched and cached again: stale
	amx	0,STUB,RD|SZB|SNP	; a snooped byte read drops it
	amgo	1
	jsr	(STUB).l
	chkl	d0,5,57

;---------------------- a push queued behind the alternate master (set $10)
; Four dirty lines fill the set ($800 apart: 128 sets); a fifth line's fill
; evicts one of them.  The bench takes the bus right after the fill, so the
; victim's push is waiting for the bus while the other master reads all
; four lines with SNOOP: every one is invalidated -- the queued push too
; (5.8) -- and their dirty data never reaches memory.
	cpusha	dc
	mwr	$A100,$5A5A0000
	mwr	$A900,$5A5A0001
	mwr	$B100,$5A5A0002
	mwr	$B900,$5A5A0003
	cwr	$A100,$D0000000
	cwr	$A900,$D0000001
	cwr	$B100,$D0000002
	cwr	$B900,$D0000003
	amx	0,$A100,RD|SZL|SNP
	amx	1,$A900,RD|SZL|SNP
	amx	2,$B100,RD|SZL|SNP
	amx	3,$B900,RD|SZL|SNP
	move.l	#$C100,(AMTRIG).l
	move.w	#$0104,(AMGO).l		; four transfers, after the fill of $C100
	crd	$C100
	bsr	amwait
	chkm	AMX+8,$5A5A0000,37
	chkm	AMX+32+8,$5A5A0001,38
	chkm	AMX+64+8,$5A5A0002,39
	chkm	AMX+96+8,$5A5A0003,40
	cpusha	dc			; nothing of the four is left to push
	amx	0,$A100,RD|SZL|NSN
	amx	1,$A900,RD|SZL|NSN
	amx	2,$B100,RD|SZL|NSN
	amx	3,$B900,RD|SZL|NSN
	amgo	4
	chkm	AMX+8,$5A5A0000,41
	chkm	AMX+32+8,$5A5A0001,42
	chkm	AMX+64+8,$5A5A0002,43
	chkm	AMX+96+8,$5A5A0003,44

;------------ the victim in the push buffer while its fill waits (set $10)
; The other master holds the bus before the 68060 misses: the victim sits
; in the push buffer while the fill waits for the bus, and is snooped
; there (5.8).  The instructions that run meanwhile come from the
; instruction cache (warmed first).
	mwr	$C900,$6B6B0000
	cwr	$A100,$E0000000
	cwr	$A900,$E0000001
	cwr	$B100,$E0000002
	cwr	$B900,$E0000003
	amx	0,$A100,RD|SZL|SNP
	amx	1,$A900,RD|SZL|SNP
	amx	2,$B100,RD|SZL|SNP
	amx	3,$B900,RD|SZL|SNP
	move.w	#200,(AMDLY).l
	lea	(DUMMY).l,a2		; warm-up: the routine into the I-cache
	lea	($A100).l,a0
	moveq	#4,d2
	bsr	pvgo
	lea	(AMGO).l,a2
	lea	($C900).l,a0
	bsr	pvgo
	bsr	amwait
	chkl	d1,$6B6B0000,45
	chkm	AMX+8,$5A5A0000,46
	chkm	AMX+32+8,$5A5A0001,47
	chkm	AMX+64+8,$5A5A0002,48
	chkm	AMX+96+8,$5A5A0003,49
	clr.w	(AMDLY).l
	cpusha	dc
	amx	0,$A100,RD|SZL|NSN
	amx	1,$A900,RD|SZL|NSN
	amx	2,$B100,RD|SZL|NSN
	amx	3,$B900,RD|SZL|NSN
	amgo	4
	chkm	AMX+8,$5A5A0000,50
	chkm	AMX+32+8,$5A5A0001,51
	chkm	AMX+64+8,$5A5A0002,52
	chkm	AMX+96+8,$5A5A0003,53

;------------------- a read from a serialized page waits for earlier writes
; (MC68060UM 7.10): the bench copies a write of $F294 into $F2A8 (another
; line: no address collision orders them), so the read sees the new value
; only if the write went out first.  DTT1 makes both supervisor accesses
; noncachable serialized.
; The pair runs from the instruction cache (a first call warms it), so
; nothing but the ordering rule keeps the read behind the write.
	lea	($F294).l,a3
	lea	($F2A8).l,a4
	move.l	#$600DF00D,d3
	bsr	serrw
	chkl	d0,$600DF00D,54
	move.l	#$0BADCAFE,d3
	bsr	serrw
	chkl	d0,$0BADCAFE,55

;------------- ... and is read once: nothing older may abort it afterwards
; (MC68060UM 7.10).  An interrupt taken at the end of a long DIVU.L would
; restart the next instruction; if its serialized read had already gone
; out, the device would be read twice.  $F2B0 counts its bus reads; the
; interrupt (bench $F148: IPL 2 after n clocks) is swept across the
; division.  The routine runs from the instruction cache.  (A property
; check: this core's DIVU.L holds the read back until its last uop, and the
; DMU additionally waits for EX and WB to drain, so both keep it to one.)
	lea	($F2B0).l,a5
	move.l	#lev2,(26*4).w
	move.l	#$7FFFFFFF,d1
	moveq	#3,d2
	bsr	divrd			; warm-up, no interrupt
	move.w	#$2000,sr		; interrupts on
	moveq	#1,d4			; n: clocks to the interrupt
.sweep:
	clr.l	($F2B4).l
	moveq	#0,d5
	move.w	d4,($F148).l
	move.l	#$7FFFFFFF,d1
	bsr	divrd
.wait:	tst.w	d5			; the interrupt has been taken
	beq.s	.wait
	cmp.l	#1,($F2B4).l
	beq.s	.once
	move.w	#$2700,sr
	failt	56
.once:
	addq.w	#1,d4
	cmp.w	#80,d4
	bls.s	.sweep
	move.w	#$2700,sr

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99

; start the transfers (d0 = command word) and wait for them
amrun:
	move.w	d0,(AMGO).l
amwait:
	move.w	(AMST).l,d0
	beq.s	amwait
	cmp.w	#1,d0
	beq.s	.ok
	failt	98
.ok:	rts

; the "go" write to (a2), then a cached read at (a0) into d1.  A later
; read may pass an earlier write on the 68040 (MC68060UM 7.10): the NOP
; puts the write on the bus first.
	cnop	0,16
pvgo:
	move.w	d2,(a2)
	nop
	moves.l	(a0),d1
	rts

; a write to (a3) of d3, then a read of (a4) into d0
	cnop	0,16
serrw:
	move.l	d3,(a3)
	move.l	(a4),d0
	rts

; a division by d2, then a read of (a5): serialized (supervisor, DTT1)
	cnop	0,16
divrd:
	divu.l	d2,d1
	move.l	(a5),d0
	rts

lev2:
	clr.w	($F110).l		; withdraw IPL
	moveq	#1,d5
	rte
