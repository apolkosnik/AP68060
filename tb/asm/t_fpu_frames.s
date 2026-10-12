; MC68060 FSAVE/FRESTORE: the three-long-word state frame (MC68060UM 6.7)
; on the wire, its addressing, and the NULL/IDLE/EXCP lifecycle.
;   word 0: the exception operand's exponent (bits 31-16), the frame
;           format (15-8: $00 NULL, $60 IDLE, $E0 EXCP), the vector code
;           (2-0: 0 BSUN, 1 INEX, 2 DZ, 3 UNFL, 4 OPERR, 5 OVFL, 6 SNAN,
;           7 UNSUP)
;   words 1, 2: the exception operand's mantissa
; An EXCP state is taken by the next FPU instruction (but FSAVE and
; FRESTORE) as a pre-instruction exception of its vector; FSAVE saves and
; leaves it; FRESTORE of an EXCP frame re-enters it, whatever the FPCR
; enables (WinUAE fpuop_restore: fp_exp_pend = 48 + v, 7: vector 55).

FRAME   equ     $3000
COPY    equ     $3100
COUNT   equ     $3800           ; per vector 48-55: COUNT+4*(v-48)
CNTFMT  equ     $3840           ; format errors
CNTPRV  equ     $3844           ; privilege violations
CNTFL   equ     $3848           ; F-line
LASTPC  equ     $384C           ; the arithmetic handlers' stacked PC
LASTFW  equ     $3850           ; ... and format/offset word
HSAVE   equ     $3860           ; the arithmetic handlers' FSAVE frame
SKIP    equ     $3870           ; F-line/format-error handler: bytes to skip
LASTEA  equ     $3854           ; the arithmetic handlers' address field (+8)
DFPCNT  equ     $3880           ; FPU-disabled exceptions (vector 11, format $4)
DFPPC   equ     $3884           ; ... their stacked PC, address, instruction PC
DFPEA   equ     $3888
DFPIP   equ     $388C

check   macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,($f100).l
        jmp     fail
ok\@:
        endm
; three frame words at \1 against \2 \3 \4
frame3  macro
        move.l  \1,d0
        check   d0,\2,\5
        move.l  4+\1,d0
        check   d0,\3,\5+1
        move.l  8+\1,d0
        check   d0,\4,\5+2
        endm
; a frame at FRAME
setfr   macro
        move.l  #\1,(FRAME).l
        move.l  #\2,(FRAME+4).l
        move.l  #\3,(FRAME+8).l
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        move.l  #priv_handler,($20).w
        move.l  #fline_handler,($2c).w
        move.l  #format_handler,($38).w
        lea     ($c0).w,a0
        lea     arith_handlers(pc),a1
        moveq   #7,d1
inst_v:
        move.l  a1,(a0)+
        lea     arith_h1-arith_h0(a1),a1
        dbra    d1,inst_v
        lea     (COUNT).l,a0
        moveq   #31,d1
clr_cnt:
        clr.l   (a0)+
        dbra    d1,clr_cnt

;------------------------------------------------ NULL after reset
        lea     (FRAME).l,a1
        move.l  #$a55a1234,(FRAME-4).l
        move.l  #$5aa54321,(FRAME+12).l
        move.l  #-1,(FRAME).l
        move.l  #-1,(FRAME+4).l
        move.l  #-1,(FRAME+8).l
        fsave   (a1)
        frame3  (FRAME).l,0,0,0,1      ; NULL, the operand zero
        move.l  (FRAME-4).l,d0
        check   d0,$a55a1234,4         ; exactly three long words
        move.l  (FRAME+12).l,d0
        check   d0,$5aa54321,5
        frestore (a1)+
        check   a1,FRAME+12,6

;------------------------------------------------ the data movements leave NULL too
        ; any implemented FPU instruction leaves the NULL state (6.7;
        ; WinUAE fpuop_arithmetic): a control register read, an FMOVEM
        ; store.  FNOP does not (WinUAE keeps the conditionals NULL).
        fmove.l fpsr,d0
        fsave   (COPY).l
        move.l  (COPY).l,d0
        check   d0,$00006000,440       ; IDLE
        clr.l   (COPY).l
        frestore (COPY).l              ; NULL again
        fmovem.x fp0,(COPY+$10).l
        fsave   (COPY).l
        move.l  (COPY).l,d0
        check   d0,$00006000,441       ; IDLE
        clr.l   (COPY).l
        frestore (COPY).l
        fnop
        fsave   (COPY).l
        move.l  (COPY).l,d0
        check   d0,0,442               ; still NULL
        clr.l   (COPY).l
        frestore (COPY).l

;------------------------------------------------ IDLE once the FPU is used
        fmove.l #7,fp0
        fsave   -(a1)
        check   a1,FRAME,7
        frame3  (FRAME).l,$00006000,0,0,8
        frestore (a1)+
        check   a1,FRAME+12,11
        ; the other control-alterable addressing modes
        lea     (FRAME+$20).l,a2
        moveq   #8,d2
        fsave   ($10,a2)               ; FRAME+$30
        move.l  (FRAME+$30).l,d0
        check   d0,$00006000,12
        fsave   (-$10,a2,d2.l*2)       ; FRAME+$20
        move.l  (FRAME+$20).l,d0
        check   d0,$00006000,13
        fsave   ($3040).w
        move.l  ($3040).l,d0
        check   d0,$00006000,14
        frestore ($10,a2)
        frestore ($3040).w

;------------------------------------------------ unimplemented: IDLE
        ; an unimplemented instruction leaves the IDLE state, its operand
        ; zero; the handler's FSAVE sees exactly that
        setfr   $12345678,$12345678,$12345678
        fsin.x  fp0
        move.l  (CNTFL).l,d0
        check   d0,1,15
        frame3  (FRAME).l,$00006000,0,0,16

;------------------------------------------------ EXCP round trips
        ; every vector code survives FRESTORE + FSAVE, the operand too,
        ; and neither instruction takes the exception
        moveq   #0,d5
excp_rt:
        move.l  d5,d0
        or.l    #$1234e000,d0
        move.l  d0,(FRAME).l
        move.l  #$89abcdef,(FRAME+4).l
        move.l  #$01234567,(FRAME+8).l
        lea     (FRAME).l,a1
        frestore (a1)+
        check   a1,FRAME+12,20
        lea     (COPY+12).l,a2
        fsave   -(a2)
        check   a2,COPY,21
        move.l  (COPY).l,d0
        move.l  d5,d1
        or.l    #$1234e000,d1
        cmp.l   d1,d0
        beq.s   excp_rt1
        move.w  #22,($f100).l
        jmp     fail
excp_rt1:
        move.l  (COPY+4).l,d0
        check   d0,$89abcdef,23
        move.l  (COPY+8).l,d0
        check   d0,$01234567,24
        ; FSAVE left the state: a second FSAVE is IDLE, its operand kept
        fsave   (COPY).l
        move.l  (COPY).l,d0
        check   d0,$12346000,25
        addq.l  #1,d5
        cmp.l   #8,d5
        bne     excp_rt
        lea     (COUNT).l,a0           ; nothing was taken
        moveq   #7,d1
excp_rt2:
        tst.l   (a0)+
        bne     fail
        dbra    d1,excp_rt2

;------------------------------------------------ EXCP: the next instruction takes it
        ; restored OPERR: the next FPU instruction takes vector 52 before it
        ; runs (format $0, its PC); the handler's FSAVE sees the frame as it
        ; was restored (the operand retains its value, MC68060UM 6.7)
        setfr   $4001e004,$c0000000,$00000001
        frestore (FRAME).l
        moveq   #3,d1                  ; integer code: nothing taken
        move.l  (COUNT+16).l,d0
        check   d0,0,30
xop1:
        fmove.l #11,fp1
        move.l  (COUNT+16).l,d0
        check   d0,1,31
        move.l  (LASTPC).l,d0
        check   d0,xop1,32
        move.l  (LASTFW).l,d0
        check   d0,$000000d0,33        ; format $0, vector 52
        frame3  (HSAVE).l,$4001e004,$c0000000,$00000001,34
        fmove.l fp1,d0
        check   d0,11,37               ; the instruction ran after the RTE

        ; with every enable clear the restored state is taken all the same
        fmove.l #0,fpcr
        setfr   $0000e002,0,0
        frestore (FRAME).l
        fnop
        move.l  (COUNT+8).l,d0
        check   d0,1,38                ; DZ
        setfr   $0000e000,0,0
        frestore (FRAME).l
        fnop
        move.l  (COUNT+0).l,d0
        check   d0,1,39                ; BSUN
        setfr   $0000e007,0,0
        frestore (FRAME).l
        lea     u55_c(pc),a0
        move.l  a0,(SKIP).l
        fnop
u55_c:
        move.l  (COUNT+28).l,d0
        check   d0,1,40                ; vector code 7: vector 55
        move.l  (LASTFW).l,d0
        check   d0,$000000dc,41

        ; the M68060SP's way back to normal operation: bit 15 of the frame
        ; format cleared, $E0 becomes $60, an IDLE frame
        setfr   $4001e004,$c0000000,$00000001
        move.b  #$60,(FRAME+2).l
        frestore (FRAME).l
        fnop
        move.l  (COUNT+16).l,d0
        check   d0,1,42                ; nothing taken

        ; an EXCP state left by a real exception, saved before it is taken
        fmove.l #$4000,fpcr            ; enable SNAN
        fmove.l #5,fp2
        move.l  #$7f800001,($3300).l
        fmove.s ($3300).l,fp2          ; the exception is pending
        fsave   (FRAME).l              ; saves it, takes nothing
        move.l  (COUNT+24).l,d0
        check   d0,0,43
        move.l  (FRAME).l,d0
        and.l   #$0000ffff,d0
        check   d0,$0000e006,44
        fnop                           ; the state was left
        move.l  (COUNT+24).l,d0
        check   d0,0,45
        frestore (FRAME).l             ; and is back
        fnop
        move.l  (COUNT+24).l,d0
        check   d0,1,46
        fmove.l fp2,d0
        check   d0,5,47                ; an enabled SNAN wrote nothing
        fmove.l #0,fpcr

;------------------------------------------------ format errors
        ; any frame format but $00, $60, $E0 is a format error (vector 14)
        ; at the FRESTORE: nothing restored, (An)+ not applied
        fmove.l #7,fp0                 ; IDLE
        move.l  #6,(SKIP).l            ; frestore (abs).l
        setfr   $00004100,0,0
        frestore (FRAME).l
        setfr   $41000000,0,0          ; (a 68040 IDLE frame: format $00 --
        frestore (FRAME).l             ; the frame's upper word is the
        fsave   (COPY).l               ; operand's exponent: NULL here)
        move.l  (COPY).l,d0
        check   d0,$41000000,50
        fmove.l #7,fp0
        setfr   $0000e100,0,0
        frestore (FRAME).l
        setfr   $00006100,0,0
        frestore (FRAME).l
        setfr   $00000100,0,0
        frestore (FRAME).l
        setfr   $0000ff00,0,0
        frestore (FRAME).l
        move.l  (CNTFMT).l,d0
        check   d0,5,51
        fsave   (COPY).l               ; the state is still IDLE
        move.l  (COPY).l,d0
        and.l   #$0000ff00,d0
        check   d0,$00006000,52
        move.l  #2,(SKIP).l            ; frestore (a1)+
        lea     (FRAME).l,a1
        frestore (a1)+
        check   a1,FRAME,53
        move.l  (CNTFMT).l,d0
        check   d0,6,54

;------------------------------------------------ NULL frames reset the FPU
        fmove.l #$00000030,fpcr
        fmove.l #$0f000000,fpsr
        fmove.l #$12345678,fpiar
        clr.l   (FRAME).l
        clr.l   (FRAME+4).l
        clr.l   (FRAME+8).l
        frestore (FRAME).l
        fmove.l fpcr,d0
        check   d0,0,60
        fmove.l fpsr,d0
        check   d0,0,61
        fmove.l fpiar,d0
        check   d0,0,62
        fmove.x fp0,(COPY).l           ; FP0 the default NaN again
        frame3  (COPY).l,$7fff0000,$ffffffff,$ffffffff,63
        fsave   (COPY).l               ; the FMOVE out made it IDLE
        move.l  (COPY).l,d0
        and.l   #$0000ff00,d0
        check   d0,$00006000,66
        clr.l   (FRAME).l
        frestore (FRAME).l
        fsave   (COPY).l
        move.l  (COPY).l,d0
        and.l   #$0000ff00,d0
        check   d0,0,67                ; NULL

;------------------------------------------------ the addressing FSAVE/FRESTORE reject
        ; FSAVE (An)+ and FRESTORE -(An) are F-line exceptions
        move.l  #2,(SKIP).l
        lea     (FRAME).l,a1
        dc.w    $f319                  ; fsave (a1)+
        check   a1,FRAME,70
        dc.w    $f361                  ; frestore -(a1)
        check   a1,FRAME,71
        move.l  (CNTFL).l,d0
        check   d0,3,72

;------------------------------------------------ privileged
        ; FSAVE and FRESTORE are privileged: vector 8 in user mode
        move.l  #2,(SKIP).l
        lea     ($d000).l,a0
        move.l  a0,usp
        andi.w  #$dfff,sr              ; user mode
        lea     (FRAME).l,a1
        fsave   (a1)
        frestore (a1)
        move.w  #$2700,sr              ; the third privilege violation:
                                       ; the handler returns to supervisor
priv_back:
        move.l  (CNTPRV).l,d0
        check   d0,3,73

;------------------------------------------------ data type frames (MC68060UM 6.5.2)
        ; a denormal SINGLE / DOUBLE source: vector 55, format $0, the next
        ; PC; the frame's operand in the 060's "skewed" extended form the
        ; M68060SP's fix_skewed_ops takes: exponent $3F80 (single) / $3C00
        ; (double), the fraction under a clear integer bit
        move.l  #$00000001,($3400).l   ; the smallest single denormal
        lea     dts_n(pc),a0
        move.l  a0,(SKIP).l
dts_op: fadd.s  ($3400).l,fp0
dts_n:  move.l  (LASTPC).l,d0
        check   d0,dts_n,400           ; the next PC
        move.l  (LASTFW).l,d0
        check   d0,$00dc,401           ; format $0, vector 55
        frame3  (HSAVE).l,$3f80e007,$00000100,$00000000,402
        clr.l   ($3410).l              ; the smallest double denormal
        move.l  #$00000001,($3414).l
        lea     dtd_n(pc),a0
        move.l  a0,(SKIP).l
        fadd.d  ($3410).l,fp0
dtd_n:  frame3  (HSAVE).l,$3c00e007,$00000000,$00000800,405
        ; an (An)+ source: the increment applied (the M68060SP emulates the
        ; instruction from the frame and returns past it, An untouched)
        lea     ($3400).l,a1
        lea     dtp_n(pc),a0
        move.l  a0,(SKIP).l
        fadd.s  (a1)+,fp0
dtp_n:  check   a1,$3404,408

        ; FMOVE OUT of a denormal register: vector 55, format $3, the next
        ; PC, nothing stored; for B/W/L/S/D the (An)+ / -(An) update stands
        ; and the address field is the operand's; for X, -(An) is not
        ; applied and the address field is An - 4 (fpsp.S _calc_ea_fout)
        move.l  #$00000000,($3420).l   ; a true extended denormal
        move.l  #$00001000,($3424).l
        move.l  #$00000000,($3428).l
        fmovem.x ($3420).l,fp1
        move.l  #$eeeeeeee,($3430).l
        lea     ($3430).l,a1
        lea     dfo_n(pc),a0
        move.l  a0,(SKIP).l
        fmove.s fp1,(a1)+
dfo_n:  move.l  (LASTFW).l,d0
        check   d0,$30dc,409           ; format $3, vector 55
        move.l  (LASTEA).l,d0
        check   d0,$3430,410
        check   a1,$3434,411           ; (An)+ applied
        move.l  ($3430).l,d0
        check   d0,$eeeeeeee,412       ; nothing stored
        lea     ($3440).l,a1
        lea     dfx_n(pc),a0
        move.l  a0,(SKIP).l
        fmove.x fp1,-(a1)
dfx_n:  move.l  (LASTEA).l,d0
        check   d0,$343c,413           ; $3440 - 12 + 8
        check   a1,$3440,414           ; -(An) not applied
        move.l  (LASTPC).l,d0
        check   d0,dfx_n,415

;------------------------------------------------ the FPU disabled (PCR.DFP)
        ; every FPU instruction: vector 11, format $4, the stacked PC the
        ; next instruction, +8 the memory operand's address (0 for a
        ; register; -(An) the first operand byte, An - 4 for extended), +$C
        ; the instruction; the address register not modified (8.2.4)
        dc.w    $4e7a,$0808            ; movec pcr,d0
        move.l  d0,d7
        bset    #1,d0                  ; DFP
        dc.w    $4e7b,$0808            ; movec d0,pcr
        clr.l   (DFPCNT).l
dfp1:   fadd.x  fp1,fp0                ; a register operand
dfp1n:  move.l  (DFPPC).l,d0
        check   d0,dfp1n,420
        move.l  (DFPEA).l,d0
        check   d0,0,421
        move.l  (DFPIP).l,d0
        check   d0,dfp1,422
        lea     ($3500).l,a1
dfp2:   fmove.x (a1)+,fp0
dfp2n:  move.l  (DFPEA).l,d0
        check   d0,$3500,423
        check   a1,$3500,424           ; (An)+ not applied
        move.l  (DFPPC).l,d0
        check   d0,dfp2n,425
        lea     ($350c).l,a1
        fmove.x -(a1),fp0
        move.l  (DFPEA).l,d0
        check   d0,$3508,426           ; An - 4 for extended
        check   a1,$350c,427
        lea     ($3510).l,a1
        fmove.s -(a1),fp0
        move.l  (DFPEA).l,d0
        check   d0,$350c,428           ; the first operand byte
        fmove.l fp0,($3520).l          ; an FPU-to-memory move
        move.l  (DFPEA).l,d0
        check   d0,$3520,429
dfp5:   fnop
        move.l  (DFPIP).l,d0
        check   d0,dfp5,430
        move.l  (DFPCNT).l,d0
        check   d0,6,431
        dc.w    $4e7b,$7808            ; movec d7,pcr: the FPU back on
        fnop
        move.l  (DFPCNT).l,d0
        check   d0,6,432

;------------------------------------------- FMOVE OUT overflow: EXC.INEX2
; OVFL (vector 53, taken by the 060 even when disabled) sets EXC.INEX2 only
; when significand bits are lost; accrued INEX comes with the overflow
; either way (MC68060UM 6.6.4, 6.6.7; WinUAE SoftFloat roundAndPackFloat32/64)
        clr.l   (COUNT+4*5).l
        fmove.l #0,fpsr
        fmove.x ovx_s,fp0              ; exactly 2^128
        fmove.s fp0,d0
        fmove.l fpsr,d1
        move.l  d1,d2
        and.l   #$ff00,d1
        check   d1,$1000,450           ; OVFL, not INEX2
        and.l   #$f8,d2
        check   d2,$48,451             ; accrued OVFL and INEX
        fmove.l #0,fpsr
        fmove.x ovx_d,fp0              ; exactly 2^1024
        fmove.d fp0,($3520).l
        fmove.l fpsr,d1
        and.l   #$ff00,d1
        check   d1,$1000,452
        fmove.l #0,fpsr
        fmove.x ovx_si,fp0             ; 2^128 (1 + 2^-63): bits lost
        fmove.s fp0,d0
        fmove.l fpsr,d1
        and.l   #$ff00,d1
        check   d1,$1200,453           ; OVFL and INEX2
        move.l  (COUNT+4*5).l,d0
        check   d0,3,454               ; three overflow exceptions
        fmove.l #0,fpsr
;------------------------------------- underflow decided before rounding
; a tiny intermediate result that rounds up to the smallest normal number
; still underflows (MC68060UM 6.4.1, 6.6.5; SoftFloat 68K): UNFL and INEX2,
; vector 51 (taken by the 060 whatever the enables) at the next FPU
; instruction, the destination register unchanged
        clr.l   (COUNT+4*3).l
        fmove.x unf_small_s,fp0        ; 2^-152
        fmove.x unf_min_s,fp1          ; -2^-126
        fmove.l #0,fpsr
        fsadd.x fp1,fp0                ; single: -2^-126 + -0.. rounds to -2^-126
        fmove.l fpsr,d0
        move.l  (COUNT+4*3).l,d1
        check   d1,1,460               ; vector 51
        and.l   #$0a00,d0
        check   d0,$0a00,461           ; UNFL and INEX2
        fmovem.x fp0,(COPY).l
        move.l  (COPY).l,d0
        check   d0,$3f670000,462       ; FP0 unchanged
        clr.l   (COUNT+4*3).l
        fmove.x unf_small_d,fp0        ; 2^-1077
        fmove.x unf_min_d,fp1          ; -2^-1022
        fmove.l #0,fpsr
        fdadd.x fp1,fp0
        fmove.l fpsr,d0
        move.l  (COUNT+4*3).l,d1
        check   d1,1,463
        and.l   #$0a00,d0
        check   d0,$0a00,464
        fmovem.x fp0,(COPY).l
        move.l  (COPY).l,d0
        check   d0,$3bca0000,465
        fmove.l #0,fpsr

        check   sp,$e000,99
        move.w  #$600d,($f102).l
        bra.s   *

;------------------------------------------------ handlers
fline_handler:
        cmpi.w  #$402c,6(sp)           ; the FPU disabled: record, go on
        bne.s   fl_not_dfp
        addq.l  #1,(DFPCNT).l
        move.l  2(sp),(DFPPC).l
        move.l  8(sp),(DFPEA).l
        move.l  12(sp),(DFPIP).l
        rte                            ; the stacked PC is the next one
fl_not_dfp:
        addq.l  #1,(CNTFL).l
        cmpi.w  #$202c,6(sp)           ; an unimplemented instruction
        beq.s   fl_unimp
        cmpi.w  #$002c,6(sp)
        bne     fail
        move.l  (SKIP).l,d0
        add.l   d0,2(sp)
        rte
fl_unimp:
        fsave   (FRAME).l
        rte
format_handler:
        cmpi.w  #$0038,6(sp)
        bne     fail
        addq.l  #1,(CNTFMT).l
        move.l  (SKIP).l,d0
        add.l   d0,2(sp)
        rte
priv_handler:
        cmpi.w  #$0020,6(sp)
        bne     fail
        addq.l  #1,(CNTPRV).l
        cmpi.l  #3,(CNTPRV).l
        beq.s   priv_ret
        addq.l  #2,2(sp)
        rte
priv_ret:
        move.l  #priv_back,2(sp)       ; the third: back in
        ori.w   #$2000,(sp)            ; supervisor mode
        rte

; vectors 48-55: count, record, FSAVE (leaving the EXCP state), return to
; the instruction (or, for vector 55, to SKIP's address)
arith_handlers:
arith_h0:
        move.w  #48,d0
        bra.s   arith_common
arith_h1:
        move.w  #49,d0
        bra.s   arith_common
        move.w  #50,d0
        bra.s   arith_common
        move.w  #51,d0
        bra.s   arith_common
        move.w  #52,d0
        bra.s   arith_common
        move.w  #53,d0
        bra.s   arith_common
        move.w  #54,d0
        bra.s   arith_common
        move.w  #55,d0
arith_common:
        move.w  6(sp),d1
        and.l   #$ffff,d1
        move.l  d1,(LASTFW).l
        move.l  2(sp),(LASTPC).l
        move.l  8(sp),(LASTEA).l       ; (format $3: the address field)
        and.w   #$0fff,d1
        lsr.w   #2,d1
        cmp.w   d0,d1
        bne     fail
        fsave   (HSAVE).l
        sub.w   #48,d0
        lsl.w   #2,d0
        lea     (COUNT).l,a0
        addq.l  #1,(a0,d0.w)
        cmp.w   #7*4,d0
        bne.s   arith_rte
        move.l  (SKIP).l,2(sp)         ; vector 55 resumes past the op
arith_rte:
        rte

fail:
        move.w  #$bad0,($f102).l
        bra.s   *

        cnop    0,4
ovx_s:  dc.l    $407f0000,$80000000,$00000000
ovx_d:  dc.l    $43ff0000,$80000000,$00000000
ovx_si: dc.l    $407f0000,$80000000,$00000001
unf_small_s: dc.l $3f670000,$80000000,$00000000
unf_min_s:   dc.l $bf810000,$80000000,$00000000
unf_small_d: dc.l $3bca0000,$80000000,$00000000
unf_min_d:   dc.l $bc010000,$80000000,$00000000
