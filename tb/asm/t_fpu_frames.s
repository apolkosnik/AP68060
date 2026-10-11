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

        check   sp,$e000,99
        move.w  #$600d,($f102).l
        bra.s   *

;------------------------------------------------ handlers
fline_handler:
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
