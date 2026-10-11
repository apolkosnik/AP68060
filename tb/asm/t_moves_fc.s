; MOVES must use the space SFC/DFC selects, even when the transfer is issued
; in the same cycle the instruction asks for it.
;
; The fast issue path starts a transfer from the calling state when the memory
; port is free, which is exactly what a cache hit leaves behind.  MOVES selects
; its space by writing fc_ovr_v/fc_ovr, and those are registers: an issue in
; that same cycle read them one edge early and fell back on the supervisor
; default.  A kernel copying to or from user memory then reads its own space --
; NetBSD's init pathname came back empty because supervisor $2000 is ROM.
;
; Page 9 is mapped supervisor-only, so the space actually used is visible from
; software: a user-space MOVES there must fault, a supervisor-space one must
; not.  The reads are run cache-hot, because that is when the port is free.
;
; Tests 4-6 chase the same defect shape on ORDINARY accesses.  Their function
; code comes from the S bit, and the issue block reads that bit in the same
; clocked block that starts the transfer -- so if the S bit were read one edge
; early, the first access after a mode change would go out in the previous
; mode's space.  The assertion is not just "did it fault": the 68060's
; access error frame (format $4, MC68060UM 8.4.4) carries the fault status
; long word, whose TM field (FSLW bits 18-16, 8.4.4.3: "the TM2-TM0 signal
; encoding for the faulted access") is the function code the access actually
; used, so the space is read back directly.  Every fault's whole FSLW is
; checked: R ($01000000) or W ($00800000), SIZE 00 = long, TT 00, TM, and SP
; ($00000100, user access to a supervisor-only page) -- nothing else.
;
; Tests 9-13 are the 68060's own MOVES rule (MC68060UM 4.2.5, table 4-2):
; the instruction spaces SFC/DFC = 2 and 6 are CONVERTED to the data spaces
; 1 and 5 ("the resulting access error stack frame contains the converted
; function code in the TM field"), with TT 00.  So SFC=2 reads page 9 as
; user data (SP fault, TM 1), SFC=6 / DFC=6 reach it as supervisor data (no
; fault), DFC=2 writes it as user data (SP fault, TM 1), and a DFC=2 write
; to the bench's $F120 goes out on the bus with TM 1 (the bench fails the
; run on any other TM there).

FAILREG         equ $F100
DONEREG         equ $F102
cnt_aerr        equ $3600
fslw_last       equ $3604               ; FSLW of the last access fault
fa_last         equ $3608               ; and its fault address
FCREG           equ $F120               ; bench: writes must carry TM 1
FS_UREAD        equ $01010100           ; R, long, TT 0, TM 1, SP
FS_UWRITE       equ $00810100           ; W, long, TT 0, TM 1, SP

        org     0
        dc.l    $3400,start
        dc.l    h_aerr                  ; vector 2: access fault
        rept    29
        dc.l    unexpected
        endr                            ; vectors 3-31
        dc.l    h_trap0                 ; vector 32: TRAP #0
        rept    223
        dc.l    unexpected
        endr                            ; vectors 33-255

        org     $400
start:
        move.w  #$2700,sr
        clr.w   (cnt_aerr).l

;----------------------------------------------------------------- tables
        lea     ($4400).l,a0
        moveq   #0,d0
        moveq   #63,d1
tloop:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2                   ; i << 12
        addq.l  #3,d2                   ; resident
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tloop
        move.l  #$00009083,($4424).l    ; page 9: supervisor only
        move.l  #$00004203,($4000).l
        move.l  #$00004403,($4200).l
        move.l  #$5A5A1234,($9000).l    ; something to read back

        move.l  #$4000,d0
        movec   d0,urp
        movec   d0,srp
        move.l  #$8000,d0               ; E=1, 4K pages
        movec   d0,tc
        pflusha

        ; caches on, and warm a line so the port is free when MOVES issues
        move.l  #$00000808,d0
        movec   d0,cacr
        move.l  #$80008000,d0
        movec   d0,cacr
        move.l  ($3000).l,d0
        move.l  ($3000).l,d0
        move.l  ($9000).l,d0            ; supervisor read: legal, warms it

;--------------------------------- 1: supervisor space must reach page 9
        moveq   #5,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        moves.l ($9000).l,d1
        cmp.l   #$5A5A1234,d1
        bne     f1
        tst.w   (cnt_aerr).l
        bne     f1

;--------------------------------- 2: user space must NOT.  This is the bug:
; with the function code read one edge early the access goes out as
; supervisor data and succeeds, so no fault is counted.
        moveq   #1,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        moveq   #0,d1
        moves.l ($9000).l,d1
        tst.w   (cnt_aerr).l
        beq     f2                      ; no fault -> wrong space was used
        cmp.l   #FS_UREAD,(fslw_last).l ; user data read, SP
        bne     f2
        cmp.l   #$9000,(fa_last).l
        bne     f2
        move.l  #$00009083,($4424).l    ; restore supervisor-only
        pflusha

;--------------------------------- 3: the same for the write direction
        moveq   #1,d0
        movec   d0,dfc
        clr.w   (cnt_aerr).l
        move.l  #$DEADBEEF,d1
        moves.l d1,($9000).l
        tst.w   (cnt_aerr).l
        beq     f3
        cmp.l   #FS_UWRITE,(fslw_last).l ; user data write, SP
        bne     f3
        cmp.l   #$9000,(fa_last).l
        bne     f3
        move.l  #$00009083,($4424).l    ; restore supervisor-only
        pflusha

;--------------------------------- 4: the first USER access after an RTE
; Page 9 is supervisor-only, so a user read of it must fault AND the FSLW must
; say the access really went out as user data (TM 1).  If the issue read the
; S bit one edge early it goes out as supervisor data and never faults.
        move.l  #$0000A083,($4428).l    ; page 10 supervisor-only: sup stack
        move.l  #$0000B083,($442C).l    ; page 11 supervisor-only: test 5 data
        move.l  #$00009083,($4424).l    ; page 9 supervisor-only again
        pflusha
        move.l  #$B0B0CAFE,($B000).l    ; supervisor writes: legal
        move.l  #$5A5A1234,($9000).l    ; test 3's retried MOVES left $DEADBEEF
        move.l  ($9000).l,d0            ; warm page 9 again
        clr.w   (cnt_aerr).l
        clr.l   (fslw_last).l

        move.l  #$3800,a0
        move.l  a0,usp
        move.l  #$AF00,a7               ; supervisor stack inside page 10
        move.w  #$0000,-(a7)            ; format 0 / vector
        move.l  #u_start,-(a7)          ; PC
        move.w  #$0000,-(a7)            ; SR: user mode, IPL 0
        rte

u_start:
        move.l  ($9000).l,d1            ; the first access in user mode
        trap    #0                      ; and back, through page 10's stack

;--------------------------------- 5: the first SUPERVISOR access after entry
; Reached only if test 6 held.  Page 11 is supervisor-only, and this is the
; first access after the mode change, so it must NOT fault.
h_trap0:
        move.l  ($B000).l,d2

;--------------------------------- 6: the exception frame push itself
; The TRAP above pushed its frame onto $AF00, inside supervisor-only page 10,
; from user mode.  The mode change precedes the push, so the push must use
; supervisor data.  A user-space push faults inside exception processing,
; which is a double fault -- reaching here at all is the assertion.

        tst.w   (cnt_aerr).l            ; exactly one fault, test 4's
        beq     f4                      ; none: user access used the wrong space
        cmpi.w  #1,(cnt_aerr).l
        bne     f5                      ; more: test 5 faulted when it must not
        move.l  (fslw_last).l,d3
        swap    d3
        andi.w  #$0007,d3               ; FSLW TM (18-16) = the space used
        cmpi.w  #1,d3
        bne     f6                      ; not user data
        cmp.l   #FS_UREAD,(fslw_last).l ; and the rest: R, long, SP only
        bne     f6
        cmp.l   #$00009000,(fa_last).l
        bne     f7
        cmp.l   #$5A5A1234,d1           ; the retried user read got the data
        bne     f8
        cmp.l   #$B0B0CAFE,d2           ; and test 5's read worked
        bne     f5

;--------------------------------- 9: SFC=2 (user program) reads as USER DATA
; Table 4-2: SFC 010 -> TT 00, TM 001.  Page 9 is supervisor-only, so the
; converted access faults with SP and the frame's TM is the converted 1.
        move.l  #$00009083,($4424).l    ; page 9 supervisor-only again
        pflusha
        moveq   #2,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        clr.l   (fslw_last).l
        moveq   #0,d1
        moves.l ($9000).l,d1
        cmpi.w  #1,(cnt_aerr).l         ; exactly one fault, then the
        bne     f9                      ; restart (S dropped) succeeds
        cmp.l   #FS_UREAD,(fslw_last).l ; TM 1, not 2; TT 00
        bne     f9
        cmp.l   #$9000,(fa_last).l
        bne     f9
        cmp.l   #$5A5A1234,d1
        bne     f9

;--------------------------------- 10: SFC=6 (supervisor program) reads as
; SUPERVISOR DATA (table 4-2: 110 -> TM 101): page 9 is reachable.
        move.l  #$00009083,($4424).l
        pflusha
        moveq   #6,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        moveq   #0,d1
        moves.l ($9000).l,d1
        tst.w   (cnt_aerr).l
        bne     f10                     ; faulted: a user (or program) space
        cmp.l   #$5A5A1234,d1
        bne     f10

;--------------------------------- 11: DFC=6 writes as supervisor data
        moveq   #6,d0
        movec   d0,dfc
        move.l  #$600DF00D,d1
        moves.l d1,($9000).l
        tst.w   (cnt_aerr).l
        bne     f11
        cmp.l   #$600DF00D,($9000).l    ; supervisor read back
        bne     f11

;--------------------------------- 12: DFC=2 writes as USER DATA: SP fault
        moveq   #2,d0
        movec   d0,dfc
        clr.l   (fslw_last).l
        move.l  #$5A5A1234,d1
        moves.l d1,($9000).l
        cmpi.w  #1,(cnt_aerr).l
        bne     f12
        cmp.l   #FS_UWRITE,(fslw_last).l ; TM 1, not 2; TT 00
        bne     f12
        cmp.l   #$9000,(fa_last).l
        bne     f12
        cmp.l   #$5A5A1234,($9000).l    ; the restarted MOVES wrote it
        bne     f12

;--------------------------------- 13: DFC=2 on the bus: TM 1
; The bench counts an error (and fails the run) for a write to $F120 whose
; TM is not 1, so this checks the converted function code on the pins.
        move.l  #$0000F003,($443C).l    ; page 15 (the bench) user-accessible
        pflusha                         ; (it is already; keep it explicit)
        move.w  #$1234,d1
        moves.w d1,(FCREG).l
        cmpi.w  #1,(cnt_aerr).l         ; no new fault
        bne     f13
        nop                             ; let the write reach the bus

        moveq   #0,d0
        movec   d0,tc
        movec   d0,cacr
        pflusha
        move.w  #$600D,(DONEREG).l
        stop    #$2700

; Repair and retry, as the other fault tests do: clear page 9's supervisor bit
; so the restarted MOVES completes, and count that the fault happened at all.
; (The 060 makes no ATC entry for a search that failed, 4.3; the PFLUSHA is
; kept only because the earlier SUPERVISOR accesses of the page did load an
; entry, which must not be reused with the old S bit after the repair --
; the ATC tags entries by FC2 so it would not be, but the test is about the
; function code, not the ATC.)  The frame: +0 SR, +2 PC, +6 $4008, +8 FA,
; +$C FSLW, at 8(sp) here.
; The count is the whole assertion -- a MOVES that never faults used the wrong
; space.
h_aerr:
        movem.l d6/a6,-(sp)             ; frame offsets below include these 8
        cmpi.w  #$4008,14(sp)           ; format $4, vector 2
        bne     unexpected
        addq.w  #1,(cnt_aerr).l
        move.l  20(sp),(fslw_last).l    ; FSLW: TM is the space the access used
        move.l  16(sp),(fa_last).l      ; and the fault address
        move.l  16(sp),d6
        lsr.l   #8,d6
        lsr.l   #4,d6                   ; page number
        lsl.l   #2,d6                   ; descriptor offset
        lea     ($4400).l,a6
        adda.l  d6,a6
        bclr    #7,3(a6)                ; drop that page's supervisor bit
        pflusha
        movem.l (sp)+,d6/a6
        rte

f1:     moveq   #1,d7
        bra     fail
f2:     moveq   #2,d7
        bra     fail
f3:     moveq   #3,d7
        bra     fail
f4:     moveq   #4,d7
        bra     fail
f5:     moveq   #5,d7
        bra     fail
f6:     moveq   #6,d7
        bra     fail
f7:     moveq   #7,d7
        bra     fail
f8:     moveq   #8,d7
        bra     fail
f9:     moveq   #9,d7
        bra     fail
f10:    moveq   #10,d7
        bra     fail
f11:    moveq   #11,d7
        bra     fail
f12:    moveq   #12,d7
        bra     fail
f13:    moveq   #13,d7
        bra     fail
unexpected:
        moveq   #15,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:
        bra     halt
