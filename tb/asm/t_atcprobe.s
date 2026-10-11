; 68060 ATC probe.  The 68040 version of this test used PTEST/MMUSR to look
; at the 68040's NEGATIVE ATC entries: on the 040 a failed search (invalid
; descriptor, bus error, protection) still loads a nonresident entry, so a
; repaired descriptor is not seen until a PFLUSH.  The MC68060 has no PTEST
; (F-line) and no MMUSR, and it does the opposite (MC68060UM 4.3: "if any
; bus errors (TEA asserted) or invalid descriptors are encountered, the ATC
; is not modified and an access error exception is taken ... all ATC
; entries represent usable translations"; WinUAE cpummu.cpp mmu_fill_atc:
; "68040 always creates ATC entry. 68060 only if valid descriptor was
; found").  Each 040 probe is therefore replaced by the 060 mechanism that
; exercises the same hardware path:
;
;   * a failed search of each kind -- invalid page (PF), invalid pointer
;     (PTB), invalid root (PTA), bus error during the search (TWE, via the
;     bench's $F146 injection), a write to a write-protected page (WP), a
;     user access to a supervisor-only page (SP, read and write), an
;     instruction fetch from an invalid page (IO|PF), PLPAR / PLPAW on an
;     invalid page, PLPAW on a write-protected page -- is repaired by the
;     handler WITHOUT a PFLUSH and the restarted access must then succeed
;     with exactly one fault: no entry was made.  The FSLW cause bits, the
;     fault address and the stacked PC (the faulting instruction, 8.4.4.1)
;     are checked for every one.
;   * the history bits of table 4-1 / 4.2.5: a write to a write-protected
;     page updates NEITHER U nor M; a user access to a supervisor-only page
;     sets U and not M (read and write); successful searches set U (read)
;     and U+M (write), PLPAR / PLPAW as a read / write would.
;   * a SUCCESSFUL search's entry persists until PFLUSH (the 040 test's
;     "protection must persist until flush"): a read of a write-protected
;     page loads an entry with W set, the descriptor is then made writable,
;     and the write still faults from the ATC (no search, descriptor
;     untouched) until the handler's PFLUSH.
;   * separate instruction and data ATCs (4.3 is per MMU): a data entry is
;     not used for a fetch and an instruction entry is not used for data.
;   * 8 Kbyte pages: the fault path, and one entry covering both 4K halves.
;   * TCR NAD / NAI (4.1.2): no allocation, so a changed descriptor is seen
;     at once; an entry loaded before NAD still hits; under NAD a write that
;     finds a valid read entry updates M and the entry stays valid.
;   * TCR FOTC (4.1.2: "the data ATC operates with 32 entries"): three
;     pages of one ATC set.  With 64 entries (4 ways) all three old
;     translations survive a descriptor change; with 32 (2 ways per set)
;     at most two can, whatever the replacement counter holds, and the
;     last one loaded always does.
;
; Descriptors here: page = PA | 1 (UDT 01), W = bit 2, U = bit 3, M = bit 4,
; S = bit 7; tables = address | 3.  Every data page probed holds, at
; offset $100, its own physical address, so a read shows which physical
; page a translation used.

FAILREG         equ $F100
DONEREG         equ $F102
ARMTW           equ $F146           ; one-shot bus error on a table search

cnt_aerr        equ $3600           ; access errors taken (word)
maxcnt          equ $3602           ; more than this many: fail (word)
f_pc            equ $3604           ; last fault: stacked PC
f_fa            equ $3608           ; fault address
f_fslw          equ $360C           ; FSLW
f_fslw1         equ $3610           ; the FIRST fault's FSLW
watch           equ $3614           ; address of a descriptor to sample
watched         equ $3618           ; its value at the FIRST fault
repair          equ $361C           ; handler's repair routine
u_ret           equ $3620           ; supervisor continuation after TRAP #0
scratch         equ $3624
tnum            equ $3628           ; current test (word)

ROOT            equ $4000           ; 128 root descriptors
PTR             equ $4200           ; 128 pointer descriptors (root 0)
PT              equ $4400           ; 64 page descriptors (pointer 0)
PT2             equ $4500           ; page table of pointer 1 (LA $40000)
PTR2            equ $4600           ; pointer table of root 1 (LA $02000000)
D9              equ PT+9*4          ; 4K page 9 ($9000)
DA              equ PT+10*4
DB              equ PT+11*4
D25             equ PT+25*4         ; $19000: same ATC set as $9000
D41             equ PT+41*4         ; $29000: same ATC set as $9000
D8K4            equ PT+4*4          ; 8K page 4 ($8000-$9FFF)

TC_E            equ $8000
TC_P            equ $4000
TC_NAD          equ $2000
TC_NAI          equ $1000
TC_FOTC         equ $0800

; FSLW (8.4.4.3; bit layout as WinUAE cpummu.cpp mmu_bus_error builds it)
FS_R            equ $01000000
FS_W            equ $00800000
FS_LK           equ $02000000
FS_SIZE         equ $00600000       ; SIZE field (long = 00)
FS_TM1          equ $00010000       ; user data
FS_TM5          equ $00050000       ; supervisor data
FS_TM6          equ $00060000       ; supervisor program
FS_IO           equ $00008000
FS_PTA          equ $00001000
FS_PTB          equ $00000800
FS_PF           equ $00000200
FS_SP           equ $00000100
FS_WP           equ $00000080
FS_TWE          equ $00000040

checkl  macro
        cmp.l   #\2,\1
        beq.s   .ok\@
        move.w  #\3,d7
        bra     fail
.ok\@:
        endm

checkw  macro
        cmp.w   #\2,\1
        beq.s   .ok\@
        move.w  #\3,d7
        bra     fail
.ok\@:
        endm

; FSLW without the SIZE field: for an instruction fetch (the 060 fetches
; long words; WinUAE reports the opword's word read) and for PLPA (it has
; no operand; WinUAE passes a dummy size) the manual does not define SIZE.
chkfs   macro
        move.l  \1,d0
        andi.l  #~FS_SIZE,d0
        checkl  d0,\2,\3
        endm

        org     0
        dc.l    $3400,start
        dc.l    h_aerr                  ; 2 access error
        rept    29
        dc.l    unexpected
        endr
        dc.l    h_trap0                 ; 32 TRAP #0: back to supervisor
        rept    223
        dc.l    unexpected
        endr

        org     $400
start:
        move.w  #$2700,sr
        moveq   #0,d0
        movec   d0,tc
        movec   d0,cacr                 ; caches off: physical pages are
        pflusha                         ; read as they are in memory

        ; markers: long at page+$100 = its own physical address, every 4K
        ; page from $8000 to $3F000 except the bench's ($xF000)
        move.l  #$8100,a0
mk_lp:  move.l  a0,d0
        andi.w  #$F000,d0
        cmpi.w  #$F000,d0
        beq.s   mk_skip
        move.l  a0,(a0)
mk_skip:
        adda.l  #$1000,a0
        cmpa.l  #$40100,a0
        bne.s   mk_lp
        ; code pages: $B000 returns 1, $C000 returns 2
        move.l  #$70014E75,($B000).l    ; MOVEQ #1,D0 ; RTS
        move.l  #$70024E75,($C000).l    ; MOVEQ #2,D0 ; RTS

;------------------------------------------------------------------ 1: PF
; Invalid page descriptor.  FSLW R, long, TM 5, PF; FA = the address; PC =
; the load.  The handler validates the descriptor and does NOT flush: the
; restart's search must find it (one fault only).  The restart's
; successful read search sets U, not M (table 4-1).
        move.w  #1,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        clr.l   (D9).l
        move.l  #D9,(watch).l
        move.l  #r_valid9,(repair).l
t1_acc: move.l  ($9100).l,d1
        checkl  d1,$9100,1
        checkw  (cnt_aerr).l,1,1
        checkl  (f_fa).l,$9100,1
        checkl  (f_pc).l,t1_acc,1
        checkl  (f_fslw).l,FS_R|FS_TM5|FS_PF,1
        checkl  (watched).l,0,1
        checkl  (D9).l,$9009,1          ; U set by the restart, M clear

;----------------------------------------------------------------- 2: PTB
; Invalid POINTER descriptor: LA $40000 is pointer entry 1 (LA 24-18),
; page table PT2 maps it to physical $9000.
        move.w  #2,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9001,(PT2).l
        move.l  #PTR+4,(watch).l
        move.l  #r_ptb,(repair).l
t2_acc: move.l  ($40100).l,d1
        checkl  d1,$9100,2
        checkw  (cnt_aerr).l,1,2
        checkl  (f_fa).l,$40100,2
        checkl  (f_pc).l,t2_acc,2
        checkl  (f_fslw).l,FS_R|FS_TM5|FS_PTB,2
        checkl  (watched).l,0,2
        checkl  (PT2).l,$9009,2

;----------------------------------------------------------------- 3: PTA
; Invalid ROOT descriptor: LA $02000000 is root entry 1 (LA 31-25), its
; pointer table PTR2 points to PT2, which maps it to physical $9000.
        move.w  #3,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9001,(PT2).l
        move.l  #PT2+3,(PTR2).l
        move.l  #ROOT+4,(watch).l
        move.l  #r_pta,(repair).l
t3_acc: move.l  ($02000100).l,d1
        checkl  d1,$9100,3
        checkw  (cnt_aerr).l,1,3
        checkl  (f_fa).l,$02000100,3
        checkl  (f_pc).l,t3_acc,3
        checkl  (f_fslw).l,FS_R|FS_TM5|FS_PTA,3
        checkl  (watched).l,0,3
        checkl  (PT2).l,$9009,3

;----------------------------------------------------------------- 4: TWE
; A bus error on the search's first descriptor read (the bench's one-shot
; $F146).  FSLW TWE with LK (8.4.4.3; WinUAE mmu_fill_atc's CATCH sets
; LK|TWE), R, TM 5.  The 040 loaded a bus-error entry that refaulted; the
; 060 loads nothing, so the handler just returns and the restart's search
; succeeds.  The sequence runs twice, the first time arming nothing, so
; every code page it fetches is already in the I-ATC when it is armed: the
; only table search after the arm is the data access's.  PFLUSH (A0) makes
; sure that one needs a search.  No NOP is needed between the arming write
; and the load: "All read and write accesses are performed in strict
; program order" (7.10), and a table search waits for the store and push
; buffers to empty before its bus cycles (5.10).  Test 18 repeats this
; with a NOP in between, which must change nothing.
twe_seq macro
        move.w  #\1,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #r_none,(repair).l
        lea     ($9100).l,a0
        moveq   #1,d6
.lp\@:  lea     (scratch).l,a2
        tst.w   d6
        bne.s   .go\@
        lea     (ARMTW).l,a2
.go\@:  pflush  (a0)
        move.w  #1,(a2)
        rept    \2
        nop
        endr
.acc\@: move.l  (a0),d1
        dbra    d6,.lp\@
        checkl  d1,$9100,\1
        checkw  (cnt_aerr).l,1,\1
        checkl  (f_fa).l,$9100,\1
        checkl  (f_pc).l,.acc\@,\1
        checkl  (f_fslw).l,FS_LK|FS_R|FS_TM5|FS_TWE,\1
        endm

        twe_seq 4,0

;------------------------------------------------------------------ 5: WP
; A write to a write-protected page.  FSLW W, TM 5, WP.  Table 4-1 (W=1,
; write): no update of U or M -- the descriptor is exactly as it was at the
; fault.  The handler clears W without a flush; the restart's write search
; then sets U and M.
        move.w  #5,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9005,(D9).l
        move.l  #D9,(watch).l
        move.l  #r_clrw9,(repair).l
        move.l  #$11223344,d2
t5_acc: move.l  d2,($9104).l
        checkw  (cnt_aerr).l,1,5
        checkl  (f_fa).l,$9104,5
        checkl  (f_pc).l,t5_acc,5
        checkl  (f_fslw).l,FS_W|FS_TM5|FS_WP,5
        checkl  (watched).l,$9005,5     ; neither U nor M
        checkl  (D9).l,$9019,5          ; restart: U and M
        checkl  ($9104).l,$11223344,5

;------------------------------------------------------------------ 6: SP
; A USER read of a supervisor-only page.  FSLW R, TM 1, SP.  4.2.5: U is
; set (locked read-modify-write) although the access faults; M is not.
        move.w  #6,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9081,(D9).l
        move.l  #D9,(watch).l
        move.l  #r_clrs9,(repair).l
        move.l  #t6_back,(u_ret).l
        bsr     to_user
t6_acc: move.l  ($9100).l,d1            ; user mode
        trap    #0
t6_back:
        checkl  d1,$9100,6
        checkw  (cnt_aerr).l,1,6
        checkl  (f_fa).l,$9100,6
        checkl  (f_pc).l,t6_acc,6
        checkl  (f_fslw).l,FS_R|FS_TM1|FS_SP,6
        checkl  (watched).l,$9089,6     ; U set, M clear
        checkl  (D9).l,$9009,6

;------------------------------------------------------------------ 7: SP
; A USER write of a supervisor-only page.  FSLW W, TM 1, SP.  U is set by a
; plain write, M stays clear (no modification is allowed).
        move.w  #7,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9081,(D9).l
        move.l  #D9,(watch).l
        move.l  #r_clrs9,(repair).l
        move.l  #t7_back,(u_ret).l
        move.l  #$55667788,d2
        bsr     to_user
t7_acc: move.l  d2,($9104).l            ; user mode
        trap    #0
t7_back:
        checkw  (cnt_aerr).l,1,7
        checkl  (f_fa).l,$9104,7
        checkl  (f_pc).l,t7_acc,7
        checkl  (f_fslw).l,FS_W|FS_TM1|FS_SP,7
        checkl  (watched).l,$9089,7     ; U set, M clear
        checkl  (D9).l,$9019,7
        checkl  ($9104).l,$55667788,7

;---------------------------------------------- 8: a successful entry stays
; A READ of a write-protected page succeeds and loads an entry with W set
; (and sets U).  The descriptor is then made writable, but no PFLUSH: the
; write hits that entry and faults from the ATC (no search, the descriptor
; is not touched), and the handler's first return without a flush faults
; again.  Only after the handler's PFLUSH does the restart search, see the
; writable descriptor and set M.  (The 040 test's "protection must persist
; until flush".)
        move.w  #8,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9005,(D9).l
        move.w  #2,(maxcnt).l
        move.l  ($9100).l,d1            ; loads the W entry
        checkl  d1,$9100,8
        checkl  (D9).l,$900D,8          ; U set by the read
        checkw  (cnt_aerr).l,0,8
        move.l  #$9009,(D9).l           ; writable now, no PFLUSH
        move.l  #D9,(watch).l
        move.l  #r_flush2nd,(repair).l
        move.l  #$99AABBCC,d2
t8_acc: move.l  d2,($9104).l
        checkw  (cnt_aerr).l,2,8        ; the entry refaulted once
        checkl  (f_fslw1).l,FS_W|FS_TM5|FS_WP,8
        checkl  (f_fslw).l,FS_W|FS_TM5|FS_WP,8
        checkl  (f_fa).l,$9104,8
        checkl  (f_pc).l,t8_acc,8
        checkl  (watched).l,$9009,8     ; an ATC fault: no search, no update
        checkl  (D9).l,$9019,8          ; the post-flush search set M
        checkl  ($9104).l,$99AABBCC,8

;------------------------------------------------- 9: fetch from invalid page
; JSR to a page whose descriptor is invalid: the fetch faults when the
; instruction is needed.  FA = the opword ($B000), PC = $B000, FSLW IO
; (opword: MA clear), R, TM 6, PF.  Repaired without a flush; the RTE
; restarts the fetch.
        move.w  #9,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        clr.l   (DB).l
        move.l  #DB,(watch).l
        move.l  #r_validB,(repair).l
        moveq   #0,d0
        jsr     ($B000).l
        checkl  d0,1,9
        checkw  (cnt_aerr).l,1,9
        checkl  (f_fa).l,$B000,9
        checkl  (f_pc).l,$B000,9
        chkfs   (f_fslw).l,FS_IO|FS_R|FS_TM6|FS_PF,9
        checkl  (watched).l,0,9
        checkl  (DB).l,$B009,9

;------------------------------------------------ 10: separate I and D ATCs
; A data read of page $B loads the DATA ATC only.  With the descriptor
; repointed to $C000 (no flush), a JSR there must search for itself and
; run $C000's code, while data reads keep the data entry ($B000).  Then
; with the descriptor put back, the fetch keeps ITS entry ($C000) and the
; data keeps $B000: each ATC holds its own translation.  (The 040 test's
; "data's negative entry must not poison the I-ATC".)  The calls are
; JSR (A1): the instruction fetch unit runs ahead of the operand pipeline
; and may fetch (and translate) a predictable target before the descriptor
; store that precedes the call has reached memory; an indirect target is
; not known until the JSR executes, after that store.
        move.w  #10,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  ($B000).l,d1            ; data entry: physical $B000
        checkl  d1,$70014E75,10
        move.l  #$C001,(DB).l           ; repoint, no flush
        moveq   #0,d0
        lea     ($B000).l,a1
        jsr     (a1)               ; instruction ATC miss: searches
        checkl  d0,2,10
        move.l  ($B000).l,d1            ; data ATC hit: still $B000
        checkl  d1,$70014E75,10
        move.l  #$B009,(DB).l           ; back to $B000, no flush
        moveq   #0,d0
        lea     ($B000).l,a1
        jsr     (a1)               ; instruction ATC hit: $C000
        checkl  d0,2,10
        move.l  ($B000).l,d1
        checkl  d1,$70014E75,10
        checkw  (cnt_aerr).l,0,10

;---------------------------------------------------------- 11: 8K pages
; TC.P: the fault path with 8 Kbyte pages (8K page 4 = $8000-$9FFF), then
; one entry covering both 4K halves: with the descriptor repointed to
; $C000 (no flush) both halves still translate to $8000/$9000; after a
; PFLUSH both go to $C000/$D000 (PA 12 is LA 12, 4.3).
        move.w  #11,(tnum).l
        move.l  #TC_E|TC_P,d0
        bsr     reset_mmu
        clr.l   (D8K4).l
        move.l  #D8K4,(watch).l
        move.l  #r_valid8k,(repair).l
t11_acc:
        move.l  ($9100).l,d1
        checkl  d1,$9100,11
        checkw  (cnt_aerr).l,1,11
        checkl  (f_fa).l,$9100,11
        checkl  (f_pc).l,t11_acc,11
        checkl  (f_fslw).l,FS_R|FS_TM5|FS_PF,11
        checkl  (D8K4).l,$8009,11
        move.l  #$C009,(D8K4).l         ; repoint, no flush
        move.l  ($8100).l,d1            ; the other half: same entry
        checkl  d1,$8100,11
        move.l  ($9100).l,d1
        checkl  d1,$9100,11
        lea     ($8000).l,a0
        pflush  (a0)
        move.l  ($8100).l,d1
        checkl  d1,$C100,11
        move.l  ($9100).l,d1
        checkl  d1,$D100,11

;--------------------------------------------------------------- 12: NAD
; Data ATC no-allocate (4.1.2): misses search and do not allocate, hits
; still hit, and "a write access which finds a corresponding valid read
; will update the M-bit and the entry remains valid".
        move.w  #12,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  ($9100).l,d1            ; page 9 allocated before NAD
        checkl  d1,$9100,12
        move.l  #TC_E|TC_NAD,d0
        movec   d0,tc
        move.l  ($A100).l,d1            ; page A: searched, not allocated
        checkl  d1,$A100,12
        move.l  #$D001,(DA).l           ; repoint, no flush:
        move.l  ($A100).l,d1            ; seen at once
        checkl  d1,$D100,12
        move.l  #$A001,(DA).l
        move.l  ($A100).l,d1
        checkl  d1,$A100,12
        move.l  #$D009,(D9).l           ; page 9's entry still hits
        move.l  ($9100).l,d1
        checkl  d1,$9100,12
        move.l  #$9009,(D9).l           ; M update of the read entry
        move.l  #$13579BDF,($9104).l
        checkl  (D9).l,$9019,12
        move.l  #$D009,(D9).l           ; and the entry stays valid
        move.l  ($9100).l,d1
        checkl  d1,$9100,12
        checkl  ($9104).l,$13579BDF,12
        checkw  (cnt_aerr).l,0,12

;--------------------------------------------------------------- 13: NAI
; Instruction ATC no-allocate: a fetch from a repointed page sees the new
; descriptor at once (compare test 10, where the entry persisted), while
; the data ATC still allocates.
        move.w  #13,(tnum).l
        move.l  #TC_E|TC_NAI,d0
        bsr     reset_mmu
        moveq   #0,d0
        lea     ($B000).l,a1
        jsr     (a1)
        checkl  d0,1,13
        move.l  #$C001,(DB).l           ; repoint, no flush
        moveq   #0,d0
        lea     ($B000).l,a1
        jsr     (a1)               ; not allocated: searched again
        checkl  d0,2,13
        move.l  ($9100).l,d1            ; data still allocates
        move.l  #$D009,(D9).l
        move.l  ($9100).l,d1
        checkl  d1,$9100,13
        checkw  (cnt_aerr).l,0,13

;-------------------------------------------------------------- 14: FOTC
; Three pages of ONE data ATC set (LA 15-12 = 9: $9000, $19000, $29000);
; nothing else this test touches maps to set 9.  Load all three, repoint
; all three descriptors (no flush) and count the reads that still see the
; old page.  64 entries (4 ways): all three survive.  32 entries (FOTC,
; 2 ways): at most two, whatever the replacement counter holds; the one
; loaded last is read first and must survive (nothing missed since it was
; loaded).
        move.w  #14,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        bsr     set3
        checkl  d4,3,14
        move.l  #TC_E|TC_FOTC,d0
        bsr     reset_mmu
        bsr     set3
        cmpi.l  #2,d4
        bls.s   t14_ok
        move.w  #14,d7
        bra     fail
t14_ok: checkw  (cnt_aerr).l,0,14

;------------------------------------------- 15: PLPAR on an invalid page
; PLPAR (A0) with DFC 5 translates A0 as a supervisor data read would; the
; failed search is an access error (FA = A0, FSLW R, TM 5, PF; PC = the
; PLPAR), repaired without a flush; the restarted PLPAR loads the ATC,
; sets U (not M) and returns the PHYSICAL address in A0.  The entry is
; then used: repointing the descriptor again changes nothing.
        move.w  #15,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        clr.l   (D9).l
        move.l  #D9,(watch).l
        move.l  #r_mapD,(repair).l
        lea     ($9100).l,a0
t15_acc:
        dc.w    $F5C8                   ; PLPAR (A0)
        checkl  a0,$D100,15
        checkw  (cnt_aerr).l,1,15
        checkl  (f_fa).l,$9100,15
        checkl  (f_pc).l,t15_acc,15
        chkfs   (f_fslw).l,FS_R|FS_TM5|FS_PF,15
        checkl  (D9).l,$D009,15         ; U, no M
        move.l  #$9009,(D9).l           ; no flush: PLPAR's entry is used
        move.l  ($9100).l,d1
        checkl  d1,$D100,15

;------------------------------------------- 16: PLPAW on an invalid page
; The same as a write: FSLW W, and the successful restart sets U and M.
        move.w  #16,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        clr.l   (D9).l
        move.l  #D9,(watch).l
        move.l  #r_mapD,(repair).l
        lea     ($9100).l,a0
t16_acc:
        dc.w    $F588                   ; PLPAW (A0)
        checkl  a0,$D100,16
        checkw  (cnt_aerr).l,1,16
        checkl  (f_fa).l,$9100,16
        checkl  (f_pc).l,t16_acc,16
        chkfs   (f_fslw).l,FS_W|FS_TM5|FS_PF,16
        checkl  (D9).l,$D019,16         ; U and M
        move.l  #$9019,(D9).l
        move.l  ($9100).l,d1
        checkl  d1,$D100,16

;-------------------------------------- 17: PLPAW on a write-protected page
; PLPAW checks write permission as a write would (4.7.3): WP fault, no U/M
; update; the handler clears W (no flush) and the restart succeeds.
        move.w  #17,(tnum).l
        move.l  #TC_E,d0
        bsr     reset_mmu
        move.l  #$9005,(D9).l
        move.l  #D9,(watch).l
        move.l  #r_clrw9,(repair).l
        lea     ($9100).l,a0
t17_acc:
        dc.w    $F588                   ; PLPAW (A0)
        checkl  a0,$9100,17
        checkw  (cnt_aerr).l,1,17
        checkl  (f_fa).l,$9100,17
        checkl  (f_pc).l,t17_acc,17
        chkfs   (f_fslw).l,FS_W|FS_TM5|FS_WP,17
        checkl  (watched).l,$9005,17
        checkl  (D9).l,$9019,17

;------------------------------------------------- 18: TWE behind a NOP
; Test 4 with a NOP between the arming write and the load.  The NOP is a
; pipeline synchronization (10.2 note 6: it waits until all previous
; instructions have completed and the buffers are empty); it does not
; discard the instruction after it, so the load's one table search still
; meets the injected bus error and reports it.
        twe_seq 18,1

        moveq   #0,d0
        movec   d0,tc
        pflusha
        move.w  #$600D,(DONEREG).l
        stop    #$2700

;---------------------------------------------------------------- helpers
; reset_mmu: D0 = the TC to run with.  Translation off, ATCs flushed, the
; tables rebuilt (identity, UDT 01, U and M clear; 4K or 8K by TC.P), the
; extra tables cleared, URP = SRP = ROOT, DFC = SFC = 5, fault bookkeeping
; cleared, then TC loaded.
reset_mmu:
        move.l  d0,d5
        moveq   #0,d0
        movec   d0,tc
        pflusha
        lea     (ROOT).l,a0             ; ROOT, PTR (128 each): clear
        move.w  #255,d1
rm_clr: clr.l   (a0)+
        dbra    d1,rm_clr
        lea     (PT2).l,a0              ; PT2 (64), PTR2 (128)
        move.w  #191,d1
rm_clr2:
        clr.l   (a0)+
        dbra    d1,rm_clr2
        move.l  #PTR+3,(ROOT).l
        move.l  #PT+3,(PTR).l
        lea     (PT).l,a0
        moveq   #0,d0
        moveq   #63,d1
rm_pt:  move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2                   ; i << 12
        btst    #14,d5
        beq.s   rm_4k
        add.l   d2,d2                   ; i << 13
rm_4k:  addq.l  #1,d2
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,rm_pt
        move.l  #ROOT,d0
        movec   d0,urp
        movec   d0,srp
        moveq   #5,d0
        movec   d0,dfc
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        move.w  #1,(maxcnt).l
        move.l  #scratch,(watch).l
        clr.l   (watched).l
        clr.l   (f_fslw).l
        clr.l   (f_fslw1).l
        clr.l   (f_fa).l
        clr.l   (f_pc).l
        move.l  #r_none,(repair).l
        movec   d5,tc
        rts

; set3: load $9000, $19000, $29000 into the data ATC, repoint their
; descriptors to $D000, $1D000, $2D000 (no flush), then read them back,
; the last loaded first.  D4 = how many still gave the old page; fails
; if the last loaded one did not.
set3:
        move.l  ($9100).l,d1
        move.l  ($19100).l,d1
        move.l  ($29100).l,d1
        move.l  #$D001,(D9).l
        move.l  #$1D001,(D25).l
        move.l  #$2D001,(D41).l
        moveq   #0,d4
        move.l  ($29100).l,d1
        checkl  d1,$29100,14            ; just loaded: must hit
        addq.l  #1,d4
        move.l  ($9100).l,d1
        cmp.l   #$9100,d1
        bne.s   s3_a
        addq.l  #1,d4
s3_a:   move.l  ($19100).l,d1
        cmp.l   #$19100,d1
        bne.s   s3_b
        addq.l  #1,d4
s3_b:   rts

; to_user: continue in user mode at the caller's return address
to_user:
        move.l  (sp)+,a1
        lea     ($3C00).l,a0
        move    a0,usp
        clr.w   -(sp)                   ; format 0
        move.l  a1,-(sp)
        clr.w   -(sp)                   ; SR: user, IPL 0
        rte

h_trap0:
        move.w  #$2700,(sp)
        move.l  (u_ret).l,2(sp)
        rte

; Access error: format $4 (8.4.4): +0 SR, +2 PC, +6 $4008, +8 FA, +$C FSLW.
; Records the fault, samples the watched descriptor at the first one,
; bounds the count, and calls the test's repair routine.  It never
; flushes on its own.
h_aerr:
        cmpi.w  #$4008,6(sp)
        bne     unexpected
        addq.w  #1,(cnt_aerr).l
        move.l  2(sp),(f_pc).l
        move.l  8(sp),(f_fa).l
        move.l  $C(sp),(f_fslw).l
        movem.l d0-d1/a0-a1,-(sp)
        move.w  (cnt_aerr).l,d0
        cmp.w   (maxcnt).l,d0
        bhi     fail_loop               ; refaulted: an entry was cached
        cmpi.w  #1,d0
        bne.s   ha_rep
        move.l  $10+$C(sp),(f_fslw1).l
        move.l  (watch).l,a1
        move.l  (a1),(watched).l
ha_rep: move.l  (repair).l,a1
        jsr     (a1)
        movem.l (sp)+,d0-d1/a0-a1
        rte

r_none: rts
r_valid9:
        move.l  #$9001,(D9).l
        rts
r_ptb:  move.l  #PT2+3,(PTR+4).l
        rts
r_pta:  move.l  #PTR2+3,(ROOT+4).l
        rts
r_clrw9:
        bclr    #2,(D9+3).l
        rts
r_clrs9:
        bclr    #7,(D9+3).l
        rts
r_validB:
        move.l  #$B001,(DB).l
        rts
r_valid8k:
        move.l  #$8001,(D8K4).l
        rts
r_mapD: move.l  #$D001,(D9).l
        rts
; test 8: first fault -- no flush, so it must refault from the ATC;
; second -- PFLUSH the page, the restart searches
r_flush2nd:
        cmpi.w  #2,(cnt_aerr).l
        bne.s   rf_1
        lea     ($9000).l,a0
        pflush  (a0)
rf_1:   rts

fail_loop:
        move.w  (tnum).l,d7
        ori.w   #$100,d7                ; test number + $100: refault
        bra.s   fail
unexpected:
        move.w  #$FF,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:   bra.s   halt
