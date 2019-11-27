;;
;; __eedata_helper: Erase and write to eedata memory
;;
;; entry 1: erase word or row
;; entry 2: erase all
;; entry 3: write a word
;; entry 4: write a row
;;
	.ifdef ffunction
	.section .libc.eedata_helper1234,code
	.else
        .section .libc,code
	.endif
        .global __eedata_helper1, __eedata_helper2
        .global __eedata_helper3, __eedata_helper4

        .equ    WR,15                         ; write control bit in NVMCON
        
        .equ    _EE_ERASE_WORD_CODE,0x4044    ; valid for all devices
        .equ    _EE_ERASE_ROW_CODE,0x4045     ; with EEDATA
        .equ    _EE_ERASE_ALL_CODE,0x4046

        .equ    _EE_WRITE_WORD_CODE,0x4004
        .equ    _EE_WRITE_ROW_CODE,0x4005

        .equ    _EE_WORD,2
        .equ    _EE_ROW,32
       
__eedata_helper1:
;;
;; Erase eedata word or row
;; 
;; Registers used:  w0 w1 w2
;;                  (preserves TBLPAG)
;; Inputs:
;;  w0,w1 = destination address   (24 bits)
;;  w2 = len (EE_WORD or EE_ROW)
;; 
;; Outputs: None
;; 
        push    TBLPAG
        mov     w1,_TBLPAG      ; TBLPAG = tblpage(dest)
        tblwtl  w0,[w0]         ; set base address of erase block
        mov     #_EE_ERASE_WORD_CODE,w1
        cp      w2,#_EE_WORD    ; erase word mode?
        bra     eq,1f           ; br if so

        mov     #_EE_ROW,W0     ; 
        cp      w2,W0           ; erase row mode?
        bra     nz,2f           ; br if no
        
        mov     #_EE_ERASE_ROW_CODE,w1
1:      mov     w1,NVMCON       ; set erase code
        
finish: disi    #5              ; block level 0-6 interrupts
        mov     #0x55, w1       ; 
        mov     w1, NVMKEY      ; 
        mov     #0xAA, w1       ; 
        mov     w1, NVMKEY      ; unlock sequence
        bset    NVMCON, #15     ; begin erase/write
        nop
        nop
2:      pop     TBLPAG
        return

__eedata_helper2:
;;
;; Erase all eedata memory
;; 
;; Registers used:  w0 w1
;;
;; Inputs: None
;; 
;; Outputs: None
;;
        
        push    TBLPAG
        mov     #_EE_ERASE_ALL_CODE,w0
        mov     w0,NVMCON       ; set erase code
        bra     finish

__eedata_helper3:
;;
;; Write a word of eedata memory
;; 
;; Registers used:  w0 w1 w2
;;                  (preserves TBLPAG)
;; Inputs:
;;  w0,w1 = destination address   (24 bits)
;;  w2 = 16-bit data
;; 
;; Outputs: None
;; 
        push    TBLPAG
        mov     w1,_TBLPAG      ; TBLPAG = tblpage(dest)
        mov     #_EE_WRITE_WORD_CODE,w1
        mov     w1,NVMCON       ; set word write mode

        tblwtl  w2,[w0]         ; write data
        bra     finish

__eedata_helper4:
;;
;; Write a row of eedata memory
;; 
;; Registers used:  w0 w1 w2 w3
;;                  (preserves TBLPAG)
;; Inputs:
;;  w0,w1 = destination address   (24 bits)
;;  w2 = source address
;; 
;; Outputs: None
;; 
        
        push    TBLPAG
        mov     w1,_TBLPAG      ; TBLPAG = tblpage(dest)
        mov     #_EE_WRITE_ROW_CODE,w1
        mov     w1,NVMCON       ; set row write mode
        mov     #_EE_ROW/2,w3   ; set word count

1:      tblwtl  [w2++],[w0++]   ; write data word
        dec     w3,w3
        bra     nz,1b           ; loop until done
        bra     finish


.include "null_signature.s"

