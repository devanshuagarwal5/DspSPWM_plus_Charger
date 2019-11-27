;;
;; __eedata_helper2: Erase and write to eedata memory
;;   on PIC24FxxK devices
;;
;; entry 5: erase words
;; entry 6: erase all
;; entry 7: write a word
;;
        .global __eedata_helper5, __eedata_helper6
        .global __eedata_helper7

        .equ    WR,15            ; write control bit in NVMCON
        
        .equ    _EE_ERASE_WORD_CODE,0x4058
        .equ    _EE_ERASE_4WORDS_CODE,0x4059
        .equ    _EE_ERASE_8WORDS_CODE,0x405A
        .equ    _EE_ERASE_ALL_CODE,0x4050
        .equ    _EE_WRITE_WORD_CODE,0x4004

        .equ    _EE_WORD,2
        .equ    _EE_4WORDS,8
        .equ    _EE_8WORDS,16

        .ifdef ffunction
	.section .libc.eedata_helper567,code
	.else
        .section .libc,code
	.endif
       
__eedata_helper5:
;;
;; Erase words
;; 
;; Registers used:  w0 w1 w2
;;                  (preserves TBLPAG)
;; Inputs:
;;  w0,w1 = destination address   (24 bits)
;;  w2 = len (_EE_WORD, _EE_4WORDS, or _EE_8WORDS)
;; 
;; Outputs: None
;; 
        push    TBLPAG
        mov     w1,_TBLPAG      ; TBLPAG = tblpage(dest)
        tblwtl  w0,[w0]         ; set base address of erase block
        mov     #_EE_ERASE_WORD_CODE,w1
        cp      w2,#_EE_WORD    ; erase word mode?
        bra     eq,1f           ; br if so

        mov     #_EE_ERASE_4WORDS_CODE,w1
        cp      w2,#_EE_4WORDS  ; erase 4 words mode?
        bra     eq,1f           ; br if so

        mov     #_EE_ERASE_8WORDS_CODE,w1
        cp      w2,#_EE_8WORDS  ; erase 8 words mode?
        bra     nz,2f           ; br if not
        
1:      mov     w1,NVMCON       ; set erase code
        
finish: disi    #5              ; block level 0-6 interrupts
        mov     #0x55, w1       ; 
        mov     w1, NVMKEY      ; 
        mov     #0xAA, w1       ; 
        mov     w1, NVMKEY      ; unlock sequence
        bset    NVMCON, #WR     ; begin erase/write
        nop
        nop
2:      pop     TBLPAG
        return

__eedata_helper6:
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

__eedata_helper7:
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

.include "null_signature.s"

