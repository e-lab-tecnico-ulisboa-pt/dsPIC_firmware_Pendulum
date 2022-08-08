.include "p33FJ128MC804.inc"

.global  _ReadFlashWord, _EraseFlashPage, _WriteFlashWord, _WriteSingleRow, _WriteMem, _ReadLatch  ;C called

;***************************************************************
;Reads the 16 least significant bits of an instruction word from the program
;memory
_ReadFlashWord: ;W0=TBLPAG, W1=offset - data returned in W0
    ;from C: int ReadFlashWord(int page, int offset)
    ;W0=page
    ;W1=offset
	MOV	W0, TBLPAG
	TBLRDL	[W1], W0
	RETURN
;***************************************************************
;KEY sequence required to write/erase the program memory
_flashDo:
;Disable interrupts while the KEY sequence is written
	DISI #6
;Write the KEY sequence
	MOV	#0x55, W2
	MOV	W2, NVMKEY
	MOV	#0xAA, W2
	MOV	W2, NVMKEY
;Start the write/erase cycle
	BSET	NVMCON, #WR
	NOP	    ;required
	NOP	    ;required
;Wait for flash to finish
DeLoop:	BTSC	NVMCON, #WR
	GOTO	DeLoop
	RETURN
;***************************************************************
_EraseFlashPage: ;W0=TBLPAG, W1=offset - no return values
    ;from C: int EraseFlashPage(int page, int offset)
    ;W0=page
    ;W1=offset
    ;Set up NVMCON for block erase operation
	MOV	#0x4042, W2
	MOV	W2, NVMCON	;Initialize NVMCON
    ;Init pointer to row to be ERASED
	MOV	W0, TBLPAG	;Initialize PM Page Boundary SFR
	TBLWTL	W1, [W1]	;Set base address of erase bloc
	CALL	_flashDo
	RETURN
;***************************************************************
;Writes the 16 least significant bits to a program memory word
_WriteFlashWord: ;W0=TBLPAG, W1=offset, W2=Word - no return values
    ;from C: void WriteFlashWord(int page, int offset, int val)
    ;W0=page
    ;W1=offset
    ;W2=word (int)
	MOV	W0, TBLPAG	
	TBLWTL	W2, [W1]
	MOV	#0x4003, W0	;Setup NVMCON for programming one word to data Flash
	MOV	W0, NVMCON
	CALL	_flashDo
	RETURN
;***************************************************************
;Loads the 16 least significant bits to the 64 write buffers to program a full 
;memory row, see page 5-12 of DS70191E
_loadWriteBuffers: ;W0=TBLPAG, W1=offset, W2=Buffer - no return values
    ;W0=page
    ;W1=offset
    ;W2=pointer to buffer (* int[64])
	MOV	W0, TBLPAG
	MOV	#64, W3
loop:
	TBLWTL	[W2++], [W1]
	INC2	W1, W1
	DEC	W3, W3
	BRA	NZ, loop
	RETURN
;***************************************************************
;Loads the 16 least significant bits to the 64 write buffers to program a full 
;memory row, see page 5-12 of DS70191E
_WriteSingleRow: ;W0=TBLPAG, W1=offset, W2=Buffer - no return values
    ;from C: void WriteSingleRow(int page, int offset, int *buf)
    ;W0=page
    ;W1=offset
    ;W2=pointer to buffer (* int[64])
	MOV	#0x4001, W4	;Setup NVMCON for programming one row to data Flash
	MOV	W4, NVMCON
	CALL	_loadWriteBuffers
	CALL	_flashDo
	RETURN
	
	
	
	
	
	
	
	
_WriteMem:	;W0=NVMCON - no return values
	mov	W0,NVMCON
	mov	#0x55,W0	;Unlock sequence - interrupts need to be off
	mov	W0,NVMKEY
	mov	#0xAA,W0
	mov	W0,NVMKEY
	bset	NVMCON,#WR
	nop				;Required
	nop
1:	btsc	NVMCON,#WR	;Wait for write end
	bra 1b
	
	return
	
;***************************************************************	
	
_ReadLatch: ;W0=TBLPAG, W1=Wn - data in W1:W0
	mov W0, TBLPAG
	TBLRDL	[W1], W0
	TBLRDH	[W1], W1	;not sure what this does
	RETURN
;***************************************************************	
	

	