;*************************************************************************
; common/up_saveusercontext.S
;
;   Copyright (C) 2007 Gregory Nutt. All rights reserved.
;   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in
;    the documentation and/or other materials provided with the
;    distribution.
; 3. Neither the name Gregory Nutt nor the names of its contributors may be
;    used to endorse or promote products derived from this software
;    without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
; COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
; BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
; OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
; AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
; LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
; ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
; POSSIBILITY OF SUCH DAMAGE.
;
;*************************************************************************

;*************************************************************************
; Constants
;*************************************************************************

	; Register save area layout

	XCPT_I		==  0	; Saved I w/interrupt state in carry
	XCPT_AF		==  2	; Saved AF register
	XCPT_BC		==  4	; Saved BC register
	XCPT_DE		==  6	; Saved DE register
	XCPT_HL		==  8	; Saved HL register
	XCPT_IX		== 10	; Saved IX register
	XCPT_IY		== 12	; Saved IY register
	XCPT_SP		== 14	; Offset to SP at time of interrupt
	XCPT_PC		== 16	; Offset to PC at time of interrupt

	; Stack frame

	FRAME_IY	==  0	; Location of IY on the stack
	FRAME_IX	==  2	; Location of IX on the stack
	FRAME_RET	==  4	; Location of return address on the stack
	FRAME_REGS	==  6	; Location of reg save area on stack

	SP_OFFSET	== -6

;*************************************************************************
; Name: up_saveusercontext
;*************************************************************************

	.area	TEXT	(ABS,OVR)
up_saveusercontext:
	; Set up a stack frame

	push	ix			; Save IX and IY
	push	iy
	ld	ix, #0
	add	ix, sp			; IX = stack frame

	; Fetch the address of the save area

	ld	l, FRAME_REGS(ix)	; HL = save area address
	ld	h, FRAME_REGS+1(ix)	;
	ld	iy, #0
	add	iy, hl			; IY = save area address

	; Then save the registers

	; Save the current interrupt state at offset 0

	ld	a, i			; Get interrupt state
	push	af
	pop	hl
	ld	(iy+XCPT_I), l		; Offset 0: I w/interrupt state in carry
	ld	(iy+XCPT_I+1), h

	; Save BC at offset 1

	ld	(iy+XCPT_BC), c		; Offset 1: BC
	ld	(iy+XCPT_BC+1), b

	; DE is not preserved (offset 2)

	; Save IX at offset 3

	ld	l, FRAME_IX(ix)		; HL = Saved alue of IX
	ld	h, FRAME_IX+1(ix)	;
	ld	(iy+XCPT_IX), l		; Offset 3: IX
	ld	(iy+XCPT_IX+1), h	;

	; Save IY at offset 4

	ld	l, FRAME_IY(ix)		; HL = Saved value of IY
	ld	h, FRAME_IY+1(ix)	;
	ld	(iy+XCPT_IY), l		; Offset 4: IY
	ld	(iy+XCPT_IY+1), h

	; Save that stack pointer as it would be upon return in offset 5

	ld	hl, #SP_OFFSET		; Value of stack pointer on return
	add	hl, sp
	ld	(iy+XCPT_SP), l		; Offset 5 SP
	ld	(iy+XCPT_SP+1), h

	; HL is saved as the value 1 at offset 6

	xor	a			; A = 0
	ld	(iy+XCPT_HL+1), a	; Offset 2: HL on return (=1)
	inc	a			; A = 1
	ld	(iy+XCPT_HL), a		;

	; AF is not preserved (offset 7)

	; Save the return address in offset 8

	ld	l, FRAME_RET(ix)	; HL = Saved return address
	ld	h, FRAME_RET+1(ix)	;
	ld	(iy+XCPT_PC), l		; Offset 8: PC
	ld	(iy+XCPT_PC+1), h

	; Return the value 0

	xor	a			; HL = return value of zero
	ld	l, a
	ld	h, a

	pop	iy
	pop	ix
	ret
