;*************************************************************************
; arch/z80/src/ez80/ez80_saveusercontext.asm
;
; Licensed to the Apache Software Foundation (ASF) under one or more
; contributor license agreements.  See the NOTICE file distributed with
; this work for additional information regarding copyright ownership.  The
; ASF licenses this file to you under the Apache License, Version 2.0 (the
; "License"); you may not use this file except in compliance with the
; License.  You may obtain a copy of the License at
;
;   http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
; License for the specific language governing permissions and limitations
; under the License.
;
;*************************************************************************

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

;**************************************************************************
; Global Symbols Expported
;**************************************************************************

	xdef	_ez80_saveusercontext

;*************************************************************************
; Constants
;*************************************************************************

CONFIG_EZ80_Z80MODE	equ	0

	.if CONFIG_EZ80_Z80MODE
	; Register save area layout

XCPT_I		equ	2*0	; Offset 0: Saved I w/interrupt state in parity
XCPT_BC		equ	2*1	; Offset 1: Saved BC register
XCPT_DE		equ	2*2	; Offset 2: Saved DE register
XCPT_IX		equ	2*3	; Offset 3: Saved IX register
XCPT_IY		equ	2*4	; Offset 4: Saved IY register
XCPT_SP		equ	2*5	; Offset 5: Offset to SP at time of interrupt
XCPT_HL		equ	2*6	; Offset 6: Saved HL register
XCPT_AF		equ	2*7	; Offset 7: Saved AF register
XCPT_PC		equ	2*8	; Offset 8: Offset to PC at time of interrupt

	; Stack frame

FRAME_IY	equ	2*0	; Location of IY on the stack
FRAME_IX	equ	2*1	; Location of IX on the stack
FRAME_RET	equ	2*2	; Location of return address on the stack
FRAME_REGS	equ	2*3	; Location of reg save area on stack

SP_OFFSET	equ	2*3
	.else
	; Register save area layout

XCPT_I		equ	3*0	; Offset 0: Saved I w/interrupt state in parity
XCPT_BC		equ	3*1	; Offset 1: Saved BC register
XCPT_DE		equ	3*2	; Offset 2: Saved DE register
XCPT_IX		equ	3*3	; Offset 3: Saved IX register
XCPT_IY		equ	3*4	; Offset 4: Saved IY register
XCPT_SP		equ	3*5	; Offset 5: Offset to SP at time of interrupt
XCPT_HL		equ	3*6	; Offset 6: Saved HL register
XCPT_AF		equ	3*7	; Offset 7: Saved AF register
XCPT_PC		equ	3*8	; Offset 8: Offset to PC at time of interrupt	.endif

	; Stack frame

FRAME_IY	equ	3*0	; Location of IY on the stack
FRAME_IX	equ	3*1	; Location of IX on the stack
FRAME_RET	equ	3*2	; Location of return address on the stack
FRAME_REGS	equ	3*3	; Location of reg save area on stack

SP_OFFSET	equ	3*3
	.endif

;**************************************************************************
; Code
;**************************************************************************

	segment	CODE
	.assume ADL=1

;*************************************************************************
; Name: z80_saveusercontext
;*************************************************************************

_ez80_saveusercontext:
	; Set up a stack frame

	push	ix			; Save IX and IY
	push	iy
	ld	ix, 0
	add	ix, sp			; IX = stack frame

	; Fetch the address of the save area

	ld	de, (ix + FRAME_REGS)	; DE = save area address
	ld	iy, 0
	add	iy, de			; IY = save area address

	; Then save the registers

	; Save the current interrupt state at offset 0

	ld	a, i			; Get interrupt state in parity bit
	push	af
	pop	hl
	ld	(iy + XCPT_I), hl	; Index 0: I w/interrupt state in parity/overflow

	; Save BC at offset 1

	ld	(iy + XCPT_BC), bc	; Index 1: BC

	; DE is not preserved (Index 2)

	; Save IX at offset 3

	ld	hl, (ix + FRAME_IX)	; HL = Saved value of IX
	ld	(iy + XCPT_IX), hl	; Index 3: IX

	; Save IY at index 4

	ld	hl, (ix + FRAME_IY)	; HL = Saved value of IY
	ld	(iy + XCPT_IY), hl	; Index 4: IY

	; Save that stack pointer as it would be upon return in offset 5

	ld	hl, SP_OFFSET		; Value of stack pointer on return
	add	hl, sp
	ld	(iy + XCPT_SP), hl	; Index 5 SP

	; HL is saved as the value 1 at offset 6

	ld	hl, 1
	ld	(iy + XCPT_HL), hl	; Index 2: HL on return (=1)

	; AF is not preserved (offset 7)

	; Save the return address at index 8

	ld	hl, (ix + FRAME_RET)	; HL = Saved return address
	ld	(iy + XCPT_PC), hl	; Index 8: PC

	; Return the value 0

	ld	hl, 0

	pop	iy
	pop	ix
	ret
	end
