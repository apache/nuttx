;**************************************************************************
; arch/z80/src/z180/z180_vectcommon.asm
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
;**************************************************************************

	.title	NuttX for the Z180
	.module	z180_vectcommon

;**************************************************************************
; Constants
;**************************************************************************

	; Register save area layout

	XCPT_I 	==  0		; Offset 0: Saved I w/interrupt state in carry
	XCPT_BC	==  2		; Offset 1: Saved BC register
	XCPT_DE	==  4		; Offset 2: Saved DE register
	XCPT_IX	==  6		; Offset 3: Saved IX register
	XCPT_IY	==  8		; Offset 4: Saved IY register
	XCPT_SP	== 10		; Offset 5: Offset to SP at time of interrupt
	XCPT_HL	== 12		; Offset 6: Saved HL register
	XCPT_AF	== 14		; Offset 7: Saved AF register
	XCPT_PC	== 16		; Offset 8: Offset to PC at time of interrupt

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_z80_doirq			; Interrupt decoding logic

;**************************************************************************
; Vector Handlers
;**************************************************************************

	.area _CODE

_up_int1::					; Vector offset 0: External /INT1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #9			; 9 = Z180_INT1
	jr		_up_vectcommon	; Remaining RST handling is common

_up_int2::					; Vector offset 2: External /INT2
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #10			; 10 = Z180_INT2
	jr		_up_vectcommon	; Remaining RST handling is common

_up_prt0::					; Vector offset 4: PRT channel 0
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #11			; 11 = Z180_PRT0
	jr		_up_vectcommon	; Remaining RST handling is common

_up_prt1::					; Vector offset 6: PRT channel 1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #12			; 12 = Z180_PRT1
	jr		_up_vectcommon	; Remaining RST handling is common

_up_dma0::					; Vector offset 8: DMA channel 0
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #13			; 13 = Z180_DMA0
	jr		_up_vectcommon	; Remaining RST handling is common

_up_dma1::					; Vector offset 8: DMA channel 1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #14			; 14 = Z180_DMA1
	jr		_up_vectcommon	; Remaining RST handling is common

_up_csio::					; Vector offset 12: Clocked serial I/O
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #15			; 15 = Z180_CSIO
	jr		_up_vectcommon	; Remaining RST handling is common

_up_asci0::					; Vector offset 14: Async channel 0
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #16			; 16 = Z180_ASCI0
	jr		_up_vectcommon	; Remaining RST handling is common

_up_asci1::					; Vector offset 16: Async channel 1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #17			; 17 = Z180_ASCI1
	jr		_up_vectcommon	; Remaining RST handling is common

_up_unused::				; Vector offset 18: Unused
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #18			; 18 = Z180_UNUSED
	jr		_up_vectcommon	; Remaining RST handling is common

;**************************************************************************
; Common Interrupt handler
;**************************************************************************

_up_vectcommon::
	; Create a register frame.  SP points to top of frame + 4, pushes
	; decrement the stack pointer.  Already have
	;
	;   Offset 8: Return PC is already on the stack
	;   Offset 7: AF (retaining flags)
	;
	; IRQ number is in A

	push	hl				; Offset 6: HL
	ld		hl, #(3*2)		;    HL is the value of the stack pointer before
	add		hl, sp			;    the interrupt occurred
	push	hl				; Offset 5: Stack pointer
	push	iy				; Offset 4: IY
	push	ix				; Offset 3: IX
	push	de				; Offset 2: DE
	push	bc				; Offset 1: BC

	ld		b, a			;   Save the reset number in B
	ld		a, i			;   Parity bit holds interrupt state
	push	af				; Offset 0: I with interrupt state in parity
	di

	; Call the interrupt decode logic. SP points to the beginning of the reg structure

	ld		hl, #0			; Argument #2 is the beginning of the reg structure
	add		hl, sp			;
	push	hl				; Place argument #2 at the top of stack
	push	bc				; Argument #1 is the Reset number
	inc		sp				; (make byte sized)
	call	_z80_doirq		; Decode the IRQ

	; On return, HL points to the beginning of the reg structure to restore
	; Note that (1) the arguments pushed on the stack are not popped, and (2) the
	; original stack pointer is lost.  In the normal case (no context switch),
	; HL will contain the value of the SP before the arguments were pushed.

	ld		sp, hl			; Use the new stack pointer

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex		af, af'			; Select alternate AF
	pop		af				; Offset 0: AF' = I with interrupt state in carry
	ex		af, af'			;   Restore original AF
	pop		bc				; Offset 1: BC
	pop		de				; Offset 2: DE
	pop		ix				; Offset 3: IX
	pop		iy				; Offset 4: IY
	exx						;   Use alternate BC/DE/HL
	ld		hl, #-2			;   Offset of SP to account for ret addr on stack
	pop		de				; Offset 5: HL' = Stack pointer after return
	add		hl, de			;   HL = Stack pointer value before return
	exx						;   Restore original BC/DE/HL
	pop		hl				; Offset 6: HL
	pop		af				; Offset 7: AF

	; Restore the stack pointer

	exx						; Use alternate BC/DE/HL
	ld		sp, hl			; Set SP = saved stack pointer value before return
	exx						; Restore original BC/DE/HL

	; Restore interrupt state

	ex		af, af'			; Recover interrupt state
	jp		po, nointenable	; Odd parity, IFF2=0, means disabled
	ex		af, af'			; Restore AF (before enabling interrupts)
	ei						; yes
	reti
nointenable::
	ex		af, af'			; Restore AF
	reti
