;**************************************************************************
; arch/z80/src/common/up_head.S
;
;   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
; 3. Neither the name NuttX nor the names of its contributors may be
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
;**************************************************************************

	.title	NuttX for the Z80
	.module	z80_head

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

	; Default stack base (needs to be fixed)

	.include	"asm_mem.h"

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_os_start		; OS entry point
	.globl	_up_decodeirq		; Interrupt decoding logic

;**************************************************************************
; Reset entry point
;**************************************************************************

	.area	START	(ABS)
	.org	0x0000

	.globl	_os_start
	di				; Disable interrupts
	ld	SP, #UP_STACK_END	; Set stack pointer
	im	1			; Set interrupt mode 1
	jp	_os_start		; jump to the OS entry point
forever:
	jp	forever

;**************************************************************************
; Interrupt handler
;
; Interrupt mode 1 behavior:
; 
; 1. M1 cycle: 7 ticks
;    Acknowledge interrupt and decrements SP
; 2. M2 cycle: 3 ticks
;    Writes the MS byte of the PC onto the stack and decrements SP
; 3. M3 cycle: 3 ticks
;    Writes the LS byte of the PC onto the stack and sets the PC to 0x0038.
;
;**************************************************************************

	.org   0x0038			; Int mode 1

	; Create a register frame.  SP points to top of frame + 2, pushes
	; decrement the stack pointer.
					; Offset 8: Return PC is already on the stack
	push	af			; Offset 7: AF (retaining flags)
	push	hl			; Offset 6: HL
	ld	hl, #(3*2)		;    HL is the value of the stack pointer before
	add	hl, sp			;    the interrupt occurred
	push	hl			; Offset 5: Stack pointer
	push	iy			; Offset 4: IY
	push	ix			; Offset 3: IX
	push	de			; Offset 2: DE
	push	bc			; Offset 1: BC

	ld	a, i			;   Carry bit holds interrupt state
	push	af			; Offset 0: I with interrupt state in carry
	di

	; Call the interrupt decode logic. SP points to the beggining of the reg structure

	ld	hl, #0			; Argument is the beginning of the reg structure
	add	hl, sp			;
	push	hl			; Place argument at the top of thest
	call	_up_decodeirq		; Decode the IRQ

	; On return, HL points to the beginning of the reg structure to restore
	; Note that (1) the argument pushed on the stack is not popped, and (2) the
	; original stack pointer is lost.  In the normal case (no context switch),
	; HL will contain the value of the SP before the argument was pushed.

	ld	sp, hl			; Use the new stack pointer

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex	af, af'			; Select alternate AF
	pop	af			; Offset 0: AF' = I with interrupt state in carry
	pop	bc			; Offset 1: BC
	pop	de			; Offset 2: DE
	pop	ix			; Offset 3: IX
	pop	iy			; Offset 4: IY
	exx				;   Use alternate BC/DE/HL
	pop	hl			; Offset 5: HL' = Stack pointer at time of interrupt
	exx
	pop	hl			; Offset 6: HL
	pop	af			; Offset 7: AF

	; Restore the stack pointer

	exx
	ld	sp, hl
	exx

	; Restore interrupt state

	ex	af, af'			; Recover interrupt state
	jr	nc, nointenable		; No carry, IFF2=0, means disabled
	ex	af, af'			; Restore AF (before enabling interrupts)
	ei				; yes
	reti
nointenable:
	ex	af, af'			; Restore AF
	reti

;**************************************************************************
; NMI interrupt handler
;**************************************************************************

	.org   0x0066
	retn


