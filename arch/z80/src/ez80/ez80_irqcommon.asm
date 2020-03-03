;**************************************************************************
; arch/z80/src/ez80/ez80_irqcommon.asm
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

;**************************************************************************
; Constants
;**************************************************************************

; Bits in the Z80 FLAGS register ******************************************

EZ80_C_FLAG		EQU	01h		; Bit 0: Carry flag
EZ80_N_FLAG		EQU	02h		; Bit 1: Add/Subtract flag
EZ80_PV_FLAG	EQU	04h		; Bit 2: Parity/Overflow flag
EZ80_H_FLAG		EQU	10h		; Bit 4: Half carry flag
EZ80_Z_FLAG		EQU	40h		; Bit 5: Zero flag
EZ80_S_FLAG		EQU	80h		; Bit 7: Sign flag

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

	xref	_z80_doirq

;**************************************************************************
; Global Symbols Exported
;**************************************************************************

	xdef	_ez80_irq_common

;**************************************************************************
; Common Interrupt handler
;**************************************************************************

	define .STARTUP, space = ROM
	segment .STARTUP
	.assume ADL=1

_ez80_irq_common:

	; Create a register frame.  SP points to top of frame + 4, pushes
	; decrement the stack pointer.  Already have
	;
	;   Offset 8: Return PC is already on the stack
	;   Offset 7: AF (retaining flags)
	;
	; IRQ number is in A

	push	hl					; Offset 6: HL
	ld		hl, #(3*3)			;    HL is the value of the stack pointer before
	add		hl, sp				;    the interrupt occurred (3 for PC, AF, HL)
	push	hl					; Offset 5: Stack pointer
	push	iy					; Offset 4: IY
	push	ix					; Offset 3: IX
	push	de					; Offset 2: DE
	push	bc					; Offset 1: BC

	; At this point, we know that interrupts were enabled (or we wouldn't be here)
	; so we can save a fake indication that will cause interrupts to restored when
	; this context is restored

	ld		bc, #EZ80_PV_FLAG	; Parity bit.  1=parity odd, IEF2=1
	push	bc					; Offset 0: I with interrupt state in parity
	di							; (not necessary)

	; Call the interrupt decode logic. SP points to the beginning of the reg structure

	ld		hl, #0				; Argument #2 is the beginning of the reg structure
	add		hl, sp				;
	push	hl					; Place argument #2 at the top of stack
	ld		bc, #0				; BC = reset number
	ld		c, a				;   Save the reset number in C
	push	bc					; Argument #1 is the Reset number
	call	_z80_doirq			; Decode the IRQ

	; On return, HL points to the beginning of the reg structure to restore
	; Note that (1) the arguments pushed on the stack are not popped, and (2) the
	; original stack pointer is lost.  In the normal case (no context switch),
	; HL will contain the value of the SP before the arguments were pushed.

	ld		sp, hl				; Use the new stack pointer

	; Restore registers.  HL points to the beginning of the reg structure to restore

	ex		af, af'				; Select alternate AF
	pop		af					; Offset 0: AF' = I with interrupt state in parity
	ex		af, af'				;   Restore original AF
	pop		bc					; Offset 1: BC
	pop		de					; Offset 2: DE
	pop		ix					; Offset 3: IX
	pop		iy					; Offset 4: IY
	exx							;   Use alternate BC/DE/HL
	pop		hl					; Offset 5: HL' = Stack pointer after return
	exx							;   Restore original BC/DE/HL
	pop		hl					; Offset 6: HL
	pop		af					; Offset 7: AF

	; Restore the stack pointer

	exx							; Use alternate BC/DE/HL
	pop		de					; Offset 8: Return address
	ld		sp, hl				; Set SP = saved stack pointer value before return
	push	de					; Set up for reti
	exx							; Restore original BC/DE/HL

	; Restore interrupt state

	ex		af, af'				; Recover interrupt state
	jp		po, nointenable		; Odd parity, IFF2=0, means disabled
	ex		af, af'				; Restore AF (before enabling interrupts)
	ei							; yes
	reti
nointenable:
	ex		af, af'				; Restore AF
	reti
	end
