;**************************************************************************
; arch/z80/src/z180/z180_head.asm
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
	.module	z180_head

;**************************************************************************
; Constants
;**************************************************************************

	; Default stack base (needs to be fixed)

	.include	"asm_mem.h"

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_nx_start			; OS entry point
	.globl	_up_vectcommon		; Common interrupt handling logic
	.globl	_z180_mmu_lowinit	; MMU initialization logic
	.globl	s__HEAP				; Start of the heap

;**************************************************************************
; Reset entry point
;**************************************************************************

	.area	_HEADER	(ABS)
	.org	0x0000

	di						; Disable interrupts
	im	1					; Set interrupt mode 1
	jr	_up_reset			; And boot the system

;**************************************************************************
; Other reset handlers
;**************************************************************************

	.org	0x0008			; RST 1

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #0			; 0 = Z180_RST1
	jp		_up_vectcommon	; Remaining RST handling is common

	.org	0x0010			; RST 2

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #1			; 1 = Z180_RST2
	jp		_up_vectcommon	; Remaining RST handling is common

	.org	0x0018			; RST 3

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #2			; 2 = Z180_RST3
	jp		_up_vectcommon	; Remaining RST handling is common

	.org	0x0020			; RST 4

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #3			; 3 = Z180_RST4
	jp		_up_vectcommon	; Remaining RST handling is common

	.org	0x0028			; RST 5

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #4			; 4 = Z180_RST5
	jp		_up_vectcommon	; Remaining RST handling is common

	.org	0x0030			; RST 6

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #5			; 5 = Z180_RST6
	jp		_up_vectcommon	; Remaining RST handling is common

	.org   0x0038			; Int mode 1 / RST 7

	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #6			; 6 = Z180_RST7
	jp		_up_vectcommon	; Remaining RST handling is common

;**************************************************************************
; NMI interrupt handler
;**************************************************************************

	.org	0x0066
	retn

;**************************************************************************
; System start logic
;**************************************************************************

_up_reset:
	; Set up the stack pointer at the location determined the Makefile
	; and stored in asm_mem.h

	ld		SP, #CONFIG_STACK_END	; Set stack pointer

	; Configure the MMU so that things will lie at the addresses that we
	; expect them to

	call	_z180_mmu_lowinit	; Initialize the MMU

	; Performed initialization unique to the SDCC toolchain

	call	gsinit			; Initialize the data section

	; Then start NuttX

	call	_nx_start		; jump to the OS entry point

	; NuttX will never return, but just in case...

_up_halt::
	halt					; We should never get here
	jp		_up_halt

;**************************************************************************
; Ordering of segments for the linker (SDCC only)
;**************************************************************************

	.area	_HOME
	.area	_CODE
	.area	_INITIALIZER
	.area	_GSINIT
	.area	_GSFINAL

	.area	_DATA
	.area	_INITIALIZED
	.area	_BSEG
	.area	_BSS
	.area	_HEAP

;**************************************************************************
; Global data initialization logic (SDCC only)
;**************************************************************************

	.area	_GSINIT
gsinit::
	ld		bc, #l__INITIALIZER
	ld		a, b
	or		a, c
	jr		Z, gsinit_next
	ld		de, #s__INITIALIZED
	ld		hl, #s__INITIALIZER
	ldir
gsinit_next:

	.area   _GSFINAL
	ret

;**************************************************************************
; The start of the heap (SDCC only).  Note that is actually resides in
; the _CODE area (which may be FLASH or ROM)
;**************************************************************************

	.area	_CODE
_g_heapbase::
	.dw		#s__HEAP
