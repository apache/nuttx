;**************************************************************************
; arch/z80/src/z180/z180_rom.asm
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
; System start logic
;**************************************************************************

_up_reset:
	; Set up the stack pointer at the location determined the Makefile
	; and stored in asm_mem.h

	ld		SP, #CONFIG_STACK_END	; Set stack pointer

	; Performed initialization unique to the SDCC toolchain

	call	gsinit			; Initialize the data section

	; Copy the reset vectors

	ld		hl, #_up_rstvectors	; code for RAM
	ld		de, #0x4000		; move it here
	ld		bc, #3*7		; 7 vectors / 3 bytes each
	ldir

	; Then start NuttX

	call	_nx_start		; jump to the OS entry point

	; NuttX will never return, but just in case...

_up_halt::
	halt					; We should never get here
	jp		_up_halt

	; Data to copy to address 0x4000

_up_rstvectors:
	jp		_up_rst1		; 0x4000 : RST 1
	jp		_up_rst2		; 0x4003 : RST 2
	jp		_up_rst3		; 0x4006 : RST 3
	jp		_up_rst4		; 0x4009 : RST 4
	jp		_up_rst5		; 0x400c : RST 5
	jp		_up_rst6		; 0x400f : RST 6
	jp		_up_rst7		; 0x4012 : RST 7

;**************************************************************************
; Other reset handlers
;**************************************************************************

_up_rst1:					; RST 1
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #0			; 0 = Z180_RST1
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst2:					; RST 2
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #1			; 1 = Z180_RST2
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst3:					; RST 3
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #2			; 2 = Z180_RST3
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst4:					; RST 4
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #3			; 3 = Z180_RST4
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst5:					; RST 5
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #4			; 4 = Z180_RST5
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst6:					; RST 6
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #5			; 5 = Z180_RST6
	jp		_up_vectcommon	; Remaining RST handling is common

_up_rst7:					; RST 7
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af				; Offset 7: AF (retaining flags)
	ld		a, #6			; 6 = Z180_RST7
	jp		_up_vectcommon	; Remaining RST handling is common

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
