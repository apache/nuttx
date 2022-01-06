;**************************************************************************
; arch/z80/src/ez80/ez80_startup.asm
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

EZ80_RAM_CTL      EQU 0b4h
EZ80_RAM_ADDR_U   EQU 0b5h

EZ80_FLASH_ADDR_U EQU 0f7h
EZ80_FLASH_CTRL   EQU 0f8h

;**************************************************************************
; Imported Symbols
;**************************************************************************

	xref	_ez80_init
	xref	_ez80_initvectors
	xref	_ez80_initsysclk
	xref	_ez80_board_initialize
	xref	__low_bss				; Low address of bss segment
	xref	__len_bss				; Length of bss segment

	xref	__low_data				; Address of initialized data section
	xref	__low_romdata			; Addr of initialized data section in ROM
	xref	__len_data				; Length of initialized data section

	xref	__copy_code_to_ram
	xref	__len_code
	xref	__low_code
	xref	__low_romcode

	xref	__RAM_ADDR_U_INIT_PARAM
	xref	__RAM_CTL_INIT_PARAM
	xref	__FLASH_ADDR_U_INIT_PARAM
	xref	__FLASH_CTL_INIT_PARAM

	xref	_nx_start
	xdef	_ez80_startup
	xdef	_ez80_halt

;**************************************************************************
; Code
;**************************************************************************

	segment	CODE
	.assume ADL=1

;**************************************************************************
; System reset start logic
;**************************************************************************

_ez80_startup:
	; Enable internal memory using settings from the linkcmd file

	ld		a, __FLASH_ADDR_U_INIT_PARAM
	out0	(EZ80_FLASH_ADDR_U), a
	ld		a, __FLASH_CTL_INIT_PARAM
	out0	(EZ80_FLASH_CTRL), a

	ld		a, __RAM_ADDR_U_INIT_PARAM
	out0	(EZ80_RAM_ADDR_U), a
	ld		a, __RAM_CTL_INIT_PARAM
	out0	(EZ80_RAM_CTL), a

	; Position the IDLE task stack point at an offset of 1Kb in on-chip SRAM
	; On-chip SRAM resides at an offset of 000e000h from the RAM base address.
	; REVISIT:  CONFIG_IDLETHREAD_STACKSIZE is not used!

	; The GNU assembler (2.36.1) cannot produce this relocation, although the
	; Z80 ELF format supports it. The instruction is instead hand assembled.
	;ld		sp, __RAM_ADDR_U_INIT_PARAM << 16 + 000e400h
	db		031h, 000h, 0e4h, __RAM_ADDR_U_INIT_PARAM

	; Perform chip-specific initialization

	call	_ez80_init

	; initialize the interrupt vector table

	call 	_ez80_initvectors

	; Initialize the system clock

	call	_ez80_initsysclk

	; Perform C initializations
	; Clear the uninitialized data section

	ld		bc, __len_bss		; Check for non-zero length
	ld		a, __len_bss >> 16
	or		a, c
	or		a, b
	jr		z, _ez80_bssdone	; BSS is zero-length ...
	xor		a, a
	ld		(__low_bss), a
	sbc		hl, hl				; hl = 0
	dec		bc					; 1st byte's taken care of
	sbc		hl, bc
	jr		z, _ez80_bssdone	; Just 1 byte ...
	ld		hl, __low_bss		; reset hl
	ld		de, __low_bss + 1	; [de] = bss + 1
	ldir
_ez80_bssdone:

	; Copy the initialized data section

	ld		bc, __len_data		; [bc] = data length
	ld		a, __len_data >> 16	; Check for non-zero length
	or		a, c
	or		a, b
	jr		z, _ez80_datadone	; __len_data is zero-length ...
	ld		hl, __low_romdata	; [hl] = data_copy
	ld		de, __low_data		; [de] = data
	ldir						; Copy the data section
_ez80_datadone:

	; Copy CODE (which may be in FLASH) to RAM if the
	; copy_code_to_ram symbol is set in the link control file

	ld		a, __copy_code_to_ram
	or		a, a
	jr		z, _ez80_codedone
	ld		bc, __len_code		; [bc] = code length
	ld		a, __len_code >> 16	; Check for non-zero length
	or		a, c
	or		a, b
	jr		z, _ez80_codedone	; __len_code is zero-length
	ld		hl, __low_romcode	; [hl] = code_copy
	ld		de, __low_code		; [de] = code
	ldir						; Copy the code section
_ez80_codedone:

	; Perform board-specific initialization

	call	_ez80_board_initialize

	; Then start NuttX

	call	_nx_start			; jump to the OS entry point

	; NuttX will never return, but just in case...

_ez80_halt:
	halt						; We should never get here
	jp		_ez80_halt
