;**************************************************************************
; arch/z80/src/z180/z180_vectors.asm
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
	.module	z180_vectors

;**************************************************************************
; Constants
;**************************************************************************

;**************************************************************************
; Global symbols used
;**************************************************************************

	.globl	_up_int1		; Vector offset 0: External /INT1
	.globl	_up_int2		; Vector offset 2: External /INT2
	.globl	_up_prt0		; Vector offset 4: PRT channel 0
	.globl	_up_prt1		; Vector offset 6: PRT channel 1
	.globl	_up_dma0		; Vector offset 8: DMA channel 0
	.globl	_up_dma1		; Vector offset 8: DMA channel 1
	.globl	_up_csio		; Vector offset 12: Clocked serial I/O
	.globl	_up_asci0		; Vector offset 14: Async channel 0
	.globl	_up_asci1		; Vector offset 16: Async channel 1
	.globl	_up_unused		; Vector offset 18: Unused

;**************************************************************************
; Interrupt Vector Table
;**************************************************************************

; The Vector Table is located at address 0x0040, between the RST vectors
; and the NMI vector

	.area	_VECTORS (ABS)
	.org	0x0040

_up_vectors::
	.dw		_up_int1		; Vector offset 0: External /INT1
	.dw		_up_int2		; Vector offset 2: External /INT2
	.dw		_up_prt0		; Vector offset 4: PRT channel 0
	.dw		_up_prt1		; Vector offset 6: PRT channel 1
	.dw		_up_dma0		; Vector offset 8: DMA channel 0
	.dw		_up_dma1		; Vector offset 8: DMA channel 1
	.dw		_up_csio		; Vector offset 12: Clocked serial I/O
	.dw		_up_asci0		; Vector offset 14: Async channel 0
	.dw		_up_asci1		; Vector offset 16: Async channel 1
	.dw		_up_unused		; Vector offset 18: Unused
	.dw		_up_unused		; Vector offset 20: Unused
	.dw		_up_unused		; Vector offset 22: Unused
	.dw		_up_unused		; Vector offset 24: Unused
	.dw		_up_unused		; Vector offset 26: Unused
	.dw		_up_unused		; Vector offset 28: Unused
	.dw		_up_unused		; Vector offset 30: Unused
