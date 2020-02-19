;**************************************************************************
; arch/z80/src/ez80/ez80f92_init.asm
;
;   Copyright (C) 2008, 2020 Gregory Nutt. All rights reserved.
;   Author: Gregory Nutt <gnutt@nuttx.org>
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

;**************************************************************************
; Included Files
;**************************************************************************

	include "ez80F92.inc"

;**************************************************************************
; Global symbols used
;**************************************************************************

; Exported symbols
	xdef	_ez80_init
	xdef	_ez80_initsysclk

; Imported symbols
	xref	__CS0_LBR_INIT_PARAM
	xref	__CS0_UBR_INIT_PARAM
	xref	__CS0_CTL_INIT_PARAM
	xref	__CS1_LBR_INIT_PARAM
	xref	__CS1_UBR_INIT_PARAM
	xref	__CS1_CTL_INIT_PARAM
	xref	__CS2_LBR_INIT_PARAM
	xref	__CS2_UBR_INIT_PARAM
	xref	__CS2_CTL_INIT_PARAM
	xref	__CS3_LBR_INIT_PARAM
	xref	__CS3_UBR_INIT_PARAM
	xref	__CS3_CTL_INIT_PARAM
	xref	__CS0_BMC_INIT_PARAM
	xref	__CS1_BMC_INIT_PARAM
	xref	__CS2_BMC_INIT_PARAM
	xref	__CS3_BMC_INIT_PARAM
	xref	__FLASH_CTL_INIT_PARAM
	xref	__FLASH_ADDR_U_INIT_PARAM
	xref	__RAM_CTL_INIT_PARAM
	xref	__RAM_ADDR_U_INIT_PARAM

;**************************************************************************
; Chip-specific initialization logic
;**************************************************************************

; Minimum default initialization for eZ80F91

	define	.STARTUP, space = ROM
	segment	.STARTUP
	.assume	ADL = 1

_ez80_init:
	; Disable internal peripheral interrupt sources

	ld		a, %ff
	out0	(PB_DDR), a			; GPIO
	out0	(PC_DDR), a
	out0	(PD_DDR), a

	ld		a, %00
	out0	(PB_ALT1), a
	out0	(PC_ALT1), a
	out0	(PD_ALT1), a

	out0	(PB_ALT2), a
	out0	(PC_ALT2), a
	out0	(PD_ALT2), a

	out0	(UART0_IER), a		; UARTs
	out0	(UART1_IER), a

	out0	(I2C_CTL), a		; I2C

	out0	(FLASH_IRQ), a		; Flash

	ld		a, %04
	out0	(SPI_CTL), a		; SPI

	in0		a, (RTC_CTRL)		; RTC,
	and		a, %be
	out0	(RTC_CTRL), a

	; Configure external memory/io

	ld		a, __CS0_LBR_INIT_PARAM
	out0	(CS0_LBR), a
	ld		a, __CS0_UBR_INIT_PARAM
	out0	(CS0_UBR), a
	ld		a, __CS0_BMC_INIT_PARAM
	out0	(CS0_BMC), a
	ld		a, __CS0_CTL_INIT_PARAM
	out0	(CS0_CTL), a

	ld		a, __CS1_LBR_INIT_PARAM
	out0	(CS1_LBR), a
	ld		a, __CS1_UBR_INIT_PARAM
	out0	(CS1_UBR), a
	ld		a, __CS1_BMC_INIT_PARAM
	out0	(CS1_BMC), a
	ld		a, __CS1_CTL_INIT_PARAM
	out0	(CS1_CTL), a

	ld		a, __CS2_LBR_INIT_PARAM
	out0	(CS2_LBR), a
	ld		a, __CS2_UBR_INIT_PARAM
	out0	(CS2_UBR), a
	ld		a, __CS2_BMC_INIT_PARAM
	out0	(CS2_BMC), a
	ld		a, __CS2_CTL_INIT_PARAM
	out0	(CS2_CTL), a

	ld		a, __CS3_LBR_INIT_PARAM
	out0	(CS3_LBR), a
	ld		a, __CS3_UBR_INIT_PARAM
	out0	(CS3_UBR), a
	ld		a, __CS3_BMC_INIT_PARAM
	out0	(CS3_BMC), a
	ld		a, __CS3_CTL_INIT_PARAM
	out0	(CS3_CTL), a

	; Enable internal memory

	ld		a, __FLASH_ADDR_U_INIT_PARAM
	out0	(FLASH_ADDR_U), a
	ld		a, __FLASH_CTL_INIT_PARAM
	out0	(FLASH_CTRL), a

	ld		a, __RAM_ADDR_U_INIT_PARAM
	out0	(RAM_ADDR_U), a
	ld		a, __RAM_CTL_INIT_PARAM
	out0	(RAM_CTL), a
	ret

;*****************************************************************************
; eZ80F91 System Clock Initialization
;*****************************************************************************

_ez80_initsysclk:
	ret					; No PLL
	end
