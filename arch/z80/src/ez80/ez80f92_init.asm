;**************************************************************************
; arch/z80/src/ez80/ez80f92_init.asm
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

	ld		a, 0ffh
	out0	(PB_DDR), a			; GPIO
	out0	(PC_DDR), a
	out0	(PD_DDR), a

	ld		a, 000h
	out0	(PB_ALT1), a
	out0	(PC_ALT1), a
	out0	(PD_ALT1), a

	out0	(PB_ALT2), a
	out0	(PC_ALT2), a
	out0	(PD_ALT2), a

	out0	(TMR0_CTL), a		; timers
	out0	(TMR1_CTL), a
	out0	(TMR2_CTL), a
	out0	(TMR3_CTL), a
	out0	(TMR4_CTL), a
	out0	(TMR5_CTL), a

	out0	(UART0_IER), a		; UARTs
	out0	(UART1_IER), a

	out0	(I2C_CTL), a		; I2C

	out0	(FLASH_IRQ), a		; Flash

	ld		a, 004h
	out0	(SPI_CTL), a		; SPI

	in0		a, (RTC_CTRL)		; RTC,
	and		a, 0beh
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

	ret

;*****************************************************************************
; eZ80F91 System Clock Initialization
;*****************************************************************************

_ez80_initsysclk:
	ret					; No PLL
	end
