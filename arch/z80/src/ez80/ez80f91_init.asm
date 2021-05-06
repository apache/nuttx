;**************************************************************************
; arch/z80/src/ez80/ez80f91_init.asm
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

	include "ez80F91.inc"

;**************************************************************************
; Constants
;**************************************************************************

;PLL_DIV_L    EQU 05Ch
;PLL_DIV_H    EQU 05Dh
;PLL_CTL0     EQU 05Eh
;PLL_CTL1     EQU 05Fh

OSC           EQU 0
PLL           EQU 1
RTC           EQU 2

CLK_MUX_OSC   EQU 000h
CLK_MUX_PLL   EQU 001h
CLK_MUX_RTC   EQU 002h

CHRP_CTL_0    EQU 000h
CHRP_CTL_1    EQU 040h
CHRP_CTL_2    EQU 080h
CHRP_CTL_3    EQU 0C0h

LDS_CTL_0     EQU 000h
LDS_CTL_1     EQU 004h
LDS_CTL_2     EQU 008h
LDS_CTL_3     EQU 00Ch

LCK_STATUS    EQU 020h
INT_LOCK      EQU 010h
INT_UNLOCK    EQU 008h
INT_LOCK_EN   EQU 004h
INT_UNLOCK_EN EQU 002h
PLL_ENABLE    EQU 001h

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
	xref	_SYS_CLK_SRC
	xref	_SYS_CLK_FREQ
	xref	_OSC_FREQ
	xref	_OSC_FREQ_MULT
	xref	__PLL_CTL0_INIT_PARAM

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
	out0	(PA_DDR), a			; GPIO
	out0	(PB_DDR), a
	out0	(PC_DDR), a
	out0	(PD_DDR), a
	ld		a, 000h
	out0	(PA_ALT1), a
	out0	(PB_ALT1), a
	out0	(PC_ALT1), a
	out0	(PD_ALT1), a
	out0	(PA_ALT2), a
	out0	(PB_ALT2), a
	out0	(PC_ALT2), a
	out0	(PD_ALT2), a
	out0	(PLL_CTL1), a		; PLL
	out0	(TMR0_IER), a		; timers
	out0	(TMR1_IER), a
	out0	(TMR2_IER), a
	out0	(TMR3_IER), a
	out0	(UART0_IER), a		; UARTs
	out0	(UART1_IER), a
	out0	(I2C_CTL), a		; I2C
	out0	(EMAC_IEN), a		; EMAC
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
	; check if the PLL should be used

	ld		a, (_ez80_sysclksrc)
	cp		a, PLL
	jr		nz, _ez80_initsysclkdone

	; Load PLL divider

	ld		a, (_ez80_oscfreqmult)		;CR 6202
	out0	(PLL_DIV_L), a
	ld		a, (_ez80_oscfreqmult+1)
	out0	(PLL_DIV_H), a

	; Set charge pump and lock criteria

	ld		a, __PLL_CTL0_INIT_PARAM
	and		a, 0CCh  ; mask off reserved and clock source bits
	out0	(PLL_CTL0), a

	; Enable PLL

	in0		a, (PLL_CTL1)
	set		0, a
	out0	(PLL_CTL1), a

	; Wait for PLL to lock

_ez80_initsysclkwait:
	in0		a, (PLL_CTL1)
	and		a, LCK_STATUS
	cp		a, LCK_STATUS
	jr		nz, _ez80_initsysclkwait

	; Select PLL as system clock source

	ld		a, __PLL_CTL0_INIT_PARAM
	set		0, a
	out0	(PLL_CTL0), a

_ez80_initsysclkdone:
	ret

;_ez80_oscfreq:
;	dl		_OSC_FREQ
_ez80_oscfreqmult:
	dw		_OSC_FREQ_MULT
;_ez80_sysclkfreq:
;	dl		_SYS_CLK_FREQ
_ez80_sysclksrc:
	db		_SYS_CLK_SRC
	end
