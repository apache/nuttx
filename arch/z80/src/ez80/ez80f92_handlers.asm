;**************************************************************************
; arch/z80/src/ez80/ez80f92_handlers.asm
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

; The IRQ number to use for unused vectors

EZ80_UNUSED		EQU	40h

;**************************************************************************
; Global Symbols Imported
;**************************************************************************

	xref	_ez80_irq_common

;**************************************************************************
; Global Symbols Exported
;**************************************************************************

	xdef	_ez80_initvectors

;**************************************************************************
; Constants
;**************************************************************************

NVECTORS	EQU	64		; max possible interrupt vectors

;**************************************************************************
; Macros
;**************************************************************************

; Define one interrupt handler

irqhandler: macro vectno
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
								; Offset 8: Return PC is already on the stack
	push	af					; Offset 7: AF (retaining flags)
	ld		a, #vectno			; A = vector number
	jp		_ez80_irq_common	; Remaining RST handling is common
	endmac	irqhandler

;**************************************************************************
; Vector Table
;**************************************************************************

; This segment must be aligned on a 256 byte boundary anywhere in RAM
; Each entry will be a 2-byte address in a 2-byte space

	define	.IVECTS, space = RAM, align = 100h
	segment	.IVECTS

; Vector table is a 2-bit address.  The MSB is the I register; the LSB is
; the vector number.  The vector table lies in FLASH.  The addresses
; contained in the refers to an entry in the handler table that re-
; directs the interrupt to common interrupt handling logic.

_ez80_vectable:
	dw	_ez80_handlers + 0*_handlersize
	dw	_ez80_handlers + 1*_handlersize
	dw	_ez80_handlers + 2*_handlersize
	dw	_ez80_handlers + 3*_handlersize
	dw	_ez80_handlers + 4*_handlersize
	dw	_ez80_handlers + 5*_handlersize
	dw	_ez80_handlers + 6*_handlersize
	dw	_ez80_handlers + 7*_handlersize
	dw	_ez80_handlers + 8*_handlersize
	dw	_ez80_handlers + 9*_handlersize
	dw	_ez80_handlers + 10*_handlersize
	dw	_ez80_handlers + 11*_handlersize
	dw	_ez80_handlers + 12*_handlersize
	dw	_ez80_handlers + 13*_handlersize
	dw	_ez80_handlers + 14*_handlersize
	dw	_ez80_handlers + 15*_handlersize
	dw	_ez80_handlers + 16*_handlersize
	dw	_ez80_handlers + 17*_handlersize
	dw	_ez80_handlers + 18*_handlersize
	dw	_ez80_handlers + 19*_handlersize
	dw	_ez80_handlers + 20*_handlersize
	dw	_ez80_handlers + 21*_handlersize
	dw	_ez80_handlers + 22*_handlersize
	dw	_ez80_handlers + 23*_handlersize
	dw	_ez80_handlers + 24*_handlersize
	dw	_ez80_handlers + 25*_handlersize
	dw	_ez80_handlers + 26*_handlersize
	dw	_ez80_handlers + 27*_handlersize
	dw	_ez80_handlers + 28*_handlersize
	dw	_ez80_handlers + 29*_handlersize
	dw	_ez80_handlers + 30*_handlersize
	dw	_ez80_handlers + 31*_handlersize
	dw	_ez80_handlers + 32*_handlersize
	dw	_ez80_handlers + 33*_handlersize
	dw	_ez80_handlers + 34*_handlersize
	dw	_ez80_handlers + 35*_handlersize
	dw	_ez80_handlers + 36*_handlersize
	dw	_ez80_handlers + 37*_handlersize
	dw	_ez80_handlers + 38*_handlersize
	dw	_ez80_handlers + 39*_handlersize
	dw	_ez80_handlers + 40*_handlersize
	dw	_ez80_handlers + 41*_handlersize
	dw	_ez80_handlers + 42*_handlersize
	dw	_ez80_handlers + 43*_handlersize
	dw	_ez80_handlers + 44*_handlersize
	dw	_ez80_handlers + 45*_handlersize
	dw	_ez80_handlers + 46*_handlersize
	dw	_ez80_handlers + 47*_handlersize
	dw	_ez80_handlers + 48*_handlersize
	dw	_ez80_handlers + 49*_handlersize
	dw	_ez80_handlers + 50*_handlersize
	dw	_ez80_handlers + 51*_handlersize
	dw	_ez80_handlers + 52*_handlersize
	dw	_ez80_handlers + 53*_handlersize
	dw	_ez80_handlers + 54*_handlersize
	dw	_ez80_handlers + 55*_handlersize
	dw	_ez80_handlers + 56*_handlersize
	dw	_ez80_handlers + 57*_handlersize
	dw	_ez80_handlers + 58*_handlersize
	dw	_ez80_handlers + 59*_handlersize
	dw	_ez80_handlers + 60*_handlersize
	dw	_ez80_handlers + 61*_handlersize
	dw	_ez80_handlers + 62*_handlersize
	dw	_ez80_handlers + 63*_handlersize

;**************************************************************************
; Interrupt Vector Handlers
;**************************************************************************

; Still in .IVECTS section

	.assume ADL=1

						; Symbol           Val VecNo Addr
						;----------------- --- ----- -----
_ez80_handlers:
	irqhandler	EZ80_UNUSED		;                0   0x000
	_handlersize EQU $-_ez80_handlers
	irqhandler	EZ80_UNUSED+1	;                1   0x002
	irqhandler	EZ80_UNUSED+2	;                2   0x004
	irqhandler	EZ80_UNUSED+3	;                3   0x006
	irqhandler	 0		; EZ80_FLASH_IRQ    0    4   0x008
	irqhandler	 1		; EZ80_TIMER0_IRQ   1    5   0x00a
	irqhandler	 2		; EZ80_TIMER1_IRQ   2    6   0x00c
	irqhandler	 3		; EZ80_TIMER2_IRQ   3    7   0x00e
	irqhandler	 4		; EZ80_TIMER3_IRQ   4    8   0x010
	irqhandler	 5		; EZ80_TIMER4_IRQ   5    9   0x012
	irqhandler	 6		; EZ80_TIMER5_IRQ   6   10   0x014
	irqhandler	 7		; EZ80_RTC_IRQ      7   11   0x016
	irqhandler	 8		; EZ80_UART0_IRQ    8   12   0x018
	irqhandler	 9		; EZ80_UART1_IRQ    9   13   0x01a
	irqhandler	10		; EZ80_I2C_IRQ     10   14   0x01c
	irqhandler	11		; EZ80_SPI_IRQ     11   15   0x01e
	irqhandler	EZ80_UNUSED+4	;               16   0x020
	irqhandler	EZ80_UNUSED+5	;               17   0x022
	irqhandler	EZ80_UNUSED+6	;               18   0x024
	irqhandler	EZ80_UNUSED+7	;               19   0x026
	irqhandler	EZ80_UNUSED+8	;               16   0x028
	irqhandler	EZ80_UNUSED+9	;               17   0x02a
	irqhandler	EZ80_UNUSED+10	;               18   0x02c
	irqhandler	EZ80_UNUSED+11	;               19   0x02e
	irqhandler	12		; EZ80_PORTB0_IRQ  12   24   0x030
	irqhandler	13		; EZ80_PORTB1_IRQ  13   25   0x032
	irqhandler	14		; EZ80_PORTB2_IRQ  14   26   0x034
	irqhandler	15		; EZ80_PORTB3_IRQ  15   27   0x036
	irqhandler	16		; EZ80_PORTB4_IRQ  16   28   0x038
	irqhandler	17		; EZ80_PORTB5_IRQ  17   29   0x03a
	irqhandler	18		; EZ80_PORTB6_IRQ  18   20   0x03c
	irqhandler	19		; EZ80_PORTB7_IRQ  19   21   0x03e
	irqhandler	20		; EZ80_PORTC0_IRQ  20   22   0x040
	irqhandler	21		; EZ80_PORTC1_IRQ  21   23   0x042
	irqhandler	22		; EZ80_PORTC2_IRQ  22   24   0x044
	irqhandler	23		; EZ80_PORTC3_IRQ  23   25   0x046
	irqhandler	24		; EZ80_PORTC4_IRQ  24   26   0x048
	irqhandler	25		; EZ80_PORTC5_IRQ  25   27   0x04a
	irqhandler	26		; EZ80_PORTC6_IRQ  26   28   0x04c
	irqhandler	27		; EZ80_PORTC7_IRQ  27   29   0x04e
	irqhandler	28		; EZ80_PORTD0_IRQ  28   40   0x050
	irqhandler	29		; EZ80_PORTD1_IRQ  29   41   0x052
	irqhandler	30		; EZ80_PORTD2_IRQ  30   42   0x054
	irqhandler	31		; EZ80_PORTD3_IRQ  31   43   0x056
	irqhandler	32		; EZ80_PORTD4_IRQ  32   44   0x058
	irqhandler	33		; EZ80_PORTD5_IRQ  33   45   0x05a
	irqhandler	34		; EZ80_PORTD6_IRQ  34   46   0x05c
	irqhandler	35		; EZ80_PORTD7_IRQ  35   47   0x05e
	irqhandler	EZ80_UNUSED+12	;               48   0x060
	irqhandler	EZ80_UNUSED+13	;               49   0x062
	irqhandler	EZ80_UNUSED+14	;               50   0x064
	irqhandler	EZ80_UNUSED+15	;               51   0x066
	irqhandler	EZ80_UNUSED+16	;               52   0x068
	irqhandler	EZ80_UNUSED+17	;               53   0x06a
	irqhandler	EZ80_UNUSED+18	;               54   0x06c
	irqhandler	EZ80_UNUSED+19	;               55   0x06e
	irqhandler	EZ80_UNUSED+20	;               56   0x070
	irqhandler	EZ80_UNUSED+21	;               57   0x072
	irqhandler	EZ80_UNUSED+22	;               58   0x074
	irqhandler	EZ80_UNUSED+23	;               59   0x076
	irqhandler	EZ80_UNUSED+24	;               60   0x078
	irqhandler	EZ80_UNUSED+25	;               61   0x07a
	irqhandler	EZ80_UNUSED+26	;               62   0x07c
	irqhandler	EZ80_UNUSED+27	;               63   0x07e

;**************************************************************************
; Vector Setup Logic
;**************************************************************************

	define .STARTUP, space = ROM
	segment .STARTUP
	.assume ADL=1

_ez80_initvectors:

	; We don't need to do much here.  The interrupt vectors and handlers
	; are all in FLASH.

	; Select interrupt mode 2

	im		2					; Interrupt mode 2

	; Write the address of the vector table into the interrupt vector base

	ld		a, _ez80_vectable >> 8 & 0ffh
	ld		i, a
	ret
	end
