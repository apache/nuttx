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
; Global Symbols Imported
;**************************************************************************

	xref	__vecstart
	xref	__vecend
	xref	_ez80_irq_common

;**************************************************************************
; Global Symbols Exported
;**************************************************************************

	xdef	_ez80_initvectors

;**************************************************************************
; Constants
;**************************************************************************

NVECTORS	EQU	64		; Max possible interrupt vectors
EZ80_UNUSED	EQU	64		; Denotes an unused vector

; RAM Memory map
;
; __vecstart    Beginning of Interrupt Redirection information.  This is
;               used to hand off to RAM-based handlers for interrupts
;               caught by FLASH interrupt vectors.
; __vecend      End of the Interrupt Redirection information.
; __loaderstart Start of RAM used exclusively by the bootloader.  This
;               memory region an be recovered by the RAM-based program.
; __loaderend   End of the bootloader RAM.
; __progstart   Start of CODE for the RAM-based program.  The program can
;               freely use the memory region from _progstart-_progend and
;               can recover the memory for _loaderstart-_loaderend for heap
;               usage.
; __progend     End of RAM/End of the RAM-based program

VECSTART	EQU	__vecstart				; Start of interrupt redirection area
VECSIZE		EQU	__vecend - __vecstart	; Size of interrupt redirection area

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
; Interrupt Vector Handlers
;**************************************************************************

	define .STARTUP, space = ROM
	segment .STARTUP
	.assume ADL=1

; This is a copy of the handler table that will be copied into RAM at the
; address given by VECSTART by _ez80_initvectors.  FLASH based interrupt
; handling will vector here to support interrupts in the RAM-based program.

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
	_copysize EQU $-_ez80_handlers

;**************************************************************************
; Vector Setup Logic
;**************************************************************************

; Still in the .STARTUP segment.

_ez80_initvectors:

	; The interrupt vector and redirection tables reside in FLASH, but the
	; handlers must be copied to into the VECSTART region in RAM.  This
	; is necessary to support interrupt hand-off from FLASH-based interrupt
	; vectors to RAM-based programs.

	; Copy the initialized data section

	ld		bc, _copysize		; [bc] = data length
	ld		hl, _ez80_handlers	; [hl] = data source
	ld		de, VECSTART		; [de] = data destination
	ldir						; Copy the interrupt handlers

	; REVISIT:  We must assume that the bootloader has configured the
	; interrupt mode correctly.  The IM register should be set to 2(i.e,
	; interrupt mode 2) and the I register should be set according to the
	; location of the 1st level interrupt vectors in FLASH.

	ret
	end
