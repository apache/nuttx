;**************************************************************************
; arch/z80/src/ez80/ez80f91_handlers.asm
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

;**************************************************************************
; Macros
;**************************************************************************

; Define one interrupt handler

ifdef irqhandler
else
irqhandler: macro vectno
	; Save AF on the stack, set the interrupt number and jump to the
	; common reset handling logic.
							; Offset 8: Return PC is already on the stack
	push	af					; Offset 7: AF (retaining flags)
	ld		a, #vectno			; A = vector number
	jp		_ez80_irq_common		; Remaining RST handling is common
	endmac	irqhandler
endif

;**************************************************************************
; Interrupt Vector Handlers
;**************************************************************************

	define .STARTUP, space = ROM
	segment .STARTUP
	.assume ADL=1

						; Symbol           Val VecNo Addr
						;----------------- --- ----- -----
_ez80_handlers:
	irqhandler	 0		; EZ80_EMACRX_IRQ   0    0   0x040
_handlersize equ $-_ez80_handlers
	irqhandler	 1		; EZ80_EMACTX_IRQ   1    1   0x044
	irqhandler	 2		; EZ80_EMACSYS_IRQ  2    2   0x048
	irqhandler	 3		; EZ80_PLL_IRQ      3    3   0x04c
	irqhandler	 4		; EZ80_FLASH_IRQ    4    4   0x050
	irqhandler	 5		; EZ80_TIMER0_IRQ   5    5   0x054
	irqhandler	 6		; EZ80_TIMER1_IRQ   6    6   0x058
	irqhandler	 7		; EZ80_TIMER2_IRQ   7    7   0x05c
	irqhandler	 8		; EZ80_TIMER3_IRQ   8    8   0x060
	irqhandler	EZ80_UNUSED		;                9   0x064
	irqhandler	EZ80_UNUSED+1	;               10   0x068
	irqhandler	 9		; EZ80_RTC_IRQ      9   11   0x06C
	irqhandler	10		; EZ80_UART0_IRQ   10   12   0x070
	irqhandler	11		; EZ80_UART1_IRQ   11   13   0x074
	irqhandler	12		; EZ80_I2C_IRQ     12   14   0x078
	irqhandler	13		; EZ80_SPI_IRQ     13   15   0x07c
	irqhandler	14		; EZ80_PORTA0_IRQ  14   16   0x080
	irqhandler	15		; EZ80_PORTA1_IRQ  15   17   0x084
	irqhandler	16		; EZ80_PORTA2_IRQ  16   18   0x088
	irqhandler	17		; EZ80_PORTA3_IRQ  17   19   0x08c
	irqhandler	18		; EZ80_PORTA4_IRQ  18   20   0x090
	irqhandler	19		; EZ80_PORTA5_IRQ  19   21   0x094
	irqhandler	20		; EZ80_PORTA6_IRQ  20   22   0x098
	irqhandler	21		; EZ80_PORTA7_IRQ  21   23   0x09c
	irqhandler	22		; EZ80_PORTB0_IRQ  22   24   0x0a0
	irqhandler	23		; EZ80_PORTB1_IRQ  23   25   0x0a4
	irqhandler	24		; EZ80_PORTB2_IRQ  24   26   0x0a8
	irqhandler	25		; EZ80_PORTB3_IRQ  25   27   0x0ac
	irqhandler	26		; EZ80_PORTB4_IRQ  26   28   0x0b0
	irqhandler	27		; EZ80_PORTB5_IRQ  27   29   0x0b4
	irqhandler	28		; EZ80_PORTB6_IRQ  28   20   0x0b8
	irqhandler	29		; EZ80_PORTB7_IRQ  29   21   0x0bc
	irqhandler	30		; EZ80_PORTC0_IRQ  30   22   0x0c0
	irqhandler	31		; EZ80_PORTC1_IRQ  31   23   0x0c4
	irqhandler	32		; EZ80_PORTC2_IRQ  32   24   0x0c8
	irqhandler	33		; EZ80_PORTC3_IRQ  33   25   0x0cc
	irqhandler	34		; EZ80_PORTC4_IRQ  34   26   0x0d0
	irqhandler	35		; EZ80_PORTC5_IRQ  35   27   0x0d4
	irqhandler	36		; EZ80_PORTC6_IRQ  36   28   0x0d8
	irqhandler	37		; EZ80_PORTC7_IRQ  37   29   0x0dc
	irqhandler	38		; EZ80_PORTD0_IRQ  38   40   0x0e0
	irqhandler	39		; EZ80_PORTD1_IRQ  39   41   0x0e4
	irqhandler	40		; EZ80_PORTD2_IRQ  40   42   0x0e8
	irqhandler	41		; EZ80_PORTD3_IRQ  41   43   0x0ec
	irqhandler	42		; EZ80_PORTD4_IRQ  42   44   0x0f0
	irqhandler	43		; EZ80_PORTD5_IRQ  43   45   0x0f4
	irqhandler	44		; EZ80_PORTD6_IRQ  44   46   0x0f8
	irqhandler	45		; EZ80_PORTD7_IRQ  45   47   0x0fc
	irqhandler	EZ80_UNUSED+2	;               48   0x100
	irqhandler	EZ80_UNUSED+3	;               49   0x104
	irqhandler	EZ80_UNUSED+4	;               50   0x108
	irqhandler	EZ80_UNUSED+5	;               51   0x10c
	irqhandler	EZ80_UNUSED+6	;               52   0x110
	irqhandler	EZ80_UNUSED+7	;               53   0x114
	irqhandler	EZ80_UNUSED+8	;               54   0x118
	irqhandler	EZ80_UNUSED+9	;               55   0x11c
	irqhandler	EZ80_UNUSED+10	;               56   0x120
	irqhandler	EZ80_UNUSED+11	;               57   0x124
	irqhandler	EZ80_UNUSED+12	;               58   0x128
	irqhandler	EZ80_UNUSED+13	;               59   0x12c
	irqhandler	EZ80_UNUSED+14	;               60   0x130
	irqhandler	EZ80_UNUSED+15	;               61   0x134
	irqhandler	EZ80_UNUSED+16	;               62   0x138
	irqhandler	EZ80_UNUSED+17	;               63   0x13c

;**************************************************************************
; Vector Setup Logic
;**************************************************************************

; Still in .STARTUP section

_ez80_initvectors:

	; Initialize the vector table

	ld		iy, _ez80_vectable
	ld		ix, 4
	ld		bc, 4
	ld		b, NVECTORS
	xor		a, a				; Clear carry; Set A to zero
	ld		de, _handlersize	; Length of one irq handler in DE
	ld		hl, _ez80_handlers 	; Start of handlers in HL

	; "The size of I register is modified to 16 bits in the eZ80F91 device
	;  differing from the previous versions of eZ80® CPU, to allow for a 16
	;  MB range of interrupt vector table placement.
	;
	; "Additionally, the size of the IVECT register is increased from 8 bits
	;  to 9 bits to provide an interrupt vector table that is expanded and
	;  more easily integrated with other interrupts.
	;
	; "The vectors are 4 bytes (32 bits) apart, even though only 3 bytes
	;  (24 bits) are required.  A fourth byte is implemented for both
	;  programmability and expansion purposes."

L1:
	ld		(iy), hl			; Store IRQ handler
	ld		(iy+3), a			; Pad with zero to 4 bytes
	add		hl, de				; Point to next handler
	push		de
	ld		de, 4
	add		iy, de				; Point to next entry in vector table
	pop		de
	djnz		L1				; Loop until all vectors have been written

	; Select interrupt mode 2

	im		2				; Interrupt mode 2

	; Write the address of the vector table into the interrupt vector base

	; The GNU assembler (2.36.1) cannot produce this relocation, although the
	; Z80 ELF format supports it. The instruction is instead hand assembled.
	;ld		hl, _ez80_vectable >> 8
	db		021h
	db		_ez80_vectable >> 8
	db		_ez80_vectable >> 16
	db		0
	ld		i, hl
	ret

;**************************************************************************
; Vector Table
;**************************************************************************

; This segment must be aligned on a 512 byte boundary anywhere in RAM
; Each entry will be a 3-byte address in a 4-byte space

	define	.IVECTS, space = RAM, align = 200h
	segment	.IVECTS

	; The first 64 bytes are not used... the vectors actually start at +0x40

_ez80_vecreserve:
	ds	64
_ez80_vectable:
	ds	NVECTORS * 2
	end
