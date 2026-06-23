/****************************************************************************
 * arch/avr/src/avrdx/avrdx_ws2812_uart_xmit.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <avrdx_swint.h>

#include <avr/io.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if (defined CONFIG_WS2812_HAS_WHITE) && \
    (defined CONFIG_AVR_AVRDX_WS2812_NO_WHITE_LED)
#  undef CONFIG_WS2812_HAS_WHITE
#endif

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.global avrdx_swint_raise
	.file	"avrdx_ws2812_uart_xmit.S"

/****************************************************************************
 * Name: __vector_avrdx_usart2_dre
 *
 * Description:
 *   Interrupt handler for LED data transmission.
 *
 *   The compiler does pretty decent job if this function is compiled
 *   from C - execution of the handler takes 66 clock cycles (compiled
 *   with avr-gcc 14.2, calculated by hand.) That's with an implementation
 *   that takes next byte and transmits it.
 *
 *   Such an implementation has a room for improvement though: every
 *   transmitted byte has 12 bits (start + 9 bits + parity + stop), that is
 *   96 CPU clock cycles. If any other interrupt request comes in, it is
 *   almost sure to take more than 30 CPU cycles (which is 96 - 66 ie. time
 *   needed to transmit minus time needed to prepare the transmission.)
 *   That is going to leave the UART hardware idle almost for sure.
 *
 *   Now, as far as author of this code understands it, there are two
 *   levels of buffering within the UART hardware. When a byte is written
 *   to the TX data register, it is moved to the TX Buffer if it is free.
 *   Then it is moved from TX Buffer to the shift register if it is free.
 *   This means that if the hardware is idle (for example because another
 *   interrupt handler was running for more than 3*96 = 288 CPU cycles),
 *   it can receive 3 new bytes.
 *
 *   This is where the improvement comes from. If the interrupt handler
 *   waits for few CPU clock cycles after pushing a byte - and that happens
 *   naturally because of bookkeeping (incrementing next byte index
 *   for example) - it can check if the TX data register became available
 *   for next write (and write another byte if possible.)
 *
 *   Making such decision is not free but on the other hand, it saves
 *   some 33 clock cycles worth of push/pop, initialization, jmp/reti
 *   and decision about white LEDs.
 *
 *   Unfortunately, this is where the compiler stops doing good job.
 *   It suddenly decides to use more registers (more push/pop), variable
 *   that takes care of limiting loop count before the interrupt handler
 *   cedes the CPU to other code is forced to be 16 bits etc. Overall,
 *   the optimized implementation becomes slower than the simple one.
 *
 *   That's why an assembly implementation is used. (It is mostly based
 *   on the compiled C code though, with some tweaks and with simplified
 *   implementation of the counter.)
 *
 *   Loop count is chosen to be 4. That is 3 bytes of levels of buffering
 *   in the hardware plus accounting for the fact that when the 3rd byte
 *   if prepared, the 1st one is probably already out. (We give other
 *   handlers a chance to run after that.)
 *
 *   This handler varies depending on CONFIG_AVR_AVRDX_WS2812_SHORT_LANE.
 *   If this option is unset, next byte index is 16 bit.
 *
 *   Numbers (calculated by hand, for 8 bit next byte index):
 *   The handler takes 70 clock ticks to run (out of 96 clock ticks
 *   needed to transmit the byte.) Sending single byte therefore causes
 *   73% CPU load.
 *
 *   If looping, every loop only needs 37 clock ticks. Therefore,
 *   if the handler succeeds in pushing more than one byte, the load
 *   is reduced (pushed bytes - CPU load):
 *     2 bytes - CPU 56% load
 *     3 bytes - CPU 50% load
 *     4 bytes - CPU 47% load
 *
 *   Numbers (16 bit next byte index):
 *   The handler takes 77 clock ticks, a 7 tick increase, one tick of that
 *   in prologue, 6 ticks in loop. CPU load of single pass increases
 *   to 80%. Loop needs 43 clock ticks, load reduction is as follows:
 *     2 bytes - CPU 63% load
 *     3 bytes - CPU 57% load
 *     4 bytes - CPU 54% load
 *
 ****************************************************************************/

	.macro WS2812_HANDLER_UART, uartnum
	.global __vector_avrdx_usart\uartnum\()_dre
__vector_avrdx_usart\uartnum\()_dre:

	; Register usage
	; r1      - zero register (part of ABI but need to clear it first)
	;           note - if CONFIG_AVR_AVRDX_WS2812_SHORT_LANE is not set,
	;           then r1 is not used and NOT SET TO ZERO
	;
	; r19     - loop counter
	; r22     - temporary value
	; r23     - value of current uart9 byte until processing of 9th bits
	;           is done (that is, until label 2 is reached)
	;         - temporary value after that
	; r24     - value of next byte (the one processed by this run)
	;           (see r25 below)
	; r30:r31 - various pointers
	; SREG(T) - flag that is set if support for LEDs with white component
	;           is enabled and in use for this lane (unused if
	;           CONFIG_WS2812_HAS_WHITE is not set)
	;
	; If CONFIG_AVR_AVRDX_WS2812_SHORT_LANE is not set:
	; r1       - not used and NOT SET TO ZERO
	; r25      - value of next byte (the one processed by this run),
	;            high byte

	; Prologue.
	;
	; Note that interrupt handler generated by C compiler saves RAMPZ.
	; Since there currently is no available AVR Dx device with more
	; than 64kB data space, we do not do that.
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	; Zero register is not used if byte counter has 16 bits
	push r1
	in r1, SREG
	push r1
	eor r1, r1
#else
	; If the zero register is not used, SREG is not saved. Use r19
	; (Doing it like this to preserve push order.)
	push r19
	in r19, SREG
#endif
	push r19
	push r22
	push r23
	push r24
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	push r25
#endif
	push r30
	push r31
	; total loop count + 1
	ldi r19, 5
	; next byte to transmit - will be needed later
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	lds r24, g_avrdx_ws2812_xmit_next+\uartnum
#else
	lds r24, g_avrdx_ws2812_xmit_next+2*\uartnum
	lds r25, g_avrdx_ws2812_xmit_next+2*\uartnum\()+1
#endif

	; 13 ticks total (+1 with not short lane)

    ; load current 9th bit byte
    lds r23, g_avrdx_ws2812_uart9_current+\uartnum

#if defined(CONFIG_WS2812_HAS_WHITE)
	; If support for LEDs with white component is enabled and the lane
	; has such LEDs, the current uart9 byte (a group of 8 9th bits)
	; is stored with highest bit set. That is normally not possible
	; (the byte is rotated to the right after load) and is used as
	; a signal.
	;
	; If support for LEDs with white component is not enabled, this
	; check is not needed.
	;
	; C code:
    ; if ((g_avrdx_ws2812_uart9_current[2] & 0x80) == 0) {
    ;
    ; (Note for the C code - index 2 is hardcoded for USART2 ISR.)

    ; Check for the signal that there is white component (therefore
    ; 9th bits are not in use, therefore can be skipped);
    ; store the information in T bit in SREG, will be used if looping
    bst r23, 7
    brts 2f
    ; 4 ticks unless branch is taken (2 ticks if not checked)
#endif

3:
	; Is current byte divisible by 8? we need to load next 9th bits
	; byte in such case
	;
	; C code:
	; if ((g_avrdx_ws2812_xmit_next[2] & 0x07) == 0) {
	;g_avrdx_ws2812_uart9_current[2] = \
	;     g_avrdx_ws2812_uart9_bytes\
	;       [2] \
	;       [ \
	;         g_avrdx_ws2812_xmit_next[2] >> 3 \
	;       ];
	; }
	;
	; Note - r24 has correct value in loops. Never overwritten and
	; incremented below.
	mov r22, r24
	andi r22, 0x07
	; Note 1 - with short lane set, compiler compares r22 with r1
	; here - pointless (andi already set the zero flag.)
	; Note 2 - with short lane not set, compiler does eor
	; on a register and does logical or with r22 - also pointless
	; (same as above, andi already set the zero flag.)
	breq 7f
	; 3 ticks if branch is not taken (7/8 cases), 4 ticks if it is

1:
	; load 9th bit into TX register
	;
	; C code:
	; USART2.TXDATAH = g_avrdx_ws2812_uart9_current[2] & 0x01;
    ; g_avrdx_ws2812_uart9_current[2] >>= 1;
	mov r22, r23
	andi r22, 0x01
	; avr/io.h provides USARTx_TXDATAH preprocessor macro
	; with address we need but we can't use it with the \uartnum
	; parameter because that is interpreted after the preprocessor
	; runs. we will work around this by applying offset to address
	; of first peripheral
	sts USART0_TXDATAH+0x20*\uartnum\(), r22
	; prepare next bit and store it back to RAM
	lsr r23
	sts g_avrdx_ws2812_uart9_current+\uartnum\(), r23
	; 7 ticks
2:
	; load current byte to TX register, trigger the transmission
	;
	; C code:
	; USART2.TXDATAL = g_avrdx_ws2812_xmit_bytes \
	;   [2] \
	;   [g_avrdx_ws2812_xmit_next[2]];
	lds r30, g_avrdx_ws2812_xmit_bytes+2*\uartnum
	lds r31, g_avrdx_ws2812_xmit_bytes+2*\uartnum\()+1
	add r30, r24
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	adc r31, r1
#else
	adc r31, r25
#endif
	ld r23, Z
	sts USART0_TXDATAL+0x20*\uartnum\(), r23
	; g_avrdx_ws2812_xmit_next[2]++;
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	subi r24, 0xff
	sts g_avrdx_ws2812_xmit_next+\uartnum, r24
#else
	adiw r24, 1
	sts g_avrdx_ws2812_xmit_next+2*\uartnum, r24
	sts g_avrdx_ws2812_xmit_next+2*\uartnum\()+1, r25
#endif

	; Are we done?
	;
	; C code:
	; if (g_avrdx_ws2812_xmit_next[2] == g_avrdx_ws2812_xmit_count[2]) {
	;   USART2.CTRLA &= ~USART_DREIE_bm;
	; }
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	lds r22, g_avrdx_ws2812_xmit_count+\uartnum
#else
	lds r22, g_avrdx_ws2812_xmit_count+2*\uartnum
	lds r23, g_avrdx_ws2812_xmit_count+2*\uartnum\()+1
#endif
	cp r24, r22
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	cpc r25, r23
#endif
	breq 8f
	; 16/17 ticks (branch not/taken; mostly not)
	; (+6 with not short lane)

	; We are not done. By this time, value loaded to TXDATA should be
	; shifted to transmit buffer, if there was space available. Maybe
	; push next byte?
	dec r19
	; Do not push next byte if we looped multiple times
	breq 9f
	; 2/3 ticks (expected to take the branch once out of three cases)

	; Check for DREIF flag in status register. If set, data register
	; is empty
	lds r23, USART0_STATUS+0x20*\uartnum\()
	andi r23, USART_DREIF_bm
	; not set, no more loops
	breq 9f
	; 4/5 ticks (not expected to take the branch)
#if defined(CONFIG_WS2812_HAS_WHITE)
	; Same as above. If T in SREG is set, LEDs with white component
	; are supported and in use here. (Skipping 9th bits load.)
	; Note - r24 has correct value of next byte, incremented above
	brts 2b
#endif
	; reload uart9 back to r23
	lds r23, g_avrdx_ws2812_uart9_current+\uartnum
	rjmp 3b

9:
	; Epilogue
	pop r31
	pop r30
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	pop r25
#endif
	pop r24
	pop r23
	pop r22
	pop r19
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	out SREG, r19
	pop r19
#else
	pop r1
	out SREG, r1
	pop r1
#endif
	reti
	; 9 ticks

7:
	; Load new uart9 byte from the array
	;
	; note - this code clobbers r23 but at this point, that value
	; is used and new needs to be loaded (which is what this does.)
	; g_avrdx_ws2812_uart9_bytes is array of pointers to arrays
	; of 9th bits bytes. locate pointer for this handler
	; (Note - \uartnum is replaced by parameter, \() is ending
	; delimiter for the parameter - needed if something follows.)
	lds r30, g_avrdx_ws2812_uart9_bytes+2*\uartnum
	lds r31, g_avrdx_ws2812_uart9_bytes+2*\uartnum\()+1
	; offset in this handler's array
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	mov r22, r24
	lsr r22
	lsr r22
	lsr r22
#else
	movw r22, r24
	lsr r23
	ror r22
	lsr r23
	ror r22
	lsr r23
	ror r22
#endif
	add r30, r22
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	adc r31, r1
#else
	adc r31, r23
#endif
	; previous byte is replaced in r23
	ld r23, Z
	rjmp 1b
	; 13 ticks (+4 with not short lane)

8:
	; Transmission finished
	;
	; clear DREIE - Data Register Empty Interrupt Enabled
	lds r24, USART0_CTRLA+0x20*\uartnum\()
	andi r24, ~USART_DREIE_bm
	sts USART0_CTRLA+0x20*\uartnum\(), r24

	; We are calling C function now. We cannot assume anything about
	; its implementation and need to adhere to the ABI. Registers
	; R18–R27, R30, R31, R0 are call-clobbered and so is T flag
	; in SREG.
	;
	; All of these must be preserved from the interrupted code's
	; point of view but as long as we do nothing else after we
	; call the function, we can lose our own data.

	; push what was not pushed in the prologue
	push r0
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	; Zero register was not pushed nor zeroed
	push r1
	eor r1, r1
#endif
	push r18
	push r20
	push r21
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	push r25
#endif
	push r26
	push r27

	; load software interrupt vector number
#if (AVRDX_SWINT_VECTORS_SIZE == 1)
	lds r24, g_avrdx_ws2812_swint_vector+\uartnum
#elif (AVRDX_SWINT_VECTORS_SIZE == 2)
	lds r24, g_avrdx_ws2812_swint_vector+2*\uartnum
	lds r25, g_avrdx_ws2812_swint_vector+2*\uartnum\()+1
#elif (AVRDX_SWINT_VECTORS_SIZE == 3)
	lds r22, g_avrdx_ws2812_swint_vector+3*\uartnum
	lds r23, g_avrdx_ws2812_swint_vector+3*\uartnum\()+1
	lds r24, g_avrdx_ws2812_swint_vector+3*\uartnum\()+2
#elif (AVRDX_SWINT_VECTORS_SIZE == 4)
	lds r22, g_avrdx_ws2812_swint_vector+4*\uartnum
	lds r23, g_avrdx_ws2812_swint_vector+4*\uartnum\()+1
	lds r24, g_avrdx_ws2812_swint_vector+4*\uartnum\()+2
	lds r25, g_avrdx_ws2812_swint_vector+4*\uartnum\()+3
#else
#  error Unhandled AVRDX_SWINT_VECTORS_SIZE
#endif

	; raise the interrupt
	call avrdx_swint_raise

	; now pop what we pushed here
	pop r27
	pop r26
#if defined(CONFIG_AVR_AVRDX_WS2812_SHORT_LANE)
	pop r25
#endif
	pop r21
	pop r20
	pop r18
#ifndef CONFIG_AVR_AVRDX_WS2812_SHORT_LANE
	pop r1
#endif
	pop r0
	rjmp 9b

	.endm

#if defined(CONFIG_AVR_AVRDX_WS2812_USART0)
	WS2812_HANDLER_UART 0
#endif
#if defined(CONFIG_AVR_AVRDX_WS2812_USART1)
	WS2812_HANDLER_UART 1
#endif
#if defined(CONFIG_AVR_AVRDX_WS2812_USART2)
	WS2812_HANDLER_UART 2
#endif
#if defined(CONFIG_AVR_AVRDX_WS2812_USART3)
	WS2812_HANDLER_UART 3
#endif
#if defined(CONFIG_AVR_AVRDX_WS2812_USART4)
	WS2812_HANDLER_UART 4
#endif
#if defined(CONFIG_AVR_AVRDX_WS2812_USART5)
	WS2812_HANDLER_UART 5
#endif

