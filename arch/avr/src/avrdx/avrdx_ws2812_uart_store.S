/****************************************************************************
 * arch/avr/src/avrdx/avrdx_ws2812_uart_store.S
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

	.file	"avrdx_ws2812_uart_store.S"

/****************************************************************************
 * Name: avrdx_ws2812_store_uart_bytes_rgb
 *
 * Description:
 *   Stores given RGB data (with offset and length limited to len
 *   to UART transmit buffer, encoded for direct transmission to WS2812
 *   LED.)
 *
 * Input Parameters:
 *   lane - enum avrdx_ws2812_lane_e, interpreted as uint8_t
 *          (register r24)
 *   data - pointer to uint32_t array with RGB data (register r22:r23)
 *   offset - this is the first pixel to be written. LED count
 *            is limited to 255, data type is uint8_t (register r20)
 *   length - length of data array, same as above for type (register r18)
 *
 * Return value: void
 *
 * Assumptions/Limitations:
 *   - parameters are checked by the call chain (VFS) and len is not zero
 *
 ****************************************************************************/

	.global avrdx_ws2812_store_uart_bytes_rgb

avrdx_ws2812_store_uart_bytes_rgb:

	; Register usage
	; r1      - zero register (part of ABI, no need to clear it first)
	;
	; In prologue
	; (Mostly just storing data, not doing anything with it. Treat
	; this list as free-for-use/do-not-touch.)
	;
	; Do not touch
	; r2      - copy of length
	; r22:r23 - pointer to destination data
	; r24:r25 - pointer to destination data
	; r28:r29 - pointer to source data
	;
	; Free for use (when, if not always)
	; r20     - function argument (after prologue ends)
	; r24     - function argument (after prologue ends)
	; Z       - temporary pointer
	;
	; In main loop
	; r2      - copy of length, decremented after each pass, return
	;           when reduced to zero
	; r18     - current pixel component
	; r19     - index register for UART byte
	; r20     - temporary byte - do not use for anything that takes
	;           more than few lines of assembly
	; r21     - working copy of 9th bits (value assembled here, then
	;           stored when all 8 bits are ready.)
	; r22:r23 - pointer to g_avrdx_ws2812_uart9_bytes, ie. to current
	;           byte in array for 9th bits.
	; r24:r25 - pointer to current byte in UART transmit buffer.
	;           The pointer is usually stored in X but if X needs
	;           to address something else, copy is stored here.
	; r26:r27 - pointer to one of the transmit buffers. Code uses
	;           XL:XH notation.
	; r28:r29 - pointer to source data. Code uses YL:YH notation
	; r30:r31 - pointer to UART bytes. Code uses ZL:ZH notation
	;

	; prologue
	push r2
	push YL
	push YH

	; move len to r2 to keep registers with more functionality free
	mov r2, r18

	; Y stores pointer to source data and is not used for anything
	; else, free r22:r23 for other use (see above)
	movw YL, r22

	; load address of current byte in 9th bits transmit buffer
	; we start with this one because this it doesn't need offset
	; multiplied.
	;
	; This is an array of pointers
	ldi ZL, lo8(g_avrdx_ws2812_uart9_bytes)
	ldi ZH, hi8(g_avrdx_ws2812_uart9_bytes)

	; add lane number to select the correct pointer;
	; (pointer is 16 bit, add twice)
	add ZL, r24   ; r24 lane function argument, keep, still needed
	adc ZH, r1
	add ZL, r24
	adc ZH, r1

	; Z now points to the address of UART buffer for 9th bits.
	; Load the address to r22:r23.
	ld r22, Z
	ldd r23, Z+1

	; apply offset - 1 byte per offset in this array
	add r22, r20  ; r20 offset function argument, keep, still needed
	adc r23, r1   ; max 255 LEDs, high byte is zero

	; now load address of current byte of regular transmit buffer
	ldi ZL, lo8(g_avrdx_ws2812_xmit_bytes)
	ldi ZH, hi8(g_avrdx_ws2812_xmit_bytes)

	; similar as above, add lane number, load the address
	add ZL, r24
	adc ZH, r1
	add ZL, r24   ; lane number no longer needed
	adc ZH, r1
	ld r24, Z
	ldd r25, Z+1

	; apply offset - this array has 8 bytes per offset
	; we can clobber the offset, do not need it after this
	;
	; note - compiler tends to do additions (bitshifts)
	; when multiplying by constant but multiplying is faster
	; here (6 instructions instead of 7)
	ldi r21, 8
	mul r21, r20
	add r24, r0
	adc r25, r1
	eor r1, r1    ; ABI - r1 is zero

	; Z points to UART bytes (ie. precalculated values that encode
	; pixel data into bytes that are transmitted by UART.)
	;
	; In case you are wondering: yes, values in this array indeed
	; have incorrect bit 7. We will deal with that on load.
	ldi ZL, lo8(avrdx_ws2812_uart_white)
	ldi ZH, hi8(avrdx_ws2812_uart_white)

	; store g_avrdx_ws2812_xmit_bytes pointer here, that's what
	; is needed most of the time
	movw XL, r24
	; initialization phase done

1:
	; green component is first in the UART data (ie. LED wants it
	; first, MSB first) but not first in xxRRGGBB pixel
	ldd r18, Y+1
	; load UART byte for G7 G6 LED bits
	rcall 8f
	; G5 goes to 9th bits. note that r21 is not cleared.
	; 8 bits will be shifted to it so no reason to do that.
	lsl r18
	ror r21       ; counting bits shifted in: 1
	; same for G4 G3 G2
	rcall 8f
	lsl r18
	ror r21       ; 2 bits in
	; G1 G0 can be dealt with using the function too
	rcall 8f
	; need the next byte now though; the LED wants red
	ldd r18, Y+2
	lsl r18
	ror r21       ; 3 bits in
	; that was R7, R6-R4 now
	rcall 8f
	lsl r18
	ror r21       ; 4 bits in
	; and R3-R1
	rcall 8f
	lsl r18
	ror r21       ; 5 bits in
	; need R0 and B7 for the function call. Manually doing
	; what the function does otherwise. We skip that part
	; of the function but can use the rest
	lsl r18
	rol r19
	ld r18, Y
	lsl r18
	rol r19
	; note - skipping the part that sets r19
	rcall 7f
	; B6 goes to the 9th bits byte
	lsl r18
	ror r21       ; 6 bits in
	; B5-B3 the usual way
	rcall 8f
	lsl r18
	ror r21       ; 7 bits in
	; same for B2-B0
	rcall 8f
	lsl r18
	ror r21       ; 8 bits in
	; we are done with the 9th bits but they are inverted
	; (0 in data is now 0 in r21 but requires short pulse,
	; ie. log.1 parity bit.) fix by inverting
	com r21

	; store the pointer for g_avrdx_ws2812_xmit_bytes, need X
	; register usable
	movw r24, XL
	movw XL, r22
	; store 9th bits byte to the new X
	st X+, r21
	; set X back to the main buffer
	movw r22, XL
	movw XL, r24

	dec r2        ; decrement len
	breq 9f       ; next loop unless len == 0

	adiw YL, 4    ; next item in source data
	rjmp 1b

	; epilogue
9:
	pop YH
	pop YL
	pop r2
	ret

8:
	; bitshift top bits from the current component
	lsl r18
	rol r19
	lsl r18
	rol r19
	; trim bits from previous call
7:	andi r19, 0x03
	; r19 is now offset in avrdx_ws2812_uart_white
	add ZL, r19
	adc ZH, r1
	; load UART byte
	ld r20, Z
	; correct its 8th bit (always 0)
	andi r20, 0x7f
	; store it, post-increment
	st X+, r20
	; reset Z to original position
	sub ZL, r19
	sbc ZH, r1

	ret
