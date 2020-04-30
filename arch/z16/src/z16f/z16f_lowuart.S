/*************************************************************************
 * arch/z16/src/z16f/z16f_lowuart.asm
 * Z16F UART management
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
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "z16_internal.h"

#ifdef USE_LOWUARTINIT

/*************************************************************************
 * Pre-processor Definitions
 *************************************************************************/

#ifndef CONFIG_Z16F_UART0
#  undef CONFIG_UART0_SERIAL_CONSOLE
#endif

#ifndef CONFIG_Z16F_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#endif

#if defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define HAVE_Z16F_SERIAL_CONSOLE
#endif

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_z16f_lowuartinit
	xref	_SYS_CLK_FREQ:EROM
#ifdef CONFIG_Z16_LOWPUTC
	xdef	_z16_lowputc
#endif
#ifdef CONFIG_Z16_LOWGETC
	xdef	_z16_lowgetc
#endif

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: z16f_lowuartinit
 *
 * Description:
 *   Initialize UART0 or UART1
 *
 * Input Parameters:
 *   None
 *
 *************************************************************************/

_z16f_lowuartinit:

#ifdef HAVE_Z16F_SERIAL_CONSOLE
	/* Calculate and set the baud rate generation register */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ld		r3, #CONFIG_UART1_BAUD	/* r3 = Selected UART1 baud */
#else
	ld		r3, #CONFIG_UART0_BAUD	/* r3 = Selected UART0 (default) baud */
#endif
	ld		r0, r3					/* r0 = baud */
	sll		r0, #3					/* r0 = baud * 8 */
	add		r0, #_SYS_CLK_FREQ		/* r3 = freq + baud * 8 */
	sll		r3, #4					/* r3 = baud * 16 */
	udiv	r0, r3					/* BRG = (freq + baud * 8)/(baud * 16) */

	/* Hacks to get a serial console available ASAP */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ld.w	Z16F_UART1_BR, r0		/* Z16F_UART1_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld		r0, #%30
	or.b	Z16F_GPIOD_AFL, r0		/* Z16F_GPIOD_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART1_CTL1 		/* Z16F_UART1_CTL1 = 0 */
	ld		r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART1_CTL0, r0		/* Z16F_UART1_CTL0 = %c0 */

#else
	ld.w	Z16F_UART0_BR, r0		/* Z16F_UART0_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld		r0, #%30
	or.b	Z16F_GPIOA_AFL, r0		/* Z16F_GPIOA_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART0_CTL1 		/* Z16F_UART0_CTL1 = 0 */
	ld		r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART0_CTL0, r0		/* Z16F_UART0_CTL0 = %c0 */

#endif
#endif /* HAVE_Z16F_SERIAL_CONSOLE */

	ret								/* Return */

#endif /* USE_LOWUARTINIT */

/*************************************************************************
 * Name: _z16_lowputc
 *
 * Description:
 *   Send one character to the selected serial console
 *
 * Input Parameters:
 *   r1 = character
 *
 * Returned Value:
 *   None
 *
 * Modifies r0 (and maybe r1)
 *
 *************************************************************************/

#ifdef CONFIG_Z16_LOWPUTC
_z16_lowputc:

#ifdef HAVE_Z16F_SERIAL_CONSOLE
	/* Check if the character to output is a linefeed */

	ext.ub	r0, r1					/* r0=Character masked to 8-bits */
	cp		r0, #10					/* Is it a linefeed ('\n') */
	jp		ne, _z16f_xmitc			/* No? Jump to _z16f_xmitc with character in r1 */

	/* Output a carriage return before the linefeed */

	ld		r1, #13					/* Output carriage reuturn ('\r') */
	call	_z16f_xmitc				/* Call _z16f_xmitc with r1='\r' */
	ld		r1, #10					/* Restore r1=linefeed to output */
									/* Fall through to _z16f_xmitc with linefeed in r1 */
#endif /* HAVE_Z16F_SERIAL_CONSOLE */

/*************************************************************************
 * Name: _z16f_xmitc
 *
 * Description:
 *   Send one character on the selected port (really a part of z16_lowputc)
 *
 * Input Parameters:
 *   r1 = character
 *
 * Returned Value:
 *   None
 *
 * Modifies r0
 *
 *************************************************************************/

_z16f_xmitc:
_z16f_xmitc1:

#ifdef HAVE_Z16F_SERIAL_CONSOLE
	ld		r0, Z16F_UARTSTAT0_TDRE	/* TDRE=Transmitter Data Register Empty */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	tm.b	Z16F_UART1_STAT0, r0	/* r0 = Z16F_UART1_STAT0 */
	jp		eq, _z16f_xmitc1 		/* While (!(Z16F_UART1_STAT0 & TDRE)) */
	ld.b	Z16F_UART1_TXD, r1		/* Z16F_UART1_TXD = r1 (character) */

#else
	tm.b	Z16F_UART0_STAT0, r0	/* r0 = Z16F_UART0_STAT1 */
	jp		eq, _z16f_xmitc1		/* While (!(Z16F_UART0_STAT0 & TDRE)) */
	ld.b	Z16F_UART0_TXD, r1		/* Z16F_UART0_TXD = r1 (character) */

#endif
#endif /* HAVE_Z16F_SERIAL_CONSOLE */

	ret								/* Return */

#endif /* CONFIG_Z16_LOWPUTC */

/*************************************************************************
 * Name: _z16_lowgetc
 *
 * Description:
 *   Get a character from the serial port
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   R0 = Character read
 *
 *************************************************************************/

#ifdef CONFIG_Z16_LOWGETC
_z16_lowgetc:
_z16_lowgetc1:

#ifdef HAVE_Z16F_SERIAL_CONSOLE
	ld		r0, #Z16F_UARTSTAT0_RDA	/* RDA=Receive data available */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	tm.b	Z16F_UART1_STAT0, r0
	jp		eq, _z16_lowgetc1		/* While (!Z16F_UART1_STAT0 & RDA)) */
	ld.ub	r0, Z16F_UART1_RXD		/* r0 = Z16F_UART1_RXD */

#else
	tm.b	Z16F_UART0_STAT0,r0		/* While (!Z16F_UART0_STAT0 & RDA) */
	jp		eq, _z16_lowgetc1
	ld.ub	r0, Z16F_UART0_RXD		/* r0 = Z16F_UART0_RXD */

#endif

	cp		r0, #%0d				/* Test for '\r' */
	jp		eq, _z16_lowgetc2

	cp		r0, #%0d				/* Test \r + high bit */
	jp		ne, _z16_lowgetc3

_z16_lowgetc2:
	ld		r0, #%0a				/* Convert '\r' to '\n' */

_z16_lowgetc3:						/* Return value in r0 */
#endif /* HAVE_Z16F_SERIAL_CONSOLE */

	ret								/* Return */
#endif

	end
