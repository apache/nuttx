/*************************************************************************
 * arch/z16/src/z16f/z16f_lowuart.asm
 * Z16F UART management
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSeqUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include "chip/chip.h"

#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_ARCH_LOWGETC) || CONFIG_NFILE_DESCRIPTORS == 0

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_z16f_lowuartinit
	xref	_SYS_CLK_FREQ:EROM
#ifdef CONFIG_ARCH_LOWPUTC
	xdef	_up_lowputc
#endif
#ifdef CONFIG_ARCH_LOWGETC
	xdef	_up_lowgetc
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
 * Parameters:
 *   None
 *
 *************************************************************************/

_z16f_lowuartinit:
	pushmlo <r0, r3> 		/* Save registers */

	/* Calculate and set the baud rate generation register */


#ifdef CONFIG_UART0_SERIAL_CONSOLE
	ld	r3, #CONFIG_UART0_BAUD	/* r3 = baud */
#else
	ld	r3, #CONFIG_UART1_BAUD	/* r3 = baud */
#endif
	ld	r0, r3			/* r0 = baud */
	sll	r0, #3			/* r0 = baud * 8 */
	add	r0, #_SYS_CLK_FREQ	/* r3 = freq + baud * 8*/
	sll	r3, #4			/* r3 = baud * 16 */
	udiv	r0, r3			/* BRG = (freq + baud * 8)/(baud * 16) */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
	ld.w	Z16F_UART1_BR, r0	/* Z16F_UART1_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld	r0, #%30
	or.b	Z16F_GPIOD_AFL, r0	/* Z16F_GPIOD_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART1_CTL1 	/* Z16F_UART1_CTL1 = 0 */
	ld	r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART1_CTL0, r0	/* Z16F_UART1_CTL0 = %c0 */
#else
	ld.w	Z16F_UART0_BR, r0	/* Z16F_UART0_BR = BRG */

	/* Set the GPIO Alternate Function Register Lo (AFL) register */

	ld	r0, #%30
	or.b	Z16F_GPIOA_AFL, r0	/* Z16F_GPIOA_AFL |= %30 */

	/* Enable UART receive (REN) and transmit (TEN) */

	clr.b	Z16F_UART0_CTL1 	/* Z16F_UART0_CTL1 = 0 */
	ld	r0, #(Z16F_UARTCTL0_TEN|Z16F_UARTCTL0_REN)
	ld.b	Z16F_UART0_CTL0, r0	/* Z16F_UART0_CTL0 = %c0 */
#endif
	popmlo	<r0, r3>		/* Restore registers */
	ret				/* Return */


/*************************************************************************
 * Name: _z16f_xmitc
 *
 * Description:
 *   Send one character on the selected port
 *
 * Parmeters:
 *   r1 = character
 *
 *************************************************************************/

#ifdef CONFIG_ARCH_LOWPUTC
_z16f_xmitc:
	pushmlo <r0>				/* Save registers */

_z16f_xmitc1:
	ld	r0, Z16F_UARTSTAT0_TDRE		/* TDRE=Transmitter Data Register Empty */
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	tm.b	Z16F_UART0_STAT0, r0		/* r0 = Z16F_UART0_STAT0 */
	jp	eq, _z16f_xmitc1 		/* While (!(Z16F_UART0_STAT0 & TDRE)) */
	ld.b	Z16F_UART0_TXD, r1		/* Z16F_UART0_TXD = r1 (character) */
#else
	tm.b	Z16F_UART1_STAT0, r0		/* r0 = Z16F_UART0_STAT1 */
	jp	eq, _z16f_xmitc1		/* While (!(Z16F_UART1_STAT0 & TDRE)) */
	ld.b	Z16F_UART1_TXD, r1		/* Z16F_UART1_TXD = r1 (character) */
#endif
	popmlo	<r0>				/* Restore registers */
	ret					/* Return */
#endif

/*************************************************************************
 * Name: _up_lowputc
 *
 * Description:
 *   Send one character to the selected serial console
 *
 * Parmeters:
 *   r1 = character
 *
 * Return
 *   R0 = 0
 *
 *************************************************************************/

#ifdef CONFIG_ARCH_LOWPUTC
_up_lowputc:
	pushmlo <r1,r5> 		/* Save registers */
	ld	r0, r1
	ext.ub	r5, r0
	cp	r5, #10
	jp	ne, _up_lowputc1	/* If (character == \n) */

	ld	r1,#13
	call	_z16f_xmitc		/* Call _z16f_xmitc with \r */

_up_lowputc1:
	ld	r1, r0
	call	_z16f_xmitc		/* Xall _z16f_xmitc with character */

	ld	r0, #0			/* return r0 = 0 */
	popmlo	<r1,r5> 		/* Restore registers */
	ret				/* Return */
#endif

/*************************************************************************
 * Name: _up_lowgetc
 *
 * Description:
 *   Get a character from the serial port
 *
 * Parmeters:
 *   None
 *
 * Return
 *   R0 = Character read
 *
 *************************************************************************/

#ifdef CONFIG_ARCH_LOWGETC
_up_lowgetc:
up_lowgetc1:
	ld	r0, #Z16F_UARTSTAT0_RDA		/* RDA=Receive data available */
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	tm.b	Z16F_UART0_STAT0, r0
	jp	eq, _up_lowgetc1		/* While (!Z16F_UART0_STAT0 & RDA)) */
	ld.ub	r0, Z16F_UART0_RXD		/* r0 = Z16F_UART0_RXD */
#else
	tm.b	Z16F_UART1_STAT0,r0		/* While (!Z16F_UART1_STAT0 & RDA) */
	jp	eq, _up_lowgetc1
	ld.ub	r0, Z16F_UART1_RXD		/* r0 = Z16F_UART1_RXD */
#endif
	cp	r0, #%0d			/* Test for '\r' */
	jp	eq, _up_lowgetc2

	cp	r0, #%0d			/* Test \r + high bit */
	jp	ne, _up_lowgetc3
_up_lowgetc2:
	ld	r0, #%0a			/* Convert '\r' to '\n' */
_up_lowgetc3:					/* Return value in r0 */
	ret					/* Return */
#endif

#endif /* CONFIG_ARCH_LOWPUTC || CONFIG_ARCH_LOWGETC */

	end
