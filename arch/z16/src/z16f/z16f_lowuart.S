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
#include <chip/chip.h>

#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_ARCH_LOWGETC)

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	z16f_lowuartinit
	xref	_SYS_CLK_FREQ:EROM
#ifdef CONFIG_ARCH_LOWPUTC
	xdef	_z16f_xmitc
	xdef	_up_lowputc
#endif
#ifdef CONFIG_ARCH_LOWGETC
	xdef	_up_lowgetc
#endif

/*************************************************************************
 * Data Allocation
 *************************************************************************/

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
 *   r1 = 0:UART0 1:UART1
 *   r2 = Frequency
 *   r3 = BAUD rate
 *
 *************************************************************************/

z16f_lowuartinit:
	pushmlo <r0,R3> 		/* Save registers */

	ld	r3, #_SYS_CLK_FREQ
	ld	r0, R3
	sll	r0, #3
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	sll	r0, #CONFIG_UART0_BAUD
#else
	sll	r0, #CONFIG_UART1_BAUD
#endif
	sll	R3, #4
	udiv	r0, R3			/* BRG = (freq + baud * 8)/(baud * 16) */

#ifdef CONFIG_UART0_SERIAL_CONSOLE
	ld.w	Z16F_UART0_BR, r0	/* Z16F_UART0_BR = BRG */
	ld	r0, #%30
	or.b	Z16F_GPIOA_AFL, r0	/* Z16F_GPIOA_AFL |= %30 */
	clr.b	Z16F_UART0_CTL1 	/* Z16F_UART0_CTL1 = 0 */
	ld	0, #%c0
	ld.b	Z16F_UART0_CTL0, r0	/* Z16F_UART0_CTL0 = %c0 */
#else
	ld.w	Z16F_UART1_BR, r0	/* Z16F_UART1_BR = BRG */
	ld	r0, #%30
	or.b	Z16F_GPIOD_AFL, r0	/* Z16F_GPIOD_AFL |= %30 */
	clr.b	Z16F_UART1_CTL1 	/* Z16F_UART1_CTL1 = 0 */
	ld	r0, #%c0
	ld.b	Z16F_UART1_CTL0, r0	/* Z16F_UART1_CTL0 = %c0 */
#endif
	popmlo	<r0, R3>		/* Restore registers */
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
	pushmlo <r0>			/* Save registers */

_z16f_xmitc1:
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	ld	r0,#4
	tm.b	Z16F_UART0_STAT0,r0
	jp	eq, _z16f_xmitc1 	/* While (!(Z16F_UART0_STAT0 & %4)) */
	ld.b	Z16F_UART0_TXD,r1	/* Z16F_UART0_TXD = r1 (character) */
#else
	ld	r0,#4
	tm.b	Z16F_UART1_STAT0,r0
	jp	eq, _z16f_xmitc1	/* While (!(Z16F_UART1_STAT0 & %4)) */
	ld.b	Z16F_UART1_TXD,r1	/* Z16F_UART1_TXD = r1 (character) */
#endif
	popmlo	<r0>			/* Restore registers */
	ret				/* Return */
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
	ld	r0,r1
	ext.ub	r5,r0
	cp	r5,#10
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
	ld	r0, #%80
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	tm.b	Z16F_UART0_STAT0,r0
	jp	eq, _up_lowgetc1		/* While (!Z16F_UART0_STAT0 & %80)) */
	ld.ub	r0, Z16F_UART0_RXD		/* r0 = Z16F_UART0_RXD */
#else
	tm.b	Z16F_UART1_STAT0,r0		/* While (!Z16F_UART1_STAT0 & %80) */
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
