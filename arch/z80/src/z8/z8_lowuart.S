/*************************************************************************
 * arch/z80/src/z8/z8_lowuart.asm
 * ez8 UART management
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
#include "common/up_internal.h"

#ifdef CONFIG_USE_LOWUARTINIT

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_z8_lowuartinit
#ifdef CONFIG_Z80_LOWPUTC
	xdef	_up_lowputc
#endif
#ifdef CONFIG_Z80_LOWGETC
	xdef	_up_lowgetc
#endif

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=ROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: z8_lowuartinit
 *
 * Description:
 *   Initialize UART0 or UART1
 *
 * Parameters:
 *   None
 *
 *************************************************************************/

_z8_lowuartinit:
#endif /* CONFIG_USE_LOWUARTINIT */

/*************************************************************************
 * Name: _up_lowputc
 *
 * Description:
 *   Send one character to the selected serial console
 *
 * Parmeters:
 *   r1 = character
 *
 * Return:
 *   None
 *
 * Modifies r0 (and maybe r1)
 *
 *************************************************************************/

#ifdef CONFIG_Z80_LOWPUTC
_up_lowputc:
	/* Check if the character to output is a linefeed */


	/* Output a carriage return before the linefeed */


/*************************************************************************
 * Name: _z8_xmitc
 *
 * Description:
 *   Send one character on the selected port (really a part of up_lowputc)
 *
 * Parameters:
 *   r1 = character
 *
 * Return:
 *   None
 *
 * Modifies r0
 *
 *************************************************************************/

_z8_xmitc:
_z8_xmitc1:
#ifdef CONFIG_UART1_SERIAL_CONSOLE
#else
#endif
	ret					/* Return */

#endif /* CONFIG_Z80_LOWPUTC */

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

#ifdef CONFIG_Z80_LOWGETC
_up_lowgetc:
_up_lowgetc1:
#ifdef CONFIG_UART1_SERIAL_CONSOLE
#else
#endif
#endif

	end

