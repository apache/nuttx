/**************************************************************************
 * arch/z16/src/z16f/z16f_head.S
 * Z16F Reset Entry Point
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

/**************************************************************************
 * External References / External Definitions
 **************************************************************************/

	xref _z16f_lowinit:EROM
	xref _os_start:EROM
	xdef _default_handler
	xdef _reset
 
/**************************************************************************
 * Flash Option Byte Setup
 **************************************************************************/

	define FOPTIONSEG, SPACE=ROM, ORG=0
	segment FOPTIONSEG
	db %FF
	db %FF
	db %FF
	db %FF

/**************************************************************************
 * vectors
 **************************************************************************/

	vector RESET=_reset
	vector SYSEXC=_default_handler
	vector TIMER2=_default_handler
	vector TIMER1=_default_handler
	vector TIMER0=_default_handler
	vector UART0_RX=_default_handler
	vector UART0_TX=_default_handler
	vector I2C=_default_handler
	vector SPI=_default_handler
	vector ADC=_default_handler
	vector P7AD=_default_handler
	vector P6AD=_default_handler
	vector P5AD=_default_handler
	vector P4AD=_default_handler
	vector P3AD=_default_handler
	vector P2AD=_default_handler
	vector P1AD=_default_handler
	vector P0AD=_default_handler
	vector PWM_TIMER=_default_handler
	vector UART1_RX=_default_handler
	vector UART1_TX=_default_handler
	vector PWM_FAULT=_default_handler
	vector C3=_default_handler
	vector C2=_default_handler
	vector C1=_default_handler
	vector C0=_default_handler
 
/**************************************************************************
 * Equates
 **************************************************************************/

STACK_TOP     equ %FFC000

/**************************************************************************
 * Data Allocation
 **************************************************************************/
 
/**************************************************************************
 * Code
 **************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/**************************************************************************
 * Name: _reset
 *
 * Description:
 *   Reset entry point
 *
 **************************************************************************/

_reset:
	ld sp, #STACK_TOP	/* Set Stack Pointer */

	call _z16f_lowinit	/* call _z16f_lowinit */
	call _os_start          /* Start the operating system */
_halted:                        /* _os_start() should not return */
        halt
        jp      _halted

/**************************************************************************
 * Name: _default_handler
 *
 * Description:
 *   This is where interrupts will go if they have not yet been intialize
 *
 **************************************************************************/

_default_handler:
	nop
	iret

	end
