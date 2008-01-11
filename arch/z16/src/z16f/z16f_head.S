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
#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_ARCH_LOWGETC)
	xref _z16f_lowuartinit:EROM
#endif
	xref _os_start:EROM
	xdef _reset
	xdef _sysexc_isr
	xdef _timer2_isr
	xdef _timer1_isr
	xdef _timer0_isr
	xdef _uart0rx_isr
	xdef _uart0tx_isr
	xdef _i2c_isr
	xdef _spi_isr
	xdef _adc_isr
	xdef _p7ad_isr
	xdef _p6ad_isr
	xdef _p5ad_isr
	xdef _p4ad_isr
	xdef _p3ad_isr
	xdef _p2ad_isr
	xdef _p1ad_isr
	xdef _p0ad_isr
	xdef _pwmtimer_isr
	xdef _uart1rx_isr
	xdef _uart1tx_isr
	xdef _pwmfault_isr
	xdef _c3_isr
	xdef _c2_isr
	xdef _c1_isr
	xdef _c0_isr
	xdef _common_isr
 
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
	vector SYSEXC=_sysexc_isr
	vector TIMER2=_timer2_isr
	vector TIMER1=_timer1_isr
	vector TIMER0=_timer0_isr
	vector UART0_RX=_uart0rx_isr
	vector UART0_TX=_uart0tx_isr
	vector I2C=_i2c_isr
	vector SPI=_spi_isr
	vector ADC=_adc_isr
	vector P7AD=_p7ad_isr
	vector P6AD=_p6ad_isr
	vector P5AD=_p5ad_isr
	vector P4AD=_p4ad_isr
	vector P3AD=_p3ad_isr
	vector P2AD=_p2ad_isr
	vector P1AD=_p1ad_isr
	vector P0AD=_p0ad_isr
	vector PWM_TIMER=_pwmtimer_isr
	vector UART1_RX=_uart1rx_isr
	vector UART1_TX=_uart1tx_isr
	vector PWM_FAULT=_pwmfault_isr
	vector C3=_c3_isr
	vector C2=_c3_isr
	vector C1=_c2_isr
	vector C0=_c0_isr
 
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
	ld	sp, #STACK_TOP		/* Set Stack Pointer */
	call	_z16f_lowinit		/* Perform low-level hardware initialization */
#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_ARCH_LOWGETC)
	call	_z16f_lowuartinit	/* Initialize the UART for debugging */
#endif
	call	_os_start		/* Start the operating system */
_halted:				/* _os_start() should not return */
	halt
	jp	_halted

/**************************************************************************
 * Name: Interrupt handlers
 *
 * Description:
 *   All interrupts will be vectored to the following locations.
 *
 **************************************************************************/

_sysexc_isr:
_timer2_isr:
_timer1_isr:
_timer0_isr:
_uart0rx_isr:
_uart0tx_isr:
_i2c_isr:
_spi_isr:
_adc_isr:
_p7ad_isr:
_p6ad_isr:
_p5ad_isr:
_p4ad_isr:
_p3ad_isr:
_p2ad_isr:
_p1ad_isr:
_p0ad_isr:
_pwmtimer_isr:
_uart1rx_isr:
_uart1tx_isr:
_pwmfault_isr:
_c3_isr:
_c2_isr:
_c1_isr:
_c0_isr:
_common_isr:
	nop
	iret

	end
