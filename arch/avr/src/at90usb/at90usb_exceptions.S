/********************************************************************************************
 * arch/avr/src/at90usb/at90usb_exceptions.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "excptmacros.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * External Symbols
 ********************************************************************************************/

	.file	"at90usb_exceptions.S"
	.global	up_doirq

/********************************************************************************************
 * Macros
 ********************************************************************************************/

/********************************************************************************************
 * Exception Vector Handlers
 ********************************************************************************************/

	.section .handlers, "ax", @progbits

	HANDLER	at90usb_int0, AT90USB_IRQ_INT0, excpt_common		/* External interrupt request 0 */
	HANDLER	at90usb_int1, AT90USB_IRQ_INT1, excpt_common		/* External interrupt request 1 */
	HANDLER	at90usb_int2, AT90USB_IRQ_INT2, excpt_common		/* External interrupt request 2 */
	HANDLER	at90usb_int3, AT90USB_IRQ_INT3, excpt_common		/* External interrupt request 3 */
	HANDLER	at90usb_int4, AT90USB_IRQ_INT4, excpt_common		/* External interrupt request 4 */
	HANDLER	at90usb_int5, AT90USB_IRQ_INT5, excpt_common		/* External interrupt request 5 */
	HANDLER	at90usb_int6, AT90USB_IRQ_INT6, excpt_common		/* External interrupt request 6 */
	HANDLER	at90usb_int7, AT90USB_IRQ_INT7, excpt_common		/* External interrupt request 7 */
	HANDLER	at90usb_pcint0, AT90USB_IRQ_PCINT0, excpt_common	/* Pin Change Interrupt Request 0 */
	HANDLER	at90usb_usbgen, AT90USB_IRQ_USBGEN, excpt_common	/* USB General USB General Interrupt request */
	HANDLER	at90usb_usbep, AT90USB_IRQ_USBEP, excpt_common		/* USB Endpoint/Pipe USB ENdpoint/Pipe Interrupt request */
	HANDLER	at90usb_wdt, AT90USB_IRQ_WDT, excpt_common			/* Watchdog Time-out Interrupt */
	HANDLER	at90usb_t2compa, AT90USB_IRQ_T2COMPA, excpt_common	/* TIMER2 COMPA Timer/Counter2 Compare Match A */
	HANDLER	at90usb_t2compb, AT90USB_IRQ_T2COMPB, excpt_common	/* TIMER2 COMPA Timer/Counter2 Compare Match B */
	HANDLER	at90usb_t2ovf, AT90USB_IRQ_T2OVF, excpt_common		/* TIMER2 OVF timer/counter2 overflow */
	HANDLER	at90usb_t1capt, AT90USB_IRQ_T1CAPT, excpt_common	/* TIMER1 CAPT timer/counter1 capture event */
	HANDLER	at90usb_t1compa, AT90USB_IRQ_T1COMPA, excpt_common	/* TIMER1 COMPA timer/counter1 compare match A */
	HANDLER	at90usb_t1compb, AT90USB_IRQ_T1COMPB, excpt_common	/* TIMER1 COMPB timer/counter1 compare match B */
	HANDLER	at90usb_t1compc, AT90USB_IRQ_T1COMPC, excpt_common	/* TIMER1 COMPC timer/counter1 compare match C */
	HANDLER	at90usb_t1ovf, AT90USB_IRQ_T1OVF, excpt_common		/* TIMER1 OVF timer/counter1 overflow */
	HANDLER	at90usb_t0compa, AT90USB_IRQ_T0COMPA, excpt_common	/* TIMER0 COMPA Timer/Counter0 Compare Match A */
	HANDLER	at90usb_t0compb, AT90USB_IRQ_T0COMPB, excpt_common	/* TIMER0 COMPB Timer/Counter0 Compare Match B */
	HANDLER	at90usb_t0ovf, AT90USB_IRQ_T0OVF, excpt_common		/* TIMER0 OVF timer/counter0 overflow */
	HANDLER	at90usb_spi, AT90USB_IRQ_SPI, excpt_common			/* STC SPI serial transfer complete */
	HANDLER	at90usb_u1rx, AT90USB_IRQ_U1RX, excpt_common		/* USART1 RX complete */
	HANDLER	at90usb_u1dre, AT90USB_IRQ_U1DRE, excpt_common		/* USART1 data register empty */
	HANDLER	at90usb_u1tx, AT90USB_IRQ_U1TX, excpt_common		/* USART1 TX complete */
	HANDLER	at90usb_anacomp, AT90USB_IRQ_ANACOMP, excpt_common	/* ANALOG COMP analog comparator */
	HANDLER	at90usb_adc, AT90USB_IRQ_ADC, excpt_common			/* ADC conversion complete */
	HANDLER	at90usb_ee, AT90USB_IRQ_EE, excpt_common			/* EEPROM ready */
	HANDLER	at90usb_t3capt, AT90USB_IRQ_T3CAPT, excpt_common	/* TIMER3 CAPT timer/counter3 capture event */
	HANDLER	at90usb_t3compa, AT90USB_IRQ_T3COMPA, excpt_common	/* TIMER3 COMPA timer/counter3 compare match a */
	HANDLER	at90usb_t3compb, AT90USB_IRQ_T3COMPB, excpt_common	/* TIMER3 COMPB timer/counter3 compare match b */
	HANDLER	at90usb_t3compc, AT90USB_IRQ_T3COMPC, excpt_common	/* TIMER3 COMPC timer/counter3 compare match c */
	HANDLER	at90usb_t3ovf, AT90USB_IRQ_T3OVF, excpt_common		/* TIMER3 OVF timer/counter3 overflow */
	HANDLER	at90usb_twi, AT90USB_IRQ_TWI, excpt_common			/* TWI two-wire serial interface */
	HANDLER	at90usb_spmrdy, AT90USB_IRQ_SPMRDY, excpt_common	/* Store program memory ready */

/* Common exception handling logic. */

excpt_common:
#warning "Missing Logic"

/****************************************************************************************************
 *  Name: up_interruptstack
 ****************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 0
	.bss
	.align	4
	.globl	up_interruptstack
	.type	up_interruptstack, object
up_interruptstack:
	.skip	CONFIG_ARCH_INTERRUPTSTACK
.Lintstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif
	.end

