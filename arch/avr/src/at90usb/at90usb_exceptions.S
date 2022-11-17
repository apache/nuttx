/********************************************************************************************
 * arch/avr/src/at90usb/at90usb_exceptions.S
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
	.global	avr_doirq
	.global	avr_fullcontextrestore

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

/********************************************************************************************
 * Name: excpt_common
 *
 * Description:
 *   Exception Vector Handlers
 *
 * On Entry:
 *	The return PC and the saved r24 is on the stack, r24 now contains the IRQ number
 *
 *	  PC1
 *	  PC0
 *	  R0
 *	  --- <- SP
 *
 ********************************************************************************************/

excpt_common:
	/* Save the remaining registers on the stack, preserving the IRQ number in r14 */

	EXCPT_PROLOGUE

	/* Call avr_doirq with r24 = IRQ number, r22-23 = Pointer to the save structure.  The stack
	 * pointer currently points to the save structure (or maybe the save structure -1 since
	 * the push operation post-decrements -- need to REVISIT this).
	 */

	in		r28, _SFR_IO_ADDR(SPL)	/* Get the save structure pointer in a Call-saved register pair */
	in		r29, _SFR_IO_ADDR(SPH)  /* Pointer can be obtained from the stack pointer */
	adiw	r28, 1					/* Remembering that push post-decrements */
	movw	r22, r28				/* Pass register save structure as the parameter 2 */
	USE_INTSTACK rx, ry, rz			/* Switch to the interrupt stack */
	call	avr_doirq				/* Dispatch the interrupt */
	RESTORE_STACK rx, ry			/* Undo the operations of USE_INTSTACK */

	/* up_doiq returns with r24-r25 equal to the new save structure.  If no context
	 * switch occurred, this will be the same as the value passed to it in r22-23.
	 * But if a context switch occurs, then the returned value will point not at a
	 * stack frame, but at a register save area inside of the new task's TCB.
	 */

	 cp		r28, r24
	 cpc	r29, r25
	 breq	.Lnoswitch

	/* A context switch has occurred, jump to avr_fullcontextrestore with r24, r25
	 * equal to the address of the new register save area.
	 */

	jmp	avr_fullcontextrestore

	/* No context switch occurred.. just return off the stack */

.Lnoswitch:
	EXCPT_EPILOGUE
	reti

/****************************************************************************************************
 *  Name: g_intstackalloc
 ****************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.align	4
	.globl	g_intstackalloc
	.type	g_intstackalloc, object
	.globl	g_intstacktop
	.type	g_intstacktop, object
g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
#endif
	.end
