/********************************************************************************************
 * arch/avr/src/atmega/atmega_exceptions.S
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * External Symbols
 ********************************************************************************************/

	.file	"atmega_exceptions.S"
	.global	up_doirq
	.global	up_fullcontextrestore

/********************************************************************************************
 * Macros
 ********************************************************************************************/

/********************************************************************************************
 * Exception Vector Handlers
 ********************************************************************************************/

	.section .handlers, "ax", @progbits

#if defined(CONFIG_ARCH_CHIP_ATMEGA128)
	HANDLER	atmega_int0, ATMEGA_IRQ_INT0, excpt_common					/* External interrupt request 0 */
	HANDLER	atmega_int1, ATMEGA_IRQ_INT1, excpt_common					/* External interrupt request 1 */
	HANDLER	atmega_int2, ATMEGA_IRQ_INT2, excpt_common					/* External interrupt request 2 */
	HANDLER	atmega_int3, ATMEGA_IRQ_INT3, excpt_common					/* External interrupt request 3 */
	HANDLER	atmega_int4, ATMEGA_IRQ_INT4, excpt_common					/* External interrupt request 4 */
	HANDLER	atmega_int5, ATMEGA_IRQ_INT5, excpt_common					/* External interrupt request 5 */
	HANDLER	atmega_int6, ATMEGA_IRQ_INT6, excpt_common					/* External interrupt request 6 */
	HANDLER	atmega_int7, ATMEGA_IRQ_INT7, excpt_common					/* External interrupt request 7 */
	HANDLER	atmega_t2comp, ATMEGA_IRQ_T2COMP, excpt_common				/* TIMER2 COMP timer/counter2 compare match */
	HANDLER	atmega_t2ovf, ATMEGA_IRQ_T2OVF, excpt_common				/* TIMER2 OVF timer/counter2 overflow */
	HANDLER	atmega_t1capt, ATMEGA_IRQ_T1CAPT, excpt_common				/* TIMER1 CAPT timer/counter1 capture event */
	HANDLER	atmega_t1compa, ATMEGA_IRQ_T1COMPA, excpt_common			/* TIMER1 COMPA timer/counter1 compare match a */
	HANDLER	atmega_t1compb, ATMEGA_IRQ_T1COMPB, excpt_common			/* TIMER1 COMPB timer/counter1 compare match b */
	HANDLER	atmega_t1ovf, ATMEGA_IRQ_T1OVF, excpt_common				/* TIMER1 OVF timer/counter1 overflow */
	HANDLER	atmega_t0comp, ATMEGA_IRQ_T0COMP, excpt_common				/* TIMER0 COMP timer/counter0 compare match */
	HANDLER	atmega_t0ovf, ATMEGA_IRQ_T0OVF, excpt_common				/* TIMER0 OVF timer/counter0 overflow */
	HANDLER	atmega_spi, ATMEGA_IRQ_SPI, excpt_common					/* STC SPI serial transfer complete */
	HANDLER	atmega_u0rx, ATMEGA_IRQ_U0RX, excpt_common					/* USART0 RX complete */
	HANDLER	atmega_u0dre, ATMEGA_IRQ_U0DRE, excpt_common				/* USART0 data register empty */
	HANDLER	atmega_u0tx, ATMEGA_IRQ_U0TX, excpt_common					/* USART0 TX complete */
	HANDLER	atmega_adc, ATMEGA_IRQ_ADC, excpt_common					/* ADC conversion complete */
	HANDLER	atmega_ee, ATMEGA_IRQ_EE, excpt_common						/* EEPROM ready */
	HANDLER	atmega_anacomp, ATMEGA_IRQ_ANACOMP, excpt_common			/* ANALOG COMP analog comparator */
	HANDLER	atmega_t1compc, ATMEGA_IRQ_T1COMPC, excpt_common			/* TIMER1 COMPC timer/countre1 compare match c */
	HANDLER	atmega_t3capt, ATMEGA_IRQ_T3CAPT, excpt_common				/* TIMER3 CAPT timer/counter3 capture event */
	HANDLER	atmega_t3compa, ATMEGA_IRQ_T3COMPA, excpt_common			/* TIMER3 COMPA timer/counter3 compare match a */
	HANDLER	atmega_t3compb, ATMEGA_IRQ_T3COMPB, excpt_common			/* TIMER3 COMPB timer/counter3 compare match b */
	HANDLER	atmega_t3compc, ATMEGA_IRQ_T3COMPC, excpt_common			/* TIMER3 COMPC timer/counter3 compare match c */
	HANDLER	atmega_t3ovf, ATMEGA_IRQ_T3OVF, excpt_common				/* TIMER3 OVF timer/counter3 overflow */
	HANDLER	atmega_u1rx, ATMEGA_IRQ_U1RX, excpt_common					/* USART1 RX complete */
	HANDLER	atmega_u1dre, ATMEGA_IRQ_U1DRE, excpt_common				/* USART1 data register empty */
	HANDLER	atmega_u1tx, ATMEGA_IRQ_U1TX, excpt_common					/* USART1 TX complete */
	HANDLER	atmega_twi, ATMEGA_IRQ_TWI, excpt_common					/* TWI two-wire serial interface */
	HANDLER	atmega_spmrdy, ATMEGA_IRQ_SPMRDY, excpt_common				/* Store program memory ready */
#elif defined(CONFIG_ARCH_CHIP_ATMEGA1284P)
	HANDLER	atmega_int0, ATMEGA_IRQ_INT0, excpt_common					/* External interrupt request 0 */
	HANDLER	atmega_int1, ATMEGA_IRQ_INT1, excpt_common					/* External interrupt request 1 */
	HANDLER	atmega_int2, ATMEGA_IRQ_INT2, excpt_common					/* External interrupt request 2 */
	HANDLER	atmega_pcint0, ATMEGA_IRQ_PCINT0, excpt_common				/* Pin change interrupt request 0 */
	HANDLER	atmega_pcint1, ATMEGA_IRQ_PCINT1, excpt_common				/* Pin change interrupt request 1 */
	HANDLER	atmega_pcint2, ATMEGA_IRQ_PCINT2, excpt_common				/* Pin change interrupt request 2 */
	HANDLER	atmega_pcint3, ATMEGA_IRQ_PCINT3, excpt_common				/* Pin change interrupt request 3 */
	HANDLER	atmega_wdt, ATMEGA_IRQ_WDT, excpt_common					/* Watchdog time-out interrupt */
	HANDLER	atmega_t2compa, ATMEGA_IRQ_T2COMPA, excpt_common			/* TIMER2 COMPA timer/counter2 compare match */
	HANDLER	atmega_t2compb, ATMEGA_IRQ_T2COMPB, excpt_common			/* TIMER2 COMPB timer/counter2 compare match */
	HANDLER	atmega_t2ovf, ATMEGA_IRQ_T2OVF, excpt_common				/* TIMER2 OVF timer/counter2 overflow */
	HANDLER	atmega_t1capt, ATMEGA_IRQ_T1CAPT, excpt_common				/* TIMER1 CAPT timer/counter1 capture event */
	HANDLER	atmega_t1compa, ATMEGA_IRQ_T1COMPA, excpt_common			/* TIMER1 COMPA timer/counter1 compare match a */
	HANDLER	atmega_t1compb, ATMEGA_IRQ_T1COMPB, excpt_common			/* TIMER1 COMPB timer/counter1 compare match b */
	HANDLER	atmega_t1ovf, ATMEGA_IRQ_T1OVF, excpt_common				/* TIMER1 OVF timer/counter1 overflow */
	HANDLER	atmega_t0compa, ATMEGA_IRQ_T0COMPA, excpt_common			/* TIMER0 COMPA timer/counter0 compare match */
	HANDLER	atmega_t0compb, ATMEGA_IRQ_T0COMPB, excpt_common			/* TIMER0 COMPB timer/counter0 compare match */
	HANDLER	atmega_t0ovf, ATMEGA_IRQ_T0OVF, excpt_common				/* TIMER0 OVF timer/counter0 overflow */
	HANDLER	atmega_spi, ATMEGA_IRQ_SPI, excpt_common					/* STC SPI serial transfer complete */
	HANDLER	atmega_u0rx, ATMEGA_IRQ_U0RX, excpt_common					/* USART0 RX complete */
	HANDLER	atmega_u0dre, ATMEGA_IRQ_U0DRE, excpt_common				/* USART0 data register empty */
	HANDLER	atmega_u0tx, ATMEGA_IRQ_U0TX, excpt_common					/* USART0 TX complete */
	HANDLER	atmega_anacomp, ATMEGA_IRQ_ANACOMP, excpt_common			/* ANALOG COMP analog comparator */
	HANDLER	atmega_adc, ATMEGA_IRQ_ADC, excpt_common					/* ADC conversion complete */
	HANDLER	atmega_ee, ATMEGA_IRQ_EE, excpt_common						/* EEPROM ready */
	HANDLER	atmega_twi , ATMEGA_IRQ_TWI, excpt_common					/* TWI two-wire serial interface */
	HANDLER	atmega_spmrdy, ATMEGA_IRQ_SPMRDY, excpt_common				/* Store program memory ready */
	HANDLER	atmega_u1rx, ATMEGA_IRQ_U1RX, excpt_common					/* USART1 RX complete */
	HANDLER	atmega_u1dre, ATMEGA_IRQ_U1DRE, excpt_common				/* USART1 data register empty */
	HANDLER	atmega_u1tx, ATMEGA_IRQ_U1TX, excpt_common					/* USART1 TX complete */
	HANDLER	atmega_t3capt, ATMEGA_IRQ_T3CAPT, excpt_common				/* TIMER3 CAPT timer/counter3 capture event */
	HANDLER	atmega_t3compa, ATMEGA_IRQ_T3COMPA, excpt_common			/* TIMER3 COMPA timer/counter3 compare match a */
	HANDLER	atmega_t3compb, ATMEGA_IRQ_T3COMPB, excpt_common			/* TIMER3 COMPB timer/counter3 compare match b */
	HANDLER	atmega_t3ovf, ATMEGA_IRQ_T3OVF, excpt_common				/* TIMER3 OVF timer/counter3 overflow */
#elif defined(CONFIG_ARCH_CHIP_ATMEGA2560)
	HANDLER	atmega_int0, ATMEGA_IRQ_INT0, excpt_common					/* 0x0002 INT0 External Interrupt Request 0 */
	HANDLER	atmega_int1, ATMEGA_IRQ_INT1, excpt_common					/* 0x0004 INT1 External Interrupt Request 1 */
	HANDLER	atmega_int2, ATMEGA_IRQ_INT2, excpt_common					/* 0x0006 INT2 External Interrupt Request 2 */
	HANDLER	atmega_int3, ATMEGA_IRQ_INT3, excpt_common					/* 0x0008 INT3 External Interrupt Request 3 */
	HANDLER	atmega_int4, ATMEGA_IRQ_INT4, excpt_common					/* 0x000A INT4 External Interrupt Request 4 */
	HANDLER	atmega_int5, ATMEGA_IRQ_INT5, excpt_common					/* 0x000C INT5  External Interrupt Request 5 */
	HANDLER	atmega_int6, ATMEGA_IRQ_INT6, excpt_common					/* 0x000E INT6 External Interrupt Request 6 */
	HANDLER	atmega_int7, ATMEGA_IRQ_INT7, excpt_common					/* 0x0010 INT7 External Interrupt Request 7 */
	HANDLER	atmega_pcint0, ATMEGA_IRQ_PCINT0, excpt_common				/* 0x0012 PCINT0 Pin Change Interrupt Req 0 */
	HANDLER	atmega_pcint1, ATMEGA_IRQ_PCINT1, excpt_common				/* 0x0014 PCINT1 Pin Change Interrupt Req 1 */
	HANDLER	atmega_pcint2, ATMEGA_IRQ_PCINT2, excpt_common				/* 0x0016 PCINT2 Pin Change Interrupt Req 2 */
	HANDLER	atmega_wdt, ATMEGA_IRQ_WDT, excpt_common					/* 0x0018 WDT Watchdog Time-out Interrupt */
	HANDLER	atmega_tim2_compa, ATMEGA_IRQ_TIM2_COMPA, excpt_common		/* 0x001A TIMER2 COMPA Timer/Counter2 Compare Match A */
	HANDLER	atmega_tim2_compb, ATMEGA_IRQ_TIM2_COMPB, excpt_common		/* 0x001C TIMER2 COMPB Timer/Counter2 Compare Match B */
	HANDLER	atmega_tim2_ovf, ATMEGA_IRQ_TIM2_OVF, excpt_common			/* 0x001E TIMER2 OVF Timer/Counter2 Overflow */
	HANDLER	atmega_tim1_capt, ATMEGA_IRQ_TIM1_CAPT, excpt_common		/* 0x0020 TIMER1 CAPT Timer/Counter1 Capture Event */
	HANDLER	atmega_tim1_compa, ATMEGA_IRQ_TIM1_COMPA, excpt_common		/* 0x0022 TIMER1 COMPA Timer/Counter1 Compare Match A */
	HANDLER	atmega_tim1_compb, ATMEGA_IRQ_TIM1_COMPB, excpt_common		/* 0x0024 TIMER1 COMPB Timer/Counter1 Compare Match B */
	HANDLER	atmega_tim1_compc, ATMEGA_IRQ_TIM1_COMPC, excpt_common		/* 0x0026 TIMER1 COMPC Timer/Counter1 Compare Match C */
	HANDLER	atmega_tim1_ovf, ATMEGA_IRQ_TIM1_OVF, excpt_common			/* 0x0028 TIMER1 OVF Timer/Counter1 Overflow */
	HANDLER	atmega_tim0_compa, ATMEGA_IRQ_TIM0_COMPA, excpt_common		/* 0x002A TIMER0 COMPA Timer/Counter0 Compare Match A */
	HANDLER	atmega_tim0_compb, ATMEGA_IRQ_TIM0_COMPB, excpt_common		/* 0x002C TIMER0 COMPB Timer/Counter0 Compare match B */
	HANDLER	atmega_tim0_ovf, ATMEGA_IRQ_TIM0_OVF, excpt_common			/* 0x002E TIMER0 OVF Timer/Counter0 Overflow */
	HANDLER	atmega_spi_stc, ATMEGA_IRQ_SPI_STC, excpt_common			/* 0x0030 SPI, STC SPI Serial Transfer Complete */
	HANDLER	atmega_u0rx, ATMEGA_IRQ_U0RX, excpt_common					/* 0x0032 USART0 RX USART0 Rx Complete */
	HANDLER	atmega_u0dre, ATMEGA_IRQ_U0DRE, excpt_common				/* 0x0034 USART0 UDRE USART0 Data Register Empty */
	HANDLER	atmega_u0tx, ATMEGA_IRQ_U0TX, excpt_common					/* 0x0036 USART0 TX USART0 Tx Complete */
	HANDLER	atmega_ana_comp, ATMEGA_IRQ_ANA_COMP, excpt_common			/* 0x0038 ANALOG COMP Analog Comparator */
	HANDLER	atmega_adc, ATMEGA_IRQ_ADC, excpt_common			 		/* 0x003A ADC ADC Conversion Complete */
	HANDLER	atmega_ee_rdy, ATMEGA_IRQ_EE_RDY, excpt_common				/* 0x003C EE READY EEPROM Ready */
	HANDLER	atmega_tim3_capt, ATMEGA_IRQ_TIM3_CAPT, excpt_common		/* 0x003E TIMER3 CAPT Timer/Counter3 Capture Event */
	HANDLER	atmega_tim3_compa, ATMEGA_IRQ_TIM3_COMPA, excpt_common		/* 0x0040 TIMER3 COMPA Timer/Counter3 Compare Match A */
	HANDLER	atmega_tim3_compb, ATMEGA_IRQ_TIM3_COMPB, excpt_common		/* 0x0042 TIMER3 COMPB Timer/Counter3 Compare Match B */
	HANDLER	atmega_tim3_compc, ATMEGA_IRQ_TIM3_COMPC, excpt_common		/* 0x0044 TIMER3 COMPC Timer/Counter3 Compare Match C */
	HANDLER	atmega_tim3_ovf, ATMEGA_IRQ_TIM3_OVF, excpt_common			/* 0x0046 TIMER3 OVF Timer/Counter3 Overflow */
	HANDLER	atmega_u1rx, ATMEGA_IRQ_U1RX, excpt_common					/* 0x0048 USART1 RX USART1 Rx Complete */
	HANDLER	atmega_u1dre, ATMEGA_IRQ_U1DRE, excpt_common				/* 0x004A USART1 UDRE USART1 Data Register Empty */
	HANDLER	atmega_u1tx, ATMEGA_IRQ_U1TX, excpt_common					/* 0x004C USART1 TX USART1 Tx Complete */
	HANDLER	atmega_twi, ATMEGA_IRQ_TWI, excpt_common					/* 0x004E TWI 2-wire Serial Interface */
	HANDLER	atmega_spm_rdy, ATMEGA_IRQ_SPM_RDY, excpt_common			/* 0x0050 SPM READY Store Program Memory Ready */
	HANDLER	atmega_tim4_capt, ATMEGA_IRQ_TIM4_CAPT, excpt_common		/* 0x0052 TIMER4 CAPT Timer/Counter4 Capture Event */
	HANDLER	atmega_tim4_compa, ATMEGA_IRQ_TIM4_COMPA, excpt_common		/* 0x0054 TIMER4 COMPA Timer/Counter4 Compare Match A */
	HANDLER	atmega_tim4_compb, ATMEGA_IRQ_TIM4_COMPB, excpt_common		/* 0x0056 TIMER4 COMPB Timer/Counter4 Compare Match B */
	HANDLER	atmega_tim4_compc, ATMEGA_IRQ_TIM4_COMPC, excpt_common		/* 0x0058 TIMER4 COMPC Timer/Counter4 Compare Match C */
	HANDLER	atmega_tim4_ovf, ATMEGA_IRQ_TIM4_OVF, excpt_common			/* 0x005A TIMER4 OVF Timer/Counter4 Overflow */
	HANDLER	atmega_tim5_capt, ATMEGA_IRQ_TIM5_CAPT, excpt_common		/* 0x005C TIMER5 CAPT Timer/Counter5 Capture Event */
	HANDLER	atmega_tim5_compa, ATMEGA_IRQ_TIM5_COMPA, excpt_common		/* 0x005E TIMER5 COMPA Timer/Counter5 Compare Match A */
	HANDLER	atmega_tim5_compb, ATMEGA_IRQ_TIM5_COMPB, excpt_common		/* 0x0060 TIMER5 COMPB Timer/Counter5 Compare Match B */
	HANDLER	atmega_tim5_compc, ATMEGA_IRQ_TIM5_COMPC, excpt_common		/* 0x0062 TIMER5 COMPC Timer/Counter5 Compare Match C */
	HANDLER	atmega_tim5_ovf, ATMEGA_IRQ_TIM5_OVF, excpt_common			/* 0x0064 TIMER5 OVF Timer/Counter5 Overflow */
	HANDLER	atmega_u2rx, ATMEGA_IRQ_U2RX, excpt_common					/* 0x0066 USART1 RX USART2 Rx Complete */
	HANDLER	atmega_u2dre, ATMEGA_IRQ_U2DRE, excpt_common				/* 0x0068 USART1 UDRE USART2 Data Register Empty */
	HANDLER	atmega_u2tx, ATMEGA_IRQ_U2TX, excpt_common					/* 0x006A USART1 TX USART2 Tx Complete */
	HANDLER	atmega_u3rx, ATMEGA_IRQ_U3RX, excpt_common					/* 0x006C USART1 RX USART3 Rx Complete */
	HANDLER	atmega_u3dre, ATMEGA_IRQ_U3DRE, excpt_common				/* 0x006E USART1 UDRE USART3 Data Register Empty */
	HANDLER	atmega_u3tx, ATMEGA_IRQ_U3TX, excpt_common					/* 0x0070 USART1 TX USART3 Tx Complete */
#else
#  error "Unrecognized chip"
#endif

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

	/* Call up_doirq with r24 = IRQ number, r22-23 = Pointer to the save structure.  The stack
	 * pointer currently points to the save structure (or maybe the save structure -1 since
	 * the push operation post-decrements -- need to REVISIT this).
	 */

	in		r28, _SFR_IO_ADDR(SPL)	/* Get the save structure pointer in a Call-saved register pair */
	in		r29, _SFR_IO_ADDR(SPH)	/* Pointer can be obtained from the stack pointer */
	adiw	r28, 1					/* Remembering that push post-decrements */
	movw	r22, r28				/* Pass register save structure as the parameter 2 */
	USE_INTSTACK rx, ry, rz			/* Switch to the interrupt stack */
	call	up_doirq				/* Dispatch the interrupt */
	RESTORE_STACK rx, ry			/* Undo the operations of USE_INTSTACK */

	/* up_doiq returns with r24-r25 equal to the new save structure.  If no context
	 * switch occurred, this will be the same as the value passed to it in r22-23.
	 * But if a context switch occurs, then the returned value will point not at a
	 * stack frame, but at a register save area inside of the new task's TCB.
	 */

	 cp		r28, r24
	 cpc	r29, r25
	 breq	.Lnoswitch

	/* A context switch has occurred, jump to up_fullcontextrestore with r24, r25
	 * equal to the address of the new register save ared.
	 */

	jmp	up_fullcontextrestore

	/* No context switch occurred.. just return off the stack */

.Lnoswitch:
	EXCPT_EPILOGUE
	reti

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
