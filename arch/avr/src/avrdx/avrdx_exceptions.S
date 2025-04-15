/********************************************************************************************
 * arch/avr/src/avrdx/avrdx_exceptions.S
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

	.file	"avrdx_exceptions.S"
	.global	avr_doirq
	.global	avr_fullcontextrestore

/********************************************************************************************
 * Macros
 ********************************************************************************************/

/********************************************************************************************
 * Exception Vector Handlers
 ********************************************************************************************/

	.section .handlers, "ax", @progbits

#if defined(CONFIG_ARCH_CHIP_AVR128DA28)
	HANDLER avrdx_nmi, AVRDX_IRQ_NMI, excpt_common
	HANDLER avrdx_bod_vlm, AVRDX_IRQ_BOD_VLM, excpt_common
	HANDLER avrdx_rtc_cnt, AVRDX_IRQ_RTC_CNT, excpt_common
	HANDLER avrdx_rtc_pit, AVRDX_IRQ_RTC_PIT, excpt_common
	HANDLER avrdx_ccl_ccl, AVRDX_IRQ_CCL_CCL, excpt_common
	HANDLER avrdx_porta_port, AVRDX_IRQ_PORTA_PORT, excpt_common
	HANDLER avrdx_tca0_ovf, AVRDX_IRQ_TCA0_OVF, excpt_common
	HANDLER avrdx_tca0_hunf, AVRDX_IRQ_TCA0_HUNF, excpt_common
	HANDLER avrdx_tca0_cmp0, AVRDX_IRQ_TCA0_CMP0, excpt_common
	HANDLER avrdx_tca0_cmp1, AVRDX_IRQ_TCA0_CMP1, excpt_common
	HANDLER avrdx_tca0_cmp2, AVRDX_IRQ_TCA0_CMP2, excpt_common
	HANDLER avrdx_tcb0_int, AVRDX_IRQ_TCB0_INT, excpt_common
	HANDLER avrdx_tcb1_int, AVRDX_IRQ_TCB1_INT, excpt_common
	HANDLER avrdx_tcd0_ovf, AVRDX_IRQ_TCD0_OVF, excpt_common
	HANDLER avrdx_tcd0_trig, AVRDX_IRQ_TCD0_TRIG, excpt_common
	HANDLER avrdx_twi0_twis, AVRDX_IRQ_TWI0_TWIS, excpt_common
	HANDLER avrdx_twi0_twim, AVRDX_IRQ_TWI0_TWIM, excpt_common
	HANDLER avrdx_spi0_int, AVRDX_IRQ_SPI0_INT, excpt_common
	HANDLER avrdx_usart0_rxc, AVRDX_IRQ_USART0_RXC, excpt_common
	HANDLER avrdx_usart0_dre, AVRDX_IRQ_USART0_DRE, excpt_common
	HANDLER avrdx_usart0_txc, AVRDX_IRQ_USART0_TXC, excpt_common
	HANDLER avrdx_portd_port, AVRDX_IRQ_PORTD_PORT, excpt_common
	HANDLER avrdx_ac0_ac, AVRDX_IRQ_AC0_AC, excpt_common
	HANDLER avrdx_adc0_resrdy, AVRDX_IRQ_ADC0_RESRDY, excpt_common
	HANDLER avrdx_adc0_wcmp, AVRDX_IRQ_ADC0_WCMP, excpt_common
	HANDLER avrdx_zcd0_zcd, AVRDX_IRQ_ZCD0_ZCD, excpt_common
	HANDLER avrdx_ptc_ptc, AVRDX_IRQ_PTC_PTC, excpt_common
	HANDLER avrdx_ac1_ac, AVRDX_IRQ_AC1_AC, excpt_common
	HANDLER avrdx_portc_port, AVRDX_IRQ_PORTC_PORT, excpt_common
	HANDLER avrdx_tcb2_int, AVRDX_IRQ_TCB2_INT, excpt_common
	HANDLER avrdx_usart1_rxc, AVRDX_IRQ_USART1_RXC, excpt_common
	HANDLER avrdx_usart1_dre, AVRDX_IRQ_USART1_DRE, excpt_common
	HANDLER avrdx_usart1_txc, AVRDX_IRQ_USART1_TXC, excpt_common
	HANDLER avrdx_portf_port, AVRDX_IRQ_PORTF_PORT, excpt_common
	HANDLER avrdx_nvmctrl_ee, AVRDX_IRQ_NVMCTRL_EE, excpt_common
	HANDLER avrdx_spi1_int, AVRDX_IRQ_SPI1_INT, excpt_common
	HANDLER avrdx_usart2_rxc, AVRDX_IRQ_USART2_RXC, excpt_common
	HANDLER avrdx_usart2_dre, AVRDX_IRQ_USART2_DRE, excpt_common
	HANDLER avrdx_usart2_txc, AVRDX_IRQ_USART2_TXC, excpt_common
	HANDLER avrdx_ac2_ac, AVRDX_IRQ_AC2_AC, excpt_common
#elif defined(CONFIG_ARCH_CHIP_AVR128DA64)
	HANDLER avrdx_nmi, AVRDX_IRQ_NMI, excpt_common
	HANDLER avrdx_bod_vlm, AVRDX_IRQ_BOD_VLM, excpt_common
	HANDLER avrdx_rtc_cnt, AVRDX_IRQ_RTC_CNT, excpt_common
	HANDLER avrdx_rtc_pit, AVRDX_IRQ_RTC_PIT, excpt_common
	HANDLER avrdx_ccl_ccl, AVRDX_IRQ_CCL_CCL, excpt_common
	HANDLER avrdx_porta_port, AVRDX_IRQ_PORTA_PORT, excpt_common
	HANDLER avrdx_tca0_lunf, AVRDX_IRQ_TCA0_LUNF, excpt_common
	HANDLER avrdx_tca0_ovf, AVRDX_IRQ_TCA0_OVF, excpt_common
	HANDLER avrdx_tca0_hunf, AVRDX_IRQ_TCA0_HUNF, excpt_common
	HANDLER avrdx_tca0_cmp0, AVRDX_IRQ_TCA0_CMP0, excpt_common
	HANDLER avrdx_tca0_lcmp0, AVRDX_IRQ_TCA0_LCMP0, excpt_common
	HANDLER avrdx_tca0_cmp1, AVRDX_IRQ_TCA0_CMP1, excpt_common
	HANDLER avrdx_tca0_lcmp1, AVRDX_IRQ_TCA0_LCMP1, excpt_common
	HANDLER avrdx_tca0_cmp2, AVRDX_IRQ_TCA0_CMP2, excpt_common
	HANDLER avrdx_tca0_lcmp2, AVRDX_IRQ_TCA0_LCMP2, excpt_common
	HANDLER avrdx_tcb0_int, AVRDX_IRQ_TCB0_INT, excpt_common
	HANDLER avrdx_tcb1_int, AVRDX_IRQ_TCB1_INT, excpt_common
	HANDLER avrdx_tcd0_ovf, AVRDX_IRQ_TCD0_OVF, excpt_common
	HANDLER avrdx_tcd0_trig, AVRDX_IRQ_TCD0_TRIG, excpt_common
	HANDLER avrdx_twi0_twis, AVRDX_IRQ_TWI0_TWIS, excpt_common
	HANDLER avrdx_twi0_twim, AVRDX_IRQ_TWI0_TWIM, excpt_common
	HANDLER avrdx_spi0_int, AVRDX_IRQ_SPI0_INT, excpt_common
	HANDLER avrdx_usart0_rxc, AVRDX_IRQ_USART0_RXC, excpt_common
	HANDLER avrdx_usart0_dre, AVRDX_IRQ_USART0_DRE, excpt_common
	HANDLER avrdx_usart0_txc, AVRDX_IRQ_USART0_TXC, excpt_common
	HANDLER avrdx_portd_port, AVRDX_IRQ_PORTD_PORT, excpt_common
	HANDLER avrdx_ac0_ac, AVRDX_IRQ_AC0_AC, excpt_common
	HANDLER avrdx_adc0_resrdy, AVRDX_IRQ_ADC0_RESRDY, excpt_common
	HANDLER avrdx_adc0_wcmp, AVRDX_IRQ_ADC0_WCMP, excpt_common
	HANDLER avrdx_zcd0_zcd, AVRDX_IRQ_ZCD0_ZCD, excpt_common
	HANDLER avrdx_ptc_ptc, AVRDX_IRQ_PTC_PTC, excpt_common
	HANDLER avrdx_ac1_ac, AVRDX_IRQ_AC1_AC, excpt_common
	HANDLER avrdx_portc_port, AVRDX_IRQ_PORTC_PORT, excpt_common
	HANDLER avrdx_tcb2_int, AVRDX_IRQ_TCB2_INT, excpt_common
	HANDLER avrdx_usart1_rxc, AVRDX_IRQ_USART1_RXC, excpt_common
	HANDLER avrdx_usart1_dre, AVRDX_IRQ_USART1_DRE, excpt_common
	HANDLER avrdx_usart1_txc, AVRDX_IRQ_USART1_TXC, excpt_common
	HANDLER avrdx_portf_port, AVRDX_IRQ_PORTF_PORT, excpt_common
	HANDLER avrdx_nvmctrl_ee, AVRDX_IRQ_NVMCTRL_EE, excpt_common
	HANDLER avrdx_spi1_int, AVRDX_IRQ_SPI1_INT, excpt_common
	HANDLER avrdx_usart2_rxc, AVRDX_IRQ_USART2_RXC, excpt_common
	HANDLER avrdx_usart2_dre, AVRDX_IRQ_USART2_DRE, excpt_common
	HANDLER avrdx_usart2_txc, AVRDX_IRQ_USART2_TXC, excpt_common
	HANDLER avrdx_ac2_ac, AVRDX_IRQ_AC2_AC, excpt_common
	HANDLER avrdx_tcb3_int, AVRDX_IRQ_TCB3_INT, excpt_common
	HANDLER avrdx_twi1_twis, AVRDX_IRQ_TWI1_TWIS, excpt_common
	HANDLER avrdx_twi1_twim, AVRDX_IRQ_TWI1_TWIM, excpt_common
	HANDLER avrdx_portb_port, AVRDX_IRQ_PORTB_PORT, excpt_common
	HANDLER avrdx_porte_port, AVRDX_IRQ_PORTE_PORT, excpt_common
	HANDLER avrdx_tca1_lunf, AVRDX_IRQ_TCA1_LUNF, excpt_common
	HANDLER avrdx_tca1_ovf, AVRDX_IRQ_TCA1_OVF, excpt_common
	HANDLER avrdx_tca1_hunf, AVRDX_IRQ_TCA1_HUNF, excpt_common
	HANDLER avrdx_tca1_cmp0, AVRDX_IRQ_TCA1_CMP0, excpt_common
	HANDLER avrdx_tca1_lcmp0, AVRDX_IRQ_TCA1_LCMP0, excpt_common
	HANDLER avrdx_tca1_cmp1, AVRDX_IRQ_TCA1_CMP1, excpt_common
	HANDLER avrdx_tca1_lcmp1, AVRDX_IRQ_TCA1_LCMP1, excpt_common
	HANDLER avrdx_tca1_cmp2, AVRDX_IRQ_TCA1_CMP2, excpt_common
	HANDLER avrdx_tca1_lcmp2, AVRDX_IRQ_TCA1_LCMP2, excpt_common
	HANDLER avrdx_zcd1_zcd, AVRDX_IRQ_ZCD1_ZCD, excpt_common
	HANDLER avrdx_usart3_rxc, AVRDX_IRQ_USART3_RXC, excpt_common
	HANDLER avrdx_usart3_dre, AVRDX_IRQ_USART3_DRE, excpt_common
	HANDLER avrdx_usart3_txc, AVRDX_IRQ_USART3_TXC, excpt_common
	HANDLER avrdx_usart4_rxc, AVRDX_IRQ_USART4_RXC, excpt_common
	HANDLER avrdx_usart4_dre, AVRDX_IRQ_USART4_DRE, excpt_common
	HANDLER avrdx_usart4_txc, AVRDX_IRQ_USART4_TXC, excpt_common
	HANDLER avrdx_portg_port, AVRDX_IRQ_PORTG_PORT, excpt_common
	HANDLER avrdx_zcd2_zcd, AVRDX_IRQ_ZCD2_ZCD, excpt_common
	HANDLER avrdx_tcb4_int, AVRDX_IRQ_TCB4_INT, excpt_common
	HANDLER avrdx_usart5_rxc, AVRDX_IRQ_USART5_RXC, excpt_common
	HANDLER avrdx_usart5_dre, AVRDX_IRQ_USART5_DRE, excpt_common
	HANDLER avrdx_usart5_txc, AVRDX_IRQ_USART5_TXC, excpt_common
#elif defined(CONFIG_ARCH_CHIP_AVR128DB64)
	HANDLER avrdx_nmi, AVRDX_IRQ_NMI, excpt_common
	HANDLER avrdx_bod_vlm, AVRDX_IRQ_BOD_VLM, excpt_common
	HANDLER avrdx_clkctrl_cfd, AVRDX_IRQ_CLKCTRL_CFD, excpt_common
	HANDLER avrdx_mvio_mvio, AVRDX_IRQ_MVIO_MVIO, excpt_common
	HANDLER avrdx_rtc_cnt, AVRDX_IRQ_RTC_CNT, excpt_common
	HANDLER avrdx_rtc_pit, AVRDX_IRQ_RTC_PIT, excpt_common
	HANDLER avrdx_ccl_ccl, AVRDX_IRQ_CCL_CCL, excpt_common
	HANDLER avrdx_porta_port, AVRDX_IRQ_PORTA_PORT, excpt_common
	HANDLER avrdx_tca0_lunf, AVRDX_IRQ_TCA0_LUNF, excpt_common
	HANDLER avrdx_tca0_ovf, AVRDX_IRQ_TCA0_OVF, excpt_common
	HANDLER avrdx_tca0_hunf, AVRDX_IRQ_TCA0_HUNF, excpt_common
	HANDLER avrdx_tca0_cmp0, AVRDX_IRQ_TCA0_CMP0, excpt_common
	HANDLER avrdx_tca0_lcmp0, AVRDX_IRQ_TCA0_LCMP0, excpt_common
	HANDLER avrdx_tca0_cmp1, AVRDX_IRQ_TCA0_CMP1, excpt_common
	HANDLER avrdx_tca0_lcmp1, AVRDX_IRQ_TCA0_LCMP1, excpt_common
	HANDLER avrdx_tca0_cmp2, AVRDX_IRQ_TCA0_CMP2, excpt_common
	HANDLER avrdx_tca0_lcmp2, AVRDX_IRQ_TCA0_LCMP2, excpt_common
	HANDLER avrdx_tcb0_int, AVRDX_IRQ_TCB0_INT, excpt_common
	HANDLER avrdx_tcb1_int, AVRDX_IRQ_TCB1_INT, excpt_common
	HANDLER avrdx_tcd0_ovf, AVRDX_IRQ_TCD0_OVF, excpt_common
	HANDLER avrdx_tcd0_trig, AVRDX_IRQ_TCD0_TRIG, excpt_common
	HANDLER avrdx_twi0_twis, AVRDX_IRQ_TWI0_TWIS, excpt_common
	HANDLER avrdx_twi0_twim, AVRDX_IRQ_TWI0_TWIM, excpt_common
	HANDLER avrdx_spi0_int, AVRDX_IRQ_SPI0_INT, excpt_common
	HANDLER avrdx_usart0_rxc, AVRDX_IRQ_USART0_RXC, excpt_common
	HANDLER avrdx_usart0_dre, AVRDX_IRQ_USART0_DRE, excpt_common
	HANDLER avrdx_usart0_txc, AVRDX_IRQ_USART0_TXC, excpt_common
	HANDLER avrdx_portd_port, AVRDX_IRQ_PORTD_PORT, excpt_common
	HANDLER avrdx_ac0_ac, AVRDX_IRQ_AC0_AC, excpt_common
	HANDLER avrdx_adc0_resrdy, AVRDX_IRQ_ADC0_RESRDY, excpt_common
	HANDLER avrdx_adc0_wcmp, AVRDX_IRQ_ADC0_WCMP, excpt_common
	HANDLER avrdx_zcd0_zcd, AVRDX_IRQ_ZCD0_ZCD, excpt_common
	HANDLER avrdx_ac1_ac, AVRDX_IRQ_AC1_AC, excpt_common
	HANDLER avrdx_portc_port, AVRDX_IRQ_PORTC_PORT, excpt_common
	HANDLER avrdx_tcb2_int, AVRDX_IRQ_TCB2_INT, excpt_common
	HANDLER avrdx_usart1_rxc, AVRDX_IRQ_USART1_RXC, excpt_common
	HANDLER avrdx_usart1_dre, AVRDX_IRQ_USART1_DRE, excpt_common
	HANDLER avrdx_usart1_txc, AVRDX_IRQ_USART1_TXC, excpt_common
	HANDLER avrdx_portf_port, AVRDX_IRQ_PORTF_PORT, excpt_common
	HANDLER avrdx_nvmctrl_ee, AVRDX_IRQ_NVMCTRL_EE, excpt_common
	HANDLER avrdx_spi1_int, AVRDX_IRQ_SPI1_INT, excpt_common
	HANDLER avrdx_usart2_rxc, AVRDX_IRQ_USART2_RXC, excpt_common
	HANDLER avrdx_usart2_dre, AVRDX_IRQ_USART2_DRE, excpt_common
	HANDLER avrdx_usart2_txc, AVRDX_IRQ_USART2_TXC, excpt_common
	HANDLER avrdx_ac2_ac, AVRDX_IRQ_AC2_AC, excpt_common
	HANDLER avrdx_twi1_twis, AVRDX_IRQ_TWI1_TWIS, excpt_common
	HANDLER avrdx_twi1_twim, AVRDX_IRQ_TWI1_TWIM, excpt_common
	HANDLER avrdx_tcb3_int, AVRDX_IRQ_TCB3_INT, excpt_common
	HANDLER avrdx_portb_port, AVRDX_IRQ_PORTB_PORT, excpt_common
	HANDLER avrdx_porte_port, AVRDX_IRQ_PORTE_PORT, excpt_common
	HANDLER avrdx_tca1_lunf, AVRDX_IRQ_TCA1_LUNF, excpt_common
	HANDLER avrdx_tca1_ovf, AVRDX_IRQ_TCA1_OVF, excpt_common
	HANDLER avrdx_tca1_hunf, AVRDX_IRQ_TCA1_HUNF, excpt_common
	HANDLER avrdx_tca1_cmp0, AVRDX_IRQ_TCA1_CMP0, excpt_common
	HANDLER avrdx_tca1_lcmp0, AVRDX_IRQ_TCA1_LCMP0, excpt_common
	HANDLER avrdx_tca1_cmp1, AVRDX_IRQ_TCA1_CMP1, excpt_common
	HANDLER avrdx_tca1_lcmp1, AVRDX_IRQ_TCA1_LCMP1, excpt_common
	HANDLER avrdx_tca1_cmp2, AVRDX_IRQ_TCA1_CMP2, excpt_common
	HANDLER avrdx_tca1_lcmp2, AVRDX_IRQ_TCA1_LCMP2, excpt_common
	HANDLER avrdx_zcd1_zcd, AVRDX_IRQ_ZCD1_ZCD, excpt_common
	HANDLER avrdx_usart3_rxc, AVRDX_IRQ_USART3_RXC, excpt_common
	HANDLER avrdx_usart3_dre, AVRDX_IRQ_USART3_DRE, excpt_common
	HANDLER avrdx_usart3_txc, AVRDX_IRQ_USART3_TXC, excpt_common
	HANDLER avrdx_usart4_rxc, AVRDX_IRQ_USART4_RXC, excpt_common
	HANDLER avrdx_usart4_dre, AVRDX_IRQ_USART4_DRE, excpt_common
	HANDLER avrdx_usart4_txc, AVRDX_IRQ_USART4_TXC, excpt_common
	HANDLER avrdx_portg_port, AVRDX_IRQ_PORTG_PORT, excpt_common
	HANDLER avrdx_zcd2_zcd, AVRDX_IRQ_ZCD2_ZCD, excpt_common
	HANDLER avrdx_tcb4_int, AVRDX_IRQ_TCB4_INT, excpt_common
	HANDLER avrdx_usart5_rxc, AVRDX_IRQ_USART5_RXC, excpt_common
	HANDLER avrdx_usart5_dre, AVRDX_IRQ_USART5_DRE, excpt_common
	HANDLER avrdx_usart5_txc, AVRDX_IRQ_USART5_TXC, excpt_common
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

	/* Call avr_doirq with r24 = IRQ number, r22-23 = Pointer to the save structure.  The stack
	 * pointer currently points to the save structure (or maybe the save structure -1 since
	 * the push operation post-decrements -- need to REVISIT this).
	 */

	in		r28, _SFR_IO_ADDR(SPL)	/* Get the save structure pointer in a Call-saved register pair */
	in		r29, _SFR_IO_ADDR(SPH)	/* Pointer can be obtained from the stack pointer */
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
	 * equal to the address of the new register save ared.
	 */

	jmp	avr_fullcontextrestore

	/* No context switch occurred.. just return off the stack */

.Lnoswitch:
	EXCPT_EPILOGUE

	/* Unlike eg. ATmega devices, from which this code is derived,
	 * Dx cores do not set Interrupt Enable on reti
	 */
	sei
	reti

/****************************************************************************************************
 *  Name: g_intstackalloc
 ****************************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 0
	.bss
	.align	4
	.globl	g_intstackalloc
	.type	g_intstackalloc, object
	.globl	g_intstacktop
	.type	g_intstacktop, object
g_intstackalloc:
	.skip	CONFIG_ARCH_INTERRUPTSTACK
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
#endif
	.end
