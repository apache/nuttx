/****************************************************************************
 * arch/avr/include/avrdx/irq.h
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
 ****************************************************************************/

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_AVRDX_IRQ_H
#define __ARCH_AVR_INCLUDE_AVRDX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AVRnDx interrupt vectors other than vector 0, the reset vector, are
 * assigned here:
 */

#if defined(CONFIG_ARCH_CHIP_AVR128DA28)

#  define AVRDX_IRQ_NMI          0 /* Non-maskable interrupt */
#  define AVRDX_IRQ_BOD_VLM      1 /* BOD interrupt */
#  define AVRDX_IRQ_RTC_CNT      2 /* RTC and periodic interrupt */
#  define AVRDX_IRQ_RTC_PIT      3
#  define AVRDX_IRQ_CCL_CCL      4 /* CCL */
#  define AVRDX_IRQ_PORTA_PORT   5 /* PORTA interrupts */
#  define AVRDX_IRQ_TCA0_LUNF    6 /* TIMER A0 Low byte underflow */
#  define AVRDX_IRQ_TCA0_OVF     6
#  define AVRDX_IRQ_TCA0_HUNF    7 /* TIMER A0 High byte underflow */
#  define AVRDX_IRQ_TCA0_CMP0    8 /* TIMER A0 compare match 0 */
#  define AVRDX_IRQ_TCA0_LCMP0   8 /* TIMER A0 Low byte compare match 0 */
#  define AVRDX_IRQ_TCA0_CMP1    9
#  define AVRDX_IRQ_TCA0_LCMP1   9
#  define AVRDX_IRQ_TCA0_CMP2   10
#  define AVRDX_IRQ_TCA0_LCMP2  10
#  define AVRDX_IRQ_TCB0_INT    11
#  define AVRDX_IRQ_TCB1_INT    12
#  define AVRDX_IRQ_TCD0_OVF    13
#  define AVRDX_IRQ_TCD0_TRIG   14
#  define AVRDX_IRQ_TWI0_TWIS   15 /* TWI client interrupt */
#  define AVRDX_IRQ_TWI0_TWIM   16 /* TWI host interrupt */
#  define AVRDX_IRQ_SPI0_INT    17
#  define AVRDX_IRQ_USART0_RXC  18
#  define AVRDX_IRQ_USART0_DRE  19
#  define AVRDX_IRQ_USART0_TXC  20
#  define AVRDX_IRQ_PORTD_PORT  21
#  define AVRDX_IRQ_AC0_AC      22 /* AC0 analog comparator interrupt */
#  define AVRDX_IRQ_ADC0_RESRDY 23 /* ADC0 result ready */
#  define AVRDX_IRQ_ADC0_WCMP   24 /* ADC0 Window compare interrupt */
#  define AVRDX_IRQ_ZCD0_ZCD    25 /* Zero-cross interrupt */
#  define AVRDX_IRQ_PTC_PTC     26 /* Peripheral touch controller interrupt */
#  define AVRDX_IRQ_AC1_AC      27
#  define AVRDX_IRQ_PORTC_PORT  28
#  define AVRDX_IRQ_TCB2_INT    29
#  define AVRDX_IRQ_USART1_RXC  30
#  define AVRDX_IRQ_USART1_DRE  31
#  define AVRDX_IRQ_USART1_TXC  32
#  define AVRDX_IRQ_PORTF_PORT  33
#  define AVRDX_IRQ_NVMCTRL_EE  34 /* NVM controller EEPROM ready interrupt */
#  define AVRDX_IRQ_SPI1_INT    35
#  define AVRDX_IRQ_USART2_RXC  36
#  define AVRDX_IRQ_USART2_DRE  37
#  define AVRDX_IRQ_USART2_TXC  38
#  define AVRDX_IRQ_AC2_AC      39

#  define NR_IRQS                40
#  define AVR_PC_SIZE            16
#  define XCPTCONTEXT_REGS       38 /* Size of the register state save array (in bytes),
                                     * 32B registers + 1B SREG + 2B PC + 2B SP (?)
                                     * +1B RAMPZ
                                     */

#  if defined(CONFIG_AVR_HAS_RAMPZ)
#    define AVR_HAS_RAMPZ
#  else
#    error CONFIG_AVR_HAS_RAMPZ is supposed to be set for this chip
#  endif

#else
  #error "Unrecognized chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_AVR_INCLUDE_AVRDX_IRQ_H */
