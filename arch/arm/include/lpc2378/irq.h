/****************************************************************************
 * arch/arm/include/lpc2378/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LPC2378_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC2378_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* LPC2378 Interrupts */

#define WDT_IRQ        0 /* Watchdog */
#define RESERVED_IRQ   1 /* SWI only */
#define DBGCOMMRX_IRQ  2 /* Embedded debug */
#define DBGCOMMTX_IRQ  3 /* Embedded debug */
#define TIMER0_IRQ     4 /* Timer 0 */
#define TIMER1_IRQ     5 /* Timer 1 */
#define UART0_IRQ      6 /* UART 0 */
#define UART1_IRQ      7 /* UART 1 */
#define PWM0_IRQ       8 /* PWM 0 */
#define I2C0_IRQ       9 /* I2C 0 */
#define SPI0_IRQ      10 /* SPI 0 */
#define SSP0_IRQ      10 /* SSP 0 */
#define SSP1_IRQ      11 /* SSP 1 */
#define PLL_IRQ       12 /* PLL Lock IRQ */
#define RTC_IRQ       13 /* Real Time Clock */
#define EINT0_IRQ     14 /* External interrupt 0 */
#define EINT1_IRQ     15 /* External interrupt 1 */
#define EINT2_IRQ     16 /* External interrupt 2 */
#define EINT3_IRQ     17 /* External interrupt 3 */
#define ADC0_IRQ      18 /* ADC 0 */
#define I2C1_IRQ      19 /* I2C 1 */
#define BOD_IRQ       20 /* Brown Out Detect */
#define EMAC_IRQ      21 /* Ethernet */
#define USB_IRQ       22 /* USB */
#define CAN_IRQ       23 /* CAN */
#define MCI_IRQ       24 /* SD/MMC Interface */
#define GPDMA_IRQ     25 /* General Purpose DMA */
#define TIMER2_IRQ    26 /* Timer 2 */
#define TIMER3_IRQ    27 /* Timer 3 */
#define UART2_IRQ     28 /* Uart 2 */
#define UART3_IRQ     29 /* Uart 3 */
#define I2C2_IRQ      30 /* I2C 2 */
#define I2S_IRQ       31 /* I2S */

#define IRQ_SYSTIMER  TIMER0_IRQ

#define NR_IRQS             32

/* There are 32 vectored interrupts.
 * If vectored interrupts are enabled, the
 * following will be used by the system.
 */
#define SYSTIMER_VEC  0 /* System timer */

#define CLASS_IRQ           0
#define CLASS_FIQ           1
#define PRIORITY_LOWEST     15
#define PRIORITY_HIGHEST    0   /* System timer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

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

#ifndef CONFIG_VECTORED_INTERRUPTS
void up_attach_vector(int irq, int priority, vic_vector_t handler);
void up_detach_vector(int vector);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LPC2378_IRQ_H */
