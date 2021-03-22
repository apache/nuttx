/****************************************************************************
 * arch/arm/include/lpc31xx/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_LPC31XX_IRQ_H
#define __ARCH_ARM_INCLUDE_LPC31XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* LPC31XX Interrupts */

                              /* IRQ0: Reserved */
#define LPC31_IRQ_IRQ0      0 /* IRQ1:  Event router cascaded IRQ0 */
#define LPC31_IRQ_IRQ1      1 /* IRQ2:  Event router cascaded IRQ1 */
#define LPC31_IRQ_IRQ2      2 /* IRQ3:  Event router cascaded IRQ2 */
#define LPC31_IRQ_IRQ3      3 /* IRQ4:  Event router cascaded IRQ3 */
#define LPC31_IRQ_TMR0      4 /* IRQ5:  Timer 0 Interrupt */
#define LPC31_IRQ_TMR1      5 /* IRQ6:  Timer 1 Interrupt */
#define LPC31_IRQ_TMR2      6 /* IRQ7:  Timer 2 Interrupt */
#define LPC31_IRQ_TMR3      7 /* IRQ8:  Timer 3 Interrupt */
#define LPC31_IRQ_ADC       8 /* IRQ9:  ADC 10-bit */
#define LPC31_IRQ_UART      9 /* IRQ10: UART */
#define LPC31_IRQ_I2C0     10 /* IRQ11: I2C0 */
#define LPC31_IRQ_I2C1     11 /* IRQ12: I2C1 */
#define LPC31_IRQ_I2STX0   12 /* IRQ13: I2S0 Transmit */
#define LPC31_IRQ_I2STX1   13 /* IRQ14: I2S1 Transmit */
#define LPC31_IRQ_I2SRX0   14 /* IRQ15: I2S0 Receive */
#define LPC31_IRQ_I2SRX1   15 /* IRQ16: I2S1 Receive */
                              /* IRQ17: Reserved */
#define LPC31_IRQ_LCD      17 /* IRQ18: LCD Interface */
#define LPC31_IRQ_SPISMS   18 /* IRQ19: SPI SMS */
#define LPC31_IRQ_SPITX    19 /* IRQ20: SPI Transmit */
#define LPC31_IRQ_SPIRX    20 /* IRQ21: SPI Receive */
#define LPC31_IRQ_SPIOVF   21 /* IRQ22: SPI Overflow */
#define LPC31_IRQ_SPI      22 /* IRQ23: SPI */
#define LPC31_IRQ_DMA      23 /* IRQ24: DMA */
#define LPC31_IRQ_NAND     24 /* IRQ25: NAND FLASH Controller */
#define LPC31_IRQ_MCI      25 /* IRQ26: MCI */
#define LPC31_IRQ_USBOTG   26 /* IRQ27: USB OTG */
#define LPC31_IRQ_ISRAM0   27 /* IRQ28: ISRAM0 MRC Finished */
#define LPC31_IRQ_ISRAM1   28 /* IRQ29: ISRAM1 MRC Finished */

#define LPC31_IRQ_SYSTIMER LPC31_IRQ_TMR0
#define NR_IRQS            (LPC31_IRQ_ISRAM1+1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LPC31XX_IRQ_H */
