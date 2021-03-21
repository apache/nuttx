/****************************************************************************
 * arch/arm/include/nuc1xx/nuc120_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_NUC1XX_NUC120_IRQ_H
#define __ARCH_ARM_INCLUDE_NUC1XX_NUC120_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* NUC120 IRQ numbers */

#define NUC_IRQ_BOD    (16) /* Brown-out low voltage detected */
#define NUC_IRQ_WDT    (17) /* Watchdog Timer */
#define NUC_IRQ_EINT0  (18) /* External interrupt from PB.14 */
#define NUC_IRQ_EINT1  (19) /* External interrupt from PB.15 */
#define NUC_IRQ_GPAB   (20) /* External interrupt from PA[15:0]/PB[13:0] */
#define NUC_IRQ_GPCDE  (21) /* External interrupt from PC[15:0]/PD[15:0]/PE[15:0] */
#define NUC_IRQ_PWMA   (22) /* PWM0-3 */
#define NUC_IRQ_PWMB   (23) /* PWM4-7 */
#define NUC_IRQ_TMR0   (24) /* Timer 0 */
#define NUC_IRQ_TMR1   (25) /* Timer 1 */
#define NUC_IRQ_TMR2   (26) /* Timer 2 */
#define NUC_IRQ_TMR3   (27) /* Timer 3 */
#define NUC_IRQ_UART02 (28) /* UART0-1 */
#define NUC_IRQ_UART1  (29) /* UART1 */
#define NUC_IRQ_SPI0   (30) /* SPI0 */
#define NUC_IRQ_SPI1   (31) /* SPI1 */
#define NUC_IRQ_SPI2   (32) /* SPI2 */
#define NUC_IRQ_SPI3   (33) /* SPI3 */
#define NUC_IRQ_I2C0   (34) /* I2C0 */
#define NUC_IRQ_I2C1   (35) /* I2C1 */
                            /* 36-38: Reserved */
#define NUC_IRQ_USB    (39) /* USB */
#define NUC_IRQ_PS2    (40) /* PS/2 interrupt */
#define NUC_IRQ_ACMP   (41) /* Analog comparator interrupt for chip wake-up from
                             * power down state */
#define NUC_IRQ_PDMA   (42) /* PDMA */
#define NUC_IRQ_I2S    (43) /* I2S */
#define NUC_IRQ_PWRWU  (44) /* Clock controller interrupt for chip wake-up from
                             * power down state */
#define NUC_IRQ_ADC    (45) /* ADC */
                            /* 46: Reserved */
#define NUC_IRQ_RTC    (47) /* Real time clock */

#define NR_IRQS        (48)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_NUC1XX_NUC120_IRQ_H */
