/****************************************************************************
 * arch/risc-v/include/rv32m1/rv32m1ri5cy_irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_RV32M1_RV32M1RI5CY_IRQ_H
#define __ARCH_RISCV_INCLUDE_RV32M1_RV32M1RI5CY_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RV32M1 RI5CY Interrupts **************************************************/

#define RV32M1_IRQ_MEXT     (RISCV_IRQ_ASYNC + 0)  /* Machine External Int */

/* Machine Global External Interrupt */

#define RV32M1_IRQ_DMA0G0   (RV32M1_IRQ_MEXT +  0) /* DMA0 Group0, channel 0/4/8/12 */
#define RV32M1_IRQ_DMA0G1   (RV32M1_IRQ_MEXT +  1) /* DMA0 Group1, channel 1/5/9/13 */
#define RV32M1_IRQ_DMA0G2   (RV32M1_IRQ_MEXT +  2) /* DMA0 Group2, channel 2/6/10/14 */
#define RV32M1_IRQ_DMA0G3   (RV32M1_IRQ_MEXT +  3) /* DMA0 Group3, channel 3/7/11/15 */
#define RV32M1_IRQ_DMA0EER  (RV32M1_IRQ_MEXT +  4) /* DAM0 Error */
#define RV32M1_IRQ_CMC0     (RV32M1_IRQ_MEXT +  5) /* Core Mode Controller 0 */
#define RV32M1_IRQ_MUA      (RV32M1_IRQ_MEXT +  6) /* MU Side A */
#define RV32M1_IRQ_USB0     (RV32M1_IRQ_MEXT +  7) /* USB0 */
#define RV32M1_IRQ_USDHC0   (RV32M1_IRQ_MEXT +  8) /* SDHC0 */
#define RV32M1_IRQ_I2S0     (RV32M1_IRQ_MEXT +  9) /* I2S0 */
#define RV32M1_IRQ_FLEXIO0  (RV32M1_IRQ_MEXT + 10) /* FlexIO0 */
#define RV32M1_IRQ_EMVSIM0  (RV32M1_IRQ_MEXT + 11) /* EMVSIM0 */
#define RV32M1_IRQ_LPIT0    (RV32M1_IRQ_MEXT + 12) /* LPIT0 */
#define RV32M1_IRQ_LPSPI0   (RV32M1_IRQ_MEXT + 13) /* LPSPI0 */
#define RV32M1_IRQ_LPSPI1   (RV32M1_IRQ_MEXT + 14) /* LPSPI1 */
#define RV32M1_IRQ_LPI2C0   (RV32M1_IRQ_MEXT + 15) /* LPI2C0 */
#define RV32M1_IRQ_LPI2C1   (RV32M1_IRQ_MEXT + 16) /* LPI2C1 */
#define RV32M1_IRQ_LPUART0  (RV32M1_IRQ_MEXT + 17) /* LPUART0 */
#define RV32M1_IRQ_PORTA    (RV32M1_IRQ_MEXT + 18) /* PORTA */
#define RV32M1_IRQ_TPM0     (RV32M1_IRQ_MEXT + 19) /* TPM0 */
#define RV32M1_IRQ_ADC0     (RV32M1_IRQ_MEXT + 20) /* ADC0 */
#define RV32M1_IRQ_LPDAC0   (RV32M1_IRQ_MEXT + 21) /* LPDAC0 */
#define RV32M1_IRQ_LPCMP0   (RV32M1_IRQ_MEXT + 22) /* LPCMP0 */
#define RV32M1_IRQ_RTC      (RV32M1_IRQ_MEXT + 23) /* RTC */
#define RV32M1_IRQ_INTMUX0  (RV32M1_IRQ_MEXT + 24) /* INTMUX0 */
#define RV32M1_IRQ_INTMUX1  (RV32M1_IRQ_MEXT + 25) /* INTMUX1 */
#define RV32M1_IRQ_INTMUX2  (RV32M1_IRQ_MEXT + 26) /* INTMUX2 */
#define RV32M1_IRQ_INTMUX3  (RV32M1_IRQ_MEXT + 27) /* INTMUX3 */
#define RV32M1_IRQ_INTMUX4  (RV32M1_IRQ_MEXT + 28) /* INTMUX4 */
#define RV32M1_IRQ_INTMUX5  (RV32M1_IRQ_MEXT + 29) /* INTMUX5 */
#define RV32M1_IRQ_INTMUX6  (RV32M1_IRQ_MEXT + 30) /* INTMUX6 */
#define RV32M1_IRQ_INTMUX7  (RV32M1_IRQ_MEXT + 31) /* INTMUX7 */
#define RV32M1_IRQ_EWM      (RV32M1_IRQ_MEXT + 32) /* EWM */
#  define RV32M1_IRQ_INTMUX RV32M1_IRQ_EWM
#define RV32M1_IRQ_FTFE_CC  (RV32M1_IRQ_MEXT + 33) /* FTFE Command Complete */
#define RV32M1_IRQ_FTFE_RC  (RV32M1_IRQ_MEXT + 34) /* FTFE Read Collision */
#define RV32M1_IRQ_LLWU0    (RV32M1_IRQ_MEXT + 35) /* Low leakage wake up 0 */
#define RV32M1_IRQ_SPM      (RV32M1_IRQ_MEXT + 36) /* SPM */
#define RV32M1_IRQ_WDOG0    (RV32M1_IRQ_MEXT + 37) /* WDOG0 */
#define RV32M1_IRQ_SCG      (RV32M1_IRQ_MEXT + 38) /* SCG */
#define RV32M1_IRQ_LPTMR0   (RV32M1_IRQ_MEXT + 39) /* LPTMR0 */
#define RV32M1_IRQ_LPTMR1   (RV32M1_IRQ_MEXT + 40) /* LPTMR1 */
#define RV32M1_IRQ_TPM1     (RV32M1_IRQ_MEXT + 41) /* TPM1 */
#define RV32M1_IRQ_TMP2     (RV32M1_IRQ_MEXT + 42) /* TPM2 */
#define RV32M1_IRQ_LPI2C2   (RV32M1_IRQ_MEXT + 43) /* LPI2C2 */
#define RV32M1_IRQ_SPI2     (RV32M1_IRQ_MEXT + 44) /* SPI2 */
#define RV32M1_IRQ_LPUART1  (RV32M1_IRQ_MEXT + 45) /* LPUART1 */
#define RV32M1_IRQ_LPUART2  (RV32M1_IRQ_MEXT + 46) /* LPUART2 */
#define RV32M1_IRQ_PORTB    (RV32M1_IRQ_MEXT + 47) /* PORTB */
#define RV32M1_IRQ_PORTC    (RV32M1_IRQ_MEXT + 48) /* PORTC */
#define RV32M1_IRQ_PORTD    (RV32M1_IRQ_MEXT + 49) /* PORTD */
#define RV32M1_IRQ_CAU3_TC  (RV32M1_IRQ_MEXT + 50) /* CAU3 Task Complete */
#define RV32M1_IRQ_CAU3_SV  (RV32M1_IRQ_MEXT + 51) /* CAU3 Security Violation */
#define RV32M1_IRQ_TRNG     (RV32M1_IRQ_MEXT + 52) /* TRNG */
#define RV32M1_IRQ_LPIT1    (RV32M1_IRQ_MEXT + 53) /* LPIT1 */
#define RV32M1_IRQ_LPTMR2   (RV32M1_IRQ_MEXT + 54) /* LPTMR2 */
#define RV32M1_IRQ_TPM3     (RV32M1_IRQ_MEXT + 55) /* TPM3 */
#define RV32M1_IRQ_LPI2C3   (RV32M1_IRQ_MEXT + 56) /* LPI2C3 */
#define RV32M1_IRQ_LPSPI3   (RV32M1_IRQ_MEXT + 57) /* LPSPI3 */
#define RV32M1_IRQ_LPUART3  (RV32M1_IRQ_MEXT + 58) /* LPUART3 */
#define RV32M1_IRQ_PORTE    (RV32M1_IRQ_MEXT + 59) /* PORTE */
#define RV32M1_IRQ_LPCMP1   (RV32M1_IRQ_MEXT + 60) /* LPCMP1 */
#define RV32M1_IRQ_RF0_0    (RV32M1_IRQ_MEXT + 61) /* RF0 Interrupt 0 */
#define RV32M1_IRQ_RF0_1    (RV32M1_IRQ_MEXT + 62) /* RF0 Interrupt 1 */

/* Total number of IRQs */

#define NR_IRQS             (RV32M1_IRQ_RF0_1 + 1)

#endif /* __ARCH_RISCV_INCLUDE_RV32M1_RV32M1RI5CY_IRQ_H */
