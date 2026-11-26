/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_cmu.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_CMU_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_CMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_CMU_STATUS_OFFSET          0x0008  /* Status */
#define EFR32_CMU_LOCK_OFFSET            0x0010  /* Configuration lock */
#define EFR32_CMU_CLKEN0_OFFSET          0x0064  /* Clock enable 0 */
#define EFR32_CMU_CLKEN1_OFFSET          0x0068  /* Clock enable 1 */
#define EFR32_CMU_SYSCLKCTRL_OFFSET      0x0070  /* System clock control */
#define EFR32_CMU_DPLLREFCLKCTRL_OFFSET  0x0100  /* DPLL reference clock */
#define EFR32_CMU_EM01GRPACLKCTRL_OFFSET 0x0120  /* EM01 group A clock */
#define EFR32_CMU_EM01GRPCCLKCTRL_OFFSET 0x0128  /* EM01 group C clock */

/* Register addresses *******************************************************/

#define EFR32_CMU_STATUS          (EFR32_CMU_BASE + EFR32_CMU_STATUS_OFFSET)
#define EFR32_CMU_LOCK            (EFR32_CMU_BASE + EFR32_CMU_LOCK_OFFSET)
#define EFR32_CMU_CLKEN0          (EFR32_CMU_BASE + EFR32_CMU_CLKEN0_OFFSET)
#define EFR32_CMU_CLKEN1          (EFR32_CMU_BASE + EFR32_CMU_CLKEN1_OFFSET)
#define EFR32_CMU_SYSCLKCTRL      (EFR32_CMU_BASE + EFR32_CMU_SYSCLKCTRL_OFFSET)
#define EFR32_CMU_DPLLREFCLKCTRL  (EFR32_CMU_BASE + EFR32_CMU_DPLLREFCLKCTRL_OFFSET)
#define EFR32_CMU_EM01GRPACLKCTRL (EFR32_CMU_BASE + EFR32_CMU_EM01GRPACLKCTRL_OFFSET)
#define EFR32_CMU_EM01GRPCCLKCTRL (EFR32_CMU_BASE + EFR32_CMU_EM01GRPCCLKCTRL_OFFSET)

#define EFR32_CMU_CLKEN0_SET      (EFR32_CMU_CLKEN0 + EFR32_SET_OFFSET)
#define EFR32_CMU_CLKEN1_SET      (EFR32_CMU_CLKEN1 + EFR32_SET_OFFSET)

/* Register bit definitions *************************************************/

/* Configuration lock */

#define CMU_LOCK_LOCKKEY_UNLOCK          0x93f7

/* Clock enable 0 */

#define CMU_CLKEN0_LDMA                  (1 << 0)
#define CMU_CLKEN0_LDMAXBAR              (1 << 1)
#define CMU_CLKEN0_RADIOAES              (1 << 2)
#define CMU_CLKEN0_GPCRC                 (1 << 3)
#define CMU_CLKEN0_TIMER0                (1 << 4)
#define CMU_CLKEN0_TIMER1                (1 << 5)
#define CMU_CLKEN0_TIMER2                (1 << 6)
#define CMU_CLKEN0_TIMER3                (1 << 7)
#define CMU_CLKEN0_TIMER4                (1 << 8)
#define CMU_CLKEN0_USART0                (1 << 9)
#define CMU_CLKEN0_IADC0                 (1 << 10)
#define CMU_CLKEN0_AMUXCP0               (1 << 11)
#define CMU_CLKEN0_LETIMER0              (1 << 12)
#define CMU_CLKEN0_WDOG0                 (1 << 13)
#define CMU_CLKEN0_I2C0                  (1 << 14)
#define CMU_CLKEN0_I2C1                  (1 << 15)
#define CMU_CLKEN0_SYSCFG                (1 << 16)
#define CMU_CLKEN0_DPLL0                 (1 << 17)
#define CMU_CLKEN0_HFRCO0                (1 << 18)
#define CMU_CLKEN0_HFRCOEM23             (1 << 19)
#define CMU_CLKEN0_HFXO0                 (1 << 20)
#define CMU_CLKEN0_FSRCO                 (1 << 21)
#define CMU_CLKEN0_LFRCO                 (1 << 22)
#define CMU_CLKEN0_LFXO                  (1 << 23)
#define CMU_CLKEN0_ULFRCO                (1 << 24)
#define CMU_CLKEN0_GPIO                  (1 << 26)
#define CMU_CLKEN0_PRS                   (1 << 27)
#define CMU_CLKEN0_BURAM                 (1 << 28)
#define CMU_CLKEN0_BURTC                 (1 << 29)
#define CMU_CLKEN0_SYSRTC0               (1 << 30)
#define CMU_CLKEN0_DCDC                  (1 << 31)

/* Clock enable 1 */

#define CMU_CLKEN1_SMU                   (1 << 14)
#define CMU_CLKEN1_ICACHE0               (1 << 15)
#define CMU_CLKEN1_MSC                   (1 << 16)
#define CMU_CLKEN1_WDOG1                 (1 << 17)
#define CMU_CLKEN1_EUSART0               (1 << 22)
#define CMU_CLKEN1_EUSART1               (1 << 23)

/* System clock control */

#define CMU_SYSCLKCTRL_CLKSEL_SHIFT      (0)
#define CMU_SYSCLKCTRL_CLKSEL_MASK       (0x7 << CMU_SYSCLKCTRL_CLKSEL_SHIFT)
#  define CMU_SYSCLKCTRL_CLKSEL_FSRCO    (1 << CMU_SYSCLKCTRL_CLKSEL_SHIFT)
#  define CMU_SYSCLKCTRL_CLKSEL_HFRCODPLL (2 << CMU_SYSCLKCTRL_CLKSEL_SHIFT)
#  define CMU_SYSCLKCTRL_CLKSEL_HFXO     (3 << CMU_SYSCLKCTRL_CLKSEL_SHIFT)
#  define CMU_SYSCLKCTRL_CLKSEL_CLKIN0   (4 << CMU_SYSCLKCTRL_CLKSEL_SHIFT)
#define CMU_SYSCLKCTRL_PCLKPRESC         (1 << 10) /* PCLK = HCLK / 2 */
#define CMU_SYSCLKCTRL_HCLKPRESC_SHIFT   (12)
#define CMU_SYSCLKCTRL_HCLKPRESC_MASK    (0xf << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#  define CMU_SYSCLKCTRL_HCLKPRESC_DIV1  (0 << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#  define CMU_SYSCLKCTRL_HCLKPRESC_DIV2  (1 << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#  define CMU_SYSCLKCTRL_HCLKPRESC_DIV4  (3 << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#  define CMU_SYSCLKCTRL_HCLKPRESC_DIV8  (7 << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#  define CMU_SYSCLKCTRL_HCLKPRESC_DIV16 (15 << CMU_SYSCLKCTRL_HCLKPRESC_SHIFT)
#define CMU_SYSCLKCTRL_RHCLKPRESC        (1 << 16) /* RHCLK = SYSCLK / 2 */

/* DPLL reference clock control */

#define CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT  (0)
#define CMU_DPLLREFCLKCTRL_CLKSEL_MASK   (0x3 << CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT)
#  define CMU_DPLLREFCLKCTRL_CLKSEL_DISABLED (0 << CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT)
#  define CMU_DPLLREFCLKCTRL_CLKSEL_HFXO (1 << CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT)
#  define CMU_DPLLREFCLKCTRL_CLKSEL_LFXO (2 << CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT)
#  define CMU_DPLLREFCLKCTRL_CLKSEL_CLKIN0 (3 << CMU_DPLLREFCLKCTRL_CLKSEL_SHIFT)

/* EM01 group A clock control */

#define CMU_EM01GRPACLKCTRL_CLKSEL_SHIFT (0)
#define CMU_EM01GRPACLKCTRL_CLKSEL_MASK  (0x7 << CMU_EM01GRPACLKCTRL_CLKSEL_SHIFT)
#  define CMU_EM01GRPACLKCTRL_CLKSEL_HFRCODPLL (1 << CMU_EM01GRPACLKCTRL_CLKSEL_SHIFT)
#  define CMU_EM01GRPACLKCTRL_CLKSEL_HFXO (2 << CMU_EM01GRPACLKCTRL_CLKSEL_SHIFT)
#  define CMU_EM01GRPACLKCTRL_CLKSEL_FSRCO (3 << CMU_EM01GRPACLKCTRL_CLKSEL_SHIFT)

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_CMU_H */
