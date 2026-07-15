/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_rcu.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_RCU_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_RCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets */

#define GD32VW55X_RCU_CTL           (GD32VW55X_RCU_BASE + 0x0000)
#define GD32VW55X_RCU_PLL           (GD32VW55X_RCU_BASE + 0x0004)
#define GD32VW55X_RCU_CFG0          (GD32VW55X_RCU_BASE + 0x0008)
#define GD32VW55X_RCU_INT           (GD32VW55X_RCU_BASE + 0x000c)
#define GD32VW55X_RCU_AHB1RST       (GD32VW55X_RCU_BASE + 0x0010)
#define GD32VW55X_RCU_AHB2RST       (GD32VW55X_RCU_BASE + 0x0014)
#define GD32VW55X_RCU_APB1RST       (GD32VW55X_RCU_BASE + 0x0020)
#define GD32VW55X_RCU_APB2RST       (GD32VW55X_RCU_BASE + 0x0024)
#define GD32VW55X_RCU_AHB1EN        (GD32VW55X_RCU_BASE + 0x0030)
#define GD32VW55X_RCU_AHB2EN        (GD32VW55X_RCU_BASE + 0x0034)
#define GD32VW55X_RCU_APB1EN        (GD32VW55X_RCU_BASE + 0x0040)
#define GD32VW55X_RCU_APB2EN        (GD32VW55X_RCU_BASE + 0x0044)
#define GD32VW55X_RCU_RSTSCK        (GD32VW55X_RCU_BASE + 0x0074)
#define GD32VW55X_RCU_PLLDIGCFG0    (GD32VW55X_RCU_BASE + 0x0084)
#define GD32VW55X_RCU_CFG1          (GD32VW55X_RCU_BASE + 0x008c)
#define GD32VW55X_RCU_PLLDIGCFG1    (GD32VW55X_RCU_BASE + 0x0094)

/* CTL register */

#define RCU_CTL_IRC16MEN            (1 << 0)   /* IRC16M enable */
#define RCU_CTL_HXTALEN             (1 << 16)  /* HXTAL enable */
#define RCU_CTL_HXTALSTB            (1 << 17)  /* HXTAL stable flag */
#define RCU_CTL_PLLDIGPU            (1 << 20)  /* PLLDIG power up */
#define RCU_CTL_PLLDIGEN            (1 << 21)  /* PLLDIG enable */
#define RCU_CTL_RFCKMEN             (1 << 22)  /* HXTAL clock monitor */
#define RCU_CTL_PLLDIGSTB           (1 << 23)  /* PLLDIG stable flag */
#define RCU_CTL_HXTALPU             (1 << 28)  /* HXTAL power up */
#define RCU_CTL_HXTALREADY          (1 << 31)  /* HXTAL ready (sw set) */

/* PLL register */

#define RCU_PLL_PLLDIGSEL           (1 << 15)  /* PLLDIG source: 0=IRC16M 1=HXTAL */

/* CFG0 register */

#define RCU_CFG0_SCS_MASK           (3 << 0)   /* System clock switch */
#define RCU_CFG0_SCS_IRC16M         (0 << 0)
#define RCU_CFG0_SCS_HXTAL          (1 << 0)
#define RCU_CFG0_SCS_PLLDIG         (2 << 0)
#define RCU_CFG0_SCSS_MASK          (3 << 2)   /* Switch status */
#define RCU_CFG0_SCSS_PLLDIG        (2 << 2)
#define RCU_CFG0_AHBPSC_MASK        (15 << 4)  /* AHB prescaler */
#define RCU_CFG0_AHBPSC_DIV1        (0 << 4)
#define RCU_CFG0_APB1PSC_MASK       (7 << 10)  /* APB1 prescaler */
#define RCU_CFG0_APB1PSC_DIV2       (4 << 10)
#define RCU_CFG0_APB2PSC_MASK       (7 << 13)  /* APB2 prescaler */
#define RCU_CFG0_APB2PSC_DIV1       (0 << 13)

/* CFG1 register */

#define RCU_CFG1_I2C0SEL_SHIFT      (26)       /* I2C0 clock source select */
#define RCU_CFG1_I2C0SEL_MASK       (3 << RCU_CFG1_I2C0SEL_SHIFT)
#  define RCU_CFG1_I2C0SEL_APB1     (0 << RCU_CFG1_I2C0SEL_SHIFT)
#  define RCU_CFG1_I2C0SEL_CKSYS    (1 << RCU_CFG1_I2C0SEL_SHIFT)
#  define RCU_CFG1_I2C0SEL_IRC16M   (2 << RCU_CFG1_I2C0SEL_SHIFT)
#define RCU_CFG1_RFPLLCALEN         (1 << 14)  /* RF PLL calibration enable */
#define RCU_CFG1_RFPLLPU            (1 << 16)  /* RF PLL power up */
#define RCU_CFG1_BGPU               (1 << 19)  /* Bandgap power up */

/* PLLDIGCFG0 register */

#define RCU_PLLDIGCFG0_OSEL_MASK    (3 << 24)  /* PLLDIG output select */
#define RCU_PLLDIGCFG0_OSEL_480M    (3 << 24)
#define RCU_PLLDIGCFG0_SYSDIV_MASK  (0x3f << 26) /* Divider for sysclk, n+1 */
#define RCU_PLLDIGCFG0_SYSDIV(n)    (((n) & 0x3f) << 26)

/* APB1RST register */

#define RCU_APB1RST_TIMER1RST       (1 << 0)   /* TIMER1 reset */
#define RCU_APB1RST_TIMER2RST       (1 << 1)   /* TIMER2 reset */
#define RCU_APB1RST_TIMER5RST       (1 << 4)   /* TIMER5 reset */

/* APB2RST register */

#define RCU_APB2RST_TIMER0RST       (1 << 0)   /* TIMER0 reset */
#define RCU_APB2RST_ADCRST          (1 << 8)   /* ADC reset */
#define RCU_APB2RST_TIMER15RST      (1 << 17)  /* TIMER15 reset */
#define RCU_APB2RST_TIMER16RST      (1 << 18)  /* TIMER16 reset */

/* APB1RST register */

#define RCU_APB1RST_I2C0RST         (1 << 21)  /* I2C0 reset */
#define RCU_APB1RST_I2C1RST         (1 << 22)  /* I2C1 reset */

/* APB2RST register */

#define RCU_APB2RST_SPIRST          (1 << 12)  /* SPI reset */

/* AHB1EN register */

#define RCU_AHB1EN_PAEN             (1 << 0)   /* GPIOA */
#define RCU_AHB1EN_PBEN             (1 << 1)   /* GPIOB */
#define RCU_AHB1EN_PCEN             (1 << 2)   /* GPIOC */
#define RCU_AHB1EN_CRCEN            (1 << 12)  /* CRC */
#define RCU_AHB1EN_WIFIEN           (1 << 13)  /* Wi-Fi */
#define RCU_AHB1EN_WIFIRUNEN        (1 << 14)  /* Wi-Fi run */
#define RCU_AHB1EN_SRAM0EN          (1 << 16)  /* SRAM0 */
#define RCU_AHB1EN_SRAM1EN          (1 << 17)  /* SRAM1 */
#define RCU_AHB1EN_SRAM2EN          (1 << 18)  /* SRAM2 */
#define RCU_AHB1EN_SRAM3EN          (1 << 19)  /* SRAM3 */
#define RCU_AHB1EN_DMAEN            (1 << 21)  /* DMA */
#define RCU_AHB1EN_BLEEN            (1 << 31)  /* BLE */

/* AHB2EN register */

#define RCU_AHB2EN_PKCAUEN          (1 << 3)   /* PKCAU */
#define RCU_AHB2EN_CAUEN            (1 << 4)   /* CAU */
#define RCU_AHB2EN_HAUEN            (1 << 5)   /* HAU */
#define RCU_AHB2EN_TRNGEN           (1 << 6)   /* TRNG */

/* AHB2RST register */

#define RCU_AHB2RST_TRNGRST         (1 << 6)   /* TRNG reset */

/* RSTSCK register */

#define RCU_RSTSCK_IRC32KEN         (1 << 0)   /* IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB        (1 << 1)   /* IRC32K stable flag */
#define RCU_RSTSCK_RSTFC            (1 << 24)  /* Reset flag clear */
#define RCU_RSTSCK_EPRSTF           (1 << 26)  /* External pin reset flag */
#define RCU_RSTSCK_PORRSTF          (1 << 27)  /* Power reset flag */
#define RCU_RSTSCK_SWRSTF           (1 << 28)  /* Software reset flag */
#define RCU_RSTSCK_FWDGTRSTF        (1 << 29)  /* Free watchdog reset flag */
#define RCU_RSTSCK_WWDGTRSTF        (1 << 30)  /* Window watchdog reset flag */
#define RCU_RSTSCK_LPRSTF           (1 << 31)  /* Low-power reset flag */

/* APB1EN register */

#define RCU_APB1EN_TIMER1EN         (1 << 0)   /* TIMER1 */
#define RCU_APB1EN_TIMER2EN         (1 << 1)   /* TIMER2 */
#define RCU_APB1EN_TIMER5EN         (1 << 4)   /* TIMER5 */
#define RCU_APB1EN_WWDGTEN          (1 << 11)  /* WWDGT */
#define RCU_APB1EN_UART1EN          (1 << 17)  /* UART1 */
#define RCU_APB1EN_USART0EN         (1 << 18)  /* USART0 */
#define RCU_APB1EN_I2C0EN           (1 << 21)  /* I2C0 */
#define RCU_APB1EN_I2C1EN           (1 << 22)  /* I2C1 */
#define RCU_APB1EN_PMUEN            (1 << 28)  /* PMU */

/* APB2EN register */

#define RCU_APB2EN_TIMER0EN         (1 << 0)   /* TIMER0 */
#define RCU_APB2EN_UART2EN          (1 << 4)   /* UART2 */
#define RCU_APB2EN_ADCEN            (1 << 8)   /* ADC */
#define RCU_APB2EN_SPIEN            (1 << 12)  /* SPI */
#define RCU_APB2EN_SYSCFGEN         (1 << 14)  /* SYSCFG */
#define RCU_APB2EN_TIMER15EN        (1 << 17)  /* TIMER15 */
#define RCU_APB2EN_TIMER16EN        (1 << 18)  /* TIMER16 */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_RCU_H */
