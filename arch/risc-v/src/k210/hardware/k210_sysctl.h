/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_sysctl.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sysctl Register Offsets */

#define K210_SYSCTL_GIT_ID      (K210_SYSCTL_BASE + 0x00)  /* Git short commit id */
#define K210_SYSCTL_CLK_FREQ    (K210_SYSCTL_BASE + 0x04)  /* System clock base frequency */
#define K210_SYSCTL_PLL0        (K210_SYSCTL_BASE + 0x08)  /* PLL0 controller */
#define K210_SYSCTL_PLL1        (K210_SYSCTL_BASE + 0x0c)  /* PLL1 controller */
#define K210_SYSCTL_PLL2        (K210_SYSCTL_BASE + 0x10)  /* PLL2 controller */

/* 0x14: Reserved */

#define K210_SYSCTL_PLL_LOCK    (K210_SYSCTL_BASE + 0x18)  /* PLL lock tester */
#define K210_SYSCTL_ROM_ERROR   (K210_SYSCTL_BASE + 0x1c)  /* AXI ROM detector */
#define K210_SYSCTL_CLKSEL0     (K210_SYSCTL_BASE + 0x20)  /* Clock select controller 0 */
#define K210_SYSCTL_CLKSEL1     (K210_SYSCTL_BASE + 0x24)  /* Clock select controller 1 */
#define K210_SYSCTL_CLK_EN_CENT (K210_SYSCTL_BASE + 0x28)  /* Central clock enable */
#define K210_SYSCTL_CLK_EN_PERI (K210_SYSCTL_BASE + 0x2c)  /* Peripheral clock enable */
#define K210_SYSCTL_SOFT_RESET  (K210_SYSCTL_BASE + 0x30)  /* Soft reset control */
#define K210_SYSCTL_PERI_RESET  (K210_SYSCTL_BASE + 0x34)  /* Peripheral reset controller */
#define K210_SYSCTL_CLK_TH0     (K210_SYSCTL_BASE + 0x38)  /* Clock threshold controller 0 */
#define K210_SYSCTL_CLK_TH1     (K210_SYSCTL_BASE + 0x3c)  /* Clock threshold controller 1 */
#define K210_SYSCTL_CLK_TH2     (K210_SYSCTL_BASE + 0x40)  /* Clock threshold controller 2 */
#define K210_SYSCTL_CLK_TH3     (K210_SYSCTL_BASE + 0x44)  /* Clock threshold controller 3 */
#define K210_SYSCTL_CLK_TH4     (K210_SYSCTL_BASE + 0x48)  /* Clock threshold controller 4 */
#define K210_SYSCTL_CLK_TH5     (K210_SYSCTL_BASE + 0x4c)  /* Clock threshold controller 5 */
#define K210_SYSCTL_CLK_TH6     (K210_SYSCTL_BASE + 0x50)  /* Clock threshold controller 6 */
#define K210_SYSCTL_MISC        (K210_SYSCTL_BASE + 0x54)  /* Miscellaneous controller */
#define K210_SYSCTL_PERI        (K210_SYSCTL_BASE + 0x58)  /* Peripheral controller */
#define K210_SYSCTL_SPI_SLEEP   (K210_SYSCTL_BASE + 0x5c)  /* SPI sleep controller */
#define K210_SYSCTL_RESET_STATUS (K210_SYSCTL_BASE + 0x60) /* Reset source status */
#define K210_SYSCTL_DMA_SEL0    (K210_SYSCTL_BASE + 0x64)  /* DMA handshake selector 0 */
#define K210_SYSCTL_DMA_SEL1    (K210_SYSCTL_BASE + 0x68)  /* DMA handshake selector 1 */
#define K210_SYSCTL_POWER_SEL   (K210_SYSCTL_BASE + 0x6c)  /* IO Power Mode Select */

/* PLL bit field extraction macros (legacy, kept for compatibility) */

#define PLL_CLK_R(n)  (n & 0x00000f)
#define PLL_CLK_F(n)  ((n & 0x0003f0) >> 4)
#define PLL_CLK_OD(n) ((n & 0x003c00) >> 10)

/* PLL register bit field definitions */

#define PLL_CLKR_SHIFT         (0)                       /* Reference clock divider */
#define PLL_CLKR_MASK          (0xf << PLL_CLKR_SHIFT)
#define PLL_CLKF_SHIFT         (4)                       /* Feedback divider */
#define PLL_CLKF_MASK          (0x3f << PLL_CLKF_SHIFT)
#define PLL_CLKOD_SHIFT        (10)                      /* Output divider */
#define PLL_CLKOD_MASK         (0xf << PLL_CLKOD_SHIFT)
#define PLL_BWADJ_SHIFT        (14)                      /* Bandwidth adjust */
#define PLL_BWADJ_MASK         (0x3f << PLL_BWADJ_SHIFT)
#define PLL_RESET_SHIFT        (20)                      /* PLL reset */
#define PLL_RESET_MASK         (1 << PLL_RESET_SHIFT)
#define PLL_PWRD_SHIFT         (21)                      /* Power down */
#define PLL_PWRD_MASK          (1 << PLL_PWRD_SHIFT)
#define PLL_BYPASS_SHIFT       (23)                      /* Bypass */
#define PLL_BYPASS_MASK        (1 << PLL_BYPASS_SHIFT)
#define PLL_OUT_EN_SHIFT       (25)                      /* Output enable */
#define PLL_OUT_EN_MASK        (1 << PLL_OUT_EN_SHIFT)

/* CLK_EN_CENT register bit definitions (Central clock enable) */

#define CLK_EN_CENT_CPU_SHIFT    (0)    /* CPU clock enable */
#define CLK_EN_CENT_CPU_MASK     (1 << CLK_EN_CENT_CPU_SHIFT)
#define CLK_EN_CENT_SRAM0_SHIFT  (1)    /* SRAM0 clock enable */
#define CLK_EN_CENT_SRAM0_MASK   (1 << CLK_EN_CENT_SRAM0_SHIFT)
#define CLK_EN_CENT_SRAM1_SHIFT  (2)    /* SRAM1 clock enable */
#define CLK_EN_CENT_SRAM1_MASK   (1 << CLK_EN_CENT_SRAM1_SHIFT)
#define CLK_EN_CENT_APB0_SHIFT   (3)    /* APB0 bus clock enable */
#define CLK_EN_CENT_APB0_MASK    (1 << CLK_EN_CENT_APB0_SHIFT)
#define CLK_EN_CENT_APB1_SHIFT   (4)    /* APB1 bus clock enable */
#define CLK_EN_CENT_APB1_MASK    (1 << CLK_EN_CENT_APB1_SHIFT)
#define CLK_EN_CENT_APB2_SHIFT   (5)    /* APB2 bus clock enable */
#define CLK_EN_CENT_APB2_MASK    (1 << CLK_EN_CENT_APB2_SHIFT)

/* CLK_EN_PERI register bit definitions (Peripheral clock enable) */

#define CLK_EN_PERI_ROM_SHIFT    (0)    /* ROM clock enable */
#define CLK_EN_PERI_ROM_MASK     (1 << CLK_EN_PERI_ROM_SHIFT)
#define CLK_EN_PERI_DMA_SHIFT    (1)    /* DMA clock enable */
#define CLK_EN_PERI_DMA_MASK     (1 << CLK_EN_PERI_DMA_SHIFT)
#define CLK_EN_PERI_AI_SHIFT     (2)    /* AI accelerator clock enable */
#define CLK_EN_PERI_AI_MASK      (1 << CLK_EN_PERI_AI_SHIFT)
#define CLK_EN_PERI_DVP_SHIFT    (3)    /* DVP camera interface clock enable */
#define CLK_EN_PERI_DVP_MASK     (1 << CLK_EN_PERI_DVP_SHIFT)
#define CLK_EN_PERI_FFT_SHIFT    (4)    /* FFT accelerator clock enable */
#define CLK_EN_PERI_FFT_MASK     (1 << CLK_EN_PERI_FFT_SHIFT)
#define CLK_EN_PERI_GPIO_SHIFT   (5)    /* GPIO clock enable */
#define CLK_EN_PERI_GPIO_MASK    (1 << CLK_EN_PERI_GPIO_SHIFT)
#define CLK_EN_PERI_SPI0_SHIFT   (6)    /* SPI0 clock enable */
#define CLK_EN_PERI_SPI0_MASK    (1 << CLK_EN_PERI_SPI0_SHIFT)
#define CLK_EN_PERI_SPI1_SHIFT   (7)    /* SPI1 clock enable */
#define CLK_EN_PERI_SPI1_MASK    (1 << CLK_EN_PERI_SPI1_SHIFT)
#define CLK_EN_PERI_SPI2_SHIFT   (8)    /* SPI2 clock enable */
#define CLK_EN_PERI_SPI2_MASK    (1 << CLK_EN_PERI_SPI2_SHIFT)
#define CLK_EN_PERI_SPI3_SHIFT   (9)    /* SPI3 clock enable */
#define CLK_EN_PERI_SPI3_MASK    (1 << CLK_EN_PERI_SPI3_SHIFT)
#define CLK_EN_PERI_I2S0_SHIFT   (10)   /* I2S0 clock enable */
#define CLK_EN_PERI_I2S0_MASK    (1 << CLK_EN_PERI_I2S0_SHIFT)
#define CLK_EN_PERI_I2S1_SHIFT   (11)   /* I2S1 clock enable */
#define CLK_EN_PERI_I2S1_MASK    (1 << CLK_EN_PERI_I2S1_SHIFT)
#define CLK_EN_PERI_I2S2_SHIFT   (12)   /* I2S2 clock enable */
#define CLK_EN_PERI_I2S2_MASK    (1 << CLK_EN_PERI_I2S2_SHIFT)
#define CLK_EN_PERI_I2C0_SHIFT   (13)   /* I2C0 clock enable */
#define CLK_EN_PERI_I2C0_MASK    (1 << CLK_EN_PERI_I2C0_SHIFT)
#define CLK_EN_PERI_I2C1_SHIFT   (14)   /* I2C1 clock enable */
#define CLK_EN_PERI_I2C1_MASK    (1 << CLK_EN_PERI_I2C1_SHIFT)
#define CLK_EN_PERI_I2C2_SHIFT   (15)   /* I2C2 clock enable */
#define CLK_EN_PERI_I2C2_MASK    (1 << CLK_EN_PERI_I2C2_SHIFT)
#define CLK_EN_PERI_UART1_SHIFT  (16)   /* UART1 clock enable */
#define CLK_EN_PERI_UART1_MASK   (1 << CLK_EN_PERI_UART1_SHIFT)
#define CLK_EN_PERI_UART2_SHIFT  (17)   /* UART2 clock enable */
#define CLK_EN_PERI_UART2_MASK   (1 << CLK_EN_PERI_UART2_SHIFT)
#define CLK_EN_PERI_UART3_SHIFT  (18)   /* UART3 clock enable */
#define CLK_EN_PERI_UART3_MASK   (1 << CLK_EN_PERI_UART3_SHIFT)
#define CLK_EN_PERI_AES_SHIFT    (19)   /* AES accelerator clock enable */
#define CLK_EN_PERI_AES_MASK     (1 << CLK_EN_PERI_AES_SHIFT)
#define CLK_EN_PERI_FPIOA_SHIFT  (20)   /* FPIOA (GPIO multiplexer) clock enable */
#define CLK_EN_PERI_FPIOA_MASK   (1 << CLK_EN_PERI_FPIOA_SHIFT)
#define CLK_EN_PERI_TIMER0_SHIFT (21)   /* TIMER0 clock enable */
#define CLK_EN_PERI_TIMER0_MASK  (1 << CLK_EN_PERI_TIMER0_SHIFT)
#define CLK_EN_PERI_TIMER1_SHIFT (22)   /* TIMER1 clock enable */
#define CLK_EN_PERI_TIMER1_MASK  (1 << CLK_EN_PERI_TIMER1_SHIFT)
#define CLK_EN_PERI_TIMER2_SHIFT (23)   /* TIMER2 clock enable */
#define CLK_EN_PERI_TIMER2_MASK  (1 << CLK_EN_PERI_TIMER2_SHIFT)
#define CLK_EN_PERI_WDT0_SHIFT   (24)   /* Watchdog timer 0 clock enable */
#define CLK_EN_PERI_WDT0_MASK    (1 << CLK_EN_PERI_WDT0_SHIFT)
#define CLK_EN_PERI_WDT1_SHIFT   (25)   /* Watchdog timer 1 clock enable */
#define CLK_EN_PERI_WDT1_MASK    (1 << CLK_EN_PERI_WDT1_SHIFT)
#define CLK_EN_PERI_SHA_SHIFT    (26)   /* SHA accelerator clock enable */
#define CLK_EN_PERI_SHA_MASK     (1 << CLK_EN_PERI_SHA_SHIFT)
#define CLK_EN_PERI_OTP_SHIFT    (27)   /* OTP (One-Time Programmable) clock enable */
#define CLK_EN_PERI_OTP_MASK     (1 << CLK_EN_PERI_OTP_SHIFT)
#define CLK_EN_PERI_RTC_SHIFT    (29)   /* RTC clock enable */
#define CLK_EN_PERI_RTC_MASK     (1 << CLK_EN_PERI_RTC_SHIFT)

/* PERI_RESET register bit definitions (Peripheral reset control) */

#define PERI_RESET_ROM_SHIFT     (0)    /* ROM reset */
#define PERI_RESET_ROM_MASK      (1 << PERI_RESET_ROM_SHIFT)
#define PERI_RESET_DMA_SHIFT     (1)    /* DMA reset */
#define PERI_RESET_DMA_MASK      (1 << PERI_RESET_DMA_SHIFT)
#define PERI_RESET_AI_SHIFT      (2)    /* AI accelerator reset */
#define PERI_RESET_AI_MASK       (1 << PERI_RESET_AI_SHIFT)
#define PERI_RESET_DVP_SHIFT     (3)    /* DVP camera interface reset */
#define PERI_RESET_DVP_MASK      (1 << PERI_RESET_DVP_SHIFT)
#define PERI_RESET_FFT_SHIFT     (4)    /* FFT accelerator reset */
#define PERI_RESET_FFT_MASK      (1 << PERI_RESET_FFT_SHIFT)
#define PERI_RESET_GPIO_SHIFT    (5)    /* GPIO reset */
#define PERI_RESET_GPIO_MASK     (1 << PERI_RESET_GPIO_SHIFT)
#define PERI_RESET_SPI0_SHIFT    (6)    /* SPI0 reset */
#define PERI_RESET_SPI0_MASK     (1 << PERI_RESET_SPI0_SHIFT)
#define PERI_RESET_SPI1_SHIFT    (7)    /* SPI1 reset */
#define PERI_RESET_SPI1_MASK     (1 << PERI_RESET_SPI1_SHIFT)
#define PERI_RESET_SPI2_SHIFT    (8)    /* SPI2 reset */
#define PERI_RESET_SPI2_MASK     (1 << PERI_RESET_SPI2_SHIFT)
#define PERI_RESET_SPI3_SHIFT    (9)    /* SPI3 reset */
#define PERI_RESET_SPI3_MASK     (1 << PERI_RESET_SPI3_SHIFT)
#define PERI_RESET_I2S0_SHIFT    (10)   /* I2S0 reset */
#define PERI_RESET_I2S0_MASK     (1 << PERI_RESET_I2S0_SHIFT)
#define PERI_RESET_I2S1_SHIFT    (11)   /* I2S1 reset */
#define PERI_RESET_I2S1_MASK     (1 << PERI_RESET_I2S1_SHIFT)
#define PERI_RESET_I2S2_SHIFT    (12)   /* I2S2 reset */
#define PERI_RESET_I2S2_MASK     (1 << PERI_RESET_I2S2_SHIFT)
#define PERI_RESET_I2C0_SHIFT    (13)   /* I2C0 reset */
#define PERI_RESET_I2C0_MASK     (1 << PERI_RESET_I2C0_SHIFT)
#define PERI_RESET_I2C1_SHIFT    (14)   /* I2C1 reset */
#define PERI_RESET_I2C1_MASK     (1 << PERI_RESET_I2C1_SHIFT)
#define PERI_RESET_I2C2_SHIFT    (15)   /* I2C2 reset */
#define PERI_RESET_I2C2_MASK     (1 << PERI_RESET_I2C2_SHIFT)
#define PERI_RESET_UART1_SHIFT   (16)   /* UART1 reset */
#define PERI_RESET_UART1_MASK    (1 << PERI_RESET_UART1_SHIFT)
#define PERI_RESET_UART2_SHIFT   (17)   /* UART2 reset */
#define PERI_RESET_UART2_MASK    (1 << PERI_RESET_UART2_SHIFT)
#define PERI_RESET_UART3_SHIFT   (18)   /* UART3 reset */
#define PERI_RESET_UART3_MASK    (1 << PERI_RESET_UART3_SHIFT)
#define PERI_RESET_AES_SHIFT     (19)   /* AES accelerator reset */
#define PERI_RESET_AES_MASK      (1 << PERI_RESET_AES_SHIFT)
#define PERI_RESET_FPIOA_SHIFT   (20)   /* FPIOA (GPIO multiplexer) reset */
#define PERI_RESET_FPIOA_MASK    (1 << PERI_RESET_FPIOA_SHIFT)
#define PERI_RESET_TIMER0_SHIFT  (21)   /* TIMER0 reset */
#define PERI_RESET_TIMER0_MASK   (1 << PERI_RESET_TIMER0_SHIFT)
#define PERI_RESET_TIMER1_SHIFT  (22)   /* TIMER1 reset */
#define PERI_RESET_TIMER1_MASK   (1 << PERI_RESET_TIMER1_SHIFT)
#define PERI_RESET_TIMER2_SHIFT  (23)   /* TIMER2 reset */
#define PERI_RESET_TIMER2_MASK   (1 << PERI_RESET_TIMER2_SHIFT)
#define PERI_RESET_WDT0_SHIFT    (24)   /* Watchdog timer 0 reset */
#define PERI_RESET_WDT0_MASK     (1 << PERI_RESET_WDT0_SHIFT)
#define PERI_RESET_WDT1_SHIFT    (25)   /* Watchdog timer 1 reset */
#define PERI_RESET_WDT1_MASK     (1 << PERI_RESET_WDT1_SHIFT)
#define PERI_RESET_SHA_SHIFT     (26)   /* SHA accelerator reset */
#define PERI_RESET_SHA_MASK      (1 << PERI_RESET_SHA_SHIFT)
#define PERI_RESET_RTC_SHIFT     (29)   /* RTC reset */
#define PERI_RESET_RTC_MASK      (1 << PERI_RESET_RTC_SHIFT)

/* RESET_STATUS register bit definitions (Reset source status) */

#define RESET_STATUS_CLR_SHIFT       (0)    /* Reset status clear */
#define RESET_STATUS_CLR_MASK        (1 << RESET_STATUS_CLR_SHIFT)
#define RESET_STATUS_PIN_SHIFT       (1)    /* Pin reset status */
#define RESET_STATUS_PIN_MASK        (1 << RESET_STATUS_PIN_SHIFT)
#define RESET_STATUS_WDT0_SHIFT      (2)    /* Watchdog timer 0 reset status */
#define RESET_STATUS_WDT0_MASK       (1 << RESET_STATUS_WDT0_SHIFT)
#define RESET_STATUS_WDT1_SHIFT      (3)    /* Watchdog timer 1 reset status */
#define RESET_STATUS_WDT1_MASK       (1 << RESET_STATUS_WDT1_SHIFT)
#define RESET_STATUS_SOFT_SHIFT      (4)    /* Soft reset status */
#define RESET_STATUS_SOFT_MASK       (1 << RESET_STATUS_SOFT_SHIFT)

/* CLKSEL0 register bit definitions (Clock select controller 0) */

#define CLKSEL0_ACLK_SEL_SHIFT       (0)
#define CLKSEL0_ACLK_SEL_MASK        (1 << CLKSEL0_ACLK_SEL_SHIFT)
#define CLKSEL0_ACLK_DIV_SHIFT       (1)
#define CLKSEL0_ACLK_DIV_MASK        (3 << CLKSEL0_ACLK_DIV_SHIFT)
#define CLKSEL0_APB0_DIV_SHIFT       (3)
#define CLKSEL0_APB0_DIV_MASK        (7 << CLKSEL0_APB0_DIV_SHIFT)
#define CLKSEL0_APB1_DIV_SHIFT       (6)
#define CLKSEL0_APB1_DIV_MASK        (7 << CLKSEL0_APB1_DIV_SHIFT)
#define CLKSEL0_APB2_DIV_SHIFT       (9)
#define CLKSEL0_APB2_DIV_MASK        (7 << CLKSEL0_APB2_DIV_SHIFT)

/* CLK_TH0 register bit definitions (Clock threshold controller 0) */

#define CLK_TH0_SRAM0_SHIFT          (0)
#define CLK_TH0_SRAM0_MASK           (0xf << CLK_TH0_SRAM0_SHIFT)
#define CLK_TH0_SRAM1_SHIFT          (4)
#define CLK_TH0_SRAM1_MASK           (0xf << CLK_TH0_SRAM1_SHIFT)
#define CLK_TH0_AI_SHIFT             (8)
#define CLK_TH0_AI_MASK              (0xf << CLK_TH0_AI_SHIFT)
#define CLK_TH0_DVP_SHIFT            (12)
#define CLK_TH0_DVP_MASK             (0xf << CLK_TH0_DVP_SHIFT)
#define CLK_TH0_ROM_SHIFT            (16)
#define CLK_TH0_ROM_MASK             (0xf << CLK_TH0_ROM_SHIFT)

/* CLK_TH1 register bit definitions (Clock threshold controller 1) */

#define CLK_TH1_SPI0_SHIFT           (0)
#define CLK_TH1_SPI0_MASK            (0xff << CLK_TH1_SPI0_SHIFT)
#define CLK_TH1_SPI1_SHIFT           (8)
#define CLK_TH1_SPI1_MASK            (0xff << CLK_TH1_SPI1_SHIFT)
#define CLK_TH1_SPI2_SHIFT           (16)
#define CLK_TH1_SPI2_MASK            (0xff << CLK_TH1_SPI2_SHIFT)
#define CLK_TH1_SPI3_SHIFT           (24)
#define CLK_TH1_SPI3_MASK            (0xff << CLK_TH1_SPI3_SHIFT)

/* CLK_TH2 register bit definitions (Clock threshold controller 2) */

#define CLK_TH2_TIMER0_SHIFT         (0)
#define CLK_TH2_TIMER0_MASK          (0xff << CLK_TH2_TIMER0_SHIFT)
#define CLK_TH2_TIMER1_SHIFT         (8)
#define CLK_TH2_TIMER1_MASK          (0xff << CLK_TH2_TIMER1_SHIFT)
#define CLK_TH2_TIMER2_SHIFT         (16)
#define CLK_TH2_TIMER2_MASK          (0xff << CLK_TH2_TIMER2_SHIFT)

/* CLK_TH3 register bit definitions (Clock threshold controller 3) */

#define CLK_TH3_I2S0_SHIFT           (0)
#define CLK_TH3_I2S0_MASK            (0xffff << CLK_TH3_I2S0_SHIFT)
#define CLK_TH3_I2S1_SHIFT           (16)
#define CLK_TH3_I2S1_MASK            (0xffff << CLK_TH3_I2S1_SHIFT)

/* CLK_TH4 register bit definitions (Clock threshold controller 4) */

#define CLK_TH4_I2S2_SHIFT           (0)
#define CLK_TH4_I2S2_MASK            (0xffff << CLK_TH4_I2S2_SHIFT)

/* CLK_TH5 register bit definitions (Clock threshold controller 5) */

#define CLK_TH5_I2C0_SHIFT           (8)
#define CLK_TH5_I2C0_MASK            (0xff << CLK_TH5_I2C0_SHIFT)
#define CLK_TH5_I2C1_SHIFT           (16)
#define CLK_TH5_I2C1_MASK            (0xff << CLK_TH5_I2C1_SHIFT)
#define CLK_TH5_I2C2_SHIFT           (24)
#define CLK_TH5_I2C2_MASK            (0xff << CLK_TH5_I2C2_SHIFT)

/* CLK_TH6 register bit definitions (Clock threshold controller 6) */

#define CLK_TH6_WDT0_SHIFT           (0)
#define CLK_TH6_WDT0_MASK            (0xff << CLK_TH6_WDT0_SHIFT)
#define CLK_TH6_WDT1_SHIFT           (8)
#define CLK_TH6_WDT1_MASK            (0xff << CLK_TH6_WDT1_SHIFT)

/* PLL_LOCK register bit definitions (PLL lock tester) */

#define PLL_LOCK_PLL0_SHIFT        (0)
#define PLL_LOCK_PLL0_MASK         (0x3 << PLL_LOCK_PLL0_SHIFT)
#define PLL_LOCK_PLL0_SLIP_CLR     (1 << 2)
#define PLL_LOCK_PLL1_SHIFT        (8)
#define PLL_LOCK_PLL1_MASK         (0x1 << PLL_LOCK_PLL1_SHIFT)
#define PLL_LOCK_PLL1_SLIP_CLR     (1 << 10)
#define PLL_LOCK_PLL2_SHIFT        (16)
#define PLL_LOCK_PLL2_MASK         (0x1 << PLL_LOCK_PLL2_SHIFT)
#define PLL_LOCK_PLL2_SLIP_CLR     (1 << 18)

#define PLL_LOCK_PLL0_LOCKED       (0x3 << PLL_LOCK_PLL0_SHIFT)
#define PLL_LOCK_PLL1_LOCKED       (0x1 << PLL_LOCK_PLL1_SHIFT)
#define PLL_LOCK_PLL2_LOCKED       (0x1 << PLL_LOCK_PLL2_SHIFT)

/* Clock select macros */

#define CLKSEL0_ACLK_SEL(n) (n & 0x00000001)

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H */
