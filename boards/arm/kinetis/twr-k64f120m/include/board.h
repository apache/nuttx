/****************************************************************************
 * boards/arm/kinetis/twr-k64f120m/include/board.h
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

#ifndef __BOARDS_ARM_KINETIS_TWR_K64F120M_INCLUDE_BOARCH_H
#define __BOARDS_ARM_KINETIS_TWR_K64F120M_INCLUDE_BOARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The K64 tower board uses a 50MHz external clock */

#define BOARD_EXTCLOCK       1              /* External clock */
#define BOARD_EXTAL_FREQ     50000000       /* 50MHz Oscillator */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.
 * Either the external clock or crystal frequency is used to select the
 * PRDIV value. Only reference clock frequencies are supported that will
 * produce a range 2MHz-4MHz reference clock to the PLL.
 *
 *   PLL Input frequency:   PLLIN  = REFCLK/PRDIV = 50MHz/20 = 2.5 MHz
 *   PLL Output frequency:  PLLOUT = PLLIN*VDIV   = 2.5Mhz*48 = 120MHz
 *   MCG Frequency:         PLLOUT = 120 MHz
 */

#define BOARD_PRDIV          20  /* PLL External Reference Divider */
#define BOARD_VDIV           48  /* PLL VCO Divider (frequency multiplier) */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       BOARD_PLLOUT_FREQ

/* Define additional MCG_C2 Setting */

#define BOARD_MCG_C2_FCFTRIM 0              /* Do not enable FCFTRIM */
#define BOARD_MCG_C2_LOCRE0  MCG_C2_LOCRE0  /* Enable reset on loss of clock */

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1          /* Core        = MCG, 120MHz */
#define BOARD_OUTDIV2        2          /* Bus         = MCG/2, 60MHz */
#define BOARD_OUTDIV3        2          /* FlexBus     = MCG/2, 60MHz */
#define BOARD_OUTDIV4        5          /* Flash clock = MCG/5, 24MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ      (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* SDHC clocking ************************************************************/

/* SDCLK configurations corresponding to various modes of operation.
 * Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is
 * the base clock.
 */

/* Identification mode:  375KHz = 120MHz / ( 64 * 5) <= 400 KHz */

#define BOARD_SDHC_IDMODE_PRESCALER    SDHC_SYSCTL_SDCLKFS_DIV64
#define BOARD_SDHC_IDMODE_DIVISOR      SDHC_SYSCTL_DVS_DIV(5)

/* MMC normal mode: 15MHz  = 120MHz / (8 * 1) <= 16 MHz */

#define BOARD_SDHC_MMCMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDHC_MMCMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(1)

/* SD normal mode (1-bit): 15MHz  = 120MHz / (8 * 1) <= 16 MHz */

#define BOARD_SDHC_SD1MODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDHC_SD1MODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(1)

/* SD normal mode (4-bit): 20MHz  = 120MHz / (2 * 3) (with DMA) <= 24MHz
 * SD normal mode (4-bit): 15MHz  = 120MHz / (8 * 1) (no DMA) <= 16MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(3)
#else
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV8
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(1)
#endif

/* LED definitions **********************************************************/

/* The TWR-K64F120M has four LEDs:
 *
 * 1. D5 / Green LED    PTE6
 * 2. D6 / Yellow LED   PTE7
 * 3. D7 / Orange LED   PTE8
 * 4  D9 / Blue LED     PTE9
 *
 * LED4 is reserved for user.
 * The 3 first LEDs are encoded as follows:
 */

#define LED_STARTED       0  /* N/A */
#define LED_HEAPALLOCATE  1  /* N/A */
#define LED_IRQSENABLED   2  /* N/A */
#define LED_STACKCREATED  3  /* LED1 - OS is started */
#define LED_INIRQ         4  /* LED2 */
#define LED_SIGNAL        5  /* LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* LED1 (blink) */

/* Open SDA serial link */

#define PIN_UART1_RX  PIN_UART1_RX_1
#define PIN_UART1_TX  PIN_UART1_TX_1

/* Ethernet */

#ifdef CONFIG_KINETIS_ENET
#  define CONFIG_KINETIS_NENET 1
#endif

#endif /* __BOARDS_ARM_KINETIS_TWR_K64F120M_INCLUDE_BOARCH_H */
