/****************************************************************************
 * boards/arm/kl/freedom-kl26z/include/board.h
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

#ifndef __BOARDS_ARM_KL_FREEDOM_KL26Z_INCLUDE_BOARD_H
#define __BOARDS_ARM_KL_FREEDOM_KL26Z_INCLUDE_BOARD_H

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

/* The KL26Z has an 8MHz crystal on board */

#undef  BOARD_EXTCLOCK                      /* Crystal */
#define BOARD_XTAL_FREQ      8000000        /* 8MHz crystal frequency (REFCLK) */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.
 *
 *   PLL Input frequency:   PLLIN     = REFCLK / PRDIV0 = 8MHz / 2 = 4MHz
 *   PLL Output frequency:  PLLOUT    = PLLIN * VDIV0   = 4Mhz * 24 = 96MHz
 *   MCGPLLCLK Frequency:   MCGPLLCLK = 96MHz
 */

#define BOARD_PRDIV0         2  /* PLL External Reference Divider */
#define BOARD_VDIV0          24 /* PLL VCO Divider (frequency multiplier) */

#define BOARD_PLLIN_FREQ     (BOARD_XTAL_FREQ / BOARD_PRDIV0)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV0)
#define BOARD_MCGPLLCLK_FREQ  BOARD_PLLOUT_FREQ

/* MCGOUTCLK:
 * MCG output of either IRC, MCGFLLCLK, MCGPLLCLK, or MCG's external
 * reference clock that sources the core, system, bus, and flash clock.
 *
 * MCGOUTCLK = MCGPLLCLK = 96MHz
 */

#define BOARD_MCGOUTCLK_FREQ  BOARD_MCGPLLCLK_FREQ

/* SIM CLKDIV1 dividers.
 *
 * Core/system clock
 *   MCGOUTCLK divided by OUTDIV1, clocks the ARM Cortex-M0+ core
 *
 * Bus clock
 *   System clock divided by OUTDIV4, clocks the bus slaves and peripherals.
 */

#define BOARD_OUTDIV1        2   /* Core/system = MCGOUTCLK / 2, 48MHz */
#define BOARD_OUTDIV4        2   /* Bus clock   = System clock / 2, 24MHz */

#define BOARD_CORECLK_FREQ  (BOARD_MCGOUTCLK_FREQ / BOARD_OUTDIV1)
#define BOARD_BUSCLK_FREQ   (BOARD_CORECLK_FREQ / BOARD_OUTDIV4)

/* SDHC clocking ************************************************************/

/* SDCLK configurations corresponding to various modes of operation.
 * Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is
 * the base clock.
 */

/* Identification mode:  400KHz = 96MHz / ( 16 * 15) */

#define BOARD_SDHC_IDMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV16
#define BOARD_SDHC_IDMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(15)

/* MMC normal mode: 16MHz  = 96MHz / (2 * 3) */

#define BOARD_SDHC_MMCMODE_PRESCALER  SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR    SDHC_SYSCTL_DVS_DIV(3)

/* SD normal mode (1-bit): 16MHz  = 96MHz / (2 * 3) */

#define BOARD_SDHC_SD1MODE_PRESCALER  SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR    SDHC_SYSCTL_DVS_DIV(3)

/* SD normal mode (4-bit): 24MHz  = 96MHz / (2 * 2) (with DMA)
 * SD normal mode (4-bit): 16MHz  = 96MHz / (2 * 3) (no DMA)
 */

#ifdef CONFIG_SDIO_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(2)
#else

/* #  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2 */

/* #  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(3) */

#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV16
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(15)
#endif

/* PWM Configuration */

/* TPM0 Channels */

#define GPIO_TPM0_CH0OUT PIN_TPM0_CH0_3 //PIN_TPM0_CH0_1
#define GPIO_TPM0_CH1OUT PIN_TPM0_CH1_1
#define GPIO_TPM0_CH2OUT PIN_TPM0_CH2_1
#define GPIO_TPM0_CH3OUT PIN_TPM0_CH3_1
#define GPIO_TPM0_CH4OUT PIN_TPM0_CH4_1
#define GPIO_TPM0_CH5OUT PIN_TPM0_CH5_1

/* TPM1 Channels */

#define GPIO_TPM1_CH0OUT PIN_TPM1_CH0_1
#define GPIO_TPM1_CH1OUT PIN_TPM1_CH1_1

/* TPM2 Channels */

#define GPIO_TPM2_CH0OUT PIN_TPM2_CH0_1
#define GPIO_TPM2_CH1OUT PIN_TPM2_CH1_1

/* LED definitions **********************************************************/

/* The Freedom KL26Z has a single RGB LED driven by the KL26Z as follows:
 *
 *   ------------- --------
 *   RGB LED       KL26Z128
 *   ------------- --------
 *   Red Cathode   PTBE29
 *   Green Cathode PTE31
 *   Blue Cathode  PTD5
 *
 * NOTE:
 * PTD5 is also connected to the I/O header on J2 pin 12 (also known as D13).
 *
 * If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom KL26Z.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                 Initially all LED is OFF
 *   -------------------  -----------------------  --------------------------
 *   LED_STARTED          NuttX has been started   R=OFF G=OFF B=OFF
 *   LED_HEAPALLOCATE     Heap has been allocated  (no change)
 *   LED_IRQSENABLED      Interrupts enabled       (no change)
 *   LED_STACKCREATED     Idle stack created       R=OFF G=OFF B=ON
 *   LED_INIRQ            In an interrupt          (no change)
 *   LED_SIGNAL           In a signal handler      (no change)
 *   LED_ASSERTION        An assertion failed      (no change)
 *   LED_PANIC            The system has crashed   R=FLASHING G=OFF B=OFF
 *   LED_IDLE             K26Z1XX is in sleep mode (Optional, not used)
 */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/* Button definitions *******************************************************/

/* The Freedom KL26Z board has no standard GPIO contact buttons */

/* Alternative pin resolution ***********************************************/

/* If there are alternative configurations for various pins in the
 * k26z128_pinmux.h header file, those alternative pins will be labeled with
 * a suffix like _1, _2, etc.
 * The logic in this file must select the correct pin configuration for the
 * board by defining a pin configuration (with no suffix) that maps to the
 * correct alternative.
 */

/* SPI0 Pinout
 * ===========
 *
 * SCK = PTD1 (D13 at connector J2 pin 12 of Freedom Board)
 * MISO = PTD3 (D12 at connector J2 pin 10 of Freedom Board)
 * MOSI = PTD2 (D11 at connector J2 pin 8 of Freedom Board)
 */

#define PIN_SPI0_SCK   (PIN_SPI0_SCK_3 | PIN_ALT2_PULLUP)
#define PIN_SPI0_MISO  (PIN_SPI0_MISO_6 | PIN_ALT2_PULLUP)
#define PIN_SPI0_MOSI  (PIN_SPI0_MOSI_5 | PIN_ALT2_PULLUP)

#define PIN_SPI1_SCK   (PIN_SPI1_SCK_2 | PIN_ALT2_PULLUP)
#define PIN_SPI1_MISO  (PIN_SPI1_MISO_3 | PIN_ALT2_PULLUP)
#define PIN_SPI1_MOSI  (PIN_SPI0_MOSI_7 | PIN_ALT2_PULLUP)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: kl_tsi_initialize
 *
 * Description:
 *   Initialize the TSI hardware and interface for the sliders on board the
 *   Freedom KL26Z board.  Register a character driver at /dev/tsi that may
 *   be used to read from each sensor.
 *
 ****************************************************************************/

#ifdef CONFIG_KL_TSI
void kl_tsi_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_KL_FREEDOM_KL26Z_INCLUDE_BOARD_H */
