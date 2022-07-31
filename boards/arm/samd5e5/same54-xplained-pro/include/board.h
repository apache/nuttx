/****************************************************************************
 * boards/arm/samd5e5/same54-xplained-pro/include/board.h
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

#ifndef __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Overview
 *
 * Per the schematic Adafruit Metro M4 Pro has one on-board crystal:
 *
 *   X4 32.768KHz XOSC32
 *
 * However, I have been unsuccessful using it and have fallen back to using
 * OSCULP32K(Unless CONFIG_SAME54_XPLAINED_PRO_32KHZXTAL=y)
 *
 * Since there is no high speed crystal, we will run from the OSC16M clock
 * source.
 *
 * OSC48M               Output     = 48Mhz
 *  |
 * FDLL                 Input      = 48MHz
 *  |                   Output     = 48MHz
 * GCLK5                Input      = 48MHz
 *  |                   Output     = 2MHz
 * DPLL0                Input      = 2MHz
 *  |                   Output     = 120MHz
 * GCLK0                Input      = 120MHz
 *  |                   Output     = 120MHz
 * MCK                  Input      = 120MHz
 *  |                   Output     = 120MHz
 * CPU                  Input      = 120MHz
 */

#define BOARD_OSC32K_FREQUENCY  32768     /* OSCULP32K frequency 32.768 KHz (nominal) */
#define BOARD_XOSC32K_FREQUENCY 32768     /* XOSC32K frequency 32.768 KHz */
#define BOARD_DFLL_FREQUENCY    48000000  /* FDLL frequency 28MHz */
#define BOARD_XOSC0_FREQUENCY   12000000  /* XOSC0 frequency 12MHz (disabled) */
#define BOARD_XOSC1_FREQUENCY   12000000  /* XOSC0 frequency 12MHz (disabled)*/
#define BOARD_DPLL0_FREQUENCY   120000000 /* DPLL0 output frueuency (120MHz) */
#define BOARD_DPLL1_FREQUENCY   47985664  /* DPLL1 output frequency (disabled) */

#define BOARD_GCLK0_FREQUENCY   BOARD_DPLL0_FREQUENCY
#define BOARD_GCLK1_FREQUENCY   BOARD_DFLL_FREQUENCY
#define BOARD_GCLK2_FREQUENCY   (BOARD_XOSC32K_FREQUENCY / 4)  /* Disabled */
#ifdef CONFIG_SAME54_XPLAINED_PRO_32KHZXTAL
#  define BOARD_GCLK3_FREQUENCY BOARD_XOSC32K_FREQUENCY        /* Enabled */
#else
#  define BOARD_GCLK3_FREQUENCY BOARD_OSC32K_FREQUENCY         /* Always-on */
#endif
#define BOARD_GCLK4_FREQUENCY   BOARD_DPLL0_FREQUENCY
#define BOARD_GCLK5_FREQUENCY   (BOARD_DFLL_FREQUENCY / 24)
#define BOARD_GCLK6_FREQUENCY   BOARD_XOSC1_FREQUENCY          /* Disabled */
#define BOARD_GCLK7_FREQUENCY   BOARD_XOSC1_FREQUENCY          /* Disabled */
#define BOARD_GCLK8_FREQUENCY   BOARD_XOSC1_FREQUENCY          /* Disabled */
#define BOARD_GCLK9_FREQUENCY   BOARD_XOSC1_FREQUENCY          /* Disabled */
#define BOARD_GCLK10_FREQUENCY  BOARD_XOSC1_FREQUENCY          /* Disabled */
#define BOARD_GCLK11_FREQUENCY  BOARD_XOSC1_FREQUENCY          /* Disabled */

#define BOARD_CPU_FREQUENCY     BOARD_GCLK0_FREQUENCY /* CPU frequency 120MHz */

/* XOSC32 */

#ifdef CONFIG_SAME54_XPLAINED_PRO_32KHZXTAL
#  define BOARD_HAVE_XOSC32K    1         /* 32.768 KHz XOSC32 crystal installed */
#  define BOARD_XOSC32K_ENABLE  TRUE      /* Enable XOSC32 */
#else
#  define BOARD_HAVE_XOSC32K    0         /* No 32.768 KHz XOSC32 crystal installed */
#  define BOARD_XOSC32K_ENABLE  FALSE     /* Disable XOSC32 */
#endif
#define BOARD_XOSC32K_XTALEN    TRUE      /* Crystal connected on XIN32 */
#define BOARD_XOSC32K_EN32K     FALSE     /* No 32KHz output */
#define BOARD_XOSC32K_EN1K      FALSE     /* No 1KHz output */
#define BOARD_XOSC32K_HIGHSPEED TRUE      /* High speed mode */
#define BOARD_XOSC32K_RUNSTDBY  FALSE     /* Don't run in standby */
#define BOARD_XOSC32K_ONDEMAND  TRUE      /* Enable on-demand control */
#define BOARD_XOSC32K_CFDEN     FALSE     /* Clock failure detector not enabled */
#define BOARD_XOSC32K_CFDEO     FALSE     /* No clock failure event */
#define BOARD_XOSC32K_CALIBEN   FALSE     /* No OSCULP32K calibration */
#define BOARD_XOSC32K_STARTUP   0         /* Startup time: 62592us */
#define BOARD_XOSC32K_CALIB     0         /* Dummy OSCULP32K calibration value */
#define BOARD_XOSC32K_RTCSEL    0         /* RTC clock = ULP1K */

/* XOSC0 */

#define BOARD_HAVE_XOSC0        0         /* No XOSC0 clock/crystal installed */
#define BOARD_XOSC0_ENABLE      FALSE     /* Don't enable XOSC0 */
#define BOARD_XOSC0_XTALEN      FALSE     /* External clock connected */
#define BOARD_XOSC0_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_XOSC0_ONDEMAND    TRUE      /* Disable on-demand control */
#define BOARD_XOSC0_LOWGAIN     FALSE     /* Disable low buffer gain */
#define BOARD_XOSC0_ENALC       FALSE     /* Disable automatic loop control */
#define BOARD_XOSC0_CFDEN       FALSE     /* Clock failure detector not enabled */
#define BOARD_XOSC0_SWBEN       FALSE     /* XOSC clock switch not enabled */
#define BOARD_XOSC0_STARTUP     0         /* XOSC0 start-up time 31µs */

/* XOSC1 */

#define BOARD_HAVE_XOSC1        0         /* No XOSC0 clock/crystal installed */
#define BOARD_XOSC1_ENABLE      FALSE     /* Don't enable XOSC1 */
#define BOARD_XOSC1_XTALEN      TRUE      /* External crystal connected */
#define BOARD_XOSC1_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_XOSC1_ONDEMAND    TRUE      /* Disable on-demand control */
#define BOARD_XOSC1_LOWGAIN     FALSE     /* Disable low buffer gain */
#define BOARD_XOSC1_ENALC       FALSE     /* Disable automatic loop control */
#define BOARD_XOSC1_CFDEN       FALSE     /* Clock failure detector not enabled */
#define BOARD_XOSC1_SWBEN       FALSE     /* XOSC clock switch not enabled */
#define BOARD_XOSC1_STARTUP     0         /* XOSC0 start-up time 31µs */

/* GCLK */

#define BOARD_GCLK_SET1         0x0020    /* Pre-configure:  GCLK5 needed by DPLL0 */
#define BOARD_GCLK_SET2         0x0fdf    /* Post-configure: All GCLKs except GCLK5 */

#define BOARD_GCLK0_ENABLE      TRUE      /* Enable GCLK0 */
#define BOARD_GCLK0_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK0_OE          TRUE      /* Generate output on GCLK_IO */
#define BOARD_GCLK0_DIVSEL      0         /* GCLK frequency is source/DIV */
#define BOARD_GCLK0_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK0_SOURCE      7         /* Select DPLL0 output as GCLK0 source */
#define BOARD_GCLK0_DIV         1         /* Division factor */

#define BOARD_GCLK1_ENABLE      TRUE      /* Enable GCLK1 */
#define BOARD_GCLK1_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK1_OE          TRUE      /* Generate output on GCLK_IO */
#define BOARD_GCLK1_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK1_SOURCE      6         /* Select DFLL output as GCLK1 source */
#define BOARD_GCLK1_DIV         1         /* Division factor */

#define BOARD_GCLK2_ENABLE      FALSE     /* Don't enable GCLK2 */
#define BOARD_GCLK2_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK2_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK2_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK2_SOURCE      1         /* Select XOSC1 as GCLK2 source */
#define BOARD_GCLK2_DIV         1         /* Division factor */

#define BOARD_GCLK3_ENABLE      TRUE      /* Enable GCLK3 */
#define BOARD_GCLK3_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK3_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK3_RUNSTDBY    FALSE     /* Don't run in standby */
#ifdef CONFIG_SAME54_XPLAINED_PRO_32KHZXTAL
#  define BOARD_GCLK3_SOURCE    5         /* Select XOSC32K as GCLK3 source */
#else
#  define BOARD_GCLK3_SOURCE    4         /* Select OSCULP32K as GCLK3 source */
#endif
#define BOARD_GCLK3_DIV         1         /* Division factor */

#define BOARD_GCLK4_ENABLE      TRUE      /* Enable GCLK4 */
#define BOARD_GCLK4_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK4_OE          TRUE      /* Generate output on GCLK_IO */
#define BOARD_GCLK4_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK4_SOURCE      7         /* Select DPLL0 output as GCLK4 source */
#define BOARD_GCLK4_DIV         1         /* Division factor */

#define BOARD_GCLK5_ENABLE      TRUE      /* Enable GCLK5 */
#define BOARD_GCLK5_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK5_OE          TRUE      /* Generate output on GCLK_IO */
#define BOARD_GCLK5_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK5_SOURCE      6         /* Select DFLL output as GCLK5 source */
#define BOARD_GCLK5_DIV         24        /* Division factor */

#define BOARD_GCLK6_ENABLE      FALSE     /* Don't enable GCLK6 */
#define BOARD_GCLK6_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK6_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK6_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK6_SOURCE      1         /* Select XOSC1 as GCLK6 source */
#define BOARD_GCLK6_DIV         1         /* Division factor */

#define BOARD_GCLK7_ENABLE      FALSE     /* Don't enable GCLK7 */
#define BOARD_GCLK7_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK7_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK7_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK7_SOURCE      1         /* Select XOSC1 as GCLK7 source */
#define BOARD_GCLK7_DIV         1         /* Division factor */

#define BOARD_GCLK8_ENABLE      FALSE     /* Don't enable GCLK8 */
#define BOARD_GCLK8_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK8_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK8_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK8_SOURCE      1         /* Select XOSC1 as GCLK8 source */
#define BOARD_GCLK8_DIV         1         /* Division factor */

#define BOARD_GCLK9_ENABLE      FALSE     /* Don't enable GCLK9 */
#define BOARD_GCLK9_OOV         FALSE     /* Clock output will be LOW */
#define BOARD_GCLK9_OE          FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK9_RUNSTDBY    FALSE     /* Don't run in standby */
#define BOARD_GCLK9_SOURCE      1         /* Select XOSC1 as GCLK9 source */
#define BOARD_GCLK9_DIV         1         /* Division factor */

#define BOARD_GCLK10_ENABLE     FALSE     /* Don't enable GCLK10 */
#define BOARD_GCLK10_OOV        FALSE     /* Clock output will be LOW */
#define BOARD_GCLK10_OE         FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK10_RUNSTDBY   FALSE     /* Don't run in standby */
#define BOARD_GCLK10_SOURCE     1         /* Select XOSC1 as GCLK10 source */
#define BOARD_GCLK10_DIV        1         /* Division factor */

#define BOARD_GCLK11_ENABLE     FALSE     /* Don't enable GCLK11 */
#define BOARD_GCLK11_OOV        FALSE     /* Clock output will be LOW */
#define BOARD_GCLK11_OE         FALSE     /* No generator output of GCLK_IO */
#define BOARD_GCLK11_RUNSTDBY   FALSE     /* Don't run in standby */
#define BOARD_GCLK11_SOURCE     1         /* Select XOSC1 as GCLK11 source */
#define BOARD_GCLK11_DIV        1         /* Division factor */
#define BOARD_GCLK11_FREQUENCY  BOARD_XOSC1_FREQUENCY

/* FDLL */

#define BOARD_DFLL_ENABLE       TRUE      /* DFLL enable */
#define BOARD_DFLL_RUNSTDBY     FALSE     /* Don't run in standby */
#define BOARD_DFLL_ONDEMAND     FALSE     /* No n-demand control */
#define BOARD_DFLL_MODE         FALSE     /* Open loop mode */
#define BOARD_DFLL_STABLE       FALSE     /* No stable DFLL frequency */
#define BOARD_DFLL_LLAW         FALSE     /* Don't ose lock after wake */
#define BOARD_DFLL_USBCRM       TRUE      /* Use USB clock recovery mode */
#define BOARD_DFLL_CCDIS        TRUE      /* Chill cycle disable */
#define BOARD_DFLL_QLDIS        FALSE     /* No Quick Lock Disable */
#define BOARD_DFLL_BPLCKC       FALSE     /* No ypass coarse clock */
#define BOARD_DFLL_WAITLOCK     TRUE      /* Wait lock */
#define BOARD_DFLL_CALIBEN      FALSE     /* Don't verwrite factory calibration */
#define BOARD_DFLL_GCLKLOCK     FALSE     /* Don't lock the GCLK source */
#define BOARD_DFLL_FCALIB       128       /* Coarse calibration value (if caliben) */
#define BOARD_DFLL_CCALIB       (31 / 4)  /* Fine calibration value (if caliben) */
#define BOARD_DFLL_FSTEP        1         /* Fine maximum step */
#define BOARD_DFLL_CSTEP        1         /* Coarse maximum step */
#define BOARD_DFLL_GCLK         3         /* GCLK source (if !usbcrm && !mode) */
#define BOARD_DFLL_MUL          0         /* DFLL multiply factor */

/* DPLL0/1
 *
 * Fckr is the frequency of the selected reference clock reference:
 *
 *    BOARD_XOSC32K_FREQENCY,
 *    BOARD_XOSCn_FREQUENCY / DIV, or
 *    BOARD_GCLKn_FREQUENCY
 *
 * The DPLL output frequency is then given by:
 *
 *   Fdpll = Fckr * (LDR + 1 + LDRFRAC / 32)
 *
 * DPLL0:
 *   Fckr  = BOARD_GCLK5_FREQUENCY = BOARD_DFLL_FREQUENCY / 24 = 2MHz
 *   Fdpll = 2Mhz * (59 + 1 + 0 / 32) = 120MHz
 *
 * DPLL1: (not enabled)
 *   Fckr  = BOARD_XOSCK32_FREQUENCY = 32.768KHz
 *   Fdpll = 32768 * (1463 + 1 + 13/32) = 47.986 MHz
 */

#define BOARD_DPLL0_ENABLE      TRUE      /* DPLL enable */
#define BOARD_DPLL0_DCOEN       FALSE     /* DCO filter enable */
#define BOARD_DPLL0_LBYPASS     FALSE     /* Lock bypass */
#define BOARD_DPLL0_WUF         FALSE     /* Wake up fast */
#define BOARD_DPLL0_RUNSTDBY    FALSE     /* Run in standby */
#define BOARD_DPLL0_ONDEMAND    FALSE     /* On demand clock activation */
#define BOARD_DPLL0_REFLOCK     FALSE     /* Do not lock reference clock section */
#define BOARD_DPLL0_REFCLK      0         /* Reference clock selection */
#define BOARD_DPLL0_LTIME       0         /* Lock time  */
#define BOARD_DPLL0_FILTER      0         /* Proportional integer filter selection */
#define BOARD_DPLL0_DCOFILTER   0         /* Sigma-delta DCO filter selection */
#define BOARD_DPLL0_GCLK        5         /* GCLK source (if refclock == 0) */
#define BOARD_DPLL0_GCLKLOCK    0         /* Don't lock GCLK source clock configuration */
#define BOARD_DPLL0_LDRFRAC     0         /* Loop divider fractional part */
#define BOARD_DPLL0_LDRINT      59        /* Loop divider ratio */
#define BOARD_DPLL0_DIV         0         /* Clock divider */

#define BOARD_DPLL1_ENABLE      FALSE     /* DPLL enable */
#define BOARD_DPLL1_DCOEN       FALSE     /* DCO filter enable */
#define BOARD_DPLL1_LBYPASS     FALSE     /* Lock bypass */
#define BOARD_DPLL1_WUF         FALSE     /* Wake up fast */
#define BOARD_DPLL1_RUNSTDBY    FALSE     /* Run in standby */
#define BOARD_DPLL1_ONDEMAND    FALSE     /* On demand clock activation */
#define BOARD_DPLL1_REFLOCK     FALSE     /* Do not lock reference clock section */
#define BOARD_DPLL1_REFCLK      1         /* Reference clock = XOSCK32 */
#define BOARD_DPLL1_LTIME       0         /* Lock time  */
#define BOARD_DPLL1_FILTER      0         /* Sigma-delta DCO filter selection */
#define BOARD_DPLL1_DCOFILTER   0         /* Sigma-delta DCO filter selection */
#define BOARD_DPLL1_GCLK        0         /* GCLK source (if refclock == 0) */
#define BOARD_DPLL1_GCLKLOCK    0         /* Don't lock GCLK source clock configuration */
#define BOARD_DPLL1_LDRFRAC     13        /* Loop divider fractional part */
#define BOARD_DPLL1_LDRINT      1463      /* Loop divider ratio */
#define BOARD_DPLL1_DIV         0         /* Clock divider */

/* Master Clock (MCLK)
 *
 * GCLK0 is always the direct source the GCLK_MAIN.
 * CPU frequency = 120MHz / 1 = 120MHz
 */

#define BOARD_MCLK_CPUDIV       1         /* MCLK divider to get CPU frequency */

#define BOARD_MCK_FREQUENCY     BOARD_GCLK0_FREQUENCY

/* Peripheral clocking */

#define BOARD_GCLK_EIC          4         /* EIC GCLK index */

/* FLASH wait states
 *
 * Vdd Range Wait states Maximum Operating Frequency
 * --------- ----------- ---------------------------
 * > 2.7V    0            24 MHz
 *           1            51 MHz
 *           2            77 MHz
 *           3           101 MHz
 *           4           119 MHz
 *           5           120 MHz
 * >1.71V    0            22 MHz
 *           1            44 MHz
 *           2            67 MHz
 *           3            89 MHz
 *           4           111 MHz
 *           5           120 MHz
 */

#define BOARD_FLASH_WAITSTATES  6

/* LED definitions **********************************************************/

/* LEDs
 *
 *   The SAME54 Xplained Pro has three LEDs,
 *   but only one is controllable by software:
 *
 *   1. LED0 near the edge of the board
 *
 *
 *   ----------------- -----------
 *   SAMD5E5           FUNCTION
 *   ----------------- -----------
 *   PC18              GPIO output
 *
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT     (1 << BOARD_LED0)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   ------------------- ---------------------------- ------
 *   SYMBOL                  Meaning                  LED
 *   ------------------- ---------------------------- ------
 */

#define LED_STARTED      0 /* NuttX has been started  OFF      */
#define LED_HEAPALLOCATE 0 /* Heap has been allocated OFF      */
#define LED_IRQSENABLED  0 /* Interrupts enabled      OFF      */
#define LED_STACKCREATED 1 /* Idle stack created      ON       */
#define LED_INIRQ        2 /* In an interrupt         N/C      */
#define LED_SIGNAL       2 /* In a signal handler     N/C      */
#define LED_ASSERTION    2 /* An assertion failed     N/C      */
#define LED_PANIC        3 /* The system has crashed  FLASH    */
#undef  LED_IDLE           /* MCU is is sleep mode    Not used */

/* Thus is LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Alternate function pin selections ****************************************/

/* SERCOM definitions *******************************************************/

/* The SERCOM bus clock (CLK_SERCOMx_APB) can be enabled and disabled in the
 * Main Clock Controller.
 * The SERCOM uses two generic clocks:
 * GCLK_SERCOMN_CORE and GCLK_SERCOM_SLOW.
 * The core clock (GCLK_SERCOMx_CORE) is required to clock the SERCOM while
 * working as a master.  The slow clock (GCLK_SERCOM_SLOW) is only  required
 * for certain functions and is common to all SERCOM modules.
 *
 * These clocks must be configured and enabled in the Generic Clock
 * Controller (GCLK) before using the SERCOM.
 */

#define BOARD_SERCOM_SLOWGEN         3                   /* 32.768KHz, common to all SERCOMS */
#define BOARD_SERCOM_SLOWLOCK        FALSE               /* Don't lock the SLOWCLOCK */
#define BOARD_SLOWCLOCK_FREQUENCY    BOARD_GCLK3_FREQUENCY

/* SERCOM2
 *
 * Built-in virtual COM port using the EDBG chip on the board.
 * DTR must be asserted by your console software in order to enable this
 * port.
 *
 *   ----------------- ---------
 *   SAMD5E5           FUNCTION
 *   ----------------- ---------
 *   PB24 SERCOM2 PAD1 RXD
 *   PB25 SERCOM2 PAD0 TXD
 *
 * NOTES:
 *   USART_CTRLA_TXPAD0_2: TxD=PAD0 XCK=N/A RTS/TE=PAD2 CTS=PAD3
 *   USART_CTRLA_RXPAD1:   RxD=PAD1
 */

#define BOARD_SERCOM2_MUXCONFIG      (USART_CTRLA_TXPAD0_2 | USART_CTRLA_RXPAD1)
#define BOARD_SERCOM2_PINMAP_PAD0    PORT_SERCOM2_PAD0_4 /* PAD0: USART TX */
#define BOARD_SERCOM2_PINMAP_PAD1    PORT_SERCOM2_PAD1_4 /* PAD1: USART RX */
#define BOARD_SERCOM2_PINMAP_PAD2    0                   /* PAD2: (not used) */
#define BOARD_SERCOM2_PINMAP_PAD3    0                   /* PAD3: (not used) */

#define BOARD_TXIRQ_SERCOM2          SAM_IRQ_SERCOM2_0   /* INTFLAG[0] DRE */
#define BOARD_RXIRQ_SERCOM2          SAM_IRQ_SERCOM2_2   /* INTFLAG[2] RXC */

#define BOARD_SERCOM2_COREGEN        1                   /* 48MHz Core clock */
#define BOARD_SERCOM2_CORELOCK       FALSE               /* Don't lock the CORECLOCK */
#define BOARD_SERCOM2_FREQUENCY      BOARD_GCLK1_FREQUENCY

/* SERCOM3
 *
 * An external RS-232 or serial-to-USB adapter can be connected on pins PA22
 * and PA23:
 *
 *   ----------------- ---------
 *   SAMD5E5           FUNCTION
 *   ----------------- ---------
 *   PA23 SERCOM3 PAD1 RXD
 *   PA22 SERCOM3 PAD0 TXD
 *
 * NOTES:
 *   USART_CTRLA_TXPAD0_2: TxD=PAD0 XCK=N/A RTS/TE=PAD2 CTS=PAD3
 *   USART_CTRLA_RXPAD1:   RxD=PAD1
 */

#define BOARD_SERCOM3_MUXCONFIG      (USART_CTRLA_TXPAD0_2 | USART_CTRLA_RXPAD1)
#define BOARD_SERCOM3_PINMAP_PAD0    PORT_SERCOM3_PAD0_1 /* PAD0: USART TX */
#define BOARD_SERCOM3_PINMAP_PAD1    PORT_SERCOM3_PAD1_1 /* PAD1: USART RX */
#define BOARD_SERCOM3_PINMAP_PAD2    0                   /* PAD2: (not used) */
#define BOARD_SERCOM3_PINMAP_PAD3    0                   /* PAD3: (not used) */

#define BOARD_TXIRQ_SERCOM3          SAM_IRQ_SERCOM3_0   /* INTFLAG[0] DRE */
#define BOARD_RXIRQ_SERCOM3          SAM_IRQ_SERCOM3_2   /* INTFLAG[2] RXC */

#define BOARD_SERCOM3_COREGEN        1                   /* 48MHz Core clock */
#define BOARD_SERCOM3_CORELOCK       FALSE               /* Don't lock the CORECLOCK */
#define BOARD_SERCOM3_FREQUENCY      BOARD_GCLK1_FREQUENCY

/* USB */

#define BOARD_USB_GCLKGEN            1                   /* GCLK1, 48MHz */

/* Ethernet */

#define BOARD_GMAC_GMDC    PORT_GMAC_GMDC_3
#define BOARD_GMAC_GMDIO   PORT_GMAC_GMDIO_3

#endif /* __BOARDS_ARM_SAMD5E5_SAME54_XPLAINED_PRO_INCLUDE_BOARD_H */
