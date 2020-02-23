/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_SAMD2L2_GPIOIRQ
#    include <arch/irq.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Overview
 *
 * Since there is no high speed crystal, we will run from the OSC16M clock
 * source.
 * We will use its default, POR frequency of 4MHz to avoid an additional
 * clock switch.
 *
 * OSC16M              Output = 4MHz
 * `- GCLK1            Input  = 4MHz  Prescaler    = 1  output         = 4MHz
 *     `- DFLL         Input  = 4MHz  Multiplier   = 12 output         = 48MHz
 *         `- GCLK0    Input  = 48MHz Prescaler    = 1  output         = 48MHz
 *             `- MCLK Input  = 48Mhz CPU divider  = 1  CPU frequency  = 48MHz
 *                                    APBA divider = 1  APBA frequency = 48MHz
 *                                    APBB divider = 1  APBB frequency = 48MHz
 *                                    APBC divider = 1  APBC frequency = 48MHz
 *                                    APBD divider = 1  APBD frequency = 48MHz
 *                                    APBE divider = 1  APBE frequency = 48MHz
 *
 * The SAML21 Xplained Pro has one on-board crystal:
 *
 *   XC101 32.768KHz XOSC32
 *
 * REVISIT: Not currently used, may want to use as GCLK1 source with
 * DFLL multiplier of ((48000000+16384)/32768) = 1465 which would yield
 * a clock of 48,005,120 MHz.
 */

/* XOSC Configuration -- Not available
 *
 *   BOARD_XOSC_ENABLE          - Boolean (defined / not defined)
 *   BOARD_XOSC_FREQUENCY       - In Hz
 *   BOARD_XOSC_STARTUPTIME     - See SYSCTRL_XOSC_STARTUP_* definitions
 *   BOARD_XOSC_ISCRYSTAL       - Boolean (defined / not defined)
 *   BOARD_XOSC_AMPGC           - Boolean (defined / not defined)
 *   BOARD_XOSC_ONDEMAND        - Boolean (defined / not defined)
 *   BOARD_XOSC_RUNINSTANDBY    - Boolean (defined / not defined)
 */

#undef  BOARD_XOSC_ENABLE
#define BOARD_XOSC_FREQUENCY         12000000UL
#define BOARD_XOSC_STARTUPTIME       SYSCTRL_XOSC_STARTUP_1S
#define BOARD_XOSC_ISCRYSTAL         1
#define BOARD_XOSC_AMPGC             1
#define BOARD_XOSC_ONDEMAND          1
#undef  BOARD_XOSC_RUNINSTANDBY

/* XOSC32 Configuration -- Not used
 *
 *   BOARD_XOSC32K_ENABLE       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_FREQUENCY    - In Hz
 *   BOARD_XOSC32K_STARTUPTIME  - See SYSCTRL_XOSC32K_STARTUP_* definitions
 *   BOARD_XOSC32K_ISCRYSTAL    - Boolean (defined / not defined)
 *   BOARD_XOSC32K_AAMPEN       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_EN1KHZ       - Boolean (defined / not defined)
 *   BOARD_XOSC32K_EN32KHZ      - Boolean (defined / not defined)
 *   BOARD_XOSC32K_ONDEMAND     - Boolean (defined / not defined)
 *   BOARD_XOSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 *   BOARD_XOSC32K_WRITELOCK    - Boolean (defined / not defined)
 */

#undef BOARD_XOSC32K_ENABLE
#ifdef CONFIG_SAML21_XPLAINED_XOSC32K
#  define BOARD_XOSC32K_ENABLE       1
#  define BOARD_XOSC32K_FREQUENCY    32768    /* 32.768KHz XTAL */
#  define BOARD_XOSC32K_STARTUPTIME  OSC32KCTRL_XOSC32K_STARTUP_100MS
#  define BOARD_XOSC32K_ISCRYSTAL    1
#  undef  BOARD_XOSC32K_EN1KHZ
#  define BOARD_XOSC32K_EN32KHZ      1
#  define BOARD_XOSC32K_ONDEMAND     1
#  undef  BOARD_XOSC32K_RUNINSTANDBY
#  undef  BOARD_XOSC32K_WRITELOCK
#endif

/* OSC32 Configuration -- not used
 *
 *   BOARD_OSC32K_ENABLE        - Boolean (defined / not defined)
 *   BOARD_OSC32K_FREQUENCY     - In Hz
 *   BOARD_OSC32K_STARTUPTIME   - See SYSCTRL_OSC32K_STARTUP_* definitions
 *   BOARD_OSC32K_EN1KHZ        - Boolean (defined / not defined)
 *   BOARD_OSC32K_EN32KHZ       - Boolean (defined / not defined)
 *   BOARD_OSC32K_ONDEMAND      - Boolean (defined / not defined)
 *   BOARD_OSC32K_RUNINSTANDBY  - Boolean (defined / not defined)
 *   BOARD_OSC32K_WRITELOCK     - Boolean (defined / not defined)
 */

#undef  BOARD_OSC32K_ENABLE
#define BOARD_OSC32K_FREQUENCY       32768    /* 32.768kHz internal oscillator */
#define BOARD_OSC32K_STARTUPTIME     SYSCTRL_OSC32K_STARTUP_4MS
#define BOARD_OSC32K_EN1KHZ          1
#define BOARD_OSC32K_EN32KHZ         1
#define BOARD_OSC32K_ONDEMAND        1
#undef  BOARD_OSC32K_RUNINSTANDBY
#undef  BOARD_OSC32K_WRITELOCK

/* OSC16M Configuration -- always enabled
 *
 *   BOARD_OSC16M_FSEL          - See OSCCTRL_OSC16MCTRL_FSEL_* definitions
 *   BOARD_OSC16M_ONDEMAND      - Boolean (defined / not defined)
 *   BOARD_OSC16M_RUNINSTANDBY  - Boolean (defined / not defined)
 */

#if defined(CONFIG_SAML21_XPLAINED_OSC16M_4MHZ)
#  define BOARD_OSC16M_FSEL          OSCCTRL_OSC16MCTRL_FSEL_4MHZ
#  define BOARD_OSC16M_ONDEMAND      1
#  undef  BOARD_OSC16M_RUNINSTANDBY
#  define BOARD_OSC16M_FREQUENCY     4000000  /* 4MHz high-accuracy internal oscillator */

#elif defined(CONFIG_SAML21_XPLAINED_OSC16M_8MHZ)
#  define BOARD_OSC16M_FSEL          OSCCTRL_OSC16MCTRL_FSEL_8MHZ
#  define BOARD_OSC16M_ONDEMAND      1
#  undef  BOARD_OSC16M_RUNINSTANDBY
#  define BOARD_OSC16M_FREQUENCY     8000000  /* 8MHz high-accuracy internal oscillator */

#elif defined(CONFIG_SAML21_XPLAINED_OSC16M_12MHZ)
#  define BOARD_OSC16M_FSEL          OSCCTRL_OSC16MCTRL_FSEL_12MHZ
#  define BOARD_OSC16M_ONDEMAND      1
#  undef  BOARD_OSC16M_RUNINSTANDBY
#  define BOARD_OSC16M_FREQUENCY     12000000 /* 12MHz high-accuracy internal oscillator */

#elif defined(CONFIG_SAML21_XPLAINED_OSC16M_16MHZ)
#  define BOARD_OSC16M_FSEL          OSCCTRL_OSC16MCTRL_FSEL_16MHZ
#  define BOARD_OSC16M_ONDEMAND      1
#  undef  BOARD_OSC16M_RUNINSTANDBY
#  define BOARD_OSC16M_FREQUENCY     16000000 /* 18MHz high-accuracy internal oscillator */

#else
#  error OSC16M operating frequency not defined (CONFIG_SAML21_XPLAINED_OSC16M_*MHZ)
#endif

/* OSCULP32K Configuration -- not used. */

#define BOARD_OSCULP32K_FREQUENCY    32000    /* 32kHz ultra-low-power internal oscillator */

/* Digital Frequency Locked Loop configuration.  In closed-loop mode, the
 * DFLL output frequency (Fdfll) is given by:
 *
 *  Fdfll = DFLLmul * Frefclk
 *        = 12 * 4000000 = 48MHz
 *
 * Where the reference clock is Generic Clock Channel 0 output of GLCK1.
 * GCLCK1 provides OSC16M, undivided.
 *
 * When operating in open-loop mode, the output frequency of the DFLL will
 * be determined by the values written to the DFLL Coarse Value bit group
 * and the DFLL Fine Value bit group in the DFLL Value register.
 *
 *   BOARD_DFLL48M_CLOSEDLOOP          - Boolean (defined / not defined)
 *   BOARD_DFLL48M_OPENLOOP            - Boolean (defined / not defined)
 *   BOARD_DFLL48M_RECOVERY            - Boolean (defined / not defined)
 *   BOARD_DFLL48M_TRACKAFTERFINELOCK  - Boolean (defined / not defined)
 *   BOARD_DFLL48M_KEEPLOCKONWAKEUP    - Boolean (defined / not defined)
 *   BOARD_DFLL48M_ENABLECHILLCYCLE    - Boolean (defined / not defined)
 *   BOARD_DFLL48M_QUICKLOCK           - Boolean (defined / not defined)
 *   BOARD_DFLL48M_RUNINSTDBY          - Boolean (defined / not defined)
 *   BOARD_DFLL48M_ONDEMAND            - Boolean (defined / not defined)
 *
 * Closed loop mode only:
 *   BOARD_DFLL48M_REFCLK_CLKGEN       - GCLK index in the range {0..8}
 *   BOARD_DFLL48M_MULTIPLIER          - Value
 *   BOARD_DFLL48M_MAXCOARSESTEP       - Value
 *   BOARD_DFLL48M_MAXFINESTEP         - Value
 *
 *   BOARD_DFLL48M_FREQUENCY           - The resulting frequency
 */

#undef BOARD_DFLL48M_ENABLE                1  /* Assume not using the DFLL48M */
#undef BOARD_DFLL48M_CLOSEDLOOP
#undef BOARD_DFLL48M_OPENLOOP
#undef BOARD_DFLL48M_RECOVERY

#ifdef CONFIG_SAML21_XPLAINED_DFLL
#  define BOARD_DFLL48M_ENABLE             1  /* Using the DFLL48M */

/* DFLL mode of operation */

#  if defined(CONFIG_SAML21_XPLAINED_DFLL_OPENLOOP)
#    define BOARD_DFLL48M_OPENLOOP         1  /* In open loop mode */
#  elif defined(CONFIG_SAML21_XPLAINED_DFLL_CLOSEDLOOP)
#    define BOARD_DFLL48M_CLOSEDLOOP       1  /* In closed loop mode */
#  elif defined(CONFIG_SAML21_XPLAINED_DFLL_RECOVERY)
#    define BOARD_DFLL48M_RECOVERY         1  /* In USB recover mode */
#  else
#    error DFLL mode not provided
#  endif

/* DFLL source clock selection */

#  if defined(CONFIG_SAML21_XPLAINED_DFLL_OSC16MSRC)
#    define BOARD_DFLL_SRC_FREQUENCY       BOARD_OSC16M_FREQUENCY
#  elif defined(CONFIG_SAML21_XPLAINED_DFLL_XOSC32KSRC)
#    define BOARD_DFLL_SRC_FREQUENCY       BOARD_XOSC32K_FREQUENCY
#  else
#    error DFLL clock source not provided
#  endif

/* Mode-independent options */

#  undef  BOARD_DFLL48M_RUNINSTDBY
#  undef  BOARD_DFLL48M_ONDEMAND

/* DFLL closed loop mode configuration */

#  define BOARD_DFLL48M_REFCLK_CLKGEN      1
#  define BOARD_DFLL48M_MULTIPLIER         (48000000 / BOARD_DFLL_SRC_FREQUENCY)
#  define BOARD_DFLL48M_QUICKLOCK          1
#  define BOARD_DFLL48M_TRACKAFTERFINELOCK 1
#  define BOARD_DFLL48M_KEEPLOCKONWAKEUP   1
#  define BOARD_DFLL48M_ENABLECHILLCYCLE   1
#  define BOARD_DFLL48M_MAXCOARSESTEP      (0x1f / 4)
#  define BOARD_DFLL48M_MAXFINESTEP        (0xff / 4)

#  ifdef CONFIG_SAML21_XPLAINED_DFLL_OPENLOOP
#    define BOARD_DFLL48M_FREQUENCY        (13720000) /* REVISIT:  Needs to be measured */
#  else
#    define BOARD_DFLL48M_FREQUENCY        (BOARD_DFLL48M_MULTIPLIER * BOARD_DFLL_SRC_FREQUENCY)
#  endif
#endif

/* Fractional Digital Phase Locked Loop configuration.
 *
 *   BOARD_FDPLL96M_ENABLE          - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_RUNINSTDBY      - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_ONDEMAND        - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_LBYPASS         - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_WUF             - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_LPEN            - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_FILTER          - See OSCCTRL_DPLLCTRLB_FILTER_* definitions
 *   BOARD_FDPLL96M_REFCLK          - See  OSCCTRL_DPLLCTRLB_REFLCK_* definitions
 *   BOARD_FDPLL96M_REFCLK_CLKGEN   - GCLK index in the range {0..8}
 *   BOARD_FDPLL96M_LOCKTIME_ENABLE - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_LOCKTIME        - See OSCCTRL_DPLLCTRLB_LTIME_* definitions
 *   BOARD_FDPLL96M_LOCKTIME_CLKGEN - GCLK index in the range {0..8}
 *   BOARD_FDPLL96M_REFDIV          - Numeric value, 1 - 2047
 *   BOARD_FDPLL96M_PRESCALER       - See OSCCTRL_DPLLPRESC_* definitions
 *   BOARD_FDPLL96M_REFFREQ         - Numeric value
 *   BOARD_FDPLL96M_FREQUENCY       - Numeric value
 */

#undef  BOARD_FDPLL96M_ENABLE
#undef  BOARD_FDPLL96M_RUNINSTDBY
#define BOARD_FDPLL96M_ONDEMAND          1
#undef  BOARD_FDPLL96M_LBYPASS
#undef  BOARD_FDPLL96M_WUF
#undef  BOARD_FDPLL96M_LPEN
#define BOARD_FDPLL96M_FILTER            OSCCTRL_DPLLCTRLB_FILTER_DEFAULT
#define BOARD_FDPLL96M_REFCLK            OSCCTRL_DPLLCTRLB_REFLCK_XOSCK32K
#define BOARD_FDPLL96M_REFCLK_CLKGEN     1
#undef  BOARD_FDPLL96M_LOCKTIME_ENABLE
#define BOARD_FDPLL96M_LOCKTIME          OSCCTRL_DPLLCTRLB_LTIME_NONE
#define BOARD_FDPLL96M_LOCKTIME_CLKGEN   1
#define BOARD_FDPLL96M_REFDIV            1
#define BOARD_FDPLL96M_PRESCALER         OSCCTRL_DPLLPRESC_DIV1

#define BOARD_FDPLL96M_REFFREQ           32768
#define BOARD_FDPLL96M_FREQUENCY         48000000

/* GCLK Configuration
 *
 * Global enable/disable.
 *
 *   BOARD_GCLK_ENABLE            - *MUST* be defined
 *
 * For n=1-7:
 *   BOARD_GCLKn_ENABLE           - Boolean (defined / not defined)
 *
 * For n=0-8:
 *   BOARD_GCLKn_RUN_IN_STANDBY   - Boolean (defined / not defined)
 *   BOARD_GCLKn_CLOCK_SOURCE     - See GCLK_GENCTRL_SRC_* definitions
 *   BOARD_GCLKn_PRESCALER        - Value
 *   BOARD_GCLKn_OUTPUT_ENABLE    - Boolean (defined / not defined)
 */

#define BOARD_GCLK_ENABLE             1

/* GCLK generator 0 (Main Clock) - Source is the DFLL */

#ifdef CONFIG_SAML21_XPLAINED_DFLL
#  undef  BOARD_GCLK0_RUN_IN_STANDBY
#  define BOARD_GCLK0_CLOCK_SOURCE    GCLK_GENCTRL_SRC_DFLL48M
#  define BOARD_GCLK0_PRESCALER       1
#  undef  BOARD_GCLK0_OUTPUT_ENABLE
#  define BOARD_GCLK0_FREQUENCY       (BOARD_DFLL48M_FREQUENCY / BOARD_GCLK0_PRESCALER)
#else
#  undef  BOARD_GCLK0_RUN_IN_STANDBY
#  define BOARD_GCLK0_CLOCK_SOURCE    GCLK_GENCTRL_SRC_OSC16M
#  define BOARD_GCLK0_PRESCALER       1
#  undef  BOARD_GCLK0_OUTPUT_ENABLE
#  define BOARD_GCLK0_FREQUENCY       (BOARD_OSC16M_FREQUENCY / BOARD_GCLK0_PRESCALER)
#endif

/* Configure GCLK generator 1 - Drives the DFLL */

#if defined(CONFIG_SAML21_XPLAINED_DFLL_OSC16MSRC)
#  define BOARD_GCLK1_ENABLE          1
#  undef  BOARD_GCLK1_RUN_IN_STANDBY
#  define BOARD_GCLK1_CLOCK_SOURCE    GCLK_GENCTRL_SRC_OSC16M
#  define BOARD_GCLK1_PRESCALER       1
#  undef  BOARD_GCLK1_OUTPUT_ENABLE
#  define BOARD_GCLK1_FREQUENCY       (BOARD_OSC16M_FREQUENCY / BOARD_GCLK1_PRESCALER)
#elif defined(CONFIG_SAML21_XPLAINED_DFLL_XOSC32KSRC)
#  define BOARD_GCLK1_ENABLE          1
#  undef  BOARD_GCLK1_RUN_IN_STANDBY
#  define BOARD_GCLK1_CLOCK_SOURCE    GCLK_GENCTRL_SRC_XOSC32K
#  define BOARD_GCLK1_PRESCALER       1
#  undef  BOARD_GCLK1_OUTPUT_ENABLE
#  define BOARD_GCLK1_FREQUENCY       (BOARD_XOSC32K_FREQUENCY / BOARD_GCLK1_PRESCALER)
#else
#  error DFLL clock source not provided
#endif

/* Configure GCLK generator 2 (RTC) */

#undef  BOARD_GCLK2_ENABLE
#undef  BOARD_GCLK2_RUN_IN_STANDBY
#define BOARD_GCLK2_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC32K
#define BOARD_GCLK2_PRESCALER         32
#undef  BOARD_GCLK2_OUTPUT_ENABLE
#define BOARD_GCLK2_FREQUENCY         (BOARD_OSC32K_FREQUENCY / BOARD_GCLK2_PRESCALER)

/* Configure GCLK generator 3 */

#undef  BOARD_GCLK3_ENABLE
#undef  BOARD_GCLK3_RUN_IN_STANDBY
#define BOARD_GCLK3_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK3_PRESCALER         1
#undef  BOARD_GCLK3_OUTPUT_ENABLE
#define BOARD_GCLK3_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK3_PRESCALER)

/* Configure GCLK generator 4 */

#undef  BOARD_GCLK4_ENABLE
#undef  BOARD_GCLK4_RUN_IN_STANDBY
#define BOARD_GCLK4_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK4_PRESCALER         1
#undef  BOARD_GCLK4_OUTPUT_ENABLE
#define BOARD_GCLK4_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK4_PRESCALER)

/* Configure GCLK generator 5 */

#undef  BOARD_GCLK5_ENABLE
#undef  BOARD_GCLK5_RUN_IN_STANDBY
#define BOARD_GCLK5_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK5_PRESCALER         1
#undef  BOARD_GCLK5_OUTPUT_ENABLE
#define BOARD_GCLK5_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK5_PRESCALER)

/* Configure GCLK generator 6 */

#undef  BOARD_GCLK6_ENABLE
#undef  BOARD_GCLK6_RUN_IN_STANDBY
#define BOARD_GCLK6_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK6_PRESCALER         1
#undef  BOARD_GCLK6_OUTPUT_ENABLE
#define BOARD_GCLK6_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK6_PRESCALER)

/* Configure GCLK generator 7 */

#undef  BOARD_GCLK7_ENABLE
#undef  BOARD_GCLK7_RUN_IN_STANDBY
#define BOARD_GCLK7_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK7_PRESCALER         1
#undef  BOARD_GCLK7_OUTPUT_ENABLE
#define BOARD_GCLK7_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK7_PRESCALER)

/* Configure GCLK generator 8 */

#undef  BOARD_GCLK8_ENABLE
#undef  BOARD_GCLK8_RUN_IN_STANDBY
#define BOARD_GCLK8_CLOCK_SOURCE      GCLK_GENCTRL_SRC_OSC16M
#define BOARD_GCLK8_PRESCALER         1
#undef  BOARD_GCLK8_OUTPUT_ENABLE
#define BOARD_GCLK8_FREQUENCY         (BOARD_OSC16M_FREQUENCY / BOARD_GCLK8_PRESCALER)

/* The source of the main clock is always GCLK_MAIN.  Also called GCLKGEN[0],
 * this is the clock feeding the Power Manager.
 * The Power Manager, in turn, generates main clock which is divided
 * down to produce the CPU, AHB, and APB clocks.
 *
 * The main clock is initially OSC16M divided by 8.
 */

#define BOARD_GCLK_MAIN_FREQUENCY     BOARD_GCLK0_FREQUENCY

/* Main clock dividers
 *
 *     BOARD_CPU_DIVIDER        - See MCLK_CPUDIV_DIV* definitions
 *     BOARD_CPU_FRQUENCY       - In Hz
 *     BOARD_CPU_FAILDECT       - Boolean (defined / not defined)
 *     BOARD_LOWPOWER_DIVIDER   - See MCLK_LPDIV_DIV_* definitions
 *     BOARD_LOWPOWER_FREQUENCY - In Hz
 *     BOARD_BACKUP_DIVIDER     - See MCLK_BUPDIV_DIV_* definitions
 *     BOARD_BACKUP_FREQUENCY   - In Hz
 */

#undef  BOARD_CPU_FAILDECT
#define BOARD_CPU_DIVIDER            MCLK_CPUDIV_DIV1
#define BOARD_LOWPOWER_DIVIDER       MCLK_LPDIV_DIV1
#define BOARD_BACKUP_DIVIDER         MCLK_BUPDIV_DIV1

/* Resulting frequencies */

#define BOARD_MCK_FREQUENCY          (BOARD_GCLK_MAIN_FREQUENCY)
#define BOARD_CPU_FREQUENCY          (BOARD_MCK_FREQUENCY)
#define BOARD_LOWPOWER_FREQUENCY     (BOARD_MCK_FREQUENCY)
#define BOARD_BACKUP_FREQUENCY       (BOARD_MCK_FREQUENCY)
#define BOARD_APBA_FREQUENCY         (BOARD_MCK_FREQUENCY)
#define BOARD_APBB_FREQUENCY         (BOARD_MCK_FREQUENCY)
#define BOARD_APBC_FREQUENCY         (BOARD_MCK_FREQUENCY)
#define BOARD_APBD_FREQUENCY         (BOARD_MCK_FREQUENCY)
#define BOARD_APBE_FREQUENCY         (BOARD_MCK_FREQUENCY)

/* FLASH wait states
 *
 * REVISIT: These values come from the SAMD20
 * Vdd Range     Wait states    Maximum Operating Frequency
 * ------------- -------------- ---------------------------
 * 1.62V to 2.7V  0             14 MHz
 *                1             28 MHz
 *                2             42 MHz
 *                3             48 MHz
 * 2.7V to 3.63V  0             24 MHz
 *                1             48 MHz
 */

#if 0
#  define BOARD_FLASH_WAITSTATES     3
#else
#  define BOARD_FLASH_WAITSTATES     1
#endif

/* SERCOM definitions *******************************************************/

/* This is the source clock generator for the GCLK_SERCOM_SLOW clock that is
 * common to SERCOM modules 0-4.
 * It will generate clocking on the common SERCOM0-4 channel.
 *
 * SERCOM5 uses a different channel and will probably need to use a different
 * GCLK generator.
 */

#define BOARD_SERCOM04_SLOW_GCLKGEN  0

/* SERCOM0 SPI is available on EXT1
 *
 *  PIN PORT SERCOM        FUNCTION
 *  --- ------------------ -----------
 *  15  PA5  SERCOM0 PAD1  SPI SS
 *  16  PA6  SERCOM0 PAD2  SPI MOSI
 *  17  PA4  SERCOM0 PAD0  SPI MISO
 *  18  PA7  SERCOM0 PAD3  SPI SCK
 */

#define BOARD_SERCOM0_GCLKGEN        0
#define BOARD_SERCOM0_SLOW_GCLKGEN   BOARD_SERCOM04_SLOW_GCLKGEN
#define BOARD_SERCOM0_MUXCONFIG      (SPI_CTRLA_DOPO_DOPAD231 | SPI_CTRLA_DIPAD0)
#define BOARD_SERCOM0_PINMAP_PAD0    PORT_SERCOM0_PAD0_2 /* SPI_MISO */
#define BOARD_SERCOM0_PINMAP_PAD1    0                   /* SPI_SS (not used) */
#define BOARD_SERCOM0_PINMAP_PAD2    PORT_SERCOM0_PAD2_2 /* SPI_MOSI */
#define BOARD_SERCOM0_PINMAP_PAD3    PORT_SERCOM0_PAD3_2 /* SPI_SCK */

#define BOARD_SERCOM0_FREQUENCY      BOARD_GCLK0_FREQUENCY

/* SERCOM1 USART is available on EXT2 and EXT3
 *
 *   PIN   EXT1 EXT2 EXT3 GPIO Function
 *   ----  ---- ---- ---- ------------------
 *    13   ---  PA19 PA19 SERCOM1 / USART RX
 *    14   ---  PA18 PA18 SERCOM1 / USART TX
 *    19   GND  GND  GND  N/A
 *    20   VCC  VCC  VCC  N/A
 */

#define BOARD_SERCOM1_GCLKGEN        0
#define BOARD_SERCOM1_SLOW_GCLKGEN   BOARD_SERCOM04_SLOW_GCLKGEN
#define BOARD_SERCOM1_MUXCONFIG      (USART_CTRLA_TXPAD2 | USART_CTRLA_RXPAD3)
#define BOARD_SERCOM1_PINMAP_PAD0    0                   /* (not used) */
#define BOARD_SERCOM1_PINMAP_PAD1    0                   /* (not used) */
#define BOARD_SERCOM1_PINMAP_PAD2    PORT_SERCOM1_PAD2_1 /* USART_TX */
#define BOARD_SERCOM1_PINMAP_PAD3    PORT_SERCOM1_PAD3_1 /* USART_RX */

#define BOARD_SERCOM1_FREQUENCY      BOARD_GCLK0_FREQUENCY

/* The SAML21 Xplained Pro contains an Embedded Debugger (EDBG) that can be
 * used to program and debug the ATSAML21J18A using Serial Wire Debug (SWD).
 * The Embedded debugger also include a Virtual COM port interface over
 * SERCOM3.  Virtual COM port connections:
 *
 *   PA22 SERCOM3 PAD[0] / USART TXD
 *   PA23 SERCOM3 PAD[1] / USART RXD
 */

#define BOARD_SERCOM3_GCLKGEN        0
#define BOARD_SERCOM3_SLOW_GCLKGEN   BOARD_SERCOM04_SLOW_GCLKGEN
#define BOARD_SERCOM3_MUXCONFIG      (USART_CTRLA_RXPAD1 | USART_CTRLA_TXPAD0_2)
#define BOARD_SERCOM3_PINMAP_PAD0    PORT_SERCOM3_PAD0_1 /* USART TX */
#define BOARD_SERCOM3_PINMAP_PAD1    PORT_SERCOM3_PAD1_1 /* USART RX */
#define BOARD_SERCOM3_PINMAP_PAD2    0                   /* (not used) */
#define BOARD_SERCOM3_PINMAP_PAD3    0                   /* (not used) */

#define BOARD_SERCOM3_FREQUENCY      BOARD_GCLK0_FREQUENCY

/* The SERCOM4 USART is available on EXT1
 *
 *   PIN   EXT1 EXT2 EXT3 GPIO Function
 *   ----  ---- ---- ---- ------------------
 *    13   PB09 ---  ---  SERCOM4 PAD[1] / USART RX
 *    14   PB08 ---  ---  SERCOM4 PAD[0] / USART TX
 *    19   GND  GND  GND  N/A
 *    20   VCC  VCC  VCC  N/A
 */

#define BOARD_SERCOM4_GCLKGEN        0
#define BOARD_SERCOM4_SLOW_GCLKGEN   BOARD_SERCOM04_SLOW_GCLKGEN
#define BOARD_SERCOM4_MUXCONFIG      (USART_CTRLA_RXPAD1 | USART_CTRLA_TXPAD0_2)
#define BOARD_SERCOM4_PINMAP_PAD0    PORT_SERCOM4_PAD0_3 /* USART TX */
#define BOARD_SERCOM4_PINMAP_PAD1    PORT_SERCOM4_PAD1_3 /* USART RX */
#define BOARD_SERCOM4_PINMAP_PAD2    0
#define BOARD_SERCOM4_PINMAP_PAD3    0

#define BOARD_SERCOM4_FREQUENCY      BOARD_GCLK0_FREQUENCY

/*  SERCOM5 is dedicated for use with SPI at the EXT2 and EXT3 connectors
 *
 *  PIN PORT SERCOM        FUNCTION
 *  --- ------------------ -----------
 *  15  PB17 SERCOM5 PAD1  SPI SS
 *  16  PB22 SERCOM5 PAD2  SPI MOSI
 *  17  PB16 SERCOM5 PAD0  SPI MISO
 *  18  PB23 SERCOM5 PAD3  SPI SCK
 */

#define BOARD_SERCOM5_GCLKGEN        0
#define BOARD_SERCOM5_SLOW_GCLKGEN   ?
#define BOARD_SERCOM5_MUXCONFIG      (SPI_CTRLA_DOPO_DOPAD231 | SPI_CTRLA_DIPAD0)
#define BOARD_SERCOM5_PINMAP_PAD0    PORT_SERCOM5_PAD0_1 /* SPI_MISO */
#define BOARD_SERCOM5_PINMAP_PAD1    0                   /* SPI_SS (not used) */
#define BOARD_SERCOM5_PINMAP_PAD2    PORT_SERCOM5_PAD2_4 /* SPI_MOSI */
#define BOARD_SERCOM5_PINMAP_PAD3    PORT_SERCOM5_PAD3_4 /* SPI_SCK */

#define BOARD_SERCOM5_FREQUENCY      BOARD_GCLK0_FREQUENCY

/* USB definitions **********************************************************/

/* This is the source clock generator for the GCLK_USB clock
 */

#define BOARD_USB_GCLKGEN            0
#define BOARD_USB_FREQUENCY          BOARD_GCLK0_FREQUENCY

/* default USB Pad calibration (not used yet by USB driver) */

#define BOARD_USB_PADCAL_P       29
#define BOARD_USB_PADCAL_N       5
#define BOARD_USB_PADCAL_TRIM    3

/* LED definitions **********************************************************/

/* There are three LEDs on board the SAML21 Xplained Pro board:  The EDBG
 * controls two of the LEDs, a power LED and a status LED.  There is only
 * one user controllable LED, a yellow LED labelled STATUS near the SAML21
 * USB connector.
 *
 * This LED is controlled by PC07 and the LED can be activated by driving
 * the PB10 to GND.
 */

/* LED index values for use with board_userled() */

#define BOARD_STATUS_LED             0
#define BOARD_NLEDS                  1

/* LED bits for use with board_userled_all() */

#define BOARD_STATUS LED_BIT         (1 << BOARD_STATUS_LED)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as defined below.
 * Thus if the LED is statically on, NuttX has successfully booted and is,
 * apparently, running normally.
 * If the LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

#define LED_STARTED                  0 /* STATUS LED=OFF */
#define LED_HEAPALLOCATE             0 /* STATUS LED=OFF */
#define LED_IRQSENABLED              0 /* STATUS LED=OFF */
#define LED_STACKCREATED             1 /* STATUS LED=ON */
#define LED_INIRQ                    2 /* STATUS LED=no change */
#define LED_SIGNAL                   2 /* STATUS LED=no change */
#define LED_ASSERTION                2 /* STATUS LED=no change */
#define LED_PANIC                    3 /* STATUS LED=flashing */

/* Button definitions *******************************************************/

/* Mechanical buttons:
 *
 * The SAML21 Xplained Pro contains two mechanical buttons.
 * One button is the RESET button connected to the SAML21 reset line and the
 * other is a generic user configurable button.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PA02 SW0
 */

/* The SAML21 Xplained Pro supports one button: */

#define BUTTON_SW0                   0
#define NUM_BUTTONS                  1

#define BUTTON_SW0_BIT               (1 << BUTTON_SW0)

#endif /* __BOARDS_ARM_SAMD2L2_SAML21_XPLAINED_INCLUDE_BOARD_H */
