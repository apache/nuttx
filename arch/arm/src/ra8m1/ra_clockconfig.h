/****************************************************************************
 * arch/arm/src/ra8m1/ra_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_RA_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_RA8M1_RA_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "hardware/ra8m1_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock frequencies derived from the BOARD_* clock options.
 *
 * The clock tree is chosen by the board in <arch/board/board.h> (see the
 * "Clocking" section of that file for every option and its valid values).
 * Everything here is a compile-time constant, and every limit in the RA8M1
 * User's Manual chapter 8 (Tables 8.1 and 8.2 and their notes) is checked
 * with #error, so a missing option or an invalid combination fails to build
 * instead of producing a clock the MCU is not specified for.  Some of the
 * results are unsigned long long constants (the PLL VCO does not fit in 32
 * bits during the calculation); cast them where a plain integer is needed.
 */

/* Options the board has to define.  There is no default: a board.h that
 * does not describe its clock tree does not build.
 */

#ifndef BOARD_HOCO_FREQUENCY
#  error "board.h must define BOARD_HOCO_FREQUENCY"
#endif
#ifndef BOARD_SYSCLK_SOURCE
#  error "board.h must define BOARD_SYSCLK_SOURCE"
#endif
#ifndef BOARD_OFS2_DCDC
#  error "board.h must define BOARD_OFS2_DCDC"
#endif
#if !defined(BOARD_CPUCLK_DIVIDER) || !defined(BOARD_ICLK_DIVIDER)  || \
    !defined(BOARD_PCLKA_DIVIDER)  || !defined(BOARD_PCLKB_DIVIDER) || \
    !defined(BOARD_PCLKC_DIVIDER)  || !defined(BOARD_PCLKD_DIVIDER) || \
    !defined(BOARD_PCLKE_DIVIDER)  || !defined(BOARD_FCLK_DIVIDER)  || \
    !defined(BOARD_BCLK_DIVIDER)
#  error "board.h must define BOARD_CPUCLK/ICLK/PCLKA-E/FCLK/BCLK_DIVIDER"
#endif

/* SCICLK source selectors, for BOARD_SCICLK_SOURCE.
 *
 * Every other *_SOURCE option reuses the real register-field value from
 * hardware/ra8m1_system.h (see board.h).  SCICKCR.SCICKSEL has no such
 * generated names -- the SVD gives this field no enumeratedValues -- so
 * this is the one selector this port has to invent, and it is kept here,
 * private to this file, rather than in board.h: board.h only references
 * these names (no #include needed for that -- a macro's replacement text
 * is resolved where it is used, not where it is written), and the actual
 * SCICKCR.SCICKSEL codes these translate to are computed in
 * ra_clockconfig.c, independently of the numbering used here.
 */

#define RA_SCICLK_SRC_MOCO        1
#define RA_SCICLK_SRC_HOCO        2
#define RA_SCICLK_SRC_MOSC        3
#define RA_SCICLK_SRC_PLL1P       4
#define RA_SCICLK_SRC_PLL1Q       5
#define RA_SCICLK_SRC_PLL1R       6
#define RA_SCICLK_SRC_PLL2P       7
#define RA_SCICLK_SRC_PLL2Q       8
#define RA_SCICLK_SRC_PLL2R       9

/* Which oscillators, PLLs and dedicated clocks the configuration needs.
 * Each RA_USE_* is 0 or 1.
 *
 * SCICLK is needed as soon as one SCI UART is enabled: the baud rate
 * generator of every SCI runs from it.
 */

#if defined(CONFIG_RA_SCI0_UART) || defined(CONFIG_RA_SCI1_UART) || \
    defined(CONFIG_RA_SCI2_UART) || defined(CONFIG_RA_SCI3_UART) || \
    defined(CONFIG_RA_SCI4_UART) || defined(CONFIG_RA_SCI9_UART)
#  define RA_USE_SCICLK           1
#else
#  define RA_USE_SCICLK           0
#endif

#if RA_USE_SCICLK
#  if !defined(BOARD_SCICLK_SOURCE) || !defined(BOARD_SCICLK_DIVIDER)
#    error "board.h must define BOARD_SCICLK_SOURCE and BOARD_SCICLK_DIVIDER"
#  endif
#endif

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_PLL || \
    (RA_USE_SCICLK && (BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1P || \
                       BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1Q || \
                       BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1R))
#  define RA_USE_PLL1             1
#else
#  define RA_USE_PLL1             0
#endif

#if RA_USE_SCICLK && (BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2P || \
                      BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2Q || \
                      BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2R)
#  define RA_USE_PLL2             1
#else
#  define RA_USE_PLL2             0
#endif

#if RA_USE_PLL1
#  if !defined(BOARD_PLL1_SOURCE) || !defined(BOARD_PLL1_INPUT_DIVIDER) || \
      !defined(BOARD_PLL1_MULTIPLIER) || !defined(BOARD_PLL1_MULTIPLIER_FRACTION) || \
      !defined(BOARD_PLL1_DIVIDER_P) || !defined(BOARD_PLL1_DIVIDER_Q) || \
      !defined(BOARD_PLL1_DIVIDER_R)
#    error "PLL1 is used: board.h must define all the BOARD_PLL1_* options"
#  endif
#endif

#if RA_USE_PLL2
#  if !defined(BOARD_PLL2_SOURCE) || !defined(BOARD_PLL2_INPUT_DIVIDER) || \
      !defined(BOARD_PLL2_MULTIPLIER) || !defined(BOARD_PLL2_MULTIPLIER_FRACTION) || \
      !defined(BOARD_PLL2_DIVIDER_P) || !defined(BOARD_PLL2_DIVIDER_Q) || \
      !defined(BOARD_PLL2_DIVIDER_R)
#    error "PLL2 is used: board.h must define all the BOARD_PLL2_* options"
#  endif
#endif

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_HOCO || \
    (RA_USE_SCICLK && BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_HOCO) || \
    (RA_USE_PLL1 && BOARD_PLL1_SOURCE == R_SYSTEM_PLLCCR_PLSRCSEL) || \
    (RA_USE_PLL2 && BOARD_PLL2_SOURCE == R_SYSTEM_PLL2CCR_PL2SRCSEL)
#  define RA_USE_HOCO             1
#else
#  define RA_USE_HOCO             0
#endif

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_MAIN_CLOCK_OSCILLATOR || \
    (RA_USE_SCICLK && BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_MOSC) || \
    (RA_USE_PLL1 && BOARD_PLL1_SOURCE == 0) || \
    (RA_USE_PLL2 && BOARD_PLL2_SOURCE == 0)
#  define RA_USE_MOSC             1
#else
#  define RA_USE_MOSC             0
#endif

#if BOARD_SYSCLK_SOURCE != R_SYSTEM_SCKSCR_CKSEL_MOCO && \
    BOARD_SYSCLK_SOURCE != R_SYSTEM_SCKSCR_CKSEL_HOCO && \
    BOARD_SYSCLK_SOURCE != R_SYSTEM_SCKSCR_CKSEL_MAIN_CLOCK_OSCILLATOR && \
    BOARD_SYSCLK_SOURCE != R_SYSTEM_SCKSCR_CKSEL_PLL
#  error "BOARD_SYSCLK_SOURCE must be R_SYSTEM_SCKSCR_CKSEL_MOCO, _HOCO, _MAIN_CLOCK_OSCILLATOR or _PLL"
#endif

/* Oscillators */

#define RA_MOCO_FREQUENCY         8000000

/* The HOCO frequency after reset is the one programmed into OFS1 (see
 * ra_option_setting.c), so BOARD_HOCO_FREQUENCY is used for both.
 */

#if BOARD_HOCO_FREQUENCY != 16000000 && BOARD_HOCO_FREQUENCY != 18000000 && \
    BOARD_HOCO_FREQUENCY != 20000000 && BOARD_HOCO_FREQUENCY != 32000000 && \
    BOARD_HOCO_FREQUENCY != 48000000
#  error "BOARD_HOCO_FREQUENCY must be 16, 18, 20, 32 or 48 MHz"
#endif

#define RA_HOCO_FREQUENCY         BOARD_HOCO_FREQUENCY

#if RA_USE_MOSC
#  if !defined(BOARD_MOSC_FREQUENCY) || !defined(BOARD_MOSC_EXTERNAL_CLOCK) || \
      !defined(BOARD_MOSC_STABILIZATION_US)
#    error "MOSC is used: board.h must define BOARD_MOSC_FREQUENCY, BOARD_MOSC_EXTERNAL_CLOCK and BOARD_MOSC_STABILIZATION_US"
#  endif
#  if BOARD_MOSC_FREQUENCY < 8000000 || BOARD_MOSC_FREQUENCY > 48000000
#    error "BOARD_MOSC_FREQUENCY must be 8 MHz to 48 MHz"
#  endif
#  if !BOARD_MOSC_EXTERNAL_CLOCK && \
      (BOARD_MOSC_STABILIZATION_US < 12 || BOARD_MOSC_STABILIZATION_US > 31139)
#    error "BOARD_MOSC_STABILIZATION_US must be 12 us to 31139 us"
#  endif
#  define RA_MOSC_FREQUENCY       BOARD_MOSC_FREQUENCY
#endif

/* Divider validity.  The dividers are the values 1, 2, 3, 4, 6, 8, 12, 16,
 * 32 and 64 (SCKDIVCR/SCKDIVCR2 encode them non-linearly).
 */

#define RA_SCKDIV_VALID(d) \
  ((d) == 1 || (d) == 2 || (d) == 3 || (d) == 4 || (d) == 6 || \
   (d) == 8 || (d) == 12 || (d) == 16 || (d) == 32 || (d) == 64)
#define RA_SCKDIV_FAMILY_A(d) \
  ((d) == 2 || (d) == 4 || (d) == 8 || (d) == 16 || (d) == 32 || (d) == 64)
#define RA_SCKDIV_FAMILY_B(d)   ((d) == 3 || (d) == 6 || (d) == 12)

#define RA_PLL_DIV_P_VALID(d) \
  ((d) == 2 || (d) == 4 || (d) == 6 || (d) == 8 || (d) == 16)
#define RA_PLL_DIV_QR_VALID(d) \
  ((d) == 2 || (d) == 3 || (d) == 4 || (d) == 5 || (d) == 6 || \
   (d) == 8 || (d) == 9)

#define RA_SCICLK_DIV_VALID(d) \
  ((d) == 1 || (d) == 2 || (d) == 3 || (d) == 4 || (d) == 5 || \
   (d) == 6 || (d) == 8)

/* PLL1 and PLL2 */

#if RA_USE_PLL1

#  if BOARD_PLL1_SOURCE == 0
#    define RA_PLL1_SOURCE_FREQUENCY  RA_MOSC_FREQUENCY
#  elif BOARD_PLL1_SOURCE == R_SYSTEM_PLLCCR_PLSRCSEL
#    define RA_PLL1_SOURCE_FREQUENCY  RA_HOCO_FREQUENCY
#  else
#    error "BOARD_PLL1_SOURCE must be 0 (MOSC) or R_SYSTEM_PLLCCR_PLSRCSEL (HOCO)"
#  endif

#  if BOARD_PLL1_INPUT_DIVIDER < 1 || BOARD_PLL1_INPUT_DIVIDER > 4
#    error "BOARD_PLL1_INPUT_DIVIDER must be 1 to 4"
#  endif
#  if BOARD_PLL1_MULTIPLIER < 53 || BOARD_PLL1_MULTIPLIER > 180
#    error "BOARD_PLL1_MULTIPLIER must be 53 to 180"
#  endif

#  if BOARD_PLL1_MULTIPLIER_FRACTION == R_SYSTEM_PLLCCR_PLLMULNF_V0_33_1_3
#    define RA_PLL1_MULF6             2      /* 1/3 = 2/6 */
#  elif BOARD_PLL1_MULTIPLIER_FRACTION == R_SYSTEM_PLLCCR_PLLMULNF_V0_50_1_2
#    define RA_PLL1_MULF6             3      /* 1/2 = 3/6 */
#  elif BOARD_PLL1_MULTIPLIER_FRACTION == R_SYSTEM_PLLCCR_PLLMULNF_V0_66_2_3
#    define RA_PLL1_MULF6             4      /* 2/3 = 4/6 */
#  elif BOARD_PLL1_MULTIPLIER_FRACTION == \
        R_SYSTEM_PLLCCR_PLLMULNF_V0_00_VALUE_AFTER_RESET
#    define RA_PLL1_MULF6             0
#  else
#    error "BOARD_PLL1_MULTIPLIER_FRACTION must be one of the R_SYSTEM_PLLCCR_PLLMULNF_* codes"
#  endif

#  define RA_PLL1_INPUT_FREQUENCY     (RA_PLL1_SOURCE_FREQUENCY / BOARD_PLL1_INPUT_DIVIDER)
#  define RA_PLL1_VCO_FREQUENCY \
  ((RA_PLL1_INPUT_FREQUENCY * (6ULL * BOARD_PLL1_MULTIPLIER + RA_PLL1_MULF6)) / 6ULL)
#  define RA_PLL1P_FREQUENCY          (RA_PLL1_VCO_FREQUENCY / BOARD_PLL1_DIVIDER_P)
#  define RA_PLL1Q_FREQUENCY          (RA_PLL1_VCO_FREQUENCY / BOARD_PLL1_DIVIDER_Q)
#  define RA_PLL1R_FREQUENCY          (RA_PLL1_VCO_FREQUENCY / BOARD_PLL1_DIVIDER_R)

#  if RA_PLL1_SOURCE_FREQUENCY < 8000000 || RA_PLL1_SOURCE_FREQUENCY > 48000000
#    error "PLL1 source clock must be 8 MHz to 48 MHz"
#  endif
#  if RA_PLL1_INPUT_FREQUENCY < 6000000 || RA_PLL1_INPUT_FREQUENCY > 12000000
#    error "PLL1 input frequency after the input divider must be 6 MHz to 12 MHz"
#  endif
#  if RA_PLL1_VCO_FREQUENCY < 640000000ULL || RA_PLL1_VCO_FREQUENCY > 1440000000ULL
#    error "PLL1 VCO frequency must be 640 MHz to 1440 MHz"
#  endif
#  if !RA_PLL_DIV_P_VALID(BOARD_PLL1_DIVIDER_P)
#    error "PLL1 output P divider must be 2, 4, 6, 8 or 16"
#  endif
#  if !RA_PLL_DIV_QR_VALID(BOARD_PLL1_DIVIDER_Q) || !RA_PLL_DIV_QR_VALID(BOARD_PLL1_DIVIDER_R)
#    error "PLL1 output Q/R dividers must be 2, 3, 4, 5, 6, 8 or 9"
#  endif
#  if RA_PLL1P_FREQUENCY < 40000000 || RA_PLL1P_FREQUENCY > 480000000
#    error "PLL1P must be 40 MHz to 480 MHz"
#  endif
#  if RA_PLL1Q_FREQUENCY < 71000000 || RA_PLL1Q_FREQUENCY > 480000000
#    error "PLL1Q must be 71 MHz to 480 MHz"
#  endif
#  if RA_PLL1R_FREQUENCY < 71000000 || RA_PLL1R_FREQUENCY > 480000000
#    error "PLL1R must be 71 MHz to 480 MHz"
#  endif

#endif /* RA_USE_PLL1 */

#if RA_USE_PLL2

#  if BOARD_PLL2_SOURCE == 0
#    define RA_PLL2_SOURCE_FREQUENCY  RA_MOSC_FREQUENCY
#  elif BOARD_PLL2_SOURCE == R_SYSTEM_PLL2CCR_PL2SRCSEL
#    define RA_PLL2_SOURCE_FREQUENCY  RA_HOCO_FREQUENCY
#  else
#    error "BOARD_PLL2_SOURCE must be 0 (MOSC) or R_SYSTEM_PLL2CCR_PL2SRCSEL (HOCO)"
#  endif

#  if BOARD_PLL2_INPUT_DIVIDER < 1 || BOARD_PLL2_INPUT_DIVIDER > 4
#    error "BOARD_PLL2_INPUT_DIVIDER must be 1 to 4"
#  endif
#  if BOARD_PLL2_MULTIPLIER < 53 || BOARD_PLL2_MULTIPLIER > 180
#    error "BOARD_PLL2_MULTIPLIER must be 53 to 180"
#  endif

#  if BOARD_PLL2_MULTIPLIER_FRACTION == R_SYSTEM_PLL2CCR_PLL2MULNF_V0_33_1_3
#    define RA_PLL2_MULF6             2      /* 1/3 = 2/6 */
#  elif BOARD_PLL2_MULTIPLIER_FRACTION == R_SYSTEM_PLL2CCR_PLL2MULNF_V0_50_1_2
#    define RA_PLL2_MULF6             3      /* 1/2 = 3/6 */
#  elif BOARD_PLL2_MULTIPLIER_FRACTION == R_SYSTEM_PLL2CCR_PLL2MULNF_V0_66_2_3
#    define RA_PLL2_MULF6             4      /* 2/3 = 4/6 */
#  elif BOARD_PLL2_MULTIPLIER_FRACTION == \
        R_SYSTEM_PLL2CCR_PLL2MULNF_V0_00_VALUE_AFTER_RESET
#    define RA_PLL2_MULF6             0
#  else
#    error "BOARD_PLL2_MULTIPLIER_FRACTION must be one of the R_SYSTEM_PLL2CCR_PLL2MULNF_* codes"
#  endif

#  define RA_PLL2_INPUT_FREQUENCY     (RA_PLL2_SOURCE_FREQUENCY / BOARD_PLL2_INPUT_DIVIDER)
#  define RA_PLL2_VCO_FREQUENCY \
  ((RA_PLL2_INPUT_FREQUENCY * (6ULL * BOARD_PLL2_MULTIPLIER + RA_PLL2_MULF6)) / 6ULL)
#  define RA_PLL2P_FREQUENCY          (RA_PLL2_VCO_FREQUENCY / BOARD_PLL2_DIVIDER_P)
#  define RA_PLL2Q_FREQUENCY          (RA_PLL2_VCO_FREQUENCY / BOARD_PLL2_DIVIDER_Q)
#  define RA_PLL2R_FREQUENCY          (RA_PLL2_VCO_FREQUENCY / BOARD_PLL2_DIVIDER_R)

#  if RA_PLL2_SOURCE_FREQUENCY < 8000000 || RA_PLL2_SOURCE_FREQUENCY > 48000000
#    error "PLL2 source clock must be 8 MHz to 48 MHz"
#  endif
#  if RA_PLL2_INPUT_FREQUENCY < 6000000 || RA_PLL2_INPUT_FREQUENCY > 12000000
#    error "PLL2 input frequency after the input divider must be 6 MHz to 12 MHz"
#  endif
#  if RA_PLL2_VCO_FREQUENCY < 640000000ULL || RA_PLL2_VCO_FREQUENCY > 1440000000ULL
#    error "PLL2 VCO frequency must be 640 MHz to 1440 MHz"
#  endif
#  if !RA_PLL_DIV_P_VALID(BOARD_PLL2_DIVIDER_P)
#    error "PLL2 output P divider must be 2, 4, 6, 8 or 16"
#  endif
#  if !RA_PLL_DIV_QR_VALID(BOARD_PLL2_DIVIDER_Q) || !RA_PLL_DIV_QR_VALID(BOARD_PLL2_DIVIDER_R)
#    error "PLL2 output Q/R dividers must be 2, 3, 4, 5, 6, 8 or 9"
#  endif
#  if RA_PLL2P_FREQUENCY < 40000000 || RA_PLL2P_FREQUENCY > 480000000
#    error "PLL2P must be 40 MHz to 480 MHz"
#  endif
#  if RA_PLL2Q_FREQUENCY < 71000000 || RA_PLL2Q_FREQUENCY > 480000000
#    error "PLL2Q must be 71 MHz to 480 MHz"
#  endif
#  if RA_PLL2R_FREQUENCY < 71000000 || RA_PLL2R_FREQUENCY > 480000000
#    error "PLL2R must be 71 MHz to 480 MHz"
#  endif

#endif /* RA_USE_PLL2 */

/* System clock source */

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_HOCO
#  define RA_SYSCLK_FREQUENCY     RA_HOCO_FREQUENCY
#elif BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_MAIN_CLOCK_OSCILLATOR
#  define RA_SYSCLK_FREQUENCY     RA_MOSC_FREQUENCY
#elif BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_PLL
#  define RA_SYSCLK_FREQUENCY     RA_PLL1P_FREQUENCY
#else
#  define RA_SYSCLK_FREQUENCY     RA_MOCO_FREQUENCY
#endif

/* Clocks derived from the system clock */

#define RA_CPUCLK_FREQUENCY       (RA_SYSCLK_FREQUENCY / BOARD_CPUCLK_DIVIDER)
#define RA_ICLK_FREQUENCY         (RA_SYSCLK_FREQUENCY / BOARD_ICLK_DIVIDER)
#define RA_PCLKA_FREQUENCY        (RA_SYSCLK_FREQUENCY / BOARD_PCLKA_DIVIDER)
#define RA_PCLKB_FREQUENCY        (RA_SYSCLK_FREQUENCY / BOARD_PCLKB_DIVIDER)
#define RA_PCLKC_FREQUENCY        (RA_SYSCLK_FREQUENCY / BOARD_PCLKC_DIVIDER)
#define RA_PCLKD_FREQUENCY        (RA_SYSCLK_FREQUENCY / BOARD_PCLKD_DIVIDER)
#define RA_PCLKE_FREQUENCY        (RA_SYSCLK_FREQUENCY / BOARD_PCLKE_DIVIDER)
#define RA_FCLK_FREQUENCY         (RA_SYSCLK_FREQUENCY / BOARD_FCLK_DIVIDER)
#define RA_BCLK_FREQUENCY         (RA_SYSCLK_FREQUENCY / BOARD_BCLK_DIVIDER)

/* SCI clock (SCICLK): the clock of the SCI baud rate generator.  When no
 * SCI UART is enabled it is left at its reset value, the MOCO.
 */

#if RA_USE_SCICLK
#  if BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_HOCO
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_HOCO_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_MOSC
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_MOSC_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1P
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL1P_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1Q
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL1Q_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL1R
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL1R_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2P
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL2P_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2Q
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL2Q_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_PLL2R
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_PLL2R_FREQUENCY
#  elif BOARD_SCICLK_SOURCE == RA_SCICLK_SRC_MOCO
#    define RA_SCICLK_SOURCE_FREQUENCY  RA_MOCO_FREQUENCY
#  else
#    error "BOARD_SCICLK_SOURCE is not a valid clock source"
#  endif
#  define RA_SCICLK_FREQUENCY \
  (RA_SCICLK_SOURCE_FREQUENCY / BOARD_SCICLK_DIVIDER)
#else
#  define RA_SCICLK_FREQUENCY     RA_MOCO_FREQUENCY
#endif

/* Package dependent limit for switching the system clock to the PLL.  The
 * switch has to happen with CPUCLK at or below this frequency (RA8M1 User's
 * Manual figures 8.13-8.15); CPUCLK is then raised by dropping its divider.
 * The package is the one of the selected part number (PLBG0224 = 224-pin
 * BGA, PLQP0176/0144/0100 = LQFP).  The "AM" package does not appear in the
 * manual, so it is treated like the most restrictive one.
 */

#if defined(CONFIG_ARCH_CHIP_R7FA8M1AFECBD) || \
    defined(CONFIG_ARCH_CHIP_R7FA8M1AHECBD)
#  define RA_PLL_SWITCH_MAX_CPUCLK  240000000
#elif defined(CONFIG_ARCH_CHIP_R7FA8M1AFECFC) || \
      defined(CONFIG_ARCH_CHIP_R7FA8M1AHECFC) || \
      defined(CONFIG_ARCH_CHIP_R7FA8M1AFECFB) || \
      defined(CONFIG_ARCH_CHIP_R7FA8M1AHECFB)
#  define RA_PLL_SWITCH_MAX_CPUCLK  200000000
#else
#  define RA_PLL_SWITCH_MAX_CPUCLK  180000000
#endif

/* Settling time after the system clock is switched to the PLL: 150 us with
 * the DCDC converter enabled, 10 us with an external VDD.
 */

#if BOARD_OFS2_DCDC
#  define RA_CLOCK_SETTLE_US        150
#else
#  define RA_CLOCK_SETTLE_US        10
#endif

/* Flash wait states (FCACHE.FLWT) required for the ICLK frequency */

#if RA_ICLK_FREQUENCY <= 48000000
#  define RA_FLASH_WAIT_STATES      0
#elif RA_ICLK_FREQUENCY <= 96000000
#  define RA_FLASH_WAIT_STATES      1
#elif RA_ICLK_FREQUENCY <= 144000000
#  define RA_FLASH_WAIT_STATES      2
#elif RA_ICLK_FREQUENCY <= 192000000
#  define RA_FLASH_WAIT_STATES      3
#else
#  define RA_FLASH_WAIT_STATES      4
#endif

/* Divider families: 3, 6 and 12 can not be mixed with 2, 4, 8, 16, 32, 64 */

#define RA_ANY_DIV_A \
  (RA_SCKDIV_FAMILY_A(BOARD_CPUCLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_ICLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_PCLKA_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_PCLKB_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_PCLKC_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_PCLKD_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_PCLKE_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_FCLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_A(BOARD_BCLK_DIVIDER))
#define RA_ANY_DIV_B \
  (RA_SCKDIV_FAMILY_B(BOARD_CPUCLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_ICLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_PCLKA_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_PCLKB_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_PCLKC_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_PCLKD_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_PCLKE_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_FCLK_DIVIDER) || \
   RA_SCKDIV_FAMILY_B(BOARD_BCLK_DIVIDER))

/* CPUCLK divider used while the PLL is first selected as the system clock
 * when CPUCLK is above RA_PLL_SWITCH_MAX_CPUCLK: twice (or, in the 3/6/12
 * family, three times) slower than the final one.
 */

#if RA_ANY_DIV_B
#  define RA_CPUCLK_SWITCH_DIVIDER  (BOARD_CPUCLK_DIVIDER * 3)
#else
#  define RA_CPUCLK_SWITCH_DIVIDER  (BOARD_CPUCLK_DIVIDER * 2)
#endif

/* Checks against RA8M1 User's Manual chapter 8, Table 8.2 and its notes */

#if !RA_SCKDIV_VALID(BOARD_CPUCLK_DIVIDER) || \
    !RA_SCKDIV_VALID(BOARD_ICLK_DIVIDER)   || \
    !RA_SCKDIV_VALID(BOARD_PCLKA_DIVIDER)  || \
    !RA_SCKDIV_VALID(BOARD_PCLKB_DIVIDER)  || \
    !RA_SCKDIV_VALID(BOARD_PCLKC_DIVIDER)  || \
    !RA_SCKDIV_VALID(BOARD_PCLKD_DIVIDER)  || \
    !RA_SCKDIV_VALID(BOARD_PCLKE_DIVIDER)  || \
    !RA_SCKDIV_VALID(BOARD_FCLK_DIVIDER)   || \
    !RA_SCKDIV_VALID(BOARD_BCLK_DIVIDER)
#  error "System clock dividers must be 1, 2, 3, 4, 6, 8, 12, 16, 32 or 64"
#endif

#if RA_ANY_DIV_A && RA_ANY_DIV_B
#  error "Clock dividers of 3, 6 and 12 can not be mixed with 2, 4, 8, 16, 32 and 64"
#endif

#if RA_CPUCLK_FREQUENCY > 480000000
#  error "CPUCLK must be at most 480 MHz"
#endif
#if RA_ICLK_FREQUENCY > 240000000
#  error "ICLK must be at most 240 MHz"
#endif
#if RA_PCLKA_FREQUENCY > 120000000
#  error "PCLKA must be at most 120 MHz"
#endif
#if RA_PCLKB_FREQUENCY > 60000000
#  error "PCLKB must be at most 60 MHz"
#endif
#if RA_PCLKC_FREQUENCY > 60000000
#  error "PCLKC must be at most 60 MHz"
#endif
#if RA_PCLKD_FREQUENCY > 120000000
#  error "PCLKD must be at most 120 MHz"
#endif
#if RA_PCLKE_FREQUENCY > 240000000
#  error "PCLKE must be at most 240 MHz"
#endif
#if RA_FCLK_FREQUENCY > 60000000
#  error "FCLK must be at most 60 MHz"
#endif
#if RA_BCLK_FREQUENCY > 120000000
#  error "BCLK must be at most 120 MHz"
#endif

/* Frequency ordering and integer ratios: CPUCLK >= ICLK >= PCLKA >= PCLKB,
 * ICLK >= FCLK, ICLK >= BCLK, PCLKD >= PCLKA, CPUCLK:ICLK = N:1,
 * ICLK:{PCLKA, PCLKB, FCLK, BCLK} = N:1 and ICLK:{PCLKC, PCLKD, PCLKE} =
 * N:1 or 1:N.
 */

#if BOARD_ICLK_DIVIDER < BOARD_CPUCLK_DIVIDER || \
    (BOARD_ICLK_DIVIDER % BOARD_CPUCLK_DIVIDER) != 0
#  error "CPUCLK:ICLK must be an integer ratio with CPUCLK >= ICLK"
#endif
#if BOARD_PCLKA_DIVIDER < BOARD_ICLK_DIVIDER || \
    (BOARD_PCLKA_DIVIDER % BOARD_ICLK_DIVIDER) != 0
#  error "ICLK:PCLKA must be an integer ratio with ICLK >= PCLKA"
#endif
#if BOARD_PCLKB_DIVIDER < BOARD_PCLKA_DIVIDER
#  error "PCLKA must be at least PCLKB"
#endif
#if BOARD_PCLKB_DIVIDER < BOARD_ICLK_DIVIDER || \
    (BOARD_PCLKB_DIVIDER % BOARD_ICLK_DIVIDER) != 0
#  error "ICLK:PCLKB must be an integer ratio with ICLK >= PCLKB"
#endif
#if BOARD_FCLK_DIVIDER < BOARD_ICLK_DIVIDER || \
    (BOARD_FCLK_DIVIDER % BOARD_ICLK_DIVIDER) != 0
#  error "ICLK:FCLK must be an integer ratio with ICLK >= FCLK"
#endif
#if BOARD_BCLK_DIVIDER < BOARD_ICLK_DIVIDER || \
    (BOARD_BCLK_DIVIDER % BOARD_ICLK_DIVIDER) != 0
#  error "ICLK:BCLK must be an integer ratio with ICLK >= BCLK"
#endif
#if BOARD_PCLKD_DIVIDER > BOARD_PCLKA_DIVIDER
#  error "PCLKD must be at least PCLKA"
#endif
#if ((BOARD_PCLKC_DIVIDER % BOARD_ICLK_DIVIDER) != 0 && \
     (BOARD_ICLK_DIVIDER % BOARD_PCLKC_DIVIDER) != 0) || \
    ((BOARD_PCLKD_DIVIDER % BOARD_ICLK_DIVIDER) != 0 && \
     (BOARD_ICLK_DIVIDER % BOARD_PCLKD_DIVIDER) != 0) || \
    ((BOARD_PCLKE_DIVIDER % BOARD_ICLK_DIVIDER) != 0 && \
     (BOARD_ICLK_DIVIDER % BOARD_PCLKE_DIVIDER) != 0)
#  error "ICLK:PCLKC/PCLKD/PCLKE must be an integer ratio (N:1 or 1:N)"
#endif

/* When the HOCO feeds PLL1 the CPU clock is limited to 360 MHz */

#if RA_USE_PLL1 && BOARD_PLL1_SOURCE == R_SYSTEM_PLLCCR_PLSRCSEL && \
    RA_CPUCLK_FREQUENCY > 360000000
#  error "CPUCLK above 360 MHz needs the main oscillator as the PLL1 source"
#endif

/* Switching to the PLL: CPUCLK has to be at or below the package limit at
 * the moment of the switch, and raised afterwards with the divider.
 */

#if BOARD_SYSCLK_SOURCE == R_SYSTEM_SCKSCR_CKSEL_PLL && \
    RA_CPUCLK_FREQUENCY > RA_PLL_SWITCH_MAX_CPUCLK
#  if BOARD_CPUCLK_DIVIDER != 1
#    error "CPUCLK above the PLL switch limit needs a CPUCLK divider of 1"
#  endif
#  if (RA_SYSCLK_FREQUENCY / RA_CPUCLK_SWITCH_DIVIDER) > \
      RA_PLL_SWITCH_MAX_CPUCLK
#    error "CPUCLK is more than 2x (3x) the limit for switching to the PLL"
#  endif
#endif

#if RA_USE_SCICLK
#  if !RA_SCICLK_DIV_VALID(BOARD_SCICLK_DIVIDER)
#    error "SCICLK divider must be 1, 2, 3, 4, 5, 6 or 8"
#  endif
#  if RA_SCICLK_FREQUENCY > 120000000
#    error "SCICLK must be at most 120 MHz"
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: ra_clockconfig
 *
 * Description:
 *   Called to initialize the RA8M1.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes starting the oscillators
 *   and PLLs, selecting the system clock source, setting the clock
 *   dividers and configuring the peripheral dedicated clocks, all from the
 *   BOARD_* clock options in board.h.
 *
 ****************************************************************************/

void ra_clockconfig(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8M1_RA_CLOCKCONFIG_H */
