/****************************************************************************
 * arch/arm/src/ra8m1/ra_option_setting.c
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

/* Option-setting memory images: OFS0, OFS1, OFS2 and OFS1_SEL.
 *
 * The MCU samples these flash words at reset to decide the watchdog start-up
 * state, the voltage detection level, the HOCO, the debug behaviour, ECC and
 * the DCDC converter.  Each one is a const object in a section of its own;
 * the linker script (ek_ra8m1.ld) places the section at the fixed hardware
 * address and the flash tool programs it together with the image.  The
 * values come from the BOARD_OFS* options in the board's board.h.
 *
 * This is a flat (no TrustZone) image, so only the secure-region registers
 * are used.  OFS1 is the register at 0x0300_A200 (OFS1_SEC in the manual);
 * OFS1_SEL is programmed as 0 so that every OFS1 field is taken from it.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "hardware/ra8m1_option_setting.h"
#include "ra_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Option values from board.h -> OFS0/OFS1/OFS2 fields.  Every value the
 * board gives is checked, so an invalid one fails to build.
 *
 * With auto start off, every field of that watchdog reads as the erased
 * (all ones) value: register start mode, and the remaining fields are not
 * used, so their BOARD_* options need not be defined.
 */

#ifndef BOARD_OFS0_IWDT_AUTOSTART
#  error "board.h must define BOARD_OFS0_IWDT_AUTOSTART"
#endif
#ifndef BOARD_OFS0_WDT_AUTOSTART
#  error "board.h must define BOARD_OFS0_WDT_AUTOSTART"
#endif
#if !defined(BOARD_OFS1_VDSEL_MV) || !defined(BOARD_OFS1_LVD0_RESET) || \
    !defined(BOARD_OFS1_PVD0_LOWPOWER) || !defined(BOARD_OFS1_HOCO_START) || \
    !defined(BOARD_OFS1_SWDBG) || !defined(BOARD_OFS1_INITECC)
#  error "board.h must define all the BOARD_OFS1_* options"
#endif
#ifndef BOARD_OFS2_DCDC
#  error "board.h must define BOARD_OFS2_DCDC"
#endif
#ifndef BOARD_HOCO_FREQUENCY
#  error "board.h must define BOARD_HOCO_FREQUENCY"
#endif

/* OFS0: IWDT */

#define RA_OFS0_IWDT_ALL_ONES \
  (R_OFS0_IWDTSTRT | \
   (R_OFS0_IWDTTOPS_MASK << R_OFS0_IWDTTOPS_SHIFT) | \
   (R_OFS0_IWDTCKS_MASK << R_OFS0_IWDTCKS_SHIFT) | \
   (R_OFS0_IWDTRPES_MASK << R_OFS0_IWDTRPES_SHIFT) | \
   (R_OFS0_IWDTRPSS_MASK << R_OFS0_IWDTRPSS_SHIFT) | \
   R_OFS0_IWDTRSTIRQS | R_OFS0_IWDTSTPCTL)

#if BOARD_OFS0_IWDT_AUTOSTART
#  if !defined(BOARD_OFS0_IWDT_TIMEOUT) || !defined(BOARD_OFS0_IWDT_CLKDIV) || \
      !defined(BOARD_OFS0_IWDT_WINDOW_END) || \
      !defined(BOARD_OFS0_IWDT_WINDOW_START) || \
      !defined(BOARD_OFS0_IWDT_RESET) || \
      !defined(BOARD_OFS0_IWDT_STOP_IN_SLEEP)
#    error "IWDT auto start: board.h must define all the BOARD_OFS0_IWDT_* options"
#  endif

#  if BOARD_OFS0_IWDT_TIMEOUT == 128
#    define RA_OFS0_IWDT_TOPS_VAL R_OFS0_IWDTTOPS_128
#  elif BOARD_OFS0_IWDT_TIMEOUT == 512
#    define RA_OFS0_IWDT_TOPS_VAL R_OFS0_IWDTTOPS_512
#  elif BOARD_OFS0_IWDT_TIMEOUT == 1024
#    define RA_OFS0_IWDT_TOPS_VAL R_OFS0_IWDTTOPS_1024
#  elif BOARD_OFS0_IWDT_TIMEOUT == 2048
#    define RA_OFS0_IWDT_TOPS_VAL R_OFS0_IWDTTOPS_2048
#  else
#    error "BOARD_OFS0_IWDT_TIMEOUT must be 128, 512, 1024 or 2048"
#  endif

#  if BOARD_OFS0_IWDT_CLKDIV == 1
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV1
#  elif BOARD_OFS0_IWDT_CLKDIV == 16
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV16
#  elif BOARD_OFS0_IWDT_CLKDIV == 32
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV32
#  elif BOARD_OFS0_IWDT_CLKDIV == 64
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV64
#  elif BOARD_OFS0_IWDT_CLKDIV == 128
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV128
#  elif BOARD_OFS0_IWDT_CLKDIV == 256
#    define RA_OFS0_IWDT_CKS_VAL R_OFS0_IWDTCKS_DIV256
#  else
#    error "BOARD_OFS0_IWDT_CLKDIV must be 1, 16, 32, 64, 128 or 256"
#  endif

#  if BOARD_OFS0_IWDT_WINDOW_END == 75
#    define RA_OFS0_IWDT_RPES_VAL R_OFS0_IWDTRPES_75
#  elif BOARD_OFS0_IWDT_WINDOW_END == 50
#    define RA_OFS0_IWDT_RPES_VAL R_OFS0_IWDTRPES_50
#  elif BOARD_OFS0_IWDT_WINDOW_END == 25
#    define RA_OFS0_IWDT_RPES_VAL R_OFS0_IWDTRPES_25
#  elif BOARD_OFS0_IWDT_WINDOW_END == 0
#    define RA_OFS0_IWDT_RPES_VAL R_OFS0_IWDTRPES_0
#  else
#    error "BOARD_OFS0_IWDT_WINDOW_END must be 75, 50, 25 or 0"
#  endif

#  if BOARD_OFS0_IWDT_WINDOW_START == 25
#    define RA_OFS0_IWDT_RPSS_VAL R_OFS0_IWDTRPSS_25
#  elif BOARD_OFS0_IWDT_WINDOW_START == 50
#    define RA_OFS0_IWDT_RPSS_VAL R_OFS0_IWDTRPSS_50
#  elif BOARD_OFS0_IWDT_WINDOW_START == 75
#    define RA_OFS0_IWDT_RPSS_VAL R_OFS0_IWDTRPSS_75
#  elif BOARD_OFS0_IWDT_WINDOW_START == 100
#    define RA_OFS0_IWDT_RPSS_VAL R_OFS0_IWDTRPSS_100
#  else
#    error "BOARD_OFS0_IWDT_WINDOW_START must be 25, 50, 75 or 100"
#  endif

#  if BOARD_OFS0_IWDT_RESET
#    define RA_OFS0_IWDT_RESET_BIT R_OFS0_IWDTRSTIRQS
#  else
#    define RA_OFS0_IWDT_RESET_BIT 0
#  endif
#  if BOARD_OFS0_IWDT_STOP_IN_SLEEP
#    define RA_OFS0_IWDT_STOP_BIT  R_OFS0_IWDTSTPCTL
#  else
#    define RA_OFS0_IWDT_STOP_BIT  0
#  endif
#  define RA_OFS0_IWDT \
  (RA_OFS0_IWDT_TOPS_VAL | RA_OFS0_IWDT_CKS_VAL | \
   RA_OFS0_IWDT_RPES_VAL | RA_OFS0_IWDT_RPSS_VAL | \
   RA_OFS0_IWDT_RESET_BIT | RA_OFS0_IWDT_STOP_BIT)
#else
#  define RA_OFS0_IWDT RA_OFS0_IWDT_ALL_ONES
#endif

/* OFS0: WDT */

#define RA_OFS0_WDT_ALL_ONES \
  (R_OFS0_WDT0STRT | \
   (R_OFS0_WDT0TOPS_MASK << R_OFS0_WDT0TOPS_SHIFT) | \
   (R_OFS0_WDT0CKS_MASK << R_OFS0_WDT0CKS_SHIFT) | \
   (R_OFS0_WDT0RPES_MASK << R_OFS0_WDT0RPES_SHIFT) | \
   (R_OFS0_WDT0RPSS_MASK << R_OFS0_WDT0RPSS_SHIFT) | \
   R_OFS0_WDT0RSTIRQS | R_OFS0_WDT0STPCTL)

#if BOARD_OFS0_WDT_AUTOSTART
#  if !defined(BOARD_OFS0_WDT_TIMEOUT) || !defined(BOARD_OFS0_WDT_CLKDIV) || \
      !defined(BOARD_OFS0_WDT_WINDOW_END) || \
      !defined(BOARD_OFS0_WDT_WINDOW_START) || \
      !defined(BOARD_OFS0_WDT_RESET) || \
      !defined(BOARD_OFS0_WDT_STOP_IN_SLEEP)
#    error "WDT auto start: board.h must define all the BOARD_OFS0_WDT_* options"
#  endif

#  if BOARD_OFS0_WDT_TIMEOUT == 1024
#    define RA_OFS0_WDT_TOPS_VAL R_OFS0_WDT0TOPS_1024
#  elif BOARD_OFS0_WDT_TIMEOUT == 4096
#    define RA_OFS0_WDT_TOPS_VAL R_OFS0_WDT0TOPS_4096
#  elif BOARD_OFS0_WDT_TIMEOUT == 8192
#    define RA_OFS0_WDT_TOPS_VAL R_OFS0_WDT0TOPS_8192
#  elif BOARD_OFS0_WDT_TIMEOUT == 16384
#    define RA_OFS0_WDT_TOPS_VAL R_OFS0_WDT0TOPS_16384
#  else
#    error "BOARD_OFS0_WDT_TIMEOUT must be 1024, 4096, 8192 or 16384"
#  endif

#  if BOARD_OFS0_WDT_CLKDIV == 4
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV4
#  elif BOARD_OFS0_WDT_CLKDIV == 64
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV64
#  elif BOARD_OFS0_WDT_CLKDIV == 128
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV128
#  elif BOARD_OFS0_WDT_CLKDIV == 512
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV512
#  elif BOARD_OFS0_WDT_CLKDIV == 2048
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV2048
#  elif BOARD_OFS0_WDT_CLKDIV == 8192
#    define RA_OFS0_WDT_CKS_VAL R_OFS0_WDT0CKS_DIV8192
#  else
#    error "BOARD_OFS0_WDT_CLKDIV must be 4, 64, 128, 512, 2048 or 8192"
#  endif

#  if BOARD_OFS0_WDT_WINDOW_END == 75
#    define RA_OFS0_WDT_RPES_VAL R_OFS0_WDT0RPES_75
#  elif BOARD_OFS0_WDT_WINDOW_END == 50
#    define RA_OFS0_WDT_RPES_VAL R_OFS0_WDT0RPES_50
#  elif BOARD_OFS0_WDT_WINDOW_END == 25
#    define RA_OFS0_WDT_RPES_VAL R_OFS0_WDT0RPES_25
#  elif BOARD_OFS0_WDT_WINDOW_END == 0
#    define RA_OFS0_WDT_RPES_VAL R_OFS0_WDT0RPES_0
#  else
#    error "BOARD_OFS0_WDT_WINDOW_END must be 75, 50, 25 or 0"
#  endif

#  if BOARD_OFS0_WDT_WINDOW_START == 25
#    define RA_OFS0_WDT_RPSS_VAL R_OFS0_WDT0RPSS_25
#  elif BOARD_OFS0_WDT_WINDOW_START == 50
#    define RA_OFS0_WDT_RPSS_VAL R_OFS0_WDT0RPSS_50
#  elif BOARD_OFS0_WDT_WINDOW_START == 75
#    define RA_OFS0_WDT_RPSS_VAL R_OFS0_WDT0RPSS_75
#  elif BOARD_OFS0_WDT_WINDOW_START == 100
#    define RA_OFS0_WDT_RPSS_VAL R_OFS0_WDT0RPSS_100
#  else
#    error "BOARD_OFS0_WDT_WINDOW_START must be 25, 50, 75 or 100"
#  endif

#  if BOARD_OFS0_WDT_RESET
#    define RA_OFS0_WDT_RESET_BIT R_OFS0_WDT0RSTIRQS
#  else
#    define RA_OFS0_WDT_RESET_BIT 0
#  endif
#  if BOARD_OFS0_WDT_STOP_IN_SLEEP
#    define RA_OFS0_WDT_STOP_BIT  R_OFS0_WDT0STPCTL
#  else
#    define RA_OFS0_WDT_STOP_BIT  0
#  endif
#  define RA_OFS0_WDT \
  (RA_OFS0_WDT_TOPS_VAL | RA_OFS0_WDT_CKS_VAL | \
   RA_OFS0_WDT_RPES_VAL | RA_OFS0_WDT_RPSS_VAL | \
   RA_OFS0_WDT_RESET_BIT | RA_OFS0_WDT_STOP_BIT)
#else
#  define RA_OFS0_WDT RA_OFS0_WDT_ALL_ONES
#endif

#define RA_OFS0 (R_OFS0_RESERVED | RA_OFS0_IWDT | RA_OFS0_WDT)

/* OFS1.  PVDAS, PVDLPSEL, HOCOEN and SWDBG are active low: the bit is set
 * when the feature is off.
 */

#if BOARD_OFS1_VDSEL_MV == 2850
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_2_85V
#elif BOARD_OFS1_VDSEL_MV == 2580
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_2_58V
#elif BOARD_OFS1_VDSEL_MV == 2150
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_2_15V
#elif BOARD_OFS1_VDSEL_MV == 2000
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_2_00V
#elif BOARD_OFS1_VDSEL_MV == 1900
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_1_90V
#elif BOARD_OFS1_VDSEL_MV == 1800
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_1_80V
#elif BOARD_OFS1_VDSEL_MV == 1700
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_1_70V
#elif BOARD_OFS1_VDSEL_MV == 1600
#  define RA_OFS1_VDSEL_VAL R_OFS1_VDSEL_1_60V
#else
#  error "BOARD_OFS1_VDSEL_MV must be 2850, 2580, 2150, 2000, 1900, 1800, 1700 or 1600"
#endif

/* The HOCO frequency after reset is the same BOARD_HOCO_FREQUENCY that
 * ra_clockconfig.h uses as the HOCO frequency.
 */

#if BOARD_HOCO_FREQUENCY == 16000000
#  define RA_OFS1_HOCOFRQ_VAL R_OFS1_HOCOFRQ0_16MHZ
#elif BOARD_HOCO_FREQUENCY == 18000000
#  define RA_OFS1_HOCOFRQ_VAL R_OFS1_HOCOFRQ0_18MHZ
#elif BOARD_HOCO_FREQUENCY == 20000000
#  define RA_OFS1_HOCOFRQ_VAL R_OFS1_HOCOFRQ0_20MHZ
#elif BOARD_HOCO_FREQUENCY == 32000000
#  define RA_OFS1_HOCOFRQ_VAL R_OFS1_HOCOFRQ0_32MHZ
#elif BOARD_HOCO_FREQUENCY == 48000000
#  define RA_OFS1_HOCOFRQ_VAL R_OFS1_HOCOFRQ0_48MHZ
#else
#  error "BOARD_HOCO_FREQUENCY must be 16, 18, 20, 32 or 48 MHz"
#endif

#if BOARD_OFS1_LVD0_RESET
#  define RA_OFS1_PVDAS_BIT      0
#else
#  define RA_OFS1_PVDAS_BIT      R_OFS1_PVDAS
#endif

#if BOARD_OFS1_PVD0_LOWPOWER
#  define RA_OFS1_PVDLPSEL_BIT   0
#else
#  define RA_OFS1_PVDLPSEL_BIT   R_OFS1_PVDLPSEL
#endif

#if BOARD_OFS1_HOCO_START
#  define RA_OFS1_HOCOEN_BIT     0
#else
#  define RA_OFS1_HOCOEN_BIT     R_OFS1_HOCOEN
#endif

#if BOARD_OFS1_SWDBG
#  define RA_OFS1_SWDBG_BIT      0
#else
#  define RA_OFS1_SWDBG_BIT      R_OFS1_SWDBG
#endif

#if BOARD_OFS1_INITECC
#  define RA_OFS1_INITECCEN_BIT  R_OFS1_INITECCEN
#else
#  define RA_OFS1_INITECCEN_BIT  0
#endif

#define RA_OFS1 \
  (R_OFS1_RESERVED | RA_OFS1_VDSEL_VAL | RA_OFS1_PVDAS_BIT | \
   RA_OFS1_PVDLPSEL_BIT | RA_OFS1_HOCOEN_BIT | RA_OFS1_HOCOFRQ_VAL | \
   RA_OFS1_SWDBG_BIT | RA_OFS1_INITECCEN_BIT)

/* OFS2 */

#if BOARD_OFS2_DCDC
#  define RA_OFS2 (R_OFS2_RESERVED | R_OFS2_DCDCEN)
#else
#  define RA_OFS2 R_OFS2_RESERVED
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* "used" and the KEEP() in the linker script keep the sections themselves.
 * The objects have external linkage and ra_start.c references them, because
 * nothing else refers to this file and the linker would otherwise not pull
 * it out of the library at all.
 */

const uint32_t g_ra_option_setting_ofs0
  __attribute__((section(".option_setting_ofs0"), used)) = RA_OFS0;

const uint32_t g_ra_option_setting_ofs2
  __attribute__((section(".option_setting_ofs2"), used)) = RA_OFS2;

const uint32_t g_ra_option_setting_ofs1
  __attribute__((section(".option_setting_ofs1"), used)) = RA_OFS1;

const uint32_t g_ra_option_setting_ofs1_sel
  __attribute__((section(".option_setting_ofs1_sel"), used)) =
  R_OFS1_SEL_ALL_SECURE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
