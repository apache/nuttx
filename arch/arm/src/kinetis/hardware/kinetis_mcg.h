/************************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_mcg.h
 *
 *   Copyright (C) 2011, 2016-2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCG_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_MCG_C1_OFFSET       0x0000 /* MCG Control 1 Register */
#define KINETIS_MCG_C2_OFFSET       0x0001 /* MCG Control 2 Register */
#define KINETIS_MCG_C3_OFFSET       0x0002 /* MCG Control 3 Register */
#define KINETIS_MCG_C4_OFFSET       0x0003 /* MCG Control 4 Register */
#define KINETIS_MCG_C5_OFFSET       0x0004 /* MCG Control 5 Register */
#define KINETIS_MCG_C6_OFFSET       0x0005 /* MCG Control 6 Register */
#if defined(KINETIS_MCG_HAS_S)
#  define KINETIS_MCG_S_OFFSET      0x0006 /* MCG Status Register */
#endif
#if defined(KINETIS_MCG_HAS_ATC) && !defined(KINETIS_MCG_HAS_SC)
#  define KINETIS_MCG_ATC_OFFSET    0x0008 /* MCG Auto Trim Control Register */
#endif
#if defined(KINETIS_MCG_HAS_SC)
#  define KINETIS_MCG_SC_OFFSET     0x0008 /* MMCG Status and Control Register */
#endif
#if defined(KINETIS_MCG_HAS_ATCVH)
#  define KINETIS_MCG_ATCVH_OFFSET  0x000a /* MCG Auto Trim Compare Value High Register */
#endif
#if defined(KINETIS_MCG_HAS_ATCVL)
#  define KINETIS_MCG_ATCVL_OFFSET  0x000b /* MCG Auto Trim Compare Value Low Register */
#endif
#if defined(KINETIS_MCG_HAS_C7)
#  define KINETIS_MCG_C7_OFFSET     0x000c /* MCG Control 7 Register */
#endif
#if defined(KINETIS_MCG_HAS_C8)
#  define KINETIS_MCG_C8_OFFSET     0x000d /* MCG Control 8 Register */
#endif
#if defined(KINETIS_MCG_HAS_C9)
#  define KINETIS_MCG_C9_OFFSET     0x000e /* MCG Control 9 Register */
#endif
#if defined(KINETIS_MCG_HAS_C10)
#  define KINETIS_MCG_C10_OFFSET    0x000f /* MCG Control 10 Register */
#endif
#if defined(KINETIS_MCG_HAS_C11)
#  define KINETIS_MCG_C11_OFFSET    0x0010 /* MCG Control 11 Register */
#endif
#if defined(KINETIS_MCG_HAS_C12)
#  define KINETIS_MCG_C12_OFFSET    0x0011 /* MCG Control 12 Register */
#endif
#if defined(KINETIS_MCG_HAS_S2)
#  define KINETIS_MCG_S2_OFFSET     0x0012 /* MCG Control S2 Register */
#endif
#if defined(KINETIS_MCG_HAS_T3)
#  define KINETIS_MCG_T3_OFFSET     0x0013 /* MCG Control T3 Register */
#endif

/* Register Addresses ***************************************************************/

#define KINETIS_MCG_C1            (KINETIS_MCG_BASE+KINETIS_MCG_C1_OFFSET)
#define KINETIS_MCG_C2            (KINETIS_MCG_BASE+KINETIS_MCG_C2_OFFSET)
#define KINETIS_MCG_C3            (KINETIS_MCG_BASE+KINETIS_MCG_C3_OFFSET)
#define KINETIS_MCG_C4            (KINETIS_MCG_BASE+KINETIS_MCG_C4_OFFSET)
#define KINETIS_MCG_C5            (KINETIS_MCG_BASE+KINETIS_MCG_C5_OFFSET)
#define KINETIS_MCG_C6            (KINETIS_MCG_BASE+KINETIS_MCG_C6_OFFSET)
#define KINETIS_MCG_S             (KINETIS_MCG_BASE+KINETIS_MCG_S_OFFSET)
#if defined(KINETIS_MCG_HAS_ATC) && !defined(KINETIS_MCG_HAS_SC)
#  define KINETIS_MCG_ATC         (KINETIS_MCG_BASE+KINETIS_MCG_ATC_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_SC)
#  define KINETIS_MCG_SC          (KINETIS_MCG_BASE+KINETIS_MCG_SC_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_ATCVH)
#  define KINETIS_MCG_ATCVH       (KINETIS_MCG_BASE+KINETIS_MCG_ATCVH_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_ATCVL)
#  define KINETIS_MCG_ATCVL       (KINETIS_MCG_BASE+KINETIS_MCG_ATCVL_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C7)
#  define KINETIS_MCG_C7          (KINETIS_MCG_BASE+KINETIS_MCG_C7_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C8)
#  define KINETIS_MCG_C8          (KINETIS_MCG_BASE+KINETIS_MCG_C8_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C9)
#  define KINETIS_MCG_C9          (KINETIS_MCG_BASE+KINETIS_MCG_C9_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C10)
#  define KINETIS_MCG_C10         (KINETIS_MCG_BASE+KINETIS_MCG_C10_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C11)
#  define KINETIS_MCG_C11         (KINETIS_MCG_BASE+KINETIS_MCG_C11_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_C12)
#  define KINETIS_MCG_C12         (KINETIS_MCG_BASE+KINETIS_MCG_C12_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_S2)
#  define KINETIS_MCG_S2          (KINETIS_MCG_BASE+KINETIS_MCG_S2_OFFSET)
#endif
#if defined(KINETIS_MCG_HAS_T3)
#  define KINETIS_MCG_T3          (KINETIS_MCG_BASE+KINETIS_MCG_T3_OFFSET)
#endif

/* Register Bit Definitions *********************************************************/

/* MCG Control 1 Register (8-bit) */

#define MCG_C1_IREFSTEN             (1 << 0)  /* Bit 0:  Internal Reference Stop Enable */
#define MCG_C1_IRCLKEN              (1 << 1)  /* Bit 1:  Internal Reference Clock Enable */
#if defined(KINETIS_MCG_HAS_C1_IREFS)
#  define MCG_C1_IREFS              (1 << 2)  /* Bit 2:  Internal Reference Select */
#endif
#if defined(KINETIS_MCG_HAS_C1_FRDIV)
#  define MCG_C1_FRDIV_SHIFT        (3)       /* Bits 3-5: FLL External Reference Divider */
#  define MCG_C1_FRDIV_MASK         (7 << MCG_C1_FRDIV_SHIFT)
#    define MCG_C1_FRDIV_R0DIV1     (0 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=1 */
#    define MCG_C1_FRDIV_R0DIV2     (1 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=2 */
#    define MCG_C1_FRDIV_R0DIV4     (2 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=4 */
#    define MCG_C1_FRDIV_R0DIV8     (3 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=8 */
#    define MCG_C1_FRDIV_R0DIV16    (4 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=16 */
#    define MCG_C1_FRDIV_R0DIV32    (5 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=32 */
#    define MCG_C1_FRDIV_R0DIV64    (6 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=64 */
#    define MCG_C1_FRDIV_R0DIV128   (7 << MCG_C1_FRDIV_SHIFT) /* RANGE==0 divider=128 */
#    define MCG_C1_FRDIV_DIV32      (0 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=32 */
#    define MCG_C1_FRDIV_DIV64      (1 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=64 */
#    define MCG_C1_FRDIV_DIV128     (2 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=128 */
#    define MCG_C1_FRDIV_DIV256     (3 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=256 */
#    define MCG_C1_FRDIV_DIV512     (4 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=512 */
#    define MCG_C1_FRDIV_DIV1024    (5 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=1024 */
#    if KINETIS_MCG_C1_FRDIV_MAX > 5
#      define MCG_C1_FRDIV_DIV1280  (6 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=1280 */
#    endif
#    if KINETIS_MCG_C1_FRDIV_MAX > 6
#      define MCG_C1_FRDIV_DIV1536  (7 << MCG_C1_FRDIV_SHIFT) /* RANGE!=0 divider=1536 */
#    endif
#endif /* defined (KINETIS_MCG_HAS  _C1_FRDIV) */
#define MCG_C1_CLKS_SHIFT           (6)       /* Bits 6-7: Clock Source Select */
#define MCG_C1_CLKS_MASK            (3 << MCG_C1_CLKS_SHIFT)
#  define MCG_C1_CLKS_PLL           (0 << MCG_C1_CLKS_SHIFT) /* FLL or PLL output */
#  define MCG_C1_CLKS_INTREF        (1 << MCG_C1_CLKS_SHIFT) /* Internal reference clock */
#  define MCG_C1_CLKS_EXTREF        (2 << MCG_C1_CLKS_SHIFT) /* External reference clock */

/* MCG Control 2 Register */

#define MCG_C2_IRCS                 (1 << 0)  /* Bit 0:  Internal Reference Clock Select */
#define MCG_C2_LP                   (1 << 1)  /* Bit 1:  Low Power Select */
#define MCG_C2_EREFS                (1 << 2)  /* Bit 2:  External Reference Select */
#if defined(KINETIS_MCG_HAS_C2_HGO)
#  define MCG_C2_HGO                (1 << 3)  /* Bit 3:  High Gain Oscillator Select */
#endif
#if defined(KINETIS_MCG_HAS_C2_RANGE)
#  define MCG_C2_RANGE_SHIFT        (4)       /* Bits 4-5: Frequency Range Select */
#  define MCG_C2_RANGE_MASK         (3 << MCG_C2_RANGE_SHIFT)
#    define MCG_C2_RANGE_LOW        (0 << MCG_C2_RANGE_SHIFT) /* Oscillator of 32 kHz to 40 kHz  */
#    define MCG_C2_RANGE_HIGH       (1 << MCG_C2_RANGE_SHIFT) /* Oscillator of 1 MHz to 8 MHz */
#    define MCG_C2_RANGE_VHIGH      (2 << MCG_C2_RANGE_SHIFT) /* Oscillator of 8 MHz to 32 MHz */
#endif
#if defined(KINETIS_MCG_HAS_C2_FCFTRIM)
#  define MCG_C2_FCFTRIM            (1 << 6)  /* Bit 6:  Fast Internal Reference Clock Fine Trim */
#endif
#if defined(KINETIS_MCG_HAS_C2_LOCRE0)
#  define MCG_C2_LOCRE0             (1 << 7)  /* Bit 7: Loss of Clock Reset Enable */
#endif

/* MCG Control 3 Register (8-bit Slow Internal Reference Clock Trim Setting) */

/* MCG Control 4 Register (8-bit) */

#define MCG_C4_SCFTRIM              (1 << 0)  /* Bit 0:  Slow Internal Reference Clock Fine Trim */
#define MCG_C4_FCTRIM_SHIFT         (1)       /* Bits 1-4: Fast Internal Reference Clock Trim Setting */
#define MCG_C4_FCTRIM_MASK          (15 << MCG_C4_FCTRIM_SHIFT)
#define MCG_C4_DRST_DRS_SHIFT       (5)       /* Bits 5-6: DCO Range Select */
#define MCG_C4_DRST_DRS_MASK        (3 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_LOW       (00 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_MID       (01 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_MIDHIGH   (10 << MCG_C4_DRST_DRS_SHIFT)
#  define MCG_C4_DRST_DRS_HIGH      (11 << MCG_C4_DRST_DRS_SHIFT)
#define MCG_C4_DMX32                (1 << 7)  /* Bit 7:  DCO Maximum Frequency with 32.768 kHz Reference */

/* MCG Control 5 Register */

#if defined(KINETIS_MCG_HAS_C5_PRDIV)
#  define MCG_C5_PRDIV_SHIFT        (0)       /* Bits 0-[2|4]: PLL External Reference Divider */
#  define MCG_C5_PRDIV_MASK         (KINETIS_MCG_C5_PRDIV_MASK << MCG_C5_PRDIV_SHIFT)
#  define MCG_C5_PRDIV(n)           ((((n) & KINETIS_MCG_C5_PRDIV_MASK)-KINETIS_MCG_C5_PRDIV_BASE) << MCG_C5_PRDIV_SHIFT) /* n=KINETIS_MCG_C5_PRDIV_BASE..KINETIS_MCG_C5_PRDIV_MAX */
#endif
#define MCG_C5_PLLSTEN              (1 << 5)  /* Bit 5:  PLL Stop Enable */
#define MCG_C5_PLLCLKEN             (1 << 6)  /* Bit 6:  PLL Clock Enable */
#if defined(KINETIS_MCG_HAS_C5_PLLREFSEL0)
#  define MCG_C5_PLLREFSEL0         (1 << 7)  /* Bit 7:  PLL0 External Reference Select */
#endif

/* MCG Control 6 Register */

#if defined(KINETIS_MCG_HAS_C6_VDIV)
#  define MCG_C6_VDIV_SHIFT         (0)       /* Bits 0-4: VCO Divider */
#  define MCG_C6_VDIV_MASK          (31 << MCG_C6_VDIV_SHIFT)
#  define MCG_C6_VDIV(n)            (((n)-KINETIS_MCG_C6_VDIV_BASE) << MCG_C6_VDIV_SHIFT) /* n=KINETIS_MCG_C6_VDIV_BASE..KINETIS_MCG_C6_VDIV_MAX */
#endif
#if defined(KINETIS_MCG_HAS_C6_CME)
#  define MCG_C6_CME                (1 << 5)  /* Bit 5:  Clock Monitor Enable */
#endif
#if defined(KINETIS_MCG_HAS_C6_PLLS)
#  define MCG_C6_PLLS               (1 << 6)  /* Bit 6:  PLL Select */
#endif
#if defined(KINETIS_MCG_HAS_C6_LOLIE0)
#  define MCG_C6__LOLIE0            (1 << 7)  /* Bit 7:  Loss of Lock Interrupt Enable */
#endif

/* MCG Status Register */

#if defined(KINETIS_MCG_HAS_S)
#  define MCG_S_IRCST               (1 << 0)  /* Bit 0:  Internal Reference Clock Status */
#  define MCG_S_OSCINIT             (1 << 1)  /* Bit 1:  OSC Initialization */
#  define MCG_S_CLKST_SHIFT         (2)       /* Bits 2-3: Clock Mode Status */
#  define MCG_S_CLKST_MASK          (3 << MCG_S_CLKST_SHIFT)
#    define MCG_S_CLKST_FLL         (0 << MCG_S_CLKST_SHIFT) /* Output of the FLL */
#    define MCG_S_CLKST_INTREF      (1 << MCG_S_CLKST_SHIFT) /* Internal reference clock */
#    define MCG_S_CLKST_EXTREF      (2 << MCG_S_CLKST_SHIFT) /* External reference clock */
#    define MCG_S_CLKST_PLL         (3 << MCG_S_CLKST_SHIFT) /* Output of the PLL */
#  define MCG_S_IREFST              (1 << 4)  /* Bit 4:  Internal Reference Status */
#  if defined(KINETIS_MCG_HAS_S_PLLST)
#    define MCG_S_PLLST             (1 << 5)  /* Bit 5:  PLL Select Status */
#  endif
#  if defined(KINETIS_MCG_HAS_S_LOCK0)
#    define MCG_S_LOCK0             (1 << 6)  /* Bit 6:  Lock Status */
#  endif
#  if defined(KINETIS_MCG_HAS_S_LOLS)
#    define MCG_S_LOLS              (1 << 7)  /* Bit 7:  Loss of Lock Status */
#  endif
#endif

/* MCG Auto Trim Control Register */

#if defined(KINETIS_MCG_HAS_ATC) && !defined(KINETIS_MCG_HAS_SC)
                                              /* Bits 0-4: Reserved */
#  define MCG_ATC_ATMF              (1 << 5)  /* Bit 5:  Automatic Trim machine Fail Flag */
#  define MCG_ATC_ATMS              (1 << 6)  /* Bit 6:  Automatic Trim Machine Select */
#  define MCG_ATC_ATME              (1 << 7)  /* Bit 7:  Automatic Trim Machine Enable */
#endif

/* MCG Auto Trim Compare Value High/Low Registers (8-bit compare value) */

#if defined(KINETIS_MCG_HAS_SC)
#  define MCG_SC_LOCS0              (1 << 0)  /* Bit 0:  OSC0 Loss of Clock Status */
#  define MCG_SC_FCRDIV_SHIFT       (1)       /* Bits 1-3: Fast Clock Internal Reference Divider */
#  define MCG_SC_FCRDIV_MASK        (7 << MCG_SC_FLTPRSRV_SHIFT)
#  define MCG_SC_FCRDIV(n)          (((n)) << MCG_SC_FLTPRSRV_SHIFT)  /* n=0..7 */
#  define MCG_SC_FCRDIV_1           (0 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 1 */
#  define MCG_SC_FCRDIV_2           (1 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 2 */
#  define MCG_SC_FCRDIV_4           (2 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 4 */
#  define MCG_SC_FCRDIV_8           (3 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 8 */
#  define MCG_SC_FCRDIV_16          (4 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 16 */
#  define MCG_SC_FCRDIV_32          (5 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 32 */
#  define MCG_SC_FCRDIV_64          (6 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 64 */
#  define MCG_SC_FCRDIV_128         (7 << MCG_SC_FLTPRSRV_SHIFT)  /* Divide Factor is 128 */
#  define MCG_SC_FLTPRSRV           (1 << 4)  /* Bit 4:  FLL Filter Preserve Enable */
#  if defined(KINETIS_MCG_HAS_SC_ATMF)
#    define MCG_SC_ATMF             (1 << 5)  /* Bit 5:  Automatic Trim machine Fail Flag */
#  endif
#  if defined(KINETIS_MCG_HAS_SC_ATMS)
#    define MCG_SC_ATMS             (1 << 6)  /* Bit 6:  Automatic Trim Machine Select */
#  endif
#  if defined(KINETIS_MCG_HAS_SC_ATME)
#    define MCG_ASC_ATME            (1 << 7)  /* Bit 7:  Automatic Trim Machine Enable */
#  endif
#endif

/* MCG Control 7 Register */

#if defined(KINETIS_MCG_HAS_C7)
#  if defined(KINETIS_MCG_HAS_C7_OSCSEL)
#    define MCG_C7_OSCSEL_SHIFT    (0)        /* Bits 0-[1]: MCG OSC Clock Select */
#    define MCG_C7_OSCSEL_MASK     (KINETIS_MCG_C7_OSCSEL_MASK << MCG_C7_OSCSEL_SHIFT)
#    define MCG_C7_OSCSEL_OSCCLK   (0 << MCG_C7_OSCSEL_SHIFT)  /* Selects Oscillator (OSCCLK) */
#    define MCG_C7_OSCSEL_32KHZ    (1 << MCG_C7_OSCSEL_SHIFT)  /* Selects 32 kHz RTC Oscillator */
#    if (KINETIS_MCG_C7_OSCSEL_MASK & 2) != 0
#      define MCG_C7_OSCSEL_OSCCLK1  (2 << MCG_C7_OSCSEL_SHIFT)  /* Selects Oscillator (OSCCLK1). */
#    endif
#  endif
#endif

/* MCG Control 8 Register */

#if defined(KINETIS_MCG_HAS_C8)
#  if defined(KINETIS_MCG_HAS_C8_LOCS1)
#    define MCG_C8_LOCS1           (1 << 0)   /* Bit 0: RTC Loss of Clock Status */
#  endif
                                              /* Bits 1-4: Reserved */
#  if defined(KINETIS_MCG_HAS_C8_CME1)
#    define MCG_C8_CME1             (1 << 5)  /* Bit 5:  Clock Monitor Enable1 */
#  endif
#  if defined(KINETIS_MCG_HAS_C8_LOLRE)
#    define MCG_C8_LOLRE            (1 << 6)  /* Bit 6:  PLL Loss of Lock Reset Enable */
#  endif
#  if defined(KINETIS_MCG_HAS_C8_LOCRE1)
#    define MCG_C8_LOCRE1           (1 << 7)  /* Bit 7:  Loss of Clock Reset Enable */
#  endif
#endif

/* MCG Control 9 Register */

#if defined(KINETIS_MCG_HAS_C9)
#  if defined(KINETIS_MCG_HAS_C9_EXT_PLL_LOCS)
#    define MCG_C9_EXT_PLL_LOCS     (1 << 0)  /* Bit 0: External PLL Loss of Clock Status */
#  endif
                                              /* Bits 1-3: Reserved */
#  if defined(KINETIS_MCG_HAS_C9_PLL_LOCRE)
#    define MCG_C9_EXT_PLL_LOCRE    (1 << 4)  /* Bit 4: MCG External PLL Loss of Clock Reset Enable */
#  endif
#  if defined(KINETIS_MCG_HAS_C9_PLL_CME)
#    define MCG_C9_EXT_C9_PLL_CME   (1 << 5)  /* Bit 5: MCG External PLL Clock Monitor Enable */
#  endif
                                              /* Bits 6-7: Reserved */
#endif

/* MCG Control 10 Register */

#if defined(KINETIS_MCG_HAS_C10)
                                              /* Bits 0-[1]: Reserved */
#  if defined(KINETIS_MCG_HAS_C10_LOCS1)
#    define MCG_C10_LOCS1_SHIFT     (1 << 1)  /* Bit 1: RTC Loss of Clock Status */
#  endif
#  define MCG_C10_EREFS1            (1 << 2)  /* Bit 2:  External Reference Select */
#  define MCG_C10_HGO1              (1 << 3)  /* Bit 3:  High Gain Oscillator1 Select */
#  define MCG_C10_RANGE1_SHIFT      (4)       /* Bits 4-5: Frequency Range1 Select */
#  define MCG_C10_RANGE_MASK        (3 << MCG_C10_RANGE_SHIFT)
#    define MCG_C10_RANGE_LOW       (0 << MCG_C10_RANGE_SHIFT) /* Oscillator of 32 kHz to 40 kHz  */
#    define MCG_C10_RANGE_HIGH      (1 << MCG_C10_RANGE_SHIFT) /* Oscillator of 1 MHz to 8 MHz */
#    define MCG_C10_RANGE_VHIGH     (2 << MCG_C10_RANGE_SHIFT) /* Oscillator of 8 MHz to 32 MHz */
                                              /* Bit 6: Reserved */
#  define MCG_C10_LOCRE2            (1 << 7)  /* Bit 7: OSC1 Loss of Clock Reset Enable */
#endif

/* MCG Control 11 Register */

#if defined(KINETIS_MCG_HAS_C11)
#  if defined(KINETIS_MCG_HAS_C11_PLL1OSC1)
#    define MCG_C11_PRDIV1_SHIFT    (0)       /* Bits 0-2: PLL1 External Reference Divider */
#    define MCG_C11_PRDIV1_MASK     (7 << MCG_C11_PRDIV1_SHIFT)
#    define MCG_C11_PRDIV1(n)       (((n)-1) << MCG_C11_PRDIV_SHIFT) /* 1..8 */
#  endif
                                              /* Bit 3: Reserved */
#  if defined(KINETIS_MCG_HAS_C11_PLLCS)
#    define MCG_C11_PLLCS           (1 << 4)  /* Bit 4:  PLL Clock Select */
#  endif
#  if defined(KINETIS_MCG_HAS_C11_PLL1OSC1)
#    define MCG_C11_PLLSTEN1        (1 << 5)  /* Bit 5:  PLL1 Stop Enable */
#    define MCG_C11_PLLCLKEN1       (1 << 6)  /* Bit 6:  PLL1 Clock Enable */
#  endif
#  if defined(KINETIS_MCG_HAS_C11_PLLREFSEL1)
#    define MCG_C11_PLLREFSEL1      (1 << 7)  /* Bit 7:  PLL1 External Reference Select */
#  endif
#endif

/* MCG Control 12 Register */

#if defined(KINETIS_MCG_HAS_C12)
#  define MCG_C12_VDIV1_SHIFT       (0)       /* Bits 0-4: VCO Divider */
#  define MCG_C12_VDIV1_MASK        (31 << MCG_C12_VDIV1_SHIFT)
#  define MCG_C12_VDIV(n)           (((n)-16) << MCG_C12_VDIV1_SHIFT) /* n=16..47 */
#  define MCG_C12_CME2              (1 << 5)  /* Bit 5:  Clock Monitor Enable2 */
                                              /* Bit 6: Reserved */
#  define MCG_C12_LOLIE1            (1 << 7)  /* Bit 7:  PLL1 Loss of Lock Interrupt Enable */
#endif

/* MCG Control S2 Register */

#if defined(KINETIS_MCG_HAS_S2)
#  if defined(KINETIS_MCG_HAS_S2_PLL1OSC1)
#    define MCG_S2_LOCS2_SHIFT      (1 << 0)  /* Bit 0: OSC1 Loss of Clock Status */
#    define MCG_S2_OSCINIT1         (1 << 1)  /* Bit 1: OSC1 Initialization */
#  endif
                                              /* Bits 2-3: Reserved */
#  if defined(KINETIS_MCG_HAS_S2_PLLCST)
#    define MCG_S2_PLLCST           (1 << 4)  /* Bit 4: PLL Clock Select Status */
#  endif
                                              /* Bit 5: Reserved */
#  if defined(KINETIS_MCG_HAS_S2_PLL1OSC1)
#    define MCG_S2_LOCK1            (1 << 6)  /* Bit 6: Lock1 Status */
#    define MCG_S2_LOLS1            (1 << 7)  /* Bit 7: Loss of Lock1 Status */
#  endif
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MCG_H */
