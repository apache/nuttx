/************************************************************************************
 * arch/arm/include/kinetis/kinetis_mcg.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_MCG_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_MCG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Note: It is envisioned that in the long term as a chip is added. The author of
 * the new chip definitions will either find the exact configuration in an existing
 * chip define and add the new chip to it Or add the MCG feature configuration
 * #defines to the chip ifdef list below. In either case the author should mark
 * it as "Verified to Document Number:" taken from the reference manual.
 *
 * To maintain backward compatibility to the version of NuttX prior to
 * 2/5/2017, the catch all KINETIS_MCG_VERSION_UKN configuration is assigned
 * to all the chips that did not have any conditional compilation based on
 * NEW_MCG or KINETIS_K64. This is  a "No worse" than the original code solution.
 * N.B. Each original chip "if"definitions have been left intact so that the
 * complete legacy definitions prior to 2/5/2017 may be filled in completely when
 * vetted.
 */

/* MCG Configuration Parameters
 *
 * KINETIS_MCG_PLL_REF_MIN           - OSCCLK/PLL_R minimum
 * KINETIS_MCG_PLL_REF_MAX           - OSCCLK/PLL_R maximum
 * KINETIS_MCG_PLL_INTERNAL_DIVBY    - The PLL clock is divided by n before VCO divider
 * KINETIS_MCG_HAS_PLL_EXTRA_DIVBY   - Is PLL clock divided by n before MCG PLL/FLL
 *                                     clock selection in the SIM module
 * KINETIS_MCG_FFCLK_DIVBY           - MCGFFCLK divided by n
 * KINETIS_MCG_HAS_IRC_48M           -  Has 48MHz internal oscillator
 * KINETIS_MCG_HAS_LOW_FREQ_IRC      - Has LTRIMRNG, LFRIM, LSTRIM and bit MC[LIRC_DIV2]
 * KINETIS_MCG_HAS_HIGH_FREQ_IRC     - Has HCTRIM, HTTRIM, HFTRIM and bit MC[HIRCEN]
 * KINETIS_MCG_HAS_PLL_INTERNAL_MODE - Has PEI mode or PBI mode
 * KINETIS_MCG_HAS_RESET_IS_BLPI     - Has Reset clock mode is BLPI
 *
 * MCG Register Configuration
 *
 * KINETIS_MCG_HAS_C1                - SoC has C1 Register
 * KINETIS_MCG_HAS_C1_IREFS          - SoC has C1[IREFS]
 * KINETIS_MCG_HAS_C1_FRDIV          - SoC has C1[FRDIV]
 * KINETIS_MCG_C1_FRDIV_MAX          - C1[FRDIV] maximum value 5=1024, 6=1280 7=1536
 * KINETIS_MCG_HAS_C2                - SoC has C2 Register
 * KINETIS_MCG_HAS_C2_HGO            - SoC has C2[HGO]
 * KINETIS_MCG_HAS_C2_RANGE          - SoC has C2[RANG]
 * KINETIS_MCG_HAS_C2_FCFTRIM        - SoC has C2[FCFTRIM]
 * KINETIS_MCG_HAS_C2_LOCRE0         - SoC has C2[LOCRE0]
 * KINETIS_MCG_HAS_C3                - SoC has C3 Register
 * KINETIS_MCG_HAS_C4                - SoC has C4 Register
 * KINETIS_MCG_HAS_C5                - SoC has C5 Register
 * KINETIS_MCG_HAS_C5_PRDIV          - SoC has C5[PRDIV]
 * KINETIS_MCG_C5_PRDIV_BASE         - PRDIV base value corresponding to 0 in C5[PRDIV]
 * KINETIS_MCG_C5_PRDIV_MAX          - The Maximum value of C5[PRVDIV])
 * KINETIS_MCG_C5_PRDIV_BITS         - Has n bits of phase-locked loop (PLL) PRDIV (register C5[PRDIV]
 * KINETIS_MCG_HAS_C5_PLLREFSEL0     - SoC has C5[PLLREFSEL0]
 * KINETIS_MCG_HAS_C6                - SoC has C6 Register
 * KINETIS_MCG_HAS_C6_VDIV           - SoC has C6[VDIV]
 * KINETIS_MCG_C6_VDIV_BASE          - VDIV base value corresponding to 0 in C6[VDIV]
 * KINETIS_MCG_C6_VDIV_MAX           - The Maximum value of C6[VDIV]
 * KINETIS_MCG_HAS_C6_CME            - SoC has C6[CME]
 * KINETIS_MCG_HAS_C6_PLLS           - SoC has C6[PLLS]
 * KINETIS_MCG_HAS_C6_LOLIE0         - SoC has C6[LOLIE0]
 * KINETIS_MCG_HAS_S                 - SoC has S Register
 * KINETIS_MCG_HAS_S_PLLST           - SoC has S[PLLST]
 * KINETIS_MCG_HAS_S_LOCK0           - SoC has S[LOCK0]
 * KINETIS_MCG_HAS_S_LOLS            - SoC has S[LOLS]
 * KINETIS_MCG_HAS_ATC               - SoC has ATC Register
 * KINETIS_MCG_HAS_ATCVH             - SoC has ATCVH Register
 * KINETIS_MCG_HAS_ATCVL             - SoC has ATCVL Register
 * KINETIS_MCG_HAS_SC                - SoC has SC Register
 * KINETIS_MCG_HAS_SC_ATMS           - SoC has SC[ATMS]
 * KINETIS_MCG_HAS_SC_ATMF           - SoC has SC[ATMF]
 * KINETIS_MCG_HAS_SC_ATME           - SoC has SC[ATME]
 * KINETIS_MCG_HAS_C7                - SoC has C7 Register
 * KINETIS_MCG_HAS_C7_OSCSEL         - SoC has C7[OSCSEL]
 * KINETIS_MCG_C7_OSCSEL_BITS        - C7[OSCSEL] is n bits wide
 * KINETIS_MCG_HAS_C8                - SoC has C8 Register
 * KINETIS_MCG_HAS_C8_LOCS1          - SoC has C8[LOCS1]
 * KINETIS_MCG_HAS_C8_CME1           - SoC has C8[CME1]
 * KINETIS_MCG_HAS_C8_LOLRE          - SoC has C8[LOLRE]
 * KINETIS_MCG_HAS_C8_LOCRE1         - SoC has C8[LOCRE1]
 * KINETIS_MCG_HAS_C9                - SoC has C9 Register
 * KINETIS_MCG_HAS_C9_EXT_PLL_LOCS   - SoC has C9_EXT_PLL[LOCS]
 * KINETIS_MCG_HAS_C9_PLL_LOCRE      - SoC has C9_PLL[LOCRE]
 * KINETIS_MCG_HAS_C9_PLL_CME        - SoC has C9_PLL[CME]
 * KINETIS_MCG_HAS_C10               - SoC has C10 Register
 * KINETIS_MCG_HAS_C10_LOCS1         - SoC has C10[LOCS1]
 * KINETIS_MCG_HAS_C11               - SoC has C11 Register
 * KINETIS_MCG_HAS_C11_PLL1OSC1      - SoC has C1[PRDIV1], C11[PLLSTEN1], C11[PLLCLKEN1], C11[PLLREFSEL1],
 * KINETIS_MCG_HAS_C11_PLLCS         - SoC has C11[PLLCS]
 * KINETIS_MCG_HAS_C11_PLLREFSEL1    - SoC has C11[PLLREFSEL1]
 * KINETIS_MCG_HAS_C12               - SoC has C12 Register
 * KINETIS_MCG_HAS_S2                - SoC has S2 Register
 * KINETIS_MCG_HAS_S2_PLL1OSC1       - SoC has S2[LOCS2], S2[OSCINIT1], S2[LOCK1], S2[LOLS1]
 * KINETIS_MCG_HAS_S2_PLLCST         - SoC has S2[PLLCST]
 */

/* Describe the version of the MCG
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_MCG_VERSION_UKN   -1 /* What was in nuttx prior to 2/5/2017 */
#define KINETIS_MCG_VERSION_01    1  /* The addition of MK60FN1M0VLQ12 Previously known as KINETIS_NEW_MCG
                                      * Verified Document Number: K60P144M150SF3RM Rev. 3, November 2014 */
#define KINETIS_MCG_VERSION_04    4  /* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */
#define KINETIS_MCG_VERSION_06    6  /* Verified to Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015 */

/* MK20DX/DN---VLH5
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DN32VLH5  50 MHz 64  LQFP     32 KB 32 KB   —       8 KB 40
 *  MK20DX32VLH5  50 MHz 64  LQFP     64 KB 32 KB   2 KB    8 KB 40
 *  MK20DN64VLH5  50 MHz 64  LQFP     64 KB 64 KB   —      16 KB 40
 *  MK20DX64VLH5  50 MHz 64  LQFP     96 KB 64 KB   2 KB   16 KB 40
 *  MK20DN128VLH5 50 MHz 64  LQFP    128 KB 128 KB  —      16 KB 40
 *  MK20DX128VLH5 50 MHz 64  LQFP    160 KB 128 KB  2 KB   16 KB 40
 */

#if defined(CONFIG_ARCH_CHIP_MK20DN32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX32VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX64VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DN128VLH5) || \
    defined(CONFIG_ARCH_CHIP_MK20DX128VLH5)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

/* MK20DX---VLH7
 *
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  PART NUMBER   CPU    PIN PACKAGE TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                FREQ   CNT         FLASH  FLASH
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 *  MK20DX64VLH7  72 MHz 64  LQFP     96 KB  64 KB  2 KB   16 KB 40
 *  MK20DX128VLH7 72 MHz 64  LQFP    160 KB 128 KB  2 KB   32 KB 40
 *  MK20DX256VLH7 72 MHz 64  LQFP    288 KB 256 KB  2 KB   64 KB 40
 *  ------------- ------ --- ------- ------ ------- ------ ----- ----
 */

#elif defined(CONFIG_ARCH_CHIP_MK20DX64VLH7) || defined(CONFIG_ARCH_CHIP_MK20DX128VLH7) || \
      defined(CONFIG_ARCH_CHIP_MK20DX256VLH7)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12)

/* Verified to Document Number: K60P144M100SF2V2RM Rev. 2 Jun 2012 */

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_01

/* MCG Configuration Parameters */

#  define KINETIS_MCG_PLL_REF_MIN            8000000   /* OSCCLK/PLL_R minimum */
#  define KINETIS_MCG_PLL_REF_MAX            16000000  /* OSCCLK/PLL_R maximum */
#  define KINETIS_MCG_PLL_INTERNAL_DIVBY     2         /* The PLL clock is divided by 2 before VCO divider */
#  define KINETIS_MCG_HAS_PLL_EXTRA_DIVBY    2         /* Is PLL clock divided by 2 before MCG PLL/FLL clock selection in the SIM module */
#  define KINETIS_MCG_FFCLK_DIVBY            2         /* MCGFFCLK divided by 2 */
#  undef  KINETIS_MCG_HAS_IRC_48M                      /* Has no 48MHz internal oscillator */
#  undef  KINETIS_MCG_HAS_LOW_FREQ_IRC                 /* Has LTRIMRNG, LFRIM, LSTRIM and bit MC[LIRC_DIV2] */
#  undef  KINETIS_MCG_HAS_HIGH_FREQ_IRC                /* Has HCTRIM, HTTRIM, HFTRIM and bit MC[HIRCEN] */
#  undef  KINETIS_MCG_HAS_PLL_INTERNAL_MODE            /* Has PEI mode or PBI mode */
#  undef  KINETIS_MCG_HAS_RESET_IS_BLPI                /* Has Reset clock mode is BLPI */

/* MCG Register Configuration */

#  define KINETIS_MCG_HAS_C1                 1         /* SoC has C1 Register */
#  define KINETIS_MCG_HAS_C1_IREFS           1         /* SoC has C1[IREFS] */
#  define KINETIS_MCG_HAS_C1_FRDIV           1         /* SoC has C1[FRDIV] */
#  define KINETIS_MCG_C1_FRDIV_MAX           7         /* C1[FRDIV] maximum value 5=1024, 6=1280 7=1536 */
#  define KINETIS_MCG_HAS_C2                 1         /* SoC has C2 Register */
#  define KINETIS_MCG_HAS_C2_HGO             1         /* SoC has C2[HGO] */
#  define KINETIS_MCG_HAS_C2_RANGE           1         /* SoC has C2[RANGE] */
#  undef  KINETIS_MCG_HAS_C2_FCFTRIM                   /* SoC has C2[FCFTRIM] */
#  define KINETIS_MCG_HAS_C2_LOCRE0          1         /* SoC has C2[LOCRE0] */
#  define KINETIS_MCG_HAS_C3                 1         /* SoC has C3 Register */
#  define KINETIS_MCG_HAS_C4                 1         /* SoC has C4 Register */
#  define KINETIS_MCG_HAS_C5                 1         /* SoC has C5 Register */
#  define KINETIS_MCG_HAS_C5_PRDIV           1         /* SoC has C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_BASE          1         /* PRDIV base value corresponding to 0 in C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_MAX           8         /* The Maximum value of C5[PRVDIV]) */
#  define KINETIS_MCG_C5_PRDIV_BITS          3         /* Has 3 bits of phase-locked loop (PLL) PRDIV (register C5[PRDIV] */
#  define KINETIS_MCG_HAS_C5_PLLREFSEL0      1         /* SoC has C5[PLLREFSEL0] */
#  define KINETIS_MCG_HAS_C6                 1         /* SoC has C6 Register */
#  define KINETIS_MCG_HAS_C6_VDIV            1         /* SoC has C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_BASE           16        /* VDIV base value corresponding to 0 in C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_MAX            47        /* The Maximum value of C6[VDIV] */
#  define KINETIS_MCG_HAS_C6_CME             1         /* SoC has C6[CME] */
#  define KINETIS_MCG_HAS_C6_PLLS            1         /* SoC has C6[PLLS] */
#  define KINETIS_MCG_HAS_C6_LOLIE0          1         /* SoC has C6[LOLIE0] */
#  define KINETIS_MCG_HAS_S                  1         /* SoC has S Register */
#  define KINETIS_MCG_HAS_S_PLLST            1         /* SoC has S[PLLST] */
#  define KINETIS_MCG_HAS_S_LOCK0            1         /* SoC has S[LOCK0] */
#  define KINETIS_MCG_HAS_S_LOLS             1         /* SoC has S[LOLS] */
#  undef  KINETIS_MCG_HAS_ATC                          /* SoC has ATC Register */
#  define KINETIS_MCG_HAS_ATCVH              1         /* SoC has ATCVH Register */
#  define KINETIS_MCG_HAS_ATCVL              1         /* SoC has ATCVL Register */
#  define KINETIS_MCG_HAS_SC                 1         /* SoC has SC Register */
#  define KINETIS_MCG_HAS_SC_ATMS            1         /* SoC has SC[ATMS] */
#  define KINETIS_MCG_HAS_SC_ATMF            1         /* SoC has SC[ATMF] */
#  define KINETIS_MCG_HAS_SC_ATME            1         /* SoC has SC[ATME] */
#  define KINETIS_MCG_HAS_C7                 1         /* SoC has C7 Register */
#  define KINETIS_MCG_HAS_C7_OSCSEL          1         /* SoC has C7[OSCSEL] */
#  define KINETIS_MCG_C7_OSCSEL_BITS         1         /* C7[OSCSEL] is n bits wide */
#  define KINETIS_MCG_HAS_C8                 1         /* SoC has C8 Register */
#  define KINETIS_MCG_HAS_C8_LOCS1           1         /* SoC has C8[LOCS1] */
#  define KINETIS_MCG_HAS_C8_CME1            1         /* SoC has C8[CME1] */
#  undef  KINETIS_MCG_HAS_C8_LOLRE                     /* SoC has C8[LOLRE] */
#  define KINETIS_MCG_HAS_C8_LOCRE1          1         /* SoC has C8[LOCRE1] */
#  undef  KINETIS_MCG_HAS_C9                 1         /* SoC has C9 Register */
#  undef  KINETIS_MCG_HAS_C9_EXT_PLL_LOCS    1         /* SoC has C9_EXT_PLL[LOCS] */
#  undef  KINETIS_MCG_HAS_C9_PLL_LOCRE       1         /* SoC has C9_PLL[LOCRE] */
#  undef  KINETIS_MCG_HAS_C9_PLL_CME         1         /* SoC has C9_PLL[CME] */
#  define KINETIS_MCG_HAS_C10                          /* SoC has C10 Register */
#  undef  KINETIS_MCG_HAS_C10_LOCS1                    /* SoC has C10[LOCS1] */
#  define KINETIS_MCG_HAS_C11                          /* SoC has C11 Register */
#  define KINETIS_MCG_HAS_C11_PLL1OSC1       1         /* SoC has C1[PRDIV1], C11[PLLSTEN1], C11[PLLCLKEN1], C11[PLLREFSEL1] */
#  define KINETIS_MCG_HAS_C11_PLLCS                    /* SoC has C11[PLLCS] */
#  define KINETIS_MCG_HAS_C11_PLLREFSEL1     1         /* SoC has C11[PLLREFSEL1] */
#  define KINETIS_MCG_HAS_C12                          /* SoC has C12 Register */
#  define KINETIS_MCG_HAS_S2                           /* SoC has S2 Register */
#  define KINETIS_MCG_HAS_S2_PLL1OSC1        1         /* SoC has S2[LOCS2], S2[OSCINIT1], S2[LOCK1], S2[LOLS1] */
#  define KINETIS_MCG_HAS_S2_PLLCST                    /* SoC has S2[PLLCST] */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

/* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_04

/* MCG Configuration Parameters */

#  define KINETIS_MCG_PLL_REF_MIN            2000000   /* OSCCLK/PLL_R minimum */
#  define KINETIS_MCG_PLL_REF_MAX            4000000   /* OSCCLK/PLL_R maximum */
#  define KINETIS_MCG_PLL_INTERNAL_DIVBY     1         /* The PLL clock is divided by 1 before VCO divider */
#  define KINETIS_MCG_HAS_PLL_EXTRA_DIVBY    1         /* Is PLL clock divided by 1 before MCG PLL/FLL clock selection in the SIM module */
#  define KINETIS_MCG_FFCLK_DIVBY            2         /* MCGFFCLK divided by 2 */
#  define KINETIS_MCG_HAS_IRC_48M            1         /* Has 48MHz internal oscillator */
#  undef  KINETIS_MCG_HAS_LOW_FREQ_IRC                 /* Has LTRIMRNG, LFRIM, LSTRIM and bit MC[LIRC_DIV2] */
#  undef  KINETIS_MCG_HAS_HIGH_FREQ_IRC                /* Has HCTRIM, HTTRIM, HFTRIM and bit MC[HIRCEN] */
#  undef  KINETIS_MCG_HAS_PLL_INTERNAL_MODE            /* Has PEI mode or PBI mode */
#  undef  KINETIS_MCG_HAS_RESET_IS_BLPI                /* Has Reset clock mode is BLPI */

/* MCG Register Configuration */

#  define KINETIS_MCG_HAS_C1                 1         /* SoC has C1 Register */
#  define KINETIS_MCG_HAS_C1_IREFS           1         /* SoC has C1[IREFS] */
#  define KINETIS_MCG_HAS_C1_FRDIV           1         /* SoC has C1[FRDIV] */
#  define KINETIS_MCG_C1_FRDIV_MAX           7         /* C1[FRDIV] maximum value 5=1024, 6=1280 7=1536 */
#  define KINETIS_MCG_HAS_C2                 1         /* SoC has C2 Register */
#  define KINETIS_MCG_HAS_C2_HGO             1         /* SoC has C2[HGO] */
#  define KINETIS_MCG_HAS_C2_RANGE           1         /* SoC has C2[RANGE] */
#  define KINETIS_MCG_HAS_C2_FCFTRIM         1         /* SoC has C2[FCFTRIM] */
#  define KINETIS_MCG_HAS_C2_LOCRE0          1         /* SoC has C2[LOCRE0] */
#  define KINETIS_MCG_HAS_C3                 1         /* SoC has C3 Register */
#  define KINETIS_MCG_HAS_C4                 1         /* SoC has C4 Register */
#  define KINETIS_MCG_HAS_C5                 1         /* SoC has C5 Register */
#  define KINETIS_MCG_HAS_C5_PRDIV           1         /* SoC has C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_BASE          1         /* PRDIV base value corresponding to 0 in C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_MAX           25        /* The Maximum value of C5[PRVDIV]) */
#  define KINETIS_MCG_C5_PRDIV_BITS          5         /* Has 5 bits of phase-locked loop (PLL) PRDIV (register C5[PRDIV] */
#  undef  KINETIS_MCG_HAS_C5_PLLREFSEL0                /* SoC has C5[PLLREFSEL0] */
#  define KINETIS_MCG_HAS_C6                 1         /* SoC has C6 Register */
#  define KINETIS_MCG_HAS_C6_VDIV            1         /* SoC has C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_BASE           24        /* VDIV base value corresponding to 0 in C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_MAX            55        /* The Maximum value of C6[VDIV] */
#  define KINETIS_MCG_HAS_C6_CME             1         /* SoC has C6[CME] */
#  define KINETIS_MCG_HAS_C6_PLLS            1         /* SoC has C6[PLLS] */
#  define KINETIS_MCG_HAS_C6_LOLIE0          1         /* SoC has C6[LOLIE0] */
#  define KINETIS_MCG_HAS_S                  1         /* SoC has S Register */
#  define KINETIS_MCG_HAS_S_PLLST            1         /* SoC has S[PLLST] */
#  define KINETIS_MCG_HAS_S_LOCK0            1         /* SoC has S[LOCK0] */
#  define KINETIS_MCG_HAS_S_LOLS             1         /* SoC has S[LOLS] */
#  undef  KINETIS_MCG_HAS_ATC                          /* SoC has ATC Register */
#  define KINETIS_MCG_HAS_ATCVH              1         /* SoC has ATCVH Register */
#  define KINETIS_MCG_HAS_ATCVL              1         /* SoC has ATCVL Register */
#  define KINETIS_MCG_HAS_SC                 1         /* SoC has SC Register */
#  define KINETIS_MCG_HAS_SC_ATMS            1         /* SoC has SC[ATMS] */
#  define KINETIS_MCG_HAS_SC_ATMF            1         /* SoC has SC[ATMF] */
#  define KINETIS_MCG_HAS_SC_ATME            1         /* SoC has SC[ATME] */
#  define KINETIS_MCG_HAS_C7                 1         /* SoC has C7 Register */
#  define KINETIS_MCG_HAS_C7_OSCSEL          1         /* SoC has C7[OSCSEL] */
#  define KINETIS_MCG_C7_OSCSEL_BITS         2         /* C7[OSCSEL] is n bits wide */
#  define KINETIS_MCG_HAS_C8                 1         /* SoC has C8 Register */
#  define KINETIS_MCG_HAS_C8_LOCS1           1         /* SoC has C8[LOCS1] */
#  define KINETIS_MCG_HAS_C8_CME1            1         /* SoC has C8[CME1] */
#  define KINETIS_MCG_HAS_C8_LOLRE           1         /* SoC has C8[LOLRE] */
#  define KINETIS_MCG_HAS_C8_LOCRE1          1         /* SoC has C8[LOCRE1] */
#  undef  KINETIS_MCG_HAS_C9                           /* SoC has C9 Register */
#  undef  KINETIS_MCG_HAS_C9_EXT_PLL_LOCS              /* SoC has C9_EXT_PLL[LOCS] */
#  undef  KINETIS_MCG_HAS_C9_PLL_LOCRE                 /* SoC has C9_PLL[LOCRE] */
#  undef  KINETIS_MCG_HAS_C9_PLL_CME                   /* SoC has C9_PLL[CME] */
#  undef  KINETIS_MCG_HAS_C10                          /* SoC has C10 Register */
#  undef  KINETIS_MCG_HAS_C10_LOCS1                    /* SoC has C10[LOCS1] */
#  undef  KINETIS_MCG_HAS_C11                          /* SoC has C11 Register */
#  undef  KINETIS_MCG_HAS_C11_PLL1OSC1                 /* SoC has C1[PRDIV1], C11[PLLSTEN1], C11[PLLCLKEN1], C11[PLLREFSEL1] */
#  undef  KINETIS_MCG_HAS_C11_PLLCS                    /* SoC has C11[PLLCS] */
#  undef  KINETIS_MCG_HAS_C11_PLLREFSEL1               /* SoC has C11[PLLREFSEL1] */
#  undef  KINETIS_MCG_HAS_C12                          /* SoC has C12 Register */
#  undef  KINETIS_MCG_HAS_S2                           /* SoC has S2 Register */
#  undef  KINETIS_MCG_HAS_S2_PLL1OSC1                  /* SoC has S2[LOCS2], S2[OSCINIT1], S2[LOCK1], S2[LOLS1] */
#  undef  KINETIS_MCG_HAS_S2_PLLCST                    /* SoC has S2[PLLCST] */

/* MK66F N/X 1M0/2M0 V MD/LQ 18
 *
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  PART NUMBER     CPU     PIN PACKAGE  TOTAL  PROGRAM EEPROM SRAM  GPIO
 *                  FREQ    CNT          FLASH  FLASH
 *  --------------- ------- --- ------- ------- ------ ------ ------ -----
 *  MK66FN2M0VMD18  180 MHz 144 MAPBGA   2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VMD18  180 MHz 144 MAPBGA  1.25 MB  1 MB   4 KB  256 KB 100
 *  MK66FN2M0VLQ18  180 MHz 144 LQFP     2   MB    —    — KB  260 KB 100
 *  MK66FX1M0VLQ18  180 MHz 144 LQFP    1.25 MB  1 MB   4 KB  256 KB 100
 */

#elif defined(CONFIG_ARCH_CHIP_MK66FN2M0VMD18) || defined(CONFIG_ARCH_CHIP_MK66FX1M0VMD18) || \
      defined(CONFIG_ARCH_CHIP_MK66FN2M0VLQ18) || defined(CONFIG_ARCH_CHIP_MK66FX1M0VLQ18)

/* Verified to Document Number: Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015 */

#  define KINETIS_MCG_VERSION KINETIS_MCG_VERSION_06

/* MCG Configuration Parameters */

#  define KINETIS_MCG_PLL_REF_MIN            8000000   /* OSCCLK/PLL_R minimum */
#  define KINETIS_MCG_PLL_REF_MAX            16000000  /* OSCCLK/PLL_R maximum */
#  define KINETIS_MCG_PLL_INTERNAL_DIVBY     2         /* The PLL clock is divided by 2 before VCO divider */
#  define KINETIS_MCG_HAS_PLL_EXTRA_DIVBY    1         /* Is PLL clock divided by 1 before MCG PLL/FLL clock selection in the SIM module */
#  define KINETIS_MCG_FFCLK_DIVBY            2         /* MCGFFCLK divided by 2 */
#  define KINETIS_MCG_HAS_IRC_48M            1         /* Has 48MHz internal oscillator */
#  undef  KINETIS_MCG_HAS_LOW_FREQ_IRC                 /* Has LTRIMRNG, LFRIM, LSTRIM and bit MC[LIRC_DIV2] */
#  undef  KINETIS_MCG_HAS_HIGH_FREQ_IRC                /* Has HCTRIM, HTTRIM, HFTRIM and bit MC[HIRCEN] */
#  undef  KINETIS_MCG_HAS_PLL_INTERNAL_MODE            /* Has PEI mode or PBI mode */
#  undef  KINETIS_MCG_HAS_RESET_IS_BLPI                /* Has Reset clock mode is BLPI */

/* MCG Register Configuration */

#  define KINETIS_MCG_HAS_C1                 1         /* SoC has C1 Register */
#  define KINETIS_MCG_HAS_C1_IREFS           1         /* SoC has C1[IREFS] */
#  define KINETIS_MCG_HAS_C1_FRDIV           1         /* SoC has C1[FRDIV] */
#  define KINETIS_MCG_C1_FRDIV_MAX           7         /* C1[FRDIV] maximum value 5=1024, 6=1280 7=1536 */
#  define KINETIS_MCG_HAS_C2                 1         /* SoC has C2 Register */
#  define KINETIS_MCG_HAS_C2_HGO             1         /* SoC has C2[HGO] */
#  define KINETIS_MCG_HAS_C2_RANGE           1         /* SoC has C2[RANGE] */
#  define KINETIS_MCG_HAS_C2_FCFTRIM         1         /* SoC has C2[FCFTRIM] */
#  define KINETIS_MCG_HAS_C2_LOCRE0          1         /* SoC has C2[LOCRE0] */
#  define KINETIS_MCG_HAS_C3                 1         /* SoC has C3 Register */
#  define KINETIS_MCG_HAS_C4                 1         /* SoC has C4 Register */
#  define KINETIS_MCG_HAS_C5                 1         /* SoC has C5 Register */
#  define KINETIS_MCG_HAS_C5_PRDIV           1         /* SoC has C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_BASE          1         /* PRDIV base value corresponding to 0 in C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_MAX           8         /* The Maximum value of C5[PRVDIV]) */
#  define KINETIS_MCG_C5_PRDIV_BITS          3         /* Has 3 bits of phase-locked loop (PLL) PRDIV (register C5[PRDIV] */
#  undef  KINETIS_MCG_HAS_C5_PLLREFSEL0                /* SoC has C5[PLLREFSEL0] */
#  define KINETIS_MCG_HAS_C6                 1         /* SoC has C6 Register */
#  define KINETIS_MCG_HAS_C6_VDIV            1         /* SoC has C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_BASE           16        /* VDIV base value corresponding to 0 in C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_MAX            47        /* The Maximum value of C6[VDIV] */
#  define KINETIS_MCG_HAS_C6_CME             1         /* SoC has C6[CME] */
#  define KINETIS_MCG_HAS_C6_PLLS            1         /* SoC has C6[PLLS] */
#  define KINETIS_MCG_HAS_C6_LOLIE0          1         /* SoC has C6[LOLIE0] */
#  define KINETIS_MCG_HAS_S                  1         /* SoC has S Register */
#  define KINETIS_MCG_HAS_S_PLLST            1         /* SoC has S[PLLST] */
#  define KINETIS_MCG_HAS_S_LOCK0            1         /* SoC has S[LOCK0] */
#  define KINETIS_MCG_HAS_S_LOLS             1         /* SoC has S[LOLS] */
#  undef  KINETIS_MCG_HAS_ATC                          /* SoC has ATC Register */
#  define KINETIS_MCG_HAS_ATCVH              1         /* SoC has ATCVH Register */
#  define KINETIS_MCG_HAS_ATCVL              1         /* SoC has ATCVL Register */
#  define KINETIS_MCG_HAS_SC                 1         /* SoC has SC Register */
#  define KINETIS_MCG_HAS_SC_ATMS            1         /* SoC has SC[ATMS] */
#  define KINETIS_MCG_HAS_SC_ATMF            1         /* SoC has SC[ATMF] */
#  define KINETIS_MCG_HAS_SC_ATME            1         /* SoC has SC[ATME] */
#  define KINETIS_MCG_HAS_C7                 1         /* SoC has C7 Register */
#  define KINETIS_MCG_HAS_C7_OSCSEL          1         /* SoC has C7[OSCSEL] */
#  define KINETIS_MCG_C7_OSCSEL_BITS         2         /* C7[OSCSEL] is n bits wide */
#  define KINETIS_MCG_HAS_C8                 1         /* SoC has C8 Register */
#  define KINETIS_MCG_HAS_C8_LOCS1           1         /* SoC has C8[LOCS1] */
#  define KINETIS_MCG_HAS_C8_CME1            1         /* SoC has C8[CME1] */
#  define KINETIS_MCG_HAS_C8_LOLRE           1         /* SoC has C8[LOLRE] */
#  define KINETIS_MCG_HAS_C8_LOCRE1          1         /* SoC has C8[LOCRE1] */
#  define KINETIS_MCG_HAS_C9                 1         /* SoC has C9 Register */
#  define KINETIS_MCG_HAS_C9_EXT_PLL_LOCS    1         /* SoC has C9_EXT_PLL[LOCS] */
#  define KINETIS_MCG_HAS_C9_PLL_LOCRE       1         /* SoC has C9_PLL[LOCRE] */
#  define KINETIS_MCG_HAS_C9_PLL_CME         1         /* SoC has C9_PLL[CME] */
#  undef  KINETIS_MCG_HAS_C10                          /* SoC has C10 Register */
#  undef  KINETIS_MCG_HAS_C10_LOCS1                    /* SoC has C10[LOCS1] */
#  define KINETIS_MCG_HAS_C11                          /* SoC has C11 Register */
#  undef  KINETIS_MCG_HAS_C11_PLL1OSC1                 /* SoC has C1[PRDIV1], C11[PLLSTEN1], C11[PLLCLKEN1], C11[PLLREFSEL1] */
#  define KINETIS_MCG_HAS_C11_PLLCS                    /* SoC has C11[PLLCS] */
#  undef  KINETIS_MCG_HAS_C11_PLLREFSEL1               /* SoC has C11[PLLREFSEL1] */
#  undef  KINETIS_MCG_HAS_C12                          /* SoC has C12 Register */
#  define KINETIS_MCG_HAS_S2                           /* SoC has S2 Register */
#  undef  KINETIS_MCG_HAS_S2_PLL1OSC1                  /* SoC has S2[LOCS2], S2[OSCINIT1], S2[LOCK1], S2[LOLS1] */
#  define KINETIS_MCG_HAS_S2_PLLCST                    /* SoC has S2[PLLCST] */

#else
#  error "Unsupported Kinetis chip"
#endif

/* Use the catch all configuration for the MCG based on the implementations in nuttx prior 2/3/2017 */

#if KINETIS_MCG_VERSION == KINETIS_MCG_VERSION_UKN

/* MCG Configuration Parameters */

#  define KINETIS_MCG_PLL_REF_MIN            2000000   /* OSCCLK/PLL_R minimum */
#  define KINETIS_MCG_PLL_REF_MAX            4000000   /* OSCCLK/PLL_R maximum */
#  define KINETIS_MCG_PLL_INTERNAL_DIVBY     1         /* The PLL clock is divided by 1 before VCO divider */
#  define KINETIS_MCG_HAS_PLL_EXTRA_DIVBY    1         /* Is PLL clock divided by 1 before MCG PLL/FLL clock selection in the SIM module */
#  define KINETIS_MCG_FFCLK_DIVBY            1         /* MCGFFCLK divided by 1 */
#  undef  KINETIS_MCG_HAS_IRC_48M                      /* Has 48MHz internal oscillator */
#  undef  KINETIS_MCG_HAS_LOW_FREQ_IRC                 /* Has LTRIMRNG, LFRIM, LSTRIM and bit MC[LIRC_DIV2] */
#  undef  KINETIS_MCG_HAS_HIGH_FREQ_IRC                /* Has HCTRIM, HTTRIM, HFTRIM and bit MC[HIRCEN] */
#  undef  KINETIS_MCG_HAS_PLL_INTERNAL_MODE            /* Has PEI mode or PBI mode */
#  undef  KINETIS_MCG_HAS_RESET_IS_BLPI                /* Has Reset clock mode is BLPI */

/* MCG Register Configuration */

#  define KINETIS_MCG_HAS_C1                 1         /* SoC has C1 Register */
#  define KINETIS_MCG_HAS_C1_IREFS           1         /* SoC has C1[IREFS] */
#  define KINETIS_MCG_HAS_C1_FRDIV           1         /* SoC has C1[FRDIV] */
#  define KINETIS_MCG_C1_FRDIV_MAX           5         /* C1[FRDIV] maximum value 5=1024, 6=1280 7=1536 */
#  define KINETIS_MCG_HAS_C2                 1         /* SoC has C2 Register */
#  define KINETIS_MCG_HAS_C2_HGO             1         /* SoC has C2[HGO] */
#  define KINETIS_MCG_HAS_C2_RANGE           1         /* SoC has C2[RANGE] */
#  undef  KINETIS_MCG_HAS_C2_FCFTRIM                   /* SoC has C2[FCFTRIM] */
#  undef  KINETIS_MCG_HAS_C2_LOCRE0                    /* SoC has C2[LOCRE0] */
#  define KINETIS_MCG_HAS_C3                 1         /* SoC has C3 Register */
#  define KINETIS_MCG_HAS_C4                 1         /* SoC has C4 Register */
#  define KINETIS_MCG_HAS_C5                 1         /* SoC has C5 Register */
#  define KINETIS_MCG_HAS_C5_PRDIV           1         /* SoC has C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_BASE          1         /* PRDIV base value corresponding to 0 in C5[PRDIV] */
#  define KINETIS_MCG_C5_PRDIV_MAX           25        /* The Maximum value of C5[PRVDIV]) */
#  define KINETIS_MCG_C5_PRDIV_BITS          5         /* Has 5 bits of phase-locked loop (PLL) PRDIV (register C5[PRDIV] */
#  undef  KINETIS_MCG_HAS_C5_PLLREFSEL0                /* SoC has C5[PLLREFSEL0] */
#  define KINETIS_MCG_HAS_C6                 1         /* SoC has C6 Register */
#  define KINETIS_MCG_HAS_C6_VDIV            1         /* SoC has C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_BASE           24        /* VDIV base value corresponding to 0 in C6[VDIV] */
#  define KINETIS_MCG_C6_VDIV_MAX            55        /* The Maximum value of C6[VDIV] */
#  define KINETIS_MCG_HAS_C6_CME             1         /* SoC has C6[CME] */
#  define KINETIS_MCG_HAS_C6_PLLS            1         /* SoC has C6[PLLS] */
#  define KINETIS_MCG_HAS_C6_LOLIE0          1         /* SoC has C6[LOLIE0] */
#  define KINETIS_MCG_HAS_S                  1         /* SoC has S Register */
#  define KINETIS_MCG_HAS_S_PLLST            1         /* SoC has S[PLLST] */
#  define KINETIS_MCG_HAS_S_LOCK0            1         /* SoC has S[LOCK0] */
#  define KINETIS_MCG_HAS_S_LOLS             1         /* SoC has S[LOLS] */
#  define KINETIS_MCG_HAS_ATC                1         /* SoC has ATC Register */
#  define KINETIS_MCG_HAS_ATCVH              1         /* SoC has ATCVH Register */
#  define KINETIS_MCG_HAS_ATCVL              1         /* SoC has ATCVL Register */
#  undef  KINETIS_MCG_HAS_SC                           /* SoC has SC Register */
#  undef  KINETIS_MCG_HAS_SC_ATMS                      /* SoC has SC[ATMS] */
#  undef  KINETIS_MCG_HAS_SC_ATMF                      /* SoC has SC[ATMF] */
#  undef  KINETIS_MCG_HAS_SC_ATME                      /* SoC has SC[ATME] */
#  undef  KINETIS_MCG_HAS_C7                           /* SoC has C7 Register */
#  undef  KINETIS_MCG_HAS_C7_OSCSEL                    /* SoC has C7[OSCSEL] */
#  undef  KINETIS_MCG_C7_OSCSEL_BITS                   /* C7[OSCSEL] is n bits wide */
#  undef  KINETIS_MCG_HAS_C8                           /* SoC has C8 Register */
#  undef  KINETIS_MCG_HAS_C8_LOCS1                     /* SoC has C8[LOCS1] */
#  undef  KINETIS_MCG_HAS_C8_CME1                      /* SoC has C8[CME1] */
#  undef  KINETIS_MCG_HAS_C8_LOLRE                     /* SoC has C8[LOLRE] */
#  undef  KINETIS_MCG_HAS_C8_LOCRE1                    /* SoC has C8[LOCRE1] */
#  undef  KINETIS_MCG_HAS_C9                           /* SoC has C9 Register */
#  undef  KINETIS_MCG_HAS_C9_EXT_PLL_LOCS              /* SoC has C9_EXT_PLL[LOCS] */
#  undef  KINETIS_MCG_HAS_C9_PLL_LOCRE                 /* SoC has C9_PLL[LOCRE] */
#  undef  KINETIS_MCG_HAS_C9_PLL_CME                   /* SoC has C9_PLL[CME] */
#  undef  KINETIS_MCG_HAS_C10                          /* SoC has C10 Register */
#  undef  KINETIS_MCG_HAS_C10_LOCS1                    /* SoC has C10[LOCS1] */
#  undef  KINETIS_MCG_HAS_C11                          /* SoC has C11 Register */
#  undef  KINETIS_MCG_HAS_C11_PLL1OSC1                 /* SoC has C1[PRDIV1], C11[PLLSTEN1], C11[PLLCLKEN1], C11[PLLREFSEL1] */
#  undef  KINETIS_MCG_HAS_C11_PLLCS                    /* SoC has C11[PLLCS] */
#  undef  KINETIS_MCG_HAS_C11_PLLREFSEL1               /* SoC has C11[PLLREFSEL1] */
#  undef  KINETIS_MCG_HAS_C12                          /* SoC has C12 Register */
#  undef  KINETIS_MCG_HAS_S2                           /* SoC has S2 Register */
#  undef  KINETIS_MCG_HAS_S2_PLL1OSC1                  /* SoC has S2[LOCS2], S2[OSCINIT1], S2[LOCK1], S2[LOLS1] */
#  undef  KINETIS_MCG_HAS_S2_PLLCST                    /* SoC has S2[PLLCST] */
#endif

#if !defined(KINETIS_MCG_VERSION)
#  error "No KINETIS_MCG_VERSION defined!"
#endif

#if defined(KINETIS_MCG_HAS_C5_PRDIV)
#  define KINETIS_MCG_C5_PRDIV_MASK  ((1 << (KINETIS_MCG_C5_PRDIV_BITS))-1)
#endif

#if defined(KINETIS_MCG_HAS_C7_OSCSEL)
#  define KINETIS_MCG_C7_OSCSEL_MASK  ((1 << (KINETIS_MCG_C7_OSCSEL_BITS))-1)
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_MCG_H */
