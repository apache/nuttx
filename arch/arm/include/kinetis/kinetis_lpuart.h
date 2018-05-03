/************************************************************************************
 * arch/arm/include/kinetis/kinetis_lpuart.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_LPUART_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_LPUART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Note: It is envisioned that in the long term as a chip is added. The author of
 * the new chip definitions will either find the exact configuration in an existing
 * chip define and add the new chip to it Or add the LPUART feature configuration
 * #defines to the chip ifdef list below. In either case the author should mark
 * it as "Verified to Document Number:" taken from the reference manual.
 *
 * To maintain backward compatibility to the version of NuttX prior to
 * 2/22/2017, the catch all KINETIS_LPUART_VERSION_UKN configuration is assigned
 * to all the chips that did not have any conditional compilation based on
 * KINETIS_K64 or KINETIS_K66. This is  a "No worse" than the original code solution.
 * N.B. Each original chip "if"definitions have been left intact so that the
 * complete legacy definitions prior to 2/22/2017 may be filled in completely when
 * vetted.
 */

/* LPUART Register Configuration
 *
 * KINETIS_LPUART_HAS_MODIR_RTSWATER    - SoC has MODIR[RTSWATER]
 * KINETIS_LPUART_HAS_FIFO              - SoC has FIFO Register
 * KINETIS_LPUART_HAS_WATER             - SoC has WATER Register
 */

/* Describe the version of the LPUART
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_LPUART_VERSION_UKN  -1  /* What was in nuttx prior to 5/3/2018 */
#define KINETIS_LPUART_VERSION_01    1  /* Verified to Document Number: K28P210M150SF5RM Rev. 4, August 2017 */

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

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

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

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

/* MK28FN2M0---15-
 *
 *  --------------- ------- --- ------- ------ ------- ------ -----
 *  PART NUMBER     CPU     PIN PACKAGE PROGRAM EEPROM SRAM  GPIO
 *                  FREQ    CNT         FLASH
 *  --------------- ------- --- ------- ------ ------- ------ -----
 *  MK28FN2M0VMI15  150 MHz 169  MAPBGA  2 MB   None    1 MB  120
 *  MK28FN2M0CAU15R 150 MHz 210  WLCSP   2 MB   None    1 MB  120
 *  --------------- ------- --- ------- ------ ------- ------ -----
 */

#elif defined(CONFIG_ARCH_CHIP_MK28FN2M0VMI15) || \
      defined(CONFIG_ARCH_CHIP_MK28FN2M0CAU15R)

/* Verified to Document Number: Verified to Document Number: K28P210M150SF5RM Rev. 4, August 2017 */

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_01

/* LPUART Register Configuration */

#  define KINETIS_LPUART_HAS_MODIR_RTSWATER    1 /* SoC has MODIR[RTSWATER] */
#  define KINETIS_LPUART_HAS_FIFO              1 /* SoC has FIFO Register */
#  define KINETIS_LPUART_HAS_WATER             1 /* SoC has WATER Register */

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

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

#  define KINETIS_LPUART_VERSION KINETIS_LPUART_VERSION_UKN

#else
#  error "Unsupported Kinetis chip"
#endif

/* Use the catch all configuration for the LPUART based on the implementations in nuttx prior 2/3/2017 */

#if KINETIS_LPUART_VERSION == KINETIS_LPUART_VERSION_UKN

/* LPUART Register Configuration */

#  undef  KINETIS_LPUART_HAS_MODIR_RTSWATER      /* SoC does not have MODIR[RTSWATER] */
#  undef  KINETIS_LPUART_HAS_FIFO                /* SoC does not have FIFO Register */
#  undef  KINETIS_LPUART_HAS_WATER               /* SoC does not have WATER Register */

#endif

#if !defined(KINETIS_LPUART_VERSION)
#  error "No KINETIS_LPUART_VERSION defined!"
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_LPUART_H */
