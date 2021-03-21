/****************************************************************************
 * arch/arm/include/kinetis/kinetis_pmc.h
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_PMC_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_PMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Note:
 * It is envisioned that in the long term as a chip is added. The author of
 * the new chip definitions will either find the exact configuration in an
 * existing chip define and add the new chip to it Or add the PMC feature
 * configuration #defines to the chip ifdef list below. In either case the
 * author should mark it as "Verified to Document Number:" taken from the
 * reference manual.
 *
 * To maintain backward compatibility to the version of NuttX prior to
 * 2/22/2017, the catch all KINETIS_PMC_VERSION_UKN configuration is assigned
 * to all the chips that did not have any conditional compilation based on
 * KINETIS_K64 or KINETIS_K66. This is  a "No worse" than the original code
 * solution. N.B. Each original chip "if"definitions have been left intact so
 * that the complete legacy definitions prior to 2/22/2017 may be filled in
 * completely when vetted.
 */

/* PMC Register Configuration
 *
 * KINETIS_PMC_HAS_REGSC             - SoC has REGSC Register
 * KINETIS_PMC_HAS_REGSC_ACKISO      - SoC has REGSC[ACKISO]
 * KINETIS_PMC_HAS_REGSC_VLPRS       - SoC has REGSC[VLPRS]
 * KINETIS_PMC_HAS_REGSC_VLPO        - SoC has REGSC[VLPO]
 * KINETIS_PMC_HAS_REGSC_REGFPM      - SoC has REGSC[REGFPM]
 * KINETIS_PMC_HAS_REGSC_BGEN        - SoC has REGSC[BGEN]
 * KINETIS_PMC_HAS_REGSC_TRAMPO      - SoC has REGSC[TRAMPO]
 * KINETIS_PMC_HAS_REGSC_REGONS      - SoC has REGSC[REGONS]
 *
 * KINETIS_PMC_HAS_HVDSC1            - SoC has HVDSC1 Register
 * KINETIS_PMC_HAS_SRAMCTL           - SoC has SRAMCTL Register
 */

/* Describe the version of the PMC
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_PMC_VERSION_UKN (-1) /* What was in nuttx prior to 2/22/2017 */
#define KINETIS_PMC_VERSION_01    1  /* Verified to Document Number: K60P144M150SF3RM Rev. 3, November 2014 */
#define KINETIS_PMC_VERSION_04    4  /* Verified to Document Numbers:
                                      * K20P64M72SF1RM  Rev. 1.1, Dec 2012
                                      * K64P144M120SF5RM Rev. 2, January 2014
                                      * K66P144M180SF5RMV2 Rev. 2, May 2015 */
#define KINETIS_PMC_VERSION_05    5  /* Verified to Document Number: K28P210M150SF5RM Rev. 4, August 2017 */

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

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

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

/* Verified to Document Number: K20P64M72SF1RM Rev. 1.1, Dec 2012 */

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_04

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  define KINETIS_PMC_HAS_REGSC_ACKISO      1 /* SoC has REGSC[ACKISO] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPRS         /* SoC does not have REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  define KINETIS_PMC_HAS_REGSC_BGEN        1 /* SoC has REGSC[BGEN] */
#  undef  KINETIS_PMC_HAS_REGSC_TRAMPO        /* SoC does not have REGSC[TRAMPO] */

#  undef  KINETIS_PMC_HAS_HVDSC1              /* SoC does not have HVDSC1 Register */
#  undef  KINETIS_PMC_HAS_SRAMCTL             /* SoC does not have SRAMCTL Register */

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

#elif defined(CONFIG_ARCH_CHIP_MK28FN2M0VMI15) || defined(CONFIG_ARCH_CHIP_MK28FN2M0CAU15R)

/* Verified to Document Number: K28P210M150SF5RM Rev. 4, August 2017 */

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_05

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  define KINETIS_PMC_HAS_REGSC_ACKISO      1 /* SoC has REGSC[ACKISO] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPRS         /* SoC does not have REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  define KINETIS_PMC_HAS_REGSC_BGEN        1 /* SoC has REGSC[BGEN] */
#  undef  KINETIS_PMC_HAS_REGSC_TRAMPO        /* SoC does not have REGSC[TRAMPO] */

#  define KINETIS_PMC_HAS_HVDSC1            1 /* SoC has HVDSC1 Register */
#  define KINETIS_PMC_HAS_SRAMCTL           1 /* SoC has SRAMCTL Register */

#elif defined(CONFIG_ARCH_CHIP_MK40X64VFX50) || defined(CONFIG_ARCH_CHIP_MK40X64VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X64VLK50) || defined(CONFIG_ARCH_CHIP_MK40X64VMB50)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VFX50) || defined(CONFIG_ARCH_CHIP_MK40X128VLH50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK50) || defined(CONFIG_ARCH_CHIP_MK40X128VMB50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL50) || defined(CONFIG_ARCH_CHIP_MK40X128VML50) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VFX72) || defined(CONFIG_ARCH_CHIP_MK40X128VLH72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLK72) || defined(CONFIG_ARCH_CHIP_MK40X128VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X128VLL72) || defined(CONFIG_ARCH_CHIP_MK40X128VML72)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLK72) || defined(CONFIG_ARCH_CHIP_MK40X256VMB72) || \
      defined(CONFIG_ARCH_CHIP_MK40X256VLL72) || defined(CONFIG_ARCH_CHIP_MK40X256VML72)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK40N512VLK100) || defined(CONFIG_ARCH_CHIP_MK40N512VMB100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLL100) || defined(CONFIG_ARCH_CHIP_MK40N512VML100) || \
      defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLL100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLL100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLL100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VML100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VML100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VML100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VLQ100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VLQ100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N256VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60X256VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_UKN

#elif defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12)

/* Verified to Document Number: K60P144M100SF2V2RM Rev. 2 Jun 2012 */

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_01

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  define KINETIS_PMC_HAS_REGSC_ACKISO      1 /* SoC has REGSC[ACKISO] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPRS         /* SoC does not have REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  undef  KINETIS_PMC_HAS_REGSC_BGEN          /* SoC does not have REGSC[BGEN] */
#  undef  KINETIS_PMC_HAS_REGSC_TRAMPO        /* SoC does not have REGSC[TRAMPO] */

#  undef  KINETIS_PMC_HAS_HVDSC1              /* SoC does not have HVDSC1 Register */
#  undef  KINETIS_PMC_HAS_SRAMCTL             /* SoC does not have SRAMCTL Register */

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

/* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_04

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  define KINETIS_PMC_HAS_REGSC_ACKISO      1 /* SoC has REGSC[ACKISO] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPRS         /* SoC does not have REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  define KINETIS_PMC_HAS_REGSC_BGEN        1 /* SoC has REGSC[BGEN] */
#  undef  KINETIS_PMC_HAS_REGSC_TRAMPO        /* SoC does not have REGSC[TRAMPO] */

#  undef  KINETIS_PMC_HAS_HVDSC1              /* SoC does not have HVDSC1 Register */
#  undef  KINETIS_PMC_HAS_SRAMCTL             /* SoC does not have SRAMCTL Register */

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

/* Verified to Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015 */

#  define KINETIS_PMC_VERSION KINETIS_PMC_VERSION_04

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  define KINETIS_PMC_HAS_REGSC_ACKISO      1 /* SoC has REGSC[ACKISO] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPRS         /* SoC does not have REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  define KINETIS_PMC_HAS_REGSC_BGEN        1 /* SoC has REGSC[BGEN] */
#  undef  KINETIS_PMC_HAS_REGSC_TRAMPO        /* SoC does not have REGSC[TRAMPO] */

#  undef  KINETIS_PMC_HAS_HVDSC1              /* SoC does not have HVDSC1 Register */
#  undef  KINETIS_PMC_HAS_SRAMCTL             /* SoC does not have SRAMCTL Register */

#else
#  error "Unsupported Kinetis chip"
#endif

/* Use the catch all configuration for the PMC based on
 * the implementations in NuttX prior to 2/3/2017
 */

#if KINETIS_PMC_VERSION == KINETIS_PMC_VERSION_UKN

/* PMC Register Configuration */

#  define KINETIS_PMC_HAS_REGSC             1 /* SoC has REGSC Register */
#  define KINETIS_PMC_HAS_REGSC_REGONS      1 /* SoC has REGSC[REGONS] */
#  undef  KINETIS_PMC_HAS_REGSC_ACKISO        /* SoC does not have REGSC[ACKISO] */
#  define KINETIS_PMC_HAS_REGSC_VLPRS       1 /* SoC has REGSC[VLPRS] */
#  undef  KINETIS_PMC_HAS_REGSC_VLPO          /* SoC does not have REGSC[VLPO] */
#  undef  KINETIS_PMC_HAS_REGSC_REGFPM        /* SoC does not have REGSC[REGFPM] */
#  undef  KINETIS_PMC_HAS_REGSC_BGEN          /* SoC does not have REGSC[BGEN] */
#  define KINETIS_PMC_HAS_REGSC_TRAMPO      1 /* SoC has REGSC[TRAMPO] */

#  undef  KINETIS_PMC_HAS_HVDSC1              /* SoC does not have HVDSC1 Register */
#  undef  KINETIS_PMC_HAS_SRAMCTL             /* SoC does not have SRAMCTL Register */

#endif

#if !defined(KINETIS_PMC_VERSION)
#  error "No KINETIS_PMC_VERSION defined!"
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_PMC_H */
