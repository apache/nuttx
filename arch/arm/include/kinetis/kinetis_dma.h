/****************************************************************************
 * arch/arm/include/kinetis/kinetis_dma.h
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMA_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Note: It is envisioned that in the long term as a chip is added. The
 * author of the new chip definitions will either find the exact
 * configuration in an existing chip define and add the new chip to it Or add
 * the DMA feature configuration #defines to the chip ifdef list below. In
 * either case the author should mark it as "Verified to Document Number:"
 * taken from the reference manual.
 */

/* DMA Register Configuration
 *
 * KINETIS_DMA_HAS_CR_ERGA                    - DMA has CR[ERGA]
 * KINETIS_DMA_HAS_CR_GRP0PRI                 - DMA has CR[GRP0PRI]
 * KINETIS_DMA_HAS_CR_GRP1PRI                 - DMA has CR[GRP1PRI]
 * KINETIS_DMA_HAS_ES_ERRCHN                  - DMA has ES[ERRCHN]
 * KINETIS_DMA_HAS_ES_ERRCHN_BITS             - DMA has 4 bit ES[ERRCHN]
 * KINETIS_DMA_HAS_ES_GPE                     - DMA has ES[GPE]
 * KINETIS_DMA_HAS_CEEI_CEEI_BITS             - DMA has 4 bit CEEi[CEEI]
 * KINETIS_DMA_HAS_SEEI_SEEI_BITS             - DMA has 4 bit SEEi[SEEI]
 * KINETIS_DMA_HAS_CERQ_CERQ_BITS             - DMA has 4 bit CERQ[CERQ]
 * KINETIS_DMA_HAS_SERQ_SERQ_BITS             - DMA has 4 bit SERQ[SERQ]
 * KINETIS_DMA_HAS_CDNE_CDNE_BITS             - DMA has 4 bit CDNE[CDNE]
 * KINETIS_DMA_HAS_SSRT_SSRT_BITS             - DMA has 4 bit SSRT[SSRT]
 * KINETIS_DMA_HAS_CERR_CERR_BITS             - DMA has 4 bit CERR[CERR]
 * KINETIS_DMA_HAS_CINT_CINT_BITS             - DMA has 4 bit CINT[CINT]
 * KINETIS_DMA_HAS_DCHPRI_CHPRI_BITS          - DMA has 4 bit DCHPRI[DCHPRI]
 * KINETIS_DMA_HAS_DCHPRI_GRPPRI              - DMA has DCHPRI[GRPPRI]
 * KINETIS_DMA_HAS_EARS                       - DMA has EARS Register
 * KINETIS_DMA_HAS_TCD_CITER1_LINKCH_BITS     - DMA has 4 bit
 *                                                  TCD_CITER[LINKCH]
 * KINETIS_DMA_HAS_TCD_CSR_MAJORLINKCH_BITS   - DMA has 4 bit
 *                                                  TCD_CSR[MAJORLINKCH]
 * KINETIS_DMA_HAS_TCD_BITER1_LINKCH_BITS     - DMA has 4 bit
 *                                                  TCD_BITER[LINKCH]
 */

/* Describe the version of the DMA
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_DMA_VERSION_UKN   -1  /* What was in nuttx prior to 8/9/2018 */
#define KINETIS_DMA_VERSION_01     1  /* Verified Document Number:
                                       * K60P144M100SF2V2RM Rev. 2, Jun 2012
                                       * K64P144M120SF5RM Rev. 2, Jan 2014
                                       */
#define KINETIS_DMA_VERSION_02     2  /* Verified Document Number:
                                       * K60P144M150SF3RM Rev. 3, Nov 2014
                                       * K66P144M180SF5RMV2 Rev. 2, May 2015
                                       */

#if defined(CONFIG_ARCH_CHIP_MK60DN256VLQ10) || defined(CONFIG_ARCH_CHIP_MK60DX256VLQ10) || \
    defined(CONFIG_ARCH_CHIP_MK60DN512VLQ10) || defined(CONFIG_ARCH_CHIP_MK60DN256VMD10) || \
    defined(CONFIG_ARCH_CHIP_MK60DX256VMD10) || defined(CONFIG_ARCH_CHIP_MK60DN512VMD10)

/* Verified to Document Number: K60P144M100SF2V2RM Rev. 2 Jun 2012 */

#  define KINETIS_DMA_VERSION KINETIS_DMA_VERSION_01

#elif defined(CONFIG_ARCH_CHIP_MK60FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VMD12) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VLQ15) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ15) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VMD15) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VMD15)

/* Verified Document Number: K60P144M150SF3RM Rev. 3, Nov 2014 */

#  define KINETIS_DMA_VERSION KINETIS_DMA_VERSION_02

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

/* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */

#  define KINETIS_DMA_VERSION KINETIS_DMA_VERSION_01

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

/* Verified to Document Number:
 * Document Number: K66P144M180SF5RMV2 Rev. 2, May 2015
 */

#  define KINETIS_DMA_VERSION KINETIS_DMA_VERSION_02
#else
#  define KINETIS_DMA_VERSION KINETIS_DMA_VERSION_UKN
#endif

/* Use the catch all configuration for the DMA based on
 * the implementations in nuttx prior 8/10/2018
 */

#if KINETIS_DMA_VERSION == KINETIS_DMA_VERSION_UKN

/* DMA Register Configuration */

#if defined(CONFIG_KINETIS_DMA)
#  warning "DMA Unsupported on this Kinetis device!"
#endif

#elif KINETIS_DMA_VERSION == KINETIS_DMA_VERSION_01

/* DMA Register Configuration */
#  undef DMA_CHN_PER_GROUP                                          /* Number of channels per group */

#  undef KINETIS_DMA_HAS_CR_ERGA                                    /* DMA has CR[ERGA   ] */
#  undef KINETIS_DMA_HAS_CR_GRP0PRI                                 /* DMA has CR[GRP0PRI] */
#  undef KINETIS_DMA_HAS_CR_GRP1PRI                                 /* DMA has CR[GRP1PRI] */
#  define KINETIS_DMA_HAS_ES_ERRCHN                           1     /* DMA has ES[ERRCHN]   */
#  define KINETIS_DMA_HAS_ES_ERRCHN_BITS                      4     /* DMA has 4 bit ES[ERRCHN] */
#  undef KINETIS_DMA_HAS_ES_GPE                                     /* DMA has ES[GPE   ]   */
#  define KINETIS_DMA_HAS_CEEI_CEEI_BITS                      4     /* DMA has 4 bit CEEi[CEEI]*/
#  define KINETIS_DMA_HAS_SEEI_SEEI_BITS                      4     /* DMA has 4 bit SEEi[SEEI]*/
#  define KINETIS_DMA_HAS_CERQ_CERQ_BITS                      4     /* DMA has 4 bit CERQ[CERQ]*/
#  define KINETIS_DMA_HAS_SERQ_SERQ_BITS                      4     /* DMA has 4 bit SERQ[SERQ]*/
#  define KINETIS_DMA_HAS_CDNE_CDNE_BITS                      4     /* DMA has 4 bit CDNE[CDNE]*/
#  define KINETIS_DMA_HAS_SSRT_SSRT_BITS                      4     /* DMA has 4 bit SSRT[SSRT]*/
#  define KINETIS_DMA_HAS_CERR_CERR_BITS                      4     /* DMA has 4 bit CERR[CERR]*/
#  define KINETIS_DMA_HAS_CINT_CINT_BITS                      4     /* DMA has 4 bit CINT[CINT]*/
#  define KINETIS_DMA_HAS_DCHPRI_CHPRI_BITS                   4     /* DMA has 4 bit DCHPRI[DCHPRI]*/
#  undef KINETIS_DMA_HAS_DCHPRI_GRPPRI                              /* DMA has DCHPRI[GRPPRI] */
#  undef KINETIS_DMA_HAS_EARS                                       /* DMA has EARS Register */
#  define KINETIS_DMA_HAS_TCD_CITER1_LINKCH_BITS              4     /* DMA has 4 bit TCD_CITER[LINKCH] */
#  define KINETIS_DMA_HAS_TCD_CSR_MAJORLINKCH_BITS            4     /* DMA has 4 bit TCD_CSR[MAJORLINKCH] */
#  define KINETIS_DMA_HAS_TCD_BITER1_LINKCH_BITS              4     /* DMA has 4 bit TCD_BITER[LINKCH] */

#elif KINETIS_DMA_VERSION == KINETIS_DMA_VERSION_02

/* DMA Register Configuration */

#  define DMA_CHN_PER_GROUP                                  16     /* Number of channels per group */
#  define KINETIS_DMA_HAS_CR_ERGA                             1     /* DMA has CR[ERGA   ] */
#  define KINETIS_DMA_HAS_CR_GRP0PRI                          1     /* DMA has CR[GRP0PRI] */
#  define KINETIS_DMA_HAS_CR_GRP1PRI                          1     /* DMA has CR[GRP1PRI] */
#  define KINETIS_DMA_HAS_ES_ERRCHN_BITS                      5     /* DMA has 5 bit ES[ERRCHN] */
#  define KINETIS_DMA_HAS_ES_GPE                              1     /* DMA has ES[GPE   ]   */
#  define KINETIS_DMA_HAS_CEEI_CEEI_BITS                      5     /* DMA has 5 bit CEEI[CEEI]*/
#  define KINETIS_DMA_HAS_SEEI_SEEI_BITS                      5     /* DMA has 5 bit SEEI[SEEI]*/
#  define KINETIS_DMA_HAS_CERQ_CERQ_BITS                      5     /* DMA has 5 bit CERQ[CERQ]*/
#  define KINETIS_DMA_HAS_SERQ_SERQ_BITS                      5     /* DMA has 5 bit SERQ[SERQ]*/
#  define KINETIS_DMA_HAS_CDNE_CDNE_BITS                      5     /* DMA has 5 bit CDNE[CDNE]*/
#  define KINETIS_DMA_HAS_SSRT_SSRT_BITS                      5     /* DMA has 5 bit SSRT[SSRT]*/
#  define KINETIS_DMA_HAS_CERR_CERR_BITS                      5     /* DMA has 5 bit CERR[CERR]*/
#  define KINETIS_DMA_HAS_CINT_CINT_BITS                      5     /* DMA has 5 bit CINT[CINT]*/
#  define KINETIS_DMA_HAS_DCHPRI_CHPRI_BITS                   5     /* DMA has 5 bit DCHPRI[DCHPRI]*/
#  define KINETIS_DMA_HAS_DCHPRI_GRPPRI                       1     /* DMA has DCHPRI[GRPPRI] */
#  define KINETIS_DMA_HAS_EARS                                1     /* DMA has EARS Register */
#  define KINETIS_DMA_HAS_TCD_CITER1_LINKCH_BITS              5     /* DMA has 5 bit TCD_CITER[LINKCH] */
#  define KINETIS_DMA_HAS_TCD_CSR_MAJORLINKCH_BITS            5     /* DMA has 5 bit TCD_CSR[MAJORLINKCH] */
#  define KINETIS_DMA_HAS_TCD_BITER1_LINKCH_BITS              5     /* DMA has 5 bit TCD_BITER[LINKCH] */
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMA_H */
