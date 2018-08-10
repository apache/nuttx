/************************************************************************************
 * arch/arm/include/kinetis/kinetis_dmamux.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Jan Okle <jan@leitwert.ch>
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

#ifndef __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMAMUX_H
#define __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMAMUX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Note: It is envisioned that in the long term as a chip is added. The author of
 * the new chip definitions will either find the exact configuration in an existing
 * chip define and add the new chip to it Or add the DMAMUX feature configuration
 * #defines to the chip ifdef list below. In either case the author should mark
 * it as "Verified to Document Number:" taken from the reference manual.
 */

/* DMAMUX Register Configuration
 *
 * KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG    0|1    - SoC has reg in 0,1,2..KINETIS_NDMACH
 */

/* Describe the version of the DMA
 *
 * These defines are not related to any NXP reference but are merely
 * a way to label the versions we are using
 */

#define KINETIS_DMAMUX_VERSION_UKN   -1  /* What was in nuttx prior to 8/9/2018 */
#define KINETIS_DMAMUX_VERSION_01     1  /* Verified Document Number:
                                          * K60P144M150SF3RM Rev. 3, Nov 2014
                                          */
#define KINETIS_DMAMUX_VERSION_02     2  /* Verified Document Number:
                                          * K60P144M100SF2V2RM Rev. 2, Jun 2012
                                          * K64P144M120SF5RM Rev. 2, Jan 2014
                                          * K66P144M180SF5RMV2 Rev. 2, May 2015
                                          */

#if defined(CONFIG_ARCH_CHIP_MK60DN256VLQ10) || defined(CONFIG_ARCH_CHIP_MK60DX256VLQ10) || \
    defined(CONFIG_ARCH_CHIP_MK60DN512VLQ10) || defined(CONFIG_ARCH_CHIP_MK60DN256VMD10) || \
    defined(CONFIG_ARCH_CHIP_MK60DX256VMD10) || defined(CONFIG_ARCH_CHIP_MK60DN512VMD10)

/* Verified to Document Number: K60P144M100SF2V2RM Rev. 2 Jun 2012 */

#  define KINETIS_DMAMUX_VERSION KINETIS_DMAMUX_VERSION_02

#elif defined(CONFIG_ARCH_CHIP_MK60FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VMD12) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VLQ15) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VLQ15) || \
      defined(CONFIG_ARCH_CHIP_MK60FX512VMD15) || defined(CONFIG_ARCH_CHIP_MK60FN1M0VMD15)

/* Verified Document Number: K60P144M150SF3RM Rev. 3, Nov 2014 */

#  define KINETIS_DMAMUX_VERSION KINETIS_DMAMUX_VERSION_01

#elif defined(CONFIG_ARCH_CHIP_MK64FN1M0VLL12) || defined(CONFIG_ARCH_CHIP_MK64FX512VLL12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VDC12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VDC12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VLQ12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VLQ12) || \
      defined(CONFIG_ARCH_CHIP_MK64FX512VMD12) || defined(CONFIG_ARCH_CHIP_MK64FN1M0VMD12)

/* Verified to Document Number: K64P144M120SF5RM Rev. 2, January 2014 */

#  define KINETIS_DMAMUX_VERSION KINETIS_DMAMUX_VERSION_02

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

#  define KINETIS_DMAMUX_VERSION KINETIS_DMAMUX_VERSION_02

#else
#  define KINETIS_DMAMUX_VERSION KINETIS_DMAMUX_VERSION_UKN
#endif

/* Use the catch all configuration for the DMAMUX based on the implementations in
 * nuttx prior 8/10/2018
 */

#if KINETIS_DMA_VERSION == KINETIS_DMAMUX_VERSION_01

/* DMAMUX Register Configuration */

#  define  KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG    0   /* SoC has reg in 3,2,1,0..KINETIS_NDMACH */

#elif KINETIS_DMA_VERSION == KINETIS_DMAMUX_VERSION_02

/* DMAMUX Register Configuration */

#  define KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG     1   /* SoC has reg in 0,1,2,3..KINETIS_NDMACH */

#elif KINETIS_DMA_VERSION == KINETIS_DMAMUX_VERSION_UKN

/* DMAMUX Register Configuration */

#  define KINETIS_DMAMUX_HAS_MONOTONIC_CHCFG     1   /* SoC has reg in 0,1,2,3..KINETIS_NDMACH */
#  if defined(CONFIG_KINETIS_DMA)
#    warning "DMAMUX Assuming monotonic CHCFG addressing!"
#  endif
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_KINETIS_DMAMUX_H */
