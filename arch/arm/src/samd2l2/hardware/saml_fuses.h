/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/sam_fuses.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_FUSES_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_FUSES_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* NVM Fuse addresses **********************************************************************/

/* NVM user row bits */

#define SAM_NVMUSER_ROW0                 (SAM_NVMUSER_ROW + 0x0000)   /* Bits 0-31 */
#define SAM_NVMUSER_ROW1                 (SAM_NVMUSER_ROW + 0x0004)   /* Bits 32-63 */

/* NVM Software Calibration Area */

#define SAM_NVMCALIB_AREA0               (SAM_NVMCALIB_AREA + 0x0000) /* Bits 0-31 */
#define SAM_NVMCALIB_AREA1               (SAM_NVMCALIB_AREA + 0x0000) /* Bits 32-63 */
#define SAM_NVMCALIB_AREA2               (SAM_NVMCALIB_AREA + 0x0000) /* Bits 64-95 */
#define SAM_NVMCALIB_AREA3               (SAM_NVMCALIB_AREA + 0x0000) /* Bits 96-127 */

/* Fuse bit-field definitions **************************************************************/
/* NVM user row bits 0-31 */

#define SAM_FUSES_BOOTPROT_ADDR          SAM_NVMUSER_ROW0
#define SAM_FUSES_BOOTPROT_SHIFT         (0)       /* Bits 0-2: Bootloader Size */
#define SAM_FUSES_BOOTPROT_MASK          (7 << SAM_FUSES_BOOTPROT_SHIFT)
#  define SAM_FUSES_BOOTPROT(n)          ((uint32_t)(n) << SAM_FUSES_BOOTPROT_SHIFT)

#define SAM_FUSES_EEPROM_SIZE_ADDR       SAM_NVMUSER_ROW0
#define SAM_FUSES_EEPROM_SIZE_SHIFT      (4)      /* Bits 4-6: EEPROM Size */
#define SAM_FUSES_EEPROM_SIZE_MASK       (7 << SAM_FUSES_EEPROM_SIZE_SHIFT)
#  define SAM_FUSES_EEPROM_SIZE(n)       ((uint32_t)(n) << SAM_FUSES_EEPROM_SIZE_SHIFT)

#define SAM_FUSES_BOD33LEVEL_ADDR        SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33LEVEL_SHIFT       (8)      /* Bits 8-13: BOD33 Level */
#define SAM_FUSES_BOD33LEVEL_MASK        (0x3f << SAM_FUSES_BOD33LEVEL_SHIFT)
#  define SAM_FUSES_BOD33LEVEL(n)        (((uint32_t)n) << SAM_FUSES_BOD33LEVEL_SHIFT)

#define SAM_FUSES_BOD33_DIS_ADDR         SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33_DIS_SHIFT        (14)     /* Bit 14: BOD33 Disable */
#define SAM_FUSES_BOD33_DIS_MASK         (1 << SAM_FUSES_BOD33_DIS_SHIFT)

#define SAM_FUSES_BOD33_ACTION_ADDR      SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33_ACTION_SHIFT (15)     /* Bits 15-16: BOD33 Action */
#define SAM_FUSES_BOD33_ACTION_MASK      (3 << SAM_FUSES_BOD33_ACTION_SHIFT)
#  define SAM_FUSES_BOD33_ACTION(n)      (((uint32_t)n) << SAM_FUSES_BOD33_ACTION_SHIFT)

#define SAM_FUSES_BOD12LEVEL_ADDR        SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD12LEVEL_SHIFT       (17)     /* Bits 17-22: BOD12 Level */
#define SAM_FUSES_BOD12LEVEL_MASK        (0x1f << SAM_FUSES_BOD12LEVEL_SHIFT)
#  define SAM_FUSES_BOD12LEVEL(n)        ((uint32_t)(n) << SAM_FUSES_BOD12LEVEL_SHIFT)

#define SAM_FUSES_BOD12_DIS_ADDR         SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD12_DIS_SHIFT        (23)      /* Bit 23: BOD12 Disable */
#define SAM_FUSES_BOD12_DIS_MASK         (1 << SAM_FUSES_BOD12_DIS_SHIFT)

#define SAM_FUSES_BOD12_ACTION_ADDR      SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD12_ACTION_SHIFT     (24)      /* Bits 24-25: BOD12 Action */
#define SAM_FUSES_BOD12_ACTION_MASK      (3 << SAM_FUSES_BOD12_ACTION_SHIFT)
#  define SAM_FUSES_BOD12_ACTION(n)      ((uint32_t)(n) << SAM_FUSES_BOD12_ACTION_SHIFT)

#define SAM_FUSES_WDT_ENA_ADDR           SAM_NVMUSER_ROW0
#define SAM_FUSES_WDT_ENA_SHIFT          (26)      /* Bit 26: WDT Enable */
#define SAM_FUSES_WDT_ENA_MASK           (1 << SAM_FUSES_WDT_ENA_SHIFT)

#define SAM_FUSES_WDT_ALWAYSON_ADDR      SAM_NVMUSER_ROW0
#define SAM_FUSES_WDT_ALWAYSON_SHIFT     (27)      /* Bit 27: WDT Always On */
#define SAM_FUSES_WDT_ALWAYSON_MASK      (1 << SAM_FUSES_WDT_ALWAYSON_SHIFT)

#define SAM_FUSES_WDT_PER_ADDR           SAM_NVMUSER_ROW0
#define SAM_FUSES_WDT_PER_SHIFT          (28)      /* Bits 28-31: WDT Period */
#define SAM_FUSES_WDT_PER_MASK           (15 << SAM_FUSES_WDT_PER_SHIFT)
#  define SAM_FUSES_WDT_PER(n)           ((uint32_t)(n) << SAM_FUSES_WDT_PER_SHIFT)

/* NVM user row bits 32-64 */

#define SAM_FUSES_WDT_WINDOW_ADDR        SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_WINDOW_SHIFT       (0)       /* Bits 32-35: WDT Window */
#define SAM_FUSES_WDT_WINDOW_MASK        (15 << SAM_FUSES_WDT_WINDOW_SHIFT)

#define SAM_FUSES_WDT_EWOFFSET_ADDR      SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_EWOFFSET_SHIFT     (4)       /* Bits 36-39:  WDT Early Warning Offset */
#define SAM_FUSES_WDT_EWOFFSET_MASK      (15 << SAM_FUSES_WDT_EWOFFSET_SHIFT)
#  define SAM_FUSES_WDT_EWOFFSET(n)      ((uint32_t)(n) << SAM_FUSES_WDT_EWOFFSET_SHIFT)

#define SAM_FUSES_WDT_WEN_ADDR           SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_WEN_SHIFT          (8)       /* Bit 40: WDT Window Mode Enable */
#define SAM_FUSES_WDT_WEN_MASK           (1 << SAM_FUSES_WDT_WEN_SHIFT)

#define SAM_FUSES_BOD33_HYST_ADDR        SAM_NVMUSER_ROW1
#define SAM_FUSES_BOD33_HYST_SHIFT       (9)      /* Bit 41: BOD33 Hysteresis */
#define SAM_FUSES_BOD33_HYST_MASK        (1 << SAM_FUSES_BOD33_HYST_SHIFT)

#define SAM_FUSES_BOD12_HYST_ADDR        SAM_NVMUSER_ROW1
#define SAM_FUSES_BOD12_HYST_SHIFT       (10)     /* Bit 42: BOD12 Hysteresis */
#define SAM_FUSES_BOD12_HYST_MASK        (1 << SAM_FUSES_BOD12_HYST_SHIFT)

#define SAM_FUSES_LOCK_ADDR              SAM_NVMUSER_ROW1
#define SAM_FUSES_LOCK_SHIFT             (16)      /* Bits 48-63: NVM Region Lock bits */
#define SAM_FUSES_LOCK_MASK              (0xffff << SAM_FUSES_LOCK_SHIFT)
#  define SAM_FUSES_LOCK(n)              ((uint32_t)(n) << SAM_FUSES_LOCK_SHIFT)

/* NVM Software Calibration Area bits 0-31 */

#define SAM_FUSES_ADC_LINEARITY_ADDR     SAM_NVMCALIB_AREA0
#define SAM_FUSES_ADC_LINEARITY_SHIFT    (0)       /* Bits 0-2: ADC Linearity bits */
#define SAM_FUSES_ADC_LINEARITY_MASK     (7 << SAM_FUSES_ADC_LINEARITY_SHIFT)
#  define SAM_FUSES_ADC_LINEARITY(n)     ((uint32_t)(n) << SAM_FUSES_ADC_LINEARITY_SHIFT)

#define SAM_FUSES_ADC_BIASCAL_ADDR       SAM_NVMCALIB_AREA0
#define SAM_FUSES_ADC_BIASCAL_SHIFT      (3)        /* Bits 3-5: ADC Bias Calibration */
#define SAM_FUSES_ADC_BIASCAL_MASK       (7 << SAM_FUSES_ADC_BIASCAL_SHIFT)
#  define SAM_FUSES_ADC_BIASCAL(n)       ((uint32_t)(n) << SAM_FUSES_ADC_BIASCAL_SHIFT)

#define SAM_FUSES_OSC32KCAL_ADDR         SAM_NVMCALIB_AREA0
#define SAM_FUSES_OSC32KCAL_SHIFT        (6)      /* Bits 6-12: OSC32K Calibration */
#define SAM_FUSES_OSC32KCAL_MASK         (0x7f << SAM_FUSES_OSC32KCAL_SHIFT)
#  define SAM_FUSES_OSC32KCAL(n)         ((uint32_t)(n) << SAM_FUSES_OSC32KCAL_SHIFT)

#define SAM_FUSES_USBTRANSN_ADDR         SAM_NVMCALIB_AREA0
#define SAM_FUSES_USBTRANSN_SHIFT        (13)     /* Bits 13-17: USB TRNSN Calibration */
#define SAM_FUSES_USBTRANSN_MASK         (31 << SAM_FUSES_USBTRANSN_SHIFT)
#  define SAM_FUSES_USBTRANSN(n)         ((uint32_t)(n) << SAM_FUSES_USBTRANSN_SHIFT)

#define SAM_FUSES_USBTRANSP_ADDR         SAM_NVMCALIB_AREA0
#define SAM_FUSES_USBTRANSP_SHIFT        (6)      /* Bits 18-22: USB TRNSP Calibration */
#define SAM_FUSES_USBTRANSP_MASK         (31 << SAM_FUSES_USBTRANSP_SHIFT)
#  define SAM_FUSES_USBTRANSP(n)         ((uint32_t)(n) << SAM_FUSES_USBTRANSP_SHIFT)

#define SAM_FUSES_USBTRIM_ADDR           SAM_NVMCALIB_AREA0
#define SAM_FUSES_USBTRIM_SHIFT          (23)     /* Bits 23-25: USB TRIM Calibration */
#define SAM_FUSES_USBTRIM_MASK           (7 << SAM_FUSES_USBTRIM_SHIFT)
#  define SAM_FUSES_USBTRIM(n)           ((uint32_t)(n) << SAM_FUSES_USBTRIM_SHIFT)

#define SAM_FUSES_DFLL48MCC_ADDR         SAM_NVMCALIB_AREA0
#define SAM_FUSES_DFLL48MCC_SHIFT        (26)     /* Bits 26-31: DFLL48M Coarse Calibration */
#define SAM_FUSES_DFLL48MCC_MASK         (0x3f << SAM_FUSES_DFLL48MCC_SHIFT)
#  define SAM_FUSES_DFLL48MCC(n)         ((uint32_t)(n) << SAM_FUSES_DFLL48MCC_SHIFT)

/* NVM Software Calibration Area bits 32-63 - Reserved */
/* NVM Software Calibration Area bits 64-95 - Reserved */
/* NVM Software Calibration Area bits 96-127 - Reserved */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_FUSES_H */
