/****************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_fuses.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_FUSES_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_FUSES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NVM Fuse addresses *******************************************************/

/* NVM user row bits.  The first eight 32-bit words contains calibration
 * information that are automatically read at device power-on.
 *  The remaining 480 bytes can be used for storing custom parameters.
 */

#define SAM_NVMUSER_ROW0                 (SAM_NVM_USERPAGE + 0x0000)   /* Bits 0-31 */
#define SAM_NVMUSER_ROW1                 (SAM_NVM_USERPAGE + 0x0004)   /* Bits 32-63 */
#define SAM_NVMUSER_ROW2                 (SAM_NVM_USERPAGE + 0x0008)   /* Bits 64-95 */
#define SAM_NVMUSER_ROW3                 (SAM_NVM_USERPAGE + 0x000c)   /* Bits 96-127 */
#define SAM_NVMUSER_ROW4                 (SAM_NVM_USERPAGE + 0x0010)   /* Bits 128-159 */
#define SAM_NVMUSER_ROW5                 (SAM_NVM_USERPAGE + 0x0014)   /* Bits 160-191 */
#define SAM_NVMUSER_ROW6                 (SAM_NVM_USERPAGE + 0x0018)   /* Bits 192-223 */
#define SAM_NVMUSER_ROW7                 (SAM_NVM_USERPAGE + 0x001c)   /* Bits 224-255 */

/* NVM Software Calibration Area */

#define SAM_NVM_CALIBAREA0               (SAM_NVM_CALIBAREA + 0x0000) /* Bits 0-31 */
#define SAM_NVM_CALIBAREA1               (SAM_NVM_CALIBAREA + 0x0004) /* Bits 32-63 */

/* NVM Software Calibration Area for Temperature (read-only) */

#define SAM_NVM_CALIBTEMP0               (SAM_NVM_CALIBAREA + 0x0080) /* Bits 0-31 */
#define SAM_NVM_CALIBTEMP1               (SAM_NVM_CALIBAREA + 0x0080) /* Bits 32-63 */
#define SAM_NVM_CALIBTEMP2               (SAM_NVM_CALIBAREA + 0x0080) /* Bits 64-95 */
#define SAM_NVM_CALIBTEMP3               (SAM_NVM_CALIBAREA + 0x0080) /* Bits 96-127 */

/* Fuse bit-field definitions ***********************************************/

/* NVM user pages */

#define SAM_FUSES_BOD33_DIS_ADDR         SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33_DIS_SHIFT        (0)      /* Bit 0: BOD33 Disable */
#define SAM_FUSES_BOD33_DIS_MASK         (1 << SAM_FUSES_BOD33_DIS_SHIFT)

#define SAM_FUSES_BOD33LEVEL_ADDR        SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33LEVEL_SHIFT       (1)      /* Bits 1-8: BOD33 Level */
#define SAM_FUSES_BOD33LEVEL_MASK        (0xff << SAM_FUSES_BOD33LEVEL_SHIFT)
#  define SAM_FUSES_BOD33LEVEL(n)        (((uint32_t)n) << SAM_FUSES_BOD33LEVEL_SHIFT)

#define SAM_FUSES_BOD33_ACTION_ADDR      SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33_ACTION_SHIFT     (9)      /* Bits 9-10: BOD33 Action */
#define SAM_FUSES_BOD33_ACTION_MASK      (3 << SAM_FUSES_BOD33_ACTION_SHIFT)
#  define SAM_FUSES_BOD33_ACTION(n)      (((uint32_t)n) << SAM_FUSES_BOD33_ACTION_SHIFT)

#define SAM_FUSES_BOD33_HYST_ADDR        SAM_NVMUSER_ROW0
#define SAM_FUSES_BOD33_HYST_SHIFT       (11)     /* Bits 11-14: BOD33 Hysteresis */
#define SAM_FUSES_BOD33_HYST_MASK        (3 << SAM_FUSES_BOD33_HYST_SHIFT)
#  define SAM_FUSES_BOD33_HYST(n)        (((uint32_t)n) << SAM_FUSES_BOD33_HYST_SHIFT)

                                                  /* Bits 15-25: Reserved */

#define SAM_FUSES_NVM_BOOT_ADDR          SAM_NVMUSER_ROW0
#define SAM_FUSES_NVM_BOOT_SHIFT         (26)     /* Bits 26-29: NVM Bootloader Size */
#define SAM_FUSES_NVM_BOOT_MASK          (15 << SAM_FUSES_NVM_BOOT_SHIFT)
#  define SAM_FUSES_NVM_BOOT(n)          (((uint32_t)n) << SAM_FUSES_NVM_BOOT_SHIFT)

                                          /* Bits 30-31: Factor settings */

#define SAM_FUSES_SEESBLK_ADDR           SAM_NVMUSER_ROW1
#define SAM_FUSES_SEESBLK_SHIFT          (0)      /* Bits 32-35: NVM Bootloader Size */
#define SAM_FUSES_SEESBLK_MASK           (15 << SAM_FUSES_SEESBLK_SHIFT)
#  define SAM_FUSES_SEESBLK(n)           (((uint32_t)n) << SAM_FUSES_SEESBLK_SHIFT)

#define SAM_FUSES_SEEPSZ_ADDR            SAM_NVMUSER_ROW1
#define SAM_FUSES_SEEPSZ_SHIFT           (4)      /* Bits 36-38: SmartEEPROM Page Size */
#define SAM_FUSES_SEEPSZ_MASK            (7 << SAM_FUSES_SEEPSZ_SHIFT)
#  define SAM_FUSES_SEEPSZ(n)            (((uint32_t)n) << SAM_FUSES_SEEPSZ_SHIFT)

#define SAM_FUSES_RAM_ECCDIS_ADDR        SAM_NVMUSER_ROW1
#define SAM_FUSES_RAM_ECCDIS_SHIFT       (7)      /* Bit 39: SmartEEPROM Page Size */
#define SAM_FUSES_RAM_ECCDIS_MASK        (1 << SAM_FUSES_RAM_ECCDIS_SHIFT)

                                          /* Bits 40-47: Factor settings */

#define SAM_FUSES_WDT_ENA_ADDR           SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_ENA_SHIFT          (16)      /* Bit 48: WDT Enable */
#define SAM_FUSES_WDT_ENA_MASK           (1 << SAM_FUSES_WDT_ENA_SHIFT)

#define SAM_FUSES_WDT_ALWAYSON_ADDR      SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_ALWAYSON_SHIFT     (17)      /* Bit 49: WDT Always On */
#define SAM_FUSES_WDT_ALWAYSON_MASK      (1 << SAM_FUSES_WDT_ALWAYSON_SHIFT)

#define SAM_FUSES_WDT_PER_ADDR           SAM_NVMUSER_ROW0
#define SAM_FUSES_WDT_PER_SHIFT          (18)      /* Bits 50-53: WDT Period */
#define SAM_FUSES_WDT_PER_MASK           (15 << SAM_FUSES_WDT_PER_SHIFT)
#  define SAM_FUSES_WDT_PER(n)           ((uint32_t)(n) << SAM_FUSES_WDT_PER_SHIFT)

#define SAM_FUSES_WDT_WINDOW_ADDR        SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_WINDOW_SHIFT       (22)      /* Bits 54-57: WDT Window */
#define SAM_FUSES_WDT_WINDOW_MASK        (15 << SAM_FUSES_WDT_WINDOW_SHIFT)

#define SAM_FUSES_WDT_EWOFFSET_ADDR      SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_EWOFFSET_SHIFT     (26)      /* Bits 58-61:  WDT Early Warning Offset */
#define SAM_FUSES_WDT_EWOFFSET_MASK      (15 << SAM_FUSES_WDT_EWOFFSET_SHIFT)
#  define SAM_FUSES_WDT_EWOFFSET(n)      ((uint32_t)(n) << SAM_FUSES_WDT_EWOFFSET_SHIFT)

#define SAM_FUSES_WDT_WEN_ADDR           SAM_NVMUSER_ROW1
#define SAM_FUSES_WDT_WEN_SHIFT          (30)      /* Bit 62: WDT Window Mode Enable*/
#define SAM_FUSES_WDT_WEN_MASK           (1 << SAM_FUSES_WDT_WEN_SHIFT)

                                           /* Bit 63: Reserved */

#define SAM_FUSES_LOCK_ADDR              SAM_NVMUSER_ROW2
#define SAM_FUSES_LOCK_SHIFT             (0)       /* Bits 64-95: NVM Region Lock bits */
#define SAM_FUSES_LOCK_MASK              (0xffffffff << SAM_FUSES_LOCK_SHIFT)
#  define SAM_FUSES_LOCK(n)              ((uint32_t)(n) << SAM_FUSES_LOCK_SHIFT)

                                           /* Bits 96-127: Usage page */

                                           /* Bits 128-159: Reserved */

                                           /* Bits 160-255: User pages */

/* NVM Calibration Area */

#define SAM_FUSES_AC_BIAS_ADDR              SAM_NVM_CALIBAREA0
#define SAM_FUSES_AC_BIAS_SHIFT             (0)       /* Bits 0-1: AC Comparator 0/1 Bias Scaling */
#define SAM_FUSES_AC_BIAS_MASK              (3 << SAM_FUSES_AC_BIAS_SHIFT)
#  define SAM_FUSES_AC_BIAS(n)              ((uint32_t)(n) << SAM_FUSES_AC_BIAS_SHIFT)

#define SAM_FUSES_ADC0_BIASCOMP_ADDR        SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC0_BIASCOMP_SHIFT       (2)       /* Bits 2-4: ADC0 Bias comparator scaling */
#define SAM_FUSES_ADC0_BIASCOMP_MASK        (7 << SAM_FUSES_ADC0_BIASCOMP_SHIFT)
#  define SAM_FUSES_ADC0_BIASCOMP(n)        ((uint32_t)(n) << SAM_FUSES_ADC0_BIASCOMP_SHIFT)

#define SAM_FUSES_ADC0_BIASREFBUF_ADDR      SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC0_BIASREFBUF_SHIFT     (5)       /* Bits 5-7: ADC0 Bias comparator scaling */
#define SAM_FUSES_ADC0_BIASREFBUF_MASK      (7 << SAM_FUSES_ADC0_BIASREFBUF_SHIFT)
#  define SAM_FUSES_ADC0_BIASREFBUF(n)      ((uint32_t)(n) << SAM_FUSES_ADC0_BIASREFBUF_SHIFT)

#define SAM_FUSES_ADC0_BIASR2R_ADDR         SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC0_BIASR2R_SHIFT        (8)       /* Bits 8-10: ADC0 Bias comparator scaling */
#define SAM_FUSES_ADC0_BIASR2R_MASK         (7 << SAM_FUSES_ADC0_BIASR2R_SHIFT)
#  define SAM_FUSES_ADC0_BIASR2R(n)         ((uint32_t)(n) << SAM_FUSES_ADC0_BIASR2R_SHIFT)

                                              /* Bits 11-15: Reserved */

#define SAM_FUSES_ADC1_BIASCOMP_ADDR        SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC1_BIASCOMP_SHIFT       (16)      /* Bits 16-18: ADC1 Bias comparator scaling */
#define SAM_FUSES_ADC1_BIASCOMP_MASK        (7 << SAM_FUSES_ADC1_BIASCOMP_SHIFT)
#  define SAM_FUSES_ADC1_BIASCOMP(n)        ((uint32_t)(n) << SAM_FUSES_ADC1_BIASCOMP_SHIFT)

#define SAM_FUSES_ADC1_BIASREFBUF_ADDR      SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC1_BIASREFBUF_SHIFT     (19)      /* Bits 19-21: ADC1 Bias comparator scaling */
#define SAM_FUSES_ADC1_BIASREFBUF_MASK      (7 << SAM_FUSES_ADC1_BIASREFBUF_SHIFT)
#  define SAM_FUSES_ADC1_BIASREFBUF(n)      ((uint32_t)(n) << SAM_FUSES_ADC1_BIASREFBUF_SHIFT)

#define SAM_FUSES_ADC1_BIASR2R_ADDR         SAM_NVM_CALIBAREA0
#define SAM_FUSES_ADC1_BIASR2R_SHIFT        (22)      /* Bits 22-24: ADC1 Bias comparator scaling */
#define SAM_FUSES_ADC1_BIASR2R_MASK         (7 << SAM_FUSES_ADC1_BIASR2R_SHIFT)
#  define SAM_FUSES_ADC1_BIASR2R(n)         ((uint32_t)(n) << SAM_FUSES_ADC1_BIASR2R_SHIFT)

                                              /* Bits 25-35: Reserved */

#define SAM_FUSES_USBTRANSN_ADDR            SAM_NVM_CALIBAREA1
#define SAM_FUSES_USBTRANSN_SHIFT           (0)     /* Bits 32-36: USB TRNSN Calibration */
#define SAM_FUSES_USBTRANSN_MASK            (31 << SAM_FUSES_USBTRANSN_SHIFT)
#  define SAM_FUSES_USBTRANSN(n)            ((uint32_t)(n) << SAM_FUSES_USBTRANSN_SHIFT)

#define SAM_FUSES_USBTRANSP_ADDR            SAM_NVM_CALIBAREA1
#define SAM_FUSES_USBTRANSP_SHIFT           (5)      /* Bits 37-41: USB TRNSP Calibration */
#define SAM_FUSES_USBTRANSP_MASK            (31 << SAM_FUSES_USBTRANSP_SHIFT)
#  define SAM_FUSES_USBTRANSP(n)            ((uint32_t)(n) << SAM_FUSES_USBTRANSP_SHIFT)

#define SAM_FUSES_USBTRIM_ADDR              SAM_NVM_CALIBAREA1
#define SAM_FUSES_USBTRIM_SHIFT             (11)     /* Bits 43-44: USB TRIM Calibration */
#define SAM_FUSES_USBTRIM_MASK              (7 << SAM_FUSES_USBTRIM_SHIFT)
#  define SAM_FUSES_USBTRIM(n)              ((uint32_t)(n) << SAM_FUSES_USBTRIM_SHIFT)

/* NVM Software Calibration Area -- Temperature Calibration Parameters */

#define SAM_FUSES_TLI_ADDR                  SAM_NVM_CALIBTEMP0
#define SAM_FUSES_TLI_SHIFT                 (0)     /* Bits 0-7: Integer part temperature TL */
#define SAM_FUSES_TLI_MASK                  (0xff << SAM_FUSES_TLI_SHIFT)
#  define SAM_FUSES_TLI(n)                  ((uint32_t)(n) << SAM_FUSES_TLI_SHIFT)

#define SAM_FUSES_TLD_ADDR                  SAM_NVM_CALIBTEMP0
#define SAM_FUSES_TLD_SHIFT                 (8)     /* Bits 8-11: Fractional part temperature TL */
#define SAM_FUSES_TLD_MASK                  (15 << SAM_FUSES_TLD_SHIFT)
#  define SAM_FUSES_TLD(n)                  ((uint32_t)(n) << SAM_FUSES_TLD_SHIFT)

#define SAM_FUSES_THI_ADDR                  SAM_NVM_CALIBTEMP0
#define SAM_FUSES_THI_SHIFT                 (12)     /* Bits 12-19: Integer part temperature TH */
#define SAM_FUSES_THI_MASK                  (0xff << SAM_FUSES_THI_SHIFT)
#  define SAM_FUSES_THI(n)                 ((uint32_t)(n) << SAM_FUSES_THI_SHIFT)

#define SAM_FUSES_THD_ADDR                  SAM_NVM_CALIBTEMP0
#define SAM_FUSES_THD_SHIFT                 (20)     /* Bits 20-23: Fractional part temperature TH */
#define SAM_FUSES_THD_MASK                  (15 << SAM_FUSES_THD_SHIFT)
#  define SAM_FUSES_THD(n)                  ((uint32_t)(n) << SAM_FUSES_THD_SHIFT)

                                                     /* 24-39: Reserved */

#define SAM_FUSES_VPL_ADDR                  SAM_NVM_CALIBTEMP1
#define SAM_FUSES_VPL_SHIFT                 (8)     /* Bits 40-51:Temperature calibration parameter */
#define SAM_FUSES_VPL_MASK                  (0xfff << SAM_FUSES_VPL_SHIFT)
#  define SAM_FUSES_VPL(n)                  ((uint32_t)(n) << SAM_FUSES_VPL_SHIFT)

#define SAM_FUSES_VPH_ADDR                  SAM_NVM_CALIBTEMP1
#define SAM_FUSES_VPH_SHIFT                 (20)     /* Bits 52-63: Temperature calibration parameter */
#define SAM_FUSES_VPH_MASK                  (0xfff << SAM_FUSES_VPH_SHIFT)
#  define SAM_FUSES_VPH(n)                  ((uint32_t)(n) << SAM_FUSES_VPH_SHIFT)

#define SAM_FUSES_VCL_ADDR                  SAM_NVM_CALIBTEMP2
#define SAM_FUSES_VCL_SHIFT                 (20)     /* Bits 64-75: Temperature calibration parameter */
#define SAM_FUSES_VCL_MASK                  (0xfff << SAM_FUSES_VCL_SHIFT)
#  define SAM_FUSES_VCL(n)                  ((uint32_t)(n) << SAM_FUSES_VCL_SHIFT)

#define SAM_FUSES_VCH_ADDR                 SAM_NVM_CALIBTEMP2
#define SAM_FUSES_VCH_SHIFT                 (11)     /* Bits 76-87: Temperature calibration parameter */
#define SAM_FUSES_VCH_MASK                  (0xfff << SAM_FUSES_VCH_SHIFT)
#  define SAM_FUSES_VCH(n)                  ((uint32_t)(n) << SAM_FUSES_VCH_SHIFT)

                                                     /* 88-127: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_FUSES_H */
