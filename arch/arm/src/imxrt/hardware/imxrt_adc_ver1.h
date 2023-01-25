/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_adc_ver1.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER1_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMXRT_ADC_HC0_OFFSET                0x0000  /* Control register for hardware triggers */
#define IMXRT_ADC_HC1_OFFSET                0x0004  /* Control register for hardware triggers */
#define IMXRT_ADC_HC2_OFFSET                0x0008  /* Control register for hardware triggers */
#define IMXRT_ADC_HC3_OFFSET                0x000c  /* Control register for hardware triggers */
#define IMXRT_ADC_HC4_OFFSET                0x0010  /* Control register for hardware triggers */
#define IMXRT_ADC_HC5_OFFSET                0x0014  /* Control register for hardware triggers */
#define IMXRT_ADC_HC6_OFFSET                0x0018  /* Control register for hardware triggers */
#define IMXRT_ADC_HC7_OFFSET                0x001c  /* Control register for hardware triggers */
#define IMXRT_ADC_HS_OFFSET                 0x0020  /* Status register for HW triggers */
#define IMXRT_ADC_R0_OFFSET                 0x0024  /* Data result register for HW triggers */
#define IMXRT_ADC_R1_OFFSET                 0x0028  /* Data result register for HW triggers */
#define IMXRT_ADC_R2_OFFSET                 0x002c  /* Data result register for HW triggers */
#define IMXRT_ADC_R3_OFFSET                 0x0030  /* Data result register for HW triggers */
#define IMXRT_ADC_R4_OFFSET                 0x0034  /* Data result register for HW triggers */
#define IMXRT_ADC_R5_OFFSET                 0x0038  /* Data result register for HW triggers */
#define IMXRT_ADC_R6_OFFSET                 0x003c  /* Data result register for HW triggers */
#define IMXRT_ADC_R7_OFFSET                 0x0040  /* Data result register for HW triggers */
#define IMXRT_ADC_CFG_OFFSET                0x0044  /* Configuration register */
#define IMXRT_ADC_GC_OFFSET                 0x0048  /* General control register */
#define IMXRT_ADC_GS_OFFSET                 0x004c  /* General status register */
#define IMXRT_ADC_CV_OFFSET                 0x0050  /* Compare value register */
#define IMXRT_ADC_OFS_OFFSET                0x0054  /* Offset correction value register */
#define IMXRT_ADC_CAL_OFFSET                0x0058  /* Calibration value register */

/* Register addresses *******************************************************/

/* ADC1 Register Addresses */

#define IMXRT_ADC1_HC0                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC_OFFSET)   /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC1                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC1_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC2                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC2_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC3                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC3_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC4                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC4_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC5                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC5_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC6                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC6_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HC7                      (IMXRT_ADC1_BASE + IMXRT_ADC_HC7_OFFSET)  /* ADC1 Control register for hardware triggers */
#define IMXRT_ADC1_HS                       (IMXRT_ADC1_BASE + IMXRT_ADC_HS_OFFSET)   /* ADC1 Status register for HW triggers */
#define IMXRT_ADC1_R0                       (IMXRT_ADC1_BASE + IMXRT_ADC_R0_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R1                       (IMXRT_ADC1_BASE + IMXRT_ADC_R1_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R2                       (IMXRT_ADC1_BASE + IMXRT_ADC_R2_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R3                       (IMXRT_ADC1_BASE + IMXRT_ADC_R3_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R4                       (IMXRT_ADC1_BASE + IMXRT_ADC_R4_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R5                       (IMXRT_ADC1_BASE + IMXRT_ADC_R5_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R6                       (IMXRT_ADC1_BASE + IMXRT_ADC_R6_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_R7                       (IMXRT_ADC1_BASE + IMXRT_ADC_R7_OFFSET)   /* ADC1 Data result register for HW triggers */
#define IMXRT_ADC1_CFG                      (IMXRT_ADC1_BASE + IMXRT_ADC_CFG_OFFSET)  /* ADC1 Configuration register */
#define IMXRT_ADC1_GC                       (IMXRT_ADC1_BASE + IMXRT_ADC_GC_OFFSET)   /* ADC1 General control register */
#define IMXRT_ADC1_GS                       (IMXRT_ADC1_BASE + IMXRT_ADC_GS_OFFSET)   /* ADC1 General status register */
#define IMXRT_ADC1_CV                       (IMXRT_ADC1_BASE + IMXRT_ADC_CV_OFFSET)   /* ADC1 Compare value register */
#define IMXRT_ADC1_OFS                      (IMXRT_ADC1_BASE + IMXRT_ADC_OFS_OFFSET)  /* ADC1 Offset correction value register */
#define IMXRT_ADC1_CAL                      (IMXRT_ADC1_BASE + IMXRT_ADC_CAL_OFFSET)  /* ADC1 Calibration value register */

/* ADC2 Register Addresses */

#define IMXRT_ADC2_HC0                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC_OFFSET)   /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC1                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC1_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC2                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC2_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC3                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC3_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC4                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC4_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC5                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC5_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC6                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC6_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HC7                      (IMXRT_ADC2_BASE + IMXRT_ADC_HC7_OFFSET)  /* ADC2 Control register for hardware triggers */
#define IMXRT_ADC2_HS                       (IMXRT_ADC2_BASE + IMXRT_ADC_HS_OFFSET)   /* ADC2 Status register for HW triggers */
#define IMXRT_ADC2_R0                       (IMXRT_ADC2_BASE + IMXRT_ADC_R0_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R1                       (IMXRT_ADC2_BASE + IMXRT_ADC_R1_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R2                       (IMXRT_ADC2_BASE + IMXRT_ADC_R2_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R3                       (IMXRT_ADC2_BASE + IMXRT_ADC_R3_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R4                       (IMXRT_ADC2_BASE + IMXRT_ADC_R4_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R5                       (IMXRT_ADC2_BASE + IMXRT_ADC_R5_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R6                       (IMXRT_ADC2_BASE + IMXRT_ADC_R6_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_R7                       (IMXRT_ADC2_BASE + IMXRT_ADC_R7_OFFSET)   /* ADC2 Data result register for HW triggers */
#define IMXRT_ADC2_CFG                      (IMXRT_ADC2_BASE + IMXRT_ADC_CFG_OFFSET)  /* ADC2 Configuration register */
#define IMXRT_ADC2_GC                       (IMXRT_ADC2_BASE + IMXRT_ADC_GC_OFFSET)   /* ADC2 General control register */
#define IMXRT_ADC2_GS                       (IMXRT_ADC2_BASE + IMXRT_ADC_GS_OFFSET)   /* ADC2 General status register */
#define IMXRT_ADC2_CV                       (IMXRT_ADC2_BASE + IMXRT_ADC_CV_OFFSET)   /* ADC2 Compare value register */
#define IMXRT_ADC2_OFS                      (IMXRT_ADC2_BASE + IMXRT_ADC_OFS_OFFSET)  /* ADC2 Offset correction value register */
#define IMXRT_ADC2_CAL                      (IMXRT_ADC2_BASE + IMXRT_ADC_CAL_OFFSET)  /* ADC2 Calibration value register */

/* Register Bit Definitions *************************************************/

/* Control register for hardware  & SW triggers for n=0,1..7 */

#define ADC_HC_ADCH_SHIFT                    (0)        /* Bits: 0-4  Input Channel Select */
#define ADC_HC_ADCH_MASK                     (31 << ADC_HC_ADCH_SHIFT)
#  define ADC_HC_ADCH(n)                     ((uint32_t)(n) << ADC_HC_ADCH_SHIFT)
#  define ADC_HC_ADCH_EXT_0                  (0 << ADC_HC_ADCH_SHIFT)   /* External channels 0 */
#  define ADC_HC_ADCH_EXT_1                  (1 << ADC_HC_ADCH_SHIFT)   /* External channels 1 */
#  define ADC_HC_ADCH_EXT_2                  (2 << ADC_HC_ADCH_SHIFT)   /* External channels 2 */
#  define ADC_HC_ADCH_EXT_3                  (3 << ADC_HC_ADCH_SHIFT)   /* External channels 3 */
#  define ADC_HC_ADCH_EXT_4                  (4 << ADC_HC_ADCH_SHIFT)   /* External channels 4 */
#  define ADC_HC_ADCH_EXT_5                  (5 << ADC_HC_ADCH_SHIFT)   /* External channels 5 */
#  define ADC_HC_ADCH_EXT_6                  (6 << ADC_HC_ADCH_SHIFT)   /* External channels 6 */
#  define ADC_HC_ADCH_EXT_7                  (7 << ADC_HC_ADCH_SHIFT)   /* External channels 7 */
#  define ADC_HC_ADCH_EXT_8                  (8 << ADC_HC_ADCH_SHIFT)   /* External channels 8 */
#  define ADC_HC_ADCH_EXT_9                  (9 << ADC_HC_ADCH_SHIFT)   /* External channels 9 */
#  define ADC_HC_ADCH_EXT_10                 (10 << ADC_HC_ADCH_SHIFT)  /* External channels 10 */
#  define ADC_HC_ADCH_EXT_11                 (11 << ADC_HC_ADCH_SHIFT)  /* External channels 11 */
#  define ADC_HC_ADCH_EXT_12                 (12 << ADC_HC_ADCH_SHIFT)  /* External channels 12 */
#  define ADC_HC_ADCH_EXT_13                 (13 << ADC_HC_ADCH_SHIFT)  /* External channels 13 */
#  define ADC_HC_ADCH_EXT_14                 (14 << ADC_HC_ADCH_SHIFT)  /* External channels 14 */
#  define ADC_HC_ADCH_EXT_15                 (15 << ADC_HC_ADCH_SHIFT)  /* External channels 15 */
#  define ADC_HC_ADCH_EXT_ADC_ETC            (16 << ADC_HC_ADCH_SHIFT)  /* External channel selection from ADC_ETC */
#  define ADC_HC_ADCH_VREFSH                 (25 << ADC_HC_ADCH_SHIFT)  /* internal channel, for ADC self-test, hard connected to VRH internally */
#  define ADC_HC_ADCH_DIS                    (31 << ADC_HC_ADCH_SHIFT)  /* */

/*                                                         Bits: 5-6
 *                                                         Reserved
 */

#define ADC_HC_AIEN                          (1 << 7)   /* Bit: 7  Conversion Complete Interrupt Enable/Disable Control */
                                                        /* Bits: 8-31 Reserved */

/* Status register for HW triggers */

#define ADC_HS_COCO0                         (1 << 0)   /* Bit: 0  Conversion Complete Flag */
                                                        /* Bits: 1-31 Reserved */

/* Data result register for HW & SW triggers */

#define ADC_R_CDATA_SHIFT                    (0)        /* Bits: 0-11  Data (result of an ADC conversion) */
#define ADC_R_CDATA_MASK                     (0xfff << ADC_R_CDATA_SHIFT)

/* Configuration register */

#define ADC_CFG_ADICLK_SHIFT                 (0)                          /* Bits: 0-1  Input Clock Select */
#define ADC_CFG_ADICLK_MASK                  (3 << ADC_CFG_ADICLK_SHIFT)
#  define ADC_CFG_ADICLK(n)                  ((uint32_t)(n) << ADC_CFG_ADICLK_SHIFT)
#  define ADC_CFG_ADICLK_IPG                 (0 << ADC_CFG_ADICLK_SHIFT)  /* IPG clock */
#  define ADC_CFG_ADICLK_IPGDIV2             (1 << ADC_CFG_ADICLK_SHIFT)  /* IPG clock divided by 2 */
#  define ADC_CFG_ADICLK_ADACK               (3 << ADC_CFG_ADICLK_SHIFT)  /* Asynchronous clock (ADACK) */
#define ADC_CFG_MODE_SHIFT                   (2)                          /* Bits: 2-3  Conversion Mode Selection */
#define ADC_CFG_MODE_MASK                    (3 << ADC_CFG_MODE_SHIFT)
#  define ADC_CFG_MODE(n)                    ((uint32_t)(n) << ADC_CFG_MODE_SHIFT)
#  define ADC_CFG_MODE_8BIT                  (0 << ADC_CFG_MODE_SHIFT)    /* 8-bit conversion */
#  define ADC_CFG_MODE_10BIT                 (1 << ADC_CFG_MODE_SHIFT)    /* 10-bit conversion */
#  define ADC_CFG_MODE_12BIT                 (2 << ADC_CFG_MODE_SHIFT)    /* 12-bit conversion */
#define ADC_CFG_ADLSMP                       (1 << 4)                     /* Bit: 4  Long Sample Time Configuration */
#define ADC_CFG_ADIV_SHIFT                   (5)                          /* Bits: 5-6  Clock Divide Select */
#define ADC_CFG_ADIV_MASK                    (3 << ADC_CFG_ADIV_SHIFT)
#  define ADC_CFG_ADIV(n)                    ((uint32_t)(n) << ADC_CFG_ADIV_SHIFT)
#  define ADC_CFG_ADIV_DIV1                  (0 << ADC_CFG_ADIV_SHIFT)    /* Input clock */
#  define ADC_CFG_ADIV_DIV2                  (1 << ADC_CFG_ADIV_SHIFT)    /* Input clock / 2 */
#  define ADC_CFG_ADIV_DIV4                  (2 << ADC_CFG_ADIV_SHIFT)    /* Input clock / 4 */
#  define ADC_CFG_ADIV_DIV8                  (3 << ADC_CFG_ADIV_SHIFT)    /* Input clock / 8 */
#define ADC_CFG_ADLPC                        (1 << 7)                     /* Bit: 7  Low-Power Configuration */
#define ADC_CFG_ADSTS_SHIFT                  (8)                          /* Bits: 8-9  Defines the sample time duration. */
#define ADC_CFG_ADSTS_MASK                   (3 << ADC_CFG_ADSTS_SHIFT)
#  define ADC_CFG_ADSTS(n)                   ((uint32_t)(n) << ADC_CFG_ADSTS_SHIFT)
#  define ADC_CFG_ADSTS_3_13                 (0 << ADC_CFG_ADSTS_SHIFT)   /* Sample period (ADC clocks) = 3 if ADLSMP=0b, 13 if ADLSMP=1b  */
#  define ADC_CFG_ADSTS_5_17                 (1 << ADC_CFG_ADSTS_SHIFT)   /* Sample period (ADC clocks) = 5 if ADLSMP=0b, 17 if ADLSMP=1b  */
#  define ADC_CFG_ADSTS_7_21                 (2 << ADC_CFG_ADSTS_SHIFT)   /* Sample period (ADC clocks) = 7 if ADLSMP=0b, 21 if ADLSMP=1b  */
#  define ADC_CFG_ADSTS_9_25                 (3 << ADC_CFG_ADSTS_SHIFT)   /* Sample period (ADC clocks) = 9 if ADLSMP=0b, 25 if ADLSMP=1b  */
#define ADC_CFG_ADHSC                        (1 << 10)                    /* Bit: 10 High Speed Configuration*/
#define ADC_CFG_REFSEL_SHIFT                 (11)                         /* Bits: 11-12  Voltage Reference Selection */
#define ADC_CFG_REFSEL_MASK                  (3 << ADC_CFG_REFSEL_SHIFT)
#  define ADC_CFG_REFSEL(n)                  ((uint32_t)(n) << ADC_CFG_REFSEL_SHIFT)
#  define ADC_CFG_REFSEL_VREF                (0 << ADC_CFG_REFSEL_SHIFT)  /* Selects VREFH/VREFL as reference voltage. */
#define ADC_CFG_ADTRG                        (1 << 13)                    /* Bit: 13 Conversion Trigger Select */
#  define ADC_CFG_ADTRG_SW                   (0 << 13)                    /* SW trigger selected */
#  define ADC_CFG_ADTRG_HW                   (1 << 13)                    /* HW trigger selected */
#define ADC_CFG_AVGS_SHIFT                   (14)                         /* Bits: 14-15  Hardware Average select */
#define ADC_CFG_AVGS_MASK                    (3 << ADC_CFG_AVGS_SHIFT)
#  define ADC_CFG_AVGS(n)                    ((uint32_t)(n) << ADC_CFG_AVGS_SHIFT)
#  define ADC_CFG_AVGS_4SMPL                 (0 << ADC_CFG_AVGS_SHIFT)    /* 4 samples averaged */
#  define ADC_CFG_AVGS_8SMPL                 (1 << ADC_CFG_AVGS_SHIFT)    /* 8 samples averaged */
#  define ADC_CFG_AVGS_16SMPL                (2 << ADC_CFG_AVGS_SHIFT)    /* 16 samples averaged */
#  define ADC_CFG_AVGS_32SMPL                (3 << ADC_CFG_AVGS_SHIFT)    /* 32 samples averaged */
#define ADC_CFG_OVWREN                       (1 << 16)                    /* Bit: 16 Data Overwrite Enable */
                                                                          /* Bits: 17-31  Reserved */

/* General control register */

#define ADC_GC_ADACKEN                       (1 << 0)   /* Bit: 0  Asynchronous clock output enable */
#define ADC_GC_DMAEN                         (1 << 1)   /* Bit: 1  DMA Enable */
#define ADC_GC_ACREN                         (1 << 2)   /* Bit: 2  Compare Function Range Enable */
#define ADC_GC_ACFGT                         (1 << 3)   /* Bit: 3  Compare Function Greater Than Enable */
#define ADC_GC_ACFE                          (1 << 4)   /* Bit: 4  Compare Function Enable */
#define ADC_GC_AVGE                          (1 << 5)   /* Bit: 5  Hardware average enable */
#define ADC_GC_ADCO                          (1 << 6)   /* Bit: 6  Continuous Conversion Enable */
#define ADC_GC_CAL                           (1 << 7)   /* Bit: 7  Calibration */
                                                        /* Bits: 8-31  Reserved */

/* General status register */

#define ADC_GS_ADACT                         (1 << 0)   /* Bit: 0  Conversion Active */
#define ADC_GS_CALF                          (1 << 1)   /* Bit: 1  Calibration Failed Flag */
#define ADC_GS_AWKST                         (1 << 2)   /* Bit: 2  Asynchronous wakeup interrupt status */
                                                        /* Bits: 3-31  Reserved */

/* Compare value register */

#define ADC_CV_CV1_SHIFT                     (0)        /* Bits: 0-11  Compare Value 1 */
#define ADC_CV_CV1_MASK                      (0xfff << ADC_CV_CV1_SHIFT)
#  define ADC_CV_CV1(n)                      ((uint32_t)(n) << ADC_CV_CV1_SHIFT)
                                                        /* Bits: 12-15  Reserved */
#define ADC_CV_CV2_SHIFT                     (16)       /* Bits: 16-27  Compare Value 2 */
#define ADC_CV_CV2_MASK                      (0xfff << ADC_CV_CV2_SHIFT)
#  define ADC_CV_CV2(n)                      ((uint32_t)(n) << ADC_CV_CV2_SHIFT)
                                                        /* Bits: 28-31  Reserved */

/* Offset correction value register */

#define ADC_OFS_OFS_SHIFT                    (0)        /* Bits: 0-11  Offset value */
#define ADC_OFS_OFS_MASK                     (0xfff << ADC_OFS_OFS_SHIFT)
#  define ADC_OFS_OFS(n)                     ((uint32_t)(n) << ADC_OFS_OFS_SHIFT)
#define ADC_OFS_SIGN                         (1 << 12)  /* Bit: 12 Sign bit */
                                                        /* Bits: 13-31  Reserved */

/* Calibration value register */

#define ADC_CAL_CAL_CODE_SHIFT               (0)        /* Bits: 0-3  Calibration Result Value */
#define ADC_CAL_CAL_CODE_MASK                (0xf << ADC_CAL_CAL_CODE_SHIFT)
                                                        /* Bits: 4-31  Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER1_H */
