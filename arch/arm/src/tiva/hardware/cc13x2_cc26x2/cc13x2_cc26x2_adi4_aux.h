/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_adi4_aux.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible
 * BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_ADI4_AUX_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_ADI4_AUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"
#include "hardware/tiva_ddi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADI3 AUX Register Offsets ************************************************/

#define TIVA_ADI4_AUX_MUX0_OFFSET                          0x0000
#define TIVA_ADI4_AUX_MUX1_OFFSET                          0x0001
#define TIVA_ADI4_AUX_MUX2_OFFSET                          0x0002
#define TIVA_ADI4_AUX_MUX3_OFFSET                          0x0003
#define TIVA_ADI4_AUX_ISRC_OFFSET                          0x0004  /* Current Source */
#define TIVA_ADI4_AUX_COMP_OFFSET                          0x0005  /* Comparator */
#define TIVA_ADI4_AUX_MUX4_OFFSET                          0x0007
#define TIVA_ADI4_AUX_ADC0_OFFSET                          0x0008  /* ADC Control 0 */
#define TIVA_ADI4_AUX_ADC1_OFFSET                          0x0009  /* ADC Control 1 */
#define TIVA_ADI4_AUX_ADCREF0_OFFSET                       0x000a  /* ADC Reference 0 */
#define TIVA_ADI4_AUX_ADCREF1_OFFSET                       0x000b  /* ADC Reference 1 */
#define TIVA_ADI4_AUX_LPMBIAS_OFFSET                       0x000e

/* ADI3 AUX Register Addresses **********************************************/

#define TIVA_ADI4_AUX_MUX0                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_MUX0_OFFSET)
#define TIVA_ADI4_AUX_MUX1                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_MUX1_OFFSET)
#define TIVA_ADI4_AUX_MUX2                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_MUX2_OFFSET)
#define TIVA_ADI4_AUX_MUX3                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_MUX3_OFFSET)
#define TIVA_ADI4_AUX_ISRC                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_ISRC_OFFSET)
#define TIVA_ADI4_AUX_COMP                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_COMP_OFFSET)
#define TIVA_ADI4_AUX_MUX4                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_MUX4_OFFSET)
#define TIVA_ADI4_AUX_ADC0                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_ADC0_OFFSET)
#define TIVA_ADI4_AUX_ADC1                                 (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_ADC1_OFFSET)
#define TIVA_ADI4_AUX_ADCREF0                              (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_ADCREF0_OFFSET)
#define TIVA_ADI4_AUX_ADCREF1                              (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_ADCREF1_OFFSET)
#define TIVA_ADI4_AUX_LPMBIAS                              (TIVA_AUX_ADI4_BASE + TIVA_ADI4_AUX_LPMBIAS_OFFSET)

/* Offsets may also be used in conjunction with access as described in
 *cc13x2_cc26x2_ddi.h
 */

#define TIVA_ADI4_AUX_DIR                                  (TIVA_AUX_ADI4_BASE + TIVA_DDI_DIR_OFFSET)
#define TIVA_ADI4_AUX_SET                                  (TIVA_AUX_ADI4_BASE + TIVA_DDI_SET_OFFSET)
#define TIVA_ADI4_AUX_CLR                                  (TIVA_AUX_ADI4_BASE + TIVA_DDI_CLR_OFFSET)
#define TIVA_ADI4_AUX_MASK4B                               (TIVA_AUX_ADI4_BASE + TIVA_DDI_MASK4B_OFFSET)
#define TIVA_ADI4_AUX_MASK8B                               (TIVA_AUX_ADI4_BASE + TIVA_DDI_MASK8B_OFFSET)
#define TIVA_ADI4_AUX_MASK16B                              (TIVA_AUX_ADI4_BASE + TIVA_DDI_MASK16B_OFFSET)

/* ADI3 AUX Register Bitfield Definitions ***********************************/

/* TIVA_ADI4_AUX_MUX0 */

#define ADI4_AUX_MUX0_COMPA_REF_SHIFT                      (0)       /* Bits 0-3 */
#define ADI4_AUX_MUX0_COMPA_REF_MASK                       (15 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF(n)                       ((uint32_t)(n) << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF_NC                       (0 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF_DCOUPL                   (1 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF_VSS                      (2 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF_VDDS                     (4 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX0_COMPA_REF_ADCVREFP                 (8 << ADI4_AUX_MUX0_COMPA_REF_SHIFT)
#define ADI4_AUX_MUX0_ADCCOMPB_IN                          (1 << 6)  /* Bit 6 */;
#  define ADI4_AUX_MUX0_ADCCOMPB_IN_VDDR_1P8V              ADI4_AUX_MUX0_ADCCOMPB_IN
#  define ADI4_AUX_MUX0_ADCCOMPB_IN_NC                     (0)

/* TIVA_ADI4_AUX_MUX1 */

#define ADI4_AUX_MUX1_COMPA_IN_SHIFT                       (0)       /* Bit 0-7 */
#define ADI4_AUX_MUX1_COMPA_IN_MASK                        (0xff << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN(n)                        ((uint32_t)(n) << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_NC                        (0   << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO26                   (1   << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO25                   (2   << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO24                   (4   << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO23                   (8   << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO22                   (16  << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO21                   (32  << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO20                   (64  << ADI4_AUX_MUX1_COMPA_IN_SHIFT)
#  define ADI4_AUX_MUX1_COMPA_IN_AUXIO19                   (128 << ADI4_AUX_MUX1_COMPA_IN_SHIFT)

/* TIVA_ADI4_AUX_MUX2 */

#define ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT                   (0)       /* Bits 0-2 */
#define ADI4_AUX_MUX2_DAC_VREF_SEL_MASK                    (7 << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#  define ADI4_AUX_MUX2_DAC_VREF_SEL(n)                    ((uint32_t)(n) << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#  define ADI4_AUX_MUX2_DAC_VREF_SEL_NC                    (0 << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#  define ADI4_AUX_MUX2_DAC_VREF_SEL_DCOUPL                (1 << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#  define ADI4_AUX_MUX2_DAC_VREF_SEL_ADCREF                (2 << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#  define ADI4_AUX_MUX2_DAC_VREF_SEL_VDDS                  (4 << ADI4_AUX_MUX2_DAC_VREF_SEL_SHIFT)
#define ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT                    (3)       /* Bits 3-7 */
#define ADI4_AUX_MUX2_ADCCOMPB_IN_MASK                     (31 << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN(n)                     ((uint32_t)(n) << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_NC                     (0  << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_ATEST0                 (1  << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_ATEST1                 (2  << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_DCOUPL                 (4  << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_VSS                    (8  << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX2_ADCCOMPB_IN_VDDS                   (16 << ADI4_AUX_MUX2_ADCCOMPB_IN_SHIFT)

/* TIVA_ADI4_AUX_MUX3 */

#define ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT                     (0)       /* Bits 0-7 */
#define ADI4_AUX_MUX3_ADCCOMPB_IN_MASK                      (0xff << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN(n)                      ((uint32_t)(n) << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_NC                      (0   << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO26                 (1   << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO25                 (2   << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO24                 (4   << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO23                 (8   << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO22                 (16  << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO21                 (32  << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO20                 (64  << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)
#  define ADI4_AUX_MUX3_ADCCOMPB_IN_AUXIO19                 (128 << ADI4_AUX_MUX3_ADCCOMPB_IN_SHIFT)

/* TIVA_ADI4_AUX_ISRC */

#define ADI4_AUX_ISRC_EN                                    (1 << 0)  /* Bit 0:  Current source enable */
#define ADI4_AUX_ISRC_TRIM_SHIFT                            (2)       /* Bits 2-7: Adjust current from current source */
#define ADI4_AUX_ISRC_TRIM_MASK                             (0x3f << ADI4_AUX_ISRC_TRIM_SHIFT)
#  define ADI4_AUX_ISRC_TRIM(n)                             ((uint32_t)(n) << ADI4_AUX_ISRC_TRIM_SHIFT)
#  define ADI4_AUX_ISRC_TRIM_NC                             (0  << ADI4_AUX_ISRC_TRIM_SHIFT) /* No current connected */
#  define ADI4_AUX_ISRC_TRIM_0p25U                          (1  << ADI4_AUX_ISRC_TRIM_SHIFT) /* 0.25 uA */
#  define ADI4_AUX_ISRC_TRIM_0p5U                           (2  << ADI4_AUX_ISRC_TRIM_SHIFT) /* 0.5 uA */
#  define ADI4_AUX_ISRC_TRIM_1p0U                           (4  << ADI4_AUX_ISRC_TRIM_SHIFT) /* 1.0 uA */
#  define ADI4_AUX_ISRC_TRIM_2p0U                           (8  << ADI4_AUX_ISRC_TRIM_SHIFT) /* 2.0 uA */
#  define ADI4_AUX_ISRC_TRIM_4p5U                           (16 << ADI4_AUX_ISRC_TRIM_SHIFT) /* 4.5 uA */
#  define ADI4_AUX_ISRC_TRIM_11p75U                         (32 << ADI4_AUX_ISRC_TRIM_SHIFT) /* 11.75 uA */

/* TIVA_ADI4_AUX_COMP */

#define ADI4_AUX_COMP_COMPA_EN                              (1 << 1)  /* Bit 1:  COMPA enable */
#define ADI4_AUX_COMP_COMPB_EN                              (1 << 2)  /* Bit 2:  COMPB enable */
#define ADI4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_SHIFT             (3)       /* Bits 3-5 */
#define ADI4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_MASK              (7 << ADI4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_SHIFT)
#  define ADI4_AUX_COMP_LPM_BIAS_WIDTH_TRIM(n)              ((uint32_t)(n) << ADI4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_SHIFT)
#define ADI4_AUX_COMP_COMPA_REF_CURR_EN                     (1 << 6)  /* Bit 6:  Enables 2uA IPTAT current from ISRC to COMPA reference */
#define ADI4_AUX_COMP_COMPA_REF_RES_EN                      (1 << 7)  /* Bit 7:  Enables 400kohm resistance from COMPA reference */

/* TIVA_ADI4_AUX_MUX4 */

#define ADI4_AUX_MUX4_COMPA_REF_SHIFT                       (0)       /* Bits 0-7 */
#define ADI4_AUX_MUX4_COMPA_REF_MASK                        (0xff << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF(n)                        ((uint32_t)(n) << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_NC                        (0   << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO26                   (1   << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO25                   (2   << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO24                   (4   << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO23                   (8   << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO22                   (16  << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO21                   (32  << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO20                   (64  << ADI4_AUX_MUX4_COMPA_REF_SHIFT)
#  define ADI4_AUX_MUX4_COMPA_REF_AUXIO19                   (128 << ADI4_AUX_MUX4_COMPA_REF_SHIFT)

/* TIVA_ADI4_AUX_ADC0 */

#define ADI4_AUX_ADC0_EN                                    (1 << 0)                                   /* Bit 0:  ADC Enable */
#define ADI4_AUX_ADC0_RESET_N                               (1 << 1)                                   /* Bit 1:  Reset ADC digital subchip, active low */
#define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT                  (3)                                        /* Bits 3-6: Controls the sampling duration
                                                                                                        * before conversion when the ADC is operated
                                                                                                        * in synchronous mode (SMPL_MODE = 0) */
#define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_MASK                   (15 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT)
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP(n)                   ((uint32_t)(n) << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT)
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_2p7_US               (3  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 16 clocks = 2.7us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_5p3_US               (4  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 32 clocks = 5.3us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_10p6_US              (5  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 64 clocks = 10.6us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_21p3_US              (6  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 128 clocks = 21.3us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_42p6_US              (7  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 256 clocks = 42.6us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_85p3_US              (8  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 512 clocks = 85.3us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_170_US               (9  << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 1024 clocks = 170us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_341_US               (10 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 2048 clocks = 341us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_682_US               (11 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 4096 clocks = 682us */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_1p37_MS              (12 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 8192 clocks = 1.37ms */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_2p73_MS              (13 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 16384 clocks = 2.73ms */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_5p46_MS              (14 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 32768 clocks = 5.46ms */
#  define ADI4_AUX_ADC0_SMPL_CYCLE_EXP_10p9_MS              (15 << ADI4_AUX_ADC0_SMPL_CYCLE_EXP_SHIFT) /* 65536 clocks = 10.9ms */
#define ADI4_AUX_ADC0_SMPL_MODE                             (1 << 7)                                   /* Bit 7:  ADC Sampling mode */
#  define ADI4_AUX_ADC0_SMPL_MODE_SYNCH                     (0)
#  define ADI4_AUX_ADC0_SMPL_MODE_ASYNCH                    ADI4_AUX_ADC0_SMPL_MODE

/* TIVA_ADI4_AUX_ADC1 */

#define ADI4_AUX_ADC1_SCALE_DIS                             (1 << 0)  /* Bit 0 */

/* TIVA_ADI4_AUX_ADCREF0 */

#define ADI4_AUX_ADCREF0_EN                                 (1 << 0)  /* Bit 0:  ADC reference module enable */
#define ADI4_AUX_ADCREF0_SRC                                (1 << 3)  /* Bit 3:  ADC reference source */
#  define ADI4_AUX_ADCREF0_SRC_FIXED                        0
#  define ADI4_AUX_ADCREF0_SRC_RELATIVE                     ADI4_AUX_ADCREF0_SRC
#define ADI4_AUX_ADCREF0_EXT                                (1 << 4)  /* Bit 4 */
#define ADI4_AUX_ADCREF0_IOMUX                              (1 << 5)  /* Bit 5 */
#define ADI4_AUX_ADCREF0_REF_ON_IDLE                        (1 << 6)  /* Bit 6:  Enable ADCREF in IDLE state */

/* TIVA_ADI4_AUX_ADCREF1 */

#define ADI4_AUX_ADCREF1_VTRIM_SHIFT                        (0)       /* Bits 0-5: Trim output voltage of ADC fixed
                                                                       * reference (64 steps, 2's complement) */
#define ADI4_AUX_ADCREF1_VTRIM_MASK                         (0x3f << ADI4_AUX_ADCREF1_VTRIM_SHIFT)
#  define ADI4_AUX_ADCREF1_VTRIM(n)                         ((uint32_t)(n) << ADI4_AUX_ADCREF1_VTRIM_SHIFT)
#  define ADI4_AUX_ADCREF1_VTRIM_NOMINAL                    (0x00 << ADI4_AUX_ADCREF1_VTRIM_SHIFT) /* Nominal voltage 1.43V */
#  define ADI4_AUX_ADCREF1_VTRIM_MAX                        (0x1f << ADI4_AUX_ADCREF1_VTRIM_SHIFT) /* Maximum voltage 1.6V */
#  define ADI4_AUX_ADCREF1_VTRIM_MIN                        (0x20 << ADI4_AUX_ADCREF1_VTRIM_SHIFT) /* Minimum voltage 1.3V */

/* TIVA_ADI4_AUX_LPMBIAS */

#define ADI4_AUX_LPMBIAS_LPM_TRIM_IOUT_SHIFT                (0)       /* Bits 0-5 */
#define ADI4_AUX_LPMBIAS_LPM_TRIM_IOUT_MASK                 (0x3f << ADI4_AUX_LPMBIAS_LPM_TRIM_IOUT_SHIFT)
#  define ADI4_AUX_LPMBIAS_LPM_TRIM_IOUT(n)                 ((uint32_t)(n) << ADI4_AUX_LPMBIAS_LPM_TRIM_IOUT_SHIFT)

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_ADI4_AUX_H */
