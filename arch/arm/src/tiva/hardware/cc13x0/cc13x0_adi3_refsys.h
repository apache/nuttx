/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_adi3_refsys.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a
 * compatible BSD license:
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI3_REFSYS_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI3_REFSYS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"
#include "hardware/tiva_ddi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADI3 REFSYS Register Offsets *********************************************/

#define TIVA_ADI3_REFSYS_SPARE0_OFFSET                        0x0001  /* Analog Test Control */
#define TIVA_ADI3_REFSYS_REFSYSCTL0_OFFSET                    0x0002
#define TIVA_ADI3_REFSYS_REFSYSCTL1_OFFSET                    0x0003
#define TIVA_ADI3_REFSYS_REFSYSCTL2_OFFSET                    0x0004
#define TIVA_ADI3_REFSYS_REFSYSCTL3_OFFSET                    0x0005
#define TIVA_ADI3_REFSYS_DCDCCTL0_OFFSET                      0x0006  /* DCDC Control 0 */
#define TIVA_ADI3_REFSYS_DCDCCTL1_OFFSET                      0x0007  /* DCDC Control 1 */
#define TIVA_ADI3_REFSYS_DCDCCTL2_OFFSET                      0x0008  /* DCDC Control 2 */
#define TIVA_ADI3_REFSYS_DCDCCTL3_OFFSET                      0x0009  /* DCDC Control 3 */
#define TIVA_ADI3_REFSYS_DCDCCTL4_OFFSET                      0x000a
#define TIVA_ADI3_REFSYS_DCDCCTL5_OFFSET                      0x000b

/* ADI3 REFSYS Register Addresses *******************************************/

#define TIVA_ADI3_REFSYS_SPARE0                               (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_SPARE0_OFFSET)
#define TIVA_ADI3_REFSYS_REFSYSCTL0                           (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_REFSYSCTL0_OFFSET)
#define TIVA_ADI3_REFSYS_REFSYSCTL1                           (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_REFSYSCTL1_OFFSET)
#define TIVA_ADI3_REFSYS_REFSYSCTL2                           (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_REFSYSCTL2_OFFSET)
#define TIVA_ADI3_REFSYS_REFSYSCTL3                           (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_REFSYSCTL3_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL0                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL0_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL1                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL1_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL2                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL2_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL3                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL3_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL4                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL4_OFFSET)
#define TIVA_ADI3_REFSYS_DCDCCTL5                             (TIVA_ADI3_BASE + TIVA_ADI3_REFSYS_DCDCCTL5_OFFSET)

/* Offsets may also be used in conjunction with access as described in
 * cc13x0_ddi.h
 */

#define TIVA_ADI3_REFSYS_DIR                                  (TIVA_ADI3_BASE + TIVA_DDI_DIR_OFFSET)
#define TIVA_ADI3_REFSYS_SET                                  (TIVA_ADI3_BASE + TIVA_DDI_SET_OFFSET)
#define TIVA_ADI3_REFSYS_CLR                                  (TIVA_ADI3_BASE + TIVA_DDI_CLR_OFFSET)
#define TIVA_ADI3_REFSYS_MASK4B                               (TIVA_ADI3_BASE + TIVA_DDI_MASK4B_OFFSET)
#define TIVA_ADI3_REFSYS_MASK8B                               (TIVA_ADI3_BASE + TIVA_DDI_MASK8B_OFFSET)
#define TIVA_ADI3_REFSYS_MASK16B                              (TIVA_ADI3_BASE + TIVA_DDI_MASK16B_OFFSET)

/* ADI3 REFSYS Bitfield Definitions *****************************************/

/* TIVA_ADI3_REFSYS_SPARE0 */

#define ADI3_REFSYS_SPARE0_SPARE0_SHIFT                       (0)       /* Bits 0-7:  Do not change */
#define ADI3_REFSYS_SPARE0_SPARE0_MASK                        (0xff << ADI3_REFSYS_SPARE0_SPARE0_SHIFT)
#  define ADI3_REFSYS_SPARE0_SPARE0(n)                        ((uint32_t)(n) << ADI3_REFSYS_SPARE0_SPARE0_SHIFT)

/* TIVA_ADI3_REFSYS_REFSYSCTL0 */

#define ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT                  (0)       /* Bits 0-7 */
#define ADI3_REFSYS_REFSYSCTL0_TESTCTL_MASK                   (0xff << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL(n)                   ((uint32_t)(n) << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_NC                   (0x00 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_IPTAT2U              (0x01 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_IVREF4U              (0x02 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_IREF4U               (0x04 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_VBG                  (0x08 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_VBGUNBUFF            (0x10 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_VREF0P8V             (0x20 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_VTEMP                (0x40 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL0_TESTCTL_BMCOMPOUT            (0x80 << ADI3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT)

/* TIVA_ADI3_REFSYS_REFSYSCTL1 */

#define ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT                  (0)       /* Bits 0-1 */
#define ADI3_REFSYS_REFSYSCTL1_TESTCTL_MASK                   (3 << ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TESTCTL(n)                   ((uint32_t)(n) << ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TESTCTL_NC                   (0 << ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TESTCTL_BMCOMPIN             (1 << ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TESTCTL_IPTAT1U              (2 << ADI3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT)
#define ADI3_REFSYS_REFSYSCTL1_BATMON_COMP_TEST_EN            (1 << 2)  /* Bit 2 */
#define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT            (3)       /* Bits 3-7 */
#define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_MASK             (31 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD(n)             ((uint32_t)(n) << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_4          (0 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_5          (1 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_6          (2 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_7          (3 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_0          (4 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_1          (5 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_2          (6 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_3          (7 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_12         (8 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_13         (9 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_14         (10 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_15         (11 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_8          (12 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_9          (13 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_10         (14 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_11         (15 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_20         (16 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_21         (17 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_22         (18 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_23         (19 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_16         (20 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_17         (21 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_18         (22 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_19         (23 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_28         (24 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_29         (25 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_30         (26 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_31         (27 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_24         (28 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_25         (29 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_26         (30 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_POS_27         (31 << ADI3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT)

/* TIVA_ADI3_REFSYS_REFSYSCTL2 */

#define ADI3_REFSYS_REFSYSCTL2_TRIM_TSENSE_SHIFT              (0)       /* Bits 0-1 */
#define ADI3_REFSYS_REFSYSCTL2_TRIM_TSENSE_MASK               (3 << ADI3_REFSYS_REFSYSCTL2_TRIM_TSENSE_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL2_TRIM_TSENSE(n)               ((uint32_t)(n) << ADI3_REFSYS_REFSYSCTL2_TRIM_TSENSE_SHIFT)

/* TIVA_ADI3_REFSYS_REFSYSCTL3 */

#define ADI3_REFSYS_REFSYSCTL3_TRIM_VBG_SHIFT                 (0)       /* Bits 0-5 */
#define ADI3_REFSYS_REFSYSCTL3_TRIM_VBG_MASK                  (0x3f << ADI3_REFSYS_REFSYSCTL3_TRIM_VBG_SHIFT)
#  define ADI3_REFSYS_REFSYSCTL3_TRIM_VBG(n)                  ((uint32_t)(n) << ADI3_REFSYS_REFSYSCTL3_TRIM_VBG_SHIFT)
#define ADI3_REFSYS_REFSYSCTL3_VTEMP_EN                       (1 << 6)  /* Bit 6 */
#define ADI3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN                 (1 << 7)  /* Bit 7 */

/* TIVA_ADI3_REFSYS_DCDCCTL0 */

#define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT                  (0)       /* Bits 0-4: Set the VDDR voltage */
                                                                        /*           Proprietary encoding */
#define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MASK                   (31 << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT)
#  define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM(n)                   ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT)
#  define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_DEFAULT              (0 << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT)  /* Default, about 1.63V */
#  define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_TYPICAL              (5 << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT)  /* Typical voltage after trim voltage 1.71V */
#  define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MAX                  (21 << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT) /* Max voltage 1.96V */
#  define ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_MIN                  (22 << ADI3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT) /* Min voltage 1.47V */

#define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT                  (5)       /* Bits 5-7:  Set charge and re-charge current level */
                                                                        /*            2's complement encoding */
#define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_MASK                   (7 << ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT)
#  define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC(n)                   ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT)
#  define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_DEFAULT              (0 << ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT) /* Default 11mA */
#  define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_MAX                  (3 << ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT) /* Max 15mA */
#  define ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_MIN                  (4 << ADI3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT) /* Max 5mA */

/* TIVA_ADI3_REFSYS_DCDCCTL1 */

#define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT            (0)       /* Bits 0-4: Set the min VDDR voltage threshold during sleep mode */
                                                                        /*           Proprietary encoding */
#define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_MASK             (31 << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT)
#  define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP(n)             ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT)
#  define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_DEFAULT        (nn << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT) /* Default, about 1.63V */
#  define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_TYPICAL        (nn << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT) /* Typical voltage after trim voltage 1.52V */
#  define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_MAX            (nn << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT) /* Max voltage 1.96V */
#  define ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_MIN            (nn << ADI3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT) /* Min voltage 1.47V */

#define ADI3_REFSYS_DCDCCTL1_VDDR_OK_HYST                     (1 << 5)  /* Bit 5:  Increase the hysteresis for when VDDR is considered ok */
                                                                        /*         0: Hysteresis = 60mV; 1: Hysteresis = 70mV */
#define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT                 (6)       /* Bits 6-7: Trim GLDO bias current */
                                                                        /*           Proprietary encoding */
#define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_MASK                  (3 << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT)
#  define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM(n)                  ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT)
#  define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_DEFAULT             (0 << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT) /* Default */
#  define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_INC1p3              (1 << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT) /* Increase GLDO bias by 1.3x */
#  define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_INC1p6              (2 << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT) /* Increase GLDO bias by 1.6x */
#  define ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_DEC0p7              (3 << ADI3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT) /* Decrease GLDO bias by 0.7x */

/* TIVA_ADI3_REFSYS_DCDCCTL2 */

#define ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT                    (0)       /* Bits 0-3: Select signal for test bus, one hot */
#define ADI3_REFSYS_DCDCCTL2_TESTSEL_MASK                     (15 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT)
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL(n)                     ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT)
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL_NC                     (0 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT) /* No signal connected to test bus */
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL_ERRAMP_OUT             (1 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT) /* Error amp output voltage connected to test bus */
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL_PASSGATE               (2 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT) /* Pass transistor gate voltage connected to test bus */
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL_IB1U                   (4 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT) /* 1uA bias current connected to test bus */
#  define ADI3_REFSYS_DCDCCTL2_TESTSEL_VDDROK                 (8 << ADI3_REFSYS_DCDCCTL2_TESTSEL_SHIFT) /* VDDR_OK connected to test bus */

#define ADI3_REFSYS_DCDCCTL2_BIAS_DIS                         (1 << 4)  /* Bit 4:  Disable dummy bias current */
#define ADI3_REFSYS_DCDCCTL2_TEST_VDDR                        (1 << 5)  /* Bit 5:  Connect VDDR to ATEST bus */
#define ADI3_REFSYS_DCDCCTL2_TURNON_EA_SW                     (1 << 6)  /* Bit 6: Turns on GLDO error amp switch */

/* TIVA_ADI3_REFSYS_DCDCCTL3 */

/* TIVA_ADI3_REFSYS_DCDCCTL4 */

#define ADI3_REFSYS_DCDCCTL4_HIGH_EN_SEL_SHIFT                (0)       /* Bits 0-2 */
#define ADI3_REFSYS_DCDCCTL4_HIGH_EN_SEL_MASK                 (7 << ADI3_REFSYS_DCDCCTL4_HIGH_EN_SEL_SHIFT)
#  define ADI3_REFSYS_DCDCCTL4_HIGH_EN_SEL(n)                 ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL4_HIGH_EN_SEL_SHIFT)
#define ADI3_REFSYS_DCDCCTL4_LOW_EN_SEL_SHIFT                 (3)       /* Bits 3-5 */
#define ADI3_REFSYS_DCDCCTL4_LOW_EN_SEL_MASK                  (7 << ADI3_REFSYS_DCDCCTL4_LOW_EN_SEL_SHIFT)
#  define ADI3_REFSYS_DCDCCTL4_LOW_EN_SEL(n)                  ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL4_LOW_EN_SEL_SHIFT)
#define ADI3_REFSYS_DCDCCTL4_DEADTIME_TRIM_SHIFT              (6)       /* Bits 6-7 */
#define ADI3_REFSYS_DCDCCTL4_DEADTIME_TRIM_MASK               (3 << ADI3_REFSYS_DCDCCTL4_DEADTIME_TRIM_SHIFT)
#  define ADI3_REFSYS_DCDCCTL4_DEADTIME_TRIM(n)               ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL4_DEADTIME_TRIM_SHIFT)

/* TIVA_ADI3_REFSYS_DCDCCTL5 */

#define ADI3_REFSYS_DCDCCTL5_IPEAK_SHIFT                      (0)       /* Bits 0-2 */
#define ADI3_REFSYS_DCDCCTL5_IPEAK_MASK                       (7 << ADI3_REFSYS_DCDCCTL5_IPEAK_SHIFT)
#  define ADI3_REFSYS_DCDCCTL5_IPEAK(n)                       ((uint32_t)(n) << ADI3_REFSYS_DCDCCTL5_IPEAK_SHIFT)
#define ADI3_REFSYS_DCDCCTL5_DITHER_EN                        (1 << 3)  /* Bit 3 */
#define ADI3_REFSYS_DCDCCTL5_TESTP                            (1 << 4)  /* Bit 4 */
#define ADI3_REFSYS_DCDCCTL5_TESTN                            (1 << 5)  /* Bit 5 */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_ADI3_REFSYS_H */
