/****************************************************************************
 * arch/arm/src/sama5/hardware/_sama5d4x_mpddrc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D4X_MPDDRC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D4X_MPDDRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPDDRC Register Offsets **************************************************/

#define SAM_MPDDRC_MR_OFFSET                 0x0000 /* MPDDRC Mode Register */
#define SAM_MPDDRC_RTR_OFFSET                0x0004 /* MPDDRC Refresh Timer Register */
#define SAM_MPDDRC_CR_OFFSET                 0x0008 /* MPDDRC Configuration Register */
#define SAM_MPDDRC_TPR0_OFFSET               0x000c /* MPDDRC Timing Parameter 0 Register */
#define SAM_MPDDRC_TPR1_OFFSET               0x0010 /* MPDDRC Timing Parameter 1 Register */
#define SAM_MPDDRC_TPR2_OFFSET               0x0014 /* MPDDRC Timing Parameter 2 Register */
                                                    /* 0x0018 Reserved */
#define SAM_MPDDRC_LPR_OFFSET                0x001c /* MPDDRC Low-power Register */
#define SAM_MPDDRC_MD_OFFSET                 0x0020 /* MPDDRC Memory Device Register */
                                                    /* 0x0024 Reserved */
#define SAM_MPDDRC_LPDDR2_LPR_OFFSET         0x0028 /* MPDDRC LPDDR2 Low-power Register */
#define SAM_MPDDRC_LPDDR2_CALMR4_OFFSET      0x002c /* MPDDRC LPDDR2 Calibration and MR4 Register */
#define SAM_MPDDRC_LPDDR2_TIMCAL_OFFSET      0x0030 /* MPDDRC LPDDR2 Timing Calibration Register */
#define SAM_MPDDRC_IO_CALIBR_OFFSET          0x0034 /* MPDDRC IO Calibration */
#define SAM_MPDDRC_OCMS_OFFSET               0x0038 /* MPDDRC OCMS Register */
#define SAM_MPDDRC_OCMS_KEY1_OFFSET          0x003c /* MPDDRC OCMS KEY1 Register */
#define SAM_MPDDRC_OCMS_KEY2_OFFSET          0x0040 /* MPDDRC OCMS KEY2 Register */
#define SAM_MPDDRC_CONF_ARBITER_OFFSET       0x0044 /* MPDDRC Configuration Arbiter Register */
#define SAM_MPDDRC_TIMEOUT_OFFSET            0x0048 /* MPDDRC Time-out Port 0/1/2/3 Register */
#define SAM_MPDDRC_REQ_PORT_0123_OFFSET      0x004c /* MPDDRC Request Port 0/1/2/3 Register */
#define SAM_MPDDRC_REQ_PORT_4567_OFFSET      0x0050 /* MPDDRC Request Port 4/5/6/7 Register */
#define SAM_MPDDRC_BDW_PORT_0123_OFFSET      0x0054 /* MPDDRC Bandwidth Port 0/1/2/3 Register */
#define SAM_MPDDRC_BDW_PORT_4567_OFFSET      0x0058 /* MPDDRC Bandwidth Port 4/5/6/7 Register */
#define SAM_MPDDRC_RD_DATA_PATH_OFFSET       0x005c /* MPDDRC Read Datapath Register */

#define SAM_MPPDRC_SAW_OFFSET(n)             (0x0060+((n) << 2))
#define SAM_MPDDRC_SAW0_OFFSET               0x0060 /* MPDDRC Smart Adaptation Wrapper 0 Register */
#define SAM_MPDDRC_SAW1_OFFSET               0x0064 /* MPDDRC Smart Adaptation Wrapper 1 Register */
#define SAM_MPDDRC_SAW2_OFFSET               0x0068 /* MPDDRC Smart Adaptation Wrapper 2 Register */
#define SAM_MPDDRC_SAW3_OFFSET               0x006c /* MPDDRC Smart Adaptation Wrapper 3 Register */
                                                    /* 0x0060-0x00e0 Reserved */
#define SAM_MPDDRC_WPCR_OFFSET               0x00e4 /* MPDDRC Write Protect Control Register */
#define SAM_MPDDRC_WPSR_OFFSET               0x00e8 /* MPDDRC Write Protect Status Register */

#define SAM_MPDDRC_DLL_OS_OFFSET             0x0100 /* MPDDRC DLL Offset Selection Register */
#define SAM_MPDDRC_DLL_MO_OFFSET             0x0104 /* MPDDRC DLL MASTER Offset Register */
#define SAM_MPDDRC_DLL_SO0_OFFSET            0x0108 /* MPDDRC DLL SLAVE Offset 0 Register */
#define SAM_MPDDRC_DLL_SO1_OFFSET            0x010c /* MPDDRC DLL SLAVE Offset 1 Register */
#define SAM_MPDDRC_DLL_WRO_OFFSET            0x0110 /* MPDDRC DLL CLKWR Offset Register */
#define SAM_MPDDRC_DLL_ADO_OFFSET            0x0114 /* MPDDRC DLL CLKAD Offset Register */

#define SAM_MPDDRC_DLL_SM_OFFSET(n)          (0x0118+((n)<<2))
#define SAM_MPDDRC_DLL_SM0_OFFSET            0x0118 /* MPDDRC DLL Status MASTER0 Register */
#define SAM_MPDDRC_DLL_SM1_OFFSET            0x011c /* MPDDRC DLL Status MASTER1 Register */
#define SAM_MPDDRC_DLL_SM2_OFFSET            0x0120 /* MPDDRC DLL Status MASTER2 Register */
#define SAM_MPDDRC_DLL_SM3_OFFSET            0x0124 /* MPDDRC DLL Status MASTER3 Register */

#define SAM_MPDDRC_DLL_SSL_OFFSET(n)         (0x0128+((n)<<2))
#define SAM_MPDDRC_DLL_SSL0_OFFSET           0x0128 /* MPDDRC DLL Status SLAVE0 Register */
#define SAM_MPDDRC_DLL_SSL1_OFFSET           0x012c /* MPDDRC DLL Status SLAVE1 Register */
#define SAM_MPDDRC_DLL_SSL2_OFFSET           0x0130 /* MPDDRC DLL Status SLAVE2 Register */
#define SAM_MPDDRC_DLL_SSL3_OFFSET           0x0134 /* MPDDRC DLL Status SLAVE3 Register */
#define SAM_MPDDRC_DLL_SSL4_OFFSET           0x0138 /* MPDDRC DLL Status SLAVE4 Register */
#define SAM_MPDDRC_DLL_SSL5_OFFSET           0x013c /* MPDDRC DLL Status SLAVE5 Register */
#define SAM_MPDDRC_DLL_SSL6_OFFSET           0x0140 /* MPDDRC DLL Status SLAVE6 Register */
#define SAM_MPDDRC_DLL_SSL7_OFFSET           0x0144 /* MPDDRC DLL Status SLAVE7 Register */

#define SAM_MPDDRC_DLL_SWR_OFFSET(n)         (0x0148+((n)<<2))
#define SAM_MPDDRC_DLL_SWR0_OFFSET           0x0148 /* MPDDRC DLL Status CLKWR0 Register */
#define SAM_MPDDRC_DLL_SWR1_OFFSET           0x014c /* MPDDRC DLL Status CLKWR1 Register */
#define SAM_MPDDRC_DLL_SWR2_OFFSET           0x0150 /* MPDDRC DLL StatusCLKWR2 Register */
#define SAM_MPDDRC_DLL_SWR3_OFFSET           0x0154 /* MPDDRC DLL Status CLKWR3 Register */

#define SAM_MPDDRC_DLL_SAD_OFFSET            0x0158 /* MPDDRC DLL Status CLKAD Register */
                                                    /* 0x015c-0x01fxc Reserved */

/* MPDDRC Register Addresses ************************************************/

#define SAM_MPDDRC_MR                        (SAM_MPDDRC_VBASE+SAM_MPDDRC_MR_OFFSET)
#define SAM_MPDDRC_RTR                       (SAM_MPDDRC_VBASE+SAM_MPDDRC_RTR_OFFSET)
#define SAM_MPDDRC_CR                        (SAM_MPDDRC_VBASE+SAM_MPDDRC_CR_OFFSET)
#define SAM_MPDDRC_TPR0                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR0_OFFSET)
#define SAM_MPDDRC_TPR1                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR1_OFFSET)
#define SAM_MPDDRC_TPR2                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR2_OFFSET)
#define SAM_MPDDRC_LPR                       (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPR_OFFSET)
#define SAM_MPDDRC_MD                        (SAM_MPDDRC_VBASE+SAM_MPDDRC_MD_OFFSET)
#define SAM_MPDDRC_LPDDR2_LPR                (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_LPR_OFFSET)
#define SAM_MPDDRC_LPDDR2_CALMR4             (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_CALMR4_OFFSET)
#define SAM_MPDDRC_LPDDR2_TIMCAL             (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_TIMCAL_OFFSET)
#define SAM_MPDDRC_IO_CALIBR                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_IO_CALIBR_OFFSET)
#define SAM_MPDDRC_OCMS                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_OFFSET)
#define SAM_MPDDRC_OCMS_KEY1                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_KEY1_OFFSET)
#define SAM_MPDDRC_OCMS_KEY2                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_KEY2_OFFSET)
#define SAM_MPDDRC_CONF_ARBITER              (SAM_MPDDRC_VBASE+SAM_MPDDRC_CONF_ARBITER_OFFSET)
#define SAM_MPDDRC_TIMEOUT                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_TIMEOUT_OFFSET)
#define SAM_MPDDRC_REQ_PORT_0123             (SAM_MPDDRC_VBASE+SAM_MPDDRC_REQ_PORT_0123_OFFSET)
#define SAM_MPDDRC_REQ_PORT_4567             (SAM_MPDDRC_VBASE+SAM_MPDDRC_REQ_PORT_4567_OFFSET)
#define SAM_MPDDRC_BDW_PORT_0123             (SAM_MPDDRC_VBASE+SAM_MPDDRC_BDW_PORT_0123_OFFSET)
#define SAM_MPDDRC_BDW_PORT_4567             (SAM_MPDDRC_VBASE+SAM_MPDDRC_BDW_PORT_4567_OFFSET)
#define SAM_MPDDRC_RD_DATA_PATH              (SAM_MPDDRC_VBASE+SAM_MPDDRC_RD_DATA_PATH_OFFSET)

#define SAM_MPPDRC_SAW(n)                    (SAM_MPDDRC_VBASE+SAM_MPPDRC_SAW_OFFSET(n)
#define SAM_MPDDRC_SAW0                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_SAW0_OFFSET)
#define SAM_MPDDRC_SAW1                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_SAW1_OFFSET)
#define SAM_MPDDRC_SAW2                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_SAW2_OFFSET)
#define SAM_MPDDRC_SAW3                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_SAW3_OFFSET)

#define SAM_MPDDRC_WPCR                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_WPCR_OFFSET)
#define SAM_MPDDRC_WPSR                      (SAM_MPDDRC_VBASE+SAM_MPDDRC_WPSR_OFFSET)

#define SAM_MPDDRC_DLL_OS                    (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_OS_OFFSET)
#define SAM_MPDDRC_DLL_MO                    (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_MO_OFFSET)
#define SAM_MPDDRC_DLL_SO0                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SO0_OFFSET)
#define SAM_MPDDRC_DLL_SO1                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SO1_OFFSET)
#define SAM_MPDDRC_DLL_WRO                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_WRO_OFFSET)
#define SAM_MPDDRC_DLL_ADO                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_ADO_OFFSET)

#define SAM_MPDDRC_DLL_SM(n)                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SM_OFFSET(n))
#define SAM_MPDDRC_DLL_SM0                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SM0_OFFSET)
#define SAM_MPDDRC_DLL_SM1                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SM1_OFFSET)
#define SAM_MPDDRC_DLL_SM2                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SM2_OFFSET)
#define SAM_MPDDRC_DLL_SM3                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SM3_OFFSET)

#define SAM_MPDDRC_DLL_SSL(n)                (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL_OFFSET(n))
#define SAM_MPDDRC_DLL_SSL0                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL0_OFFSET)
#define SAM_MPDDRC_DLL_SSL1                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL1_OFFSET)
#define SAM_MPDDRC_DLL_SSL2                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL2_OFFSET)
#define SAM_MPDDRC_DLL_SSL3                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL3_OFFSET)
#define SAM_MPDDRC_DLL_SSL4                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL4_OFFSET)
#define SAM_MPDDRC_DLL_SSL5                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL5_OFFSET)
#define SAM_MPDDRC_DLL_SSL6                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL6_OFFSET)
#define SAM_MPDDRC_DLL_SSL7                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SSL7_OFFSET)

#define SAM_MPDDRC_DLL_SWR(n)                (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SWR_OFFSET(n))
#define SAM_MPDDRC_DLL_SWR0                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SWR0_OFFSET)
#define SAM_MPDDRC_DLL_SWR1                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SWR1_OFFSET)
#define SAM_MPDDRC_DLL_SWR2                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SWR2_OFFSET)
#define SAM_MPDDRC_DLL_SWR3                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SWR3_OFFSET)

#define SAM_MPDDRC_DLL_SAD                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SAD_OFFSET)

/* MPDDRC Register Bit Definitions ******************************************/

/* MPDDRC Mode Register */

#define MPDDRC_MR_MODE_SHIFT                 (0)       /* Bits 0-2: MPDDRC Command Mode */
#define MPDDRC_MR_MODE_MASK                  (7 << MPDDRC_MR_MODE_SHIFT)
#  define MPDDRC_MR_MODE_NORMAL              (0 << MPDDRC_MR_MODE_SHIFT) /* Normal Mode */
#  define MPDDRC_MR_MODE_NOP                 (1 << MPDDRC_MR_MODE_SHIFT) /* NOP when device accessed */
#  define MPDDRC_MR_MODE_PRCGALL             (2 << MPDDRC_MR_MODE_SHIFT) /* 'All Banks Precharge' when device accessed */
#  define MPDDRC_MR_MODE_LMR                 (3 << MPDDRC_MR_MODE_SHIFT) /* 'Load Mode Register' command when device accessed */
#  define MPDDRC_MR_MODE_RFSH                (4 << MPDDRC_MR_MODE_SHIFT) /* 'Auto-Refresh' when device accessed */
#  define MPDDRC_MR_MODE_EXTLMR              (5 << MPDDRC_MR_MODE_SHIFT) /* 'Extended Load Mode Register' when device accessed */
#  define MPDDRC_MR_MODE_DEEP                (6 << MPDDRC_MR_MODE_SHIFT) /* Deep power mode */
#  define MPDDRC_MR_MODE_LPDDR2              (7 << MPDDRC_MR_MODE_SHIFT) /* LPDDR2 Mode Register' when device accessed */

#define MPDDRC_MR_MRS_SHIFT                  (8)       /* Bits 8-15: Mode Register Select LPDDR2 */
#define MPDDRC_MR_MRS_MASK                   (0xff << MPDDRC_MR_MRS_SHIFT)
#  define MPDDRC_MR_MRS(n)                   ((n) << MPDDRC_MR_MRS_SHIFT)

/* MPDDRC Refresh Timer Register */

#define MPDDRC_RTR_COUNT_SHIFT               (0)       /* Bits 0-11: MPDDRC Refresh Timer Count */
#define MPDDRC_RTR_COUNT_MASK                (0xfff << MPDDRC_RTR_COUNT_SHIFT)
#  define MPDDRC_RTR_COUNT(n)                ((n) << MPDDRC_RTR_COUNT_SHIFT)
#define MPDDRC_RTR_ADJ_REF                   (1 << 16) /* Bit 16: Adjust Refresh Rate */
#define MPDDRC_RTR_REF_PB                    (1 << 17) /* Bit 17: Refresh Per Bank */
#define MPDDRC_RTR_MR4_VALUE_SHIFT           (20)      /* Bits 20-22: Content of MR4 Register */
#define MPDDRC_RTR_MR4_VALUE_MASK            (7 << MPDDRC_RTR_MR4_VALUE_SHIFT)
#  define MPDDRC_RTR_MR4_VALUE(n)            ((n) << MPDDRC_RTR_MR4_VALUE_SHIFT)

/* MPDDRC Configuration Register */

#define MPDDRC_CR_NC_SHIFT                   (0)       /* Bits 0-1: Number of Column Bits */
#define MPDDRC_CR_NC_MASK                    (3 << MPDDRC_CR_NC_SHIFT)
#  define MPDDRC_CR_NC_9                     (0 << MPDDRC_CR_NC_SHIFT) /* 9 DDR column bits */
#  define MPDDRC_CR_NC_10                    (1 << MPDDRC_CR_NC_SHIFT) /* 10 DDR column bits */
#  define MPDDRC_CR_NC_11                    (2 << MPDDRC_CR_NC_SHIFT) /* 11 DDR column bits */
#  define MPDDRC_CR_NC_12                    (3 << MPDDRC_CR_NC_SHIFT) /* 12 DDR column bits */

#define MPDDRC_CR_NR_SHIFT                   (2)       /* Bits 2-3: Number of Row Bits */
#define MPDDRC_CR_NR_MASK                    (3 << MPDDRC_CR_NR_SHIFT)
#  define MPDDRC_CR_NR_11                    (0 << MPDDRC_CR_NR_SHIFT) /* 00 ROW_11 11 row bits */
#  define MPDDRC_CR_NR_12                    (1 << MPDDRC_CR_NR_SHIFT) /* 01 ROW_12 12 row bits */
#  define MPDDRC_CR_NR_13                    (2 << MPDDRC_CR_NR_SHIFT) /* 10 ROW_13 13 row bits */
#  define MPDDRC_CR_NR_14                    (3 << MPDDRC_CR_NR_SHIFT) /* 11 ROW_14 14 row bits */

#define MPDDRC_CR_CAS_SHIFT                  (4)       /* Bits 4-6: CAS Latency */
#define MPDDRC_CR_CAS_MASK                   (7 << MPDDRC_CR_CAS_SHIFT)
#  define MPDDRC_CR_CAS_2                    (2 << MPDDRC_CR_CAS_SHIFT) /* 010 DDR_CAS2 LPDDR1 CAS Latency 2 */
#  define MPDDRC_CR_CAS_3                    (3 << MPDDRC_CR_CAS_SHIFT) /* 011 DDR_CAS3 DDR2/LPDDR2/LPDDR1 CAS Latency 3 */
#  define MPDDRC_CR_CAS_4                    (4 << MPDDRC_CR_CAS_SHIFT) /* 100 DDR_CAS4 DDR2/LPDDR2 CAS Latency 4 */
#  define MPDDRC_CR_CAS_5                    (5 << MPDDRC_CR_CAS_SHIFT) /* 101 DDR_CAS5 DDR2/LPDDR2 CAS Latency 5 */
#  define MPDDRC_CR_CAS_6                    (6 << MPDDRC_CR_CAS_SHIFT) /* 110 DDR_CAS6 DDR2 CAS Latency 6 */

#define MPDDRC_CR_DLL                        (1 << 7)  /* Bit 7:  Reset DLL */
#define MPDDRC_CR_DIC_DS                     (1 << 8)  /* Bit 8:  Output Driver Impedance Control(Drive Strength) */
#define MPDDRC_CR_DIS_DLL                    (1 << 9)  /* Bit 9:  Disable DLL */
#define MPDDRC_CR_ZQ_SHIFT                   (10)      /* Bits 10-11: ZQ Calibration */
#define MPDDRC_CR_ZQ_MASK                    (3 << MPDDRC_CR_ZQ_SHIFT)
#  define MPDDRC_CR_ZQ_INIT                  (0 << MPDDRC_CR_ZQ_SHIFT) /* Calibration command after initialization */
#  define MPDDRC_CR_ZQ_LONG                  (1 << MPDDRC_CR_ZQ_SHIFT) /* Long calibration */
#  define MPDDRC_CR_ZQ_SHORT                 (2 << MPDDRC_CR_ZQ_SHIFT) /* Short calibration */
#  define MPDDRC_CR_ZQ_RESET                 (3 << MPDDRC_CR_ZQ_SHIFT) /* ZQ Reset */

#define MPDDRC_CR_OCD_SHIFT                  (12)      /* Bits 12-14: Off-chip Driver */
#define MPDDRC_CR_OCD_MASK                   (7 << MPDDRC_CR_OCD_SHIFT)
#  define MPDDRC_CR_OCD_EXIT                 (0 << MPDDRC_CR_OCD_SHIFT) /* OCD calibration mode exit, maintain setting */
#  define MPDDRC_CR_OCD_DEFAULT              (7 << MPDDRC_CR_OCD_SHIFT) /* OCD calibration default */

#define MPDDRC_CR_DQMS                       (1 << 16) /* Bit 16: Mask Data is Shared */
#define MPDDRC_CR_ENRDM                      (1 << 17) /* Bit 17: Enable Read Measure */
#define MPDDRC_CR_LC_LPDDR1                  (1 << 19) /* Bit 19: Low-cost Low-power DDR1 */
#define MPDDRC_CR_NB                         (1 << 20) /* Bit 20: Number of Banks */

#  define MPDDRC_CR_4BANKS                   (0)          /* 4 banks */
#  define MPDDRC_CR_8BANKS                   MPDDRC_CR_NB /* 8 banks */

#define MPDDRC_CR_NDQS                       (1 << 21) /* Bit 21: Not DQS */
#define MPDDRC_CR_DECOD                      (1 << 22) /* Bit 22: Type of Decoding */
#define MPDDRC_CR_UNAL                       (1 << 23) /* Bit 23: Support Unaligned Access */

/* MPDDRC Timing Parameter 0 Register */

#define MPDDRC_TPR0_TRAS_SHIFT               (0)       /* Bits 0-3: Active to Precharge Delay */
#define MPDDRC_TPR0_TRAS_MASK                (15 << MPDDRC_TPR0_TRAS_SHIFT)
#  define MPDDRC_TPR0_TRAS(n)                ((n) << MPDDRC_TPR0_TRAS_SHIFT)
#define MPDDRC_TPR0_TRCD_SHIFT               (4)       /* Bits 4-7: Row to Column Delay */
#define MPDDRC_TPR0_TRCD_MASK                (15 << MPDDRC_TPR0_TRCD_SHIFT)
#  define MPDDRC_TPR0_TRCD(n)                ((n) << MPDDRC_TPR0_TRCD_SHIFT)
#define MPDDRC_TPR0_TWR_SHIFT                (8)       /* Bits 8-11: Write Recovery Delay */
#define MPDDRC_TPR0_TWR_MASK                 (15 << MPDDRC_TPR0_TWR_SHIFT)
#  define MPDDRC_TPR0_TWR(n)                 ((n) << MPDDRC_TPR0_TWR_SHIFT)
#define MPDDRC_TPR0_TRC_SHIFT                (12)      /* Bits 12-15: Row Cycle Delay */
#define MPDDRC_TPR0_TRC_MASK                 (15 << MPDDRC_TPR0_TRC_SHIFT)
#  define MPDDRC_TPR0_TRC(n)                 ((n) << MPDDRC_TPR0_TRC_SHIFT)
#define MPDDRC_TPR0_TRP_SHIFT                (16)      /* Bits 16-19: Row Precharge Delay */
#define MPDDRC_TPR0_TRP_MASK                 (15 << MPDDRC_TPR0_TRP_SHIFT)
#  define MPDDRC_TPR0_TRP(n)                 ((n) << MPDDRC_TPR0_TRP_SHIFT)
#define MPDDRC_TPR0_TRRD_SHIFT               (20)      /* Bits 20-23: Active BankA to Active BankB */
#define MPDDRC_TPR0_TRRD_MASK                (15 << MPDDRC_TPR0_TRRD_SHIFT)
#  define MPDDRC_TPR0_TRRD(n)                ((n) << MPDDRC_TPR0_TRRD_SHIFT)
#define MPDDRC_TPR0_TWTR_SHIFT               (24)      /* Bits 24-26: Internal Write to Read Delay */
#define MPDDRC_TPR0_TWTR_MASK                (7 << MPDDRC_TPR0_TWTR_SHIFT)
#  define MPDDRC_TPR0_TWTR(n)                ((n) << MPDDRC_TPR0_TWTR_SHIFT)
#define MPDDRC_TPR0_RDC_WRRD                 (1 << 27) /* Bit 27: Reduce Write to Read Delay */
#define MPDDRC_TPR0_TMRD_SHIFT               (28)      /* Bits 18-31: Load Mode Register Command to Activate or Refresh Command */
#define MPDDRC_TPR0_TMRD_MASK                (15 << MPDDRC_TPR0_TMRD_SHIFT)
#  define MPDDRC_TPR0_TMRD(n)                ((n) << MPDDRC_TPR0_TMRD_SHIFT)

/* MPDDRC Timing Parameter 1 Register */

#define MPDDRC_TPR1_TRFC_SHIFT               (0)       /* Bits 0-6: Row Cycle Delay */
#define MPDDRC_TPR1_TRFC_MASK                (0x7f << MPDDRC_TPR1_TRFC_SHIFT)
#  define MPDDRC_TPR1_TRFC(n)                ((n) << MPDDRC_TPR1_TRFC_SHIFT)
#define MPDDRC_TPR1_TXSNR_SHIFT              (8)       /* Bits 8-15: Exit Self Refresh Delay to Non Read Command */
#define MPDDRC_TPR1_TXSNR_MASK               (0xff << MPDDRC_TPR1_TXSNR_SHIFT)
#  define MPDDRC_TPR1_TXSNR(n)               ((n) << MPDDRC_TPR1_TXSNR_SHIFT)
#define MPDDRC_TPR1_TXSRD_SHIFT              (16)      /* Bits 16-23: Exit Self Refresh Delay to Read Command */
#define MPDDRC_TPR1_TXSRD_MASK               (0xff << MPDDRC_TPR1_TXSRD_SHIFT)
#  define MPDDRC_TPR1_TXSRD(n)               ((n) << MPDDRC_TPR1_TXSRD_SHIFT)
#define MPDDRC_TPR1_TXP_SHIFT                (24)      /* Bits 24-27: Exit Power-down Delay to First Command */
#define MPDDRC_TPR1_TXP_MASK                 (15 << MPDDRC_TPR1_TXP_SHIFT)
#  define MPDDRC_TPR1_TXP(n)                 ((n) << MPDDRC_TPR1_TXP_SHIFT)

/* MPDDRC Timing Parameter 2 Register */

#define MPDDRC_TPR2_TXARD_SHIFT              (0)       /* Bits 0-3: Exit Active Power Down Delay to Read Command in Mode 'Fast Exit' */
#define MPDDRC_TPR2_TXARD_MASK               (15 << MPDDRC_TPR2_TXARD_SHIFT)
#  define MPDDRC_TPR2_TXARD(n)               ((n) << MPDDRC_TPR2_TXARD_SHIFT)
#define MPDDRC_TPR2_TXARDS_SHIFT             (4)       /* Bits 4-7: Exit Active Power Down Delay to Read Command in Mode 'Slow Exit' */
#define MPDDRC_TPR2_TXARDS_MASK              (15 << MPDDRC_TPR2_TXARDS_SHIFT)
#  define MPDDRC_TPR2_TXARDS(n)              ((n) << MPDDRC_TPR2_TXARDS_SHIFT)
#define MPDDRC_TPR2_TRPA_SHIFT               (8)       /* Bits 8-11: Row Precharge All Delay */
#define MPDDRC_TPR2_TRPA_MASK                (15 << MPDDRC_TPR2_TRPA_SHIFT)
#  define MPDDRC_TPR2_TRPA(n)                ((n) << MPDDRC_TPR2_TRPA_SHIFT)
#define MPDDRC_TPR2_TRTP_SHIFT               (12)      /* Bits 12-14: Read to Precharge */
#define MPDDRC_TPR2_TRTP_MASK                (7 << MPDDRC_TPR2_TRTP_SHIFT)
#  define MPDDRC_TPR2_TRTP(n)                ((n) << MPDDRC_TPR2_TRTP_SHIFT)
#define MPDDRC_TPR2_TFAW_SHIFT               (16)      /* Bits 16-19: Four Active Windows */
#define MPDDRC_TPR2_TFAW_MASK                (15 << MPDDRC_TPR2_TFAW_SHIFT)
#  define MPDDRC_TPR2_TFAW(n)                ((n) << MPDDRC_TPR2_TFAW_SHIFT)

/* MPDDRC Low-power Register */

#define MPDDRC_LPR_LPCB_SHIFT                (0)       /* Bits 0-1: Low-power Command*/
#define MPDDRC_LPR_LPCB_MASK                 (3 << MPDDRC_LPR_LPCB_SHIFT)
#  define MPDDRC_LPR_LPCB_DISABLED           (0 << MPDDRC_LPR_LPCB_SHIFT) /* Low-power Feature is inhibited */
#  define MPDDRC_LPR_LPCB_SELFREFRESH        (1 << MPDDRC_LPR_LPCB_SHIFT) /* Issues a 'Self Refresh' to device, clocks deactivated */
#  define MPDDRC_LPR_LPCB_POWERDOWN          (2 << MPDDRC_LPR_LPCB_SHIFT) /* Issues a 'Power-down' to device after each access */
#  define MPDDRC_LPR_LPCB_DEEPPWD            (3 << MPDDRC_LPR_LPCB_SHIFT) /* TIssues a 'Deep Power-down' to Low-power device */

#define MPDDRC_LPR_CLK_FR                    (1 << 2)  /* Bit 2:  Clock Frozen Command */
#define MPDDRC_LPR_LPDDR2_PWOFF              (1 << 3)  /* Bit 3:  LPDDR2 Power Off */
#define MPDDRC_LPR_PASR_SHIFT                (4)       /* Bits 4-6: Partial Array Self Refresh */
#define MPDDRC_LPR_PASR_MASK                 (7 << MPDDRC_LPR_PASR_SHIFT)
#  define MPDDRC_LPR_PASR(n)                 ((n) << MPDDRC_LPR_PASR_SHIFT)
#define MPDDRC_LPR_DS_SHIFT                  (8)       /* Bits 8-10: Drive Strength */
#define MPDDRC_LPR_DS_MASK                   (7 << MPDDRC_LPR_DS_SHIFT)
#  define MPDDRC_LPR_DS(n)                   ((n) << MPDDRC_LPR_DS_SHIFT)
#define MPDDRC_LPR_TIMEOUT_SHIFT             (12)      /* Bits 12-13: Enter Low-power Mode */
#define MPDDRC_LPR_TIMEOUT_MASK              (3 << MPDDRC_LPR_TIMEOUT_SHIFT)
#  define MPDDRC_LPR_TIMEOUT_0CLKS           (0 << MPDDRC_LPR_TIMEOUT_SHIFT) /* Activates low-power mode after the end of transfer */
#  define MPDDRC_LPR_TIMEOUT_64CLKS          (1 << MPDDRC_LPR_TIMEOUT_SHIFT) /* Activates low-power mode 64 clocks after the end of transfer */
#  define MPDDRC_LPR_TIMEOUT_128CLKS         (2 << MPDDRC_LPR_TIMEOUT_SHIFT) /* 28 Activates low-power mode 128 clocks after the end of transfer */

#define MPDDRC_LPR_APDE                      (1 << 16) /* Bit 16:  ctive Power Down Exit Time */
#  define MPDDRC_LPR_APDE_FAST               (0)
#  define MPDDRC_LPR_APDE_SLOW               MPDDRC_LPR_APDE
#define MPDDRC_LPR_UPD_MR_SHIFT              (20)      /* Bits 20-21: Update Load Mode Register and Extended Mode Register */
#define MPDDRC_LPR_UPD_MR_MASK               (3 << MPDDRC_LPR_UPD_MR_SHIFT)
#  define MPDDRC_LPR_UPD_MR_DISABLED         (0 << MPDDRC_LPR_UPD_MR_SHIFT) /* DISABLED Update is disabled */
#  define MPDDRC_LPR_UPD_MR_SHARED           (1 << MPDDRC_LPR_UPD_MR_SHIFT) /* MPDDRC shares external bus */
#  define MPDDRC_LPR_UPD_MR_UNSHARED         (2 << MPDDRC_LPR_UPD_MR_SHIFT) /* MPDDRC does not share external bus */

/* MPDDRC Memory Device Register */

#define MPDDRC_MD_SHIFT                      (0)       /* Bits 0-2: Memory Device */
#define MPDDRC_MD_MASK                       (7 << MPDDRC_MD_SHIFT)
#  define MPDDRC_MD_DDR_SDRAM                (2 << MPDDRC_MD_SHIFT) /* DDR1-SDRAM */
#  define MPDDRC_MD_LPDDR_SDRAM              (3 << MPDDRC_MD_SHIFT) /* Low-power DDR1-SDRAM */
#  define MPDDRC_MD_DDR2_SDRAM               (6 << MPDDRC_MD_SHIFT) /* DDR2-SDRAM */
#  define MPDDRC_MD_LPDDR2_SDRAM             (7 << MPDDRC_MD_SHIFT) /* Low-power DDR2-SDRAM */

#define MPDDRC_MD_DBW                        (1 << 4)  /* Bit 4:  Data Bus Width */

#  define MPDDRC_MD_DBW32                    (0)           /* Data bus width is 32 bits */
#  define MPDDRC_MD_DBW16                    MPDDRC_MD_DBW /* Data bus width is 16 bits */

/* MPDDRC LPDDR2 Low-power Register */

#define MPDDRC_LPDDR2_LPR_BK_MASK_PASR_SHIFT (0)  /* Bits 0-7: Bank Mask Bit/PASR */
#define MPDDRC_LPDDR2_LPR_BK_MASK_PASR_MASK  (0xff << MPDDRC_LPDDR2_LPR_BK_MASK_PASR_SHIFT)
#  define MPDDRC_LPDDR2_LPR_BK_MASK_PASR(n)  ((n) << MPDDRC_LPDDR2_LPR_BK_MASK_PASR_SHIFT)
#define MPDDRC_LPDDR2_LPR_SEG_MASK_SHIFT     (8)  /* Bits 8-23: Segment Mask*/
#define MPDDRC_LPDDR2_LPR_SEG_MASK_MASK      (0xffff << MPDDRC_LPDDR2_LPR_SEG_MASK_SHIFT)
#  define MPDDRC_LPDDR2_LPR_SEG_MASK(n)      ((n) << MPDDRC_LPDDR2_LPR_SEG_MASK_SHIFT)
#define MPDDRC_LPDDR2_LPR_DS_SHIFT           (24) /* Bits 24-27: Drive strength */
#define MPDDRC_LPDDR2_LPR_DS_MASK            (15 << MPDDRC_LPDDR2_LPR_DS_SHIFT)
#  define MPDDRC_LPDDR2_LPR_DS(n)            ((n) << MPDDRC_LPDDR2_LPR_DS_SHIFT)

/* MPDDRC LPDDR2 Calibration and MR4 Register */

#define MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT (0)  /* Bits 0-15: LPDDR2 Calibration Timer Count */
#define MPDDRC_LPDDR2_CALMR4_COUNT_CAL_MASK  (0xffff << MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT)
#  define MPDDRC_LPDDR2_CALMR4_COUNT_CAL(n)  ((n) << MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT)
#define MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT  (16) /* Bits 16-31: Mode Register 4 Read Interval */
#define MPDDRC_LPDDR2_CALMR4_MR4_READ_MASK   (0xffff << MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT)
#  define MPDDRC_LPDDR2_CALMR4_MR4_READ(n)   ((n) << MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT)

/* MPDDRC LPDDR2 Timing Calibration Register */

#define MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT      (0)       /* Bits 0-7: ZQ Calibration Short */
#define MPDDRC_LPDDR2_TIMCAL_ZQCS_MASK       (0xff << MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT)
#  define MPDDRC_LPDDR2_TIMCAL_ZQCS(n)       ((n) << MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT)

/* MPDDRC IO Calibration */

#define MPDDRC_IO_CALIBR_RDIV_SHIFT          (0)       /* Bits 0-2: Resistor Divider, Output Driver Impedance */
#define MPDDRC_IO_CALIBR_RDIV_MASK           (7 << MPDDRC_IO_CALIBR_RDIV_SHIFT)
#  define MPDDRC_IO_CALIBR_RZQ34_NA          (1 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2 RZQ = 34.3 Ohm DDR2/LPDDR1: Not applicable */
#  define MPDDRC_IO_CALIBR_RZQ40_33          (2 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 40 Ohm   DDR2/LPDDR1: RZQ = 33.3 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ48_40          (3 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 48 Ohm   DDR2/LPDDR1: RZQ = 40 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ60_50          (4 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 60 Ohm   DDR2/LPDDR1: RZQ = 50 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ80_67          (6 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2 RZQ = 80 Ohm   DDR2/LPDDR1: RZQ = 66.7 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ120_100        (7 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 120 Oh m DDR2/LPDDR1: RZQ = 100 Ohm */

#define MPDDRC_IO_CALIBR_EN_CALIB            (1 << 4)  /* Bit 4: Enable of the Calibration */
#define MPDDRC_IO_CALIBR_TZQIO_SHIFT         (8)       /* Bits 8-10: IO Calibration */
#define MPDDRC_IO_CALIBR_TZQIO_MASK          (7 << MPDDRC_IO_CALIBR_TZQIO_SHIFT)
#  define MPDDRC_IO_CALIBR_TZQIO(n)          ((n) << MPDDRC_IO_CALIBR_TZQIO_SHIFT)
#define MPDDRC_IO_CALIBR_CALCODEP_SHIFT      (16)      /* Bits 16-19: Number of Transistor P */
#define MPDDRC_IO_CALIBR_CALCODEP_MASK       (15 << MPDDRC_IO_CALIBR_CALCODEP_SHIFT)
#  define MPDDRC_IO_CALIBR_CALCODEP(n)       ((n) << MPDDRC_IO_CALIBR_CALCODEP_SHIFT)
#define MPDDRC_IO_CALIBR_CALCODEN_SHIFT      (20)      /* Bits 20-23: Number of Transistor N */
#define MPDDRC_IO_CALIBR_CALCODEN_MASK       (15 << MPDDRC_IO_CALIBR_CALCODEN_SHIFT)
#  define MPDDRC_IO_CALIBR_CALCODEN(n)       ((n) << MPDDRC_IO_CALIBR_CALCODEN_SHIFT)

/* MPDDRC OCMS Register */

#define MPDDRC_OCMS_SCR_EN                   (1 << 0)  /* Bit 0:  Scrambling enable */

/* MPDDRC OCMS KEY1 Register (32-bit key value) */

/* MPDDRC OCMS KEY2 Register (32-bit key value) */

/* MPDDRC Configuration Arbiter Register */

#define MPDDRC_CONF_ARBITER_ARB_SHIFT        (0)       /* Bits 0-1: Type of Arbitration */
#define MPDDRC_CONF_ARBITER_ARB_MASK         (3 << MPDDRC_CONF_ARBITER_ARB_SHIFT)
#  define MPDDRC_CONF_ARBITER_ARB_ROUND      (0 << MPDDRC_CONF_ARBITER_ARB_SHIFT) /* Round Robin */
#  define MPDDRC_CONF_ARBITER_ARB_NB_REQUEST (1 << MPDDRC_CONF_ARBITER_ARB_SHIFT) /* Request Policy */
#  define MPDDRC_CONF_ARBITER_ARB_BANDWIDTH  (2 << MPDDRC_CONF_ARBITER_ARB_SHIFT) /* Bandwidth Policy */

#define MPDDRC_CONF_ARBITER_BDW_BURST        (1 << 2)      /* Bit 2:  Bandwidth is Reached or Bandwidth and Current Burst Access is Ended */
#define MPDDRC_CONF_ARBITER_BDW_MAX_CUR      (1 << 3)      /* Bit 3:  Bandwidth Max or Current */
#define MPDDRC_CONF_ARBITER_RQ_WD_P(n)       (1 << ((n)+8) /* Bits 8-15:  Request or Word from Port n */

#  define MPDDRC_CONF_ARBITER_RQ_WD_P0       (1 << 8)        /* Bit 8:  Request or Word from Port 0 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P1       (1 << 9)        /* Bit 9:  Request or Word from Port 1 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P2       (1 << 10)       /* Bit 10: Request or Word from Port 2 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P3       (1 << 11)       /* Bit 11: Request or Word from Port 3 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P4       (1 << 12)       /* Bit 12: Request or Word from Port 4 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P5       (1 << 13)       /* Bit 13: Request or Word from Port 5 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P6       (1 << 14)       /* Bit 14: Request or Word from Port 6 */
#  define MPDDRC_CONF_ARBITER_RQ_WD_P7       (1 << 15)       /* Bit 15: Request or Word from Port 7 */
#define MPDDRC_CONF_ARBITER_MA_PR_P(n)       (1 << ((n)+16)) /* Bits 16-23: Master or Software Provide Information */

#  define MPDDRC_CONF_ARBITER_MA_PR_P0       (1 << 16) /* Bit 16: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P1       (1 << 17) /* Bit 17: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P2       (1 << 18) /* Bit 18: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P3       (1 << 19) /* Bit 19: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P4       (1 << 20) /* Bit 20: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P5       (1 << 21) /* Bit 21: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P6       (1 << 22) /* Bit 22: Master or Software Provide Information */
#  define MPDDRC_CONF_ARBITER_MA_PR_P7       (1 << 23) /* Bit 23: Master or Software Provide Information */

/* MPDDRC Time-out Port 0/1/2/3 Register */

#define MPDDRC_TIMEOUT_TIMEOUT_P_SHIFT(n)    ((n)<<4)  /* Time-out for Port n, n=0..7 */
#define MPDDRC_TIMEOUT_TIMEOUT_P_MASK(n)     (15 << MPDDRC_TIMEOUT_TIMEOUT_P_SHIFT(n))
#  define MPDDRC_TIMEOUT_TIMEOUT_P(n,v)      ((uint32_t)(v) << MPDDRC_TIMEOUT_TIMEOUT_P_SHIFT(n))
#  define MPDDRC_TIMEOUT_TIMEOUT_P0_SHIFT    (0)       /* Bits 4-7: Time-out for Port 0 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P0_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P0_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P0(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P0_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P1_SHIFT    (4)       /* Bits 0-3: Time-out for Port 1 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P1_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P1_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P1(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P1_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P2_SHIFT    (8)       /* Bits 8-7: Time-out for Port 2 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P2_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P2_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P2(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P2_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P3_SHIFT    (12)      /* Bits 12-15: Time-out for Port 3 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P3_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P3_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P3(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P3_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P4_SHIFT    (16)      /* Bits 16-19: Time-out for Port 4 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P4_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P4_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P4(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P4_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P5_SHIFT    (20)      /* Bits 20-23: Time-out for Port 5 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P5_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P5_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P5(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P5_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P6_SHIFT    (24)      /* Bits 24-27: Time-out for Port 6 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P6_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P6_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P6(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P6_SHIFT)
#  define MPDDRC_TIMEOUT_TIMEOUT_P7_SHIFT    (28)      /* Bits 28-31: Time-out for Port 7 */
#  define MPDDRC_TIMEOUT_TIMEOUT_P7_MASK     (15 << MPDDRC_TIMEOUT_TIMEOUT_P7_SHIFT)
#    define MPDDRC_TIMEOUT_TIMEOUT_P7(n)     ((uint32_t)(n) << MPDDRC_TIMEOUT_TIMEOUT_P7_SHIFT)

/* MPDDRC Request Port 0/1/2/3 Register */

#define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P_SHIFT(n) ((n)<<8) /* Number of Requests/Words/Allocation from Port n, n=0..3 */
#define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P_MASK(n)  (0xff << PDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P_SHIFT(n))

#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P(n,v)   ((uint32_t)(v) << PDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P_SHIFT(n))
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P0_SHIFT (0)   /* Bits 0-7: Number of Requests/Words/Allocation from Port 0 */
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P0_MASK  (0xff << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P0_SHIFT)
#    define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P0(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P0_SHIFT)
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P1_SHIFT (8)  /* Bits 8-15: Number of Requests/Words/Allocation from Port 1 */
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P1_MASK  (0xff << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P1_SHIFT)
#    define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P1(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P1_SHIFT)
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P2_SHIFT (16) /* Bits 16-23: Number of Requests/Words/Allocation from Port 2 */
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P2_MASK  (0xff << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P2_SHIFT)
#    define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P2(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P2_SHIFT)
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P3_SHIFT (24) /* Bits 24-31: Number of Requests/Words/Allocation from Port 3 */
#  define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P3_MASK  (0xff << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P3_SHIFT)
#    define MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P3(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_0123_NRQ_NWD_BDW_P3_SHIFT)

/* MPDDRC Request Port 4/5/6/7 Register */

#define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P_SHIFT(n) ((n)<<8) /* Number of Requests/Words/Allocation from port n, n=4..7 */
#define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P_MASK(n)  (0xff << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P_SHIFT(n))

#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P(n,v)   ((uint32_t)(v) << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P_SHIFT(n))
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P4_SHIFT (0)  /* Bits 0-7: Number of Requests/Words/Allocation from port 4 */
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P4_MASK  (0xff << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P4_SHIFT)
#    define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P4(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P4_SHIFT)
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P5_SHIFT (8)  /* Bits 8-15: Number of Requests/Words/Allocation from port 5 */
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P5_MASK  (0xff << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P5_SHIFT)
#    define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P5(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P5_SHIFT)
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P6_SHIFT (16) /* Bits 16-23: Number of Requests/Words/Allocation from port 6 */
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P6_MASK  (0xff << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P6_SHIFT)
#    define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P6(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P6_SHIFT)
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P7_SHIFT (24) /* Bits 24-31: Number of Requests/Words/Allocation from port 7 */
#  define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P7_MASK  (0xff << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P7_SHIFT)
#    define MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P7(n)  ((uint32_t)(n) << MPDDRC_REQ_PORT_4567_NRQ_NWD_BDW_P7_SHIFT)

/* MPDDRC Bandwidth Port 0/1/2/3 Register */

#define MPDDRC_BDW_PORT_0123_BDW_P_SHIFT(n)  ((n)<<8)  /* Current/Maximum Bandwidth from Port n, n=0..3 */
#define MPDDRC_BDW_PORT_0123_BDW_P_MASK(n)   (0x7f << PDDRC_BDW_PORT_0123_BDW_P_SHIFT(n))
#  define MPDDRC_BDW_PORT_0123_BDW_P(n,v)    ((uint32_t)(v) << PDDRC_BDW_PORT_0123_BDW_P_SHIFT(n))
#  define MPDDRC_BDW_PORT_0123_BDW_P0_SHIFT  (0)       /* Bits 0-6: Current/Maximum Bandwidth from Port 0 */
#  define MPDDRC_BDW_PORT_0123_BDW_P0_MASK   (0x7f << MPDDRC_BDW_PORT_0123_BDW_P0_SHIFT)
#    define MPDDRC_BDW_PORT_0123_BDW_P0(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_0123_BDW_P0_SHIFT)
#  define MPDDRC_BDW_PORT_0123_BDW_P1_SHIFT  (8)       /* Bits 8-14: Current/Maximum Bandwidth from Port 1 */
#  define MPDDRC_BDW_PORT_0123_BDW_P1_MASK   (0x7f << MPDDRC_BDW_PORT_0123_BDW_P1_SHIFT)
#    define MPDDRC_BDW_PORT_0123_BDW_P1(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_0123_BDW_P1_SHIFT)
#  define MPDDRC_BDW_PORT_0123_BDW_P2_SHIFT  (16)      /* Bits 16-22: Current/Maximum Bandwidth from Port 2 */
#  define MPDDRC_BDW_PORT_0123_BDW_P2_MASK   (0x7f << MPDDRC_BDW_PORT_0123_BDW_P2_SHIFT)
#    define MPDDRC_BDW_PORT_0123_BDW_P2(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_0123_BDW_P2_SHIFT)
#  define MPDDRC_BDW_PORT_0123_BDW_P3_SHIFT  (24)      /* Bits 24-30: Current/Maximum Bandwidth from Port 3 */
#  define MPDDRC_BDW_PORT_0123_BDW_P3_MASK   (0x7f << MPDDRC_BDW_PORT_0123_BDW_P3_SHIFT)
#    define MPDDRC_BDW_PORT_0123_BDW_P3(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_0123_BDW_P3_SHIFT)

/* MPDDRC Bandwidth Port 4/5/6/7 Register */

#define MPDDRC_BDW_PORT_4567_BDW_P_SHIFT(n)  ((n)<<8)  /* Current/Maximum Bandwidth from Port n, n=4..7 */
#define MPDDRC_BDW_PORT_4567_BDW_P_MASK(n)   (0x7f << PDDRC_BDW_PORT_4567_BDW_P_SHIFT(n))
#  define MPDDRC_BDW_PORT_4567_BDW_P(n,v)    ((uint32_t)(v) << PDDRC_BDW_PORT_4567_BDW_P_SHIFT(n))
#  define MPDDRC_BDW_PORT_4567_BDW_P4_SHIFT  (0)       /* Bits 0-6: Current/Maximum Bandwidth from Port 4 */
#  define MPDDRC_BDW_PORT_4567_BDW_P4_MASK   (0x7f << MPDDRC_BDW_PORT_4567_BDW_P4_SHIFT)
#    define MPDDRC_BDW_PORT_4567_BDW_P4(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_4567_BDW_P4_SHIFT)
#  define MPDDRC_BDW_PORT_4567_BDW_P5_SHIFT  (8)       /* Bits 8-14: Current/Maximum Bandwidth from Port 5 */
#  define MPDDRC_BDW_PORT_4567_BDW_P5_MASK   (0x7f << MPDDRC_BDW_PORT_4567_BDW_P5_SHIFT)
#    define MPDDRC_BDW_PORT_4567_BDW_P5(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_4567_BDW_P5_SHIFT)
#  define MPDDRC_BDW_PORT_4567_BDW_P6_SHIFT  (16)      /* Bits 16-22: Current/Maximum Bandwidth from Port 6 */
#  define MPDDRC_BDW_PORT_4567_BDW_P6_MASK   (0x7f << MPDDRC_BDW_PORT_4567_BDW_P6_SHIFT)
#    define MPDDRC_BDW_PORT_4567_BDW_P6(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_4567_BDW_P6_SHIFT)
#  define MPDDRC_BDW_PORT_4567_BDW_P7_SHIFT  (24)      /* Bits 24-30: Current/Maximum Bandwidth from Port 7 */
#  define MPDDRC_BDW_PORT_4567_BDW_P7_MASK   (0x7f << MPDDRC_BDW_PORT_4567_BDW_P7_SHIFT)
#    define MPDDRC_BDW_PORT_4567_BDW_P7(n)   ((uint32_t)(n) << MPDDRC_BDW_PORT_4567_BDW_P7_SHIFT)

/* MPDDRC Read Datapath Register */

#define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT     (0) /* Bits 0-1: Shift Sampling Point of Data */
#define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_MASK      (3 << MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT)
#  define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_NONE    (0 << MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT) /* Initial sampling point */
#  define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_1CYCLE  (1 << MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT) /* Sampling shifted by 1 cycle */
#  define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_2CYCLES (2 << MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT) /* Sampling point shifted by 2 cycles */
#  define MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_3CYCLES (3 << MPDDRC_RD_DATA_PATH_SHIFT_SAMPLING_SHIFT) /* Sampling point shifted by 3 cycles */

/* MPDDRC Smart Adaptation Wrapper 0-3 Register */

#define MPDDRC_SAW_FLUSH_MAX_SHIFT           (0)       /* Bits 0-7: Clears FIFO Content */
#define MPDDRC_SAW_FLUSH_MAX_MASK            (0xff << MPDDRC_SAW_FLUSH_MAX_SHIFT)
#  define MPDDRC_SAW_FLUSH_MAX(n)            ((uint32_t)(n) << MPDDRC_SAW_FLUSH_MAX_SHIFT)
#define MPDDRC_SAW_INCR_THRESH_SHIFT         (8)       /* Bits 8-13: Incremental Threshold */
#define MPDDRC_SAW_INCR_THRESH_MASK          (0x3f << MPDDRC_SAW_INCR_THRESH_SHIFT)
#  define MPDDRC_SAW_INCR_THRESH_1WORD       (1 << MPDDRC_SAW_INCR_THRESH_SHIFT)  /* 1 word/dword max */
#  define MPDDRC_SAW_INCR_THRESH_2WORDS      (2 << MPDDRC_SAW_INCR_THRESH_SHIFT)  /* 2 word/dword max */
#  define MPDDRC_SAW_INCR_THRESH_4WORDS      (4 << MPDDRC_SAW_INCR_THRESH_SHIFT)  /* 4 word/dword max */
#  define MPDDRC_SAW_INCR_THRESH_8WORDS      (8 << MPDDRC_SAW_INCR_THRESH_SHIFT)  /* 8 word/dword max */
#  define MPDDRC_SAW_INCR_THRESH_16WORDS     (16 << MPDDRC_SAW_INCR_THRESH_SHIFT) /* 16 word/dword max */
#  define MPDDRC_SAW_INCR_THRESH_32WORDS     (21 << MPDDRC_SAW_INCR_THRESH_SHIFT) /* 32 word/dword max */

#define MPDDRC_SAW_PFCH_THRESH_SHIFT         (16)      /* Bits 16-21: Prefetch Threshold */
#define MPDDRC_SAW_PFCH_THRESH_MASK          (0x3f << MPDDRC_SAW_PFCH_THRESH_SHIFT)
#  define MPDDRC_SAW_PFCH_THRESH_2WORDS      (2 << MPDDRC_SAW_PFCH_THRESH_SHIFT) /* 2 word/dword max */
#  define MPDDRC_SAW_PFCH_THRESH_4WORDS      (4 << MPDDRC_SAW_PFCH_THRESH_SHIFT) /* 4 word/dword max */
#  define MPDDRC_SAW_PFCH_THRESH_8WORDS      (8 << MPDDRC_SAW_PFCH_THRESH_SHIFT) /* 8 word/dword max */

/* MPDDRC Write Protect Control Register */

#define MPDDRC_WPCR_WPEN                     (1 << 0)  /* Bit 0:  Write Protection Enable */
#define MPDDRC_WPCR_WPKEY_SHIFT              (8)       /* Bits 8-31: Write Protection KEY */
#define MPDDRC_WPCR_WPKEY_MASK               (0x00ffffff << MPDDRC_WPCR_WPKEY_SHIFT)
#  define MPDDRC_WPCR_WPKEY                  (0x00444452 << MPDDRC_WPCR_WPKEY_SHIFT)

/* MPDDRC Write Protect Status Register */

#define MPDDRC_WPSR_WPVS                     (1 << 0)  /* Bit 0:  Write Protection Enable */
#define MPDDRC_WPSR_WPVSRC_SHIFT             (8)       /* Bits 8-23: Write Protection Violation Source */
#define MPDDRC_WPSR_WPVSRC_MASK              (0xffff << MPDDRC_WPSR_WPVSRC_SHIFT)

/* MPDDRC DLL Offset Selection Register */

#define MPDDRC_DLL_OS_SELOFF                 (1 << 0)  /* Bit 0:  Offset Selection */

/* MPDDRC DLL MASTER Offset Register */

#define MPDDRC_DLL_MO_M0OFF_SHIFT            (0)       /* Bits 0-7: Master 0 Delay Line Offset */
#define MPDDRC_DLL_MO_M0OFF_MASK             (0xff < MPDDRC_DLL_MO_M0OFF_SHIFT)
#  define MPDDRC_DLL_MO_M0OFF(n)             ((uint32_t)(n) < MPDDRC_DLL_MO_M0OFF_SHIFT)

/* MPDDRC DLL SLAVE Offset 0 Register */

#define MPDDRC_DLL_SO0_SOFF_SHIFT(n)         (0)       /* SLAVEn Delay Line Offset, n=0..3 */
#define MPDDRC_DLL_SO0_SOFF_MASK(n)          (0xff < MPDDRC_DLL_SO0_SOFF_SHIFT(n))
#  define MPDDRC_DLL_SO0_SOFF(n,v)           ((uint32_t)(v) < MPDDRC_DLL_SO0_SOFF_SHIFT(n))
#  define MPDDRC_DLL_SO0_S0OFF_SHIFT         (0)       /* Bits 0-7: SLAVE0 Delay Line Offset */
#  define MPDDRC_DLL_SO0_S0OFF_MASK          (0xff < MPDDRC_DLL_SO0_S0OFF_SHIFT)
#    define MPDDRC_DLL_SO0_S0OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO0_S0OFF_SHIFT)
#  define MPDDRC_DLL_SO0_S1OFF_SHIFT         (8)       /* Bits 8-15: SLAVE1 Delay Line Offset */
#  define MPDDRC_DLL_SO0_S1OFF_MASK          (0xff < MPDDRC_DLL_SO0_S1OFF_SHIFT)
#    define MPDDRC_DLL_SO0_S1OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO0_S1OFF_SHIFT)
#  define MPDDRC_DLL_SO0_S2OFF_SHIFT         (16)      /* Bits 16-23: SLAVE2 Delay Line Offset */
#  define MPDDRC_DLL_SO0_S2OFF_MASK          (0xff < MPDDRC_DLL_SO0_S2OFF_SHIFT)
#    define MPDDRC_DLL_SO0_S2OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO0_S2OFF_SHIFT)
#  define MPDDRC_DLL_SO0_S3OFF_SHIFT         (24)      /* Bits 24-31: SLAVE3 Delay Line Offset */
#  define MPDDRC_DLL_SO0_S3OFF_MASK          (0xff < MPDDRC_DLL_SO0_S3OFF_SHIFT)
#    define MPDDRC_DLL_SO0_S3OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO0_S3OFF_SHIFT)

/* MPDDRC DLL SLAVE Offset 1 Register */

#define MPDDRC_DLL_SO1_SOFF_SHIFT(n)         (0)       /* SLAVEn Delay Line Offset, n=4..7 */
#define MPDDRC_DLL_SO1_SOFF_MASK(n)          (0xff < MPDDRC_DLL_SO1_SOFF_SHIFT(n))
#  define MPDDRC_DLL_SO1_SOFF(n,v)           ((uint32_t)(v) < MPDDRC_DLL_SO1_SOFF_SHIFT(n))
#  define MPDDRC_DLL_SO1_S4OFF_SHIFT         (0)       /* Bits 0-7: SLAVE4 Delay Line Offset */
#  define MPDDRC_DLL_SO1_S4OFF_MASK          (0xff < MPDDRC_DLL_SO1_S4OFF_SHIFT)
#    define MPDDRC_DLL_SO1_S4OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO1_S4OFF_SHIFT)
#  define MPDDRC_DLL_SO1_S5OFF_SHIFT         (8)       /* Bits 8-15: SLAVE5 Delay Line Offset */
#  define MPDDRC_DLL_SO1_S5OFF_MASK          (0xff < MPDDRC_DLL_SO1_S5OFF_SHIFT)
#    define MPDDRC_DLL_SO1_S5OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO1_S5OFF_SHIFT)
#  define MPDDRC_DLL_SO1_S6OFF_SHIFT         (16)      /* Bits 16-23: SLAVE6 Delay Line Offset */
#  define MPDDRC_DLL_SO1_S6OFF_MASK          (0xff < MPDDRC_DLL_SO1_S6OFF_SHIFT)
#    define MPDDRC_DLL_SO1_S6OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO1_S6OFF_SHIFT)
#  define MPDDRC_DLL_SO1_S7OFF_SHIFT         (24)      /* Bits 24-31: SLAVE7 Delay Line Offset */
#  define MPDDRC_DLL_SO1_S7OFF_MASK          (0xff < MPDDRC_DLL_SO1_S7OFF_SHIFT)
#    define MPDDRC_DLL_SO1_S7OFF(n)          ((uint32_t)(n) < MPDDRC_DLL_SO1_S7OFF_SHIFT)

/* MPDDRC DLL CLKWR Offset Register */

#define MPDDRC_DLL_WRO_WROFF_SHIFT(n)        (0)       /* CLKWRn Delay Line Offset, n=0..3 */
#define MPDDRC_DLL_WRO_WROFF_MASK(n)         (0xff < MPDDRC_DLL_WRO_WROFF_SHIFT(n))
#  define MPDDRC_DLL_WRO_WROFF(n,v)          ((uint32_t)(n) < MPDDRC_DLL_WRO_WROFF_SHIFT(n))
#  define MPDDRC_DLL_WRO_WR0OFF_SHIFT        (0)       /* Bits 0-7: CLKWR0 Delay Line Offset */
#  define MPDDRC_DLL_WRO_WR0OFF_MASK         (0xff < MPDDRC_DLL_WRO_WR0OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR0OFF(n)           ((uint32_t)(n) < MPDDRC_DLL_WRO_WR0OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR1OFF_SHIFT        (8)       /* Bits 8-15: CLKWR1 Delay Line Offset */
#  define MPDDRC_DLL_WRO_WR1OFF_MASK         (0xff < MPDDRC_DLL_WRO_WR1OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR1OFF(n)           ((uint32_t)(n) < MPDDRC_DLL_WRO_WR1OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR2OFF_SHIFT        (16)      /* Bits 16-23: CLKWR2 Delay Line Offset */
#  define MPDDRC_DLL_WRO_WR2OFF_MASK         (0xff < MPDDRC_DLL_WRO_WR2OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR2OFF(n)           ((uint32_t)(n) < MPDDRC_DLL_WRO_WR2OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR3OFF_SHIFT        (24)      /* Bits 24-31: CLKWR3 Delay Line Offset */
#  define MPDDRC_DLL_WRO_WR3OFF_MASK         (0xff < MPDDRC_DLL_WRO_WR3OFF_SHIFT)
#  define MPDDRC_DLL_WRO_WR3OFF(n)           ((uint32_t)(n) < MPDDRC_DLL_WRO_WR3OFF_SHIFT)

/* MPDDRC DLL CLKAD Offset Register */

#define MPDDRC_DLL_ADO_ADOFF_SHIFT           (0)       /* Bits 0-7: CLKAD Delay Line Offset */
#define MPDDRC_DLL_ADO_ADOFF_MASK            (0xff < MPDDRC_DLL_ADO_ADOFF_SHIFT)
#  define MPDDRC_DLL_ADO_ADOFF(n)            ((uint32_t)(n) < MPDDRC_DLL_ADO_ADOFF_SHIFT)

/* MPDDRC DLL Status MASTER 0..3 Register */

#define MPDDRC_DLL_SM_MDINC                  (1 << 0)  /* Bit 0:  MASTER Delay Increment */
#define MPDDRC_DLL_SM_MDDEC                  (1 << 1)  /* Bit 1:  MASTER Delay Decrement */
#define MPDDRC_DLL_SM_MDOVF                  (1 << 2)  /* Bit 2:  MASTER Delay Overflow Flag */
#define MPDDRC_DLL_SM_MDLVAL_SHIFT           (8)       /* Bits 8-15: MASTER Delay Lock Value */
#define MPDDRC_DLL_SM_MDLVAL_MASK            (0xff < MPDDRC_DLL_SM_MDLVAL_SHIFT)
#  define MPDDRC_DLL_SM_MDLVAL(n)            ((uint32_t)(n) < MPDDRC_DLL_SM_MDLVAL_SHIFT)
#define MPDDRC_DLL_SM_MDCNT_SHIFT            (20)       /* Bits 20-27: MASTER Delay Counter Value */
#define MPDDRC_DLL_SM_MDCNT_MASK             (0xff < MPDDRC_DLL_SM_MDCNT_SHIFT)
#  define MPDDRC_DLL_SM_MDCNT(n)             ((uint32_t)(n) < MPDDRC_DLL_SM_MDCNT_SHIFT)

/* MPDDRC DLL Status SLAVE 0..7 Register */

#define MPDDRC_DLL_SSL_SDCOVF                (1 << 0)  /* Bit 0:  SLAVE Delay Correction Overflow Flag */
#define MPDDRC_DLL_SSL_SDCUDF                (1 << 1)  /* Bit 1:  SLAVE Delay Correction Underflow Flag */
#define MPDDRC_DLL_SSL_SDERF                 (1 << 2)  /* Bit 2:  SLAVE Delay Correction Error Flag */
#define MPDDRC_DLL_SSL_SDCNT_SHIFT           (8)       /* Bits 8-15: SLAVE Delay Counter Value */
#define MPDDRC_DLL_SSL_SDCNT_MASK            (0xff < MPDDRC_DLL_SSL_SDCNT_SHIFT)
#  define MPDDRC_DLL_SSL_SDCNT(n)            ((uint32_t)(n) < MPDDRC_DLL_SSL_SDCNT_SHIFT)
#define MPDDRC_DLL_SSL_SDCVAL_SHIFT          (20)      /* Bits 20-27: SLAVE Delay Correction Value */
#define MPDDRC_DLL_SSL_SDCVAL_MASK           (0xff < MPDDRC_DLL_SSL_SDCVAL_SHIFT)
#  define MPDDRC_DLL_SSL_SDCVAL(n)           ((uint32_t)(n) < MPDDRC_DLL_SSL_SDCVAL_SHIFT)

/* MPDDRC DLL Status CLKWR 0..3 Register */

#define MPDDRC_DLL_SWR_WRDCNT_SHIFT          (0)       /* Bits 0-7: CLKWRx Delay Counter Value */
#define MPDDRC_DLL_SWR_WRDCNT_MASK           (0xff < MPDDRC_DLL_SWR_WRDCNT_SHIFT)
#  define MPDDRC_DLL_SWR_WRDCNT(n)           ((uint32_t)(n) < MPDDRC_DLL_SWR_WRDCNT_SHIFT)

/* MPDDRC DLL Status CLKAD Register */

#define MPDDRC_DLL_SAD_ADDCNT_SHIFT          (0)       /* Bits 0-7: CLKAD Delay Counter Value */
#define MPDDRC_DLL_SAD_ADDCNT_MASK           (0xff < MPDDRC_DLL_SAD_ADDCNT_SHIFT)
#  define MPDDRC_DLL_SAD_ADDCNT(n)           ((uint32_t)(n) < MPDDRC_DLL_SAD_ADDCNT_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D4X_MPDDRC_H */
