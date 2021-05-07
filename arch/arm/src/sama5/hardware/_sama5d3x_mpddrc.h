/****************************************************************************
 * arch/arm/src/sama5/hardware/_sama5d3x_mpddrc.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D3X_MPDDRC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D3X_MPDDRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MPDDRC Register Offsets **************************************************/

#define SAM_MPDDRC_MR_OFFSET            0x0000 /* MPDDRC Mode Register */
#define SAM_MPDDRC_RTR_OFFSET           0x0004 /* MPDDRC Refresh Timer Register */
#define SAM_MPDDRC_CR_OFFSET            0x0008 /* MPDDRC Configuration Register */
#define SAM_MPDDRC_TPR0_OFFSET          0x000c /* MPDDRC Timing Parameter 0 Register */
#define SAM_MPDDRC_TPR1_OFFSET          0x0010 /* MPDDRC Timing Parameter 1 Register */
#define SAM_MPDDRC_TPR2_OFFSET          0x0014 /* MPDDRC Timing Parameter 2 Register */
                                               /* 0x0018 Reserved */
#define SAM_MPDDRC_LPR_OFFSET           0x001c /* MPDDRC Low-power Register */
#define SAM_MPDDRC_MD_OFFSET            0x0020 /* MPDDRC Memory Device Register */
#define SAM_MPDDRC_HS_OFFSET            0x0024 /* MPDDRC High Speed Register */
#define SAM_MPDDRC_LPDDR2_LPR_OFFSET    0x0028 /* MPDDRC LPDDR2 Low-power Register */
#define SAM_MPDDRC_LPDDR2_CALMR4_OFFSET 0x002c /* MPDDRC LPDDR2 Calibration and MR4 Register */
#define SAM_MPDDRC_LPDDR2_TIMCAL_OFFSET 0x0030 /* MPDDRC LPDDR2 Timing Calibration Register */
#define SAM_MPDDRC_IO_CALIBR_OFFSET     0x0034 /* MPDDRC IO Calibration */
#define SAM_MPDDRC_OCMS_OFFSET          0x0038 /* MPDDRC OCMS Register */
#define SAM_MPDDRC_OCMS_KEY1_OFFSET     0x003c /* MPDDRC OCMS KEY1 Register */
#define SAM_MPDDRC_OCMS_KEY2_OFFSET     0x0040 /* MPDDRC OCMS KEY2 Register */
                                               /* 0x0044-0x0070 Reserved */
#define SAM_MPDDRC_DLL_MOR_OFFSET       0x0074 /* MPDDRC DLL Master Offset Register */
#define SAM_MPDDRC_DLL_SOR_OFFSET       0x0078 /* MPDDRC DLL Slave Offset Register */
#define SAM_MPDDRC_DLL_MS_OFFSET        0x007c /* MPDDRC DLL Status Master Register */
#define SAM_MPDDRC_DLL_SS0_OFFSET       0x0080 /* MPDDRC DLL Status Slave 0 Register */
#define SAM_MPDDRC_DLL_SS1_OFFSET       0x0084 /* MPDDRC DLL Status Slave 1 Register */
#define SAM_MPDDRC_DLL_SS2_OFFSET       0x0088 /* MPDDRC DLL Status Slave 2 Register */
#define SAM_MPDDRC_DLL_SS3_OFFSET       0x008c /* MPDDRC DLL Status Slave 3 Register */
                                               /* 0x0094-0x00e0 Reserved */
#define SAM_MPDDRC_WPCR_OFFSET          0x00e4 /* MPDDRC Write Protect Control Register */
#define SAM_MPDDRC_WPSR_OFFSET          0x00e8 /* MPDDRC Write Protect Status Register */
                                               /* 0x158-0x1cc Reserved */
                                               /* 0x1dc-0x1f8 Reserved */

/* MPDDRC Register Addresses ************************************************/

#define SAM_MPDDRC_MR                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_MR_OFFSET)
#define SAM_MPDDRC_RTR                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_RTR_OFFSET)
#define SAM_MPDDRC_CR                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_CR_OFFSET)
#define SAM_MPDDRC_TPR0                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR0_OFFSET)
#define SAM_MPDDRC_TPR1                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR1_OFFSET)
#define SAM_MPDDRC_TPR2                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_TPR2_OFFSET)
#define SAM_MPDDRC_LPR                  (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPR_OFFSET)
#define SAM_MPDDRC_MD                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_MD_OFFSET)
#define SAM_MPDDRC_HS                   (SAM_MPDDRC_VBASE+SAM_MPDDRC_HS_OFFSET)
#define SAM_MPDDRC_LPDDR2_LPR           (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_LPR_OFFSET)
#define SAM_MPDDRC_LPDDR2_CALMR4        (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_CALMR4_OFFSET)
#define SAM_MPDDRC_LPDDR2_TIMCAL        (SAM_MPDDRC_VBASE+SAM_MPDDRC_LPDDR2_TIMCAL_OFFSET)
#define SAM_MPDDRC_IO_CALIBR            (SAM_MPDDRC_VBASE+SAM_MPDDRC_IO_CALIBR_OFFSET)
#define SAM_MPDDRC_OCMS                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_OFFSET)
#define SAM_MPDDRC_OCMS_KEY1            (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_KEY1_OFFSET)
#define SAM_MPDDRC_OCMS_KEY2            (SAM_MPDDRC_VBASE+SAM_MPDDRC_OCMS_KEY2_OFFSET)
#define SAM_MPDDRC_DLL_MOR              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_MOR_OFFSET)
#define SAM_MPDDRC_DLL_SOR              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SOR_OFFSET)
#define SAM_MPDDRC_DLL_MS               (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_MS_OFFSET)
#define SAM_MPDDRC_DLL_SS0              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SS0_OFFSET)
#define SAM_MPDDRC_DLL_SS1              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SS1_OFFSET)
#define SAM_MPDDRC_DLL_SS2              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SS2_OFFSET)
#define SAM_MPDDRC_DLL_SS3              (SAM_MPDDRC_VBASE+SAM_MPDDRC_DLL_SS3_OFFSET)
#define SAM_MPDDRC_WPCR                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_WPCR_OFFSET)
#define SAM_MPDDRC_WPSR                 (SAM_MPDDRC_VBASE+SAM_MPDDRC_WPSR_OFFSET)

/* MPDDRC Register Bit Definitions ******************************************/

/* MPDDRC Mode Register */

#define MPDDRC_MR_MODE_SHIFT            (0)       /* Bits 0-2: MPDDRC Command Mode */
#define MPDDRC_MR_MODE_MASK             (7 << MPDDRC_MR_MODE_SHIFT)
#  define MPDDRC_MR_MODE_NORMAL         (0 << MPDDRC_MR_MODE_SHIFT) /* Normal Mode */
#  define MPDDRC_MR_MODE_NOP            (1 << MPDDRC_MR_MODE_SHIFT) /* NOP when device accessed */
#  define MPDDRC_MR_MODE_PRCGALL        (2 << MPDDRC_MR_MODE_SHIFT) /* 'All Banks Precharge' when device accessed */
#  define MPDDRC_MR_MODE_LMR            (3 << MPDDRC_MR_MODE_SHIFT) /* 'Load Mode Register' command when device accessed */
#  define MPDDRC_MR_MODE_RFSH           (4 << MPDDRC_MR_MODE_SHIFT) /* 'Auto-Refresh' when device accessed */
#  define MPDDRC_MR_MODE_EXTLMR         (5 << MPDDRC_MR_MODE_SHIFT) /* 'Extended Load Mode Register' when device accessed */
#  define MPDDRC_MR_MODE_DEEP           (6 << MPDDRC_MR_MODE_SHIFT) /* Deep power mode */
#  define MPDDRC_MR_MODE_LPDDR2         (7 << MPDDRC_MR_MODE_SHIFT) /* LPDDR2 Mode Register' when device accessed */

#define MPDDRC_MR_MRS_SHIFT             (8)       /* Bits 8-15: Mode Register Select LPDDR2 */
#define MPDDRC_MR_MRS_MASK              (0xff << MPDDRC_MR_MRS_SHIFT)
#  define MPDDRC_MR_MRS(n)              ((n) << MPDDRC_MR_MRS_SHIFT)

/* MPDDRC Refresh Timer Register */

#define MPDDRC_RTR_COUNT_SHIFT          (0)       /* Bits 0-11: MPDDRC Refresh Timer Count */
#define MPDDRC_RTR_COUNT_MASK           (0xfff << MPDDRC_RTR_COUNT_SHIFT)
#  define MPDDRC_RTR_COUNT(n)           ((n) << MPDDRC_RTR_COUNT_SHIFT)
#define MPDDRC_RTR_ADJ_REF              (1 << 16) /* Bit 16: Adjust Refresh Rate */
#define MPDDRC_RTR_REF_PB               (1 << 17) /* Bit 17: Refresh Per Bank */
#define MPDDRC_RTR_MR4_VALUE_SHIFT      (20)      /* Bits 20-22: Content of MR4 Register */
#define MPDDRC_RTR_MR4_VALUE_MASK       (7 << MPDDRC_RTR_MR4_VALUE_SHIFT)
#  define MPDDRC_RTR_MR4_VALUE(n)       ((n) << MPDDRC_RTR_MR4_VALUE_SHIFT)

/* MPDDRC Configuration Register */

#define MPDDRC_CR_NC_SHIFT              (0)       /* Bits 0-1: Number of Column Bits */
#define MPDDRC_CR_NC_MASK               (3 << MPDDRC_CR_NC_SHIFT)
#  define MPDDRC_CR_NC_9                (0 << MPDDRC_CR_NC_SHIFT) /* 9 DDR column bits */
#  define MPDDRC_CR_NC_10               (1 << MPDDRC_CR_NC_SHIFT) /* 10 DDR column bits */
#  define MPDDRC_CR_NC_11               (2 << MPDDRC_CR_NC_SHIFT) /* 11 DDR column bits */
#  define MPDDRC_CR_NC_12               (3 << MPDDRC_CR_NC_SHIFT) /* 12 DDR column bits */

#define MPDDRC_CR_NR_SHIFT              (2)       /* Bits 2-3: Number of Row Bits */
#define MPDDRC_CR_NR_MASK               (3 << MPDDRC_CR_NR_SHIFT)
#  define MPDDRC_CR_NR_11               (0 << MPDDRC_CR_NR_SHIFT) /* 00 ROW_11 11 row bits */
#  define MPDDRC_CR_NR_12               (1 << MPDDRC_CR_NR_SHIFT) /* 01 ROW_12 12 row bits */
#  define MPDDRC_CR_NR_13               (2 << MPDDRC_CR_NR_SHIFT) /* 10 ROW_13 13 row bits */
#  define MPDDRC_CR_NR_14               (3 << MPDDRC_CR_NR_SHIFT) /* 11 ROW_14 14 row bits */

#define MPDDRC_CR_CAS_SHIFT             (4)       /* Bits 4-6: CAS Latency */
#define MPDDRC_CR_CAS_MASK              (7 << MPDDRC_CR_CAS_SHIFT)
#  define MPDDRC_CR_CAS_2               (2 << MPDDRC_CR_CAS_SHIFT) /* 010 DDR_CAS2 LPDDR1 CAS Latency 2 */
#  define MPDDRC_CR_CAS_3               (3 << MPDDRC_CR_CAS_SHIFT) /* 011 DDR_CAS3 DDR2/LPDDR2/LPDDR1 CAS Latency 3 */
#  define MPDDRC_CR_CAS_4               (4 << MPDDRC_CR_CAS_SHIFT) /* 100 DDR_CAS4 DDR2/LPDDR2 CAS Latency 4 */
#  define MPDDRC_CR_CAS_5               (5 << MPDDRC_CR_CAS_SHIFT) /* 101 DDR_CAS5 DDR2/LPDDR2 CAS Latency 5 */
#  define MPDDRC_CR_CAS_6               (6 << MPDDRC_CR_CAS_SHIFT) /* 110 DDR_CAS6 DDR2 CAS Latency 6 */

#define MPDDRC_CR_DLL                   (1 << 7)  /* Bit 7:  Reset DLL */
#define MPDDRC_CR_DIC_DS                (1 << 8)  /* Bit 8:  Output Driver Impedance Control (Drive Strength) */
#define MPDDRC_CR_DIS_DLL               (1 << 9)  /* Bit 9:  Disable DLL */
#define MPDDRC_CR_ZQ_SHIFT              (10)      /* Bits 10-11: ZQ Calibration */
#define MPDDRC_CR_ZQ_MASK               (3 << MPDDRC_CR_ZQ_SHIFT)
#  define MPDDRC_CR_ZQ_INIT             (0 << MPDDRC_CR_ZQ_SHIFT) /* Calibration command after initialization */
#  define MPDDRC_CR_ZQ_LONG             (1 << MPDDRC_CR_ZQ_SHIFT) /* Long calibration */
#  define MPDDRC_CR_ZQ_SHORT            (2 << MPDDRC_CR_ZQ_SHIFT) /* Short calibration */
#  define MPDDRC_CR_ZQ_RESET            (3 << MPDDRC_CR_ZQ_SHIFT) /* ZQ Reset */

#define MPDDRC_CR_OCD_SHIFT             (12)      /* Bits 12-14: Off-chip Driver */
#define MPDDRC_CR_OCD_MASK              (7 << MPDDRC_CR_OCD_SHIFT)
#  define MPDDRC_CR_OCD_EXIT            (0 << MPDDRC_CR_OCD_SHIFT) /* OCD calibration mode exit, maintain setting */
#  define MPDDRC_CR_OCD_DEFAULT         (7 << MPDDRC_CR_OCD_SHIFT) /* OCD calibration default */

#define MPDDRC_CR_DQMS                  (1 << 16) /* Bit 16: Mask Data is Shared */
#define MPDDRC_CR_ENRDM                 (1 << 17) /* Bit 17: Enable Read Measure */
#define MPDDRC_CR_NB                    (1 << 20) /* Bit 20: Number of Banks */

#  define MPDDRC_CR_4BANKS              (0)          /* 4 banks */
#  define MPDDRC_CR_8BANKS              MPDDRC_CR_NB /* 8 banks */

#define MPDDRC_CR_NDQS                  (1 << 21) /* Bit 21: Not DQS */
#define MPDDRC_CR_DECOD                 (1 << 22) /* Bit 22: Type of Decoding */
#define MPDDRC_CR_UNAL                  (1 << 23) /* Bit 23: Support Unaligned Access */

/* MPDDRC Timing Parameter 0 Register */

#define MPDDRC_TPR0_TRAS_SHIFT          (0)       /* Bits 0-3: Active to Precharge Delay */
#define MPDDRC_TPR0_TRAS_MASK           (15 << MPDDRC_TPR0_TRAS_SHIFT)
#  define MPDDRC_TPR0_TRAS(n)           ((n) << MPDDRC_TPR0_TRAS_SHIFT)
#define MPDDRC_TPR0_TRCD_SHIFT          (4)       /* Bits 4-7: Row to Column Delay */
#define MPDDRC_TPR0_TRCD_MASK           (15 << MPDDRC_TPR0_TRCD_SHIFT)
#  define MPDDRC_TPR0_TRCD(n)           ((n) << MPDDRC_TPR0_TRCD_SHIFT)
#define MPDDRC_TPR0_TWR_SHIFT           (8)       /* Bits 8-11: Write Recovery Delay */
#define MPDDRC_TPR0_TWR_MASK            (15 << MPDDRC_TPR0_TWR_SHIFT)
#  define MPDDRC_TPR0_TWR(n)            ((n) << MPDDRC_TPR0_TWR_SHIFT)
#define MPDDRC_TPR0_TRC_SHIFT           (12)      /* Bits 12-15: Row Cycle Delay */
#define MPDDRC_TPR0_TRC_MASK            (15 << MPDDRC_TPR0_TRC_SHIFT)
#  define MPDDRC_TPR0_TRC(n)            ((n) << MPDDRC_TPR0_TRC_SHIFT)
#define MPDDRC_TPR0_TRP_SHIFT           (16)      /* Bits 16-19: Row Precharge Delay */
#define MPDDRC_TPR0_TRP_MASK            (15 << MPDDRC_TPR0_TRP_SHIFT)
#  define MPDDRC_TPR0_TRP(n)            ((n) << MPDDRC_TPR0_TRP_SHIFT)
#define MPDDRC_TPR0_TRRD_SHIFT          (20)      /* Bits 20-23: Active BankA to Active BankB */
#define MPDDRC_TPR0_TRRD_MASK           (15 << MPDDRC_TPR0_TRRD_SHIFT)
#  define MPDDRC_TPR0_TRRD(n)           ((n) << MPDDRC_TPR0_TRRD_SHIFT)
#define MPDDRC_TPR0_TWTR_SHIFT          (24)      /* Bits 24-26: Internal Write to Read Delay */
#define MPDDRC_TPR0_TWTR_MASK           (7 << MPDDRC_TPR0_TWTR_SHIFT)
#  define MPDDRC_TPR0_TWTR(n)           ((n) << MPDDRC_TPR0_TWTR_SHIFT)
#define MPDDRC_TPR0_RDC_WRRD            (1 << 27) /* Bit 27: Reduce Write to Read Delay */
#define MPDDRC_TPR0_TMRD_SHIFT          (28)      /* Bits 18-31: Load Mode Register Command to Activate or Refresh Command */
#define MPDDRC_TPR0_TMRD_MASK           (15 << MPDDRC_TPR0_TMRD_SHIFT)
#  define MPDDRC_TPR0_TMRD(n)           ((n) << MPDDRC_TPR0_TMRD_SHIFT)

/* MPDDRC Timing Parameter 1 Register */

#define MPDDRC_TPR1_TRFC_SHIFT          (0)       /* Bits 0-6: Row Cycle Delay */
#define MPDDRC_TPR1_TRFC_MASK           (0x7f << MPDDRC_TPR1_TRFC_SHIFT)
#  define MPDDRC_TPR1_TRFC(n)           ((n) << MPDDRC_TPR1_TRFC_SHIFT)
#define MPDDRC_TPR1_TXSNR_SHIFT         (8)       /* Bits 8-15: Exit Self Refresh Delay to Non Read Command */
#define MPDDRC_TPR1_TXSNR_MASK          (0xff << MPDDRC_TPR1_TXSNR_SHIFT)
#  define MPDDRC_TPR1_TXSNR(n)          ((n) << MPDDRC_TPR1_TXSNR_SHIFT)
#define MPDDRC_TPR1_TXSRD_SHIFT         (16)      /* Bits 16-23: Exit Self Refresh Delay to Read Command */
#define MPDDRC_TPR1_TXSRD_MASK          (0xff << MPDDRC_TPR1_TXSRD_SHIFT)
#  define MPDDRC_TPR1_TXSRD(n)          ((n) << MPDDRC_TPR1_TXSRD_SHIFT)
#define MPDDRC_TPR1_TXP_SHIFT           (24)      /* Bits 24-27: Exit Power-down Delay to First Command */
#define MPDDRC_TPR1_TXP_MASK            (15 << MPDDRC_TPR1_TXP_SHIFT)
#  define MPDDRC_TPR1_TXP(n)            ((n) << MPDDRC_TPR1_TXP_SHIFT)

/* MPDDRC Timing Parameter 2 Register */

#define MPDDRC_TPR2_TXARD_SHIFT         (0)       /* Bits 0-3: Exit Active Power Down Delay to Read Command in Mode 'Fast Exit' */
#define MPDDRC_TPR2_TXARD_MASK          (15 << MPDDRC_TPR2_TXARD_SHIFT)
#  define MPDDRC_TPR2_TXARD(n)          ((n) << MPDDRC_TPR2_TXARD_SHIFT)
#define MPDDRC_TPR2_TXARDS_SHIFT        (4)       /* Bits 4-7: Exit Active Power Down Delay to Read Command in Mode 'Slow Exit' */
#define MPDDRC_TPR2_TXARDS_MASK         (15 << MPDDRC_TPR2_TXARDS_SHIFT)
#  define MPDDRC_TPR2_TXARDS(n)         ((n) << MPDDRC_TPR2_TXARDS_SHIFT)
#define MPDDRC_TPR2_TRPA_SHIFT          (8)       /* Bits 8-11: Row Precharge All Delay */
#define MPDDRC_TPR2_TRPA_MASK           (15 << MPDDRC_TPR2_TRPA_SHIFT)
#  define MPDDRC_TPR2_TRPA(n)           ((n) << MPDDRC_TPR2_TRPA_SHIFT)
#define MPDDRC_TPR2_TRTP_SHIFT          (12)      /* Bits 12-14: Read to Precharge */
#define MPDDRC_TPR2_TRTP_MASK           (7 << MPDDRC_TPR2_TRTP_SHIFT)
#  define MPDDRC_TPR2_TRTP(n)           ((n) << MPDDRC_TPR2_TRTP_SHIFT)
#define MPDDRC_TPR2_TFAW_SHIFT          (16)      /* Bits 16-19: Four Active Windows */
#define MPDDRC_TPR2_TFAW_MASK           (15 << MPDDRC_TPR2_TFAW_SHIFT)
#  define MPDDRC_TPR2_TFAW(n)           ((n) << MPDDRC_TPR2_TFAW_SHIFT)

/* MPDDRC Low-power Register */

#define MPDDRC_LPR_LPCB_SHIFT           (0)       /* Bits 0-1: Low-power Command*/
#define MPDDRC_LPR_LPCB_MASK            (3 << MPDDRC_LPR_LPCB_SHIFT)
#  define MPDDRC_LPR_LPCB_DISABLED      (0 << MPDDRC_LPR_LPCB_SHIFT) /* Low-power Feature is inhibited */
#  define MPDDRC_LPR_LPCB_SELFREFRESH   (1 << MPDDRC_LPR_LPCB_SHIFT) /* Issues a 'Self Refresh' to device, clocks deactivated */
#  define MPDDRC_LPR_LPCB_POWERDOWN     (2 << MPDDRC_LPR_LPCB_SHIFT) /* Issues a 'Power-down' to device after each access */
#  define MPDDRC_LPR_LPCB_DEEPPWD       (3 << MPDDRC_LPR_LPCB_SHIFT) /* TIssues a 'Deep Power-down' to Low-power device */

#define MPDDRC_LPR_CLK_FR               (1 << 2)  /* Bit 2:  Clock Frozen Command */
#define MPDDRC_LPR_LPDDR2_PWOFF         (1 << 3)  /* Bit 3:  LPDDR2 Power Off */
#define MPDDRC_LPR_PASR_SHIFT           (4)       /* Bits 4-6: Partial Array Self Refresh */
#define MPDDRC_LPR_PASR_MASK            (7 << MPDDRC_LPR_PASR_SHIFT)
#  define MPDDRC_LPR_PASR(n)            ((n) << MPDDRC_LPR_PASR_SHIFT)
#define MPDDRC_LPR_DS_SHIFT             (8)       /* Bits 8-10: Drive Strength */
#define MPDDRC_LPR_DS_MASK              (7 << MPDDRC_LPR_DS_SHIFT)
#  define MPDDRC_LPR_DS(n)              ((n) << MPDDRC_LPR_DS_SHIFT)
#define MPDDRC_LPR_TIMEOUT_SHIFT        (12)      /* Bits 12-13: Enter Low-power Mode */
#define MPDDRC_LPR_TIMEOUT_MASK         (3 << MPDDRC_LPR_TIMEOUT_SHIFT)
#  define MPDDRC_LPR_TIMEOUT_0CLKS      (0 << MPDDRC_LPR_TIMEOUT_SHIFT) /* Activates low-power mode after the end of transfer */
#  define MPDDRC_LPR_TIMEOUT_64CLKS     (1 << MPDDRC_LPR_TIMEOUT_SHIFT) /* Activates low-power mode 64 clocks after the end of transfer */
#  define MPDDRC_LPR_TIMEOUT_128CLKS    (2 << MPDDRC_LPR_TIMEOUT_SHIFT) /* 28 Activates low-power mode 128 clocks after the end of transfer */

#define MPDDRC_LPR_APDE                 (1 << 16) /* Bit 16:  ctive Power Down Exit Time */
#  define MPDDRC_LPR_APDE_FAST          (0)
#  define MPDDRC_LPR_APDE_SLOW          MPDDRC_LPR_APDE
#define MPDDRC_LPR_UPD_MR_SHIFT         (20)      /* Bits 20-21: Update Load Mode Register and Extended Mode Register */
#define MPDDRC_LPR_UPD_MR_MASK          (3 << MPDDRC_LPR_UPD_MR_SHIFT)
#  define MPDDRC_LPR_UPD_MR_DISABLED    (0 << MPDDRC_LPR_UPD_MR_SHIFT) /* DISABLED Update is disabled */
#  define MPDDRC_LPR_UPD_MR_SHARED      (1 << MPDDRC_LPR_UPD_MR_SHIFT) /* MPDDRC shares external bus */
#  define MPDDRC_LPR_UPD_MR_UNSHARED    (2 << MPDDRC_LPR_UPD_MR_SHIFT) /* MPDDRC does not share external bus */

/* MPDDRC Memory Device Register */

#define MPDDRC_MD_SHIFT                 (0)       /* Bits 0-2: Memory Device */
#define MPDDRC_MD_MASK                  (7 << MPDDRC_MD_SHIFT)
#  define MPDDRC_MD_LPDDR_SDRAM         (2 << MPDDRC_MD_SHIFT) /* Low-power DDR1-SDRAM */
#  define MPDDRC_MD_DDR2_SDRAM          (6 << MPDDRC_MD_SHIFT) /* DDR2-SDRAM */
#  define MPDDRC_MD_LPDDR2_SDRAM        (7 << MPDDRC_MD_SHIFT) /* Low-Power DDR2-SDRAM */

#define MPDDRC_MD_DBW                   (1 << 4)  /* Bit 4:  Data Bus Width */
#  define MPDDRC_MD_DBW32               (0)       /* Data bus width is 32 bits */

#  define MPDDRC_MD_DBW16               MPDDRC_MD_DBW /* Data bus width is 16 bits */

/* MPDDRC High Speed Register */

#define MPDDRC_HS_DIS_ANTICIP_READ      (1 << 2) /* Bit 2:  Disable Anticip Read Access */
#define MPDDRC_HS_AUTOREFRESH_CAL       (1 << 5) /* Bit 5:  calibration during autorefresh (REVISIT) */

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
#define MPDDRC_LPDDR2_LPR_SR_SHIFT           (28) /* Bits 28-31: Slew Rate (REVISIT) */
#define MPDDRC_LPDDR2_LPR_SR_MASK            (15 << MPDDRC_LPDDR2_LPR_SR_SHIFT)
#  define MPDDRC_LPDDR2_LPR_SR(n)            ((n) << MPDDRC_LPDDR2_LPR_SR_SHIFT)

/* MPDDRC LPDDR2 Calibration and MR4 Register */

#define MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT (0)  /* Bits 0-15: LPDDR2 Calibration Timer Count */
#define MPDDRC_LPDDR2_CALMR4_COUNT_CAL_MASK  (0xffff << MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT)
#  define MPDDRC_LPDDR2_CALMR4_COUNT_CAL(n)  ((n) << MPDDRC_LPDDR2_CALMR4_COUNT_CAL_SHIFT)
#define MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT  (16) /* Bits 16-31: Mode Register 4 Read Interval */
#define MPDDRC_LPDDR2_CALMR4_MR4_READ_MASK   (0xffff << MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT)
#  define MPDDRC_LPDDR2_CALMR4_MR4_READ(n)   ((n) << MPDDRC_LPDDR2_CALMR4_MR4_READ_SHIFT)

/* MPDDRC LPDDR2 Timing Calibration Register */

#define MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT (0)       /* Bits 0-7: ZQ Calibration Short */
#define MPDDRC_LPDDR2_TIMCAL_ZQCS_MASK  (0xff << MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT)
#  define MPDDRC_LPDDR2_TIMCAL_ZQCS(n)  ((n) << MPDDRC_LPDDR2_TIMCAL_ZQCS_SHIFT)

/* MPDDRC IO Calibration */

#define MPDDRC_IO_CALIBR_RDIV_SHIFT     (0)       /* Bits 0-2: Resistor Divider, Output Driver Impedance */
#define MPDDRC_IO_CALIBR_RDIV_MASK      (7 << MPDDRC_IO_CALIBR_RDIV_SHIFT)
#  define MPDDRC_IO_CALIBR_RZQ34_NA     (1 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2 RZQ = 34.3 Ohm DDR2/LPDDR1: Not applicable */
#  define MPDDRC_IO_CALIBR_RZQ40_33     (2 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 40 Ohm   DDR2/LPDDR1: RZQ = 33.3 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ48_40     (3 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 48 Ohm   DDR2/LPDDR1: RZQ = 40 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ60_50     (4 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 60 Ohm   DDR2/LPDDR1: RZQ = 50 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ80_67     (6 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2 RZQ = 80 Ohm   DDR2/LPDDR1: RZQ = 66.7 Ohm */
#  define MPDDRC_IO_CALIBR_RZQ120_100   (7 << MPDDRC_IO_CALIBR_RDIV_SHIFT) /* LPDDR2:RZQ = 120 Oh m DDR2/LPDDR1: RZQ = 100 Ohm */

#define MPDDRC_IO_CALIBR_TZQIO_SHIFT    (8)       /* Bits 8-10: IO Calibration */
#define MPDDRC_IO_CALIBR_TZQIO_MASK     (7 << MPDDRC_IO_CALIBR_TZQIO_SHIFT)
#  define MPDDRC_IO_CALIBR_TZQIO(n)     ((n) << MPDDRC_IO_CALIBR_TZQIO_SHIFT)
#define MPDDRC_IO_CALIBR_CALCODEP_SHIFT (16)      /* Bits 16-19: Number of Transistor P */
#define MPDDRC_IO_CALIBR_CALCODEP_MASK  (15 << MPDDRC_IO_CALIBR_CALCODEP_SHIFT)
#  define MPDDRC_IO_CALIBR_CALCODEP(n)  ((n) << MPDDRC_IO_CALIBR_CALCODEP_SHIFT)
#define MPDDRC_IO_CALIBR_CALCODEN_SHIFT (20)      /* Bits 20-23: Number of Transistor N */
#define MPDDRC_IO_CALIBR_CALCODEN_MASK  (15 << MPDDRC_IO_CALIBR_CALCODEN_SHIFT)
#  define MPDDRC_IO_CALIBR_CALCODEN(n)  ((n) << MPDDRC_IO_CALIBR_CALCODEN_SHIFT)

/* MPDDRC OCMS Register */

#define MPDDRC_OCMS_SCR_EN              (1 << 0)  /* Bit 0:  Scrambling enable */

/* MPDDRC OCMS KEY1 Register (32-bit key value) */

/* MPDDRC OCMS KEY2 Register (32-bit key value) */

/* MPDDRC DLL Master Offset Register */

#define MPDDRC_DLL_MOR_MOFF_SHIFT       (0)       /* Bits 0-3: DLL Master Delay Line Offset */
#define MPDDRC_DLL_MOR_MOFF_MASK        (15 << MPDDRC_DLL_MOR_MOFF_SHIFT)
#  define MPDDRC_DLL_MOR_MOFF(n)        ((n) << MPDDRC_DLL_MOR_MOFF_SHIFT)
#define MPDDRC_DLL_MOR_CLK90OFF_SHIFT   (8)       /* Bits 8-12: DLL CLK90 Delay Line Offset */
#define MPDDRC_DLL_MOR_CLK90OFF_MASK    (31 << MPDDRC_DLL_MOR_CLK90OFF_SHIFT)
#  define MPDDRC_DLL_MOR_CLK90OFF(n)    ((n) << MPDDRC_DLL_MOR_CLK90OFF_SHIFT)
#define MPDDRC_DLL_MOR_SELOFF           (1 << 16) /* Bit 16: DLL Offset Selection */
#define MPDDRC_DLL_MOR_KEY_SHIFT        (24)      /* Bits 24-31: DLL CLK90 Delay Line Offset (REVISIT) */
#define MPDDRC_DLL_MOR_KEY_MASK         (0xff << MPDDRC_DLL_MOR_KEY_SHIFT)
#  define MPDDRC_DLL_MOR_KEY            (0xc5 << MPDDRC_DLL_MOR_KEY_SHIFT)

/* MPDDRC DLL Slave Offset Register */

#define MPDDRC_DLL_SOR_S0OFF_SHIFT      (0)     /* Bits 0-4: DLL Slave 0 Delay Line Offset */
#define MPDDRC_DLL_SOR_S0OFF_MASK       (31 << MPDDRC_DLL_SOR_S0OFF_SHIFT)
#  define MPDDRC_DLL_SOR_S0OFF(n)       ((n) << MPDDRC_DLL_SOR_S0OFF_SHIFT)
#define MPDDRC_DLL_SOR_S1OFF_SHIFT      (8)     /* Bits 8-12: DLL Slave 1 Delay Line Offset */
#define MPDDRC_DLL_SOR_S1OFF_MASK       (31 << MPDDRC_DLL_SOR_S1OFF_SHIFT)
#  define MPDDRC_DLL_SOR_S1OFF(n)       ((n) << MPDDRC_DLL_SOR_S1OFF_SHIFT)
#define MPDDRC_DLL_SOR_S2OFF_SHIFT      (16)     /* Bits 16-20: DLL Slave 2 Delay Line Offset */
#define MPDDRC_DLL_SOR_S2OFF_MASK       (31 << MPDDRC_DLL_SOR_S2OFF_SHIFT)
#  define MPDDRC_DLL_SOR_S2OFF(n)       ((n) << MPDDRC_DLL_SOR_S2OFF_SHIFT)
#define MPDDRC_DLL_SOR_S3OFF_SHIFT      (24)     /* Bits 24-28: DLL Slave 3 Delay Line Offset */
#define MPDDRC_DLL_SOR_S3OFF_MASK       (31 << MPDDRC_DLL_SOR_S3OFF_SHIFT)
#  define MPDDRC_DLL_SOR_S3OFF(n)       ((n) << MPDDRC_DLL_SOR_S3OFF_SHIFT)

/* MPDDRC DLL Status Master Register */

#define MPDDRC_DLL_MS_MDINC             (1 << 0)  /* Bit 0:  DLL Master Delay Increment */
#define MPDDRC_DLL_MS_MDDEC             (1 << 1)  /* Bit 1:  DLL Master Delay Decrement */
#define MPDDRC_DLL_MS_MDOVF             (1 << 2)  /* Bit 2:  DLL Master Delay Overflow */
#define MPDDRC_DLL_MS_MDVAL_SHIFT       (8)       /* Bits 8-15: DLL Master Delay Value */
#define MPDDRC_DLL_MS_MDVAL_MASK        (0xff << MPDDRC_DLL_MS_MDVAL_SHIFT)

/* MPDDRC DLL Status Slave 0-3 Registers */

#define MPDDRC_DLL_SS_SDCOVF            (1 << 0)  /* Bit 0:  DLL Slave x Delay Correction Overflow */
#define MPDDRC_DLL_SS_SDCUDF            (1 << 1)  /* Bit 1:  DLL Slave x Delay Correction Underflow */
#define MPDDRC_DLL_SS_SDERF             (1 << 2)  /* Bit 2:  DLL Slave x Delay Correction Error */
#define MPDDRC_DLL_SS_SDVAL_SHIFT       (8)       /* Bits 8-15: DLL Slave x Delay Value */
#define MPDDRC_DLL_SS_SDVAL_MASK        (0xff << MPDDRC_DLL_SS_SDVAL_SHIFT)
#define MPDDRC_DLL_SS_SDCVAL_SHIFT      (16)      /* Bits 16-23: DLL Slave x Delay Correction Value */
#define MPDDRC_DLL_SS_SDCVAL_MASK       (0xff << MPDDRC_DLL_SS_SDCVAL_SHIFT)

/* MPDDRC Write Protect Control Register */

#define MPDDRC_WPCR_WPEN                (1 << 0)  /* Bit 0:  Write Protection Enable */
#define MPDDRC_WPCR_WPKEY_SHIFT         (8)       /* Bits 8-31: Write Protection KEY */
#define MPDDRC_WPCR_WPKEY_MASK          (0x00ffffff << MPDDRC_WPCR_WPKEY_SHIFT)
#  define MPDDRC_WPCR_WPKEY             (0x00444452 << MPDDRC_WPCR_WPKEY_SHIFT)

/* MPDDRC Write Protect Status Register */

#define MPDDRC_WPSR_WPVS                (1 << 0)  /* Bit 0:  Write Protection Enable */
#define MPDDRC_WPSR_WPVSRC_SHIFT        (8)       /* Bits 8-23: Write Protection Violation Source */
#define MPDDRC_WPSR_WPVSRC_MASK         (0xffff << MPDDRC_WPSR_WPVSRC_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE__SAMA5D3X_MPDDRC_H */
