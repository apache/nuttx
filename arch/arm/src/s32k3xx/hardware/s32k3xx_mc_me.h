/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mc_me.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_ME_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_ME_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MC_ME Register Offsets ***************************************************/

#define S32K3XX_MC_ME_CTL_KEY_OFFSET           (0x0000) /* Control Key Register (CTL_KEY) */
#define S32K3XX_MC_ME_MODE_CONF_OFFSET         (0x0004) /* Mode Configuration Register (MODE_CONF) */
#define S32K3XX_MC_ME_MODE_UPD_OFFSET          (0x0008) /* Mode Update Register (MODE_UPD) */
#define S32K3XX_MC_ME_MODE_STAT_OFFSET         (0x000c) /* Mode Status Register (MODE_STAT) */
#define S32K3XX_MC_ME_MAIN_COREID_OFFSET       (0x0010) /* Main Core ID Register (MAIN_COREID) */
#define S32K3XX_MC_ME_PRTN0_PCONF_OFFSET       (0x0100) /* Partition 0 Process Configuration Register (PRTN0_PCONF) */
#define S32K3XX_MC_ME_PRTN0_PUPD_OFFSET        (0x0104) /* Partition 0 Process Update Register (PRTN0_PUPD) */
#define S32K3XX_MC_ME_PRTN0_STAT_OFFSET        (0x0108) /* Partition 0 Status Register (PRTN0_STAT) */
#define S32K3XX_MC_ME_PRTN0_COFB1_STAT_OFFSET  (0x0114) /* Partition 0 COFB Set 1 Clock Status Register (PRTN0_COFB1_STAT) */
#define S32K3XX_MC_ME_PRTN0_COFB1_CLKEN_OFFSET (0x0134) /* Partition 0 COFB Set 1 Clock Enable Register (PRTN0_COFB1_CLKEN) */
#define S32K3XX_MC_ME_PRTN0_CORE0_PCONF_OFFSET (0x0140) /* Partition 0 Core 0 Process Configuration Register (PRTN0_CORE0_PCONF) */
#define S32K3XX_MC_ME_PRTN0_CORE0_PUPD_OFFSET  (0x0144) /* Partition 0 Core 0 Process Update Register (PRTN0_CORE0_PUPD) */
#define S32K3XX_MC_ME_PRTN0_CORE0_STAT_OFFSET  (0x0148) /* Partition 0 Core 0 Status Register (PRTN0_CORE0_STAT) */
#define S32K3XX_MC_ME_PRTN0_CORE0_ADDR_OFFSET  (0x014c) /* Partition 0 Core 0 Address Register (PRTN0_CORE0_ADDR) */
#define S32K3XX_MC_ME_PRTN0_CORE1_PCONF_OFFSET (0x0160) /* Partition 0 Core 1 Process Configuration Register (PRTN0_CORE1_PCONF) */
#define S32K3XX_MC_ME_PRTN0_CORE1_PUPD_OFFSET  (0x0164) /* Partition 0 Core 1 Process Update Register (PRTN0_CORE1_PUPD) */
#define S32K3XX_MC_ME_PRTN0_CORE1_STAT_OFFSET  (0x0168) /* Partition 0 Core 1 Status Register (PRTN0_CORE1_STAT) */
#define S32K3XX_MC_ME_PRTN0_CORE1_ADDR_OFFSET  (0x016c) /* Partition 0 Core 1 Address Register (PRTN0_CORE1_ADDR) */
#define S32K3XX_MC_ME_PRTN0_CORE2_STAT_OFFSET  (0x0188) /* Partition 0 Core 2 Status Register (PRTN0_CORE2_STAT) */
#define S32K3XX_MC_ME_PRTN0_CORE2_ADDR_OFFSET  (0x018c) /* Partition 0 Core 2 Address Register (PRTN0_CORE2_ADDR) */
#define S32K3XX_MC_ME_PRTN1_PCONF_OFFSET       (0x0300) /* Partition 1 Process Configuration Register (PRTN1_PCONF) */
#define S32K3XX_MC_ME_PRTN1_PUPD_OFFSET        (0x0304) /* Partition 1 Process Update Register (PRTN1_PUPD) */
#define S32K3XX_MC_ME_PRTN1_STAT_OFFSET        (0x0308) /* Partition 1 Status Register (PRTN1_STAT) */
#define S32K3XX_MC_ME_PRTN1_COFB0_STAT_OFFSET  (0x0310) /* Partition 1 COFB Set 0 Clock Status Register (PRTN1_COFB0_STAT) */
#define S32K3XX_MC_ME_PRTN1_COFB1_STAT_OFFSET  (0x0314) /* Partition 1 COFB Set 1 Clock Status Register (PRTN1_COFB1_STAT) */
#define S32K3XX_MC_ME_PRTN1_COFB2_STAT_OFFSET  (0x0318) /* Partition 1 COFB Set 2 Clock Status Register (PRTN1_COFB2_STAT) */
#define S32K3XX_MC_ME_PRTN1_COFB3_STAT_OFFSET  (0x031c) /* Partition 1 COFB Set 3 Clock Status Register (PRTN1_COFB3_STAT) */
#define S32K3XX_MC_ME_PRTN1_COFB0_CLKEN_OFFSET (0x0330) /* Partition 1 COFB Set 0 Clock Enable Register (PRTN1_COFB0_CLKEN) */
#define S32K3XX_MC_ME_PRTN1_COFB1_CLKEN_OFFSET (0x0334) /* Partition 1 COFB Set 1 Clock Enable Register (PRTN1_COFB1_CLKEN) */
#define S32K3XX_MC_ME_PRTN1_COFB2_CLKEN_OFFSET (0x0338) /* Partition 1 COFB Set 2 Clock Enable Register (PRTN1_COFB2_CLKEN) */
#define S32K3XX_MC_ME_PRTN1_COFB3_CLKEN_OFFSET (0x033c) /* Partition 1 COFB Set 3 Clock Enable Register (PRTN1_COFB3_CLKEN) */
#define S32K3XX_MC_ME_PRTN2_PCONF_OFFSET       (0x0500) /* Partition 2 Process Configuration Register (PRTN2_PCONF) */
#define S32K3XX_MC_ME_PRTN2_PUPD_OFFSET        (0x0504) /* Partition 2 Process Update Register (PRTN2_PUPD) */
#define S32K3XX_MC_ME_PRTN2_STAT_OFFSET        (0x0508) /* Partition 2 Status Register (PRTN2_STAT) */
#define S32K3XX_MC_ME_PRTN2_COFB0_STAT_OFFSET  (0x0510) /* Partition 2 COFB Set 0 Clock Status Register (PRTN2_COFB0_STAT) */
#define S32K3XX_MC_ME_PRTN2_COFB1_STAT_OFFSET  (0x0514) /* Partition 2 COFB Set 1 Clock Status Register (PRTN2_COFB1_STAT) */
#define S32K3XX_MC_ME_PRTN2_COFB0_CLKEN_OFFSET (0x0530) /* Partition 2 COFB Set 0 Clock Enable Register (PRTN2_COFB0_CLKEN) */
#define S32K3XX_MC_ME_PRTN2_COFB1_CLKEN_OFFSET (0x0534) /* Partition 2 COFB Set 1 Clock Enable Register (PRTN2_COFB1_CLKEN) */

/* MC_ME Register Addresses *************************************************/

#define S32K3XX_MC_ME_CTL_KEY                  (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_CTL_KEY_OFFSET)
#define S32K3XX_MC_ME_MODE_CONF                (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_MODE_CONF_OFFSET)
#define S32K3XX_MC_ME_MODE_UPD                 (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_MODE_UPD_OFFSET)
#define S32K3XX_MC_ME_MODE_STAT                (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_MODE_STAT_OFFSET)
#define S32K3XX_MC_ME_MAIN_COREID              (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_MAIN_COREID_OFFSET)
#define S32K3XX_MC_ME_PRTN0_PCONF              (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_PCONF_OFFSET)
#define S32K3XX_MC_ME_PRTN0_PUPD               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_PUPD_OFFSET)
#define S32K3XX_MC_ME_PRTN0_STAT               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN0_COFB1_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_COFB1_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN0_COFB1_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_COFB1_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE0_PCONF        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE0_PCONF_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE0_PUPD         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE0_PUPD_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE0_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE0_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE0_ADDR         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE0_ADDR_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE1_PCONF        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE1_PCONF_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE1_PUPD         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE1_PUPD_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE1_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE1_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE1_ADDR         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE1_ADDR_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE2_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE2_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN0_CORE2_ADDR         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN0_CORE2_ADDR_OFFSET)
#define S32K3XX_MC_ME_PRTN1_PCONF              (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_PCONF_OFFSET)
#define S32K3XX_MC_ME_PRTN1_PUPD               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_PUPD_OFFSET)
#define S32K3XX_MC_ME_PRTN1_STAT               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB0_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB0_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB1_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB1_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB2_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB2_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB3_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB3_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB0_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB0_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB1_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB1_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB2_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB2_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN1_COFB3_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN1_COFB3_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN2_PCONF              (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_PCONF_OFFSET)
#define S32K3XX_MC_ME_PRTN2_PUPD               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_PUPD_OFFSET)
#define S32K3XX_MC_ME_PRTN2_STAT               (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN2_COFB0_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_COFB0_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN2_COFB1_STAT         (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_COFB1_STAT_OFFSET)
#define S32K3XX_MC_ME_PRTN2_COFB0_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_COFB0_CLKEN_OFFSET)
#define S32K3XX_MC_ME_PRTN2_COFB1_CLKEN        (S32K3XX_MC_ME_BASE + S32K3XX_MC_ME_PRTN2_COFB1_CLKEN_OFFSET)

/* MC_ME Register Bitfield Definitions **************************************/

/* Control Key Register (CTL_KEY) */

#define MC_ME_CTL_KEY_SHIFT               (0)       /* Bits 0-15: Control Key (KEY) */
#define MC_ME_CTL_KEY_MASK                (0xffff << MC_ME_CTL_KEY_SHIFT)
#define MC_ME_CTL_KEY(n)                  ((0xffff & (n)) << MC_ME_CTL_KEY_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* Mode Configuration Register (MODE_CONF) */
#define MC_ME_MODE_CONF_DEST_RST          (1 << 0)  /* Bit 0: Destructive reset request (DEST_RST) */
#define MC_ME_MODE_CONF_FUNC_RST          (1 << 1)  /* Bit 1: Functional reset request (FUNC_RST) */
                                                    /* Bits 2-14: Reserved */
#define MC_ME_MODE_CONF_STANDBY           (1 << 15) /* Bit 15: Standby request (STANDBY) */
                                                    /* Bits 16-31: Reserved */

/* Mode Update Register (MODE_UPD) */

#define MC_ME_MODE_UPD                    (1 << 0)  /* Bit 0: Mode update (MODE_UPD) */
                                                    /* Bits 1-31: Reserved */

/* Mode Status Register (MODE_STAT) */

#define MC_ME_MODE_STAT_PREV_MODE         (1 << 0)  /* Bit 0: Previous mode (PREV_MODE) */
                                                    /* Bits 1-31: Reserved */

/* Main Core ID Register (MAIN_COREID) */

#define MC_ME_MAIN_COREID_CIDX_SHIFT      (0)       /* Bits 0-2: Core index (CIDX) */
#define MC_ME_MAIN_COREID_CIDX_MASK       (0x07 << MC_ME_MAIN_COREID_CIDX_SHIFT)
                                                    /* Bits 3-7: Reserved */
#define MC_ME_MAIN_COREID_PIDX_SHIFT      (8)       /* Bits 8-12: Partition index (PIDX) */
#define MC_ME_MAIN_COREID_PIDX_MASK       (0x1f << MC_ME_MAIN_COREID_PIDX_SHIFT)
                                                    /* Bits 13-31: Reserved */

/* Partition n Process Configuration Register (PRTNn_PCONF) */

#define MC_ME_PRTN_PCONF_PCE              (1 << 0)  /* Bit 0: Partition clock enable (PCE) */
                                                    /* Bits 1-31: Reserved */

/* Partition n Process Update Register (PRTNn_PUPD) */

#define MC_ME_PRTN_PUPD_PCUD              (1 << 0)  /* Bit 0: Partition clock update (PCUD) */
                                                    /* Bits 1-31: Reserved */

/* Partition n Status Register (PRTN_STAT) */

#define MC_ME_PRTN_STAT_PCS               (1 << 0)  /* Bit 0: Partition clock status (PCS) */
                                                    /* Bits 1-31: Reserved */

/* Partition 0 COFB Set 1 Clock Status Register (PRTN0_COFB1_STAT) */

#define MC_ME_PRTN0_COFB1_STAT_TRGMUX     (1 << 0)  /* Bit 0: TRGMUX clock status (BLOCK32) */
#define MC_ME_PRTN0_COFB1_STAT_BCTU       (1 << 1)  /* Bit 1: BCTU clock status (BLOCK33) */
#define MC_ME_PRTN0_COFB1_STAT_EMIOS0     (1 << 2)  /* Bit 2: EMIOS0 clock status (BLOCK34) */
#define MC_ME_PRTN0_COFB1_STAT_EMIOS1     (1 << 3)  /* Bit 3: EMIOS1 clock status (BLOCK35) */
#define MC_ME_PRTN0_COFB1_STAT_EMIOS2     (1 << 4)  /* Bit 4: EMIOS2 clock status (BLOCK36) */
                                                    /* Bit 5: Reserved */
#define MC_ME_PRTN0_COFB1_STAT_LCU0       (1 << 6)  /* Bit 6: LCU0 clock status (BLOCK38) */
#define MC_ME_PRTN0_COFB1_STAT_LCU1       (1 << 7)  /* Bit 7: LCU1 clock status (BLOCK39) */
#define MC_ME_PRTN0_COFB1_STAT_ADC0       (1 << 8)  /* Bit 8: ADC0 clock status (BLOCK40) */
#define MC_ME_PRTN0_COFB1_STAT_ADC1       (1 << 9)  /* Bit 9: ADC1 clock status (BLOCK41) */
#define MC_ME_PRTN0_COFB1_STAT_ADC2       (1 << 10) /* Bit 10: ADC2 clock status (BLOCK42) */
                                                    /* Bit 11: Reserved */
#define MC_ME_PRTN0_COFB1_STAT_PIT0       (1 << 12) /* Bit 12: PIT0 clock status (BLOCK44) */
#define MC_ME_PRTN0_COFB1_STAT_PIT1       (1 << 13) /* Bit 13: PIT1 clock status (BLOCK45) */
#define MC_ME_PRTN0_COFB1_STAT_MU2_MUA    (1 << 14) /* Bit 14: MU2_MUA clock status (BLOCK46) */
#define MC_ME_PRTN0_COFB1_STAT_MU2_MUB    (1 << 15) /* Bit 15: MU2_MUB clock status (BLOCK47) */
                                                    /* Bits 16-31: Reserved */

/* Partition 0 COFB Set 1 Clock Enable Register (PRTN0_COFB1_CLKEN) */

#define MC_ME_PRTN0_COFB1_CLKEN_TRGMUX    (1 << 0)  /* Bit 0: TRGMUX clock enable (REQ32) */
#define MC_ME_PRTN0_COFB1_CLKEN_BCTU      (1 << 1)  /* Bit 1: BCTU clock enable (REQ33) */
#define MC_ME_PRTN0_COFB1_CLKEN_EMIOS0    (1 << 2)  /* Bit 2: EMIOS0 clock enable (REQ34) */
#define MC_ME_PRTN0_COFB1_CLKEN_EMIOS1    (1 << 3)  /* Bit 3: EMIOS1 clock enable (REQ35) */
#define MC_ME_PRTN0_COFB1_CLKEN_EMIOS2    (1 << 4)  /* Bit 4: EMIOS2 clock enable (REQ36) */
                                                    /* Bit 5: Reserved */
#define MC_ME_PRTN0_COFB1_CLKEN_LCU0      (1 << 6)  /* Bit 6: LCU0 clock enable (REQ38) */
#define MC_ME_PRTN0_COFB1_CLKEN_LCU1      (1 << 7)  /* Bit 7: LCU1 clock enable (REQ39) */
#define MC_ME_PRTN0_COFB1_CLKEN_ADC0      (1 << 8)  /* Bit 8: ADC0 clock enable (REQ40) */
#define MC_ME_PRTN0_COFB1_CLKEN_ADC1      (1 << 9)  /* Bit 9: ADC1 clock enable (REQ41) */
#define MC_ME_PRTN0_COFB1_CLKEN_ADC2      (1 << 10) /* Bit 10: ADC2 clock enable (REQ42) */
                                                    /* Bit 11: Reserved */
#define MC_ME_PRTN0_COFB1_CLKEN_PIT0      (1 << 12) /* Bit 12: PIT0 clock enable (REQ44) */
#define MC_ME_PRTN0_COFB1_CLKEN_PIT1      (1 << 13) /* Bit 13: PIT1 clock enable (REQ45) */
#define MC_ME_PRTN0_COFB1_CLKEN_MU2_MUA   (1 << 14) /* Bit 14: MU2_MUA clock enable (REQ46) */
#define MC_ME_PRTN0_COFB1_CLKEN_MU2_MUB   (1 << 15) /* Bit 15: MU2_MUB clock enable (REQ47) */
#define MC_ME_PRTN0_COFB1_CLKEN_I3C       (1 << 16) /* Bit 16: I3C clock enable (REQ48) */
                                                    /* Bits 17-31: Reserved */

/* Partition 0 Core n Process Configuration Register (PRTN0_COREn_PCONF) */

#define MC_ME_PRTN0_CORE_PCONF            (1 << 0)  /* Bit 0: Core n clock enable (CCE) */
                                                    /* Bits 1-31: Reserved */

/* Partition 0 Core n Process Update Register (PRTN0_COREn_PUPD) */

#define MC_ME_PRTN0_CORE_PUPD             (1 << 0)  /* Bit 0: Core n clock update (CCUPD) */
                                                    /* Bits 1-31: Reserved */

/* Partition 0 Core n Status Register (PRTN0_COREn_STAT) */

#define MC_ME_PRTN0_CORE_STAT             (1 << 0)  /* Bit 0: Core n clock process status (CCS) */
                                                    /* Bits 1-30: Reserved */
#define MC_ME_PRTN0_CORE_WFI              (1 << 31) /* Bit 31: Wait for interrupt status (WFI) */

/* Partition 0 Core n Address Register (PRTN0_COREn_ADDR) */

                                                    /* Bits 0-1: Reserved */
#define MC_ME_PRTN0_CORE_ADDR_SHIFT       (2)       /* Bits 2-31: Address (ADDR) */
#define MC_ME_PRTN0_CORE_ADDR_MASK        (0x3fffffff << MC_ME_PRTN0_CORE0_ADDR_SHIFT)

/* Partition 1 COFB Set 0 Clock Status Register (PRTN1_COFB0_STAT) */

#define MC_ME_PRTN1_COFB0_STAT_AXBS       (1 << 0)  /* Bit 0: AXBS clock status (BLOCK0) */
#define MC_ME_PRTN1_COFB0_STAT_XBIC0      (1 << 1)  /* Bit 1: XBIC clock status (BLOCK1) */
#define MC_ME_PRTN1_COFB0_STAT_XBIC1      (1 << 2)  /* Bit 2: XBIC clock status (BLOCK2) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA       (1 << 3)  /* Bit 3: eDMA clock status (BLOCK3) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD0  (1 << 4)  /* Bit 4: eDMA TCD0 clock status (BLOCK4) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD1  (1 << 5)  /* Bit 5: eDMA TCD1 clock status (BLOCK5) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD2  (1 << 6)  /* Bit 6: eDMA TCD2 clock status (BLOCK6) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD3  (1 << 7)  /* Bit 7: eDMA TCD3 clock status (BLOCK7) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD4  (1 << 8)  /* Bit 8: eDMA TCD4 clock status (BLOCK8) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD5  (1 << 9)  /* Bit 9: eDMA TCD5 clock status (BLOCK9) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD6  (1 << 10) /* Bit 10: eDMA TCD6 clock status (BLOCK10) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD7  (1 << 11) /* Bit 11: eDMA TCD7 clock status (BLOCK11) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD8  (1 << 12) /* Bit 12: eDMA TCD8 clock status (BLOCK12) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD9  (1 << 13) /* Bit 13: eDMA TCD9 clock status (BLOCK13) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD10 (1 << 14) /* Bit 14: eDMA TCD10 clock status (BLOCK14) */
#define MC_ME_PRTN1_COFB0_STAT_EDMA_TCD11 (1 << 15) /* Bit 15: eDMA TCD11 clock status (BLOCK15) */
#define MC_ME_PRTN1_COFB0_STAT_DEBUG_APB0 (1 << 16) /* Bit 16: Debug_APB clock status (BLOCK16) */
#define MC_ME_PRTN1_COFB0_STAT_DEBUG_APB1 (1 << 17) /* Bit 17: Debug_APB clock status (BLOCK17) */
#define MC_ME_PRTN1_COFB0_STAT_DEBUG_APB2 (1 << 18) /* Bit 18: Debug_APB clock status (BLOCK18) */
#define MC_ME_PRTN1_COFB0_STAT_DEBUG_APB3 (1 << 19) /* Bit 19: Debug_APB clock status (BLOCK19) */
#define MC_ME_PRTN1_COFB0_STAT_DEBUG_APB4 (1 << 20) /* Bit 20: Debug_APB clock status (BLOCK20) */
#define MC_ME_PRTN1_COFB0_STAT_SDA_AP     (1 << 21) /* Bit 21: SDA-AP clock status (BLOCK21) */
#define MC_ME_PRTN1_COFB0_STAT_EIM        (1 << 22) /* Bit 22: EIM clock status (BLOCK22) */
#define MC_ME_PRTN1_COFB0_STAT_ERM        (1 << 23) /* Bit 23: ERM clock status (BLOCK23) */
#define MC_ME_PRTN1_COFB0_STAT_MSCM       (1 << 24) /* Bit 24: MSCM clock status (BLOCK24) */
#define MC_ME_PRTN1_COFB0_STAT_PRAMC0     (1 << 25) /* Bit 25: PRAMC0 clock status (BLOCK25) */
#define MC_ME_PRTN1_COFB0_STAT_PFLASH     (1 << 26) /* Bit 26: PFLASH clock status (BLOCK26) */
#define MC_ME_PRTN1_COFB0_STAT_PFASH_ALT  (1 << 27) /* Bit 27: PFLASH_alt clock status (BLOCK27) */
#define MC_ME_PRTN1_COFB0_STAT_SWT0       (1 << 28) /* Bit 28: SWT0 clock status (BLOCK28) */
#define MC_ME_PRTN1_COFB0_STAT_STM0       (1 << 29) /* Bit 29: STM0 clock status (BLOCK29) */
#define MC_ME_PRTN1_COFB0_STAT_XRDC       (1 << 30) /* Bit 30: XRDC clock status (BLOCK30) */
#define MC_ME_PRTN1_COFB0_STAT_INTM       (1 << 31) /* Bit 31: INTM clock status (BLOCK31) */

/* Partition 1 COFB Set 1 Clock Status Register (PRTN1_COFB1_STAT) */

#define MC_ME_PRTN1_COFB1_STAT_DMAMUX0    (1 << 0)  /* Bit 0: DMAMUX0 clock status (BLOCK32) */
#define MC_ME_PRTN1_COFB1_STAT_DMAMUX1    (1 << 1)  /* Bit 1: DMAMUX1 clock status (BLOCK33) */
#define MC_ME_PRTN1_COFB1_STAT_RTC        (1 << 2)  /* Bit 2: RTC clock status (BLOCK34) */
#define MC_ME_PRTN1_COFB1_STAT_MC_RGM     (1 << 3)  /* Bit 3: MC_RGM clock status (BLOCK35) */

#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC0_HSE0   (1 << 4)  /* Bit 4: SIUL_VIRTWRAPPER_PDAC0_HSE clock status (BLOCK36) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC0_HSE1   (1 << 5)  /* Bit 5: SIUL_VIRTWRAPPER_PDAC0_HSE clock status (BLOCK37) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC1_M7_0_0 (1 << 6)  /* Bit 6: SIUL_VIRTWRAPPER_PDAC1_M7_0 clock status (BLOCK38) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC1_M7_0_1 (1 << 7)  /* Bit 7: SIUL_VIRTWRAPPER_PDAC1_M7_0 clock status (BLOCK39) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC2_M7_1_0 (1 << 8)  /* Bit 8: SIUL_VIRTWRAPPER_PDAC2_M7_1 clock status (BLOCK40) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC2_M7_1_1 (1 << 9)  /* Bit 9: SIUL_VIRTWRAPPER_PDAC2_M7_1 clock status (BLOCK41) */
#define MC_ME_PRTN1_COFB1_STAT_SIUL_VIRTWRAPPER_PDAC3        (1 << 10) /* Bit 10: SIUL_VIRTWRAPPER_PDAC3 clock status (BLOCK42) */

#define MC_ME_PRTN1_COFB1_STAT_DCM        (1 << 11) /* Bit 11: DCM clock status (BLOCK43) */
                                                    /* Bit 12: Reserved */
#define MC_ME_PRTN1_COFB1_STAT_WKPU       (1 << 13) /* Bit 13: WKPU clock status (BLOCK45) */
                                                    /* Bit 14: Reserved */
#define MC_ME_PRTN1_COFB1_STAT_CMU        (1 << 15) /* Bit 15: CMU0-5 clock status (BLOCK47) */
                                                    /* Bit 16: Reserved */
#define MC_ME_PRTN1_COFB1_STAT_TSPC       (1 << 17) /* Bit 17: TSPC clock status (BLOCK49) */
#define MC_ME_PRTN1_COFB1_STAT_SIRC       (1 << 18) /* Bit 18: SIRC clock status (BLOCK50) */
#define MC_ME_PRTN1_COFB1_STAT_SXOSC      (1 << 19) /* Bit 19: SXOSC clock status (BLOCK51) */
#define MC_ME_PRTN1_COFB1_STAT_FIRC       (1 << 20) /* Bit 20: FIRC clock status (BLOCK52) */
#define MC_ME_PRTN1_COFB1_STAT_FXOSC      (1 << 21) /* Bit 21: FXOSC clock status (BLOCK53) */
#define MC_ME_PRTN1_COFB1_STAT_MC_CGM     (1 << 22) /* Bit 22: MC_CGM clock status (BLOCK54) */
#define MC_ME_PRTN1_COFB1_STAT_MC_ME      (1 << 23) /* Bit 23: MC_ME clock status (BLOCK55) */
#define MC_ME_PRTN1_COFB1_STAT_PLL        (1 << 24) /* Bit 24: PLL clock status (BLOCK56) */
                                                    /* Bit 25: Reserved */
#define MC_ME_PRTN1_COFB1_STAT_PMC        (1 << 26) /* Bit 26: PMC clock status (BLOCK58) */
#define MC_ME_PRTN1_COFB1_STAT_FMU        (1 << 27) /* Bit 27: FMU clock status (BLOCK59) */
#define MC_ME_PRTN1_COFB1_STAT_FMU_ALT    (1 << 28) /* Bit 28: FMU_alt clock status (BLOCK60) */
                                                    /* Bits 29-30: Reserved */
#define MC_ME_PRTN1_COFB1_STAT_PIT2       (1 << 31) /* Bit 31: PIT2 clock status (BLOCK63) */

/* Partition 1 COFB Set 2 Clock Status Register (PRTN1_COFB2_STAT) */

                                                    /* Bit 0: Reserved */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN0   (1 << 1)  /* Bit 1: FlexCAN0 clock status (BLOCK65) */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN1   (1 << 2)  /* Bit 2: FlexCAN1 clock status (BLOCK66) */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN2   (1 << 3)  /* Bit 3: FlexCAN2 clock status (BLOCK67) */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN3   (1 << 4)  /* Bit 4: FlexCAN3 clock status (BLOCK68) */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN4   (1 << 5)  /* Bit 5: FlexCAN4 clock status (BLOCK69) */
#define MC_ME_PRTN1_COFB2_STAT_FLEXCAN5   (1 << 6)  /* Bit 6: FlexCAN5 clock status (BLOCK70) */
                                                    /* Bits 7-8: Reserved */
#define MC_ME_PRTN1_COFB2_STAT_FLEXIO     (1 << 9)  /* Bit 9: FlexIO clock status (BLOCK73) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART0    (1 << 10) /* Bit 10: LPUART0 clock status (BLOCK74) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART1    (1 << 11) /* Bit 11: LPUART1 clock status (BLOCK75) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART2    (1 << 12) /* Bit 12: LPUART2 clock status (BLOCK76) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART3    (1 << 13) /* Bit 13: LPUART3 clock status (BLOCK77) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART4    (1 << 14) /* Bit 14: LPUART4 clock status (BLOCK78) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART5    (1 << 15) /* Bit 15: LPUART5 clock status (BLOCK79) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART6    (1 << 16) /* Bit 16: LPUART6 clock status (BLOCK80) */
#define MC_ME_PRTN1_COFB2_STAT_LPUART7    (1 << 17) /* Bit 17: LPUART7 clock status (BLOCK81) */
                                                    /* Bits 18-19: Reserved */
#define MC_ME_PRTN1_COFB2_STAT_LPI2C0     (1 << 20) /* Bit 20: LPI2C0 clock status (BLOCK84) */
#define MC_ME_PRTN1_COFB2_STAT_LPI2C1     (1 << 21) /* Bit 21: LPI2C1 clock status (BLOCK85) */
#define MC_ME_PRTN1_COFB2_STAT_LPSPI0     (1 << 22) /* Bit 22: LPSPI0 clock status (BLOCK86) */
#define MC_ME_PRTN1_COFB2_STAT_LPSPI1     (1 << 23) /* Bit 23: LPSPI1 clock status (BLOCK87) */
#define MC_ME_PRTN1_COFB2_STAT_LPSPI2     (1 << 24) /* Bit 24: LPSPI2 clock status (BLOCK88) */
#define MC_ME_PRTN1_COFB2_STAT_LPSPI3     (1 << 25) /* Bit 25: LPSPI3 clock status (BLOCK89) */
                                                    /* Bit 26: Reserved */
#define MC_ME_PRTN1_COFB2_STAT_SAI0       (1 << 27) /* Bit 27: SAI0 clock status (BLOCK91) */
#define MC_ME_PRTN1_COFB2_STAT_LPCMP0     (1 << 28) /* Bit 28: LPCMP0 clock status (BLOCK92) */
#define MC_ME_PRTN1_COFB2_STAT_LPCMP1     (1 << 29) /* Bit 29: LPCMP1 clock status (BLOCK93) */
                                                    /* Bit 30: Reserved */
#define MC_ME_PRTN1_COFB2_STAT_TMU        (1 << 31) /* Bit 31: TMU clock status (BLOCK95) */

/* Partition 1 COFB Set 3 Clock Status Register (PRTN1_COFB3_STAT) */

#define MC_ME_PRTN1_COFB3_STAT_CRC        (1 << 0)  /* Bit 0: CRC clock status (BLOCK96) */
#define MC_ME_PRTN1_COFB3_STAT_FCCU       (1 << 1)  /* Bit 1: FCCU clock status (BLOCK97) */
#define MC_ME_PRTN1_COFB3_STAT_MTR        (1 << 2)  /* Bit 2: MTR clock status (BLOCK98) */
#define MC_ME_PRTN1_COFB3_STAT_HSE        (1 << 3)  /* Bit 3: HSE clock status (BLOCK99) */
                                                    /* Bit 4: Reserved */
#define MC_ME_PRTN1_COFB3_STAT_JDC        (1 << 5)  /* Bit 5: JDC clock status (BLOCK101) */
                                                    /* Bit 6: Reserved */
#define MC_ME_PRTN1_COFB3_STAT_CONFIG_GPR (1 << 7)  /* Bit 7: CONFIG_GPR clock status (BLOCK103) */
#define MC_ME_PRTN1_COFB3_STAT_STCU2      (1 << 8)  /* Bit 8: STCU2 clock status (BLOCK104) */
                                                    /* Bits 9-11: Reserved */

#define MC_ME_PRTN1_COFB3_STAT_SELFTEST_GPR (1 << 12) /* Bit 12: SELFTEST_GPR clock status (BLOCK108) */

                                                    /* Bits 13-31: Reserved */

/* Partition 1 COFB Set 0 Clock Enable Register (PRTN1_COFB0_CLKEN) */

                                                    /* Bits 0-2: Reserved */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA      (1 << 3)  /* Bit 3: eDMA clock enable (REQ3) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD0 (1 << 4)  /* Bit 4: eDMA TCD0 clock enable (REQ4) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD1 (1 << 5)  /* Bit 5: eDMA TCD1 clock enable (REQ5) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD2 (1 << 6)  /* Bit 6: eDMA TCD2 clock enable (REQ6) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD3 (1 << 7)  /* Bit 7: eDMA TCD3 clock enable (REQ7) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD4 (1 << 8)  /* Bit 8: eDMA TCD4 clock enable (REQ8) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD5 (1 << 9)  /* Bit 9: eDMA TCD5 clock enable (REQ9) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD6 (1 << 10) /* Bit 10: eDMA TCD6 clock enable (REQ10) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD7 (1 << 11) /* Bit 11: eDMA TCD7 clock enable (REQ11) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD8 (1 << 12) /* Bit 12: eDMA TCD8 clock enable (REQ12) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD9 (1 << 13) /* Bit 13: eDMA TCD9 clock enable (REQ13) */

#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD10 (1 << 14) /* Bit 14: eDMA TCD10 clock enable (REQ14) */
#define MC_ME_PRTN1_COFB0_CLKEN_EDMA_TCD11 (1 << 15) /* Bit 15: eDMA TCD11 clock enable (REQ15) */

                                                    /* Bits 16-20: Reserved */
#define MC_ME_PRTN1_COFB0_CLKEN_SDA_AP    (1 << 21) /* Bit 21: SDA-AP clock enable (REQ21) */
#define MC_ME_PRTN1_COFB0_CLKEN_EIM       (1 << 22) /* Bit 22: EIM clock enable (REQ22) */
#define MC_ME_PRTN1_COFB0_CLKEN_ERM       (1 << 23) /* Bit 23: ERM clock enable (REQ23) */
#define MC_ME_PRTN1_COFB0_CLKEN_MSCM      (1 << 24) /* Bit 24: MSCM clock enable (REQ24) */
                                                    /* Bits 25-27: Reserved */
#define MC_ME_PRTN1_COFB0_CLKEN_SWT0      (1 << 28) /* Bit 28: SWT0 clock enable (REQ28) */
#define MC_ME_PRTN1_COFB0_CLKEN_STM0      (1 << 29) /* Bit 29: STM0 clock enable (REQ29) */
                                                    /* Bit 30: Reserved */
#define MC_ME_PRTN1_COFB0_CLKEN_INTM      (1 << 31) /* Bit 31: INTM clock enable (REQ31) */

/* Partition 1 COFB Set 1 Clock Enable Register (PRTN1_COFB1_CLKEN) */

#define MC_ME_PRTN1_COFB1_CLKEN_DMAMUX0   (1 << 0)  /* Bit 0: DMAMUX0 clock enable (REQ32) */
#define MC_ME_PRTN1_COFB1_CLKEN_DMAMUX1   (1 << 1)  /* Bit 1: DMAMUX1 clock enable (REQ33) */
#define MC_ME_PRTN1_COFB1_CLKEN_RTC       (1 << 2)  /* Bit 2: RTC clock enable (REQ34) */
                                                    /* Bits 3-9: Reserved */

#define MC_ME_PRTN1_COFB1_CLKEN_SIUL_VIRTWRAPPER_PDAC3 (1 << 10) /* Bit 10: SIUL_VIRTWRAPPER_PDAC3 clock enable (REQ42) */

                                                    /* Bits 11-12: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_WKPU      (1 << 13) /* Bit 13: WKPU clock enable (REQ45) */
                                                    /* Bit 14: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_CMU       (1 << 15) /* Bit 15: CMU0-5 clock enable (REQ47) */
                                                    /* Bit 16: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_TSPC      (1 << 17) /* Bit 17: TSPC clock enable (REQ49) */
                                                    /* Bit 18: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_SXOSC     (1 << 19) /* Bit 19: SXOSC clock enable (REQ51) */
                                                    /* Bit 20: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_FXOSC     (1 << 21) /* Bit 21: FXOSC clock enable (REQ53) */
                                                    /* Bits 22-23: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_PLL       (1 << 24) /* Bit 24: PLL clock enable (REQ56) */
                                                    /* BIts 25-30: Reserved */
#define MC_ME_PRTN1_COFB1_CLKEN_PIT2      (1 << 31) /* Bit 31: PIT2 clock enable (REQ63) */

/* Partition 1 COFB Set 2 Clock Enable Register (PRTN1_COFB2_CLKEN) */

                                                    /* Bit 0: Reserved */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN0  (1 << 1)  /* Bit 1: FlexCAN0 clock enable (REQ65) */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN1  (1 << 2)  /* Bit 2: FlexCAN1 clock enable (REQ66) */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN2  (1 << 3)  /* Bit 3: FlexCAN2 clock enable (REQ67) */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN3  (1 << 4)  /* Bit 4: FlexCAN3 clock enable (REQ68) */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN4  (1 << 5)  /* Bit 5: FlexCAN4 clock enable (REQ69) */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXCAN5  (1 << 6)  /* Bit 6: FlexCAN5 clock enable (REQ70) */
                                                    /* Bits 7-8: Reserved */
#define MC_ME_PRTN1_COFB2_CLKEN_FLEXIO    (1 << 9)  /* Bit 9: FlexIO clock enable (REQ73) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART0   (1 << 10) /* Bit 10: LPUART0 clock enable (REQ74) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART1   (1 << 11) /* Bit 11: LPUART1 clock enable (REQ75) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART2   (1 << 12) /* Bit 12: LPUART2 clock enable (REQ76) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART3   (1 << 13) /* Bit 13: LPUART3 clock enable (REQ77) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART4   (1 << 14) /* Bit 14: LPUART4 clock enable (REQ78) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART5   (1 << 15) /* Bit 15: LPUART5 clock enable (REQ79) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART6   (1 << 16) /* Bit 16: LPUART6 clock enable (REQ80) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPUART7   (1 << 17) /* Bit 17: LPUART7 clock enable (REQ81) */
                                                    /* Bits 18-19: Reserved */
#define MC_ME_PRTN1_COFB2_CLKEN_LPI2C0    (1 << 20) /* Bit 20: LPI2C0 clock enable (REQ84) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPI2C1    (1 << 21) /* Bit 21: LPI2C1 clock enable (REQ85) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPSPI0    (1 << 22) /* Bit 22: LPSPI0 clock enable (REQ86) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPSPI1    (1 << 23) /* Bit 23: LPSPI1 clock enable (REQ87) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPSPI2    (1 << 24) /* Bit 24: LPSPI2 clock enable (REQ88) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPSPI3    (1 << 25) /* Bit 25: LPSPI3 clock enable (REQ89) */
                                                    /* Bit 26: Reserved */
#define MC_ME_PRTN1_COFB2_CLKEN_SAI0      (1 << 27) /* Bit 27: SAI0 clock enable (REQ91) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPCMP0    (1 << 28) /* Bit 28: LPCMP0 clock enable (REQ92) */
#define MC_ME_PRTN1_COFB2_CLKEN_LPCMP1    (1 << 29) /* Bit 29: LPCMP1 clock enable (REQ93) */
                                                    /* Bit 30: Reserved */
#define MC_ME_PRTN1_COFB2_CLKEN_TMU       (1 << 31) /* Bit 31: TMU clock enable (REQ95) */

/* Partition 1 COFB Set 3 Clock Enable Register (PRTN1_COFB3_CLKEN) */

#define MC_ME_PRTN1_COFB3_CLKEN_CRC       (1 << 0)  /* Bit 0: CRC clock enable (REQ96) */
                                                    /* Bits 1-7: Reserved */
#define MC_ME_PRTN1_COFB3_CLKEN_STCU2     (1 << 8)  /* Bit 8: STCU2 clock enable (REQ104) */
                                                    /* Bits 9-31: Reserved */

/* Partition 2 COFB Set 0 Clock Status Register (PRTN2_COFB0_STAT) */

#define MC_ME_PRTN2_COFB0_STAT_XBIC2      (1 << 0)  /* Bit 0: XBIC2 clock status (BLOCK0) */
#define MC_ME_PRTN2_COFB0_STAT_XBIC3      (1 << 1)  /* Bit 1: XBIC3 clock status (BLOCK1) */
                                                    /* Bits 2-3: Reserved */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD12 (1 << 4)  /* Bit 4: eDMA TCD12 clock status (BLOCK4) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD13 (1 << 5)  /* Bit 5: eDMA TCD13 clock status (BLOCK5) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD14 (1 << 6)  /* Bit 6: eDMA TCD14 clock status (BLOCK6) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD15 (1 << 7)  /* Bit 7: eDMA TCD15 clock status (BLOCK7) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD16 (1 << 8)  /* Bit 8: eDMA TCD16 clock status (BLOCK8) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD17 (1 << 9)  /* Bit 9: eDMA TCD17 clock status (BLOCK9) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD18 (1 << 10) /* Bit 10: eDMA TCD18 clock status (BLOCK10) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD19 (1 << 11) /* Bit 11: eDMA TCD19 clock status (BLOCK11) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD20 (1 << 12) /* Bit 12: eDMA TCD20 clock status (BLOCK12) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD21 (1 << 13) /* Bit 13: eDMA TCD21 clock status (BLOCK13) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD22 (1 << 14) /* Bit 14: eDMA TCD22 clock status (BLOCK14) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD23 (1 << 15) /* Bit 15: eDMA TCD23 clock status (BLOCK15) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD24 (1 << 16) /* Bit 16: eDMA TCD24 clock status (BLOCK16) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD25 (1 << 17) /* Bit 17: eDMA TCD25 clock status (BLOCK17) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD26 (1 << 18) /* Bit 18: eDMA TCD26 clock status (BLOCK18) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD27 (1 << 19) /* Bit 19: eDMA TCD27 clock status (BLOCK19) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD28 (1 << 20) /* Bit 20: eDMA TCD28 clock status (BLOCK20) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD29 (1 << 21) /* Bit 21: eDMA TCD29 clock status (BLOCK21) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD30 (1 << 22) /* Bit 22: eDMA TCD30 clock status (BLOCK22) */
#define MC_ME_PRTN2_COFB0_STAT_EDMA_TCD31 (1 << 23) /* Bit 23: eDMA TCD31 clock status (BLOCK23) */
#define MC_ME_PRTN2_COFB0_STAT_SEMA42     (1 << 24) /* Bit 24: SEMA42 clock status (BLOCK24) */
#define MC_ME_PRTN2_COFB0_STAT_PRAMC1     (1 << 25) /* Bit 25: PRAMC1 clock status (BLOCK25) */
                                                    /* Bit 26: Reserved */
#define MC_ME_PRTN2_COFB0_STAT_SWT1       (1 << 27) /* Bit 27: SWT1 clock status (BLOCK27) */
                                                    /* Bit 28: Reserved */
#define MC_ME_PRTN2_COFB0_STAT_STM1       (1 << 29) /* Bit 29: STM1 clock status (BLOCK29) */
                                                    /* Bit 30-31: Reserved */

/* Partition 2 COFB Set 1 Clock Status Register (PRTN2_COFB1_STAT) */

#define MC_ME_PRTN2_COFB1_STAT_EMAC       (1 << 0)  /* Bit 0: EMAC clock status (BLOCK32) */
                                                    /* Bits 1-2: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_LPUART8    (1 << 3)  /* Bit 3: LPUART8 clock status (BLOCK35) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART9    (1 << 4)  /* Bit 4: LPUART9 clock status (BLOCK36) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART10   (1 << 5)  /* Bit 5: LPUART10 clock status (BLOCK37) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART11   (1 << 6)  /* Bit 6: LPUART11 clock status (BLOCK38) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART12   (1 << 7)  /* Bit 7: LPUART12 clock status (BLOCK39) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART13   (1 << 8)  /* Bit 8: LPUART13 clock status (BLOCK40) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART14   (1 << 9)  /* Bit 9: LPUART14 clock status (BLOCK41) */
#define MC_ME_PRTN2_COFB1_STAT_LPUART15   (1 << 10) /* Bit 10: LPUART15 clock status (BLOCK42) */
                                                    /* Bits 11-14: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_LPSPI4     (1 << 15) /* Bit 15: LPSPI4 clock status (BLOCK47) */
#define MC_ME_PRTN2_COFB1_STAT_LPSPI5     (1 << 16) /* Bit 16: LPSPI5 clock status (BLOCK48) */
                                                    /* Bits 17-18: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_QSPI       (1 << 19) /* Bit 19: QuadSPI clock status (BLOCK51) */
                                                    /* Bit 20-22: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_SAI1       (1 << 23) /* Bit 23: SAI1 clock status (BLOCK55) */
                                                    /* Bits 24-25: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_LCMP2      (1 << 26) /* Bit 26: LPCMP2 clock status (BLOCK58) */
#define MC_ME_PRTN2_COFB1_STAT_HSE        (1 << 27) /* Bit 27: HSE clock status (BLOCK59) */
                                                    /* Bit 28-29: Reserved */
#define MC_ME_PRTN2_COFB1_STAT_CM7_0_TCM  (1 << 30) /* Bit 30: CM7_0_TCM clock status (BLOCK62) */
#define MC_ME_PRTN2_COFB1_STAT_CM7_1_TCM  (1 << 31) /* Bit 31: CM7_1_TCM clock status (BLOCK63) */

/* Partition 2 COFB Set 0 Clock Enable Register (PRTN2_COFB0_CLKEN) */

                                                    /* Bits 0-3: Reserved */

#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD12 (1 << 4)  /* Bit 4: eDMA TCD12 clock enable (REQ4) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD13 (1 << 5)  /* Bit 5: eDMA TCD13 clock enable (REQ5) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD14 (1 << 6)  /* Bit 6: eDMA TCD14 clock enable (REQ6) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD15 (1 << 7)  /* Bit 7: eDMA TCD15 clock enable (REQ7) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD16 (1 << 8)  /* Bit 8: eDMA TCD16 clock enable (REQ8) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD17 (1 << 9)  /* Bit 9: eDMA TCD17 clock enable (REQ9) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD18 (1 << 10) /* Bit 10: eDMA TCD18 clock enable (REQ10) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD19 (1 << 11) /* Bit 11: eDMA TCD19 clock enable (REQ11) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD20 (1 << 12) /* Bit 12: eDMA TCD20 clock enable (REQ12) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD21 (1 << 13) /* Bit 13: eDMA TCD21 clock enable (REQ13) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD22 (1 << 14) /* Bit 14: eDMA TCD22 clock enable (REQ14) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD23 (1 << 15) /* Bit 15: eDMA TCD23 clock enable (REQ15) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD24 (1 << 16) /* Bit 16: eDMA TCD24 clock enable (REQ16) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD25 (1 << 17) /* Bit 17: eDMA TCD25 clock enable (REQ17) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD26 (1 << 18) /* Bit 18: eDMA TCD26 clock enable (REQ18) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD27 (1 << 19) /* Bit 19: eDMA TCD27 clock enable (REQ19) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD28 (1 << 20) /* Bit 20: eDMA TCD28 clock enable (REQ20) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD29 (1 << 21) /* Bit 21: eDMA TCD29 clock enable (REQ21) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD30 (1 << 22) /* Bit 22: eDMA TCD30 clock enable (REQ22) */
#define MC_ME_PRTN2_COFB0_CLKEN_EDMA_TCD31 (1 << 23) /* Bit 23: eDMA TCD31 clock enable (REQ23) */

#define MC_ME_PRTN2_COFB0_CLKEN_SEMA42    (1 << 24) /* Bit 24: SEMA42 clock enable (REQ24) */
                                                    /* Bits 25-26: Reserved */
#define MC_ME_PRTN2_COFB0_CLKEN_SWT1      (1 << 27) /* Bit 27: SWT1 clock enable (REQ27) */
                                                    /* Bit 28: Reserved */
#define MC_ME_PRTN2_COFB0_CLKEN_STM1      (1 << 29) /* Bit 29: STM1 clock enable (REQ29) */
                                                    /* Bit 30-31: Reserved */

/* Partition 2 COFB Set 1 Clock Enable Register (PRTN2_COFB1_CLKEN) */

#define MC_ME_PRTN2_COFB1_CLKEN_EMAC      (1 << 0)  /* Bit 0: EMAC clock enable (REQ32) */
                                                    /* Bits 1-2: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART8   (1 << 3)  /* Bit 3: LPUART8 clock enable (REQ35) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART9   (1 << 4)  /* Bit 4: LPUART9 clock enable (REQ36) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART10  (1 << 5)  /* Bit 5: LPUART10 clock enable (REQ37) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART11  (1 << 6)  /* Bit 6: LPUART11 clock enable (REQ38) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART12  (1 << 7)  /* Bit 7: LPUART12 clock enable (REQ39) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART13  (1 << 8)  /* Bit 8: LPUART13 clock enable (REQ40) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART14  (1 << 9)  /* Bit 9: LPUART14 clock enable (REQ41) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPUART15  (1 << 10) /* Bit 10: LPUART15 clock enable (REQ42) */
                                                    /* Bits 11-14: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_LPSPI4    (1 << 15) /* Bit 15: LPSPI4 clock enable (REQ47) */
#define MC_ME_PRTN2_COFB1_CLKEN_LPSPI5    (1 << 16) /* Bit 16: LPSPI5 clock enable (REQ48) */
                                                    /* Bits 17-18: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_QSPI      (1 << 19) /* Bit 19: QuadSPI clock enable (REQ51) */
                                                    /* Bit 20-22: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_SAI1      (1 << 23) /* Bit 23: SAI1 clock enable (REQ55) */
                                                    /* Bits 24-25: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_LCMP2     (1 << 26) /* Bit 26: LPCMP2 clock enable (REQ58) */
                                                    /* Bit 27-29: Reserved */
#define MC_ME_PRTN2_COFB1_CLKEN_CM7_0_TCM (1 << 30) /* Bit 30: CM7_0_TCM clock enable (REQ62) */
#define MC_ME_PRTN2_COFB1_CLKEN_CM7_1_TCM (1 << 31) /* Bit 31: CM7_1_TCM clock enable (REQ63) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_ME_H */
