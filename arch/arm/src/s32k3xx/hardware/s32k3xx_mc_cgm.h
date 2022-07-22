/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_mc_cgm.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_CGM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_CGM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MC_CGM Register Offsets **************************************************/

#define S32K3XX_MC_CGM_PCFS_SFDUR_OFFSET          (0x0000) /* PCFS Step Duration Register (PCFS_SDUR) */
#define S32K3XX_MC_CGM_PCFS_DIVC8_OFFSET          (0x0058) /* PCFS Divider Change 8 Register (PCFS_DIVC8) */
#define S32K3XX_MC_CGM_PCFS_DIVE8_OFFSET          (0x005c) /* PCFS Divider End 8 Register (PCFS_DIVE8) */
#define S32K3XX_MC_CGM_PCFS_DIVS8_OFFSET          (0x0060) /* PCFS Divider Start 8 Register (PCFS_DIVS8) */
#define S32K3XX_MC_CGM_MUX_0_CSC_OFFSET           (0x0300) /* Clock Mux 0 Select Control Register (MUX_0_CSC) */
#define S32K3XX_MC_CGM_MUX_0_CSS_OFFSET           (0x0304) /* Clock Mux 0 Select Status Register (MUX_0_CSS) */
#define S32K3XX_MC_CGM_MUX_0_DC_0_OFFSET          (0x0308) /* Clock Mux 0 Divider 0 Control Register (MUX_0_DC_0) */
#define S32K3XX_MC_CGM_MUX_0_DC_1_OFFSET          (0x030c) /* Clock Mux 0 Divider 1 Control Register (MUX_0_DC_1) */
#define S32K3XX_MC_CGM_MUX_0_DC_2_OFFSET          (0x0310) /* Clock Mux 0 Divider 2 Control Register (MUX_0_DC_2) */
#define S32K3XX_MC_CGM_MUX_0_DC_3_OFFSET          (0x0314) /* Clock Mux 0 Divider 3 Control Register (MUX_0_DC_3) */
#define S32K3XX_MC_CGM_MUX_0_DC_4_OFFSET          (0x0318) /* Clock Mux 0 Divider 4 Control Register (MUX_0_DC_4) */
#define S32K3XX_MC_CGM_MUX_0_DC_5_OFFSET          (0x031c) /* Clock Mux 0 Divider 5 Control Register (MUX_0_DC_5) */
#define S32K3XX_MC_CGM_MUX_0_DC_6_OFFSET          (0x0320) /* Clock Mux 0 Divider 6 Control Register (MUX_0_DC_6) */
#define S32K3XX_MC_CGM_MUX_0_DIV_TRIG_CTRL_OFFSET (0x0334) /* Clock Mux 0 Divider Trigger Control Register (MUX_0_DIV_TRIG_CTRL) */
#define S32K3XX_MC_CGM_MUX_0_DIV_TRIG_OFFSET      (0x0338) /* Clock Mux 0 Divider Trigger Register (MUX_0_DIV_TRIG) */
#define S32K3XX_MC_CGM_MUX_0_DIV_UPD_STAT_OFFSET  (0x033c) /* Clock Mux 0 Divider Update Status Register (MUX_0_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_1_CSC_OFFSET           (0x0340) /* Clock Mux 1 Select Control Register (MUX_1_CSC) */
#define S32K3XX_MC_CGM_MUX_1_CSS_OFFSET           (0x0344) /* Clock Mux 1 Select Status Register (MUX_1_CSS) */
#define S32K3XX_MC_CGM_MUX_1_DC_0_OFFSET          (0x0348) /* Clock Mux 1 Divider 0 Control Register (MUX_1_DC_0) */
#define S32K3XX_MC_CGM_MUX_1_DIV_UPD_STAT_OFFSET  (0x037c) /* Clock Mux 1 Divider Update Status Register (MUX_1_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_2_CSC_OFFSET           (0x0380) /* Clock Mux 2 Select Control Register (MUX_2_CSC) */
#define S32K3XX_MC_CGM_MUX_2_CSS_OFFSET           (0x0384) /* Clock Mux 2 Select Status Register (MUX_2_CSS) */
#define S32K3XX_MC_CGM_MUX_2_DC_0_OFFSET          (0x0388) /* Clock Mux 2 Divider 0 Control Register (MUX_2_DC_0) */
#define S32K3XX_MC_CGM_MUX_2_DIV_UPD_STAT_OFFSET  (0x03bc) /* Clock Mux 2 Divider Update Status Register (MUX_2_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_3_CSC_OFFSET           (0x03c0) /* Clock Mux 3 Select Control Register (MUX_3_CSC) */
#define S32K3XX_MC_CGM_MUX_3_CSS_OFFSET           (0x03c4) /* Clock Mux 3 Select Status Register (MUX_3_CSS) */
#define S32K3XX_MC_CGM_MUX_3_DC_0_OFFSET          (0x03c8) /* Clock Mux 3 Divider 0 Control Register (MUX_3_DC_0) */
#define S32K3XX_MC_CGM_MUX_3_DIV_UPD_STAT_OFFSET  (0x03fc) /* Clock Mux 3 Divider Update Status Register (MUX_3_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_4_CSC_OFFSET           (0x0400) /* Clock Mux 4 Select Control Register (MUX_4_CSC) */
#define S32K3XX_MC_CGM_MUX_4_CSS_OFFSET           (0x0404) /* Clock Mux 4 Select Status Register (MUX_4_CSS) */
#define S32K3XX_MC_CGM_MUX_4_DC_0_OFFSET          (0x0408) /* Clock Mux 4 Divider 0 Control Register (MUX_4_DC_0) */
#define S32K3XX_MC_CGM_MUX_4_DIV_UPD_STAT_OFFSET  (0x043c) /* Clock Mux 4 Divider Update Status Register (MUX_4_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_5_CSC_OFFSET           (0x0440) /* Clock Mux 5 Select Control Register (MUX_5_CSC) */
#define S32K3XX_MC_CGM_MUX_5_CSS_OFFSET           (0x0444) /* Clock Mux 5 Select Status Register (MUX_5_CSS) */
#define S32K3XX_MC_CGM_MUX_5_DC_0_OFFSET          (0x0448) /* Clock Mux 5 Divider 0 Control Register (MUX_5_DC_0) */
#define S32K3XX_MC_CGM_MUX_5_DIV_UPD_STAT_OFFSET  (0x047c) /* Clock Mux 5 Divider Update Status Register (MUX_5_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_6_CSC_OFFSET           (0x0480) /* Clock Mux 6 Select Control Register (MUX_6_CSC) */
#define S32K3XX_MC_CGM_MUX_6_CSS_OFFSET           (0x0484) /* Clock Mux 6 Select Status Register (MUX_6_CSS) */
#define S32K3XX_MC_CGM_MUX_6_DC_0_OFFSET          (0x0488) /* Clock Mux 6 Divider 0 Control Register (MUX_6_DC_0) */
#define S32K3XX_MC_CGM_MUX_6_DIV_UPD_STAT_OFFSET  (0x04bc) /* Clock Mux 6 Divider Update Status Register (MUX_6_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_7_CSC_OFFSET           (0x04c0) /* Clock Mux 7 Select Control Register (MUX_7_CSC) */
#define S32K3XX_MC_CGM_MUX_7_CSS_OFFSET           (0x04c4) /* Clock Mux 7 Select Status Register (MUX_7_CSS) */
#define S32K3XX_MC_CGM_MUX_7_DC_0_OFFSET          (0x04c8) /* Clock Mux 7 Divider 0 Control Register (MUX_7_DC_0) */
#define S32K3XX_MC_CGM_MUX_7_DIV_UPD_STAT_OFFSET  (0x04fc) /* Clock Mux 7 Divider Update Status Register (MUX_7_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_8_CSC_OFFSET           (0x0500) /* Clock Mux 8 Select Control Register (MUX_8_CSC) */
#define S32K3XX_MC_CGM_MUX_8_CSS_OFFSET           (0x0504) /* Clock Mux 8 Select Status Register (MUX_8_CSS) */
#define S32K3XX_MC_CGM_MUX_8_DC_0_OFFSET          (0x0508) /* Clock Mux 8 Divider 0 Control Register (MUX_8_DC_0) */
#define S32K3XX_MC_CGM_MUX_8_DIV_UPD_STAT_OFFSET  (0x053c) /* Clock Mux 8 Divider Update Status Register (MUX_8_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_9_CSC_OFFSET           (0x0540) /* Clock Mux 9 Select Control Register (MUX_9_CSC) */
#define S32K3XX_MC_CGM_MUX_9_CSS_OFFSET           (0x0544) /* Clock Mux 9 Select Status Register (MUX_9_CSS) */
#define S32K3XX_MC_CGM_MUX_9_DC_0_OFFSET          (0x0548) /* Clock Mux 9 Divider 0 Control Register (MUX_9_DC_0) */
#define S32K3XX_MC_CGM_MUX_9_DIV_UPD_STAT_OFFSET  (0x057c) /* Clock Mux 9 Divider Update Status Register (MUX_9_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_10_CSC_OFFSET          (0x0580) /* Clock Mux 10 Select Control Register (MUX_10_CSC) */
#define S32K3XX_MC_CGM_MUX_10_CSS_OFFSET          (0x0584) /* Clock Mux 10 Select Status Register (MUX_10_CSS) */
#define S32K3XX_MC_CGM_MUX_10_DC_0_OFFSET         (0x0588) /* Clock Mux 10 Divider 0 Control Register (MUX_10_DC_0) */
#define S32K3XX_MC_CGM_MUX_10_DIV_UPD_STAT_OFFSET (0x05bc) /* Clock Mux 10 Divider Update Status Register (MUX_10_DIV_UPD_STAT) */
#define S32K3XX_MC_CGM_MUX_11_CSC_OFFSET          (0x05c0) /* Clock Mux 11 Select Control Register (MUX_11_CSC) */
#define S32K3XX_MC_CGM_MUX_11_CSS_OFFSET          (0x05c4) /* Clock Mux 11 Select Status Register (MUX_11_CSS) */
#define S32K3XX_MC_CGM_MUX_11_DC_0_OFFSET         (0x05c8) /* Clock Mux 11 Divider 0 Control Register (MUX_11_DC_0) */
#define S32K3XX_MC_CGM_MUX_11_DIV_UPD_STAT_OFFSET (0x05fc) /* Clock Mux 11 Divider Update Status Register (MUX_11_DIV_UPD_STAT) */

/* Relative offset to S32K3XX_MC_CGM_MUX_X_CSC_OFFSET */

#define S32K3XX_MC_CGM_MUX_X_CSC_OFFSET           (0x0000) /* Clock Mux X Select Control Register (MUX_X_CSC) */
#define S32K3XX_MC_CGM_MUX_X_CSS_OFFSET           (0x0004) /* Clock Mux X Select Status Register (MUX_X_CSS) */
#define S32K3XX_MC_CGM_MUX_X_DC_0_OFFSET          (0x0008) /* Clock Mux X Divider 0 Control Register (MUX_X_DC_0) */
#define S32K3XX_MC_CGM_MUX_X_DIV_UPD_STAT_OFFSET  (0x003c) /* Clock Mux X Divider Update Status Register (MUX_X_DIV_UPD_STAT) */

/* MC_CGM Register Addresses ************************************************/

#define S32K3XX_MC_CGM_PCFS_SFDUR                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_PCFS_SFDUR_OFFSET)
#define S32K3XX_MC_CGM_PCFS_DIVC8                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_PCFS_DIVC8_OFFSET)
#define S32K3XX_MC_CGM_PCFS_DIVE8                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_PCFS_DIVE8_OFFSET)
#define S32K3XX_MC_CGM_PCFS_DIVS8                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_PCFS_DIVS8_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_1                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_1_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_2                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_2_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_3                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_3_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_4                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_4_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_5                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_5_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DC_6                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DC_6_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DIV_TRIG_CTRL        (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DIV_TRIG_CTRL_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DIV_TRIG             (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DIV_TRIG_OFFSET)
#define S32K3XX_MC_CGM_MUX_0_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_0_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_1_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_1_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_1_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_1_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_1_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_1_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_1_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_1_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_2_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_2_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_2_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_2_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_2_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_2_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_2_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_2_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_3_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_3_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_3_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_3_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_3_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_3_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_3_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_3_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_4_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_4_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_4_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_4_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_4_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_4_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_4_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_4_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_5_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_5_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_5_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_5_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_5_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_5_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_5_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_5_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_6_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_6_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_6_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_6_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_6_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_6_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_6_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_6_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_7_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_7_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_7_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_7_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_7_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_7_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_7_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_7_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_8_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_8_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_8_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_8_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_8_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_8_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_8_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_8_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_9_CSC                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_9_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_9_CSS                  (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_9_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_9_DC_0                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_9_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_9_DIV_UPD_STAT         (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_9_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_10_CSC                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_10_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_10_CSS                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_10_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_10_DC_0                (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_10_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_10_DIV_UPD_STAT        (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_10_DIV_UPD_STAT_OFFSET)
#define S32K3XX_MC_CGM_MUX_11_CSC                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_11_CSC_OFFSET)
#define S32K3XX_MC_CGM_MUX_11_CSS                 (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_11_CSS_OFFSET)
#define S32K3XX_MC_CGM_MUX_11_DC_0                (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_11_DC_0_OFFSET)
#define S32K3XX_MC_CGM_MUX_11_DIV_UPD_STAT        (S32K3XX_MC_CGM_BASE + S32K3XX_MC_CGM_MUX_11_DIV_UPD_STAT_OFFSET)

/* MC_CGM Register Bitfield Definitions *************************************/

/* PCFS Step Duration Register (PCFS_SDUR) */

#define MC_CGM_PCFS_SDUR_SHIFT            (0)       /* Bits 0-15: Step duration (SDUR) */
#define MC_CGM_PCFS_SDUR_MASK             (0xffff << MC_CGM_PCFS_SDUR_SHIFT)
                                                    /* Bits 16-31: Reserved */

/* PCFS Divider Change 8 Register (PCFS_DIVC8) */

#define MC_CGM_PCFS_DIVC8_RATE_SHIFT      (0)       /* Bits 0-7: Divider change rate (RATE) */
#define MC_CGM_PCFS_DIVC8_RATE_MASK       (0xff << MC_CGM_PCFS_DIVC8_RATE_SHIFT)
                                                    /* Bits 8-15: Reserved */
#define MC_CGM_PCFS_DIVC8_INIT_SHIFT      (16)      /* Bits 16-31: Divider change initial value (INIT) */
#define MC_CGM_PCFS_DIVC8_INIT_MASK       (0xffff << MC_CGM_PCFS_DIVC8_INIT_SHIFT)

/* PCFS Divider End 8 Register (PCFS_DIVE8) */

#define MC_CGM_PCFS_DIVE8_DIVE_SHIFT      (0)       /* Bits 0-19: Divider end value (DIVE) */
#define MC_CGM_PCFS_DIVE8_DIVE_MASK       (0x0fffff << MC_CGM_PCFS_DIVE8_DIVE_SHIFT)
                                                    /* Bits 20-31: Reserved */

/* PCFS Divider Start 8 Register (PCFS_DIVS8) */

#define MC_CGM_PCFS_DIVS8_DIVS_SHIFT      (0)       /* Bits 0-19: Divider start value (DIVS) */
#define MC_CGM_PCFS_DIVS8_DIVS_MASK       (0x0fffff << MC_CGM_PCFS_DIVS8_DIVS_SHIFT)
                                                    /* Bits 20-31: Reserved */

/* Clock Mux n Divider n Control Register (MC_CGM_MUX_n_DC_n) */

#define MC_CGM_MUX_DC_DE                  (1 << 31)                                  /* Bit 31: Divider Enable (DE) */
#define MC_CGM_MUX_DC_DIV_SHIFT           (16)                                       /* Bit 16-19: Divider (DIV) */
#define MC_CGM_MUX_DC_DIV_MASK            (0xf << MC_CGM_MUX_DC_DIV_SHIFT)           /* Bit 16-19: Divider (DIV) */
#define MC_CGM_MUX_DC_DIV(n)              ((0xf & (n-1)) << MC_CGM_MUX_DC_DIV_SHIFT) /* Bit 16-19: Divider (DIV) */

/* Clock Mux n Select Control Register (MUX_n_CSC) */

#define MC_CGM_MUX_CSC_RAMPUP             (1 << 0)  /* Bit 0: PCFS ramp-up (RAMPUP) */
#define MC_CGM_MUX_CSC_RAMPDOWN           (1 << 1)  /* Bit 1: PCFS ramp-down (RAMPDOWN) */
#define MC_CGM_MUX_CSC_CLK_SW             (1 << 2)  /* Bit 2: Clock switch (CLK_SW) */
#define MC_CGM_MUX_CSC_CG                 (1 << 2)  /* Bit 2: Clock gate (CG) */
#define MC_CGM_MUX_CSC_SAFE_SW            (1 << 3)  /* Bit 3: Safe clock request (SAFE_SW) */
#define MC_CGM_MUX_CSC_FCG                (1 << 3)  /* Bit 3: Force clock gate (FCG) */
                                                    /* Bits 4-23: Reserved */

#define MC_CGM_MUX_CSC_SELCTL_SHIFT            (24) /* Bits 24-29: Clock source selection control (SELCTL) */
#define MC_CGM_MUX_0_CSC_SELCTL_MASK           (0x0f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_1_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_2_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_3_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_4_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_5_CSC_SELCTL_MASK           (0x3f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_6_CSC_SELCTL_MASK           (0x3f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_7_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_8_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_9_CSC_SELCTL_MASK           (0x1f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_10_CSC_SELCTL_MASK          (0x0f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_11_CSC_SELCTL_MASK          (0x0f << MC_CGM_MUX_CSC_SELCTL_SHIFT)
#define MC_CGM_MUX_CSC_SELCTL_FIRC             (0x00 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* FIRC */
#define MC_CGM_MUX_CSC_SELCTL_SIRC             (0x01 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* SIRC */
#define MC_CGM_MUX_CSC_SELCTL_FXOSC            (0x02 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* FXOSC */
#define MC_CGM_MUX_CSC_SELCTL_SXOSC            (0x04 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* SXOSC */
#define MC_CGM_MUX_CSC_SELCTL_PLL_PHI0_CLK     (0x08 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* PLL_PHI0_CLK */
#define MC_CGM_MUX_CSC_SELCTL_PLL_PHI1_CLK     (0x09 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* PLL_PHI1_CLK */
#define MC_CGM_MUX_CSC_SELCTL_CORE_CLK         (0x10 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* CORE_CLK */
#define MC_CGM_MUX_CSC_SELCTL_HSE_CLK          (0x13 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* HSE_CLK */
#define MC_CGM_MUX_CSC_SELCTL_AIPS_PLAT_CLK    (0x16 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* AIPS_PLAT_CLK */
#define MC_CGM_MUX_CSC_SELCTL_AIPS_SLOW_CLK    (0x17 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* AIPS_SLOW_CLK */
#define MC_CGM_MUX_CSC_SELCTL_EMAC_RMII_TX_CLK (0x18 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* EMAC_RMII_TX_CLK */
#define MC_CGM_MUX_CSC_SELCTL_EMAC_RX_CLK      (0x19 << MC_CGM_MUX_CSC_SELCTL_SHIFT) /* EMAC_RX_CLK */

                                                    /* Bits 30-31: Reserved */

/* Clock Mux n Select Status Register (MUX_n_CSS) */

#define MC_CGM_MUX_CSS_RAMPUP             (1 << 0)  /* Bit 0: PCFS ramp-up (RAMPUP) */
#define MC_CGM_MUX_CSS_RAMPDOWN           (1 << 1)  /* Bit 1: PCFS ramp-down (RAMPDOWN) */
#define MC_CGM_MUX_CSS_CLK_SW             (1 << 2)  /* Bit 2: Clock switch (CLK_SW) */
#define MC_CGM_MUX_CSS_SAFE_SW            (1 << 3)  /* Bit 3: Safe clock request (SAFE_SW) */
                                                    /* Bit 4-15: Reserved */
#define MC_CGM_MUX_CSS_SWIP               (1 << 16) /* Bit 16: Switch in progress (SWIP) */
#define MC_CGM_MUX_CSS_GRIP               (1 << 16) /* Bit 16: Grating request is in progress (GRIP) */
#define MC_CGM_MUX_CSS_SWTRG_SHIFT        (17)      /* Bits 17-19: Switch trigger cause (SWTRG) */
#define MC_CGM_MUX_CSS_SWTRG_MASK         (0x07 << MC_CGM_MUX_CSS_SWTRG_SHIFT)
#define MC_CGM_MUX_CSS_SWTRG_RQS          (0x01 << MC_CGM_MUX_CSS_SWTRG_SHIFT) /* Switch after request succeeded */
#define MC_CGM_MUX_CSS_SWTRG_RQFINT       (0x02 << MC_CGM_MUX_CSS_SWTRG_SHIFT) /* Switch after the request failed because of an inactive target clock and the current clock is FIRC */
#define MC_CGM_MUX_CSS_SWTRG_RQFINC       (0x03 << MC_CGM_MUX_CSS_SWTRG_SHIFT) /* Switch after the request failed because of an inactive current clock and the current clock is FIRC */
#define MC_CGM_MUX_CSS_SWTRG_SCRQ         (0x04 << MC_CGM_MUX_CSS_SWTRG_SHIFT) /* Switch to FIRC because of a safe clock request or reset succeeded */
#define MC_CGM_MUX_CSS_SWTRG_SCRQPI       (0x05 << MC_CGM_MUX_CSS_SWTRG_SHIFT) /* Switch to FIRC because of a safe clock request or reset succeeded, but the previous current clock source was inactive */

#define MC_CGM_MUX_CSS_CS                 (1 << 17) /* Bit 17: Clock status (CS) */
                                                    /* Bits 20-23: Reserved */

#define MC_CGM_MUX_CSS_SELSTAT_SHIFT            (24) /* Bits 24-29: Clock source selection status (SELSTAT) */
#define MC_CGM_MUX_0_CSS_SELSTAT_MASK           (0x0f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_1_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_2_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_3_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_4_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_5_CSS_SELSTAT_MASK           (0x3f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_6_CSS_SELSTAT_MASK           (0x3f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_7_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_8_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_9_CSS_SELSTAT_MASK           (0x1f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_10_CSS_SELSTAT_MASK          (0x0f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_11_CSS_SELSTAT_MASK          (0x0f << MC_CGM_MUX_CSS_SELSTAT_SHIFT)
#define MC_CGM_MUX_CSS_SELSTAT_FIRC             (0x00 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* FIRC */
#define MC_CGM_MUX_CSS_SELSTAT_SIRC             (0x01 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* SIRC */
#define MC_CGM_MUX_CSS_SELSTAT_FXOSC            (0x02 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* FXOSC */
#define MC_CGM_MUX_CSS_SELSTAT_SXOSC            (0x04 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* SXOSC */
#define MC_CGM_MUX_CSS_SELSTAT_PLL_PHI0_CLK     (0x08 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* PLL_PHI0_CLK */
#define MC_CGM_MUX_CSS_SELSTAT_PLL_PHI1_CLK     (0x09 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* PLL_PHI1_CLK */
#define MC_CGM_MUX_CSS_SELSTAT_CORE_CLK         (0x10 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* CORE_CLK */
#define MC_CGM_MUX_CSS_SELSTAT_HSE_CLK          (0x13 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* HSE_CLK */
#define MC_CGM_MUX_CSC_SELSTAT_AIPS_PLAT_CLK    (0x16 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* AIPS_PLAT_CLK */
#define MC_CGM_MUX_CSC_SELSTAT_AIPS_SLOW_CLK    (0x17 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* AIPS_SLOW_CLK */
#define MC_CGM_MUX_CSC_SELSTAT_EMAC_RMII_TX_CLK (0x18 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* EMAC_RMII_TX_CLK */
#define MC_CGM_MUX_CSC_SELSTAT_EMAC_RX_CLK      (0x19 << MC_CGM_MUX_CSS_SELSTAT_SHIFT) /* EMAC_RX_CLK */

                                                    /* Bits 30-31: Reserved */

/* Clock Mux 0 Divider Trigger Control Register (MUX_0_DIV_TRIG_CTRL) */

#define MC_CGM_MUX_0_DIV_TRIG_CTRL_TCTL   (1 << 0)  /* Bit 0: Trigger control (TCTL) */
                                                    /* Bits 1-30: Reserved */
#define MC_CGM_MUX_0_DIV_TRIG_CTRL_HHEN   (31 << 0) /* Bit 31: Halt handshake enable (HHEN) */

/* Clock Mux 0 Divider Trigger Register (MUX_0_DIV_TRIG) */

#define MC_CGM_MUX_0_DIV_TRIG_SHIFT       (0)       /* Bits 0-31: Trigger for divider update (TRIGGER) */
#define MC_CGM_MUX_0_DIV_TRIG_MASK        (0xffffffff << MC_CGM_MUX_0_DIV_TRIG_SHIFT)

/* Clock Mux n Divider Update Status Register (MUX_n_DIV_UPD_STAT) */

#define MC_CGM_MUX_DIV_UPD_STAT_DIV_STAT  (1 << 0)  /* Bit 0: Divider status for clock mux 0 (DIV_STAT) */
                                                    /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_MC_CGM_H */
