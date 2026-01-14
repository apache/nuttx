/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_hstx_ctrl.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_HSTX_CTRL_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_HSTX_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_HSTX_CTRL_CSR_OFFSET            0x00000000
#define RP23XX_HSTX_CTRL_BIT_OFFSET(n)         ((n) * 4 + 0x000004)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_OFFSET   0x00000024
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_OFFSET    0x00000028

/* Register definitions *****************************************************/

#define RP23XX_HSTX_CTRL_CSR           (RP23XX_HSTX_CTRL_BASE + RP23XX_HSTX_CTRL_CSR_OFFSET)
#define RP23XX_HSTX_CTRL_BIT(n)        (RP23XX_HSTX_CTRL_BASE + RP23XX_HSTX_CTRL_BIT_OFFSET(n))
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT  (RP23XX_HSTX_CTRL_BASE + RP23XX_HSTX_CTRL_EXPAND_SHIFT_OFFSET)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS   (RP23XX_HSTX_CTRL_BASE + RP23XX_HSTX_CTRL_EXPAND_TMDS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_HSTX_CTRL_CSR_MASK                           (0xff1f1f73)
#define RP23XX_HSTX_CTRL_CSR_CLKDIV_MASK                    (0xf0000000)
#define RP23XX_HSTX_CTRL_CSR_CLKPHASE_MASK                  (0x0f000000)
#define RP23XX_HSTX_CTRL_CSR_N_SHIFTS_MASK                  (0x001f0000)
#define RP23XX_HSTX_CTRL_CSR_SHIFT_MASK                     (0x00001f00)
#define RP23XX_HSTX_CTRL_CSR_COUPLED_SEL_MASK               (0x00000060)
#define RP23XX_HSTX_CTRL_CSR_COUPLED_MODE                   (1 << 4)
#define RP23XX_HSTX_CTRL_CSR_EXPAND_EN                      (1 << 1)
#define RP23XX_HSTX_CTRL_CSR_EN                             (1 << 0)
#define RP23XX_HSTX_CTRL_BIT_MASK                           (0x00031f1f)
#define RP23XX_HSTX_CTRL_BIT_CLK_MASK                       (1 << 25)
#define RP23XX_HSTX_CTRL_BIT_INV_MASK                       (1 << 24)
#define RP23XX_HSTX_CTRL_BIT_SEL_N_MASK                     (0x00001f00)
#define RP23XX_HSTX_CTRL_BIT_SEL_P_MASK                     (0x0000001f)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_MASK                  (0x1f1f1f1f)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_MASK     (0x1f000000)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_MASK        (0x001f0000)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_MASK     (0x00001f00)
#define RP23XX_HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_MASK        (0x0000001f)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_MASK                   (0x00ffffff)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L2_NBITS_MASK          (0x00e00000)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L2_ROT_MASK            (0x001f0000)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L1_NBITS_MASK          (0x0000e000)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L1_ROT_MASK            (0x00001f00)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L0_NBITS_MASK          (0x000000e0)
#define RP23XX_HSTX_CTRL_EXPAND_TMDS_L0_ROT_MASK            (0x0000001f)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_HSTX_CTRL_H */
