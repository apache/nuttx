/****************************************************************************
 * arch/arm/src/imx9/hardware/imx9_lptmr.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_LPTMR_H_
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_LPTMR_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define IMX9_LPTMR_CSR_OFFSET        0x0000 /* Control Status */
#define IMX9_LPTMR_PSR_OFFSET        0x0004 /* Prescale */
#define IMX9_LPTMR_CMR_OFFSET        0x0008 /* Compare */
#define IMX9_LPTMR_CNR_OFFSET        0x000c /* Counter */

/* Register Address *********************************************************/

#define LPTMR_CSR(n)                 ((n) + IMX9_LPTMR_CSR_OFFSET)
#define LPTMR_PSR(n)                 ((n) + IMX9_LPTMR_PSR_OFFSET)
#define LPTMR_CMR(n)                 ((n) + IMX9_LPTMR_CMR_OFFSET)
#define LPTMR_CNR(n)                 ((n) + IMX9_LPTMR_CNR_OFFSET)

/* Register Bitfield Definitions ********************************************/

#define LPTMR_CSR_TDRE               (1 << 8) /* Timer DMA Request Enable */
#define LPTMR_CSR_TCF                (1 << 7) /* Timer Compare Flag */
#define LPTMR_CSR_TIE                (1 << 6) /* Timer Interrupt Enable */

#define LPTMR_CSR_TPS_SHIFT          (4) /* Bit[5:4]: Timer Pin Select */
#define LPTMR_CSR_TPS_MASK           (3 << LPTMR_CSR_TPS_SHIFT)
#define LPTMR_CSR_TPS0               (0 << LPTMR_CSR_TPS_SHIFT)
#define LPTMR_CSR_TPS1               (1 << LPTMR_CSR_TPS_SHIFT)
#define LPTMR_CSR_TPS2               (2 << LPTMR_CSR_TPS_SHIFT)
#define LPTMR_CSR_TPS3               (3 << LPTMR_CSR_TPS_SHIFT)

#define LPTMR_CSR_TPP                (1 << 3) /* Timer Pin Polarity */
#define LPTMR_CSR_TFC                (1 << 2) /* Timer Free-Running Counter */
#define LPTMR_CSR_TMS                (1 << 1) /* Timer Mode Select */
#define LPTMR_CSR_TEN                (1 << 0) /* Timer Enable */

#define LPTMR_PSR_PRESCALE_SHIFT     (3) /* Bit[6:3]: Prescale Value */
#define LPTMR_PSR_PRESCALE_MASK      (0xf << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV2      (0   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV4      (1   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV8      (2   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV16     (3   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV32     (4   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV64     (5   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV128    (6   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV256    (7   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV512    (8   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV1024   (9   << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV2048   (10  << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV4096   (11  << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV8192   (12  << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV16384  (13  << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV32768  (14  << LPTMR_PSR_PRESCALE_SHIFT)
#define LPTMR_PSR_PRESCALE_DIV65536  (15  << LPTMR_PSR_PRESCALE_SHIFT)

#define LPTMR_PSR_PBYP               (1 << 2) /* Prescaler Bypass */

/* Clock sources (module clock / root clock)
 * ipg_clk_irclk -> lptmrx_clk_root
 * ipg_clk_1khz  -> 32k_clk_root
 * ipg_clk_32khz -> 32k_clk_root
 * ipg_clk_ercl  -> 32k_clk_root
 */

#define LPTMR_PSR_PCS_SHIFT          (0) /* Bit[1:0]: Prescaler Clock Select */
#define LPTMR_PSR_PCS_MASK           (3 << LPTMR_PSR_PCS_SHIFT)
#define LPTMR_PSR_PCS_REF_INT        (0 << LPTMR_PSR_PCS_SHIFT) /* Internal reference clock */
#define LPTMR_PSR_PCS_LPO            (1 << LPTMR_PSR_PCS_SHIFT) /* LPO 1K Hz */
#define LPTMR_PSR_PCS_RTC            (2 << LPTMR_PSR_PCS_SHIFT) /* RTC 32768 Hz */
#define LPTMR_PSR_PCS_REF_EXT        (3 << LPTMR_PSR_PCS_SHIFT) /* External reference clock */
#  define LPTMR_PSR_PCS_SOSC         LPTMR_PSR_PCS_RFOSC

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_LPTMR_H_ */
