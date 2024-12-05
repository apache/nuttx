/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_adc.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ADC_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_ADC_CS_OFFSET        0x00000000
#define RP23XX_ADC_RESULT_OFFSET    0x00000004
#define RP23XX_ADC_FCS_OFFSET       0x00000008
#define RP23XX_ADC_FIFO_OFFSET      0x0000000c
#define RP23XX_ADC_DIV_OFFSET       0x00000010
#define RP23XX_ADC_INTR_OFFSET      0x00000014
#define RP23XX_ADC_INTE_OFFSET      0x00000018
#define RP23XX_ADC_INTF_OFFSET      0x0000001c
#define RP23XX_ADC_INTS_OFFSET      0x00000020

/* Register definitions *****************************************************/

#define RP23XX_ADC_CS          (RP23XX_ADC_BASE + RP23XX_ADC_CS_OFFSET)
#define RP23XX_ADC_RESULT      (RP23XX_ADC_BASE + RP23XX_ADC_RESULT_OFFSET)
#define RP23XX_ADC_FCS         (RP23XX_ADC_BASE + RP23XX_ADC_FCS_OFFSET)
#define RP23XX_ADC_FIFO        (RP23XX_ADC_BASE + RP23XX_ADC_FIFO_OFFSET)
#define RP23XX_ADC_DIV         (RP23XX_ADC_BASE + RP23XX_ADC_DIV_OFFSET)
#define RP23XX_ADC_INTR        (RP23XX_ADC_BASE + RP23XX_ADC_INTR_OFFSET)
#define RP23XX_ADC_INTE        (RP23XX_ADC_BASE + RP23XX_ADC_INTE_OFFSET)
#define RP23XX_ADC_INTF        (RP23XX_ADC_BASE + RP23XX_ADC_INTF_OFFSET)
#define RP23XX_ADC_INTS        (RP23XX_ADC_BASE + RP23XX_ADC_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_ADC_CS_MASK              (0x01fff70f)
#define RP23XX_ADC_CS_RROBIN_SHIFT      (16)
#define RP23XX_ADC_CS_RROBIN_MASK       (0x01ff << RP23XX_ADC_CS_RROBIN_SHIFT)
#define RP23XX_ADC_CS_AINSEL_SHIFT      (12)
#define RP23XX_ADC_CS_AINSEL_MASK       (0x000fl << RP23XX_ADC_CS_AINSEL_SHIFT)
#define RP23XX_ADC_CS_ERR_STICKY        (1 << 10)
#define RP23XX_ADC_CS_ERR               (1 << 9)
#define RP23XX_ADC_CS_READY             (1 << 8)
#define RP23XX_ADC_CS_START_MANY        (1 << 3)
#define RP23XX_ADC_CS_START_ONCE        (1 << 2)
#define RP23XX_ADC_CS_TS_EN             (1 << 1)
#define RP23XX_ADC_CS_EN                (1 << 0)
#define RP23XX_ADC_RESULT_MASK          (0x00000fff)
#define RP23XX_ADC_FCS_MASK             (0x0f0f0f0f)
#define RP23XX_ADC_FCS_THRESH_SHIFT     (24)
#define RP23XX_ADC_FCS_THRESH_MASK      (0x000fl << RP23XX_ADC_FCS_THRESH_SHIFT)
#define RP23XX_ADC_FCS_LEVEL_SHIFT      (16)
#define RP23XX_ADC_FCS_LEVEL_MASK       (0x000f << RP23XX_ADC_FCS_LEVEL_SHIFT)
#define RP23XX_ADC_FCS_OVER             (1 << 11)
#define RP23XX_ADC_FCS_UNDER            (1 << 10)
#define RP23XX_ADC_FCS_FULL             (1 << 9)
#define RP23XX_ADC_FCS_EMPTY            (1 << 8)
#define RP23XX_ADC_FCS_DREQ_EN          (1 << 3)
#define RP23XX_ADC_FCS_ERR              (1 << 2)
#define RP23XX_ADC_FCS_SHIFT            (1 << 1)
#define RP23XX_ADC_FCS_EN               (1 << 0)
#define RP23XX_ADC_FIFO_MASK            (0x00008fff)
#define RP23XX_ADC_FIFO_ERR             (1 << 15)
#define RP23XX_ADC_FIFO_VAL_MASK        (0x00000fff)
#define RP23XX_ADC_DIV_MASK             (0x00ffffff)
#define RP23XX_ADC_DIV_INT_MASK         (0x00ffff00)
#define RP23XX_ADC_DIV_FRAC_MASK        (0x000000ff)
#define RP23XX_ADC_INTR_FIFO            (1 << 0)
#define RP23XX_ADC_INTE_FIFO            (1 << 0)
#define RP23XX_ADC_INTF_FIFO            (1 << 0)
#define RP23XX_ADC_INTS_FIFO            (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_ADC_H */

