/****************************************************************************
 * arch/arm/src/ht32f491x3/hardware/ht32f491x3_tmr.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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

#ifndef __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_TMR_H
#define __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_TMR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define HT32_TMR_CTRL1_OFFSET          0x0000
#define HT32_TMR_CTRL2_OFFSET          0x0004
#define HT32_TMR_SWEVT_OFFSET          0x0014
#define HT32_TMR_CM1_OFFSET            0x0018
#define HT32_TMR_CM2_OFFSET            0x001c
#define HT32_TMR_CCTRL_OFFSET          0x0020
#define HT32_TMR_CVAL_OFFSET           0x0024
#define HT32_TMR_DIV_OFFSET            0x0028
#define HT32_TMR_PR_OFFSET             0x002c
#define HT32_TMR_RPR_OFFSET            0x0030
#define HT32_TMR_C1DT_OFFSET           0x0034
#define HT32_TMR_C2DT_OFFSET           0x0038
#define HT32_TMR_C3DT_OFFSET           0x003c
#define HT32_TMR_C4DT_OFFSET           0x0040
#define HT32_TMR_BRK_OFFSET            0x0044

/* Control register 1 *******************************************************/

#define HT32_TMR_CTRL1_TMREN           (1u << 0)
#define HT32_TMR_CTRL1_CNTDIR_SHIFT    4
#define HT32_TMR_CTRL1_CNTDIR_MASK     (7u << HT32_TMR_CTRL1_CNTDIR_SHIFT)
#define HT32_TMR_CTRL1_COUNTUP         (0u << HT32_TMR_CTRL1_CNTDIR_SHIFT)
#define HT32_TMR_CTRL1_PRBEN           (1u << 7)

/* Event generation register ************************************************/

#define HT32_TMR_SWEVT_OVFSWTR         (1u << 0)

/* Compare mode register helpers ********************************************/

#define HT32_TMR_CM_CAPTURE_SEL_SHIFT(slot)   ((slot) * 8)
#define HT32_TMR_CM_CAPTURE_SEL_MASK(slot)    (3u << \
                                               HT32_TMR_CM_CAPTURE_SEL_SHIFT(slot))
#define HT32_TMR_CM_OUTPUT_BUFFER_SHIFT(slot) ((slot) * 8 + 3)
#define HT32_TMR_CM_OUTPUT_BUFFER(slot)       (1u << \
                                               HT32_TMR_CM_OUTPUT_BUFFER_SHIFT(slot))
#define HT32_TMR_CM_OUTPUT_MODE_SHIFT(slot)   ((slot) * 8 + 4)
#define HT32_TMR_CM_OUTPUT_MODE_MASK(slot)    (7u << \
                                               HT32_TMR_CM_OUTPUT_MODE_SHIFT(slot))
#define HT32_TMR_CM_OUTPUT_MODE(slot, mode)   ((uint32_t)(mode) << \
                                               HT32_TMR_CM_OUTPUT_MODE_SHIFT(slot))

/* Capture compare control register helpers *********************************/

#define HT32_TMR_CCTRL_EN_SHIFT(ch)    (((ch) - 1u) * 4u)
#define HT32_TMR_CCTRL_EN(ch)          (1u << HT32_TMR_CCTRL_EN_SHIFT(ch))
#define HT32_TMR_CCTRL_POL_SHIFT(ch)   (HT32_TMR_CCTRL_EN_SHIFT(ch) + 1u)
#define HT32_TMR_CCTRL_POL(ch)         (1u << HT32_TMR_CCTRL_POL_SHIFT(ch))

/* Brake register ***********************************************************/

#define HT32_TMR_BRK_OEN               (1u << 15)

/* Output compare mode values ***********************************************/

#define HT32_TMR_OUTPUT_CONTROL_PWM_A  6u

/* Helper macros ************************************************************/

#define HT32_TMR_CCR_OFFSET(ch)        (HT32_TMR_C1DT_OFFSET + (((ch) - 1u) * 4u))

#endif /* __ARCH_ARM_SRC_HT32F491X3_HARDWARE_HT32F491X3_TMR_H */
