/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_timer.h
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_TIMER_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/bl602_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 0x0 : TCCR */

#define TIMER_TCCR_OFFSET (0x0)
#define TIMER_CS_1        TIMER_CS_1
#define TIMER_CS_1_POS    (2U)
#define TIMER_CS_1_LEN    (2U)
#define TIMER_CS_1_MSK    (((1U << TIMER_CS_1_LEN) - 1) << TIMER_CS_1_POS)
#define TIMER_CS_1_UMSK   (~(((1U << TIMER_CS_1_LEN) - 1) << TIMER_CS_1_POS))
#define TIMER_CS_2        TIMER_CS_2
#define TIMER_CS_2_POS    (5U)
#define TIMER_CS_2_LEN    (2U)
#define TIMER_CS_2_MSK    (((1U << TIMER_CS_2_LEN) - 1) << TIMER_CS_2_POS)
#define TIMER_CS_2_UMSK   (~(((1U << TIMER_CS_2_LEN) - 1) << TIMER_CS_2_POS))
#define TIMER_CS_WDT      TIMER_CS_WDT
#define TIMER_CS_WDT_POS  (8U)
#define TIMER_CS_WDT_LEN  (2U)
#define TIMER_CS_WDT_MSK  (((1U << TIMER_CS_WDT_LEN) - 1) << TIMER_CS_WDT_POS)
#define TIMER_CS_WDT_UMSK \
  (~(((1U << TIMER_CS_WDT_LEN) - 1) << TIMER_CS_WDT_POS))

/* 0x10 : TMR2_0 */

#define TIMER_TMR2_0_OFFSET (0x10)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x14 : TMR2_1 */

#define TIMER_TMR2_1_OFFSET (0x14)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x18 : TMR2_2 */

#define TIMER_TMR2_2_OFFSET (0x18)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x1C : TMR3_0 */

#define TIMER_TMR3_0_OFFSET (0x1C)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x20 : TMR3_1 */

#define TIMER_TMR3_1_OFFSET (0x20)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x24 : TMR3_2 */

#define TIMER_TMR3_2_OFFSET (0x24)
#define TIMER_TMR           TIMER_TMR
#define TIMER_TMR_POS       (0U)
#define TIMER_TMR_LEN       (32U)
#define TIMER_TMR_MSK       (((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS)
#define TIMER_TMR_UMSK      (~(((1U << TIMER_TMR_LEN) - 1) << TIMER_TMR_POS))

/* 0x2C : TCR2 */

#define TIMER_TCR2_OFFSET (0x2C)
#define TIMER_TCR         TIMER_TCR
#define TIMER_TCR_POS     (0U)
#define TIMER_TCR_LEN     (32U)
#define TIMER_TCR_MSK     (((1U << TIMER_TCR_LEN) - 1) << TIMER_TCR_POS)
#define TIMER_TCR_UMSK    (~(((1U << TIMER_TCR_LEN) - 1) << TIMER_TCR_POS))

/* 0x30 : TCR3 */

#define TIMER_TCR3_OFFSET      (0x30)
#define TIMER_TCR3_COUNTER     TIMER_TCR3_COUNTER
#define TIMER_TCR3_COUNTER_POS (0U)
#define TIMER_TCR3_COUNTER_LEN (32U)
#define TIMER_TCR3_COUNTER_MSK \
  (((1U << TIMER_TCR3_COUNTER_LEN) - 1) << TIMER_TCR3_COUNTER_POS)
#define TIMER_TCR3_COUNTER_UMSK \
  (~(((1U << TIMER_TCR3_COUNTER_LEN) - 1) << TIMER_TCR3_COUNTER_POS))

/* 0x38 : TMSR2 */

#define TIMER_TMSR2_OFFSET (0x38)
#define TIMER_TMSR_0       TIMER_TMSR_0
#define TIMER_TMSR_0_POS   (0U)
#define TIMER_TMSR_0_LEN   (1U)
#define TIMER_TMSR_0_MSK   (((1U << TIMER_TMSR_0_LEN) - 1) << TIMER_TMSR_0_POS)
#define TIMER_TMSR_0_UMSK \
  (~(((1U << TIMER_TMSR_0_LEN) - 1) << TIMER_TMSR_0_POS))
#define TIMER_TMSR_1     TIMER_TMSR_1
#define TIMER_TMSR_1_POS (1U)
#define TIMER_TMSR_1_LEN (1U)
#define TIMER_TMSR_1_MSK (((1U << TIMER_TMSR_1_LEN) - 1) << TIMER_TMSR_1_POS)
#define TIMER_TMSR_1_UMSK \
  (~(((1U << TIMER_TMSR_1_LEN) - 1) << TIMER_TMSR_1_POS))
#define TIMER_TMSR_2     TIMER_TMSR_2
#define TIMER_TMSR_2_POS (2U)
#define TIMER_TMSR_2_LEN (1U)
#define TIMER_TMSR_2_MSK (((1U << TIMER_TMSR_2_LEN) - 1) << TIMER_TMSR_2_POS)
#define TIMER_TMSR_2_UMSK \
  (~(((1U << TIMER_TMSR_2_LEN) - 1) << TIMER_TMSR_2_POS))

/* 0x3C : TMSR3 */

#define TIMER_TMSR3_OFFSET (0x3C)
#define TIMER_TMSR_0       TIMER_TMSR_0
#define TIMER_TMSR_0_POS   (0U)
#define TIMER_TMSR_0_LEN   (1U)
#define TIMER_TMSR_0_MSK   (((1U << TIMER_TMSR_0_LEN) - 1) << TIMER_TMSR_0_POS)
#define TIMER_TMSR_0_UMSK \
  (~(((1U << TIMER_TMSR_0_LEN) - 1) << TIMER_TMSR_0_POS))
#define TIMER_TMSR_1     TIMER_TMSR_1
#define TIMER_TMSR_1_POS (1U)
#define TIMER_TMSR_1_LEN (1U)
#define TIMER_TMSR_1_MSK (((1U << TIMER_TMSR_1_LEN) - 1) << TIMER_TMSR_1_POS)
#define TIMER_TMSR_1_UMSK \
  (~(((1U << TIMER_TMSR_1_LEN) - 1) << TIMER_TMSR_1_POS))
#define TIMER_TMSR_2     TIMER_TMSR_2
#define TIMER_TMSR_2_POS (2U)
#define TIMER_TMSR_2_LEN (1U)
#define TIMER_TMSR_2_MSK (((1U << TIMER_TMSR_2_LEN) - 1) << TIMER_TMSR_2_POS)
#define TIMER_TMSR_2_UMSK \
  (~(((1U << TIMER_TMSR_2_LEN) - 1) << TIMER_TMSR_2_POS))

/* 0x44 : TIER2 */

#define TIMER_TIER2_OFFSET (0x44)
#define TIMER_TIER_0       TIMER_TIER_0
#define TIMER_TIER_0_POS   (0U)
#define TIMER_TIER_0_LEN   (1U)
#define TIMER_TIER_0_MSK   (((1U << TIMER_TIER_0_LEN) - 1) << TIMER_TIER_0_POS)
#define TIMER_TIER_0_UMSK \
  (~(((1U << TIMER_TIER_0_LEN) - 1) << TIMER_TIER_0_POS))
#define TIMER_TIER_1     TIMER_TIER_1
#define TIMER_TIER_1_POS (1U)
#define TIMER_TIER_1_LEN (1U)
#define TIMER_TIER_1_MSK (((1U << TIMER_TIER_1_LEN) - 1) << TIMER_TIER_1_POS)
#define TIMER_TIER_1_UMSK \
  (~(((1U << TIMER_TIER_1_LEN) - 1) << TIMER_TIER_1_POS))
#define TIMER_TIER_2     TIMER_TIER_2
#define TIMER_TIER_2_POS (2U)
#define TIMER_TIER_2_LEN (1U)
#define TIMER_TIER_2_MSK (((1U << TIMER_TIER_2_LEN) - 1) << TIMER_TIER_2_POS)
#define TIMER_TIER_2_UMSK \
  (~(((1U << TIMER_TIER_2_LEN) - 1) << TIMER_TIER_2_POS))

/* 0x48 : TIER3 */

#define TIMER_TIER3_OFFSET (0x48)
#define TIMER_TIER_0       TIMER_TIER_0
#define TIMER_TIER_0_POS   (0U)
#define TIMER_TIER_0_LEN   (1U)
#define TIMER_TIER_0_MSK   (((1U << TIMER_TIER_0_LEN) - 1) << TIMER_TIER_0_POS)
#define TIMER_TIER_0_UMSK \
  (~(((1U << TIMER_TIER_0_LEN) - 1) << TIMER_TIER_0_POS))
#define TIMER_TIER_1     TIMER_TIER_1
#define TIMER_TIER_1_POS (1U)
#define TIMER_TIER_1_LEN (1U)
#define TIMER_TIER_1_MSK (((1U << TIMER_TIER_1_LEN) - 1) << TIMER_TIER_1_POS)
#define TIMER_TIER_1_UMSK \
  (~(((1U << TIMER_TIER_1_LEN) - 1) << TIMER_TIER_1_POS))
#define TIMER_TIER_2     TIMER_TIER_2
#define TIMER_TIER_2_POS (2U)
#define TIMER_TIER_2_LEN (1U)
#define TIMER_TIER_2_MSK (((1U << TIMER_TIER_2_LEN) - 1) << TIMER_TIER_2_POS)
#define TIMER_TIER_2_UMSK \
  (~(((1U << TIMER_TIER_2_LEN) - 1) << TIMER_TIER_2_POS))

/* 0x50 : TPLVR2 */

#define TIMER_TPLVR2_OFFSET (0x50)
#define TIMER_TPLVR         TIMER_TPLVR
#define TIMER_TPLVR_POS     (0U)
#define TIMER_TPLVR_LEN     (32U)
#define TIMER_TPLVR_MSK     (((1U << TIMER_TPLVR_LEN) - 1) << TIMER_TPLVR_POS)
#define TIMER_TPLVR_UMSK    (~(((1U << TIMER_TPLVR_LEN) - 1) << TIMER_TPLVR_POS))

/* 0x54 : TPLVR3 */
#define TIMER_TPLVR3_OFFSET (0x54)
#define TIMER_TPLVR         TIMER_TPLVR
#define TIMER_TPLVR_POS     (0U)
#define TIMER_TPLVR_LEN     (32U)
#define TIMER_TPLVR_MSK     (((1U << TIMER_TPLVR_LEN) - 1) << TIMER_TPLVR_POS)
#define TIMER_TPLVR_UMSK    (~(((1U << TIMER_TPLVR_LEN) - 1) << TIMER_TPLVR_POS))

/* 0x5C : TPLCR2 */

#define TIMER_TPLCR2_OFFSET (0x5C)
#define TIMER_TPLCR         TIMER_TPLCR
#define TIMER_TPLCR_POS     (0U)
#define TIMER_TPLCR_LEN     (2U)
#define TIMER_TPLCR_MSK     (((1U << TIMER_TPLCR_LEN) - 1) << TIMER_TPLCR_POS)
#define TIMER_TPLCR_UMSK    (~(((1U << TIMER_TPLCR_LEN) - 1) << TIMER_TPLCR_POS))

/* 0x60 : TPLCR3 */

#define TIMER_TPLCR3_OFFSET (0x60)
#define TIMER_TPLCR         TIMER_TPLCR
#define TIMER_TPLCR_POS     (0U)
#define TIMER_TPLCR_LEN     (2U)
#define TIMER_TPLCR_MSK     (((1U << TIMER_TPLCR_LEN) - 1) << TIMER_TPLCR_POS)
#define TIMER_TPLCR_UMSK    (~(((1U << TIMER_TPLCR_LEN) - 1) << TIMER_TPLCR_POS))

/* 0x64 : WMER */

#define TIMER_WMER_OFFSET (0x64)
#define TIMER_WE          TIMER_WE
#define TIMER_WE_POS      (0U)
#define TIMER_WE_LEN      (1U)
#define TIMER_WE_MSK      (((1U << TIMER_WE_LEN) - 1) << TIMER_WE_POS)
#define TIMER_WE_UMSK     (~(((1U << TIMER_WE_LEN) - 1) << TIMER_WE_POS))
#define TIMER_WRIE        TIMER_WRIE
#define TIMER_WRIE_POS    (1U)
#define TIMER_WRIE_LEN    (1U)
#define TIMER_WRIE_MSK    (((1U << TIMER_WRIE_LEN) - 1) << TIMER_WRIE_POS)
#define TIMER_WRIE_UMSK   (~(((1U << TIMER_WRIE_LEN) - 1) << TIMER_WRIE_POS))

/* 0x68 : WMR */

#define TIMER_WMR_OFFSET (0x68)
#define TIMER_WMR        TIMER_WMR
#define TIMER_WMR_POS    (0U)
#define TIMER_WMR_LEN    (16U)
#define TIMER_WMR_MSK    (((1U << TIMER_WMR_LEN) - 1) << TIMER_WMR_POS)
#define TIMER_WMR_UMSK   (~(((1U << TIMER_WMR_LEN) - 1) << TIMER_WMR_POS))

/* 0x6C : WVR */

#define TIMER_WVR_OFFSET (0x6C)
#define TIMER_WVR        TIMER_WVR
#define TIMER_WVR_POS    (0U)
#define TIMER_WVR_LEN    (16U)
#define TIMER_WVR_MSK    (((1U << TIMER_WVR_LEN) - 1) << TIMER_WVR_POS)
#define TIMER_WVR_UMSK   (~(((1U << TIMER_WVR_LEN) - 1) << TIMER_WVR_POS))

/* 0x70 : WSR */

#define TIMER_WSR_OFFSET (0x70)
#define TIMER_WTS        TIMER_WTS
#define TIMER_WTS_POS    (0U)
#define TIMER_WTS_LEN    (1U)
#define TIMER_WTS_MSK    (((1U << TIMER_WTS_LEN) - 1) << TIMER_WTS_POS)
#define TIMER_WTS_UMSK   (~(((1U << TIMER_WTS_LEN) - 1) << TIMER_WTS_POS))

/* 0x78 : TICR2 */

#define TIMER_TICR2_OFFSET (0x78)
#define TIMER_TCLR_0       TIMER_TCLR_0
#define TIMER_TCLR_0_POS   (0U)
#define TIMER_TCLR_0_LEN   (1U)
#define TIMER_TCLR_0_MSK   (((1U << TIMER_TCLR_0_LEN) - 1) << TIMER_TCLR_0_POS)
#define TIMER_TCLR_0_UMSK \
  (~(((1U << TIMER_TCLR_0_LEN) - 1) << TIMER_TCLR_0_POS))
#define TIMER_TCLR_1     TIMER_TCLR_1
#define TIMER_TCLR_1_POS (1U)
#define TIMER_TCLR_1_LEN (1U)
#define TIMER_TCLR_1_MSK (((1U << TIMER_TCLR_1_LEN) - 1) << TIMER_TCLR_1_POS)
#define TIMER_TCLR_1_UMSK \
  (~(((1U << TIMER_TCLR_1_LEN) - 1) << TIMER_TCLR_1_POS))
#define TIMER_TCLR_2     TIMER_TCLR_2
#define TIMER_TCLR_2_POS (2U)
#define TIMER_TCLR_2_LEN (1U)
#define TIMER_TCLR_2_MSK (((1U << TIMER_TCLR_2_LEN) - 1) << TIMER_TCLR_2_POS)
#define TIMER_TCLR_2_UMSK \
  (~(((1U << TIMER_TCLR_2_LEN) - 1) << TIMER_TCLR_2_POS))

/* 0x7C : TICR3 */

#define TIMER_TICR3_OFFSET (0x7C)
#define TIMER_TCLR_0       TIMER_TCLR_0
#define TIMER_TCLR_0_POS   (0U)
#define TIMER_TCLR_0_LEN   (1U)
#define TIMER_TCLR_0_MSK   (((1U << TIMER_TCLR_0_LEN) - 1) << TIMER_TCLR_0_POS)
#define TIMER_TCLR_0_UMSK \
  (~(((1U << TIMER_TCLR_0_LEN) - 1) << TIMER_TCLR_0_POS))
#define TIMER_TCLR_1     TIMER_TCLR_1
#define TIMER_TCLR_1_POS (1U)
#define TIMER_TCLR_1_LEN (1U)
#define TIMER_TCLR_1_MSK (((1U << TIMER_TCLR_1_LEN) - 1) << TIMER_TCLR_1_POS)
#define TIMER_TCLR_1_UMSK \
  (~(((1U << TIMER_TCLR_1_LEN) - 1) << TIMER_TCLR_1_POS))
#define TIMER_TCLR_2     TIMER_TCLR_2
#define TIMER_TCLR_2_POS (2U)
#define TIMER_TCLR_2_LEN (1U)
#define TIMER_TCLR_2_MSK (((1U << TIMER_TCLR_2_LEN) - 1) << TIMER_TCLR_2_POS)
#define TIMER_TCLR_2_UMSK \
  (~(((1U << TIMER_TCLR_2_LEN) - 1) << TIMER_TCLR_2_POS))

/* 0x80 : WICR */

#define TIMER_WICR_OFFSET (0x80)
#define TIMER_WICLR       TIMER_WICLR
#define TIMER_WICLR_POS   (0U)
#define TIMER_WICLR_LEN   (1U)
#define TIMER_WICLR_MSK   (((1U << TIMER_WICLR_LEN) - 1) << TIMER_WICLR_POS)
#define TIMER_WICLR_UMSK  (~(((1U << TIMER_WICLR_LEN) - 1) << TIMER_WICLR_POS))

/* 0x84 : TCER */

#define TIMER_TCER_OFFSET (0x84)
#define TIMER2_EN         TIMER2_EN
#define TIMER2_EN_POS     (1U)
#define TIMER2_EN_LEN     (1U)
#define TIMER2_EN_MSK     (((1U << TIMER2_EN_LEN) - 1) << TIMER2_EN_POS)
#define TIMER2_EN_UMSK    (~(((1U << TIMER2_EN_LEN) - 1) << TIMER2_EN_POS))
#define TIMER3_EN         TIMER3_EN
#define TIMER3_EN_POS     (2U)
#define TIMER3_EN_LEN     (1U)
#define TIMER3_EN_MSK     (((1U << TIMER3_EN_LEN) - 1) << TIMER3_EN_POS)
#define TIMER3_EN_UMSK    (~(((1U << TIMER3_EN_LEN) - 1) << TIMER3_EN_POS))

/* 0x88 : TCMR */

#define TIMER_TCMR_OFFSET (0x88)
#define TIMER2_MODE       TIMER2_MODE
#define TIMER2_MODE_POS   (1U)
#define TIMER2_MODE_LEN   (1U)
#define TIMER2_MODE_MSK   (((1U << TIMER2_MODE_LEN) - 1) << TIMER2_MODE_POS)
#define TIMER2_MODE_UMSK  (~(((1U << TIMER2_MODE_LEN) - 1) << TIMER2_MODE_POS))
#define TIMER3_MODE       TIMER3_MODE
#define TIMER3_MODE_POS   (2U)
#define TIMER3_MODE_LEN   (1U)
#define TIMER3_MODE_MSK   (((1U << TIMER3_MODE_LEN) - 1) << TIMER3_MODE_POS)
#define TIMER3_MODE_UMSK  (~(((1U << TIMER3_MODE_LEN) - 1) << TIMER3_MODE_POS))

/* 0x90 : TILR2 */

#define TIMER_TILR2_OFFSET (0x90)
#define TIMER_TILR_0       TIMER_TILR_0
#define TIMER_TILR_0_POS   (0U)
#define TIMER_TILR_0_LEN   (1U)
#define TIMER_TILR_0_MSK   (((1U << TIMER_TILR_0_LEN) - 1) << TIMER_TILR_0_POS)
#define TIMER_TILR_0_UMSK \
  (~(((1U << TIMER_TILR_0_LEN) - 1) << TIMER_TILR_0_POS))
#define TIMER_TILR_1     TIMER_TILR_1
#define TIMER_TILR_1_POS (1U)
#define TIMER_TILR_1_LEN (1U)
#define TIMER_TILR_1_MSK (((1U << TIMER_TILR_1_LEN) - 1) << TIMER_TILR_1_POS)
#define TIMER_TILR_1_UMSK \
  (~(((1U << TIMER_TILR_1_LEN) - 1) << TIMER_TILR_1_POS))
#define TIMER_TILR_2     TIMER_TILR_2
#define TIMER_TILR_2_POS (2U)
#define TIMER_TILR_2_LEN (1U)
#define TIMER_TILR_2_MSK (((1U << TIMER_TILR_2_LEN) - 1) << TIMER_TILR_2_POS)
#define TIMER_TILR_2_UMSK \
  (~(((1U << TIMER_TILR_2_LEN) - 1) << TIMER_TILR_2_POS))

/* 0x94 : TILR3 */

#define TIMER_TILR3_OFFSET (0x94)
#define TIMER_TILR_0       TIMER_TILR_0
#define TIMER_TILR_0_POS   (0U)
#define TIMER_TILR_0_LEN   (1U)
#define TIMER_TILR_0_MSK   (((1U << TIMER_TILR_0_LEN) - 1) << TIMER_TILR_0_POS)
#define TIMER_TILR_0_UMSK \
  (~(((1U << TIMER_TILR_0_LEN) - 1) << TIMER_TILR_0_POS))
#define TIMER_TILR_1     TIMER_TILR_1
#define TIMER_TILR_1_POS (1U)
#define TIMER_TILR_1_LEN (1U)
#define TIMER_TILR_1_MSK (((1U << TIMER_TILR_1_LEN) - 1) << TIMER_TILR_1_POS)
#define TIMER_TILR_1_UMSK \
  (~(((1U << TIMER_TILR_1_LEN) - 1) << TIMER_TILR_1_POS))
#define TIMER_TILR_2     TIMER_TILR_2
#define TIMER_TILR_2_POS (2U)
#define TIMER_TILR_2_LEN (1U)
#define TIMER_TILR_2_MSK (((1U << TIMER_TILR_2_LEN) - 1) << TIMER_TILR_2_POS)
#define TIMER_TILR_2_UMSK \
  (~(((1U << TIMER_TILR_2_LEN) - 1) << TIMER_TILR_2_POS))

/* 0x98 : WCR */

#define TIMER_WCR_OFFSET (0x98)
#define TIMER_WCR        TIMER_WCR
#define TIMER_WCR_POS    (0U)
#define TIMER_WCR_LEN    (1U)
#define TIMER_WCR_MSK    (((1U << TIMER_WCR_LEN) - 1) << TIMER_WCR_POS)
#define TIMER_WCR_UMSK   (~(((1U << TIMER_WCR_LEN) - 1) << TIMER_WCR_POS))

/* 0x9C : WFAR */

#define TIMER_WFAR_OFFSET (0x9C)
#define TIMER_WFAR        TIMER_WFAR
#define TIMER_WFAR_POS    (0U)
#define TIMER_WFAR_LEN    (16U)
#define TIMER_WFAR_MSK    (((1U << TIMER_WFAR_LEN) - 1) << TIMER_WFAR_POS)
#define TIMER_WFAR_UMSK   (~(((1U << TIMER_WFAR_LEN) - 1) << TIMER_WFAR_POS))

/* 0xA0 : WSAR */

#define TIMER_WSAR_OFFSET (0xA0)
#define TIMER_WSAR        TIMER_WSAR
#define TIMER_WSAR_POS    (0U)
#define TIMER_WSAR_LEN    (16U)
#define TIMER_WSAR_MSK    (((1U << TIMER_WSAR_LEN) - 1) << TIMER_WSAR_POS)
#define TIMER_WSAR_UMSK   (~(((1U << TIMER_WSAR_LEN) - 1) << TIMER_WSAR_POS))

/* 0xA8 : TCVWR2 */

#define TIMER_TCVWR2_OFFSET (0xA8)
#define TIMER_TCVWR         TIMER_TCVWR
#define TIMER_TCVWR_POS     (0U)
#define TIMER_TCVWR_LEN     (32U)
#define TIMER_TCVWR_MSK     (((1U << TIMER_TCVWR_LEN) - 1) << TIMER_TCVWR_POS)
#define TIMER_TCVWR_UMSK    (~(((1U << TIMER_TCVWR_LEN) - 1) << TIMER_TCVWR_POS))

/* 0xAC : TCVWR3 */

#define TIMER_TCVWR3_OFFSET (0xAC)
#define TIMER_TCVWR         TIMER_TCVWR
#define TIMER_TCVWR_POS     (0U)
#define TIMER_TCVWR_LEN     (32U)
#define TIMER_TCVWR_MSK     (((1U << TIMER_TCVWR_LEN) - 1) << TIMER_TCVWR_POS)
#define TIMER_TCVWR_UMSK    (~(((1U << TIMER_TCVWR_LEN) - 1) << TIMER_TCVWR_POS))

/* 0xB4 : TCVSYN2 */

#define TIMER_TCVSYN2_OFFSET (0xB4)
#define TIMER_TCVSYN2        TIMER_TCVSYN2
#define TIMER_TCVSYN2_POS    (0U)
#define TIMER_TCVSYN2_LEN    (32U)
#define TIMER_TCVSYN2_MSK \
  (((1U << TIMER_TCVSYN2_LEN) - 1) << TIMER_TCVSYN2_POS)
#define TIMER_TCVSYN2_UMSK \
  (~(((1U << TIMER_TCVSYN2_LEN) - 1) << TIMER_TCVSYN2_POS))

/* 0xB8 : TCVSYN3 */

#define TIMER_TCVSYN3_OFFSET (0xB8)
#define TIMER_TCVSYN3        TIMER_TCVSYN3
#define TIMER_TCVSYN3_POS    (0U)
#define TIMER_TCVSYN3_LEN    (32U)
#define TIMER_TCVSYN3_MSK \
  (((1U << TIMER_TCVSYN3_LEN) - 1) << TIMER_TCVSYN3_POS)
#define TIMER_TCVSYN3_UMSK \
  (~(((1U << TIMER_TCVSYN3_LEN) - 1) << TIMER_TCVSYN3_POS))

/* 0xBC : TCDR */

#define TIMER_TCDR_OFFSET (0xBC)
#define TIMER_TCDR2       TIMER_TCDR2
#define TIMER_TCDR2_POS   (8U)
#define TIMER_TCDR2_LEN   (8U)
#define TIMER_TCDR2_MSK   (((1U << TIMER_TCDR2_LEN) - 1) << TIMER_TCDR2_POS)
#define TIMER_TCDR2_UMSK  (~(((1U << TIMER_TCDR2_LEN) - 1) << TIMER_TCDR2_POS))
#define TIMER_TCDR3       TIMER_TCDR3
#define TIMER_TCDR3_POS   (16U)
#define TIMER_TCDR3_LEN   (8U)
#define TIMER_TCDR3_MSK   (((1U << TIMER_TCDR3_LEN) - 1) << TIMER_TCDR3_POS)
#define TIMER_TCDR3_UMSK  (~(((1U << TIMER_TCDR3_LEN) - 1) << TIMER_TCDR3_POS))
#define TIMER_WCDR        TIMER_WCDR
#define TIMER_WCDR_POS    (24U)
#define TIMER_WCDR_LEN    (8U)
#define TIMER_WCDR_MSK    (((1U << TIMER_WCDR_LEN) - 1) << TIMER_WCDR_POS)
#define TIMER_WCDR_UMSK   (~(((1U << TIMER_WCDR_LEN) - 1) << TIMER_WCDR_POS))

/* TIMER channel type definition */

#define TIMER_CH0    0 /* TIMER channel 0 port define */
#define TIMER_CH1    1 /* TIMER channel 1 port define */
#define TIMER_CH_MAX 2

/* TIMER clock source type definition */

#define TIMER_CLKSRC_FCLK 0 /* TIMER clock source :System CLK */
#define TIMER_CLKSRC_32K  1 /* TIMER clock source :32K CLK */
#define TIMER_CLKSRC_1K   2 /* TIMER clock source :1K CLK,Only for
                             * Timer not for  Watchdog */

#define TIMER_CLKSRC_XTAL 3  /* TIMER clock source :XTAL CLK */

/* TIMER match compare ID type definition */

#define TIMER_COMP_ID_0 0 /* TIMER match compare ID 0 define */
#define TIMER_COMP_ID_1 1 /* TIMER match compare ID 1 define */
#define TIMER_COMP_ID_2 2 /* TIMER match compare ID 2 define */

/* TIMER preload source type definition */

#define TIMER_PRELOAD_TRIG_NONE 0 /* TIMER no preload source, just free run \
                                   */
#define TIMER_PRELOAD_TRIG_COMP0 \
  1 /* TIMER count register preload triggered by \
     * comparator 0 */

#define TIMER_PRELOAD_TRIG_COMP1 \
  2 /* TIMER count register preload triggered by \
     * comparator 1 */

#define TIMER_PRELOAD_TRIG_COMP2 \
  3 /* TIMER count register preload triggered by \
     * comparator 2 */

/* TIMER count register run mode type definition */

#define TIMER_COUNT_PRELOAD \
  0 /* TIMER count register preload from comparator \
     * register */

#define TIMER_COUNT_FREERUN 1 /* TIMER count register free run */

/* TIMER interrupt type definition */

#define TIMER_INT_COMP_0 0 /* Comparator 0 match cause interrupt */
#define TIMER_INT_COMP_1 1 /* Comparator 1 match cause interrupt */
#define TIMER_INT_COMP_2 2 /* Comparator 2 match cause interrupt */
#define TIMER_INT_ALL    3

/* Watchdog timer interrupt type definition */

#define WDT_INT     0 /* Comparator 0 match cause interrupt */
#define WDT_INT_ALL 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* TIMER configuration structure type definition */

struct timer_cfg_s
{
  uint32_t timer_ch; /* Timer channel */
  uint32_t clk_src;  /* Timer clock source */

  /* Timer count register preload trigger source slelect */

  uint32_t pl_trig_src;

  uint32_t count_mode;     /* Timer count mode */
  uint8_t  clock_division; /* Timer clock divison value */
  uint32_t match_val0;     /* Timer match 0 value 0 */
  uint32_t match_val1;     /* Timer match 1 value 0 */
  uint32_t match_val2;     /* Timer match 2 value 0 */
  uint32_t pre_load_val;   /* Timer preload value */
};
typedef struct timer_cfg_s timer_cfg_t;

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN uint32_t bl602_timer_getcompvalue(uint32_t timer_ch, uint32_t cmp_no);
EXTERN void bl602_timer_setcompvalue(uint32_t timer_ch, uint32_t cmp_no,
                                     uint32_t val);
EXTERN uint32_t bl602_timer_getcountervalue(uint32_t timer_ch);
EXTERN uint32_t bl602_timer_getmatchstatus(uint32_t timer_ch,
                                           uint32_t cmp_no);
EXTERN uint32_t bl602_timer_getpreloadvalue(uint32_t timer_ch);
EXTERN void bl602_timer_setpreloadvalue(uint32_t timer_ch, uint32_t val);
EXTERN void bl602_timer_setpreloadtrigsrc(uint32_t timer_ch,
                                          uint32_t pl_src);
EXTERN void bl602_timer_setcountmode(uint32_t timer_ch, uint32_t count_mode);
EXTERN void bl602_timer_clearintstatus(uint32_t timer_ch, uint32_t cmp_no);
EXTERN void bl602_timer_init(timer_cfg_t *timer_cfg);
EXTERN void bl602_timer_enable(uint32_t timer_ch);
EXTERN void bl602_timer_disable(uint32_t timer_ch);
EXTERN void bl602_timer_intmask(uint32_t timer_ch, uint32_t int_type,
                                uint32_t int_mask);
EXTERN void bl602_wdt_set_clock(uint32_t clk_src, uint8_t div);
EXTERN uint32_t bl602_wdt_getmatchvalue(void);
EXTERN void bl602_wdt_setcompvalue(uint16_t val);
EXTERN uint16_t bl602_wdt_getcountervalue(void);
EXTERN void bl602_wdt_resetcountervalue(void);
EXTERN uint32_t bl602_wdt_getresetstatus(void);
EXTERN void bl602_wdt_clearresetstatus(void);
EXTERN void bl602_wdt_enable(void);
EXTERN void bl602_wdt_disable(void);
EXTERN void bl602_wdt_intmask(uint32_t int_type, uint32_t int_mask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIO_H */
