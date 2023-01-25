/*********************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_osc.h
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
 *********************************************************************************/

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OSC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OSC_H

/*********************************************************************************
 * Included Files
 *********************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/*********************************************************************************
 * Pre-processor Definitions
 *********************************************************************************/

/* ANADIG_OSC Register Offsets ***************************************************/
#define IMXRT_ANADIG_OSC_OSC_48M_CTRL_OFFSET    (0x0010)
#define IMXRT_ANADIG_OSC_OSC_24M_CTRL_OFFSET    (0x0020)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL0_OFFSET  (0x0040)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL1_OFFSET  (0x0050)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL2_OFFSET  (0x0060)
#define IMXRT_ANADIG_OSC_OSC_16M_CTRL_OFFSET    (0x00c0)

/* ANADIG_OSC Register Addresses *************************************************/
#define IMXRT_ANADIG_OSC_OSC_48M_CTRL    (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_48M_CTRL_OFFSET)
#define IMXRT_ANADIG_OSC_OSC_24M_CTRL    (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_24M_CTRL_OFFSET)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL0  (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_400M_CTRL0_OFFSET)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL1  (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_400M_CTRL1_OFFSET)
#define IMXRT_ANADIG_OSC_OSC_400M_CTRL2  (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_400M_CTRL2_OFFSET)
#define IMXRT_ANADIG_OSC_OSC_16M_CTRL    (IMXRT_ANADIG_BASE + IMXRT_ANADIG_OSC_OSC_16M_CTRL_OFFSET)

/* 48MHz RCOSC Control Register (OSC_48M_CTRL) */
#define ANADIG_OSC_OSC_48M_CTRL_TEN                       (1 << 1)   /* Bit 1: 48MHz RCOSC Enable */
#define ANADIG_OSC_OSC_48M_CTRL_RC_48M_DIV2_EN            (1 << 24)  /* Bit 24: RCOSC_48M_DIV2 Enable */
#define ANADIG_OSC_OSC_48M_CTRL_RC_48M_DIV2_CONTROL_MODE  (1 << 30)  /* Bit 30: RCOSC_48M_DIV2 Control Mode */
#define ANADIG_OSC_OSC_48M_CTRL_RC_48M_CONTROL_MODE       (1 << 31)  /* Bit 31: 48MHz RCOSC Control Mode */

/* 24MHz OSC Control Register (OSC_24M_CTRL) */
#define ANADIG_OSC_OSC_24M_CTRL_BYPASS_CLK            (1 << 0)   /* Bit 0: 24MHz OSC Bypass Clock */
#define ANADIG_OSC_OSC_24M_CTRL_BYPASS_EN             (1 << 1)   /* Bit 1: 24MHz OSC Bypass Enable */
#define ANADIG_OSC_OSC_24M_CTRL_LP_EN                 (1 << 2)   /* Bit 2: 24MHz OSC Low-Power Mode Enable */
#define ANADIG_OSC_OSC_24M_CTRL_OSC_COMP_MODE         (1 << 3)   /* Bit 3: 24MHz OSC Comparator Mode */
#define ANADIG_OSC_OSC_24M_CTRL_OSC_EN                (1 << 4)   /* Bit 4: 24MHz OSC Enable */
#define ANADIG_OSC_OSC_24M_CTRL_OSC_24M_GATE          (1 << 7)   /* Bit 7: 24MHz OSC Gate Control */
#define ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE        (1 << 30)  /* Bit 30: 24MHz OSC Stable */
#define ANADIG_OSC_OSC_24M_CTRL_OSC_24M_CONTROL_MODE  (1 << 31)  /* Bit 31: 24MHz OSC Control Mode */

/* 400MHz RCOSC Control0 Register (OSC_400M_CTRL0) */
#define ANADIG_OSC_OSC_400M_CTRL0_OSC400M_AI_BUSY  (1 << 31)  /* Bit 31: 400MHz OSC AI BUSY */

/* 400MHz RCOSC Control1 Register (OSC_400M_CTRL1) */
#define ANADIG_OSC_OSC_400M_CTRL1_PWD                   (1 << 0)   /* Bit 0: Power down control for 400MHz RCOSC */
#define ANADIG_OSC_OSC_400M_CTRL1_CLKGATE_400MEG        (1 << 1)   /* Bit 1: Clock gate control for 400MHz RCOSC */
#define ANADIG_OSC_OSC_400M_CTRL1_RC_400M_CONTROL_MODE  (1 << 31)  /* Bit 31: 400MHz RCOSC Control mode */

/* 400MHz RCOSC Control2 Register (OSC_400M_CTRL2) */
#define ANADIG_OSC_OSC_400M_CTRL2_ENABLE_CLK          (1 << 0)   /* Bit 0: Clock enable */
#define ANADIG_OSC_OSC_400M_CTRL2_TUNE_BYP            (1 << 10)  /* Bit 10: Bypass tuning logic */
#define ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL_SHIFT  (24)       /* Bits 24-32: Oscillator Tune Value */
#define ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL_MASK   (0x8 << ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL_SHIFT)
#define ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL(n)     (((n) << ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL_SHIFT) & ANADIG_OSC_OSC_400M_CTRL2_OSC_TUNE_VAL_MASK)

/* 16MHz RCOSC Control Register (OSC_16M_CTRL) */
#define ANADIG_OSC_OSC_16M_CTRL_EN_IRC4M16M          (1 << 1)   /* Bit 1: Enable Clock Output */
#define ANADIG_OSC_OSC_16M_CTRL_EN_POWER_SAVE        (1 << 3)   /* Bit 3: Power Save Enable */
#define ANADIG_OSC_OSC_16M_CTRL_SOURCE_SEL_16M       (1 << 8)   /* Bit 8: Source select */
#define ANADIG_OSC_OSC_16M_CTRL_RC_16M_CONTROL_MODE  (1 << 31)  /* Bit 31: Control Mode for 16MHz Oscillator */

/* OSC_RC_400M Register Offsets **************************************************/
#define IMXRT_OSC_RC_400M_CTRL0_OFFSET      (0x0000)
#define IMXRT_OSC_RC_400M_CTRL0_SET_OFFSET  (0x0004)
#define IMXRT_OSC_RC_400M_CTRL0_CLR_OFFSET  (0x0008)
#define IMXRT_OSC_RC_400M_CTRL0_TOG_OFFSET  (0x000c)
#define IMXRT_OSC_RC_400M_CTRL1_OFFSET      (0x0010)
#define IMXRT_OSC_RC_400M_CTRL1_SET_OFFSET  (0x0014)
#define IMXRT_OSC_RC_400M_CTRL1_CLR_OFFSET  (0x0018)
#define IMXRT_OSC_RC_400M_CTRL1_TOG_OFFSET  (0x001c)
#define IMXRT_OSC_RC_400M_CTRL2_OFFSET      (0x0020)
#define IMXRT_OSC_RC_400M_CTRL2_SET_OFFSET  (0x0024)
#define IMXRT_OSC_RC_400M_CTRL2_CLR_OFFSET  (0x0028)
#define IMXRT_OSC_RC_400M_CTRL2_TOG_OFFSET  (0x002c)
#define IMXRT_OSC_RC_400M_CTRL3_OFFSET      (0x0030)
#define IMXRT_OSC_RC_400M_CTRL3_SET_OFFSET  (0x0034)
#define IMXRT_OSC_RC_400M_CTRL3_CLR_OFFSET  (0x0038)
#define IMXRT_OSC_RC_400M_CTRL3_TOG_OFFSET  (0x003c)
#define IMXRT_OSC_RC_400M_STAT0_OFFSET      (0x0050)
#define IMXRT_OSC_RC_400M_STAT0_SET_OFFSET  (0x0054)
#define IMXRT_OSC_RC_400M_STAT0_CLR_OFFSET  (0x0058)
#define IMXRT_OSC_RC_400M_STAT0_TOG_OFFSET  (0x005c)
#define IMXRT_OSC_RC_400M_STAT1_OFFSET      (0x0060)
#define IMXRT_OSC_RC_400M_STAT1_SET_OFFSET  (0x0064)
#define IMXRT_OSC_RC_400M_STAT1_CLR_OFFSET  (0x0068)
#define IMXRT_OSC_RC_400M_STAT1_TOG_OFFSET  (0x006c)
#define IMXRT_OSC_RC_400M_STAT2_OFFSET      (0x0070)
#define IMXRT_OSC_RC_400M_STAT2_SET_OFFSET  (0x0074)
#define IMXRT_OSC_RC_400M_STAT2_CLR_OFFSET  (0x0078)
#define IMXRT_OSC_RC_400M_STAT2_TOG_OFFSET  (0x007c)

/* OSC_RC_400M Register Addresses ************************************************/
#define IMXRT_OSC_RC_400M_CTRL0      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL0_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL0_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL0_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL0_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL0_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL0_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL0_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL1_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL1_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL1_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL1_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL1_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL2_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL2_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL2_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL2_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL2_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL3_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL3_SET_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL3_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_CTRL3_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_CTRL3_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_STAT0      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT0_OFFSET)
#define IMXRT_OSC_RC_400M_STAT0_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT0_SET_OFFSET)
#define IMXRT_OSC_RC_400M_STAT0_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT0_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_STAT0_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT0_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_STAT1      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT1_OFFSET)
#define IMXRT_OSC_RC_400M_STAT1_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT1_SET_OFFSET)
#define IMXRT_OSC_RC_400M_STAT1_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT1_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_STAT1_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT1_TOG_OFFSET)
#define IMXRT_OSC_RC_400M_STAT2      (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT2_OFFSET)
#define IMXRT_OSC_RC_400M_STAT2_SET  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT2_SET_OFFSET)
#define IMXRT_OSC_RC_400M_STAT2_CLR  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT2_CLR_OFFSET)
#define IMXRT_OSC_RC_400M_STAT2_TOG  (IMXRT_OSC_RC_400M_BASE + IMXRT_OSC_RC_400M_STAT2_TOG_OFFSET)

/* Control Register 0 (CTRL0) */
#define OSC_RC_400M_CTRL0_REF_CLK_DIV_SHIFT  (24)  /* Bits 24-30: Divide value for ref_clk to generate slow_clk (used inside this IP) */
#define OSC_RC_400M_CTRL0_REF_CLK_DIV_MASK   (0x6 << OSC_RC_400M_CTRL0_REF_CLK_DIV_SHIFT)
#define OSC_RC_400M_CTRL0_REF_CLK_DIV(n)     (((n) << OSC_RC_400M_CTRL0_REF_CLK_DIV_SHIFT) & OSC_RC_400M_CTRL0_REF_CLK_DIV_MASK)

/* Control Register 0 (CTRL0_SET) */
#define OSC_RC_400M_CTRL0_SET_REF_CLK_DIV_SHIFT  (24)  /* Bits 24-30: Divide value for ref_clk to generate slow_clk (used inside this IP) */
#define OSC_RC_400M_CTRL0_SET_REF_CLK_DIV_MASK   (0x6 << OSC_RC_400M_CTRL0_SET_REF_CLK_DIV_SHIFT)
#define OSC_RC_400M_CTRL0_SET_REF_CLK_DIV(n)     (((n) << OSC_RC_400M_CTRL0_SET_REF_CLK_DIV_SHIFT) & OSC_RC_400M_CTRL0_SET_REF_CLK_DIV_MASK)

/* Control Register 0 (CTRL0_CLR) */
#define OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV_SHIFT  (24)  /* Bits 24-30: Divide value for ref_clk to generate slow_clk (used inside this IP) */
#define OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV_MASK   (0x6 << OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV_SHIFT)
#define OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV(n)     (((n) << OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV_SHIFT) & OSC_RC_400M_CTRL0_CLR_REF_CLK_DIV_MASK)

/* Control Register 0 (CTRL0_TOG) */
#define OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV_SHIFT  (24)  /* Bits 24-30: Divide value for ref_clk to generate slow_clk (used inside this IP) */
#define OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV_MASK   (0x6 << OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV_SHIFT)
#define OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV(n)     (((n) << OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV_SHIFT) & OSC_RC_400M_CTRL0_TOG_REF_CLK_DIV_MASK)

/* Control Register 1 (CTRL1) */
#define OSC_RC_400M_CTRL1_HYST_MINUS_SHIFT    (0)   /* Bits 0-4: Negative hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_HYST_MINUS_MASK     (0x4 << OSC_RC_400M_CTRL1_HYST_MINUS_SHIFT)
#define OSC_RC_400M_CTRL1_HYST_MINUS(n)       (((n) << OSC_RC_400M_CTRL1_HYST_MINUS_SHIFT) & OSC_RC_400M_CTRL1_HYST_MINUS_MASK)
#define OSC_RC_400M_CTRL1_HYST_PLUS_SHIFT     (8)   /* Bits 8-12: Positive hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_HYST_PLUS_MASK      (0x4 << OSC_RC_400M_CTRL1_HYST_PLUS_SHIFT)
#define OSC_RC_400M_CTRL1_HYST_PLUS(n)        (((n) << OSC_RC_400M_CTRL1_HYST_PLUS_SHIFT) & OSC_RC_400M_CTRL1_HYST_PLUS_MASK)
#define OSC_RC_400M_CTRL1_TARGET_COUNT_SHIFT  (16)  /* Bits 16-32: Target count for the fast clock */
#define OSC_RC_400M_CTRL1_TARGET_COUNT_MASK   (0x10 << OSC_RC_400M_CTRL1_TARGET_COUNT_SHIFT)
#define OSC_RC_400M_CTRL1_TARGET_COUNT(n)     (((n) << OSC_RC_400M_CTRL1_TARGET_COUNT_SHIFT) & OSC_RC_400M_CTRL1_TARGET_COUNT_MASK)

/* Control Register 1 (CTRL1_SET) */
#define OSC_RC_400M_CTRL1_SET_HYST_MINUS_SHIFT    (0)   /* Bits 0-4: Negative hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_SET_HYST_MINUS_MASK     (0x4 << OSC_RC_400M_CTRL1_SET_HYST_MINUS_SHIFT)
#define OSC_RC_400M_CTRL1_SET_HYST_MINUS(n)       (((n) << OSC_RC_400M_CTRL1_SET_HYST_MINUS_SHIFT) & OSC_RC_400M_CTRL1_SET_HYST_MINUS_MASK)
#define OSC_RC_400M_CTRL1_SET_HYST_PLUS_SHIFT     (8)   /* Bits 8-12: Positive hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_SET_HYST_PLUS_MASK      (0x4 << OSC_RC_400M_CTRL1_SET_HYST_PLUS_SHIFT)
#define OSC_RC_400M_CTRL1_SET_HYST_PLUS(n)        (((n) << OSC_RC_400M_CTRL1_SET_HYST_PLUS_SHIFT) & OSC_RC_400M_CTRL1_SET_HYST_PLUS_MASK)
#define OSC_RC_400M_CTRL1_SET_TARGET_COUNT_SHIFT  (16)  /* Bits 16-32: Target count for the fast clock */
#define OSC_RC_400M_CTRL1_SET_TARGET_COUNT_MASK   (0x10 << OSC_RC_400M_CTRL1_SET_TARGET_COUNT_SHIFT)
#define OSC_RC_400M_CTRL1_SET_TARGET_COUNT(n)     (((n) << OSC_RC_400M_CTRL1_SET_TARGET_COUNT_SHIFT) & OSC_RC_400M_CTRL1_SET_TARGET_COUNT_MASK)

/* Control Register 1 (CTRL1_CLR) */
#define OSC_RC_400M_CTRL1_CLR_HYST_MINUS_SHIFT    (0)   /* Bits 0-4: Negative hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_CLR_HYST_MINUS_MASK     (0x4 << OSC_RC_400M_CTRL1_CLR_HYST_MINUS_SHIFT)
#define OSC_RC_400M_CTRL1_CLR_HYST_MINUS(n)       (((n) << OSC_RC_400M_CTRL1_CLR_HYST_MINUS_SHIFT) & OSC_RC_400M_CTRL1_CLR_HYST_MINUS_MASK)
#define OSC_RC_400M_CTRL1_CLR_HYST_PLUS_SHIFT     (8)   /* Bits 8-12: Positive hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_CLR_HYST_PLUS_MASK      (0x4 << OSC_RC_400M_CTRL1_CLR_HYST_PLUS_SHIFT)
#define OSC_RC_400M_CTRL1_CLR_HYST_PLUS(n)        (((n) << OSC_RC_400M_CTRL1_CLR_HYST_PLUS_SHIFT) & OSC_RC_400M_CTRL1_CLR_HYST_PLUS_MASK)
#define OSC_RC_400M_CTRL1_CLR_TARGET_COUNT_SHIFT  (16)  /* Bits 16-32: Target count for the fast clock */
#define OSC_RC_400M_CTRL1_CLR_TARGET_COUNT_MASK   (0x10 << OSC_RC_400M_CTRL1_CLR_TARGET_COUNT_SHIFT)
#define OSC_RC_400M_CTRL1_CLR_TARGET_COUNT(n)     (((n) << OSC_RC_400M_CTRL1_CLR_TARGET_COUNT_SHIFT) & OSC_RC_400M_CTRL1_CLR_TARGET_COUNT_MASK)

/* Control Register 1 (CTRL1_TOG) */
#define OSC_RC_400M_CTRL1_TOG_HYST_MINUS_SHIFT    (0)   /* Bits 0-4: Negative hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_TOG_HYST_MINUS_MASK     (0x4 << OSC_RC_400M_CTRL1_TOG_HYST_MINUS_SHIFT)
#define OSC_RC_400M_CTRL1_TOG_HYST_MINUS(n)       (((n) << OSC_RC_400M_CTRL1_TOG_HYST_MINUS_SHIFT) & OSC_RC_400M_CTRL1_TOG_HYST_MINUS_MASK)
#define OSC_RC_400M_CTRL1_TOG_HYST_PLUS_SHIFT     (8)   /* Bits 8-12: Positive hysteresis value for the tuned clock */
#define OSC_RC_400M_CTRL1_TOG_HYST_PLUS_MASK      (0x4 << OSC_RC_400M_CTRL1_TOG_HYST_PLUS_SHIFT)
#define OSC_RC_400M_CTRL1_TOG_HYST_PLUS(n)        (((n) << OSC_RC_400M_CTRL1_TOG_HYST_PLUS_SHIFT) & OSC_RC_400M_CTRL1_TOG_HYST_PLUS_MASK)
#define OSC_RC_400M_CTRL1_TOG_TARGET_COUNT_SHIFT  (16)  /* Bits 16-32: Target count for the fast clock */
#define OSC_RC_400M_CTRL1_TOG_TARGET_COUNT_MASK   (0x10 << OSC_RC_400M_CTRL1_TOG_TARGET_COUNT_SHIFT)
#define OSC_RC_400M_CTRL1_TOG_TARGET_COUNT(n)     (((n) << OSC_RC_400M_CTRL1_TOG_TARGET_COUNT_SHIFT) & OSC_RC_400M_CTRL1_TOG_TARGET_COUNT_MASK)

/* Control Register 2 (CTRL2) */
#define OSC_RC_400M_CTRL2_TUNE_BYP            (1 << 10)  /* Bit 10: Bypass the tuning logic */
#define OSC_RC_400M_CTRL2_TUNE_EN             (1 << 12)  /* Bit 12: Freeze/Unfreeze the tuning value */
#define OSC_RC_400M_CTRL2_TUNE_START          (1 << 14)  /* Bit 14: Start/Stop tuning */
#define OSC_RC_400M_CTRL2_OSC_TUNE_VAL_SHIFT  (24)       /* Bits 24-32: Program the oscillator frequency */
#define OSC_RC_400M_CTRL2_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_CTRL2_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_CTRL2_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_CTRL2_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_CTRL2_OSC_TUNE_VAL_MASK)

/* Control Register 2 (CTRL2_SET) */
#define OSC_RC_400M_CTRL2_SET_TUNE_BYP            (1 << 10)  /* Bit 10: Bypass the tuning logic */
#define OSC_RC_400M_CTRL2_SET_TUNE_EN             (1 << 12)  /* Bit 12: Freeze/Unfreeze the tuning value */
#define OSC_RC_400M_CTRL2_SET_TUNE_START          (1 << 14)  /* Bit 14: Start/Stop tuning */
#define OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL_SHIFT  (24)       /* Bits 24-32: Program the oscillator frequency */
#define OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_CTRL2_SET_OSC_TUNE_VAL_MASK)

/* Control Register 2 (CTRL2_CLR) */
#define OSC_RC_400M_CTRL2_CLR_TUNE_BYP            (1 << 10)  /* Bit 10: Bypass the tuning logic */
#define OSC_RC_400M_CTRL2_CLR_TUNE_EN             (1 << 12)  /* Bit 12: Freeze/Unfreeze the tuning value */
#define OSC_RC_400M_CTRL2_CLR_TUNE_START          (1 << 14)  /* Bit 14: Start/Stop tuning */
#define OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL_SHIFT  (24)       /* Bits 24-32: Program the oscillator frequency */
#define OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_CTRL2_CLR_OSC_TUNE_VAL_MASK)

/* Control Register 2 (CTRL2_TOG) */
#define OSC_RC_400M_CTRL2_TOG_TUNE_BYP            (1 << 10)  /* Bit 10: Bypass the tuning logic */
#define OSC_RC_400M_CTRL2_TOG_TUNE_EN             (1 << 12)  /* Bit 12: Freeze/Unfreeze the tuning value */
#define OSC_RC_400M_CTRL2_TOG_TUNE_START          (1 << 14)  /* Bit 14: Start/Stop tuning */
#define OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL_SHIFT  (24)       /* Bits 24-32: Program the oscillator frequency */
#define OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_CTRL2_TOG_OSC_TUNE_VAL_MASK)

/* Control Register 3 (CTRL3) */
#define OSC_RC_400M_CTRL3_CLR_ERR             (1 << 0)   /* Bit 0: Clear the error flag CLK1M_ERR */
#define OSC_RC_400M_CTRL3_EN_1M_CLK           (1 << 8)   /* Bit 8: Enable 1MHz output Clock */
#define OSC_RC_400M_CTRL3_MUX_1M_CLK          (1 << 10)  /* Bit 10: Select free/locked 1MHz output */
#define OSC_RC_400M_CTRL3_COUNT_1M_CLK_SHIFT  (16)       /* Bits 16-32: Count for the locked clk_1m_out */
#define OSC_RC_400M_CTRL3_COUNT_1M_CLK_MASK   (0x10 << OSC_RC_400M_CTRL3_COUNT_1M_CLK_SHIFT)
#define OSC_RC_400M_CTRL3_COUNT_1M_CLK(n)     (((n) << OSC_RC_400M_CTRL3_COUNT_1M_CLK_SHIFT) & OSC_RC_400M_CTRL3_COUNT_1M_CLK_MASK)

/* Control Register 3 (CTRL3_SET) */
#define OSC_RC_400M_CTRL3_SET_CLR_ERR             (1 << 0)   /* Bit 0: Clear the error flag CLK1M_ERR */
#define OSC_RC_400M_CTRL3_SET_EN_1M_CLK           (1 << 8)   /* Bit 8: Enable 1MHz output Clock */
#define OSC_RC_400M_CTRL3_SET_MUX_1M_CLK          (1 << 10)  /* Bit 10: Select free/locked 1MHz output */
#define OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK_SHIFT  (16)       /* Bits 16-32: Count for the locked clk_1m_out */
#define OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK_MASK   (0x10 << OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK_SHIFT)
#define OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK(n)     (((n) << OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK_SHIFT) & OSC_RC_400M_CTRL3_SET_COUNT_1M_CLK_MASK)

/* Control Register 3 (CTRL3_CLR) */
#define OSC_RC_400M_CTRL3_CLR_CLR_ERR             (1 << 0)   /* Bit 0: Clear the error flag CLK1M_ERR */
#define OSC_RC_400M_CTRL3_CLR_EN_1M_CLK           (1 << 8)   /* Bit 8: Enable 1MHz output Clock */
#define OSC_RC_400M_CTRL3_CLR_MUX_1M_CLK          (1 << 10)  /* Bit 10: Select free/locked 1MHz output */
#define OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK_SHIFT  (16)       /* Bits 16-32: Count for the locked clk_1m_out */
#define OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK_MASK   (0x10 << OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK_SHIFT)
#define OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK(n)     (((n) << OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK_SHIFT) & OSC_RC_400M_CTRL3_CLR_COUNT_1M_CLK_MASK)

/* Control Register 3 (CTRL3_TOG) */
#define OSC_RC_400M_CTRL3_TOG_CLR_ERR             (1 << 0)   /* Bit 0: Clear the error flag CLK1M_ERR */
#define OSC_RC_400M_CTRL3_TOG_EN_1M_CLK           (1 << 8)   /* Bit 8: Enable 1MHz output Clock */
#define OSC_RC_400M_CTRL3_TOG_MUX_1M_CLK          (1 << 10)  /* Bit 10: Select free/locked 1MHz output */
#define OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK_SHIFT  (16)       /* Bits 16-32: Count for the locked clk_1m_out */
#define OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK_MASK   (0x10 << OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK_SHIFT)
#define OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK(n)     (((n) << OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK_SHIFT) & OSC_RC_400M_CTRL3_TOG_COUNT_1M_CLK_MASK)

/* Status Register 0 (STAT0) */
#define OSC_RC_400M_STAT0_CLK1M_ERR  (1 << 0)  /* Bit 0: Error flag for clk_1m_locked */

/* Status Register 0 (STAT0_SET) */
#define OSC_RC_400M_STAT0_SET_CLK1M_ERR  (1 << 0)  /* Bit 0: Error flag for clk_1m_locked */

/* Status Register 0 (STAT0_CLR) */
#define OSC_RC_400M_STAT0_CLR_CLK1M_ERR  (1 << 0)  /* Bit 0: Error flag for clk_1m_locked */

/* Status Register 0 (STAT0_TOG) */
#define OSC_RC_400M_STAT0_TOG_CLK1M_ERR  (1 << 0)  /* Bit 0: Error flag for clk_1m_locked */

/* Status Register 1 (STAT1) */
#define OSC_RC_400M_STAT1_CURR_COUNT_VAL_SHIFT  (16)  /* Bits 16-32: Current count for the fast clock */
#define OSC_RC_400M_STAT1_CURR_COUNT_VAL_MASK   (0x10 << OSC_RC_400M_STAT1_CURR_COUNT_VAL_SHIFT)
#define OSC_RC_400M_STAT1_CURR_COUNT_VAL(n)     (((n) << OSC_RC_400M_STAT1_CURR_COUNT_VAL_SHIFT) & OSC_RC_400M_STAT1_CURR_COUNT_VAL_MASK)

/* Status Register 1 (STAT1_SET) */
#define OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL_SHIFT  (16)  /* Bits 16-32: Current count for the fast clock */
#define OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL_MASK   (0x10 << OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL_SHIFT)
#define OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL(n)     (((n) << OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL_SHIFT) & OSC_RC_400M_STAT1_SET_CURR_COUNT_VAL_MASK)

/* Status Register 1 (STAT1_CLR) */
#define OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL_SHIFT  (16)  /* Bits 16-32: Current count for the fast clock */
#define OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL_MASK   (0x10 << OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL_SHIFT)
#define OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL(n)     (((n) << OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL_SHIFT) & OSC_RC_400M_STAT1_CLR_CURR_COUNT_VAL_MASK)

/* Status Register 1 (STAT1_TOG) */
#define OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL_SHIFT  (16)  /* Bits 16-32: Current count for the fast clock */
#define OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL_MASK   (0x10 << OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL_SHIFT)
#define OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL(n)     (((n) << OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL_SHIFT) & OSC_RC_400M_STAT1_TOG_CURR_COUNT_VAL_MASK)

/* Status Register 2 (STAT2) */
#define OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL_SHIFT  (24)  /* Bits 24-32: Current tuning value used by oscillator */
#define OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_STAT2_CURR_OSC_TUNE_VAL_MASK)

/* Status Register 2 (STAT2_SET) */
#define OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL_SHIFT  (24)  /* Bits 24-32: Current tuning value used by oscillator */
#define OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_STAT2_SET_CURR_OSC_TUNE_VAL_MASK)

/* Status Register 2 (STAT2_CLR) */
#define OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL_SHIFT  (24)  /* Bits 24-32: Current tuning value used by oscillator */
#define OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_STAT2_CLR_CURR_OSC_TUNE_VAL_MASK)

/* Status Register 2 (STAT2_TOG) */
#define OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL_SHIFT  (24)  /* Bits 24-32: Current tuning value used by oscillator */
#define OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL_MASK   (0x8 << OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL_SHIFT)
#define OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL(n)     (((n) << OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL_SHIFT) & OSC_RC_400M_STAT2_TOG_CURR_OSC_TUNE_VAL_MASK)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_OSC_H */
