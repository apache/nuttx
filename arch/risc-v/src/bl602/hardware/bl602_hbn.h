/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_hbn.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/bl602_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 0x0 : HBN_CTL */

#define HBN_CTL_OFFSET    (0x0)
#define HBN_RTC_CTL       HBN_RTC_CTL
#define HBN_RTC_CTL_POS   (0)
#define HBN_RTC_CTL_LEN   (7)
#define HBN_RTC_CTL_MSK   (((1 << HBN_RTC_CTL_LEN) - 1) << HBN_RTC_CTL_POS)
#define HBN_RTC_CTL_UMSK  (~(((1 << HBN_RTC_CTL_LEN) - 1) << HBN_RTC_CTL_POS))
#define HBN_MODE          HBN_MODE
#define HBN_MODE_POS      (7)
#define HBN_MODE_LEN      (1)
#define HBN_MODE_MSK      (((1 << HBN_MODE_LEN) - 1) << HBN_MODE_POS)
#define HBN_MODE_UMSK     (~(((1 << HBN_MODE_LEN) - 1) << HBN_MODE_POS))
#define HBN_TRAP_MODE     HBN_TRAP_MODE
#define HBN_TRAP_MODE_POS (8)
#define HBN_TRAP_MODE_LEN (1)
#define HBN_TRAP_MODE_MSK \
  (((1 << HBN_TRAP_MODE_LEN) - 1) << HBN_TRAP_MODE_POS)
#define HBN_TRAP_MODE_UMSK \
  (~(((1 << HBN_TRAP_MODE_LEN) - 1) << HBN_TRAP_MODE_POS))
#define HBN_PWRDN_HBN_CORE     HBN_PWRDN_HBN_CORE
#define HBN_PWRDN_HBN_CORE_POS (9)
#define HBN_PWRDN_HBN_CORE_LEN (1)
#define HBN_PWRDN_HBN_CORE_MSK \
  (((1 << HBN_PWRDN_HBN_CORE_LEN) - 1) << HBN_PWRDN_HBN_CORE_POS)
#define HBN_PWRDN_HBN_CORE_UMSK \
  (~(((1 << HBN_PWRDN_HBN_CORE_LEN) - 1) << HBN_PWRDN_HBN_CORE_POS))
#define HBN_PWRDN_HBN_RTC     HBN_PWRDN_HBN_RTC
#define HBN_PWRDN_HBN_RTC_POS (11)
#define HBN_PWRDN_HBN_RTC_LEN (1)
#define HBN_PWRDN_HBN_RTC_MSK \
  (((1 << HBN_PWRDN_HBN_RTC_LEN) - 1) << HBN_PWRDN_HBN_RTC_POS)
#define HBN_PWRDN_HBN_RTC_UMSK \
  (~(((1 << HBN_PWRDN_HBN_RTC_LEN) - 1) << HBN_PWRDN_HBN_RTC_POS))
#define HBN_SW_RST                HBN_SW_RST
#define HBN_SW_RST_POS            (12)
#define HBN_SW_RST_LEN            (1)
#define HBN_SW_RST_MSK            (((1 << HBN_SW_RST_LEN) - 1) << HBN_SW_RST_POS)
#define HBN_SW_RST_UMSK           (~(((1 << HBN_SW_RST_LEN) - 1) << HBN_SW_RST_POS))
#define HBN_DIS_PWR_OFF_LDO11     HBN_DIS_PWR_OFF_LDO11
#define HBN_DIS_PWR_OFF_LDO11_POS (13)
#define HBN_DIS_PWR_OFF_LDO11_LEN (1)
#define HBN_DIS_PWR_OFF_LDO11_MSK \
  (((1 << HBN_DIS_PWR_OFF_LDO11_LEN) - 1) << HBN_DIS_PWR_OFF_LDO11_POS)
#define HBN_DIS_PWR_OFF_LDO11_UMSK \
  (~(((1 << HBN_DIS_PWR_OFF_LDO11_LEN) - 1) << HBN_DIS_PWR_OFF_LDO11_POS))
#define HBN_DIS_PWR_OFF_LDO11_RT     HBN_DIS_PWR_OFF_LDO11_RT
#define HBN_DIS_PWR_OFF_LDO11_RT_POS (14)
#define HBN_DIS_PWR_OFF_LDO11_RT_LEN (1)
#define HBN_DIS_PWR_OFF_LDO11_RT_MSK \
  (((1 << HBN_DIS_PWR_OFF_LDO11_RT_LEN) - 1) << HBN_DIS_PWR_OFF_LDO11_RT_POS)
#define HBN_DIS_PWR_OFF_LDO11_RT_UMSK \
  (~(((1 << HBN_DIS_PWR_OFF_LDO11_RT_LEN) - 1) \
     << HBN_DIS_PWR_OFF_LDO11_RT_POS))
#define HBN_LDO11_RT_VOUT_SEL     HBN_LDO11_RT_VOUT_SEL
#define HBN_LDO11_RT_VOUT_SEL_POS (15)
#define HBN_LDO11_RT_VOUT_SEL_LEN (4)
#define HBN_LDO11_RT_VOUT_SEL_MSK \
  (((1 << HBN_LDO11_RT_VOUT_SEL_LEN) - 1) << HBN_LDO11_RT_VOUT_SEL_POS)
#define HBN_LDO11_RT_VOUT_SEL_UMSK \
  (~(((1 << HBN_LDO11_RT_VOUT_SEL_LEN) - 1) << HBN_LDO11_RT_VOUT_SEL_POS))
#define HBN_LDO11_AON_VOUT_SEL     HBN_LDO11_AON_VOUT_SEL
#define HBN_LDO11_AON_VOUT_SEL_POS (19)
#define HBN_LDO11_AON_VOUT_SEL_LEN (4)
#define HBN_LDO11_AON_VOUT_SEL_MSK \
  (((1 << HBN_LDO11_AON_VOUT_SEL_LEN) - 1) << HBN_LDO11_AON_VOUT_SEL_POS)
#define HBN_LDO11_AON_VOUT_SEL_UMSK \
  (~(((1 << HBN_LDO11_AON_VOUT_SEL_LEN) - 1) << HBN_LDO11_AON_VOUT_SEL_POS))
#define HBN_PU_DCDC18_AON     HBN_PU_DCDC18_AON
#define HBN_PU_DCDC18_AON_POS (23)
#define HBN_PU_DCDC18_AON_LEN (1)
#define HBN_PU_DCDC18_AON_MSK \
  (((1 << HBN_PU_DCDC18_AON_LEN) - 1) << HBN_PU_DCDC18_AON_POS)
#define HBN_PU_DCDC18_AON_UMSK \
  (~(((1 << HBN_PU_DCDC18_AON_LEN) - 1) << HBN_PU_DCDC18_AON_POS))
#define HBN_RTC_DLY_OPTION     HBN_RTC_DLY_OPTION
#define HBN_RTC_DLY_OPTION_POS (24)
#define HBN_RTC_DLY_OPTION_LEN (1)
#define HBN_RTC_DLY_OPTION_MSK \
  (((1 << HBN_RTC_DLY_OPTION_LEN) - 1) << HBN_RTC_DLY_OPTION_POS)
#define HBN_RTC_DLY_OPTION_UMSK \
  (~(((1 << HBN_RTC_DLY_OPTION_LEN) - 1) << HBN_RTC_DLY_OPTION_POS))
#define HBN_PWR_ON_OPTION     HBN_PWR_ON_OPTION
#define HBN_PWR_ON_OPTION_POS (25)
#define HBN_PWR_ON_OPTION_LEN (1)
#define HBN_PWR_ON_OPTION_MSK \
  (((1 << HBN_PWR_ON_OPTION_LEN) - 1) << HBN_PWR_ON_OPTION_POS)
#define HBN_PWR_ON_OPTION_UMSK \
  (~(((1 << HBN_PWR_ON_OPTION_LEN) - 1) << HBN_PWR_ON_OPTION_POS))
#define HBN_SRAM_SLP_OPTION     HBN_SRAM_SLP_OPTION
#define HBN_SRAM_SLP_OPTION_POS (26)
#define HBN_SRAM_SLP_OPTION_LEN (1)
#define HBN_SRAM_SLP_OPTION_MSK \
  (((1 << HBN_SRAM_SLP_OPTION_LEN) - 1) << HBN_SRAM_SLP_OPTION_POS)
#define HBN_SRAM_SLP_OPTION_UMSK \
  (~(((1 << HBN_SRAM_SLP_OPTION_LEN) - 1) << HBN_SRAM_SLP_OPTION_POS))
#define HBN_SRAM_SLP     HBN_SRAM_SLP
#define HBN_SRAM_SLP_POS (27)
#define HBN_SRAM_SLP_LEN (1)
#define HBN_SRAM_SLP_MSK (((1 << HBN_SRAM_SLP_LEN) - 1) << HBN_SRAM_SLP_POS)
#define HBN_SRAM_SLP_UMSK \
  (~(((1 << HBN_SRAM_SLP_LEN) - 1) << HBN_SRAM_SLP_POS))
#define HBN_STATE      HBN_STATE
#define HBN_STATE_POS  (28)
#define HBN_STATE_LEN  (4)
#define HBN_STATE_MSK  (((1 << HBN_STATE_LEN) - 1) << HBN_STATE_POS)
#define HBN_STATE_UMSK (~(((1 << HBN_STATE_LEN) - 1) << HBN_STATE_POS))

/* 0x4 : HBN_TIME_L */

#define HBN_TIME_L_OFFSET (0x4)
#define HBN_TIME_L        HBN_TIME_L
#define HBN_TIME_L_POS    (0)
#define HBN_TIME_L_LEN    (32)
#define HBN_TIME_L_MSK    (((1 << HBN_TIME_L_LEN) - 1) << HBN_TIME_L_POS)
#define HBN_TIME_L_UMSK   (~(((1 << HBN_TIME_L_LEN) - 1) << HBN_TIME_L_POS))

/* 0x8 : HBN_TIME_H */

#define HBN_TIME_H_OFFSET (0x8)
#define HBN_TIME_H        HBN_TIME_H
#define HBN_TIME_H_POS    (0)
#define HBN_TIME_H_LEN    (8)
#define HBN_TIME_H_MSK    (((1 << HBN_TIME_H_LEN) - 1) << HBN_TIME_H_POS)
#define HBN_TIME_H_UMSK   (~(((1 << HBN_TIME_H_LEN) - 1) << HBN_TIME_H_POS))

/* 0xC : RTC_TIME_L */

#define HBN_RTC_TIME_L_OFFSET    (0xC)
#define HBN_RTC_TIME_LATCH_L     HBN_RTC_TIME_LATCH_L
#define HBN_RTC_TIME_LATCH_L_POS (0)
#define HBN_RTC_TIME_LATCH_L_LEN (32)
#define HBN_RTC_TIME_LATCH_L_MSK \
  (((1 << HBN_RTC_TIME_LATCH_L_LEN) - 1) << HBN_RTC_TIME_LATCH_L_POS)
#define HBN_RTC_TIME_LATCH_L_UMSK \
  (~(((1 << HBN_RTC_TIME_LATCH_L_LEN) - 1) << HBN_RTC_TIME_LATCH_L_POS))

/* 0x10 : RTC_TIME_H */

#define HBN_RTC_TIME_H_OFFSET    (0x10)
#define HBN_RTC_TIME_LATCH_H     HBN_RTC_TIME_LATCH_H
#define HBN_RTC_TIME_LATCH_H_POS (0)
#define HBN_RTC_TIME_LATCH_H_LEN (8)
#define HBN_RTC_TIME_LATCH_H_MSK \
  (((1 << HBN_RTC_TIME_LATCH_H_LEN) - 1) << HBN_RTC_TIME_LATCH_H_POS)
#define HBN_RTC_TIME_LATCH_H_UMSK \
  (~(((1 << HBN_RTC_TIME_LATCH_H_LEN) - 1) << HBN_RTC_TIME_LATCH_H_POS))
#define HBN_RTC_TIME_LATCH     HBN_RTC_TIME_LATCH
#define HBN_RTC_TIME_LATCH_POS (31)
#define HBN_RTC_TIME_LATCH_LEN (1)
#define HBN_RTC_TIME_LATCH_MSK \
  (((1 << HBN_RTC_TIME_LATCH_LEN) - 1) << HBN_RTC_TIME_LATCH_POS)
#define HBN_RTC_TIME_LATCH_UMSK \
  (~(((1 << HBN_RTC_TIME_LATCH_LEN) - 1) << HBN_RTC_TIME_LATCH_POS))

/* 0x14 : HBN_IRQ_MODE */

#define HBN_IRQ_MODE_OFFSET     (0x14)
#define HBN_PIN_WAKEUP_MODE     HBN_PIN_WAKEUP_MODE
#define HBN_PIN_WAKEUP_MODE_POS (0)
#define HBN_PIN_WAKEUP_MODE_LEN (3)
#define HBN_PIN_WAKEUP_MODE_MSK \
  (((1 << HBN_PIN_WAKEUP_MODE_LEN) - 1) << HBN_PIN_WAKEUP_MODE_POS)
#define HBN_PIN_WAKEUP_MODE_UMSK \
  (~(((1 << HBN_PIN_WAKEUP_MODE_LEN) - 1) << HBN_PIN_WAKEUP_MODE_POS))
#define HBN_PIN_WAKEUP_MASK     HBN_PIN_WAKEUP_MASK
#define HBN_PIN_WAKEUP_MASK_POS (3)
#define HBN_PIN_WAKEUP_MASK_LEN (2)
#define HBN_PIN_WAKEUP_MASK_MSK \
  (((1 << HBN_PIN_WAKEUP_MASK_LEN) - 1) << HBN_PIN_WAKEUP_MASK_POS)
#define HBN_PIN_WAKEUP_MASK_UMSK \
  (~(((1 << HBN_PIN_WAKEUP_MASK_LEN) - 1) << HBN_PIN_WAKEUP_MASK_POS))
#define HBN_REG_AON_PAD_IE_SMT     HBN_REG_AON_PAD_IE_SMT
#define HBN_REG_AON_PAD_IE_SMT_POS (8)
#define HBN_REG_AON_PAD_IE_SMT_LEN (1)
#define HBN_REG_AON_PAD_IE_SMT_MSK \
  (((1 << HBN_REG_AON_PAD_IE_SMT_LEN) - 1) << HBN_REG_AON_PAD_IE_SMT_POS)
#define HBN_REG_AON_PAD_IE_SMT_UMSK \
  (~(((1 << HBN_REG_AON_PAD_IE_SMT_LEN) - 1) << HBN_REG_AON_PAD_IE_SMT_POS))
#define HBN_REG_EN_HW_PU_PD     HBN_REG_EN_HW_PU_PD
#define HBN_REG_EN_HW_PU_PD_POS (16)
#define HBN_REG_EN_HW_PU_PD_LEN (1)
#define HBN_REG_EN_HW_PU_PD_MSK \
  (((1 << HBN_REG_EN_HW_PU_PD_LEN) - 1) << HBN_REG_EN_HW_PU_PD_POS)
#define HBN_REG_EN_HW_PU_PD_UMSK \
  (~(((1 << HBN_REG_EN_HW_PU_PD_LEN) - 1) << HBN_REG_EN_HW_PU_PD_POS))
#define HBN_IRQ_BOR_EN     HBN_IRQ_BOR_EN
#define HBN_IRQ_BOR_EN_POS (18)
#define HBN_IRQ_BOR_EN_LEN (1)
#define HBN_IRQ_BOR_EN_MSK \
  (((1 << HBN_IRQ_BOR_EN_LEN) - 1) << HBN_IRQ_BOR_EN_POS)
#define HBN_IRQ_BOR_EN_UMSK \
  (~(((1 << HBN_IRQ_BOR_EN_LEN) - 1) << HBN_IRQ_BOR_EN_POS))
#define HBN_IRQ_ACOMP0_EN     HBN_IRQ_ACOMP0_EN
#define HBN_IRQ_ACOMP0_EN_POS (20)
#define HBN_IRQ_ACOMP0_EN_LEN (2)
#define HBN_IRQ_ACOMP0_EN_MSK \
  (((1 << HBN_IRQ_ACOMP0_EN_LEN) - 1) << HBN_IRQ_ACOMP0_EN_POS)
#define HBN_IRQ_ACOMP0_EN_UMSK \
  (~(((1 << HBN_IRQ_ACOMP0_EN_LEN) - 1) << HBN_IRQ_ACOMP0_EN_POS))
#define HBN_IRQ_ACOMP1_EN     HBN_IRQ_ACOMP1_EN
#define HBN_IRQ_ACOMP1_EN_POS (22)
#define HBN_IRQ_ACOMP1_EN_LEN (2)
#define HBN_IRQ_ACOMP1_EN_MSK \
  (((1 << HBN_IRQ_ACOMP1_EN_LEN) - 1) << HBN_IRQ_ACOMP1_EN_POS)
#define HBN_IRQ_ACOMP1_EN_UMSK \
  (~(((1 << HBN_IRQ_ACOMP1_EN_LEN) - 1) << HBN_IRQ_ACOMP1_EN_POS))
#define HBN_PIN_WAKEUP_SEL     HBN_PIN_WAKEUP_SEL
#define HBN_PIN_WAKEUP_SEL_POS (24)
#define HBN_PIN_WAKEUP_SEL_LEN (3)
#define HBN_PIN_WAKEUP_SEL_MSK \
  (((1 << HBN_PIN_WAKEUP_SEL_LEN) - 1) << HBN_PIN_WAKEUP_SEL_POS)
#define HBN_PIN_WAKEUP_SEL_UMSK \
  (~(((1 << HBN_PIN_WAKEUP_SEL_LEN) - 1) << HBN_PIN_WAKEUP_SEL_POS))
#define HBN_PIN_WAKEUP_EN     HBN_PIN_WAKEUP_EN
#define HBN_PIN_WAKEUP_EN_POS (27)
#define HBN_PIN_WAKEUP_EN_LEN (1)
#define HBN_PIN_WAKEUP_EN_MSK \
  (((1 << HBN_PIN_WAKEUP_EN_LEN) - 1) << HBN_PIN_WAKEUP_EN_POS)
#define HBN_PIN_WAKEUP_EN_UMSK \
  (~(((1 << HBN_PIN_WAKEUP_EN_LEN) - 1) << HBN_PIN_WAKEUP_EN_POS))

/* 0x18 : HBN_IRQ_STAT */

#define HBN_IRQ_STAT_OFFSET (0x18)
#define HBN_IRQ_STAT        HBN_IRQ_STAT
#define HBN_IRQ_STAT_POS    (0)
#define HBN_IRQ_STAT_LEN    (32)
#define HBN_IRQ_STAT_MSK    (((1 << HBN_IRQ_STAT_LEN) - 1) << HBN_IRQ_STAT_POS)
#define HBN_IRQ_STAT_UMSK \
  (~(((1 << HBN_IRQ_STAT_LEN) - 1) << HBN_IRQ_STAT_POS))

/* 0x1C : HBN_IRQ_CLR */

#define HBN_IRQ_CLR_OFFSET (0x1C)
#define HBN_IRQ_CLR        HBN_IRQ_CLR
#define HBN_IRQ_CLR_POS    (0)
#define HBN_IRQ_CLR_LEN    (32)
#define HBN_IRQ_CLR_MSK    (((1 << HBN_IRQ_CLR_LEN) - 1) << HBN_IRQ_CLR_POS)
#define HBN_IRQ_CLR_UMSK   (~(((1 << HBN_IRQ_CLR_LEN) - 1) << HBN_IRQ_CLR_POS))

/* 0x20 : HBN_PIR_CFG */

#define HBN_PIR_CFG_OFFSET  (0x20)
#define HBN_PIR_HPF_SEL     HBN_PIR_HPF_SEL
#define HBN_PIR_HPF_SEL_POS (0)
#define HBN_PIR_HPF_SEL_LEN (2)
#define HBN_PIR_HPF_SEL_MSK \
  (((1 << HBN_PIR_HPF_SEL_LEN) - 1) << HBN_PIR_HPF_SEL_POS)
#define HBN_PIR_HPF_SEL_UMSK \
  (~(((1 << HBN_PIR_HPF_SEL_LEN) - 1) << HBN_PIR_HPF_SEL_POS))
#define HBN_PIR_LPF_SEL     HBN_PIR_LPF_SEL
#define HBN_PIR_LPF_SEL_POS (2)
#define HBN_PIR_LPF_SEL_LEN (1)
#define HBN_PIR_LPF_SEL_MSK \
  (((1 << HBN_PIR_LPF_SEL_LEN) - 1) << HBN_PIR_LPF_SEL_POS)
#define HBN_PIR_LPF_SEL_UMSK \
  (~(((1 << HBN_PIR_LPF_SEL_LEN) - 1) << HBN_PIR_LPF_SEL_POS))
#define HBN_PIR_DIS        HBN_PIR_DIS
#define HBN_PIR_DIS_POS    (4)
#define HBN_PIR_DIS_LEN    (2)
#define HBN_PIR_DIS_MSK    (((1 << HBN_PIR_DIS_LEN) - 1) << HBN_PIR_DIS_POS)
#define HBN_PIR_DIS_UMSK   (~(((1 << HBN_PIR_DIS_LEN) - 1) << HBN_PIR_DIS_POS))
#define HBN_PIR_EN         HBN_PIR_EN
#define HBN_PIR_EN_POS     (7)
#define HBN_PIR_EN_LEN     (1)
#define HBN_PIR_EN_MSK     (((1 << HBN_PIR_EN_LEN) - 1) << HBN_PIR_EN_POS)
#define HBN_PIR_EN_UMSK    (~(((1 << HBN_PIR_EN_LEN) - 1) << HBN_PIR_EN_POS))
#define HBN_GPADC_CGEN     HBN_GPADC_CGEN
#define HBN_GPADC_CGEN_POS (8)
#define HBN_GPADC_CGEN_LEN (1)
#define HBN_GPADC_CGEN_MSK \
  (((1 << HBN_GPADC_CGEN_LEN) - 1) << HBN_GPADC_CGEN_POS)
#define HBN_GPADC_CGEN_UMSK \
  (~(((1 << HBN_GPADC_CGEN_LEN) - 1) << HBN_GPADC_CGEN_POS))
#define HBN_GPADC_NOSYNC     HBN_GPADC_NOSYNC
#define HBN_GPADC_NOSYNC_POS (9)
#define HBN_GPADC_NOSYNC_LEN (1)
#define HBN_GPADC_NOSYNC_MSK \
  (((1 << HBN_GPADC_NOSYNC_LEN) - 1) << HBN_GPADC_NOSYNC_POS)
#define HBN_GPADC_NOSYNC_UMSK \
  (~(((1 << HBN_GPADC_NOSYNC_LEN) - 1) << HBN_GPADC_NOSYNC_POS))

/* 0x24 : HBN_PIR_VTH */

#define HBN_PIR_VTH_OFFSET (0x24)
#define HBN_PIR_VTH        HBN_PIR_VTH
#define HBN_PIR_VTH_POS    (0)
#define HBN_PIR_VTH_LEN    (14)
#define HBN_PIR_VTH_MSK    (((1 << HBN_PIR_VTH_LEN) - 1) << HBN_PIR_VTH_POS)
#define HBN_PIR_VTH_UMSK   (~(((1 << HBN_PIR_VTH_LEN) - 1) << HBN_PIR_VTH_POS))

/* 0x28 : HBN_PIR_INTERVAL */

#define HBN_PIR_INTERVAL_OFFSET (0x28)
#define HBN_PIR_INTERVAL        HBN_PIR_INTERVAL
#define HBN_PIR_INTERVAL_POS    (0)
#define HBN_PIR_INTERVAL_LEN    (12)
#define HBN_PIR_INTERVAL_MSK \
  (((1 << HBN_PIR_INTERVAL_LEN) - 1) << HBN_PIR_INTERVAL_POS)
#define HBN_PIR_INTERVAL_UMSK \
  (~(((1 << HBN_PIR_INTERVAL_LEN) - 1) << HBN_PIR_INTERVAL_POS))

/* 0x2C : HBN_BOR_CFG */

#define HBN_BOR_CFG_OFFSET (0x2C)
#define HBN_BOR_SEL        HBN_BOR_SEL
#define HBN_BOR_SEL_POS    (0)
#define HBN_BOR_SEL_LEN    (1)
#define HBN_BOR_SEL_MSK    (((1 << HBN_BOR_SEL_LEN) - 1) << HBN_BOR_SEL_POS)
#define HBN_BOR_SEL_UMSK   (~(((1 << HBN_BOR_SEL_LEN) - 1) << HBN_BOR_SEL_POS))
#define HBN_BOR_VTH        HBN_BOR_VTH
#define HBN_BOR_VTH_POS    (1)
#define HBN_BOR_VTH_LEN    (1)
#define HBN_BOR_VTH_MSK    (((1 << HBN_BOR_VTH_LEN) - 1) << HBN_BOR_VTH_POS)
#define HBN_BOR_VTH_UMSK   (~(((1 << HBN_BOR_VTH_LEN) - 1) << HBN_BOR_VTH_POS))
#define HBN_PU_BOR         HBN_PU_BOR
#define HBN_PU_BOR_POS     (2)
#define HBN_PU_BOR_LEN     (1)
#define HBN_PU_BOR_MSK     (((1 << HBN_PU_BOR_LEN) - 1) << HBN_PU_BOR_POS)
#define HBN_PU_BOR_UMSK    (~(((1 << HBN_PU_BOR_LEN) - 1) << HBN_PU_BOR_POS))
#define HBN_R_BOR_OUT      HBN_R_BOR_OUT
#define HBN_R_BOR_OUT_POS  (3)
#define HBN_R_BOR_OUT_LEN  (1)
#define HBN_R_BOR_OUT_MSK \
  (((1 << HBN_R_BOR_OUT_LEN) - 1) << HBN_R_BOR_OUT_POS)
#define HBN_R_BOR_OUT_UMSK \
  (~(((1 << HBN_R_BOR_OUT_LEN) - 1) << HBN_R_BOR_OUT_POS))

/* 0x30 : HBN_GLB */

#define HBN_GLB_OFFSET       (0x30)
#define HBN_ROOT_CLK_SEL     HBN_ROOT_CLK_SEL
#define HBN_ROOT_CLK_SEL_POS (0)
#define HBN_ROOT_CLK_SEL_LEN (2)
#define HBN_ROOT_CLK_SEL_MSK \
  (((1 << HBN_ROOT_CLK_SEL_LEN) - 1) << HBN_ROOT_CLK_SEL_POS)
#define HBN_ROOT_CLK_SEL_UMSK \
  (~(((1 << HBN_ROOT_CLK_SEL_LEN) - 1) << HBN_ROOT_CLK_SEL_POS))
#define HBN_UART_CLK_SEL     HBN_UART_CLK_SEL
#define HBN_UART_CLK_SEL_POS (2)
#define HBN_UART_CLK_SEL_LEN (1)
#define HBN_UART_CLK_SEL_MSK \
  (((1 << HBN_UART_CLK_SEL_LEN) - 1) << HBN_UART_CLK_SEL_POS)
#define HBN_UART_CLK_SEL_UMSK \
  (~(((1 << HBN_UART_CLK_SEL_LEN) - 1) << HBN_UART_CLK_SEL_POS))
#define HBN_F32K_SEL     HBN_F32K_SEL
#define HBN_F32K_SEL_POS (3)
#define HBN_F32K_SEL_LEN (2)
#define HBN_F32K_SEL_MSK (((1 << HBN_F32K_SEL_LEN) - 1) << HBN_F32K_SEL_POS)
#define HBN_F32K_SEL_UMSK \
  (~(((1 << HBN_F32K_SEL_LEN) - 1) << HBN_F32K_SEL_POS))
#define HBN_PU_RC32K     HBN_PU_RC32K
#define HBN_PU_RC32K_POS (5)
#define HBN_PU_RC32K_LEN (1)
#define HBN_PU_RC32K_MSK (((1 << HBN_PU_RC32K_LEN) - 1) << HBN_PU_RC32K_POS)
#define HBN_PU_RC32K_UMSK \
  (~(((1 << HBN_PU_RC32K_LEN) - 1) << HBN_PU_RC32K_POS))
#define HBN_SW_LDO11SOC_VOUT_SEL_AON     HBN_SW_LDO11SOC_VOUT_SEL_AON
#define HBN_SW_LDO11SOC_VOUT_SEL_AON_POS (16)
#define HBN_SW_LDO11SOC_VOUT_SEL_AON_LEN (4)
#define HBN_SW_LDO11SOC_VOUT_SEL_AON_MSK \
  (((1 << HBN_SW_LDO11SOC_VOUT_SEL_AON_LEN) - 1) \
   << HBN_SW_LDO11SOC_VOUT_SEL_AON_POS)
#define HBN_SW_LDO11SOC_VOUT_SEL_AON_UMSK \
  (~(((1 << HBN_SW_LDO11SOC_VOUT_SEL_AON_LEN) - 1) \
     << HBN_SW_LDO11SOC_VOUT_SEL_AON_POS))
#define HBN_SW_LDO11_RT_VOUT_SEL     HBN_SW_LDO11_RT_VOUT_SEL
#define HBN_SW_LDO11_RT_VOUT_SEL_POS (24)
#define HBN_SW_LDO11_RT_VOUT_SEL_LEN (4)
#define HBN_SW_LDO11_RT_VOUT_SEL_MSK \
  (((1 << HBN_SW_LDO11_RT_VOUT_SEL_LEN) - 1) << HBN_SW_LDO11_RT_VOUT_SEL_POS)
#define HBN_SW_LDO11_RT_VOUT_SEL_UMSK \
  (~(((1 << HBN_SW_LDO11_RT_VOUT_SEL_LEN) - 1) \
     << HBN_SW_LDO11_RT_VOUT_SEL_POS))
#define HBN_SW_LDO11_AON_VOUT_SEL     HBN_SW_LDO11_AON_VOUT_SEL
#define HBN_SW_LDO11_AON_VOUT_SEL_POS (28)
#define HBN_SW_LDO11_AON_VOUT_SEL_LEN (4)
#define HBN_SW_LDO11_AON_VOUT_SEL_MSK \
  (((1 << HBN_SW_LDO11_AON_VOUT_SEL_LEN) - 1) \
   << HBN_SW_LDO11_AON_VOUT_SEL_POS)
#define HBN_SW_LDO11_AON_VOUT_SEL_UMSK \
  (~(((1 << HBN_SW_LDO11_AON_VOUT_SEL_LEN) - 1) \
     << HBN_SW_LDO11_AON_VOUT_SEL_POS))

/* 0x34 : HBN_SRAM */

#define HBN_SRAM_OFFSET    (0x34)
#define HBN_RETRAM_RET     HBN_RETRAM_RET
#define HBN_RETRAM_RET_POS (6)
#define HBN_RETRAM_RET_LEN (1)
#define HBN_RETRAM_RET_MSK \
  (((1 << HBN_RETRAM_RET_LEN) - 1) << HBN_RETRAM_RET_POS)
#define HBN_RETRAM_RET_UMSK \
  (~(((1 << HBN_RETRAM_RET_LEN) - 1) << HBN_RETRAM_RET_POS))
#define HBN_RETRAM_SLP     HBN_RETRAM_SLP
#define HBN_RETRAM_SLP_POS (7)
#define HBN_RETRAM_SLP_LEN (1)
#define HBN_RETRAM_SLP_MSK \
  (((1 << HBN_RETRAM_SLP_LEN) - 1) << HBN_RETRAM_SLP_POS)
#define HBN_RETRAM_SLP_UMSK \
  (~(((1 << HBN_RETRAM_SLP_LEN) - 1) << HBN_RETRAM_SLP_POS))

/* 0x100 : HBN_RSV0 */

#define HBN_RSV0_OFFSET (0x100)
#define HBN_RSV0        HBN_RSV0
#define HBN_RSV0_POS    (0)
#define HBN_RSV0_LEN    (32)
#define HBN_RSV0_MSK    (((1 << HBN_RSV0_LEN) - 1) << HBN_RSV0_POS)
#define HBN_RSV0_UMSK   (~(((1 << HBN_RSV0_LEN) - 1) << HBN_RSV0_POS))

/* 0x104 : HBN_RSV1 */

#define HBN_RSV1_OFFSET (0x104)
#define HBN_RSV1        HBN_RSV1
#define HBN_RSV1_POS    (0)
#define HBN_RSV1_LEN    (32)
#define HBN_RSV1_MSK    (((1 << HBN_RSV1_LEN) - 1) << HBN_RSV1_POS)
#define HBN_RSV1_UMSK   (~(((1 << HBN_RSV1_LEN) - 1) << HBN_RSV1_POS))

/* 0x108 : HBN_RSV2 */

#define HBN_RSV2_OFFSET (0x108)
#define HBN_RSV2        HBN_RSV2
#define HBN_RSV2_POS    (0)
#define HBN_RSV2_LEN    (32)
#define HBN_RSV2_MSK    (((1 << HBN_RSV2_LEN) - 1) << HBN_RSV2_POS)
#define HBN_RSV2_UMSK   (~(((1 << HBN_RSV2_LEN) - 1) << HBN_RSV2_POS))

/* 0x10C : HBN_RSV3 */

#define HBN_RSV3_OFFSET (0x10C)
#define HBN_RSV3        HBN_RSV3
#define HBN_RSV3_POS    (0)
#define HBN_RSV3_LEN    (32)
#define HBN_RSV3_MSK    (((1 << HBN_RSV3_LEN) - 1) << HBN_RSV3_POS)
#define HBN_RSV3_UMSK   (~(((1 << HBN_RSV3_LEN) - 1) << HBN_RSV3_POS))

/* 0x200 : rc32k_ctrl0 */

#define HBN_RC32K_CTRL0_OFFSET (0x200)
#define HBN_RC32K_CAL_DONE     HBN_RC32K_CAL_DONE
#define HBN_RC32K_CAL_DONE_POS (0)
#define HBN_RC32K_CAL_DONE_LEN (1)
#define HBN_RC32K_CAL_DONE_MSK \
  (((1 << HBN_RC32K_CAL_DONE_LEN) - 1) << HBN_RC32K_CAL_DONE_POS)
#define HBN_RC32K_CAL_DONE_UMSK \
  (~(((1 << HBN_RC32K_CAL_DONE_LEN) - 1) << HBN_RC32K_CAL_DONE_POS))
#define HBN_RC32K_RDY     HBN_RC32K_RDY
#define HBN_RC32K_RDY_POS (1)
#define HBN_RC32K_RDY_LEN (1)
#define HBN_RC32K_RDY_MSK \
  (((1 << HBN_RC32K_RDY_LEN) - 1) << HBN_RC32K_RDY_POS)
#define HBN_RC32K_RDY_UMSK \
  (~(((1 << HBN_RC32K_RDY_LEN) - 1) << HBN_RC32K_RDY_POS))
#define HBN_RC32K_CAL_INPROGRESS     HBN_RC32K_CAL_INPROGRESS
#define HBN_RC32K_CAL_INPROGRESS_POS (2)
#define HBN_RC32K_CAL_INPROGRESS_LEN (1)
#define HBN_RC32K_CAL_INPROGRESS_MSK \
  (((1 << HBN_RC32K_CAL_INPROGRESS_LEN) - 1) << HBN_RC32K_CAL_INPROGRESS_POS)
#define HBN_RC32K_CAL_INPROGRESS_UMSK \
  (~(((1 << HBN_RC32K_CAL_INPROGRESS_LEN) - 1) \
     << HBN_RC32K_CAL_INPROGRESS_POS))
#define HBN_RC32K_CAL_DIV     HBN_RC32K_CAL_DIV
#define HBN_RC32K_CAL_DIV_POS (3)
#define HBN_RC32K_CAL_DIV_LEN (2)
#define HBN_RC32K_CAL_DIV_MSK \
  (((1 << HBN_RC32K_CAL_DIV_LEN) - 1) << HBN_RC32K_CAL_DIV_POS)
#define HBN_RC32K_CAL_DIV_UMSK \
  (~(((1 << HBN_RC32K_CAL_DIV_LEN) - 1) << HBN_RC32K_CAL_DIV_POS))
#define HBN_RC32K_CAL_PRECHARGE     HBN_RC32K_CAL_PRECHARGE
#define HBN_RC32K_CAL_PRECHARGE_POS (5)
#define HBN_RC32K_CAL_PRECHARGE_LEN (1)
#define HBN_RC32K_CAL_PRECHARGE_MSK \
  (((1 << HBN_RC32K_CAL_PRECHARGE_LEN) - 1) << HBN_RC32K_CAL_PRECHARGE_POS)
#define HBN_RC32K_CAL_PRECHARGE_UMSK \
  (~(((1 << HBN_RC32K_CAL_PRECHARGE_LEN) - 1) << HBN_RC32K_CAL_PRECHARGE_POS))
#define HBN_RC32K_DIG_CODE_FR_CAL     HBN_RC32K_DIG_CODE_FR_CAL
#define HBN_RC32K_DIG_CODE_FR_CAL_POS (6)
#define HBN_RC32K_DIG_CODE_FR_CAL_LEN (10)
#define HBN_RC32K_DIG_CODE_FR_CAL_MSK \
  (((1 << HBN_RC32K_DIG_CODE_FR_CAL_LEN) - 1) \
   << HBN_RC32K_DIG_CODE_FR_CAL_POS)
#define HBN_RC32K_DIG_CODE_FR_CAL_UMSK \
  (~(((1 << HBN_RC32K_DIG_CODE_FR_CAL_LEN) - 1) \
     << HBN_RC32K_DIG_CODE_FR_CAL_POS))
#define HBN_RC32K_VREF_DLY     HBN_RC32K_VREF_DLY
#define HBN_RC32K_VREF_DLY_POS (16)
#define HBN_RC32K_VREF_DLY_LEN (2)
#define HBN_RC32K_VREF_DLY_MSK \
  (((1 << HBN_RC32K_VREF_DLY_LEN) - 1) << HBN_RC32K_VREF_DLY_POS)
#define HBN_RC32K_VREF_DLY_UMSK \
  (~(((1 << HBN_RC32K_VREF_DLY_LEN) - 1) << HBN_RC32K_VREF_DLY_POS))
#define HBN_RC32K_ALLOW_CAL     HBN_RC32K_ALLOW_CAL
#define HBN_RC32K_ALLOW_CAL_POS (18)
#define HBN_RC32K_ALLOW_CAL_LEN (1)
#define HBN_RC32K_ALLOW_CAL_MSK \
  (((1 << HBN_RC32K_ALLOW_CAL_LEN) - 1) << HBN_RC32K_ALLOW_CAL_POS)
#define HBN_RC32K_ALLOW_CAL_UMSK \
  (~(((1 << HBN_RC32K_ALLOW_CAL_LEN) - 1) << HBN_RC32K_ALLOW_CAL_POS))
#define HBN_RC32K_EXT_CODE_EN     HBN_RC32K_EXT_CODE_EN
#define HBN_RC32K_EXT_CODE_EN_POS (19)
#define HBN_RC32K_EXT_CODE_EN_LEN (1)
#define HBN_RC32K_EXT_CODE_EN_MSK \
  (((1 << HBN_RC32K_EXT_CODE_EN_LEN) - 1) << HBN_RC32K_EXT_CODE_EN_POS)
#define HBN_RC32K_EXT_CODE_EN_UMSK \
  (~(((1 << HBN_RC32K_EXT_CODE_EN_LEN) - 1) << HBN_RC32K_EXT_CODE_EN_POS))
#define HBN_RC32K_CAL_EN     HBN_RC32K_CAL_EN
#define HBN_RC32K_CAL_EN_POS (20)
#define HBN_RC32K_CAL_EN_LEN (1)
#define HBN_RC32K_CAL_EN_MSK \
  (((1 << HBN_RC32K_CAL_EN_LEN) - 1) << HBN_RC32K_CAL_EN_POS)
#define HBN_RC32K_CAL_EN_UMSK \
  (~(((1 << HBN_RC32K_CAL_EN_LEN) - 1) << HBN_RC32K_CAL_EN_POS))
#define HBN_RC32K_CODE_FR_EXT     HBN_RC32K_CODE_FR_EXT
#define HBN_RC32K_CODE_FR_EXT_POS (22)
#define HBN_RC32K_CODE_FR_EXT_LEN (10)
#define HBN_RC32K_CODE_FR_EXT_MSK \
  (((1 << HBN_RC32K_CODE_FR_EXT_LEN) - 1) << HBN_RC32K_CODE_FR_EXT_POS)
#define HBN_RC32K_CODE_FR_EXT_UMSK \
  (~(((1 << HBN_RC32K_CODE_FR_EXT_LEN) - 1) << HBN_RC32K_CODE_FR_EXT_POS))

/* 0x204 : xtal32k */

#define HBN_XTAL32K_OFFSET      (0x204)
#define HBN_XTAL32K_EXT_SEL     HBN_XTAL32K_EXT_SEL
#define HBN_XTAL32K_EXT_SEL_POS (2)
#define HBN_XTAL32K_EXT_SEL_LEN (1)
#define HBN_XTAL32K_EXT_SEL_MSK \
  (((1 << HBN_XTAL32K_EXT_SEL_LEN) - 1) << HBN_XTAL32K_EXT_SEL_POS)
#define HBN_XTAL32K_EXT_SEL_UMSK \
  (~(((1 << HBN_XTAL32K_EXT_SEL_LEN) - 1) << HBN_XTAL32K_EXT_SEL_POS))
#define HBN_XTAL32K_AMP_CTRL     HBN_XTAL32K_AMP_CTRL
#define HBN_XTAL32K_AMP_CTRL_POS (3)
#define HBN_XTAL32K_AMP_CTRL_LEN (2)
#define HBN_XTAL32K_AMP_CTRL_MSK \
  (((1 << HBN_XTAL32K_AMP_CTRL_LEN) - 1) << HBN_XTAL32K_AMP_CTRL_POS)
#define HBN_XTAL32K_AMP_CTRL_UMSK \
  (~(((1 << HBN_XTAL32K_AMP_CTRL_LEN) - 1) << HBN_XTAL32K_AMP_CTRL_POS))
#define HBN_XTAL32K_REG     HBN_XTAL32K_REG
#define HBN_XTAL32K_REG_POS (5)
#define HBN_XTAL32K_REG_LEN (2)
#define HBN_XTAL32K_REG_MSK \
  (((1 << HBN_XTAL32K_REG_LEN) - 1) << HBN_XTAL32K_REG_POS)
#define HBN_XTAL32K_REG_UMSK \
  (~(((1 << HBN_XTAL32K_REG_LEN) - 1) << HBN_XTAL32K_REG_POS))
#define HBN_XTAL32K_OUTBUF_STRE     HBN_XTAL32K_OUTBUF_STRE
#define HBN_XTAL32K_OUTBUF_STRE_POS (7)
#define HBN_XTAL32K_OUTBUF_STRE_LEN (1)
#define HBN_XTAL32K_OUTBUF_STRE_MSK \
  (((1 << HBN_XTAL32K_OUTBUF_STRE_LEN) - 1) << HBN_XTAL32K_OUTBUF_STRE_POS)
#define HBN_XTAL32K_OUTBUF_STRE_UMSK \
  (~(((1 << HBN_XTAL32K_OUTBUF_STRE_LEN) - 1) << HBN_XTAL32K_OUTBUF_STRE_POS))
#define HBN_XTAL32K_OTF_SHORT     HBN_XTAL32K_OTF_SHORT
#define HBN_XTAL32K_OTF_SHORT_POS (8)
#define HBN_XTAL32K_OTF_SHORT_LEN (1)
#define HBN_XTAL32K_OTF_SHORT_MSK \
  (((1 << HBN_XTAL32K_OTF_SHORT_LEN) - 1) << HBN_XTAL32K_OTF_SHORT_POS)
#define HBN_XTAL32K_OTF_SHORT_UMSK \
  (~(((1 << HBN_XTAL32K_OTF_SHORT_LEN) - 1) << HBN_XTAL32K_OTF_SHORT_POS))
#define HBN_XTAL32K_INV_STRE     HBN_XTAL32K_INV_STRE
#define HBN_XTAL32K_INV_STRE_POS (9)
#define HBN_XTAL32K_INV_STRE_LEN (2)
#define HBN_XTAL32K_INV_STRE_MSK \
  (((1 << HBN_XTAL32K_INV_STRE_LEN) - 1) << HBN_XTAL32K_INV_STRE_POS)
#define HBN_XTAL32K_INV_STRE_UMSK \
  (~(((1 << HBN_XTAL32K_INV_STRE_LEN) - 1) << HBN_XTAL32K_INV_STRE_POS))
#define HBN_XTAL32K_CAPBANK     HBN_XTAL32K_CAPBANK
#define HBN_XTAL32K_CAPBANK_POS (11)
#define HBN_XTAL32K_CAPBANK_LEN (6)
#define HBN_XTAL32K_CAPBANK_MSK \
  (((1 << HBN_XTAL32K_CAPBANK_LEN) - 1) << HBN_XTAL32K_CAPBANK_POS)
#define HBN_XTAL32K_CAPBANK_UMSK \
  (~(((1 << HBN_XTAL32K_CAPBANK_LEN) - 1) << HBN_XTAL32K_CAPBANK_POS))
#define HBN_XTAL32K_AC_CAP_SHORT     HBN_XTAL32K_AC_CAP_SHORT
#define HBN_XTAL32K_AC_CAP_SHORT_POS (17)
#define HBN_XTAL32K_AC_CAP_SHORT_LEN (1)
#define HBN_XTAL32K_AC_CAP_SHORT_MSK \
  (((1 << HBN_XTAL32K_AC_CAP_SHORT_LEN) - 1) << HBN_XTAL32K_AC_CAP_SHORT_POS)
#define HBN_XTAL32K_AC_CAP_SHORT_UMSK \
  (~(((1 << HBN_XTAL32K_AC_CAP_SHORT_LEN) - 1) \
     << HBN_XTAL32K_AC_CAP_SHORT_POS))
#define HBN_PU_XTAL32K_BUF     HBN_PU_XTAL32K_BUF
#define HBN_PU_XTAL32K_BUF_POS (18)
#define HBN_PU_XTAL32K_BUF_LEN (1)
#define HBN_PU_XTAL32K_BUF_MSK \
  (((1 << HBN_PU_XTAL32K_BUF_LEN) - 1) << HBN_PU_XTAL32K_BUF_POS)
#define HBN_PU_XTAL32K_BUF_UMSK \
  (~(((1 << HBN_PU_XTAL32K_BUF_LEN) - 1) << HBN_PU_XTAL32K_BUF_POS))
#define HBN_PU_XTAL32K     HBN_PU_XTAL32K
#define HBN_PU_XTAL32K_POS (19)
#define HBN_PU_XTAL32K_LEN (1)
#define HBN_PU_XTAL32K_MSK \
  (((1 << HBN_PU_XTAL32K_LEN) - 1) << HBN_PU_XTAL32K_POS)
#define HBN_PU_XTAL32K_UMSK \
  (~(((1 << HBN_PU_XTAL32K_LEN) - 1) << HBN_PU_XTAL32K_POS))

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* HBN UART clock type definition */

#define HBN_UART_CLK_FCLK 0 /* Select FCLK as UART clock */
#define HBN_UART_CLK_160M 1 /* Select 160M as UART clock */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_set_uart_clk_sel
 *
 * Description:
 *   Select uart clock source.
 *
 * Input Parameters:
 *   clk_sel: uart clock type selection
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

EXTERN void bl602_hbn_set_uart_clk_sel(int clk_sel);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_HBN_H */
