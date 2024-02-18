/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm6300/hpm6300_pcfg.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_PCFG_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_PCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hpm_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_PCFG_BANDGAP                (HPM_PCFG_BASE + 0x0000)
#define HPM_PCFG_LDO1P1                 (HPM_PCFG_BASE + 0x0004)
#define HPM_PCFG_LDO2P5                 (HPM_PCFG_BASE + 0x0008)
#define HPM_PCFG_DCDC_MODE              (HPM_PCFG_BASE + 0x0010)
#define HPM_PCFG_DCDC_LPMODE            (HPM_PCFG_BASE + 0x0014)
#define HPM_PCFG_DCDC_PROT              (HPM_PCFG_BASE + 0x0018)
#define HPM_PCFG_DCDC_CURRENT           (HPM_PCFG_BASE + 0x001C)
#define HPM_PCFG_DCDC_ADVMODE           (HPM_PCFG_BASE + 0x0020)
#define HPM_PCFG_DCDC_ADVPARAM          (HPM_PCFG_BASE + 0x0024)
#define HPM_PCFG_DCDC_MISC              (HPM_PCFG_BASE + 0x0028)
#define HPM_PCFG_DCDC_DEBUG             (HPM_PCFG_BASE + 0x002C)
#define HPM_PCFG_DCDC_START_TIME        (HPM_PCFG_BASE + 0x0030)
#define HPM_PCFG_DCDC_RESUME_TIME       (HPM_PCFG_BASE + 0x0034)
#define HPM_PCFG_POWER_TRAP             (HPM_PCFG_BASE + 0x0040)
#define HPM_PCFG_WAKE_CAUSE             (HPM_PCFG_BASE + 0x0044)
#define HPM_PCFG_WAK_MASK               (HPM_PCFG_BASE + 0x0048)
#define HPM_PCFG_SCG_CTRL               (HPM_PCFG_BASE + 0x004C)
#define HPM_PCFG_DEBUG_STOP             (HPM_PCFG_BASE + 0x0050)
#define HPM_PCFG_RC24M                  (HPM_PCFG_BASE + 0x0060)
#define HPM_PCFG_RC24M_TRACK            (HPM_PCFG_BASE + 0x0064)
#define HPM_PCFG_TRACK_TARGET           (HPM_PCFG_BASE + 0x0068)
#define HPM_PCFG_STATUS                 (HPM_PCFG_BASE + 0x006C)

#define HPM_PCFG_BANDGAP_VBG_P50_TRIM_SHIFT         (0)
#define HPM_PCFG_BANDGAP_VBG_P50_TRIM_MASK          (0x1F << HPM_PCFG_BANDGAP_VBG_P50_TRIM_SHIFT)
#define HPM_PCFG_BANDGAP_VBG_P50_TRIM(n)            ((n) << HPM_PCFG_BANDGAP_VBG_P50_TRIM_SHIFT)

#define HPM_PCFG_BANDGAP_VBG_P65_TRIM_SHIFT         (8)
#define HPM_PCFG_BANDGAP_VBG_P65_TRIM_MASK          (0x1F << HPM_PCFG_BANDGAP_VBG_P65_TRIM_SHIFT)
#define HPM_PCFG_BANDGAP_VBG_P65_TRIM(n)            ((n) << HPM_PCFG_BANDGAP_VBG_P65_TRIM_SHIFT)

#define HPM_PCFG_BANDGAP_VBG_1P0_TRIM_SHIFT         (16)
#define HPM_PCFG_BANDGAP_VBG_1P0_TRIM_MASK          (0x1F << HPM_PCFG_BANDGAP_VBG_1P0_TRIM_SHIFT)
#define HPM_PCFG_BANDGAP_VBG_1P0_TRIM(n)            ((n) << HPM_PCFG_BANDGAP_VBG_1P0_TRIM_SHIFT)

#define HPM_PCFG_BANDGAP_POWER_SAVE_SHIFT           (24)
#define HPM_PCFG_BANDGAP_POWER_SAVE_NORMAL          (0 << HPM_PCFG_BANDGAP_POWER_SAVE_SHIFT)
#define HPM_PCFG_BANDGAP_POWER_SAVE_LOW             (1 << HPM_PCFG_BANDGAP_POWER_SAVE_SHIFT)

#define HPM_PCFG_BANDGAP_LOWPOWER_MODE_SHIFT        (25)
#define HPM_PCFG_BANDGAP_LOWPOWER_MODE_NORMAL       (0 << HPM_PCFG_BANDGAP_LOWPOWER_MODE_SHIFT)
#define HPM_PCFG_BANDGAP_LOWPOWER_MODE_LOW          (1 << HPM_PCFG_BANDGAP_LOWPOWER_MODE_SHIFT)

#define HPM_PCFG_BANDGAP_VBG_TRIMMED_SHIFT          (31)
#define HPM_PCFG_BANDGAP_VBG_TRIMMED_UNCALIBRATED   (0 << HPM_PCFG_BANDGAP_VBG_TRIMMED_SHIFT)
#define HPM_PCFG_BANDGAP_VBG_TRIMMED_CALIBRATED     (1 << HPM_PCFG_BANDGAP_VBG_TRIMMED_SHIFT)

#define HPM_PCFG_LDO1P1_VOLT_SHIFT                  (0)
#define HPM_PCFG_LDO1P1_VOLT_MASK                   (0x7FF << HPM_PCFG_LDO1P1_VOLT_SHIFT)
#define HPM_PCFG_LDO1P1_VOLT(n)                     ((n) << HPM_PCFG_LDO1P1_VOLT_SHIFT)

#define HPM_PCFG_LDO2P5_VOLT_SHIFT                  (0)
#define HPM_PCFG_LDO2P5_VOLT_MASK                   (0x7FF << HPM_PCFG_LDO2P5_VOLT_SHIFT)
#define HPM_PCFG_LDO2P5_VOLT(n)                     ((n) << HPM_PCFG_LDO2P5_VOLT_SHIFT)

#define HPM_PCFG_LDO2P5_ENABLE_SHIFT                (16)
#define HPM_PCFG_LDO2P5_ENABLE_DISABL               (0 << HPM_PCFG_LDO2P5_ENABLE_SHIFT)
#define HPM_PCFG_LDO2P5_ENABLE_ENABLE               (1 << HPM_PCFG_LDO2P5_ENABLE_SHIFT)

#define HPM_PCFG_LDO2P5_READY_SHIFT                 (28)
#define HPM_PCFG_LDO2P5_READY_NOT_RE                (0 << HPM_PCFG_LDO2P5_READY_SHIFT)
#define HPM_PCFG_LDO2P5_READY_READY                 (1 << HPM_PCFG_LDO2P5_READY_SHIFT)

#define HPM_PCFG_DCDC_MODE_VOLT_SHIFT               (0)
#define HPM_PCFG_DCDC_MODE_VOLT_MASK                (0x7FF << HPM_PCFG_DCDC_MODE_VOLT_SHIFT)
#define HPM_PCFG_DCDC_MODE_VOLT(n)                  ((n) << HPM_PCFG_DCDC_MODE_VOLT_SHIFT)

#define HPM_PCFG_DCDC_MODE_MODE_SHIFT               (16)
#define HPM_PCFG_DCDC_MODE_MODE_MASK                (0x07 << HPM_PCFG_DCDC_MODE_MODE_SHIFT)
#define HPM_PCFG_DCDC_MODE_MODE(n)                  ((n) << HPM_PCFG_DCDC_MODE_MODE_SHIFT)
#define HPM_PCFG_DCDC_MODE_CLOSE                    (0 << HPM_PCFG_DCDC_MODE_MODE_SHIFT)
#define HPM_PCFG_DCDC_MODE_NORMAL                   (1 << HPM_PCFG_DCDC_MODE_MODE_SHIFT)
#define HPM_PCFG_DCDC_MODE_UNIVERSAL                (3 << HPM_PCFG_DCDC_MODE_MODE_SHIFT)
#define HPM_PCFG_DCDC_MODE_PRO                      (7 << HPM_PCFG_DCDC_MODE_MODE_SHIFT)

#define HPM_PCFG_DCDC_MODE_READY_SHIFT              (28)
#define HPM_PCFG_DCDC_MODE_READY_NOT_READY          (0 << HPM_PCFG_DCDC_MODE_READY_SHIFT)
#define HPM_PCFG_DCDC_MODE_READY_READY              (1 << HPM_PCFG_DCDC_MODE_READY_SHIFT)

#define HPM_PCFG_DCDC_LPMODE_STBY_VOLT_SHIFT        (0)
#define HPM_PCFG_DCDC_LPMODE_STBY_VOLT_MASK         (0x7FF << HPM_PCFG_DCDC_LPMODE_STBY_VOLT_SHIFT)
#define HPM_PCFG_DCDC_LPMODE_STBY_VOLT(n)           ((n) << HPM_PCFG_DCDC_LPMODE_STBY_VOLT_SHIFT)

#endif /* __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM6300_HPM6300_PCFG_H */
