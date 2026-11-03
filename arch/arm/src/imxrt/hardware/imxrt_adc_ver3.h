/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_adc_ver3.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER3_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_ADC_VER3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_ADC_CTRL_OFFSET       0x010
#define IMXRT_ADC_STAT_OFFSET       0x014
#define IMXRT_ADC_IE_OFFSET         0x018
#define IMXRT_ADC_CFG_OFFSET        0x020
#define IMXRT_ADC_SWTRIG_OFFSET     0x034
#define IMXRT_ADC_TCTRL0_OFFSET     0x0a0
#define IMXRT_ADC_FCTRL0_OFFSET     0x0e0
#define IMXRT_ADC_CMDL1_OFFSET      0x100
#define IMXRT_ADC_CMDH1_OFFSET      0x104
#define IMXRT_ADC_RESFIFO0_OFFSET   0x300

#define IMXRT_ADC_CTRL_ADCEN        (1u << 0)
#define IMXRT_ADC_CTRL_RST          (1u << 1)
#define IMXRT_ADC_CTRL_CAL_REQ      (1u << 3)
#define IMXRT_ADC_CTRL_RSTFIFO0     (1u << 8)
#define IMXRT_ADC_CTRL_RSTFIFO1     (1u << 9)

#define IMXRT_ADC_STAT_CAL_RDY      (1u << 10)
#define IMXRT_ADC_IE_FWMIE0         (1u << 0)
#define IMXRT_ADC_CFG_REFSEL_SHIFT  6
#define IMXRT_ADC_CFG_PUDLY_SHIFT   16
#define IMXRT_ADC_CFG_PWREN         (1u << 28)

#define IMXRT_ADC_TCTRL_TCMD(x)     (((uint32_t)(x) & 0xfu) << 24)
#define IMXRT_ADC_FCTRL_FCOUNT_MASK 0x1fu
#define IMXRT_ADC_FCTRL_FWMARK(x)   (((uint32_t)(x) & 0xfu) << 16)

#define IMXRT_ADC_CMDL_ADCH(x)      ((uint32_t)(x) & 0x1fu)
#define IMXRT_ADC_CMDL_CTYPE(x)     (((uint32_t)(x) & 0x3u) << 5)
#define IMXRT_ADC_CMDL_CTYPE_A      0
#define IMXRT_ADC_CMDL_CTYPE_B      1
#define IMXRT_ADC_CMDL_CTYPE_SINGLE 0
#define IMXRT_ADC_CMDL_MODE_12BIT   0
#define IMXRT_ADC_CMDL_CSCALE(x)    (((uint32_t)(x) & 0x3u) << 13)

#define IMXRT_ADC_RESFIFO_VALID     (1u << 31)
#define IMXRT_ADC_RESFIFO_DATA_MASK 0xffffu

#endif
