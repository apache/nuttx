/****************************************************************************
 * arch/arm/src/eoss3/hardware/eoss3_intr.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_INTR_H
#define __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_INTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define EOSS3_INTR_GPIO_OFFSET           0x0000
#define EOSS3_INTR_GPIO_RAW_OFFSET       0x0004
#define EOSS3_INTR_GPIO_TYPE_OFFSET      0x0008
#define EOSS3_INTR_GPIO_POL_OFFSET       0x000c
#define EOSS3_INTR_GPIO_EN_AP_OFFSET     0x0010
#define EOSS3_INTR_GPIO_EN_M4_OFFSET     0x0014
#define EOSS3_INTR_GPIO_EN_FFE0_OFFSET   0x0018
#define EOSS3_INTR_GPIO_EN_FFE1_OFFSET   0x001C
#define EOSS3_INTR_OTHER_OFFSET          0x0030
#define EOSS3_INTR_OTHER_EN_AP_OFFSET    0x0034
#define EOSS3_INTR_OTHER_EN_M4_OFFSET    0x0038
#define EOSS3_INTR_SW_1_OFFSET           0x0040
#define EOSS3_INTR_SW_1_EN_AP_OFFSET     0x0044
#define EOSS3_INTR_SW_1_EN_M4_OFFSET     0x0048
#define EOSS3_INTR_SW_2_OFFSET           0x0050
#define EOSS3_INTR_SW_2_EN_AP_OFFSET     0x0054
#define EOSS3_INTR_SW_2_EN_M4_OFFSET     0x0058
#define EOSS3_INTR_FFE_OFFSET            0x0060
#define EOSS3_INTR_FFE_EN_AP_OFFSET      0x0064
#define EOSS3_INTR_FFE_EN_M4_OFFSET      0x0068
#define EOSS3_INTR_FFE1_FB_OFFSET        0x0070
#define EOSS3_INTR_FFE1_FB_EN_AP_OFFSET  0x0074
#define EOSS3_INTR_FFE1_FB_EN_M4_OFFSET  0x0078
#define EOSS3_INTR_FB_OFFSET             0x0080
#define EOSS3_INTR_FB_RAW_OFFSET         0x0084
#define EOSS3_INTR_FB_TYPE_OFFSET        0x0088
#define EOSS3_INTR_FB_POL_OFFSET         0x008c
#define EOSS3_INTR_FB_EN_AP_OFFSET       0x0090
#define EOSS3_INTR_FB_EN_M4_OFFSET       0x0094
#define EOSS3_INTR_M4_MEM_AON_OFFSET     0x00a0
#define EOSS3_INTR_M4_MEM_AON_EN_OFFSET  0x00a4

/* Register Addresses *******************************************************/

#define EOSS3_INTR_GPIO           (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_OFFSET)
#define EOSS3_INTR_GPIO_RAW       (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_RAW_OFFSET)
#define EOSS3_INTR_GPIO_TYPE      (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_TYPE_OFFSET)
#define EOSS3_INTR_GPIO_POL       (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_POL_OFFSET)
#define EOSS3_INTR_GPIO_EN_AP     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_EN_AP_OFFSET)
#define EOSS3_INTR_GPIO_EN_M4     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_EN_M4_OFFSET)
#define EOSS3_INTR_GPIO_EN_FFE0   (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_EN_FFE0_OFFSET)
#define EOSS3_INTR_GPIO_EN_FFE1   (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_GPIO_EN_FFE1_OFFSET)
#define EOSS3_INTR_OTHER          (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_OTHER_OFFSET)
#define EOSS3_INTR_OTHER_EN_AP    (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_OTHER_EN_AP_OFFSET)
#define EOSS3_INTR_OTHER_EN_M4    (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_OTHER_EN_M4_OFFSET)
#define EOSS3_INTR_SW_1           (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_1_OFFSET)
#define EOSS3_INTR_SW_1_EN_AP     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_1_EN_AP_OFFSET)
#define EOSS3_INTR_SW_1_EN_M4     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_1_EN_M4_OFFSET)
#define EOSS3_INTR_SW_2           (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_2_OFFSET)
#define EOSS3_INTR_SW_2_EN_AP     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_2_EN_AP_OFFSET)
#define EOSS3_INTR_SW_2_EN_M4     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_SW_2_EN_M4_OFFSET)
#define EOSS3_INTR_FFE            (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE_OFFSET)
#define EOSS3_INTR_FFE_EN_AP      (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE_EN_AP_OFFSET)
#define EOSS3_INTR_FFE_EN_M4      (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE_EN_M4_OFFSET)
#define EOSS3_INTR_FFE1_FB        (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE1_FB_OFFSET)
#define EOSS3_INTR_FFE1_FB_EN_AP  (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE1_FB_EN_AP_OFFSET)
#define EOSS3_INTR_FFE1_FB_EN_M4  (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FFE1_FB_EN_M4_OFFSET)
#define EOSS3_INTR_FB             (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_OFFSET)
#define EOSS3_INTR_FB_RAW         (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_RAW_OFFSET)
#define EOSS3_INTR_FB_TYPE        (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_TYPE_OFFSET)
#define EOSS3_INTR_FB_POL         (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_POL_OFFSET)
#define EOSS3_INTR_FB_EN_AP       (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_EN_AP_OFFSET)
#define EOSS3_INTR_FB_EN_M4       (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_FB_EN_M4_OFFSET)
#define EOSS3_INTR_M4_MEM_AON     (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_M4_MEM_AON_OFFSET)
#define EOSS3_INTR_M4_MEM_AON_EN  (EOSS3_INTR_CTRL_BASE + EOSS3_INTR_M4_MEM_AON_EN_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* EOSS3_INTR_OTHER Register */

#define INTR_M4_SRAM_DET          (1 << 0)
#define INTR_UART_DET             (1 << 1)
#define INTR_TIMER_DET            (1 << 2)
#define INTR_WDT_DET              (1 << 3)
#define INTR_WDT_RST_DET          (1 << 4)
#define INTR_TIMEOUT_DET          (1 << 5)
#define INTR_FPU_DET              (1 << 6)
#define INTR_PKFB_DET             (1 << 7)
#define INTR_I2S_DET              (1 << 8)
#define INTR_AUD_DET              (1 << 9)
#define INTR_SPI_MS_DET           (1 << 10)
#define INTR_CFG_DMA_DET          (1 << 11)
#define INTR_PMU_TMR_DET          (1 << 12)
#define INTR_ADC_DET              (1 << 13)
#define INTR_RTC_DET              (1 << 14)
#define INTR_RST_DET              (1 << 15)
#define INTR_FFE0_DET             (1 << 16)
#define INTR_WDT_FFE_DET          (1 << 17)
#define INTR_APBOOT_EN_DET        (1 << 18)
#define INTR_LPSD_VOICE_DET       (1 << 22)
#define INTR_DMIC_VOICE_DET       (1 << 23)

/* EOSS3_INTR_OTHER_EN_AP Register */

#define INTR_M4_SRAM_EN_AP        (1 << 0)
#define INTR_UART_EN_AP           (1 << 1)
#define INTR_TIMER_EN_AP          (1 << 2)
#define INTR_WDT_EN_AP            (1 << 3)
#define INTR_WDT_RST_EN_AP        (1 << 4)
#define INTR_TIMEOUT_EN_AP        (1 << 5)
#define INTR_FPU_EN_AP            (1 << 6)
#define INTR_PKFB_EN_AP           (1 << 7)
#define INTR_I2S_EN_AP            (1 << 8)
#define INTR_AUD_EN_AP            (1 << 9)
#define INTR_SPI_MS_EN_AP         (1 << 10)
#define INTR_CFG_DMA_EN_AP        (1 << 11)
#define INTR_PMU_TMR_EN_AP        (1 << 12)
#define INTR_ADC_EN_AP            (1 << 13)
#define INTR_RTC_EN_AP            (1 << 14)
#define INTR_RST_EN_AP            (1 << 15)
#define INTR_FFE0_EN_AP           (1 << 16)
#define INTR_WDT_FFE_EN_AP        (1 << 17)
#define INTR_APBOOT_EN_AP         (1 << 18)
#define INTR_LD030_PG_EN_AP       (1 << 19)
#define INTR_LD050_PG_EN_AP       (1 << 20)
#define INTR_SRAM_128KB_EN_AP     (1 << 21)
#define INTR_LPSD_VOICE_EN_AP     (1 << 22)
#define INTR_DMIC_VOICE_EN_AP     (1 << 23)

/* EOSS3_INTR_OTHER_EN_M4 Register */

#define INTR_M4_SRAM_EN_M4        (1 << 0)
#define INTR_UART_EN_M4           (1 << 1)
#define INTR_TIMER_EN_M4          (1 << 2)
#define INTR_WDT_EN_M4            (1 << 3)
#define INTR_WDT_RST_EN_M4        (1 << 4)
#define INTR_TIMEOUT_EN_M4        (1 << 5)
#define INTR_FPU_EN_M4            (1 << 6)
#define INTR_PKFB_EN_M4           (1 << 7)
#define INTR_I2S_EN_M4            (1 << 8)
#define INTR_AUD_EN_M4            (1 << 9)
#define INTR_SPI_MS_EN_M4         (1 << 10)
#define INTR_CFG_DMA_EN_M4        (1 << 11)
#define INTR_PMU_TMR_EN_M4        (1 << 12)
#define INTR_ADC_EN_M4            (1 << 13)
#define INTR_RTC_EN_M4            (1 << 14)
#define INTR_RST_EN_M4            (1 << 15)
#define INTR_FFE0_EN_M4           (1 << 16)
#define INTR_WDT_FFE_EN_M4        (1 << 17)
#define INTR_APBOOT_EN_M4         (1 << 18)
#define INTR_LD030_PG_EN_M4       (1 << 19)
#define INTR_LD050_PG_EN_M4       (1 << 20)
#define INTR_SRAM_128KB_EN_M4     (1 << 21)
#define INTR_LPSD_VOICE_EN_M4     (1 << 22)
#define INTR_DMIC_VOICE_EN_M4     (1 << 23)

#endif /* __ARCH_ARM_SRC_EOSS3_HARDWARE_EOSS3_INTR_H */
