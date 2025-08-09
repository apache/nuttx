/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx95/imx95_clock.h
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_CLOCK_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef enum
{
    EXT                    = 0,
    OSC_32K                = 1,
    OSC_24M                = 2,
    FRO                    = 3,
    SYS_PLL1_CTL           = 4,
    SYS_PLL1_DFS0_CTL      = 5,
    SYS_PLL1_DFS0          = 6,
    SYS_PLL1_DFS0_DIV2     = 7,
    SYS_PLL1_DFS1_CTL      = 8,
    SYS_PLL1_DFS1          = 9,
    SYS_PLL1_DFS1_DIV2     = 10,
    SYS_PLL1_DFS2_CTL      = 11,
    SYS_PLL1_DFS2          = 12,
    SYS_PLL1_DFS2_DIV2     = 13,
    AUDIO_PLL1_CTL         = 14,
    AUDIO_PLL1             = 15,
    AUDIO_PLL2_CTL         = 16,
    AUDIO_PLL2             = 17,
    VIDEO_PLL1_CTL         = 18,
    VIDEO_PLL1             = 19,
    VIDEO_PLL2_CTL         = 20,
    VIDEO_PLL2             = 21,
    VIDEO_PLL3_CTL         = 22,
    VIDEO_PLL3             = 23,
    ARM_PLL_CTL            = 24,
    ARM_PLL_DFS0_CTL       = 25,
    ARM_PLL_DFS0           = 26,
    ARM_PLL_DFS1_CTL       = 27,
    ARM_PLL_DFS1           = 28,
    ARM_PLL_DFS2_CTL       = 29,
    ARM_PLL_DFS2           = 30,
    ARM_PLL_DFS3_CTL       = 31,
    ARM_PLL_DFS3           = 32,
    DRAM_PLL_CTL           = 33,
    DRAM_PLL               = 34,
    HSIO_PLL_CTL           = 35,
    HSIO_PLL               = 36,
    LDB_PLL_CTL            = 37,
    LDB_PLL                = 38,
} clock_id_e;

#define CCM_CR_COUNT       122
#define ROOT_CLOCK_OFFSET  41

typedef uint32_t clock_config_t;

#define CLOCK_DIV_SHIFT    (16)
#define CLOCK_DIV_MASK     (0xff << CLOCK_DIV_SHIFT)
#define CLOCK_DIV(n)       (((n) << CLOCK_DIV_SHIFT) & CLOCK_DIV_MASK)
#define GET_DIV(n)         (((n) & CLOCK_DIV_MASK) >> CLOCK_DIV_SHIFT)

#define ROOT_SHIFT         (8)
#define ROOT_MASK          (0x7f << ROOT_SHIFT)
#define ROOT(n)            (((n) << ROOT_SHIFT) & ROOT_MASK)
#define GET_ROOT(n)        (((n) & ROOT_MASK) >> ROOT_SHIFT)

#define ID_SHIFT           (3)
#define ID_MASK            (0x1f << ID_SHIFT)
#define ID(n)              (((n) << ID_SHIFT) & ID_MASK)
#define GET_ID(n)          (((n) & ID_MASK) >> ID_SHIFT)

#define MUX_SHIFT          (0)
#define MUX_MASK           (0x07 << MUX_SHIFT)
#define MUX(n)             (((n) << MUX_SHIFT) & MUX_MASK)

/* ADC_gROOT */
#define ADC_OSC_24M        ROOT(0U) | MUX(0U) | ID(OSC_24M)
#define ADC_DFS0_DIV2      ROOT(0U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define ADC_DFS1_DIV2      ROOT(0U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define ADC_FRO            ROOT(0U) | MUX(3U) | ID(FRO)

/* TMU_gROOT */
#define TMU_OSC_24M        ROOT(1U) | MUX(0U) | ID(OSC_24M)
#define TMU_DFS0_DIV2      ROOT(1U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define TMU_DFS1_DIV2      ROOT(1U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define TMU_FRO            ROOT(1U) | MUX(3U) | ID(FRO)

/* BUS_AON_gROOT */
#define BUS_AON_OSC_24M    ROOT(2U) | MUX(0U) | ID(OSC_24M)
#define BUS_AON_DFS0_DIV2  ROOT(2U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define BUS_AON_DFS1_DIV2  ROOT(2U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define BUS_AON_FRO        ROOT(2U) | MUX(3U) | ID(FRO)

/* CAN1_gROOT */
#define CAN1_OSC_24M       ROOT(3U) | MUX(0U) | ID(OSC_24M)
#define CAN1_DFS0_DIV2     ROOT(3U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAN1_DFS1_DIV2     ROOT(3U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAN1_FRO           ROOT(3U) | MUX(3U) | ID(FRO)

/* I3C1_gROOT */
#define I3C1_OSC_24M       ROOT(4U) | MUX(0U) | ID(OSC_24M)
#define I3C1_DFS0_DIV2     ROOT(4U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define I3C1_DFS1_DIV2     ROOT(4U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define I3C1_FRO           ROOT(4U) | MUX(3U) | ID(FRO)

/* I3C1_SLOW_gROOT */
#define I3C1_SL_OSC_24M    ROOT(5U) | MUX(0U) | ID(OSC_24M)
#define I3C1_SL_DFS0_DIV2  ROOT(5U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define I3C1_SL_DFS1_DIV2  ROOT(5U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define I3C1_SL_FRO        ROOT(5U) | MUX(3U) | ID(FRO)

/* LPI2C1_gROOT */
#define LPI2C1_OSC_24M     ROOT(6U) | MUX(0U) | ID(OSC_24M)
#define LPI2C1_DFS0_DIV2   ROOT(6U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C1_DFS1_DIV2   ROOT(6U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C1_FRO         ROOT(6U) | MUX(3U) | ID(FRO)

/* LPI2C2_gROOT */
#define LPI2C2_OSC_24M     ROOT(7U) | MUX(0U) | ID(OSC_24M)
#define LPI2C2_DFS0_DIV2   ROOT(7U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C2_DFS1_DIV2   ROOT(7U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C2_FRO         ROOT(7U) | MUX(3U) | ID(FRO)

/* LPSPI1_gROOT */
#define LPSPI1_OSC_24M     ROOT(8U) | MUX(0U) | ID(OSC_24M)
#define LPSPI1_DFS0_DIV2   ROOT(8U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI1_DFS1_DIV2   ROOT(8U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI1_FRO         ROOT(8U) | MUX(3U) | ID(FRO)

/* LPSPI2_gROOT */
#define LPSPI2_OSC_24M     ROOT(9U) | MUX(0U) | ID(OSC_24M)
#define LPSPI2_DFS0_DIV2   ROOT(9U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI2_DFS1_DIV2   ROOT(9U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI2_FRO         ROOT(9U) | MUX(3U) | ID(FRO)

/* LPTMR1_gROOT */
#define LPTMR1_OSC_24M     ROOT(10U) | MUX(0U) | ID(OSC_24M)
#define LPTMR1_DFS0_DIV2   ROOT(10U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPTMR1_DFS1_DIV2   ROOT(10U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPTMR1_FRO         ROOT(10U) | MUX(3U) | ID(FRO)

/* LPUART1_gROOT */
#define LPUART1_OSC_24M    ROOT(11U) | MUX(0U) | ID(OSC_24M)
#define LPUART1_DFS0_DIV2  ROOT(11U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART1_DFS1_DIV2  ROOT(11U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART1_FRO        ROOT(11U) | MUX(3U) | ID(FRO)

/* LPUART2_gROOT */
#define LPUART2_OSC_24M    ROOT(12U) | MUX(0U) | ID(OSC_24M)
#define LPUART2_DFS0_DIV2  ROOT(12U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART2_DFS1_DIV2  ROOT(12U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART2_FRO        ROOT(12U) | MUX(3U) | ID(FRO)

/* M33_gROOT */
#define M33_OSC_24M        ROOT(13U) | MUX(0U) | ID(OSC_24M)
#define M33_DFS0           ROOT(13U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define M33_DFS1_DIV2      ROOT(13U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define M33_FRO            ROOT(13U) | MUX(3U) | ID(FRO)

/* M33_SYSTICK_gROOT */
#define M33_TICK_OSC_24M   ROOT(14U) | MUX(0U) | ID(OSC_24M)
#define M33_TICK_DFS0_DIV2 ROOT(14U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define M33_TICK_DFS1_DIV2 ROOT(14U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define M33_TICK_FRO       ROOT(14U) | MUX(3U) | ID(FRO)

/* MQS1_gROOT */
#define MQS1_OSC_24M       ROOT(15U) | MUX(0U) | ID(OSC_24M)
#define MQS1_AUDIO_PLL1    ROOT(15U) | MUX(1U) | ID(AUDIO_PLL1)
#define MQS1_AUDIO_PLL2    ROOT(15U) | MUX(2U) | ID(AUDIO_PLL2)
#define MQS1_EXT           ROOT(15U) | MUX(3U) | ID(EXT)

/* PDM_gROOT */
#define PDM_OSC_24M        ROOT(16U) | MUX(0U) | ID(OSC_24M)
#define PDM_AUDIO_PLL1     ROOT(16U) | MUX(1U) | ID(AUDIO_PLL1)
#define PDM_AUDIO_PLL2     ROOT(16U) | MUX(2U) | ID(AUDIO_PLL2)
#define PDM_EXT            ROOT(16U) | MUX(3U) | ID(EXT)

/* SAI1_gROOT */
#define SAI1_OSC_24M       ROOT(17U) | MUX(0U) | ID(OSC_24M)
#define SAI1_AUDIO_PLL1    ROOT(17U) | MUX(1U) | ID(AUDIO_PLL1)
#define SAI1_AUDIO_PLL2    ROOT(17U) | MUX(2U) | ID(AUDIO_PLL2)
#define SAI1_EXT           ROOT(17U) | MUX(3U) | ID(EXT)

/* SENTINEL_gROOT */
#define SENTINEL_OSC_24M   ROOT(18U) | MUX(0U) | ID(OSC_24M)
#define SENTINEL_DFS0      ROOT(18U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define SENTINEL_DFS1_DIV2 ROOT(18U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define SENTINEL_FRO       ROOT(18U) | MUX(3U) | ID(FRO)

/* TPM2_gROOT */
#define TPM2_OSC_24M       ROOT(19U) | MUX(0U) | ID(OSC_24M)
#define TPM2_DFS0          ROOT(19U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define TPM2_AUDIO_PLL1    ROOT(19U) | MUX(2U) | ID(AUDIO_PLL1)
#define TPM2_EXT           ROOT(19U) | MUX(3U) | ID(EXT)

/* TSTMR1_gROOT */
#define TSTMR1_OSC_24M     ROOT(20U) | MUX(0U) | ID(OSC_24M)
#define TSTMR1_DFS0_DIV2   ROOT(20U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define TSTMR1_DFS1_DIV2   ROOT(20U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define TSTMR1_FRO         ROOT(20U) | MUX(3U) | ID(FRO)

/* CAM_APB_gROOT */
#define CAM_APB_OSC_24M    ROOT(21U) | MUX(0U) | ID(OSC_24M)
#define CAM_APB_DFS0_DIV2  ROOT(21U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAM_APB_DFS1_DIV2  ROOT(21U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAM_APB_FRO        ROOT(21U) | MUX(3U) | ID(FRO)

/* CAM_AXI_gROOT */
#define CAM_AXI_OSC_24M    ROOT(22U) | MUX(0U) | ID(OSC_24M)
#define CAM_AXI_DFS0       ROOT(22U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CAM_AXI_DFS1       ROOT(22U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define CAM_AXI_DFS2       ROOT(22U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* CAM_CM0_gROOT */
#define CAM_CM0_OSC_24M    ROOT(23U) | MUX(0U) | ID(OSC_24M)
#define CAM_CM0_DFS0       ROOT(23U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CAM_CM0_DFS1       ROOT(23U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define CAM_CM0_DFS2       ROOT(23U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* CAM_ISI_gROOT */
#define CAM_ISI_OSC_24M    ROOT(24U) | MUX(0U) | ID(OSC_24M)
#define CAM_ISI_DFS0       ROOT(24U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CAM_ISI_DFS1       ROOT(24U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define CAM_ISI_DFS2       ROOT(24U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* MIPI_PHY_CFG_gROOT */
#define MIPI_PHY_OSC_24M    ROOT(25U) | MUX(0U) | ID(OSC_24M)
#define MIPI_PHY_AUDIO_PLL1 ROOT(25U) | MUX(1U) | ID(AUDIO_PLL1)
#define MIPI_PHY_VIDEO_PLL1 ROOT(25U) | MUX(2U) | ID(VIDEO_PLL1)

/* MIPI_PHY_PLL_BYPASS_gROOT */
#define MIPI_PHY_BPS_OSC_24M    ROOT(26U) | MUX(0U) | ID(OSC_24M)
#define MIPI_PHY_BPS_AUDIO_PLL1 ROOT(26U) | MUX(1U) | ID(AUDIO_PLL1)
#define MIPI_PHY_BPS_VIDEO_PLL1 ROOT(26U) | MUX(2U) | ID(VIDEO_PLL1)

/* MIPI_PHY_PLL_REF_gROOT */
#define MIPI_PHY_REF_OSC_24M    ROOT(27U) | MUX(0U) | ID(OSC_24M)
#define MIPI_PHY_REF_AUDIO_PLL1 ROOT(27U) | MUX(1U) | ID(AUDIO_PLL1)
#define MIPI_PHY_REF_VIDEO_PLL1 ROOT(27U) | MUX(2U) | ID(VIDEO_PLL1)

/* MIPI_TEST_BYTE_gROOT */
#define MIPI_TEST_OSC_24M       ROOT(28U) | MUX(0U) | ID(OSC_24M)
#define MIPI_TEST_AUDIO_PLL1    ROOT(28U) | MUX(1U) | ID(AUDIO_PLL1)
#define MIPI_TEST_VIDEO_PLL1    ROOT(28U) | MUX(2U) | ID(VIDEO_PLL1)

/* ARM_A55_gROOT */
#define ARM_A55_OSC_24M         ROOT(29U) | MUX(0U) | ID(OSC_24M)
#define ARM_A55_DFS0            ROOT(29U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ARM_A55_DFS1            ROOT(29U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define ARM_A55_DFS2            ROOT(29U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* ARM_A55_MTR_BUS_gROOT */
#define ARM_A55_MTR_OSC_24M     ROOT(30U) | MUX(0U) | ID(OSC_24M)
#define ARM_A55_MTR_DFS0_DIV2   ROOT(30U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define ARM_A55_MTR_DFS1_DIV2   ROOT(30U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define ARM_A55_MTR_FRO         ROOT(30U) | MUX(3U) | ID(FRO)

/* ARM_A55_PERIPH_gROOT */
#define ARM_A55_PERIPH_OSC_24M  ROOT(31U) | MUX(0U) | ID(OSC_24M)
#define ARM_A55_PERIPH_DFS0     ROOT(31U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ARM_A55_PERIPH_DFS1     ROOT(31U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define ARM_A55_PERIPH_DFS2     ROOT(31U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DRAM_ALT_gROOT */
#define DRAM_ALT_OSC_24M        ROOT(32U) | MUX(0U) | ID(OSC_24M)
#define DRAM_ALT_DFS0           ROOT(32U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define DRAM_ALT_DFS1           ROOT(32U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define DRAM_ALT_DFS2           ROOT(32U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DRAM_APB_gROOT */
#define DRAM_APB_OSC_24M       ROOT(33U) | MUX(0U) | ID(OSC_24M)
#define DRAM_APB_DFS0_DIV2     ROOT(33U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define DRAM_APB_DFS1_DIV2     ROOT(33U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define DRAM_APB_FRO           ROOT(33U) | MUX(3U) | ID(FRO)

/* DISP_APB_gROOT */
#define DISP_APB_OSC_24M       ROOT(34U) | MUX(0U) | ID(OSC_24M)
#define DISP_APB_DFS0_DIV2     ROOT(34U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define DISP_APB_DFS1_DIV2     ROOT(34U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define DISP_APB_FRO           ROOT(34U) | MUX(3U) | ID(FRO)

/* DISP_AXI_gROOT */
#define DISP_AXI_OSC_24M       ROOT(35U) | MUX(0U) | ID(OSC_24M)
#define DISP_AXI_DFS0          ROOT(35U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define DISP_AXI_DFS1          ROOT(35U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define DISP_AXI_DFS2          ROOT(35U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DISP_DP_gROOT */
#define DISP_DP_OSC_24M        ROOT(36U) | MUX(0U) | ID(OSC_24M)
#define DISP_DP_DFS0           ROOT(36U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define DISP_DP_DFS1           ROOT(36U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define DISP_DP_DFS2           ROOT(36U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DISP_OCRAM_gROOT */
#define DISP_OCRAM_OSC_24M     ROOT(37U) | MUX(0U) | ID(OSC_24M)
#define DISP_OCRAM_DFS0        ROOT(37U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define DISP_OCRAM_DFS1        ROOT(37U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define DISP_OCRAM_DFS2        ROOT(37U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DISP_USB31_gROOT */
#define DISP_USB31_OSC_24M     ROOT(38U) | MUX(0U) | ID(OSC_24M)
#define DISP_USB31_DFS0        ROOT(38U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define DISP_USB31_DFS1        ROOT(38U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define DISP_USB31_DFS2        ROOT(38U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* DISP1_PIX_gROOT */
#define DISP1_PIX_OSC_24M      ROOT(39U) | MUX(0U) | ID(OSC_24M)
#define DISP1_PIX_AUDIO_PLL1   ROOT(39U) | MUX(1U) | ID(AUDIO_PLL1)
#define DISP1_PIX_VIDEO_PLL1   ROOT(39U) | MUX(2U) | ID(VIDEO_PLL1)

/* GPU_APB_gROOT */
#define GPU_APB_OSC_24M        ROOT(42U) | MUX(0U) | ID(OSC_24M)
#define GPU_APB_DFS0_DIV2      ROOT(42U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define GPU_APB_DFS1_DIV2      ROOT(42U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define GPU_APB_FRO            ROOT(42U) | MUX(3U) | ID(FRO)

/* GPU_gROOT */
#define GPU_OSC_24M            ROOT(43U) | MUX(0U) | ID(OSC_24M)
#define GPU_DFS0               ROOT(43U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define GPU_DFS1               ROOT(43U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define GPU_DFS2               ROOT(43U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_ACSCAN_480M_gROOT */
#define HSIO_ACSCAN_480_OSC_24M  ROOT(44U) | MUX(0U) | ID(OSC_24M)
#define HSIO_ACSCAN_480_AUDIO    ROOT(44U) | MUX(1U) | ID(AUDIO_PLL1)
#define HSIO_ACSCAN_480_VIDEO    ROOT(44U) | MUX(2U) | ID(VIDEO_PLL1)
#define HSIO_ACSCAN_480_DFS2     ROOT(44U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_ACSCAN_80M_gROOT */
#define HSIO_ACSCAN_80_OSC_24M   ROOT(45U) | MUX(0U) | ID(OSC_24M)
#define HSIO_ACSCAN_80_DFS0_DIV2 ROOT(45U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define HSIO_ACSCAN_80_DFS1_DIV2 ROOT(45U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define HSIO_ACSCAN_80_FRO       ROOT(45U) | MUX(3U) | ID(FRO)

/* HSIO_gROOT */
#define HSIO_OSC_24M             ROOT(46U) | MUX(0U) | ID(OSC_24M)
#define HSIO_DFS0                ROOT(46U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define HSIO_DFS1                ROOT(46U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define HSIO_DFS2                ROOT(46U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_PCIE_AUX_gROOT */
#define HSIO_PCIE_AUX_OSC_24M    ROOT(47U) | MUX(0U) | ID(OSC_24M)
#define HSIO_PCIE_AUX_DFS0_DIV2  ROOT(47U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define HSIO_PCIE_AUX_DFS1_DIV2  ROOT(47U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define HSIO_PCIE_AUX_FRO        ROOT(47U) | MUX(3U) | ID(FRO)

/* HSIO_PCIE_TEST_160M_gROOT */
#define HSIO_PCIE_160_OSC_24M    ROOT(48U) | MUX(0U) | ID(OSC_24M)
#define HSIO_PCIE_160_DFS0       ROOT(48U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define HSIO_PCIE_160_DFS1       ROOT(48U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define HSIO_PCIE_160_DFS2       ROOT(48U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_PCIE_TEST_400M_gROOT */
#define HSIO_PCIE_400_OSC_24M    ROOT(49U) | MUX(0U) | ID(OSC_24M)
#define HSIO_PCIE_400_DFS0       ROOT(49U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define HSIO_PCIE_400_DFS1       ROOT(49U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define HSIO_PCIE_400_DFS2       ROOT(49U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_PCIE_TEST_500M_gROOT */
#define HSIO_PCIE_500_OSC_24M    ROOT(50U) | MUX(0U) | ID(OSC_24M)
#define HSIO_PCIE_500_DFS0       ROOT(50U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define HSIO_PCIE_500_DFS1       ROOT(50U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define HSIO_PCIE_500_DFS2       ROOT(50U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* HSIO_USB_TEST_50M_gROOT */
#define HSIO_USB_50_OSC_24M      ROOT(51U) | MUX(0U) | ID(OSC_24M)
#define HSIO_USB_50_DFS0_DIV2    ROOT(51U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define HSIO_USB_50_DFS1_DIV2    ROOT(51U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define HSIO_USB_50_FRO          ROOT(51U) | MUX(3U) | ID(FRO)

/* HSIO_USB_TEST_60M_gROOT */
#define HSIO_USB_60_OSC_24M      ROOT(52U) | MUX(0U) | ID(OSC_24M)
#define HSIO_USB_60_DFS0_DIV2    ROOT(52U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define HSIO_USB_60_DFS1_DIV2    ROOT(52U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define HSIO_USB_60_FRO          ROOT(52U) | MUX(3U) | ID(FRO)

/* BUS_M7_gROOT */
#define BUS_M7_OSC_24M           ROOT(53U) | MUX(0U) | ID(OSC_24M)
#define BUS_M7_DFS0_DIV2         ROOT(53U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define BUS_M7_DFS1_DIV2         ROOT(53U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define BUS_M7_FRO               ROOT(53U) | MUX(3U) | ID(FRO)

/* M7_gROOT */
#define M7_OSC_24M               ROOT(54U) | MUX(0U) | ID(OSC_24M)
#define M7_DFS0                  ROOT(54U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define M7_DFS1                  ROOT(54U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define M7_DFS2                  ROOT(54U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* M7_SYSTICK_gROOT */
#define M7_SYSTICK_OSC_24M       ROOT(55U) | MUX(0U) | ID(OSC_24M)
#define M7_SYSTICK_DFS0_DIV2     ROOT(55U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define M7_SYSTICK_DFS1_DIV2     ROOT(55U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define M7_SYSTICK_FRO           ROOT(55U) | MUX(3U) | ID(FRO)

/* BUS_NETCMIX_gROOT */
#define BUS_NETCMIX_OSC_24M      ROOT(56U) | MUX(0U) | ID(OSC_24M)
#define BUS_NETCMIX_DFS0_DIV2    ROOT(56U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define BUS_NETCMIX_DFS1_DIV2    ROOT(56U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define BUS_NETCMIX_FRO          ROOT(56U) | MUX(3U) | ID(FRO)

/* ENET_gROOT */
#define ENET_OSC_24M             ROOT(57U) | MUX(0U) | ID(OSC_24M)
#define ENET_DFS0                ROOT(57U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ENET_DFS1                ROOT(57U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define ENET_DFS2                ROOT(57U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* ENET_PHY_TEST_200M_gROOT */
#define ENET_PHY_200_OSC_24M     ROOT(58U) | MUX(0U) | ID(OSC_24M)
#define ENET_PHY_200_DFS0        ROOT(58U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ENET_PHY_200_DFS1_DIV2   ROOT(58U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define ENET_PHY_200_FRO         ROOT(58U) | MUX(3U) | ID(FRO)

/* ENET_PHY_TEST_500M_gROOT */
#define ENET_PHY_500_OSC_24M    ROOT(59U) | MUX(0U) | ID(OSC_24M)
#define ENET_PHY_500_DFS0       ROOT(59U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ENET_PHY_500_DFS1       ROOT(59U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define ENET_PHY_500_DFS2       ROOT(59U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* ENET_PHY_TEST_667M_gROOT */
#define ENET_PHY_667_OSC_24M    ROOT(60U) | MUX(0U) | ID(OSC_24M)
#define ENET_PHY_667_DFS0       ROOT(60U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ENET_PHY_667_DFS1       ROOT(60U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define ENET_PHY_667_DFS2       ROOT(60U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* ENET_REF_gROOT */
#define ENET_REF_OSC_24M        ROOT(61U) | MUX(0U) | ID(OSC_24M)
#define ENET_REF_DFS0           ROOT(61U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define ENET_REF_DFS1_DIV2      ROOT(61U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define ENET_REF_FRO            ROOT(61U) | MUX(3U) | ID(FRO)

/* ENET_TIMER1_gROOT */
#define ENET_TIMER1_OSC_24M     ROOT(62U) | MUX(0U) | ID(OSC_24M)
#define ENET_TIMER1_DFS0_DIV2   ROOT(62U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define ENET_TIMER1_DFS1_DIV2   ROOT(62U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define ENET_TIMER1_FRO         ROOT(62U) | MUX(3U) | ID(FRO)

/* MQS2_gROOT */
#define MQS2_OSC_24M            ROOT(63U) | MUX(0U) | ID(OSC_24M)
#define MQS2_AUDIO_PLL1         ROOT(63U) | MUX(1U) | ID(AUDIO_PLL1)
#define MQS2_AUDIO_PLL2         ROOT(63U) | MUX(2U) | ID(AUDIO_PLL2)
#define MQS2_EXT                ROOT(63U) | MUX(3U) | ID(EXT)

/* SAI2_gROOT */
#define SAI2_OSC_24M            ROOT(64U) | MUX(0U) | ID(OSC_24M)
#define SAI2_AUDIO_PLL1         ROOT(64U) | MUX(1U) | ID(AUDIO_PLL1)
#define SAI2_AUDIO_PLL2         ROOT(64U) | MUX(2U) | ID(AUDIO_PLL2)
#define SAI2_EXT                ROOT(64U) | MUX(3U) | ID(EXT)

/* NOC_APB_gROOT */
#define NOC_APB_OSC_24M         ROOT(65U) | MUX(0U) | ID(OSC_24M)
#define NOC_APB_DFS0_DIV2       ROOT(65U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define NOC_APB_DFS1_DIV2       ROOT(65U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define NOC_APB_FRO             ROOT(65U) | MUX(3U) | ID(FRO)

/* NOC_gROOT */
#define NOC_OSC_24M             ROOT(66U) | MUX(0U) | ID(OSC_24M)
#define NOC_DFS0                ROOT(66U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define NOC_DFS1                ROOT(66U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define NOC_DFS2                ROOT(66U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* NPU_APB_gROOT */
#define NPU_APB_OSC_24M         ROOT(67U) | MUX(0U) | ID(OSC_24M)
#define NPU_APB_DFS0_DIV2       ROOT(67U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define NPU_APB_DFS1_DIV2       ROOT(67U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define NPU_APB_FRO             ROOT(67U) | MUX(3U) | ID(FRO)

/* NPU_gROOT */
#define NPU_OSC_24M             ROOT(68U) | MUX(0U) | ID(OSC_24M)
#define NPU_DFS0                ROOT(68U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define NPU_DFS1                ROOT(68U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define NPU_DFS2                ROOT(68U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* CCM_CKO1_gROOT */
#define CCM_CKO1_OSC_24M        ROOT(69U) | MUX(0U) | ID(OSC_24M)
#define CCM_CKO1_DFS0           ROOT(69U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CCM_CKO1_OSC_32K        ROOT(69U) | MUX(2U) | ID(OSC_32K)
#define CCM_CKO1_AUDIO_PLL1     ROOT(69U) | MUX(3U) | ID(AUDIO_PLL1)

/* CCM_CKO2_gROOT */
#define CCM_CKO2_OSC_24M        ROOT(70U) | MUX(0U) | ID(OSC_24M)
#define CCM_CKO2_DFS0           ROOT(70U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CCM_CKO2_OSC_32K        ROOT(70U) | MUX(2U) | ID(OSC_32K)
#define CCM_CKO2_VIDEO_PLL1     ROOT(70U) | MUX(3U) | ID(VIDEO_PLL1)

/* CCM_CKO3_gROOT */
#define CCM_CKO3_OSC_24M        ROOT(71U) | MUX(0U) | ID(OSC_24M)
#define CCM_CKO3_DFS0           ROOT(71U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CCM_CKO3_OSC_32K        ROOT(71U) | MUX(2U) | ID(OSC_32K)
#define CCM_CKO3_AUDIO_PLL2     ROOT(71U) | MUX(3U) | ID(AUDIO_PLL2)

/* CCM_CKO4_gROOT */
#define CCM_CKO4_OSC_24M        ROOT(72U) | MUX(0U) | ID(OSC_24M)
#define CCM_CKO4_DFS0           ROOT(72U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define CCM_CKO4_OSC_32K        ROOT(72U) | MUX(2U) | ID(OSC_32K)
#define CCM_CKO4_VIDEO_PLL1     ROOT(72U) | MUX(3U) | ID(VIDEO_PLL1)

/* VPU_APB_gROOT */
#define VPU_APB_OSC_24M         ROOT(73U) | MUX(0U) | ID(OSC_24M)
#define VPU_APB_DFS0_DIV2       ROOT(73U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define VPU_APB_DFS1_DIV2       ROOT(73U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define VPU_APB_FRO             ROOT(73U) | MUX(3U) | ID(FRO)

/* VPU_gROOT */
#define VPU_OSC_24M             ROOT(74U) | MUX(0U) | ID(OSC_24M)
#define VPU_DFS0                ROOT(74U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define VPU_DFS1                ROOT(74U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define VPU_DFS2                ROOT(74U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* VPU_DSP_gROOT */
#define VPU_DSP_OSC_24M         ROOT(75U) | MUX(0U) | ID(OSC_24M)
#define VPU_DSP_DFS0            ROOT(75U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define VPU_DSP_DFS1            ROOT(75U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define VPU_DSP_DFS2            ROOT(75U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* VPU_JPEG_gROOT */
#define VPU_JPEG_OSC_24M        ROOT(76U) | MUX(0U) | ID(OSC_24M)
#define VPU_JPEG_DFS0           ROOT(76U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define VPU_JPEG_DFS1           ROOT(76U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define VPU_JPEG_DFS2           ROOT(76U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* AUDIO_XCVR_gROOT */
#define AUDIO_XCVR_OSC_24M      ROOT(77U) | MUX(0U) | ID(OSC_24M)
#define AUDIO_XCVR_DFS0         ROOT(77U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define AUDIO_XCVR_DFS1_DIV2    ROOT(77U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define AUDIO_XCVR_FRO          ROOT(77U) | MUX(3U) | ID(FRO)

/* BUS_WAKEUP_gROOT */
#define BUS_WAKEUP_OSC_24M      ROOT(78U) | MUX(0U) | ID(OSC_24M)
#define BUS_WAKEUP_DFS0_DIV2    ROOT(78U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define BUS_WAKEUP_DFS1_DIV2    ROOT(78U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define BUS_WAKEUP_FRO          ROOT(78U) | MUX(3U) | ID(FRO)

/* CAN2_gROOT */
#define CAN2_OSC_24M            ROOT(79U) | MUX(0U) | ID(OSC_24M)
#define CAN2_DFS0_DIV2          ROOT(79U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAN2_DFS1_DIV2          ROOT(79U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAN2_FRO                ROOT(79U) | MUX(3U) | ID(FRO)

/* CAN3_gROOT */
#define CAN3_OSC_24M            ROOT(80U) | MUX(0U) | ID(OSC_24M)
#define CAN3_DFS0_DIV2          ROOT(80U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAN3_DFS1_DIV2          ROOT(80U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAN3_FRO                ROOT(80U) | MUX(3U) | ID(FRO)

/* CAN4_gROOT */
#define CAN4_OSC_24M            ROOT(81U) | MUX(0U) | ID(OSC_24M)
#define CAN4_DFS0_DIV2          ROOT(81U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAN4_DFS1_DIV2          ROOT(81U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAN4_FRO                ROOT(81U) | MUX(3U) | ID(FRO)

/* CAN5_gROOT */
#define CAN5_OSC_24M            ROOT(82U) | MUX(0U) | ID(OSC_24M)
#define CAN5_DFS0_DIV2          ROOT(82U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define CAN5_DFS1_DIV2          ROOT(82U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define CAN5_FRO                ROOT(82U) | MUX(3U) | ID(FRO)

/* FLEXIO1_gROOT */
#define FLEXIO1_OSC_24M         ROOT(83U) | MUX(0U) | ID(OSC_24M)
#define FLEXIO1_DFS0_DIV2       ROOT(83U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define FLEXIO1_DFS1_DIV2       ROOT(83U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define FLEXIO1_FRO             ROOT(83U) | MUX(3U) | ID(FRO)

/* FLEXIO2_gROOT */
#define FLEXIO2_OSC_24M         ROOT(84U) | MUX(0U) | ID(OSC_24M)
#define FLEXIO2_DFS0_DIV2       ROOT(84U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define FLEXIO2_DFS1_DIV2       ROOT(84U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define FLEXIO2_FRO             ROOT(84U) | MUX(3U) | ID(FRO)

/* FLEXSPI1_gROOT */
#define FLEXSPI1_OSC_24M        ROOT(85U) | MUX(0U) | ID(OSC_24M)
#define FLEXSPI1_DFS0           ROOT(85U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define FLEXSPI1_DFS1           ROOT(85U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define FLEXSPI1_DFS2           ROOT(85U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* I3C2_gROOT */
#define I3C2_OSC_24M            ROOT(86U) | MUX(0U) | ID(OSC_24M)
#define I3C2_DFS0_DIV2          ROOT(86U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define I3C2_DFS1_DIV2          ROOT(86U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define I3C2_FRO                ROOT(86U) | MUX(3U) | ID(FRO)

/* I3C2_SLOW_gROOT */
#define I3C2_SLOW_OSC_24M       ROOT(87U) | MUX(0U) | ID(OSC_24M)
#define I3C2_SLOW_DFS0_DIV2     ROOT(87U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define I3C2_SLOW_DFS1_DIV2     ROOT(87U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define I3C2_SLOW_FRO           ROOT(87U) | MUX(3U) | ID(FRO)

/* LPI2C3_gROOT */
#define LPI2C3_OSC_24M          ROOT(88U) | MUX(0U) | ID(OSC_24M)
#define LPI2C3_DFS0_DIV2        ROOT(88U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C3_DFS1_DIV2        ROOT(88U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C3_FRO              ROOT(88U) | MUX(3U) | ID(FRO)

/* LPI2C4_gROOT */
#define LPI2C4_OSC_24M          ROOT(89U) | MUX(0U) | ID(OSC_24M)
#define LPI2C4_DFS0_DIV2        ROOT(89U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C4_DFS1_DIV2        ROOT(89U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C4_FRO              ROOT(89U) | MUX(3U) | ID(FRO)

/* LPI2C5_gROOT */
#define LPI2C5_OSC_24M          ROOT(90U) | MUX(0U) | ID(OSC_24M)
#define LPI2C5_DFS0_DIV2        ROOT(90U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C5_DFS1_DIV2        ROOT(90U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C5_FRO              ROOT(90U) | MUX(3U) | ID(FRO)

/* LPI2C6_gROOT */
#define LPI2C6_OSC_24M          ROOT(91U) | MUX(0U) | ID(OSC_24M)
#define LPI2C6_DFS0_DIV2        ROOT(91U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C6_DFS1_DIV2        ROOT(91U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C6_FRO              ROOT(91U) | MUX(3U) | ID(FRO)

/* LPI2C7_gROOT */
#define LPI2C7_OSC_24M          ROOT(92U) | MUX(0U) | ID(OSC_24M)
#define LPI2C7_DFS0_DIV2        ROOT(92U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C7_DFS1_DIV2        ROOT(92U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C7_FRO              ROOT(92U) | MUX(3U) | ID(FRO)

/* LPI2C8_gROOT */
#define LPI2C8_OSC_24M          ROOT(93U) | MUX(0U) | ID(OSC_24M)
#define LPI2C8_DFS0_DIV2        ROOT(93U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPI2C8_DFS1_DIV2        ROOT(93U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPI2C8_FRO              ROOT(93U) | MUX(3U) | ID(FRO)

/* LPSPI3_gROOT */
#define LPSPI3_OSC_24M          ROOT(94U) | MUX(0U) | ID(OSC_24M)
#define LPSPI3_DFS0_DIV2        ROOT(94U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI3_DFS1_DIV2        ROOT(94U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI3_FRO              ROOT(94U) | MUX(3U) | ID(FRO)

/* LPSPI4_gROOT */
#define LPSPI4_OSC_24M          ROOT(95U) | MUX(0U) | ID(OSC_24M)
#define LPSPI4_DFS0_DIV2        ROOT(95U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI4_DFS1_DIV2        ROOT(95U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI4_FRO              ROOT(95U) | MUX(3U) | ID(FRO)

/* LPSPI5_gROOT */
#define LPSPI5_OSC_24M          ROOT(96U) | MUX(0U) | ID(OSC_24M)
#define LPSPI5_DFS0_DIV2        ROOT(96U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI5_DFS1_DIV2        ROOT(96U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI5_FRO              ROOT(96U) | MUX(3U) | ID(FRO)

/* LPSPI6_gROOT */
#define LPSPI6_OSC_24M          ROOT(97U) | MUX(0U) | ID(OSC_24M)
#define LPSPI6_DFS0_DIV2        ROOT(97U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI6_DFS1_DIV2        ROOT(97U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI6_FRO              ROOT(97U) | MUX(3U) | ID(FRO)

/* LPSPI7_gROOT */
#define LPSPI7_OSC_24M          ROOT(98U) | MUX(0U) | ID(OSC_24M)
#define LPSPI7_DFS0_DIV2        ROOT(98U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI7_DFS1_DIV2        ROOT(98U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI7_FRO              ROOT(98U) | MUX(3U) | ID(FRO)

/* LPSPI8_gROOT */
#define LPSPI8_OSC_24M          ROOT(99U) | MUX(0U) | ID(OSC_24M)
#define LPSPI8_DFS0_DIV2        ROOT(99U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPSPI8_DFS1_DIV2        ROOT(99U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPSPI8_FRO              ROOT(99U) | MUX(3U) | ID(FRO)

/* LPTMR2_gROOT */
#define LPTMR2_OSC_24M          ROOT(100U) | MUX(0U) | ID(OSC_24M)
#define LPTMR2_DFS0_DIV2        ROOT(100U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPTMR2_DFS1_DIV2        ROOT(100U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPTMR2_FRO              ROOT(100U) | MUX(3U) | ID(FRO)

/* LPUART3_gROOT */
#define LPUART3_OSC_24M         ROOT(101U) | MUX(0U) | ID(OSC_24M)
#define LPUART3_DFS0_DIV2       ROOT(101U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART3_DFS1_DIV2       ROOT(101U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART3_FRO             ROOT(101U) | MUX(3U) | ID(FRO)

/* LPUART4_gROOT */
#define LPUART4_OSC_24M         ROOT(102U) | MUX(0U) | ID(OSC_24M)
#define LPUART4_DFS0_DIV2       ROOT(102U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART4_DFS1_DIV2       ROOT(102U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART4_FRO             ROOT(102U) | MUX(3U) | ID(FRO)

/* LPUART5_gROOT */
#define LPUART5_OSC_24M         ROOT(103U) | MUX(0U) | ID(OSC_24M)
#define LPUART5_DFS0_DIV2       ROOT(103U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART5_DFS1_DIV2       ROOT(103U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART5_FRO             ROOT(103U) | MUX(3U) | ID(FRO)

/* LPUART6_gROOT */
#define LPUART6_OSC_24M         ROOT(104U) | MUX(0U) | ID(OSC_24M)
#define LPUART6_DFS0_DIV2       ROOT(104U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART6_DFS1_DIV2       ROOT(104U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART6_FRO             ROOT(104U) | MUX(3U) | ID(FRO)

/* LPUART7_gROOT */
#define LPUART7_OSC_24M         ROOT(105U) | MUX(0U) | ID(OSC_24M)
#define LPUART7_DFS0_DIV2       ROOT(105U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART7_DFS1_DIV2       ROOT(105U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART7_FRO             ROOT(105U) | MUX(3U) | ID(FRO)

/* LPUART8_gROOT */
#define LPUART8_OSC_24M         ROOT(106U) | MUX(0U) | ID(OSC_24M)
#define LPUART8_DFS0_DIV2       ROOT(106U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define LPUART8_DFS1_DIV2       ROOT(106U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define LPUART8_FRO             ROOT(106U) | MUX(3U) | ID(FRO)

/* SAI3_gROOT */
#define SAI3_OSC_24M            ROOT(107U) | MUX(0U) | ID(OSC_24M)
#define SAI3_AUDIO_PLL1         ROOT(107U) | MUX(1U) | ID(AUDIO_PLL1)
#define SAI3_AUDIO_PLL2         ROOT(107U) | MUX(2U) | ID(AUDIO_PLL2)
#define SAI3_EXT                ROOT(107U) | MUX(3U) | ID(EXT)

/* SAI4_gROOT */
#define SAI4_OSC_24M            ROOT(108U) | MUX(0U) | ID(OSC_24M)
#define SAI4_AUDIO_PLL1         ROOT(108U) | MUX(1U) | ID(AUDIO_PLL1)
#define SAI4_AUDIO_PLL2         ROOT(108U) | MUX(2U) | ID(AUDIO_PLL2)
#define SAI4_EXT                ROOT(108U) | MUX(3U) | ID(EXT)

/* SAI5_gROOT */
#define SAI5_OSC_24M            ROOT(109U) | MUX(0U) | ID(OSC_24M)
#define SAI5_AUDIO_PLL1         ROOT(109U) | MUX(1U) | ID(AUDIO_PLL1)
#define SAI5_AUDIO_PLL2         ROOT(109U) | MUX(2U) | ID(AUDIO_PLL2)
#define SAI5_EXT                ROOT(109U) | MUX(3U) | ID(EXT)

/* SPDIF_gROOT */
#define SPDIF_OSC_24M           ROOT(110U) | MUX(0U) | ID(OSC_24M)
#define SPDIF_AUDIO_PLL1        ROOT(110U) | MUX(1U) | ID(AUDIO_PLL1)
#define SPDIF_AUDIO_PLL2        ROOT(110U) | MUX(2U) | ID(AUDIO_PLL2)
#define SPDIF_EXT               ROOT(110U) | MUX(3U) | ID(EXT)

/* SWO_TRACE_gROOT */
#define SWO_TRACE_OSC_24M       ROOT(111U) | MUX(0U) | ID(OSC_24M)
#define SWO_TRACE_DFS0_DIV2     ROOT(111U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define SWO_TRACE_DFS1_DIV2     ROOT(111U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define SWO_TRACE_FRO           ROOT(111U) | MUX(3U) | ID(FRO)

/* TPM4_gROOT */
#define TPM4_OSC_24M            ROOT(112U) | MUX(0U) | ID(OSC_24M)
#define TPM4_DFS0               ROOT(112U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define TPM4_AUDIO_PLL1         ROOT(112U) | MUX(2U) | ID(AUDIO_PLL1)
#define TPM4_EXT                ROOT(112U) | MUX(3U) | ID(EXT)

/* TPM5_gROOT */
#define TPM5_OSC_24M            ROOT(113U) | MUX(0U) | ID(OSC_24M)
#define TPM5_DFS0               ROOT(113U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define TPM5_AUDIO_PLL1         ROOT(113U) | MUX(2U) | ID(AUDIO_PLL1)
#define TPM5_EXT                ROOT(113U) | MUX(3U) | ID(EXT)

/* TPM6_gROOT */
#define TPM6_OSC_24M            ROOT(114U) | MUX(0U) | ID(OSC_24M)
#define TPM6_DFS0               ROOT(114U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define TPM6_AUDIO_PLL1         ROOT(114U) | MUX(2U) | ID(AUDIO_PLL1)
#define TPM6_EXT                ROOT(114U) | MUX(3U) | ID(EXT)

/* TSTMR2_gROOT */
#define TSTMR2_OSC_24M          ROOT(115U) | MUX(0U) | ID(OSC_24M)
#define TSTMR2_DFS0_DIV2        ROOT(115U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define TSTMR2_DFS1_DIV2        ROOT(115U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define TSTMR2_FRO              ROOT(115U) | MUX(3U) | ID(FRO)

/* USB_PHY_BURUNIN_gROOT */
#define USB_PHY_OSC_24M         ROOT(116U) | MUX(0U) | ID(OSC_24M)
#define USB_PHY_DFS0_DIV2       ROOT(116U) | MUX(1U) | ID(SYS_PLL1_DFS0_DIV2)
#define USB_PHY_DFS1_DIV2       ROOT(116U) | MUX(2U) | ID(SYS_PLL1_DFS1_DIV2)
#define USB_PHY_FRO             ROOT(116U) | MUX(3U) | ID(FRO)

/* USDHC1_gROOT */
#define USDHC1_OSC_24M          ROOT(117U) | MUX(0U) | ID(OSC_24M)
#define USDHC1_DFS0             ROOT(117U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define USDHC1_DFS1             ROOT(117U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define USDHC1_DFS2             ROOT(117U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* USDHC2_gROOT */
#define USDHC2_OSC_24M          ROOT(118U) | MUX(0U) | ID(OSC_24M)
#define USDHC2_DFS0             ROOT(118U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define USDHC2_DFS1             ROOT(118U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define USDHC2_DFS2             ROOT(118U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* USDHC3_gROOT */
#define USDHC3_OSC_24M          ROOT(119U) | MUX(0U) | ID(OSC_24M)
#define USDHC3_DFS0             ROOT(119U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define USDHC3_DFS1             ROOT(119U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define USDHC3_DFS2             ROOT(119U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* V2X_PK_gROOT */
#define V2X_PK_OSC_24M          ROOT(120U) | MUX(0U) | ID(OSC_24M)
#define V2X_PK_DFS0             ROOT(120U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define V2X_PK_DFS1             ROOT(120U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define V2X_PK_DFS2             ROOT(120U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* WAKEUP_AXI_gROOT */
#define WAKEUP_AXI_OSC_24M      ROOT(121U) | MUX(0U) | ID(OSC_24M)
#define WAKEUP_AXI_DFS0         ROOT(121U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define WAKEUP_AXI_DFS1         ROOT(121U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define WAKEUP_AXI_DFS2         ROOT(121U) | MUX(3U) | ID(SYS_PLL1_DFS2)

/* XSPI_SLV_gROOT */
#define XSPI_SLV_OSC_24M        ROOT(122U) | MUX(0U) | ID(OSC_24M)
#define XSPI_SLV_DFS0           ROOT(122U) | MUX(1U) | ID(SYS_PLL1_DFS0)
#define XSPI_SLV_DFS1           ROOT(122U) | MUX(2U) | ID(SYS_PLL1_DFS1)
#define XSPI_SLV_DFS2           ROOT(122U) | MUX(3U) | ID(SYS_PLL1_DFS2)

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX93_IMX93_EDMA_H */
