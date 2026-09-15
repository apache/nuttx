/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_ccm.h
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

/* Copyright 2021-2025 NXP */

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register counts **********************************************************/

#define IMXRT_CCM_ROOT_COUNT             74
#define IMXRT_CCM_OBSERVE_COUNT          2
#define IMXRT_CCM_GPR_SHARED_COUNT       16
#define IMXRT_CCM_GPR_PRIVATE_COUNT      4
#define IMXRT_CCM_OSCPLL_COUNT           25
#define IMXRT_CCM_LPCG_COUNT             149

/* Register offsets *********************************************************/

#define IMXRT_CCM_ROOT_CTRL_OFFSET(n)     (0x0000u + ((n) << 7))
#define IMXRT_CCM_ROOT_SET_OFFSET(n)      (0x0004u + ((n) << 7))
#define IMXRT_CCM_ROOT_CLR_OFFSET(n)      (0x0008u + ((n) << 7))
#define IMXRT_CCM_ROOT_TOG_OFFSET(n)      (0x000cu + ((n) << 7))
#define IMXRT_CCM_ROOT_STAT_OFFSET(n)     (0x0020u + ((n) << 7))
#define IMXRT_CCM_ROOT_AUTH_OFFSET(n)     (0x0030u + ((n) << 7))

#define IMXRT_CCM_OBS_CTRL_OFFSET(n)      (0x4400u + ((n) << 7))
#define IMXRT_CCM_OBS_SET_OFFSET(n)       (0x4404u + ((n) << 7))
#define IMXRT_CCM_OBS_CLR_OFFSET(n)       (0x4408u + ((n) << 7))
#define IMXRT_CCM_OBS_TOG_OFFSET(n)       (0x440cu + ((n) << 7))
#define IMXRT_CCM_OBS_STAT_OFFSET(n)      (0x4420u + ((n) << 7))
#define IMXRT_CCM_OBS_AUTH_OFFSET(n)      (0x4430u + ((n) << 7))
#define IMXRT_CCM_OBS_FREQ_CUR_OFFSET(n)  (0x4440u + ((n) << 7))
#define IMXRT_CCM_OBS_FREQ_MIN_OFFSET(n)  (0x4444u + ((n) << 7))
#define IMXRT_CCM_OBS_FREQ_MAX_OFFSET(n)  (0x4448u + ((n) << 7))
#define IMXRT_CCM_OBS_PER_CUR_OFFSET(n)   (0x4450u + ((n) << 7))
#define IMXRT_CCM_OBS_PER_MIN_OFFSET(n)   (0x4454u + ((n) << 7))
#define IMXRT_CCM_OBS_PER_MAX_OFFSET(n)   (0x4458u + ((n) << 7))
#define IMXRT_CCM_OBS_HIGH_CUR_OFFSET(n)  (0x4460u + ((n) << 7))
#define IMXRT_CCM_OBS_HIGH_MIN_OFFSET(n)  (0x4464u + ((n) << 7))
#define IMXRT_CCM_OBS_HIGH_MAX_OFFSET(n)  (0x4468u + ((n) << 7))
#define IMXRT_CCM_OBS_LOW_CUR_OFFSET(n)   (0x4470u + ((n) << 7))
#define IMXRT_CCM_OBS_LOW_MIN_OFFSET(n)   (0x4474u + ((n) << 7))
#define IMXRT_CCM_OBS_LOW_MAX_OFFSET(n)   (0x4478u + ((n) << 7))

#define IMXRT_CCM_GPR_SHARED_OFFSET(n)    (0x4800u + ((n) << 5))
#define IMXRT_CCM_GPR_SHARED_SET_OFFSET(n) (0x4804u + ((n) << 5))
#define IMXRT_CCM_GPR_SHARED_CLR_OFFSET(n) (0x4808u + ((n) << 5))
#define IMXRT_CCM_GPR_SHARED_TOG_OFFSET(n) (0x480cu + ((n) << 5))
#define IMXRT_CCM_GPR_SHARED_AUTH_OFFSET(n) (0x4810u + ((n) << 5))
#define IMXRT_CCM_GPR_SHARED_STAT_OFFSET(n) (0x4a00u + ((n) << 2))

#define IMXRT_CCM_GPR_PRIVATE_OFFSET(n)   (0x4c00u + ((n) << 5))
#define IMXRT_CCM_GPR_PRIVATE_SET_OFFSET(n) (0x4c04u + ((n) << 5))
#define IMXRT_CCM_GPR_PRIVATE_CLR_OFFSET(n) (0x4c08u + ((n) << 5))
#define IMXRT_CCM_GPR_PRIVATE_TOG_OFFSET(n) (0x4c0cu + ((n) << 5))
#define IMXRT_CCM_GPR_PRIVATE_AUTH_OFFSET(n) (0x4c10u + ((n) << 5))

#define IMXRT_CCM_OSCPLL_DIR_OFFSET(n)    (0x5000u + ((n) << 6))
#define IMXRT_CCM_OSCPLL_LPM0_OFFSET(n)   (0x5010u + ((n) << 6))
#define IMXRT_CCM_OSCPLL_LPM1_OFFSET(n)   (0x5014u + ((n) << 6))
#define IMXRT_CCM_OSCPLL_LPMCUR_OFFSET(n) (0x501cu + ((n) << 6))
#define IMXRT_CCM_OSCPLL_STAT0_OFFSET(n)  (0x5020u + ((n) << 6))
#define IMXRT_CCM_OSCPLL_STAT1_OFFSET(n)  (0x5024u + ((n) << 6))
#define IMXRT_CCM_OSCPLL_AUTH_OFFSET(n)   (0x5030u + ((n) << 6))

#define IMXRT_CCM_LPCG_DIR_OFFSET(n)      (0x8000u + ((n) << 6))
#define IMXRT_CCM_LPCG_LPM0_OFFSET(n)     (0x8010u + ((n) << 6))
#define IMXRT_CCM_LPCG_LPM1_OFFSET(n)     (0x8014u + ((n) << 6))
#define IMXRT_CCM_LPCG_LPMCUR_OFFSET(n)   (0x801cu + ((n) << 6))
#define IMXRT_CCM_LPCG_STAT0_OFFSET(n)    (0x8020u + ((n) << 6))
#define IMXRT_CCM_LPCG_STAT1_OFFSET(n)    (0x8024u + ((n) << 6))
#define IMXRT_CCM_LPCG_AUTH_OFFSET(n)     (0x8030u + ((n) << 6))

/* Register addresses *******************************************************/

#define IMXRT_CCM_ROOT_CTRL(n)      (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_CTRL_OFFSET(n))
#define IMXRT_CCM_ROOT_SET(n)       (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_SET_OFFSET(n))
#define IMXRT_CCM_ROOT_CLR(n)       (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_CLR_OFFSET(n))
#define IMXRT_CCM_ROOT_TOG(n)       (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_TOG_OFFSET(n))
#define IMXRT_CCM_ROOT_STAT(n)      (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_STAT_OFFSET(n))
#define IMXRT_CCM_ROOT_AUTH(n)      (IMXRT_CCM_BASE + IMXRT_CCM_ROOT_AUTH_OFFSET(n))

#define IMXRT_CCM_OBS_CTRL(n)       (IMXRT_CCM_BASE + IMXRT_CCM_OBS_CTRL_OFFSET(n))
#define IMXRT_CCM_OBS_SET(n)        (IMXRT_CCM_BASE + IMXRT_CCM_OBS_SET_OFFSET(n))
#define IMXRT_CCM_OBS_CLR(n)        (IMXRT_CCM_BASE + IMXRT_CCM_OBS_CLR_OFFSET(n))
#define IMXRT_CCM_OBS_TOG(n)        (IMXRT_CCM_BASE + IMXRT_CCM_OBS_TOG_OFFSET(n))
#define IMXRT_CCM_OBS_STAT(n)       (IMXRT_CCM_BASE + IMXRT_CCM_OBS_STAT_OFFSET(n))
#define IMXRT_CCM_OBS_AUTH(n)       (IMXRT_CCM_BASE + IMXRT_CCM_OBS_AUTH_OFFSET(n))
#define IMXRT_CCM_OBS_FREQ_CUR(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_FREQ_CUR_OFFSET(n))
#define IMXRT_CCM_OBS_FREQ_MIN(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_FREQ_MIN_OFFSET(n))
#define IMXRT_CCM_OBS_FREQ_MAX(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_FREQ_MAX_OFFSET(n))
#define IMXRT_CCM_OBS_PER_CUR(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_PER_CUR_OFFSET(n))
#define IMXRT_CCM_OBS_PER_MIN(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_PER_MIN_OFFSET(n))
#define IMXRT_CCM_OBS_PER_MAX(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_PER_MAX_OFFSET(n))
#define IMXRT_CCM_OBS_HIGH_CUR(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_HIGH_CUR_OFFSET(n))
#define IMXRT_CCM_OBS_HIGH_MIN(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_HIGH_MIN_OFFSET(n))
#define IMXRT_CCM_OBS_HIGH_MAX(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OBS_HIGH_MAX_OFFSET(n))
#define IMXRT_CCM_OBS_LOW_CUR(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_LOW_CUR_OFFSET(n))
#define IMXRT_CCM_OBS_LOW_MIN(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_LOW_MIN_OFFSET(n))
#define IMXRT_CCM_OBS_LOW_MAX(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OBS_LOW_MAX_OFFSET(n))

#define IMXRT_CCM_GPR_SHARED(n)     (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_OFFSET(n))
#define IMXRT_CCM_GPR_SHARED_SET(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_SET_OFFSET(n))
#define IMXRT_CCM_GPR_SHARED_CLR(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_SHARED_TOG(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_TOG_OFFSET(n))
#define IMXRT_CCM_GPR_SHARED_AUTH(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_AUTH_OFFSET(n))
#define IMXRT_CCM_GPR_SHARED_STAT(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_SHARED_STAT_OFFSET(n))

#define IMXRT_CCM_GPR_PRIVATE(n)    (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PRIVATE_OFFSET(n))
#define IMXRT_CCM_GPR_PRIVATE_SET(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PRIVATE_SET_OFFSET(n))
#define IMXRT_CCM_GPR_PRIVATE_CLR(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PRIVATE_CLR_OFFSET(n))
#define IMXRT_CCM_GPR_PRIVATE_TOG(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PRIVATE_TOG_OFFSET(n))
#define IMXRT_CCM_GPR_PRIVATE_AUTH(n) (IMXRT_CCM_BASE + IMXRT_CCM_GPR_PRIVATE_AUTH_OFFSET(n))

#define IMXRT_CCM_OSCPLL_DIR(n)     (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_DIR_OFFSET(n))
#define IMXRT_CCM_OSCPLL_LPM0(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_LPM0_OFFSET(n))
#define IMXRT_CCM_OSCPLL_LPM1(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_LPM1_OFFSET(n))
#define IMXRT_CCM_OSCPLL_LPMCUR(n)  (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_LPMCUR_OFFSET(n))
#define IMXRT_CCM_OSCPLL_STAT0(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_STAT0_OFFSET(n))
#define IMXRT_CCM_OSCPLL_STAT1(n)   (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_STAT1_OFFSET(n))
#define IMXRT_CCM_OSCPLL_AUTH(n)    (IMXRT_CCM_BASE + IMXRT_CCM_OSCPLL_AUTH_OFFSET(n))

#define IMXRT_CCM_LPCG_DIR(n)       (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_DIR_OFFSET(n))
#define IMXRT_CCM_LPCG_LPM0(n)      (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_LPM0_OFFSET(n))
#define IMXRT_CCM_LPCG_LPM1(n)      (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_LPM1_OFFSET(n))
#define IMXRT_CCM_LPCG_LPMCUR(n)    (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_LPMCUR_OFFSET(n))
#define IMXRT_CCM_LPCG_STAT0(n)     (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_STAT0_OFFSET(n))
#define IMXRT_CCM_LPCG_STAT1(n)     (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_STAT1_OFFSET(n))
#define IMXRT_CCM_LPCG_AUTH(n)      (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_AUTH_OFFSET(n))

/* Clock root control and status ********************************************/

#define CCM_ROOT_DIV_SHIFT           0
#define CCM_ROOT_DIV_MASK            (0xffu << CCM_ROOT_DIV_SHIFT)
#define CCM_ROOT_DIV(n)              (((n) - 1u) << CCM_ROOT_DIV_SHIFT)
#define CCM_ROOT_MUX_SHIFT           8
#define CCM_ROOT_MUX_MASK            (0x03u << CCM_ROOT_MUX_SHIFT)
#define CCM_ROOT_MUX(n)              ((uint32_t)(n) << CCM_ROOT_MUX_SHIFT)
#define CCM_ROOT_OFF                 (1u << 24)
#define CCM_ROOT_STAT_BUSY           (1u << 28)
#define CCM_ROOT_STAT_UPDATE         (1u << 29)
#define CCM_ROOT_STAT_CHANGING       (1u << 31)

/* Access control registers *************************************************/

#define CCM_AUTH_TZ_USER             (1u << 8)
#define CCM_AUTH_TZ_NS               (1u << 9)
#define CCM_AUTH_LOCK_TZ             (1u << 11)
#define CCM_AUTH_LOCK_LIST           (1u << 15)
#define CCM_AUTH_WHITE_LIST_SHIFT    16
#define CCM_AUTH_WHITE_LIST_MASK     (0xffffu << CCM_AUTH_WHITE_LIST_SHIFT)
#define CCM_AUTH_WHITE_LIST(n)       ((uint32_t)(n) << CCM_AUTH_WHITE_LIST_SHIFT)

/* Low-power setting registers **********************************************/

#define CCM_LPM_SHIFT(n)             (((n) & 7u) << 2)
#define CCM_LPM_MASK(n)              (0x07u << CCM_LPM_SHIFT(n))
#define CCM_LPM_SETTING(n, v)        ((uint32_t)(v) << CCM_LPM_SHIFT(n))
#define CCM_LPM_RUN                  0u
#define CCM_LPM_WAIT                 1u
#define CCM_LPM_STOP                 2u
#define CCM_LPM_SUSPEND              3u

/* OSC/PLL and LPCG controls ************************************************/

#define CCM_OSCPLL_DIR_ON            (1u << 0)
#define CCM_OSCPLL_STAT0_ON          (1u << 0)
#define CCM_OSCPLL_STAT0_IN_USE      (1u << 28)
#define CCM_LPCG_DIR_ON              (1u << 0)
#define CCM_LPCG_DIR_TIMEOUT_EN      (1u << 2)
#define CCM_LPCG_STAT0_ON            (1u << 0)
#define CCM_LPCG_STAT0_IN_USE        (1u << 28)

#define CCM_CG_OFF                   0u
#define CCM_CG_RUN                   CCM_LPCG_DIR_ON
#define CCM_CG_ALL                   CCM_LPCG_DIR_ON

/* Clock roots **************************************************************/

#define CCM_CR_M7                    0
#define CCM_CR_M33                   1
#define CCM_CR_EDGELOCK              2
#define CCM_CR_BUS_AON               3
#define CCM_CR_BUS_WAKEUP            4
#define CCM_CR_WAKEUP_AXI            5
#define CCM_CR_SWO_TRACE             6
#define CCM_CR_M33_SYSTICK           7
#define CCM_CR_M7_SYSTICK            8
#define CCM_CR_FLEXIO1               9
#define CCM_CR_FLEXIO2               10
#define CCM_CR_LPIT3                 11
#define CCM_CR_LPTMR1                12
#define CCM_CR_LPTMR2                13
#define CCM_CR_LPTMR3                14
#define CCM_CR_TPM2                  15
#define CCM_CR_TPM4                  16
#define CCM_CR_TPM5                  17
#define CCM_CR_TPM6                  18
#define CCM_CR_GPT1                  19
#define CCM_CR_GPT2                  20
#define CCM_CR_FLEXSPI1              21
#define CCM_CR_FLEXSPI2              22
#define CCM_CR_FLEXSPI_SLV           23
#define CCM_CR_CAN1                  24
#define CCM_CR_CAN2                  25
#define CCM_CR_CAN3                  26
#define CCM_CR_LPUART0102            27
#define CCM_CR_LPUART0304            28
#define CCM_CR_LPUART0506            29
#define CCM_CR_LPUART0708            30
#define CCM_CR_LPUART0910            31
#define CCM_CR_LPUART1112            32
#define CCM_CR_LPI2C0102             33
#define CCM_CR_LPI2C0304             34
#define CCM_CR_LPI2C0506             35
#define CCM_CR_LPSPI0102             36
#define CCM_CR_LPSPI0304             37
#define CCM_CR_LPSPI0506             38
#define CCM_CR_I3C1                  39
#define CCM_CR_I3C2                  40
#define CCM_CR_USDHC1                41
#define CCM_CR_USDHC2                42
#define CCM_CR_SEMC                  43
#define CCM_CR_ADC1                  44
#define CCM_CR_ADC2                  45
#define CCM_CR_ACMP                  46
#define CCM_CR_ECAT                  47
#define CCM_CR_ENET                  48
#define CCM_CR_TMR_1588              49
#define CCM_CR_NETC                  50
#define CCM_CR_MAC0                  51
#define CCM_CR_MAC1                  52
#define CCM_CR_MAC2                  53
#define CCM_CR_MAC3                  54
#define CCM_CR_MAC4                  55
#define CCM_CR_SERDES0               56
#define CCM_CR_SERDES1               57
#define CCM_CR_SERDES2               58
#define CCM_CR_SERDES0_1G            59
#define CCM_CR_SERDES1_1G            60
#define CCM_CR_SERDES2_1G            61
#define CCM_CR_XCELBUSX              62
#define CCM_CR_XRIOCU4               63
#define CCM_CR_MCTRL                 64
#define CCM_CR_SAI1                  65
#define CCM_CR_SAI2                  66
#define CCM_CR_SAI3                  67
#define CCM_CR_SAI4                  68
#define CCM_CR_SPDIF                 69
#define CCM_CR_ASRC                  70
#define CCM_CR_MIC                   71
#define CCM_CR_CKO1                  72
#define CCM_CR_CKO2                  73

/* LPCG indices *************************************************************/

#define CCM_CCGR_M7                  0
#define CCM_CCGR_M33                 1
#define CCM_CCGR_EDGELOCK            2
#define CCM_CCGR_SIM_AON             3
#define CCM_CCGR_SIM_WAKEUP          4
#define CCM_CCGR_SIM_MEGA            5
#define CCM_CCGR_SIM_R               6
#define CCM_CCGR_ANADIG              7
#define CCM_CCGR_DCDC                8
#define CCM_CCGR_SRC                 9
#define CCM_CCGR_CCM                 10
#define CCM_CCGR_GPC                 11
#define CCM_CCGR_ADC1                12
#define CCM_CCGR_ADC2                13
#define CCM_CCGR_DAC                 14
#define CCM_CCGR_ACMP1               15
#define CCM_CCGR_ACMP2               16
#define CCM_CCGR_ACMP3               17
#define CCM_CCGR_ACMP4               18
#define CCM_CCGR_WDOG1               19
#define CCM_CCGR_WDOG2               20
#define CCM_CCGR_WDOG3               21
#define CCM_CCGR_WDOG4               22
#define CCM_CCGR_WDOG5               23
#define CCM_CCGR_EWM0                24
#define CCM_CCGR_SEMA1               25
#define CCM_CCGR_SEMA2               26
#define CCM_CCGR_MU_A                27
#define CCM_CCGR_MU_B                28
#define CCM_CCGR_EDMA3               29
#define CCM_CCGR_EDMA4               30
#define CCM_CCGR_ROMCP               31
#define CCM_CCGR_OCRAM1              32
#define CCM_CCGR_OCRAM2              33
#define CCM_CCGR_FLEXSPI1            34
#define CCM_CCGR_FLEXSPI2            35
#define CCM_CCGR_FLEXSPI_SLV         36
#define CCM_CCGR_TRDC                37
#define CCM_CCGR_OCOTP               38
#define CCM_CCGR_SEMC                39
#define CCM_CCGR_IEE                 40
#define CCM_CCGR_CSTRACE             41
#define CCM_CCGR_CSSWO               42
#define CCM_CCGR_IOMUXC1             43
#define CCM_CCGR_IOMUXC2             44
#define CCM_CCGR_GPIO1               45
#define CCM_CCGR_GPIO2               46
#define CCM_CCGR_GPIO3               47
#define CCM_CCGR_GPIO4               48
#define CCM_CCGR_GPIO5               49
#define CCM_CCGR_GPIO6               50
#define CCM_CCGR_FLEXIO1             51
#define CCM_CCGR_FLEXIO2             52
#define CCM_CCGR_LPIT1               53
#define CCM_CCGR_LPIT2               54
#define CCM_CCGR_LPIT3               55
#define CCM_CCGR_LPTMR1              56
#define CCM_CCGR_LPTMR2              57
#define CCM_CCGR_LPTMR3              58
#define CCM_CCGR_TPM1                59
#define CCM_CCGR_TPM2                60
#define CCM_CCGR_TPM3                61
#define CCM_CCGR_TPM4                62
#define CCM_CCGR_TPM5                63
#define CCM_CCGR_TPM6                64
#define CCM_CCGR_QTIMER1             65
#define CCM_CCGR_QTIMER2             66
#define CCM_CCGR_QTIMER3             67
#define CCM_CCGR_QTIMER4             68
#define CCM_CCGR_QTIMER5             69
#define CCM_CCGR_QTIMER6             70
#define CCM_CCGR_QTIMER7             71
#define CCM_CCGR_QTIMER8             72
#define CCM_CCGR_GPT1                73
#define CCM_CCGR_GPT2                74
#define CCM_CCGR_SYSCOUNT            75
#define CCM_CCGR_CAN1                76
#define CCM_CCGR_CAN2                77
#define CCM_CCGR_CAN3                78
#define CCM_CCGR_LPUART1             79
#define CCM_CCGR_LPUART2             80
#define CCM_CCGR_LPUART3             81
#define CCM_CCGR_LPUART4             82
#define CCM_CCGR_LPUART5             83
#define CCM_CCGR_LPUART6             84
#define CCM_CCGR_LPUART7             85
#define CCM_CCGR_LPUART8             86
#define CCM_CCGR_LPUART9             87
#define CCM_CCGR_LPUART10            88
#define CCM_CCGR_LPUART11            89
#define CCM_CCGR_LPUART12            90
#define CCM_CCGR_LPI2C1              91
#define CCM_CCGR_LPI2C2              92
#define CCM_CCGR_LPI2C3              93
#define CCM_CCGR_LPI2C4              94
#define CCM_CCGR_LPI2C5              95
#define CCM_CCGR_LPI2C6              96
#define CCM_CCGR_LPSPI1              97
#define CCM_CCGR_LPSPI2              98
#define CCM_CCGR_LPSPI3              99
#define CCM_CCGR_LPSPI4              100
#define CCM_CCGR_LPSPI5              101
#define CCM_CCGR_LPSPI6              102
#define CCM_CCGR_I3C1                103
#define CCM_CCGR_I3C2                104
#define CCM_CCGR_USDHC1              105
#define CCM_CCGR_USDHC2              106
#define CCM_CCGR_USB                 107
#define CCM_CCGR_SINC1               108
#define CCM_CCGR_SINC2               109
#define CCM_CCGR_SINC3               110
#define CCM_CCGR_XBAR1               111
#define CCM_CCGR_XBAR2               112
#define CCM_CCGR_XBAR3               113
#define CCM_CCGR_AOI1                114
#define CCM_CCGR_AOI2                115
#define CCM_CCGR_AOI3                116
#define CCM_CCGR_AOI4                117
#define CCM_CCGR_ENC1                118
#define CCM_CCGR_ENC2                119
#define CCM_CCGR_ENC3                120
#define CCM_CCGR_ENC4                121
#define CCM_CCGR_KPP                 122
#define CCM_CCGR_PWM1                123
#define CCM_CCGR_PWM2                124
#define CCM_CCGR_PWM3                125
#define CCM_CCGR_PWM4                126
#define CCM_CCGR_ECAT                127
#define CCM_CCGR_NETC                128
#define CCM_CCGR_SERDES1             129
#define CCM_CCGR_SERDES2             130
#define CCM_CCGR_SERDES3             131
#define CCM_CCGR_XCELBUSX            132
#define CCM_CCGR_XRIOCU4             133
#define CCM_CCGR_SPTP                134
#define CCM_CCGR_MCTRL               135
#define CCM_CCGR_SAI1                136
#define CCM_CCGR_SAI2                137
#define CCM_CCGR_SAI3                138
#define CCM_CCGR_SAI4                139
#define CCM_CCGR_SPDIF               140
#define CCM_CCGR_ASRC                141
#define CCM_CCGR_PDM                 142
#define CCM_CCGR_VREF                143
#define CCM_CCGR_BIST                144
#define CCM_CCGR_SSI_W2M7            145
#define CCM_CCGR_SSI_M72W            146
#define CCM_CCGR_SSI_W2AO            147
#define CCM_CCGR_SSI_AO2W            148

/* Compatibility aliases used by existing i.MX RT peripheral drivers. *******/

#define CCM_CCGR_DMA                 CCM_CCGR_EDMA3
#define CCM_CCGR_EWM                 CCM_CCGR_EWM0
#define CCM_CCGR_IOMUXC              CCM_CCGR_IOMUXC1
#define CCM_CCGR_OCOTP_CTRL          CCM_CCGR_OCOTP
#define CCM_CCGR_OCRAM               CCM_CCGR_OCRAM1
#define CCM_CCGR_PIT                 CCM_CCGR_LPIT1

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H */
