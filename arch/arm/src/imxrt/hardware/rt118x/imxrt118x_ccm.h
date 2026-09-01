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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMXRT_CCM_CR_CTRL_OFFSET(n)     (0x0000 + ((n) << 7))
#define IMXRT_CCM_CR_CTRL_SET_OFFSET(n) (0x0004 + ((n) << 7))
#define IMXRT_CCM_CR_CTRL_CLR_OFFSET(n) (0x0008 + ((n) << 7))
#define IMXRT_CCM_CR_CTRL_TOG_OFFSET(n) (0x000c + ((n) << 7))
#define IMXRT_CCM_CR_STAT0_OFFSET(n)    (0x0020 + ((n) << 7))
#define IMXRT_CCM_CR_AUTH_OFFSET(n)     (0x0030 + ((n) << 7))

#define IMXRT_CCM_LPCG_DIR_OFFSET(n)    (0x8000 + ((n) << 6))
#define IMXRT_CCM_LPCG_STAT0_OFFSET(n)  (0x8020 + ((n) << 6))
#define IMXRT_CCM_LPCG_AUTH_OFFSET(n)   (0x8030 + ((n) << 6))

/* Register addresses *******************************************************/

#define IMXRT_CCM_CR_CTRL(n)     \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_SET(n) \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_SET_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_CLR(n) \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_CLR_OFFSET(n))
#define IMXRT_CCM_CR_CTRL_TOG(n) \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_CTRL_TOG_OFFSET(n))
#define IMXRT_CCM_CR_STAT0(n)    \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_STAT0_OFFSET(n))
#define IMXRT_CCM_CR_AUTH(n)     \
  (IMXRT_CCM_BASE + IMXRT_CCM_CR_AUTH_OFFSET(n))

#define IMXRT_CCM_LPCG_DIR(n)    \
  (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_DIR_OFFSET(n))
#define IMXRT_CCM_LPCG_STAT0(n)  \
  (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_STAT0_OFFSET(n))
#define IMXRT_CCM_LPCG_AUTH(n)   \
  (IMXRT_CCM_BASE + IMXRT_CCM_LPCG_AUTH_OFFSET(n))

/* Register bit definitions *************************************************/

/* Clock root control (CLOCK_ROOTn_CONTROL) */

#define CCM_CR_CTRL_DIV_SHIFT     (0)
#define CCM_CR_CTRL_DIV_MASK      (0xff << CCM_CR_CTRL_DIV_SHIFT)
#  define CCM_CR_CTRL_DIV(n)      (((n) - 1) << CCM_CR_CTRL_DIV_SHIFT)
#define CCM_CR_CTRL_MUX_SHIFT     (8)
#define CCM_CR_CTRL_MUX_MASK      (0x03 << CCM_CR_CTRL_MUX_SHIFT)
#  define CCM_CR_CTRL_MUX_SRCSEL(n) ((n) << CCM_CR_CTRL_MUX_SHIFT)
#define CCM_CR_CTRL_OFF           (1 << 24)

/* Clock root working status (CLOCK_ROOTn_STATUS0) */

#define CCM_CR_STAT0_DIV_SHIFT    (0)
#define CCM_CR_STAT0_DIV_MASK     (0xff << CCM_CR_STAT0_DIV_SHIFT)
#define CCM_CR_STAT0_MUX_SHIFT    (8)
#define CCM_CR_STAT0_MUX_MASK     (0x03 << CCM_CR_STAT0_MUX_SHIFT)
#define CCM_CR_STAT0_OFF          (1 << 24)
#define CCM_CR_STAT0_SLICE_BUSY   (1 << 28)
#define CCM_CR_STAT0_CHANGING     (1 << 31)

/* Clock root access control (CLOCK_ROOTn_AUTHEN) */

#define CCM_CR_AUTH_TZ_USER          (1u << 8)
#define CCM_CR_AUTH_TZ_NS            (1u << 9)
#define CCM_CR_AUTH_LOCK_TZ          (1u << 11)
#define CCM_CR_AUTH_LOCK_LIST        (1u << 15)
#define CCM_CR_AUTH_WHITE_LIST_SHIFT (16)
#define CCM_CR_AUTH_WHITE_LIST_MASK  (0xffffu << \
                                      CCM_CR_AUTH_WHITE_LIST_SHIFT)
#  define CCM_CR_AUTH_DOMAIN(n)      (1u << \
                                      (CCM_CR_AUTH_WHITE_LIST_SHIFT + (n)))

/* Low-power clock gate (LPCGn_DIRECT / LPCGn_STATUS0) */

#define CCM_LPCG_DIR_ON           (1 << 0)
#define CCM_LPCG_STAT0_ON         (1 << 0)

/* Clock roots (IMXRT1180RM Ch. 20, extracted from NXP MCUXpresso SDK
 * clock_root_t enumeration).
 */

#define CCM_CR_M7                   (0)
#define CCM_CR_M33                  (1)
#define CCM_CR_EDGELOCK             (2)
#define CCM_CR_BUS_AON              (3)
#define CCM_CR_BUS_WAKEUP           (4)
#define CCM_CR_WAKEUP_AXI           (5)
#define CCM_CR_SWO_TRACE            (6)
#define CCM_CR_M33_SYSTICK          (7)
#define CCM_CR_M7_SYSTICK           (8)
#define CCM_CR_FLEXIO1              (9)
#define CCM_CR_FLEXIO2              (10)
#define CCM_CR_LPIT3                (11)
#define CCM_CR_LPTIMER1             (12)
#define CCM_CR_LPTIMER2             (13)
#define CCM_CR_LPTIMER3             (14)
#define CCM_CR_TPM2                 (15)
#define CCM_CR_TPM4                 (16)
#define CCM_CR_TPM5                 (17)
#define CCM_CR_TPM6                 (18)
#define CCM_CR_GPT1                 (19)
#define CCM_CR_GPT2                 (20)
#define CCM_CR_FLEXSPI1             (21)
#define CCM_CR_FLEXSPI2             (22)
#define CCM_CR_FLEXSPI_SLV          (23)
#define CCM_CR_CAN1                 (24)
#define CCM_CR_CAN2                 (25)
#define CCM_CR_CAN3                 (26)
#define CCM_CR_LPUART0102           (27)
#define CCM_CR_LPUART0304           (28)
#define CCM_CR_LPUART0506           (29)
#define CCM_CR_LPUART0708           (30)
#define CCM_CR_LPUART0910           (31)
#define CCM_CR_LPUART1112           (32)
#define CCM_CR_LPI2C0102            (33)
#define CCM_CR_LPI2C0304            (34)
#define CCM_CR_LPI2C0506            (35)
#define CCM_CR_LPSPI0102            (36)
#define CCM_CR_LPSPI0304            (37)
#define CCM_CR_LPSPI0506            (38)

/* On RT1180 the LPUART / LPI2C / LPSPI clock roots are shared between
 * pairs of instances.  Per-instance aliases keep source code that names
 * individual LPUART1..12, LPI2C1..6 or LPSPI1..6 roots compatible with
 * both RT117x and RT118x.
 */

#define CCM_CR_LPUART1              CCM_CR_LPUART0102
#define CCM_CR_LPUART2              CCM_CR_LPUART0102
#define CCM_CR_LPUART3              CCM_CR_LPUART0304
#define CCM_CR_LPUART4              CCM_CR_LPUART0304
#define CCM_CR_LPUART5              CCM_CR_LPUART0506
#define CCM_CR_LPUART6              CCM_CR_LPUART0506
#define CCM_CR_LPUART7              CCM_CR_LPUART0708
#define CCM_CR_LPUART8              CCM_CR_LPUART0708
#define CCM_CR_LPUART9              CCM_CR_LPUART0910
#define CCM_CR_LPUART10             CCM_CR_LPUART0910
#define CCM_CR_LPUART11             CCM_CR_LPUART1112
#define CCM_CR_LPUART12             CCM_CR_LPUART1112

#define CCM_CR_LPI2C1               CCM_CR_LPI2C0102
#define CCM_CR_LPI2C2               CCM_CR_LPI2C0102
#define CCM_CR_LPI2C3               CCM_CR_LPI2C0304
#define CCM_CR_LPI2C4               CCM_CR_LPI2C0304
#define CCM_CR_LPI2C5               CCM_CR_LPI2C0506
#define CCM_CR_LPI2C6               CCM_CR_LPI2C0506

#define CCM_CR_LPSPI1               CCM_CR_LPSPI0102
#define CCM_CR_LPSPI2               CCM_CR_LPSPI0102
#define CCM_CR_LPSPI3               CCM_CR_LPSPI0304
#define CCM_CR_LPSPI4               CCM_CR_LPSPI0304
#define CCM_CR_LPSPI5               CCM_CR_LPSPI0506
#define CCM_CR_LPSPI6               CCM_CR_LPSPI0506
#define CCM_CR_I3C1                 (39)
#define CCM_CR_I3C2                 (40)
#define CCM_CR_USDHC1               (41)
#define CCM_CR_USDHC2               (42)
#define CCM_CR_SEMC                 (43)
#define CCM_CR_ADC1                 (44)
#define CCM_CR_ADC2                 (45)
#define CCM_CR_ACMP                 (46)
#define CCM_CR_ECAT                 (47)
#define CCM_CR_ENET                 (48)
#define CCM_CR_TMR_1588             (49)
#define CCM_CR_NETC                 (50)
#define CCM_CR_MAC0                 (51)
#define CCM_CR_MAC1                 (52)
#define CCM_CR_MAC2                 (53)
#define CCM_CR_MAC3                 (54)
#define CCM_CR_MAC4                 (55)
#define CCM_CR_SERDES0              (56)
#define CCM_CR_SERDES1              (57)
#define CCM_CR_SERDES2              (58)
#define CCM_CR_SERDES0_1G           (59)
#define CCM_CR_SERDES1_1G           (60)
#define CCM_CR_SERDES2_1G           (61)
#define CCM_CR_XCELBUSX             (62)
#define CCM_CR_XRIOCU4              (63)
#define CCM_CR_MCTRL                (64)
#define CCM_CR_SAI1                 (65)
#define CCM_CR_SAI2                 (66)
#define CCM_CR_SAI3                 (67)
#define CCM_CR_SAI4                 (68)
#define CCM_CR_SPDIF                (69)
#define CCM_CR_ASRC                 (70)
#define CCM_CR_MIC                  (71)
#define CCM_CR_CKO1                 (72)
#define CCM_CR_CKO2                 (73)

/* Low-power clock gates (IMXRT1180RM Ch. 20, extracted from NXP MCUXpresso
 * SDK clock_lpcg_t enumeration).
 */

#define CCM_LPCG_M7                   (0)
#define CCM_LPCG_M33                  (1)
#define CCM_LPCG_EDGELOCK             (2)
#define CCM_LPCG_SIM_AON              (3)
#define CCM_LPCG_SIM_WAKEUP           (4)
#define CCM_LPCG_SIM_MEGA             (5)
#define CCM_LPCG_SIM_R                (6)
#define CCM_LPCG_ANADIG               (7)
#define CCM_LPCG_DCDC                 (8)
#define CCM_LPCG_SRC                  (9)
#define CCM_LPCG_CCM                  (10)
#define CCM_LPCG_GPC                  (11)
#define CCM_LPCG_ADC1                 (12)
#define CCM_LPCG_ADC2                 (13)
#define CCM_LPCG_DAC                  (14)
#define CCM_LPCG_ACMP1                (15)
#define CCM_LPCG_ACMP2                (16)
#define CCM_LPCG_ACMP3                (17)
#define CCM_LPCG_ACMP4                (18)
#define CCM_LPCG_WDOG1                (19)
#define CCM_LPCG_WDOG2                (20)
#define CCM_LPCG_WDOG3                (21)
#define CCM_LPCG_WDOG4                (22)
#define CCM_LPCG_WDOG5                (23)
#define CCM_LPCG_EWM0                 (24)
#define CCM_LPCG_SEMA1                (25)
#define CCM_LPCG_SEMA2                (26)
#define CCM_LPCG_MU_A                 (27)
#define CCM_LPCG_MU_B                 (28)
#define CCM_LPCG_EDMA3                (29)
#define CCM_LPCG_EDMA4                (30)
#define CCM_LPCG_ROMCP                (31)
#define CCM_LPCG_OCRAM1               (32)
#define CCM_LPCG_OCRAM2               (33)
#define CCM_LPCG_FLEXSPI1             (34)
#define CCM_LPCG_FLEXSPI2             (35)
#define CCM_LPCG_FLEXSPI_SLV          (36)
#define CCM_LPCG_TRDC                 (37)
#define CCM_LPCG_OCOTP                (38)
#define CCM_LPCG_SEMC                 (39)
#define CCM_LPCG_IEE                  (40)
#define CCM_LPCG_CSTRACE              (41)
#define CCM_LPCG_CSSWO                (42)
#define CCM_LPCG_IOMUXC1              (43)
#define CCM_LPCG_IOMUXC2              (44)
#define CCM_LPCG_GPIO1                (45)
#define CCM_LPCG_GPIO2                (46)
#define CCM_LPCG_GPIO3                (47)
#define CCM_LPCG_GPIO4                (48)
#define CCM_LPCG_GPIO5                (49)
#define CCM_LPCG_GPIO6                (50)
#define CCM_LPCG_FLEXIO1              (51)
#define CCM_LPCG_FLEXIO2              (52)
#define CCM_LPCG_LPIT1                (53)
#define CCM_LPCG_LPIT2                (54)
#define CCM_LPCG_LPIT3                (55)
#define CCM_LPCG_LPTMR1               (56)
#define CCM_LPCG_LPTMR2               (57)
#define CCM_LPCG_LPTMR3               (58)
#define CCM_LPCG_TPM1                 (59)
#define CCM_LPCG_TPM2                 (60)
#define CCM_LPCG_TPM3                 (61)
#define CCM_LPCG_TPM4                 (62)
#define CCM_LPCG_TPM5                 (63)
#define CCM_LPCG_TPM6                 (64)
#define CCM_LPCG_QTIMER1              (65)
#define CCM_LPCG_QTIMER2              (66)
#define CCM_LPCG_QTIMER3              (67)
#define CCM_LPCG_QTIMER4              (68)
#define CCM_LPCG_QTIMER5              (69)
#define CCM_LPCG_QTIMER6              (70)
#define CCM_LPCG_QTIMER7              (71)
#define CCM_LPCG_QTIMER8              (72)
#define CCM_LPCG_GPT1                 (73)
#define CCM_LPCG_GPT2                 (74)
#define CCM_LPCG_SYSCOUNT             (75)
#define CCM_LPCG_CAN1                 (76)
#define CCM_LPCG_CAN2                 (77)
#define CCM_LPCG_CAN3                 (78)
#define CCM_LPCG_LPUART1              (79)
#define CCM_LPCG_LPUART2              (80)
#define CCM_LPCG_LPUART3              (81)
#define CCM_LPCG_LPUART4              (82)
#define CCM_LPCG_LPUART5              (83)
#define CCM_LPCG_LPUART6              (84)
#define CCM_LPCG_LPUART7              (85)
#define CCM_LPCG_LPUART8              (86)
#define CCM_LPCG_LPUART9              (87)
#define CCM_LPCG_LPUART10             (88)
#define CCM_LPCG_LPUART11             (89)
#define CCM_LPCG_LPUART12             (90)
#define CCM_LPCG_LPI2C1               (91)
#define CCM_LPCG_LPI2C2               (92)
#define CCM_LPCG_LPI2C3               (93)
#define CCM_LPCG_LPI2C4               (94)
#define CCM_LPCG_LPI2C5               (95)
#define CCM_LPCG_LPI2C6               (96)
#define CCM_LPCG_LPSPI1               (97)
#define CCM_LPCG_LPSPI2               (98)
#define CCM_LPCG_LPSPI3               (99)
#define CCM_LPCG_LPSPI4               (100)
#define CCM_LPCG_LPSPI5               (101)
#define CCM_LPCG_LPSPI6               (102)
#define CCM_LPCG_I3C1                 (103)
#define CCM_LPCG_I3C2                 (104)
#define CCM_LPCG_USDHC1               (105)
#define CCM_LPCG_USDHC2               (106)
#define CCM_LPCG_USB                  (107)
#define CCM_LPCG_SINC1                (108)
#define CCM_LPCG_SINC2                (109)
#define CCM_LPCG_SINC3                (110)
#define CCM_LPCG_XBAR1                (111)
#define CCM_LPCG_XBAR2                (112)
#define CCM_LPCG_XBAR3                (113)
#define CCM_LPCG_AOI1                 (114)
#define CCM_LPCG_AOI2                 (115)
#define CCM_LPCG_AOI3                 (116)
#define CCM_LPCG_AOI4                 (117)
#define CCM_LPCG_ENC1                 (118)
#define CCM_LPCG_ENC2                 (119)
#define CCM_LPCG_ENC3                 (120)
#define CCM_LPCG_ENC4                 (121)
#define CCM_LPCG_KPP                  (122)
#define CCM_LPCG_PWM1                 (123)
#define CCM_LPCG_PWM2                 (124)
#define CCM_LPCG_PWM3                 (125)
#define CCM_LPCG_PWM4                 (126)
#define CCM_LPCG_ECAT                 (127)
#define CCM_LPCG_NETC                 (128)
#define CCM_LPCG_SERDES1              (129)
#define CCM_LPCG_SERDES2              (130)
#define CCM_LPCG_SERDES3              (131)
#define CCM_LPCG_XCELBUSX             (132)
#define CCM_LPCG_XRIOCU4              (133)
#define CCM_LPCG_SPTP                 (134)
#define CCM_LPCG_MCTRL                (135)
#define CCM_LPCG_SAI1                 (136)
#define CCM_LPCG_SAI2                 (137)
#define CCM_LPCG_SAI3                 (138)
#define CCM_LPCG_SAI4                 (139)
#define CCM_LPCG_SPDIF                (140)
#define CCM_LPCG_ASRC                 (141)
#define CCM_LPCG_PDM                  (142)
#define CCM_LPCG_VREF                 (143)
#define CCM_LPCG_BIST                 (144)
#define CCM_LPCG_SSI_W2M7             (145)
#define CCM_LPCG_SSI_M72W             (146)
#define CCM_LPCG_SSI_W2AO             (147)
#define CCM_LPCG_SSI_AO2W             (148)

/* Counts */

#define ROOT_MUX_MAX              (4)
#define CCM_CR_COUNT              (74)
#define CCM_LPCG_COUNT            (149)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock source names used by the g_ccm_root_mux[] LUT and consumed by
 * imxrt_get_clock() to return each source's frequency.  The M33 boot ROM
 * has already brought up ARM_PLL, SYS_PLL1/2/3 and their PFDs; NuttX only
 * reads their frequencies (see imxrt_clockconfig_ver3.c).
 */

enum ccm_clock_name_e
{
  OSC_RC_24M    = 0,
  OSC_RC_400M   = 1,
  OSC_24M       = 2,
  ARM_PLL_OUT   = 3,
  SYS_PLL1_OUT  = 4,
  SYS_PLL1_DIV2 = 5,
  SYS_PLL1_DIV5 = 6,
  SYS_PLL2_OUT  = 7,
  SYS_PLL2_PFD0 = 8,
  SYS_PLL2_PFD1 = 9,
  SYS_PLL2_PFD2 = 10,
  SYS_PLL2_PFD3 = 11,
  SYS_PLL3_OUT  = 12,
  SYS_PLL3_DIV2 = 13,
  SYS_PLL3_PFD0 = 14,
  SYS_PLL3_PFD1 = 15,
  SYS_PLL3_PFD2 = 16,
  SYS_PLL3_PFD3 = 17,
  AUDIO_PLL_OUT = 18,
};

/* LUT mapping root -> { MUX 0, MUX 1, MUX 2, MUX 3 } clock sources. */

static const int g_ccm_root_mux[CCM_CR_COUNT][ROOT_MUX_MAX] =
{
  { OSC_RC_24M, OSC_RC_400M, ARM_PLL_OUT, SYS_PLL3_OUT },              /* M7 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, ARM_PLL_OUT },              /* M33 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_OUT, SYS_PLL2_PFD1 },            /* Edgelock */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL2_OUT, SYS_PLL3_PFD2 },            /* Bus_Aon */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL2_OUT, SYS_PLL3_PFD1 },            /* Bus_Wakeup */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, SYS_PLL2_PFD1 },            /* Wakeup_Axi */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL1_DIV5 },           /* Swo_Trace */
  { OSC_RC_24M, OSC_RC_400M, OSC_24M, SYS_PLL3_DIV2 },                 /* M33_Systick */
  { OSC_RC_24M, OSC_RC_400M, OSC_24M, SYS_PLL3_DIV2 },                 /* M7_Systick */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL1_DIV5 },           /* Flexio1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL1_DIV5 },           /* Flexio2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpit3 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lptimer1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lptimer2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lptimer3 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Tpm2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Tpm4 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Tpm5 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Tpm6 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Gpt1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Gpt2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD0, SYS_PLL2_PFD0 },           /* Flexspi1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD2, SYS_PLL2_PFD1 },           /* Flexspi2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL2_OUT, SYS_PLL1_OUT },             /* Flexspi_Slv */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, OSC_24M },                  /* Can1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, OSC_24M },                  /* Can2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, OSC_24M },                  /* Can3 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart0102 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart0304 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart0506 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart0708 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart0910 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpuart1112 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpi2c0102 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpi2c0304 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Lpi2c0506 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD1, SYS_PLL2_OUT },            /* Lpspi0102 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD1, SYS_PLL2_OUT },            /* Lpspi0304 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD1, SYS_PLL2_OUT },            /* Lpspi0506 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* I3c1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* I3c2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL2_PFD2, SYS_PLL1_DIV5 },           /* Usdhc1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL2_PFD2, SYS_PLL1_DIV5 },           /* Usdhc2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_OUT, SYS_PLL2_PFD0 },            /* Semc */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Adc1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL2_PFD3 },           /* Adc2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, SYS_PLL2_PFD3 },            /* Acmp */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Ecat */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Enet */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, SYS_PLL2_PFD3 },            /* Tmr_1588 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_PFD3, SYS_PLL2_PFD1 },           /* Netc */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Mac0 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Mac1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Mac2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Mac3 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Mac4 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Serdes0 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Serdes1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV2, SYS_PLL1_DIV5 },           /* Serdes2 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_OUT, AUDIO_PLL_OUT },            /* Serdes0_1G */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_OUT, AUDIO_PLL_OUT },            /* Serdes1_1G */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_OUT, AUDIO_PLL_OUT },            /* Serdes2_1G */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, SYS_PLL3_PFD1 },            /* Xcelbusx */
  { OSC_RC_24M, OSC_RC_400M, OSC_24M, SYS_PLL3_DIV2 },                 /* Xriocu4 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV5, AUDIO_PLL_OUT },           /* Mctrl */
  { OSC_RC_24M, OSC_RC_400M, AUDIO_PLL_OUT, SYS_PLL3_PFD2 },           /* Sai1 */
  { OSC_RC_24M, OSC_RC_400M, AUDIO_PLL_OUT, SYS_PLL3_PFD2 },           /* Sai2 */
  { OSC_RC_24M, OSC_RC_400M, AUDIO_PLL_OUT, SYS_PLL3_PFD2 },           /* Sai3 */
  { OSC_RC_24M, OSC_RC_400M, AUDIO_PLL_OUT, SYS_PLL3_PFD2 },           /* Sai4 */
  { OSC_RC_24M, OSC_RC_400M, AUDIO_PLL_OUT, SYS_PLL3_PFD2 },           /* Spdif */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_OUT, AUDIO_PLL_OUT },            /* Asrc */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, AUDIO_PLL_OUT },           /* Mic */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL3_DIV2, SYS_PLL1_DIV2 },           /* Cko1 */
  { OSC_RC_24M, OSC_RC_400M, SYS_PLL1_DIV5, ARM_PLL_OUT },             /* Cko2 */
};

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_CCM_H */
