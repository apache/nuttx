/****************************************************************************
 * arch/arm/src/am67/am67_mcspi.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_MCSPI_H
#define __ARCH_ARM_SRC_AM67_AM67_MCSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MCU_MCSPI0 on Gemstone O1 (MCU domain) */

#define AM67_MCSPI0_BASE           0x04b00000u
#define AM67_MCSPI0_IRQ            207

/* K3 MCU_MCSPI0 register map (TRM offsets).  The peripheral prepends an
 * HL header block (HL_REV/HL_HWINFO/HL_SYSCONFIG) at 0x000-0x01f; the
 * functional registers therefore start at 0x100, not 0x000 as in the
 * older OMAP2 map.
 */

/* HL header registers (0x000-0x01f) */

#define AM67_MCSPI_HL_SYSCONFIG    0x010

/* HL_SYSCONFIG bits */

#define AM67_MCSPI_HL_SYSCONFIG_NOIDLE             (1u << 2)

/* Functional registers start at 0x100 */

#define AM67_MCSPI_REVISION        0x100
#define AM67_MCSPI_SYSCONFIG       0x110
#define AM67_MCSPI_SYSSTATUS       0x114
#define AM67_MCSPI_IRQSTATUS       0x118
#define AM67_MCSPI_IRQENABLE       0x11c
#define AM67_MCSPI_WAKEUPENABLE    0x120
#define AM67_MCSPI_SYST            0x124
#define AM67_MCSPI_MODULCTRL       0x128
#define AM67_MCSPI_CHCONF0         0x12c
#define AM67_MCSPI_CHSTAT0         0x130
#define AM67_MCSPI_CHCTRL0         0x134
#define AM67_MCSPI_TX0             0x138
#define AM67_MCSPI_RX0             0x13c

#define AM67_MCSPI_CH_OFFSET(n)    ((uint32_t)(n) * 0x14u)

#define AM67_MCSPI_SYSCONFIG_AUTOIDLE           (1u << 0)
#define AM67_MCSPI_SYSCONFIG_SOFTRESET          (1u << 1)
#define AM67_MCSPI_SYSCONFIG_SIDLEMODE_SMART    (2u << 3)
#define AM67_MCSPI_SYSCONFIG_SIDLEMODE_NO       (1u << 3)
#define AM67_MCSPI_SYSCONFIG_CLKACT_BOTH        (3u << 8)
#define AM67_MCSPI_SYSSTATUS_RESETDONE          (1u << 0)

#define AM67_MCSPI_MODULCTRL_SINGLE (1u << 0)
#define AM67_MCSPI_MODULCTRL_MS     (1u << 2)

#define AM67_MCSPI_CHCONF_PHA       (1u << 0)
#define AM67_MCSPI_CHCONF_POL       (1u << 1)
#define AM67_MCSPI_CHCONF_CLKD_SHIFT 2
#define AM67_MCSPI_CHCONF_CLKD_MASK (0x0fu << AM67_MCSPI_CHCONF_CLKD_SHIFT)
#define AM67_MCSPI_CHCONF_EPOL      (1u << 6)
#define AM67_MCSPI_CHCONF_WL_SHIFT  7
#define AM67_MCSPI_CHCONF_WL_MASK   (0x1fu << AM67_MCSPI_CHCONF_WL_SHIFT)
#define AM67_MCSPI_CHCONF_TRM_RX    (1u << 12)
#define AM67_MCSPI_CHCONF_TRM_TX    (1u << 13)
#define AM67_MCSPI_CHCONF_DPE0      (1u << 16)
#define AM67_MCSPI_CHCONF_DPE1      (1u << 17)
#define AM67_MCSPI_CHCONF_IS        (1u << 18)
#define AM67_MCSPI_CHCONF_FORCE           (1u << 20)
#define AM67_MCSPI_CHCONF_SPIENSLV_SHIFT  21
#define AM67_MCSPI_CHCONF_SPIENSLV_MASK   (3u << AM67_MCSPI_CHCONF_SPIENSLV_SHIFT)
#define AM67_MCSPI_CHCONF_CLKG            (1u << 29)

#define AM67_MCSPI_CHSTAT_RXS       (1u << 0)
#define AM67_MCSPI_CHSTAT_TXS       (1u << 1)
#define AM67_MCSPI_CHSTAT_EOT       (1u << 2)

#define AM67_MCSPI_CHCTRL_EN        (1u << 0)
#define AM67_MCSPI_CHCTRL_EXTCLK_SHIFT 8
#define AM67_MCSPI_CHCTRL_EXTCLK_MASK  (0xffu << AM67_MCSPI_CHCTRL_EXTCLK_SHIFT)

/* Functional clock (MCU_PLL0 path); the divider is applied in the driver */

#ifndef CONFIG_AM67_MCSPI0_FCLK
#  define CONFIG_AM67_MCSPI0_FCLK  48000000
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void am67_spiinitialize(void);
FAR struct spi_dev_s *am67_spibus_initialize(int port);
void am67_mcspi_board_select(FAR struct spi_dev_s *dev, uint8_t channel,
                             bool selected);

#endif /* __ARCH_ARM_SRC_AM67_AM67_MCSPI_H */
