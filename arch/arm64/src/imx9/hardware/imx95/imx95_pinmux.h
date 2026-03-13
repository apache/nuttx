/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx95/imx95_pinmux.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_IMX95_PINMUX_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_IMX95_PINMUX_H

/* SD1 / USDHC1 pin configurations (ALT0, no daisy chain)
 *
 * IOMUX_PADCFG(mux_ctl_reg, mux_mode, daisy_reg, daisy_val, pad_ctl_reg)
 *
 * Addresses: IMX9_IOMUXC1_BASE (0x443c0000) + offset
 *   MUX_CTL: SD1_CLK=0x0128, SD1_CMD=0x012c, SD1_DATA0-7=0x0130-0x014c,
 *            SD1_STROBE=0x0150
 *   PAD_CTL: SD1_CLK=0x032c, SD1_CMD=0x0330, SD1_DATA0-7=0x0334-0x0350,
 *            SD1_STROBE=0x0354
 */

#define IOMUXC_PAD_SD1_CLK_USDHC1_CLK       IOMUX_PADCFG(0x443c0128, 0x0, 0x00000000, 0x00000000, 0x443c032c)
#define IOMUXC_PAD_SD1_CMD_USDHC1_CMD       IOMUX_PADCFG(0x443c012c, 0x0, 0x00000000, 0x00000000, 0x443c0330)
#define IOMUXC_PAD_SD1_DATA0_USDHC1_DATA0   IOMUX_PADCFG(0x443c0130, 0x0, 0x00000000, 0x00000000, 0x443c0334)
#define IOMUXC_PAD_SD1_DATA1_USDHC1_DATA1   IOMUX_PADCFG(0x443c0134, 0x0, 0x00000000, 0x00000000, 0x443c0338)
#define IOMUXC_PAD_SD1_DATA2_USDHC1_DATA2   IOMUX_PADCFG(0x443c0138, 0x0, 0x00000000, 0x00000000, 0x443c033c)
#define IOMUXC_PAD_SD1_DATA3_USDHC1_DATA3   IOMUX_PADCFG(0x443c013c, 0x0, 0x00000000, 0x00000000, 0x443c0340)
#define IOMUXC_PAD_SD1_DATA4_USDHC1_DATA4   IOMUX_PADCFG(0x443c0140, 0x0, 0x00000000, 0x00000000, 0x443c0344)
#define IOMUXC_PAD_SD1_DATA5_USDHC1_DATA5   IOMUX_PADCFG(0x443c0144, 0x0, 0x00000000, 0x00000000, 0x443c0348)
#define IOMUXC_PAD_SD1_DATA6_USDHC1_DATA6   IOMUX_PADCFG(0x443c0148, 0x0, 0x00000000, 0x00000000, 0x443c034c)
#define IOMUXC_PAD_SD1_DATA7_USDHC1_DATA7   IOMUX_PADCFG(0x443c014c, 0x0, 0x00000000, 0x00000000, 0x443c0350)
#define IOMUXC_PAD_SD1_STROBE_USDHC1_STROBE IOMUX_PADCFG(0x443c0150, 0x0, 0x00000000, 0x00000000, 0x443c0354)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_IMX95_PINMUX_H */
