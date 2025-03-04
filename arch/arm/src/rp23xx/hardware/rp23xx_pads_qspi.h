/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_pads_qspi.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PADS_QSPI_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PADS_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_PADS_QSPI_VOLTAGE_SELECT_OFFSET  0x00000000
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_OFFSET  0x00000004
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_OFFSET   0x00000008
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_OFFSET   0x0000000c
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_OFFSET   0x00000010
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_OFFSET   0x00000014
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_OFFSET    0x00000018

/* Register definitions *****************************************************/

#define RP23XX_PADS_QSPI_VOLTAGE_SELECT  (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_VOLTAGE_SELECT_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK  (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0   (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SD0_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1   (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SD1_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2   (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SD2_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3   (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SD3_OFFSET)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS    (RP23XX_PADS_QSPI_BASE + RP23XX_PADS_QSPI_GPIO_QSPI_SS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_PADS_QSPI_VOLTAGE_SELECT             (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_MASK        0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_ISO         (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_OD          (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_IE          (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_DRIVE_MASK  0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_PUE         (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_PDE         (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_SCHMITT     (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SCLK_SLEWFAST    (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_MASK         0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_ISO          (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_OD           (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_IE           (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_DRIVE_MASK   0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_PUE          (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_PDE          (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_SCHMITT      (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD0_SLEWFAST     (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_MASK         0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_ISO          (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_OD           (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_IE           (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_DRIVE_MASK   0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_PUE          (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_PDE          (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_SCHMITT      (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD1_SLEWFAST     (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_MASK         0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_ISO          (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_OD           (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_IE           (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_DRIVE_MASK   0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_PUE          (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_PDE          (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_SCHMITT      (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD2_SLEWFAST     (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_MASK         0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_ISO          (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_OD           (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_IE           (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_DRIVE_MASK   0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_PUE          (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_PDE          (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_SCHMITT      (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SD3_SLEWFAST     (1 << 0)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_MASK          0x000001ff
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_ISO           (1 << 8)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_OD            (1 << 7)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_IE            (1 << 6)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_DRIVE_MASK    0x00000030
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_PUE           (1 << 3)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_PDE           (1 << 2)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_SCHMITT       (1 << 1)
#define RP23XX_PADS_QSPI_GPIO_QSPI_SS_SLEWFAST      (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PADS_QSPI_H */
