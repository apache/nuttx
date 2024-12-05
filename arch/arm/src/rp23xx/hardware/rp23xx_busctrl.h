/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_busctrl.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BUSCTRL_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BUSCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_BUSCTRL_BUS_PRIORITY_OFFSET      0x00000000
#define RP23XX_BUSCTRL_BUS_PRIORITY_ACK_OFFSET  0x00000004
#define RP23XX_BUSCTRL_PERFCTR_EN_OFFSET        0x00000008
#define RP23XX_BUSCTRL_PERFCTR_OFFSET(n)        ((n) * 8 + 0x00000c)
#define RP23XX_BUSCTRL_PERFSEL_OFFSET(n)        ((n) * 8 + 0x000010)

/* Register definitions *****************************************************/

#define RP23XX_BUSCTRL_BUS_PRIORITY      (RP23XX_BUSCTRL_BASE + RP23XX_BUSCTRL_BUS_PRIORITY_OFFSET)
#define RP23XX_BUSCTRL_BUS_PRIORITY_ACK  (RP23XX_BUSCTRL_BASE + RP23XX_BUSCTRL_BUS_PRIORITY_ACK_OFFSET)
#define RP23XX_BUSCTRL_PERFCTR_EN        (RP23XX_BUSCTRL_BASE + RP23XX_BUSCTRL_PERFCTR_EN_OFFSET)
#define RP23XX_BUSCTRL_PERFCTR(n)        (RP23XX_BUSCTRL_BASE + RP23XX_BUSCTRL_PERFCTR_OFFSET(n))
#define RP23XX_BUSCTRL_PERFSEL(n)        (RP23XX_BUSCTRL_BASE + RP23XX_BUSCTRL_PERFSEL_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP23XX_BUSCTRL_BUS_PRIORITY_MASK    0x00001111
#define RP23XX_BUSCTRL_BUS_PRIORITY_DMA_W   (1 << 12)
#define RP23XX_BUSCTRL_BUS_PRIORITY_DMA_R   (1 << 8)
#define RP23XX_BUSCTRL_BUS_PRIORITY_PROC1   (1 << 4)
#define RP23XX_BUSCTRL_BUS_PRIORITY_PROC0   (1 << 0)
#define RP23XX_BUSCTRL_BUS_PRIORITY_ACK     (1 << 0)
#define RP23XX_BUSCTRL_PERFCTR_EN           (1 << 0)
#define RP23XX_BUSCTRL_PERFCTR_MASK         0x00ffffff
#define RP23XX_BUSCTRL_PERFSEL_MASK         0x0000007f

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_BUSCTRL_H */
