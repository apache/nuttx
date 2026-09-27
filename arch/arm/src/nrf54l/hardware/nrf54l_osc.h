/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_osc.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_OSC_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_OSC_XOSC32M_INTCAP_OFFSET  0x071c  /* HFXO load capacitance */
#define NRF54L_OSC_PLL_FREQ_OFFSET        0x0800  /* Requested MCU clock */
#define NRF54L_OSC_PLL_CURRENTFREQ_OFFSET 0x0804  /* Current MCU clock */
#define NRF54L_OSC_XOSC32KI_BYPASS_OFFSET 0x0900  /* LFXO external clock */
#define NRF54L_OSC_XOSC32KI_INTCAP_OFFSET 0x0904  /* LFXO load capacitance */
#define NRF54L_OSC_XOSC32KI_STATUS_OFFSET 0x0914  /* LFXO status */

/* Register addresses *******************************************************/

#define NRF54L_OSC_XOSC32M_INTCAP  (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_XOSC32M_INTCAP_OFFSET)
#define NRF54L_OSC_PLL_FREQ        (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_PLL_FREQ_OFFSET)
#define NRF54L_OSC_PLL_CURRENTFREQ (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_PLL_CURRENTFREQ_OFFSET)
#define NRF54L_OSC_XOSC32KI_BYPASS (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_XOSC32KI_BYPASS_OFFSET)
#define NRF54L_OSC_XOSC32KI_INTCAP (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_XOSC32KI_INTCAP_OFFSET)
#define NRF54L_OSC_XOSC32KI_STATUS (NRF54L_OSCILLATORS_BASE + NRF54L_OSC_XOSC32KI_STATUS_OFFSET)

/* Register bit definitions *************************************************/

/* XOSC32M_INTCAP Register */

#define OSC_XOSC32M_INTCAP_SHIFT       (0)
#define OSC_XOSC32M_INTCAP_MASK        (0x3f << OSC_XOSC32M_INTCAP_SHIFT)

/* PLL_FREQ Register */

#define OSC_PLL_FREQ_SHIFT            (0)
#define OSC_PLL_FREQ_MASK             (0x3 << OSC_PLL_FREQ_SHIFT)
#  define OSC_PLL_FREQ_128M           (1 << OSC_PLL_FREQ_SHIFT)
#  define OSC_PLL_FREQ_64M            (3 << OSC_PLL_FREQ_SHIFT)

/* PLL_CURRENTFREQ Register */

#define OSC_PLL_CURRENTFREQ_SHIFT     (0)
#define OSC_PLL_CURRENTFREQ_MASK      (0x3 << OSC_PLL_CURRENTFREQ_SHIFT)
#  define OSC_PLL_CURRENTFREQ_128M    (1 << OSC_PLL_CURRENTFREQ_SHIFT)
#  define OSC_PLL_CURRENTFREQ_64M     (3 << OSC_PLL_CURRENTFREQ_SHIFT)

/* XOSC32KI Registers */

#define OSC_XOSC32KI_BYPASS           (1 << 0)
#define OSC_XOSC32KI_INTCAP_SHIFT     (0)
#define OSC_XOSC32KI_INTCAP_MASK      (0x1f << OSC_XOSC32KI_INTCAP_SHIFT)
#define OSC_XOSC32KI_STATUS_RUNNING   (1 << 2)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_OSC_H */
