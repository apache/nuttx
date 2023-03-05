/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_osc.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_OSC_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_OSC_XOSC32MCAPS_OFFSET      0x5C4  /* Programmable capacitance of XC1 and XC2 */
#define NRF53_OSC_XOSC32KI_BYPASS_OFFSET  0x6C0  /* Enable or disable bypass of LFCLK crystal oscillator with external clock source */
#define NRF53_OSC_XOSC32KI_INTCAP_OFFSET  0x6D0  /* Control usage of internal load capacitors */

/* Register definitions *****************************************************/

#define NRF53_OSC_XOSC32MCAPS       (NRF53_OSCILLATORS_BASE + NRF53_OSC_XOSC32MCAPS_OFFSET)
#define NRF53_OSC_XOSC32KI_BYPASS   (NRF53_OSCILLATORS_BASE + NRF53_OSC_XOSC32KI_BYPASS_OFFSET)
#define NRF53_OSC_XOSC32KI_INTCAP   (NRF53_OSCILLATORS_BASE + NRF53_OSC_XOSC32KI_INTCAP_OFFSET)

/* Register bit definitions *************************************************/

#define OSC_XOSC32KI_INTCAP_SHIFT      (0)
#define OSC_XOSC32KI_INTCAP_MASK       (0x3 << OSC_XOSC32KI_INTCAP_SHIFT)
#  define OSC_XOSC32KI_INTCAP_EXT      (0x0 << OSC_XOSC32KI_INTCAP_SHIFT)
#  define OSC_XOSC32KI_INTCAP_C6PF     (0x1 << OSC_XOSC32KI_INTCAP_SHIFT)
#  define OSC_XOSC32KI_INTCAP_C7PF     (0x2 << OSC_XOSC32KI_INTCAP_SHIFT)
#  define OSC_XOSC32KI_INTCAP_C9PF     (0x3 << OSC_XOSC32KI_INTCAP_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_OSC_H */
