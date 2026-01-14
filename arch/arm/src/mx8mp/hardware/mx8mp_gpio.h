/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_gpio.h
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

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPIO_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mx8mp_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_NPORTS               5      /* Seven total ports */
#define GPIO_NPINS                32     /* Up to 32 pins per port */

/* GPIO Register Offsets ****************************************************/

#define DR_OFFSET                 0x0000  /* Data Register */
#define GDIR_OFFSET               0x0004  /* Data Direction Register */
#define PSR_OFFSET                0x0008  /* Pad Status Register */
#define ICR1_OFFSET               0x000c  /* Interrupt Configuration Register 1 */
#define ICR2_OFFSET               0x0010  /* Interrupt Configuration Register 2 */
#define IMR_OFFSET                0x0014  /* Interrupt Mask Register */
#define ISR_OFFSET                0x0018  /* Interrupt Status Register */
#define EDGE_OFFSET               0x001c  /* Interrupt Status Register */

/* GPIO Register Addresses **************************************************/

#define GPIO_DR(n)                (((n - 1) * 0x10000 + MX8M_GPIO) + DR_OFFSET)
#define GPIO_GDIR(n)              (((n - 1) * 0x10000 + MX8M_GPIO) + GDIR_OFFSET)
#define GPIO_PSR(n)               (((n - 1) * 0x10000 + MX8M_GPIO) + PSR_OFFSET)
#define GPIO_ICR1(n)              (((n - 1) * 0x10000 + MX8M_GPIO) + ICR1_OFFSET)
#define GPIO_ICR2(n)              (((n - 1) * 0x10000 + MX8M_GPIO) + ICR2_OFFSET)
#define GPIO_IMR(n)               (((n - 1) * 0x10000 + MX8M_GPIO) + IMR_OFFSET)
#define GPIO_ISR(n)               (((n - 1) * 0x10000 + MX8M_GPIO) + ISR_OFFSET)
#define GPIO_EDGE(n)              (((n - 1) * 0x10000 + MX8M_GPIO) + EDGE_OFFSET)

/* GPIO Register Bit Definitions ********************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)              (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* GPIO interrupt configuration register 1/2 */

#define ICR_INDEX(n)              (((n) >> 4) & 1)
#define ICR_OFFSET(n)             (ICR1_OFFSET + (ICR_INDEX(n) << 2))

#define ICR_LOW_LEVEL             0          /* Interrupt is low-level sensitive */
#define ICR_HIGH_LEVEL            1          /* Interrupt is high-level sensitive */
#define ICR_RISING_EDGE           2          /* Interrupt is rising-edge sensitive */
#define ICR_FALLING_EDGE          3          /* Interrupt is falling-edge sensitive */

#define ICR_SHIFT(n)              (((n) & 15) << 1)
#define ICR_MASK(n)               (3 << ICR_SHIFT(n))
#define ICR(i,n)                  ((uint32_t)(i) << ICR_SHIFT(n))

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_GPIO_H */
