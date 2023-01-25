/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#  include "hardware/rt117x/imxrt117x_gpio.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO1                     0      /* Port 1 index */
#define GPIO2                     1      /* Port 2 index */
#define GPIO3                     2      /* Port 3 index */
#define GPIO4                     3      /* Port 4 index */
#define GPIO5                     4      /* Port 5 index */
#if IMXRT_GPIO_NPORTS > 5
#define GPIO6                     5      /* Port 6 index */
#define GPIO7                     6      /* Port 7 index */
#define GPIO8                     7      /* Port 8 index */
#define GPIO9                     8      /* Port 9 index */
#endif
#if IMXRT_GPIO_NPORTS > 9
#define GPIO10                    9      /* Port 10 index */
#define GPIO11                   10      /* Port 11 index */
#define GPIO12                   11      /* Port 12 index */
#define GPIO13                   12      /* Port 13 index */
#endif
#define IMXRT_GPIO_NPINS         32      /* Up to 32 pins per port */

/* Register bit definitions *************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)              (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* GPIO interrupt configuration register 1/2 */

#define GPIO_ICR_INDEX(n)        (((n) >> 4) & 1)
#define GPIO_ICR_OFFSET(n)       (IMXRT_GPIO_ICR1_OFFSET + (GPIO_ICR_INDEX(n) << 2))

#define GPIO_ICR_LOWLEVEL        0          /* Interrupt is low-level sensitive */
#define GPIO_ICR_HIGHLEVEL       1          /* Interrupt is high-level sensitive */
#define GPIO_ICR_RISINGEDGE      2          /* Interrupt is rising-edge sensitive */
#define GPIO_ICR_FALLINGEDGE     3          /* Interrupt is falling-edge sensitive */

#define GPIO_ICR_SHIFT(n)        (((n) & 15) << 1)
#define GPIO_ICR_MASK(n)         (3 << GPIO_ICR_SHIFT(n))
#define GPIO_ICR(i,n)            ((uint32_t)(i) << GPIO_ICR_SHIFT(n))

#define GPIO_EDGE_MASK(n)         (1 << n)
#define GPIO_EDGE(i,n)            ((uint32_t)(i) << n)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPIO_H */
