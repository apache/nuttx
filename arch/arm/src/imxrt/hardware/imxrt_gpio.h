/********************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_GPIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_gpio.h"
#else
#  error Unrecognized i.MX RT architecture
#endif

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

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
#define IMXRT_GPIO_NPINS         32      /* Up to 32 pins per port */

/* Register bit definitions *****************************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)              (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* GPIO interrupt configuration register 1/2 */

#define GPIO_ICR_INDEX(n)        (((n) >> 4) & 1)
#define GPIO_ICR_OFFSET(n)       (GPIO_ICR1_OFFSET + (GPIO_ICR_INDEX(n) << 2))

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
