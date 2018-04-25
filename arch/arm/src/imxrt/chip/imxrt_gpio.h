/********************************************************************************************
 * arch/arm/src/imxrt/imxrt_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_GPIO_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define GPIO1                     0      /* Port 1 index */
#define GPIO2                     1      /* Port 2 index */
#define GPIO3                     2      /* Port 3 index */
#define GPIO4                     3      /* Port 4 index */
#define GPIO5                     4      /* Port 5 index */

#define IMXRT_GPIO_NPORTS         5      /* Five total ports */
#define IMXRT_GPIO_NPINS         32      /* Up to 32 pins per port */

/* Register offsets *************************************************************************/

#define IMXRT_GPIO_DR_OFFSET     0x0000  /* GPIO data register */
#define IMXRT_GPIO_GDIR_OFFSET   0x0004  /* GPIO direction register */
#define IMXRT_GPIO_PSR_OFFSET    0x0008  /* GPIO pad status register */
#define IMXRT_GPIO_ICR1_OFFSET   0x000c  /* GPIO interrupt configuration register1 */
#define IMXRT_GPIO_ICR2_OFFSET   0x0010  /* GPIO interrupt configuration register2 */
#define IMXRT_GPIO_IMR_OFFSET    0x0014  /* GPIO interrupt mask register */
#define IMXRT_GPIO_ISR_OFFSET    0x0018  /* GPIO interrupt status register */
#define IMXRT_GPIO_EDGE_OFFSET   0x001c  /* GPIO edge select register */

/* Register addresses ***********************************************************************/

#define IMXRT_GPIO1_DR           (IMXRT_GPIO1_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO1_GDIR         (IMXRT_GPIO1_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO1_PSR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO1_ICR1         (IMXRT_GPIO1_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO1_ICR2         (IMXRT_GPIO1_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO1_IMR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO1_ISR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO1_EDGE         (IMXRT_GPIO1_BASE + IMXRT_GPIO_EDGE_OFFSET)

#define IMXRT_GPIO2_DR           (IMXRT_GPIO2_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO2_GDIR         (IMXRT_GPIO2_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO2_PSR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO2_ICR1         (IMXRT_GPIO2_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO2_ICR2         (IMXRT_GPIO2_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO2_IMR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO2_ISR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO2_EDGE         (IMXRT_GPIO2_BASE + IMXRT_GPIO_EDGE_OFFSET)

#define IMXRT_GPIO3_DR           (IMXRT_GPIO3_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO3_GDIR         (IMXRT_GPIO3_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO3_PSR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO3_ICR1         (IMXRT_GPIO3_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO3_ICR2         (IMXRT_GPIO3_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO3_IMR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO3_ISR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO3_EDGE         (IMXRT_GPIO3_BASE + IMXRT_GPIO_EDGE_OFFSET)

#define IMXRT_GPIO4_DR           (IMXRT_GPIO4_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO4_GDIR         (IMXRT_GPIO4_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO4_PSR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO4_ICR1         (IMXRT_GPIO4_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO4_ICR2         (IMXRT_GPIO4_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO4_IMR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO4_ISR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO4_EDGE         (IMXRT_GPIO4_BASE + IMXRT_GPIO_EDGE_OFFSET)

#define IMXRT_GPIO5_DR           (IMXRT_GPIO5_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO5_GDIR         (IMXRT_GPIO5_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO5_PSR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO5_ICR1         (IMXRT_GPIO5_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO5_ICR2         (IMXRT_GPIO5_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO5_IMR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO5_ISR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO5_EDGE         (IMXRT_GPIO5_BASE + IMXRT_GPIO_EDGE_OFFSET)

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

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_GPIO_H */
