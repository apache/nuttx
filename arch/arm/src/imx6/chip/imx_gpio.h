/************************************************************************************
 * arch/arm/src/imx6/imx_gpio.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPIO_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define GPIO1                    0       /* Port 1 index */
#define GPIO2                    1       /* Port 2 index */
#define GPIO3                    2       /* Port 3 index */
#define GPIO4                    3       /* Port 4 index */
#define GPIO5                    4       /* Port 5 index */
#define GPIO6                    5       /* Port 6 index */
#define GPIO7                    6       /* Port 7 index */

/* GPIO Register Offsets ************************************************************/

#define GPIO_DR_OFFSET           0x0000  /* Data Register */
#define GPIO_GDIR_OFFSET         0x0004  /* Data Direction Register */
#define GPIO_PSR_OFFSET          0x0008  /* Pad Status Register */
#define GPIO_ICR1_OFFSET         0x000c  /* Interrupt Configuration Register 1 */
#define GPIO_ICR2_OFFSET         0x0010  /* Interrupt Configuration Register 2 */
#define GPIO_IMR_OFFSET          0x0014  /* Interrupt Mask Register */
#define GPIO_ISR_OFFSET          0x0018  /* Interrupt Status Register */
#define GPIO_EDGE_OFFSET         0x001c  /* Interrupt Status Register */

/* GPIO Register Addresses **********************************************************/

#define GPIO1_DR                 (IMX_GPIO1_VBASE+GPIO_DR_OFFSET)
#define GPIO1_GDIR               (IMX_GPIO1_VBASE+GPIO_GDIR_OFFSET)
#define GPIO1_PSR                (IMX_GPIO1_VBASE+GPIO_PSR_OFFSET)
#define GPIO1_ICR1               (IMX_GPIO1_VBASE+GPIO_ICR1_OFFSET)
#define GPIO1_ICR2               (IMX_GPIO1_VBASE+GPIO_ICR2_OFFSET)
#define GPIO1_IMR                (IMX_GPIO1_VBASE+GPIO_IMR_OFFSET)
#define GPIO1_ISR                (IMX_GPIO1_VBASE+GPIO_ISR_OFFSET)
#define GPIO1_EDGE               (IMX_GPIO1_VBASE+GPIO_EDGE_OFFSET)

#define GPIO2_DR                 (IMX_GPIO2_VBASE+GPIO_DR_OFFSET)
#define GPIO2_GDIR               (IMX_GPIO2_VBASE+GPIO_GDIR_OFFSET)
#define GPIO2_PSR                (IMX_GPIO2_VBASE+GPIO_PSR_OFFSET)
#define GPIO2_ICR1               (IMX_GPIO2_VBASE+GPIO_ICR1_OFFSET)
#define GPIO2_ICR2               (IMX_GPIO2_VBASE+GPIO_ICR2_OFFSET)
#define GPIO2_IMR                (IMX_GPIO2_VBASE+GPIO_IMR_OFFSET)
#define GPIO2_ISR                (IMX_GPIO2_VBASE+GPIO_ISR_OFFSET)
#define GPIO2_EDGE               (IMX_GPIO2_VBASE+GPIO_EDGE_OFFSET)

#define GPIO3_DR                 (IMX_GPIO3_VBASE+GPIO_DR_OFFSET)
#define GPIO3_GDIR               (IMX_GPIO3_VBASE+GPIO_GDIR_OFFSET)
#define GPIO3_PSR                (IMX_GPIO3_VBASE+GPIO_PSR_OFFSET)
#define GPIO3_ICR1               (IMX_GPIO3_VBASE+GPIO_ICR1_OFFSET)
#define GPIO3_ICR2               (IMX_GPIO3_VBASE+GPIO_ICR2_OFFSET)
#define GPIO3_IMR                (IMX_GPIO3_VBASE+GPIO_IMR_OFFSET)
#define GPIO3_ISR                (IMX_GPIO3_VBASE+GPIO_ISR_OFFSET)
#define GPIO3_EDGE               (IMX_GPIO3_VBASE+GPIO_EDGE_OFFSET)

#define GPIO4_DR                 (IMX_GPIO4_VBASE+GPIO_DR_OFFSET)
#define GPIO4_GDIR               (IMX_GPIO4_VBASE+GPIO_GDIR_OFFSET)
#define GPIO4_PSR                (IMX_GPIO4_VBASE+GPIO_PSR_OFFSET)
#define GPIO4_ICR1               (IMX_GPIO4_VBASE+GPIO_ICR1_OFFSET)
#define GPIO4_ICR2               (IMX_GPIO4_VBASE+GPIO_ICR2_OFFSET)
#define GPIO4_IMR                (IMX_GPIO4_VBASE+GPIO_IMR_OFFSET)
#define GPIO4_ISR                (IMX_GPIO4_VBASE+GPIO_ISR_OFFSET)
#define GPIO4_EDGE               (IMX_GPIO4_VBASE+GPIO_EDGE_OFFSET)

#define GPIO5_DR                 (IMX_GPIO5_VBASE+GPIO_DR_OFFSET)
#define GPIO5_GDIR               (IMX_GPIO5_VBASE+GPIO_GDIR_OFFSET)
#define GPIO5_PSR                (IMX_GPIO5_VBASE+GPIO_PSR_OFFSET)
#define GPIO5_ICR1               (IMX_GPIO5_VBASE+GPIO_ICR1_OFFSET)
#define GPIO5_ICR2               (IMX_GPIO5_VBASE+GPIO_ICR2_OFFSET)
#define GPIO5_IMR                (IMX_GPIO5_VBASE+GPIO_IMR_OFFSET)
#define GPIO5_ISR                (IMX_GPIO5_VBASE+GPIO_ISR_OFFSET)
#define GPIO5_EDGE               (IMX_GPIO5_VBASE+GPIO_EDGE_OFFSET)

#define GPIO6_DR                 (IMX_GPIO6_VBASE+GPIO_DR_OFFSET)
#define GPIO6_GDIR               (IMX_GPIO6_VBASE+GPIO_GDIR_OFFSET)
#define GPIO6_PSR                (IMX_GPIO6_VBASE+GPIO_PSR_OFFSET)
#define GPIO6_ICR1               (IMX_GPIO6_VBASE+GPIO_ICR1_OFFSET)
#define GPIO6_ICR2               (IMX_GPIO6_VBASE+GPIO_ICR2_OFFSET)
#define GPIO6_IMR                (IMX_GPIO6_VBASE+GPIO_IMR_OFFSET)
#define GPIO6_ISR                (IMX_GPIO6_VBASE+GPIO_ISR_OFFSET)
#define GPIO6_EDGE               (IMX_GPIO6_VBASE+GPIO_EDGE_OFFSET)

#define GPIO7_DR                 (IMX_GPIO7_VBASE+GPIO_DR_OFFSET)
#define GPIO7_GDIR               (IMX_GPIO7_VBASE+GPIO_GDIR_OFFSET)
#define GPIO7_PSR                (IMX_GPIO7_VBASE+GPIO_PSR_OFFSET)
#define GPIO7_ICR1               (IMX_GPIO7_VBASE+GPIO_ICR1_OFFSET)
#define GPIO7_ICR2               (IMX_GPIO7_VBASE+GPIO_ICR2_OFFSET)
#define GPIO7_IMR                (IMX_GPIO7_VBASE+GPIO_IMR_OFFSET)
#define GPIO7_ISR                (IMX_GPIO7_VBASE+GPIO_ISR_OFFSET)
#define GPIO7_EDGE               (IMX_GPIO7_VBASE+GPIO_EDGE_OFFSET)

/* GPIO Register Bit Definitions ****************************************************/

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
#define GPIO_ICR(i,n)            ((uint32_t)(n) << GPIO_ICR_SHIFT(n))

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPIO_H */
