/************************************************************************************
 * arch/arm/src/imx6/hardware/imx_gpio.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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

#ifndef __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_GPIO_H
#define __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define GPIO1                     0      /* Port 1 index */
#define GPIO2                     1      /* Port 2 index */
#define GPIO3                     2      /* Port 3 index */
#define GPIO4                     3      /* Port 4 index */
#define GPIO5                     4      /* Port 5 index */
#define GPIO6                     5      /* Port 6 index */
#define GPIO7                     6      /* Port 7 index */

#define IMX_GPIO_NPORTS           7      /* Seven total ports */
#define IMX_GPIO_NPINS           32      /* Up to 32 pins per port */

/* GPIO Register Offsets ************************************************************/

#define IMX_GPIO_DR_OFFSET       0x0000  /* Data Register */
#define IMX_GPIO_GDIR_OFFSET     0x0004  /* Data Direction Register */
#define IMX_GPIO_PSR_OFFSET      0x0008  /* Pad Status Register */
#define IMX_GPIO_ICR1_OFFSET     0x000c  /* Interrupt Configuration Register 1 */
#define IMX_GPIO_ICR2_OFFSET     0x0010  /* Interrupt Configuration Register 2 */
#define IMX_GPIO_IMR_OFFSET      0x0014  /* Interrupt Mask Register */
#define IMX_GPIO_ISR_OFFSET      0x0018  /* Interrupt Status Register */
#define IMX_GPIO_EDGE_OFFSET     0x001c  /* Interrupt Status Register */

/* GPIO Register Addresses **********************************************************/

#define IMX_GPIO_DR(n)           (IMX_GPIO_VBASE(n)+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO_GDIR(n)         (IMX_GPIO_VBASE(n)+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO_PSR(n)          (IMX_GPIO_VBASE(n)+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO_ICR1(n)         (IMX_GPIO_VBASE(n)+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO_ICR2(n)         (IMX_GPIO_VBASE(n)+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO_IMR(n)          (IMX_GPIO_VBASE(n)+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO_ISR(n)          (IMX_GPIO_VBASE(n)+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO_EDGE(n)         (IMX_GPIO_VBASE(n)+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO1_DR             (IMX_GPIO1_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO1_GDIR           (IMX_GPIO1_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO1_PSR            (IMX_GPIO1_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO1_ICR1           (IMX_GPIO1_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO1_ICR2           (IMX_GPIO1_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO1_IMR            (IMX_GPIO1_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO1_ISR            (IMX_GPIO1_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO1_EDGE           (IMX_GPIO1_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO2_DR             (IMX_GPIO2_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO2_GDIR           (IMX_GPIO2_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO2_PSR            (IMX_GPIO2_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO2_ICR1           (IMX_GPIO2_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO2_ICR2           (IMX_GPIO2_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO2_IMR            (IMX_GPIO2_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO2_ISR            (IMX_GPIO2_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO2_EDGE           (IMX_GPIO2_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO3_DR             (IMX_GPIO3_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO3_GDIR           (IMX_GPIO3_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO3_PSR            (IMX_GPIO3_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO3_ICR1           (IMX_GPIO3_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO3_ICR2           (IMX_GPIO3_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO3_IMR            (IMX_GPIO3_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO3_ISR            (IMX_GPIO3_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO3_EDGE           (IMX_GPIO3_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO4_DR             (IMX_GPIO4_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO4_GDIR           (IMX_GPIO4_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO4_PSR            (IMX_GPIO4_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO4_ICR1           (IMX_GPIO4_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO4_ICR2           (IMX_GPIO4_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO4_IMR            (IMX_GPIO4_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO4_ISR            (IMX_GPIO4_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO4_EDGE           (IMX_GPIO4_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO5_DR             (IMX_GPIO5_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO5_GDIR           (IMX_GPIO5_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO5_PSR            (IMX_GPIO5_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO5_ICR1           (IMX_GPIO5_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO5_ICR2           (IMX_GPIO5_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO5_IMR            (IMX_GPIO5_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO5_ISR            (IMX_GPIO5_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO5_EDGE           (IMX_GPIO5_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO6_DR             (IMX_GPIO6_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO6_GDIR           (IMX_GPIO6_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO6_PSR            (IMX_GPIO6_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO6_ICR1           (IMX_GPIO6_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO6_ICR2           (IMX_GPIO6_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO6_IMR            (IMX_GPIO6_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO6_ISR            (IMX_GPIO6_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO6_EDGE           (IMX_GPIO6_VBASE+IMX_GPIO_EDGE_OFFSET)

#define IMX_GPIO7_DR             (IMX_GPIO7_VBASE+IMX_GPIO_DR_OFFSET)
#define IMX_GPIO7_GDIR           (IMX_GPIO7_VBASE+IMX_GPIO_GDIR_OFFSET)
#define IMX_GPIO7_PSR            (IMX_GPIO7_VBASE+IMX_GPIO_PSR_OFFSET)
#define IMX_GPIO7_ICR1           (IMX_GPIO7_VBASE+IMX_GPIO_ICR1_OFFSET)
#define IMX_GPIO7_ICR2           (IMX_GPIO7_VBASE+IMX_GPIO_ICR2_OFFSET)
#define IMX_GPIO7_IMR            (IMX_GPIO7_VBASE+IMX_GPIO_IMR_OFFSET)
#define IMX_GPIO7_ISR            (IMX_GPIO7_VBASE+IMX_GPIO_ISR_OFFSET)
#define IMX_GPIO7_EDGE           (IMX_GPIO7_VBASE+IMX_GPIO_EDGE_OFFSET)

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
#define GPIO_ICR(i,n)            ((uint32_t)(i) << GPIO_ICR_SHIFT(n))

#endif /* __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_GPIO_H */
