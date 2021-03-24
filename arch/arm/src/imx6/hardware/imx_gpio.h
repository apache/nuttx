/****************************************************************************
 * arch/arm/src/imx6/hardware/imx_gpio.h
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
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual",
 *   Document Number IMX6DQRM, Rev. 3, 07/2015, FreeScale.
 */

#ifndef __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_GPIO_H
#define __ARCH_ARM_SRC_IMX6_HARDWARE_IMX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO1                     0      /* Port 1 index */
#define GPIO2                     1      /* Port 2 index */
#define GPIO3                     2      /* Port 3 index */
#define GPIO4                     3      /* Port 4 index */
#define GPIO5                     4      /* Port 5 index */
#define GPIO6                     5      /* Port 6 index */
#define GPIO7                     6      /* Port 7 index */

#define IMX_GPIO_NPORTS           7      /* Seven total ports */
#define IMX_GPIO_NPINS           32      /* Up to 32 pins per port */

/* GPIO Register Offsets ****************************************************/

#define IMX_GPIO_DR_OFFSET       0x0000  /* Data Register */
#define IMX_GPIO_GDIR_OFFSET     0x0004  /* Data Direction Register */
#define IMX_GPIO_PSR_OFFSET      0x0008  /* Pad Status Register */
#define IMX_GPIO_ICR1_OFFSET     0x000c  /* Interrupt Configuration Register 1 */
#define IMX_GPIO_ICR2_OFFSET     0x0010  /* Interrupt Configuration Register 2 */
#define IMX_GPIO_IMR_OFFSET      0x0014  /* Interrupt Mask Register */
#define IMX_GPIO_ISR_OFFSET      0x0018  /* Interrupt Status Register */
#define IMX_GPIO_EDGE_OFFSET     0x001c  /* Interrupt Status Register */

/* GPIO Register Addresses **************************************************/

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

/* GPIO Register Bit Definitions ********************************************/

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
