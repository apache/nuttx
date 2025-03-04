/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPIO_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_IMX93)
#  include "hardware/imx93/imx93_gpio.h"
#else
#  error Unrecognized i.MX9 architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO1                           0      /* Port 1 index */
#define GPIO2                           1      /* Port 2 index */
#define GPIO3                           2      /* Port 3 index */
#define GPIO4                           3      /* Port 4 index */
#define GPIO5                           4      /* Port 5 index */
#define GPIO6                           5      /* Port 6 index */
#define GPIO7                           6      /* Port 7 index */
#define GPIO8                           7      /* Port 8 index */
#define GPIO9                           8      /* Port 9 index */
#define GPIO10                          9      /* Port 10 index */
#define GPIO11                          10     /* Port 11 index */
#define GPIO12                          11     /* Port 12 index */
#define GPIO13                          12     /* Port 13 index */

#define IMX9_GPIO_NPINS                 32     /* Up to 32 pins per port */

/* Register bit definitions *************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)                     (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* ICRN Register */

#define IMX9_GPIO_ICRN_ISF              (1 << 24) /* Bit 24: Interrupt Status Flag */
#define IMX9_GPIO_ICRN_LK               (1 << 23) /* Bit 23: Lock Register */
#define IMX9_GPIO_ICRN_IRQS             (1 << 20) /* Bit 20: Configures the selected interrupt, or DMA request. */
#define IMX9_GPIO_ICRN_SHIFT            (16)      /* Bits 16-19:  Interrupt Configuration */
#define IMX9_GPIO_ICRN_MASK             (0xf << IMX9_GPIO_ICRN_SHIFT)
#  define IMX9_GPIO_ICRN_DISABLED       (0 << IMX9_GPIO_ICRN_SHIFT)  /* Interrupt Status Flag (ISF) is disabled */
#  define IMX9_GPIO_ICRN_DMARISING      (1 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on rising edge */
#  define IMX9_GPIO_ICRN_DMAFALLING     (2 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on falling edge */
#  define IMX9_GPIO_ICRN_DMABOTH        (3 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on either edge */
#  define IMX9_GPIO_ICRN_ISFRISING      (5 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag sets on rising edge */
#  define IMX9_GPIO_ICRN_ISFFALLING     (6 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag sets on falling edge */
#  define IMX9_GPIO_ICRN_ISFBOTH        (7 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag sets on either edge */
#  define IMX9_GPIO_ICRN_ZERO           (8 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag and Interrupt when logic 0 */
#  define IMX9_GPIO_ICRN_RISING         (9 << IMX9_GPIO_ICRN_SHIFT)  /* ISF flag and Interrupt on rising-edge */
#  define IMX9_GPIO_ICRN_FALLING        (10 << IMX9_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt on falling-edge */
#  define IMX9_GPIO_ICRN_BOTH           (11 << IMX9_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt on either edge */
#  define IMX9_GPIO_ICRN_ONE            (12 << IMX9_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt when logic 1 */

/* Global Interrupt Control Low Register */

#define IMX9_GPIO_GICLR_GIWD_SHIFT      (0)       /* Bits 0-15: Global Interrupt Write Data */
#define IMX9_GPIO_GICLR_GIWD_MASK       (0xffff << IMX9_GPIO_GICLR_GIWD_SHIFT)
#  define IMX9_GPIO_GICLR_GIWD_PIN(n)   ((uint32_t)(n) << IMX9_GPIO_GICLR_GIWD_SHIFT) /* Pin n=0..15 */

#define IMX9_GPIO_GICLR_GIWE_SHIFT      (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define IMX9_GPIO_GICLR_GIWE_MASK       (0xffff << IMX9_GPIO_GICLR_GIWE_SHIFT)
#  define IMX9_GPIO_GICLR_GIWE_PIN(n)   ((uint32_t)(n) << IMX9_GPIO_GICLR_GIWE_SHIFT) /* Pin n=0..15 */

/* Global Interrupt Control High Register */

#define IMX9_GPIO_GICHR_GIWD_SHIFT      (0)       /* Bits 0-15: Global Interrupt Write Data */
#define IMX9_GPIO_GICHR_GIWD_MASK       (0xffff << IMX9_GPIO_GICHR_GIWD_SHIFT)
#  define IMX9_GPIO_GICHR_GIWD_PIN(n)   ((uint32_t)((n) - 16) << IMX9_GPIO_GICHR_GIWD_SHIFT) /* Pin n=16..31 */

#define IMX9_GPIO_GICHR_GIWE_SHIFT      (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define IMX9_GPIO_GICHR_GIWE_MASK       (0xffff << IMX9_GPIO_GICHR_GIWE_SHIFT)
#  define IMX9_GPIO_GICHR_GIWE_PIN(n)   ((uint32_t)((n) - 16) << IMX9_GPIO_GICHR_GIWE_SHIFT) /* Pin n=16..31 */

/* Interrupt Status Flag Register */

#define IMX9_GPIO_ISFR(n)               (1 << (n))  /* Interrupt Status Flag, n=0-31 */

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX9_GPIO_H */
