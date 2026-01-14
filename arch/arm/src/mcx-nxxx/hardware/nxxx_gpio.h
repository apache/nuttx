/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_gpio.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GPIO_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_N236)
#  include "hardware/n236/n236_gpio.h"
#else
#  error Unrecognized NXXx architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO0                           0      /* Port 0 index */
#define GPIO1                           1      /* Port 1 index */
#define GPIO2                           2      /* Port 2 index */
#define GPIO3                           3      /* Port 3 index */
#define GPIO4                           4      /* Port 4 index */
#define GPIO5                           5      /* Port 5 index */
#define GPIO6                           6      /* Port 6 index */
#define GPIO7                           7      /* Port 7 index */
#define GPIO8                           8      /* Port 8 index */
#define GPIO9                           9      /* Port 9 index */
#define GPIO10                          10     /* Port 10 index */
#define GPIO11                          11     /* Port 11 index */
#define GPIO12                          12     /* Port 12 index */
#define GPIO13                          13     /* Port 13 index */

#define NXXX_GPIO_NPINS                 32     /* Up to 32 pins per port */

/* Register bit definitions *************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)                     (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* ICRN Register */

#define NXXX_GPIO_ICRN_ISF              (1 << 24) /* Bit 24: Interrupt Status Flag */
#define NXXX_GPIO_ICRN_LK               (1 << 23) /* Bit 23: Lock Register */
#define NXXX_GPIO_ICRN_IRQS             (1 << 20) /* Bit 20: Configures the selected interrupt, or DMA request. */
#define NXXX_GPIO_ICRN_SHIFT            (16)      /* Bits 16-19:  Interrupt Configuration */
#define NXXX_GPIO_ICRN_MASK             (0xf << NXXX_GPIO_ICRN_SHIFT)
#  define NXXX_GPIO_ICRN_DISABLED       (0 << NXXX_GPIO_ICRN_SHIFT)  /* Interrupt Status Flag (ISF) is disabled */
#  define NXXX_GPIO_ICRN_DMARISING      (1 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on rising edge */
#  define NXXX_GPIO_ICRN_DMAFALLING     (2 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on falling edge */
#  define NXXX_GPIO_ICRN_DMABOTH        (3 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag and DMA request on either edge */
#  define NXXX_GPIO_ICRN_ISFRISING      (5 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag sets on rising edge */
#  define NXXX_GPIO_ICRN_ISFFALLING     (6 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag sets on falling edge */
#  define NXXX_GPIO_ICRN_ISFBOTH        (7 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag sets on either edge */
#  define NXXX_GPIO_ICRN_ZERO           (8 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag and Interrupt when logic 0 */
#  define NXXX_GPIO_ICRN_RISING         (9 << NXXX_GPIO_ICRN_SHIFT)  /* ISF flag and Interrupt on rising-edge */
#  define NXXX_GPIO_ICRN_FALLING        (10 << NXXX_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt on falling-edge */
#  define NXXX_GPIO_ICRN_BOTH           (11 << NXXX_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt on either edge */
#  define NXXX_GPIO_ICRN_ONE            (12 << NXXX_GPIO_ICRN_SHIFT) /* ISF flag and Interrupt when logic 1 */

/* Global Interrupt Control Low Register */

#define NXXX_GPIO_GICLR_GIWD_SHIFT      (0)       /* Bits 0-15: Global Interrupt Write Data */
#define NXXX_GPIO_GICLR_GIWD_MASK       (0xffff << NXXX_GPIO_GICLR_GIWD_SHIFT)
#  define NXXX_GPIO_GICLR_GIWD_PIN(n)   ((uint32_t)(n) << NXXX_GPIO_GICLR_GIWD_SHIFT) /* Pin n=0..15 */

#define NXXX_GPIO_GICLR_GIWE_SHIFT      (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define NXXX_GPIO_GICLR_GIWE_MASK       (0xffff << NXXX_GPIO_GICLR_GIWE_SHIFT)
#  define NXXX_GPIO_GICLR_GIWE_PIN(n)   ((uint32_t)(n) << NXXX_GPIO_GICLR_GIWE_SHIFT) /* Pin n=0..15 */

/* Global Interrupt Control High Register */

#define NXXX_GPIO_GICHR_GIWD_SHIFT      (0)       /* Bits 0-15: Global Interrupt Write Data */
#define NXXX_GPIO_GICHR_GIWD_MASK       (0xffff << NXXX_GPIO_GICHR_GIWD_SHIFT)
#  define NXXX_GPIO_GICHR_GIWD_PIN(n)   ((uint32_t)((n) - 16) << NXXX_GPIO_GICHR_GIWD_SHIFT) /* Pin n=16..31 */

#define NXXX_GPIO_GICHR_GIWE_SHIFT      (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define NXXX_GPIO_GICHR_GIWE_MASK       (0xffff << NXXX_GPIO_GICHR_GIWE_SHIFT)
#  define NXXX_GPIO_GICHR_GIWE_PIN(n)   ((uint32_t)((n) - 16) << NXXX_GPIO_GICHR_GIWE_SHIFT) /* Pin n=16..31 */

/* Interrupt Status Flag Register */

#define NXXX_GPIO_ISFR(n)               (1 << (n))  /* Interrupt Status Flag, n=0-31 */

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GPIO_H */
