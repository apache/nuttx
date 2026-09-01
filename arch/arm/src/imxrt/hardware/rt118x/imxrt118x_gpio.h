/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RGPIO register offsets (IMXRT1180RM chapter 16) **************************/

#define IMXRT_GPIO_VERID_OFFSET         (0x0000)  /* Version ID */
#define IMXRT_GPIO_PARAM_OFFSET         (0x0004)  /* Parameter */
#define IMXRT_GPIO_LOCK_OFFSET          (0x000c)  /* Lock */
#define IMXRT_GPIO_PCNS_OFFSET          (0x0010)  /* Pin Control Nonsecure */
#define IMXRT_GPIO_ICNS_OFFSET          (0x0014)  /* Interrupt Control Nonsecure */
#define IMXRT_GPIO_PCNP_OFFSET          (0x0018)  /* Pin Control Nonprivilege */
#define IMXRT_GPIO_ICNP_OFFSET          (0x001c)  /* Interrupt Control Nonprivilege */
#define IMXRT_GPIO_PDOR_OFFSET          (0x0040)  /* Port Data Output */
#define IMXRT_GPIO_PSOR_OFFSET          (0x0044)  /* Port Set Output */
#define IMXRT_GPIO_PCOR_OFFSET          (0x0048)  /* Port Clear Output */
#define IMXRT_GPIO_PTOR_OFFSET          (0x004c)  /* Port Toggle Output */
#define IMXRT_GPIO_PDIR_OFFSET          (0x0050)  /* Port Data Input */
#define IMXRT_GPIO_PDDR_OFFSET          (0x0054)  /* Port Data Direction */
#define IMXRT_GPIO_PIDR_OFFSET          (0x0058)  /* Port Input Disable */
#define IMXRT_GPIO_P0DR_OFFSET          (0x0060)  /* Pin Data (0-31 at n * 4h) */
#define IMXRT_GPIO_ICR0_OFFSET          (0x0080)  /* Interrupt Control (0-31 at n * 4h) */
#define IMXRT_GPIO_GICLR_OFFSET         (0x0100)  /* Global Interrupt Control Low */
#define IMXRT_GPIO_GICHR_OFFSET         (0x0104)  /* Global Interrupt Control High */
#define IMXRT_GPIO_ISFR0_OFFSET         (0x0120)  /* Interrupt Status Flag 0 (0..15) */
#define IMXRT_GPIO_ISFR1_OFFSET         (0x0124)  /* Interrupt Status Flag 1 (16..31) */

/* GPIO port enumeration ****************************************************/

#define GPIO1                           0
#define GPIO2                           1
#define GPIO3                           2
#define GPIO4                           3
#define GPIO5                           4
#define GPIO6                           5

#define IMXRT_GPIO_NPORTS               6
#define IMXRT_GPIO_NPINS                32

/* Register bit definitions *************************************************/

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)                     (1u << (n))  /* Bit n: Pin n, n=0-31 */

/* ICRN register (per-pin interrupt configuration) */

#define IMXRT_GPIO_ICRN_ISF             (1u << 24)   /* Bit 24: Interrupt Status Flag */
#define IMXRT_GPIO_ICRN_LK              (1u << 23)   /* Bit 23: Lock */
#define IMXRT_GPIO_ICRN_IRQS            (1u << 20)   /* Bit 20: Selects the interrupt/DMA channel */
#define IMXRT_GPIO_ICRN_SHIFT           (16)         /* Bits 16-19: Interrupt Configuration */
#define IMXRT_GPIO_ICRN_MASK            (0xfu << IMXRT_GPIO_ICRN_SHIFT)
#  define IMXRT_GPIO_ICRN_DISABLED      (0u  << IMXRT_GPIO_ICRN_SHIFT)  /* ISF disabled */
#  define IMXRT_GPIO_ICRN_DMARISING     (1u  << IMXRT_GPIO_ICRN_SHIFT)  /* DMA rising */
#  define IMXRT_GPIO_ICRN_DMAFALLING    (2u  << IMXRT_GPIO_ICRN_SHIFT)  /* DMA falling */
#  define IMXRT_GPIO_ICRN_DMABOTH       (3u  << IMXRT_GPIO_ICRN_SHIFT)  /* DMA both */
#  define IMXRT_GPIO_ICRN_ISFRISING     (5u  << IMXRT_GPIO_ICRN_SHIFT)  /* ISF-only rising */
#  define IMXRT_GPIO_ICRN_ISFFALLING    (6u  << IMXRT_GPIO_ICRN_SHIFT)  /* ISF-only falling */
#  define IMXRT_GPIO_ICRN_ISFBOTH       (7u  << IMXRT_GPIO_ICRN_SHIFT)  /* ISF-only both */
#  define IMXRT_GPIO_ICRN_ZERO          (8u  << IMXRT_GPIO_ICRN_SHIFT)  /* IRQ when 0 */
#  define IMXRT_GPIO_ICRN_RISING        (9u  << IMXRT_GPIO_ICRN_SHIFT)  /* IRQ rising */
#  define IMXRT_GPIO_ICRN_FALLING       (10u << IMXRT_GPIO_ICRN_SHIFT)  /* IRQ falling */
#  define IMXRT_GPIO_ICRN_BOTH          (11u << IMXRT_GPIO_ICRN_SHIFT)  /* IRQ both */
#  define IMXRT_GPIO_ICRN_ONE           (12u << IMXRT_GPIO_ICRN_SHIFT)  /* IRQ when 1 */

/* Global Interrupt Control Low (pins 0..15) */

#define IMXRT_GPIO_GICLR_GIWD_SHIFT     (0)
#define IMXRT_GPIO_GICLR_GIWD_MASK      (0xffffu << IMXRT_GPIO_GICLR_GIWD_SHIFT)
#  define IMXRT_GPIO_GICLR_GIWD_PIN(n)  ((uint32_t)(n) << IMXRT_GPIO_GICLR_GIWD_SHIFT)
#define IMXRT_GPIO_GICLR_GIWE_SHIFT     (16)
#define IMXRT_GPIO_GICLR_GIWE_MASK      (0xffffu << IMXRT_GPIO_GICLR_GIWE_SHIFT)
#  define IMXRT_GPIO_GICLR_GIWE_PIN(n)  ((uint32_t)(n) << IMXRT_GPIO_GICLR_GIWE_SHIFT)

/* Global Interrupt Control High (pins 16..31) */

#define IMXRT_GPIO_GICHR_GIWD_SHIFT     (0)
#define IMXRT_GPIO_GICHR_GIWD_MASK      (0xffffu << IMXRT_GPIO_GICHR_GIWD_SHIFT)
#  define IMXRT_GPIO_GICHR_GIWD_PIN(n)  ((uint32_t)((n) - 16) << IMXRT_GPIO_GICHR_GIWD_SHIFT)
#define IMXRT_GPIO_GICHR_GIWE_SHIFT     (16)
#define IMXRT_GPIO_GICHR_GIWE_MASK      (0xffffu << IMXRT_GPIO_GICHR_GIWE_SHIFT)
#  define IMXRT_GPIO_GICHR_GIWE_PIN(n)  ((uint32_t)((n) - 16) << IMXRT_GPIO_GICHR_GIWE_SHIFT)

/* Interrupt Status Flag Register */

#define IMXRT_GPIO_ISFR(n)              (1u << (n))  /* Interrupt Status Flag, n=0-31 */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H */
