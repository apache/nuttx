/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_gpio.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIO_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_GPIO_OUT_OFFSET        0x0000              /* Write GPIO port */
#define NRF54L_GPIO_OUTSET_OFFSET     0x0004              /* Set output pins */
#define NRF54L_GPIO_OUTCLR_OFFSET     0x0008              /* Clear output pins */
#define NRF54L_GPIO_IN_OFFSET         0x000c              /* Read GPIO port */
#define NRF54L_GPIO_DIR_OFFSET        0x0010              /* Pin direction */
#define NRF54L_GPIO_DIRSET_OFFSET     0x0014              /* Set pins as outputs */
#define NRF54L_GPIO_DIRCLR_OFFSET     0x0018              /* Set pins as inputs */
#define NRF54L_GPIO_LATCH_OFFSET      0x0020              /* Latched pin detection */
#define NRF54L_GPIO_DETECTMODE_OFFSET 0x0024              /* Select DETECT signal behavior */
#define NRF54L_GPIO_PIN_CNF_OFFSET(n) (0x0080 + 4 * (n))  /* Pin n configuration */

/* Register addresses *******************************************************/

#define NRF54L_GPIO0_OUT        (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_OUT_OFFSET)
#define NRF54L_GPIO0_OUTSET     (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_OUTSET_OFFSET)
#define NRF54L_GPIO0_OUTCLR     (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_OUTCLR_OFFSET)
#define NRF54L_GPIO0_IN         (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_IN_OFFSET)
#define NRF54L_GPIO0_DIR        (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_DIR_OFFSET)
#define NRF54L_GPIO0_DIRSET     (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_DIRSET_OFFSET)
#define NRF54L_GPIO0_DIRCLR     (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_DIRCLR_OFFSET)
#define NRF54L_GPIO0_LATCH      (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_LATCH_OFFSET)
#define NRF54L_GPIO0_DETECTMODE (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_DETECTMODE_OFFSET)
#define NRF54L_GPIO0_CNF(n)     (NRF54L_GPIO_P0_BASE + NRF54L_GPIO_PIN_CNF_OFFSET(n))

#define NRF54L_GPIO1_OUT        (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_OUT_OFFSET)
#define NRF54L_GPIO1_OUTSET     (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_OUTSET_OFFSET)
#define NRF54L_GPIO1_OUTCLR     (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_OUTCLR_OFFSET)
#define NRF54L_GPIO1_IN         (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_IN_OFFSET)
#define NRF54L_GPIO1_DIR        (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_DIR_OFFSET)
#define NRF54L_GPIO1_DIRSET     (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_DIRSET_OFFSET)
#define NRF54L_GPIO1_DIRCLR     (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_DIRCLR_OFFSET)
#define NRF54L_GPIO1_LATCH      (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_LATCH_OFFSET)
#define NRF54L_GPIO1_DETECTMODE (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_DETECTMODE_OFFSET)
#define NRF54L_GPIO1_CNF(n)     (NRF54L_GPIO_P1_BASE + NRF54L_GPIO_PIN_CNF_OFFSET(n))

#define NRF54L_GPIO2_OUT        (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_OUT_OFFSET)
#define NRF54L_GPIO2_OUTSET     (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_OUTSET_OFFSET)
#define NRF54L_GPIO2_OUTCLR     (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_OUTCLR_OFFSET)
#define NRF54L_GPIO2_IN         (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_IN_OFFSET)
#define NRF54L_GPIO2_DIR        (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_DIR_OFFSET)
#define NRF54L_GPIO2_DIRSET     (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_DIRSET_OFFSET)
#define NRF54L_GPIO2_DIRCLR     (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_DIRCLR_OFFSET)
#define NRF54L_GPIO2_LATCH      (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_LATCH_OFFSET)
#define NRF54L_GPIO2_DETECTMODE (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_DETECTMODE_OFFSET)
#define NRF54L_GPIO2_CNF(n)     (NRF54L_GPIO_P2_BASE + NRF54L_GPIO_PIN_CNF_OFFSET(n))

#define NRF54L_GPIO3_OUT        (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_OUT_OFFSET)
#define NRF54L_GPIO3_OUTSET     (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_OUTSET_OFFSET)
#define NRF54L_GPIO3_OUTCLR     (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_OUTCLR_OFFSET)
#define NRF54L_GPIO3_IN         (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_IN_OFFSET)
#define NRF54L_GPIO3_DIR        (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_DIR_OFFSET)
#define NRF54L_GPIO3_DIRSET     (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_DIRSET_OFFSET)
#define NRF54L_GPIO3_DIRCLR     (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_DIRCLR_OFFSET)
#define NRF54L_GPIO3_LATCH      (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_LATCH_OFFSET)
#define NRF54L_GPIO3_DETECTMODE (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_DETECTMODE_OFFSET)
#define NRF54L_GPIO3_CNF(n)     (NRF54L_GPIO_P3_BASE + NRF54L_GPIO_PIN_CNF_OFFSET(n))

/* Register bit definitions *************************************************/

/* DETECTMODE register */

#define GPIO_DETECTMODE_DEFAULT (0)
#define GPIO_DETECTMODE_LDETECT (1)

/* PIN_CNF register */

#define GPIO_CNF_DIR                 (1 << 0)
#define GPIO_CNF_INPUT               (1 << 1)
#define GPIO_CNF_PULL_SHIFT          (2)
#define GPIO_CNF_PULL_MASK           (3 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_DISABLED     (0 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_DOWN         (1 << GPIO_CNF_PULL_SHIFT)
#  define GPIO_CNF_PULL_UP           (3 << GPIO_CNF_PULL_SHIFT)
#define GPIO_CNF_DRIVE0_SHIFT        (8)
#define GPIO_CNF_DRIVE0_MASK         (3 << GPIO_CNF_DRIVE0_SHIFT)
#  define GPIO_CNF_DRIVE0_STANDARD   (0 << GPIO_CNF_DRIVE0_SHIFT)
#  define GPIO_CNF_DRIVE0_HIGH       (1 << GPIO_CNF_DRIVE0_SHIFT)
#  define GPIO_CNF_DRIVE0_DISCONNECT (2 << GPIO_CNF_DRIVE0_SHIFT)
#  define GPIO_CNF_DRIVE0_EXTRA      (3 << GPIO_CNF_DRIVE0_SHIFT)
#define GPIO_CNF_DRIVE1_SHIFT        (10)
#define GPIO_CNF_DRIVE1_MASK         (3 << GPIO_CNF_DRIVE1_SHIFT)
#  define GPIO_CNF_DRIVE1_STANDARD   (0 << GPIO_CNF_DRIVE1_SHIFT)
#  define GPIO_CNF_DRIVE1_HIGH       (1 << GPIO_CNF_DRIVE1_SHIFT)
#  define GPIO_CNF_DRIVE1_DISCONNECT (2 << GPIO_CNF_DRIVE1_SHIFT)
#  define GPIO_CNF_DRIVE1_EXTRA      (3 << GPIO_CNF_DRIVE1_SHIFT)
#define GPIO_CNF_SENSE_SHIFT         (16)
#define GPIO_CNF_SENSE_MASK          (3 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_DISABLED    (0 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_HIGH        (2 << GPIO_CNF_SENSE_SHIFT)
#  define GPIO_CNF_SENSE_LOW         (3 << GPIO_CNF_SENSE_SHIFT)
#define GPIO_CNF_CTRLSEL_SHIFT       (28)
#define GPIO_CNF_CTRLSEL_MASK        (7 << GPIO_CNF_CTRLSEL_SHIFT)
#  define GPIO_CNF_CTRLSEL_GPIO      (0 << GPIO_CNF_CTRLSEL_SHIFT)
#  define GPIO_CNF_CTRLSEL_VPR       (1 << GPIO_CNF_CTRLSEL_SHIFT)
#  define GPIO_CNF_CTRLSEL_GRTC      (4 << GPIO_CNF_CTRLSEL_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_GPIO_H */
