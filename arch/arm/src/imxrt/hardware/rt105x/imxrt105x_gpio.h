/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt105x/imxrt105x_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT105X_IMXRT105X_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT105X_IMXRT105X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define IMXRT_GPIO_DR_OFFSET     0x0000  /* GPIO data register */
#define IMXRT_GPIO_GDIR_OFFSET   0x0004  /* GPIO direction register */
#define IMXRT_GPIO_PSR_OFFSET    0x0008  /* GPIO pad status register */
#define IMXRT_GPIO_ICR1_OFFSET   0x000c  /* GPIO interrupt configuration register1 */
#define IMXRT_GPIO_ICR2_OFFSET   0x0010  /* GPIO interrupt configuration register2 */
#define IMXRT_GPIO_IMR_OFFSET    0x0014  /* GPIO interrupt mask register */
#define IMXRT_GPIO_ISR_OFFSET    0x0018  /* GPIO interrupt status register */
#define IMXRT_GPIO_EDGE_OFFSET   0x001c  /* GPIO edge select register */
#define IMXRT_GPIO_SET_OFFSET    0x0084  /* GPIO data register SET */
#define IMXRT_GPIO_CLEAR_OFFSET  0x0088  /* GPIO data register CLEAR */
#define IMXRT_GPIO_TOGGLE_OFFSET 0x008c  /* GPIO data register TOGGLE */

/* Register addresses *******************************************************/

#define IMXRT_GPIO1_DR           (IMXRT_GPIO1_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO1_GDIR         (IMXRT_GPIO1_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO1_PSR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO1_ICR1         (IMXRT_GPIO1_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO1_ICR2         (IMXRT_GPIO1_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO1_IMR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO1_ISR          (IMXRT_GPIO1_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO1_EDGE         (IMXRT_GPIO1_BASE + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO1_SET          (IMXRT_GPIO1_BASE + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO1_CLEAR        (IMXRT_GPIO1_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO1_TOGGLE       (IMXRT_GPIO1_BASE + IMXRT_GPIO_TOGGLE_OFFSET)

#define IMXRT_GPIO2_DR           (IMXRT_GPIO2_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO2_GDIR         (IMXRT_GPIO2_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO2_PSR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO2_ICR1         (IMXRT_GPIO2_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO2_ICR2         (IMXRT_GPIO2_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO2_IMR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO2_ISR          (IMXRT_GPIO2_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO2_EDGE         (IMXRT_GPIO2_BASE + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO2_SET          (IMXRT_GPIO2_BASE + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO2_CLEAR        (IMXRT_GPIO2_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO2_TOGGLE       (IMXRT_GPIO2_BASE + IMXRT_GPIO_TOGGLE_OFFSET)

#define IMXRT_GPIO3_DR           (IMXRT_GPIO3_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO3_GDIR         (IMXRT_GPIO3_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO3_PSR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO3_ICR1         (IMXRT_GPIO3_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO3_ICR2         (IMXRT_GPIO3_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO3_IMR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO3_ISR          (IMXRT_GPIO3_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO3_EDGE         (IMXRT_GPIO3_BASE + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO3_SET          (IMXRT_GPIO3_BASE + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO3_CLEAR        (IMXRT_GPIO3_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO3_TOGGLE       (IMXRT_GPIO3_BASE + IMXRT_GPIO_TOGGLE_OFFSET)

#define IMXRT_GPIO4_DR           (IMXRT_GPIO4_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO4_GDIR         (IMXRT_GPIO4_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO4_PSR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO4_ICR1         (IMXRT_GPIO4_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO4_ICR2         (IMXRT_GPIO4_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO4_IMR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO4_ISR          (IMXRT_GPIO4_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO4_EDGE         (IMXRT_GPIO4_BASE + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO4_SET          (IMXRT_GPIO4_BASE + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO4_CLEAR        (IMXRT_GPIO4_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO4_TOGGLE       (IMXRT_GPIO4_BASE + IMXRT_GPIO_TOGGLE_OFFSET)

#define IMXRT_GPIO5_DR           (IMXRT_GPIO5_BASE + IMXRT_GPIO_DR_OFFSET)
#define IMXRT_GPIO5_GDIR         (IMXRT_GPIO5_BASE + IMXRT_GPIO_GDIR_OFFSET)
#define IMXRT_GPIO5_PSR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_PSR_OFFSET)
#define IMXRT_GPIO5_ICR1         (IMXRT_GPIO5_BASE + IMXRT_GPIO_ICR1_OFFSET)
#define IMXRT_GPIO5_ICR2         (IMXRT_GPIO5_BASE + IMXRT_GPIO_ICR2_OFFSET)
#define IMXRT_GPIO5_IMR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_IMR_OFFSET)
#define IMXRT_GPIO5_ISR          (IMXRT_GPIO5_BASE + IMXRT_GPIO_ISR_OFFSET)
#define IMXRT_GPIO5_EDGE         (IMXRT_GPIO5_BASE + IMXRT_GPIO_EDGE_OFFSET)
#define IMXRT_GPIO5_SET          (IMXRT_GPIO5_BASE + IMXRT_GPIO_SET_OFFSET)
#define IMXRT_GPIO5_CLEAR        (IMXRT_GPIO5_BASE + IMXRT_GPIO_CLEAR_OFFSET)
#define IMXRT_GPIO5_TOGGLE       (IMXRT_GPIO5_BASE + IMXRT_GPIO_TOGGLE_OFFSET)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT105X_IMXRT105X_GPIO_H */
