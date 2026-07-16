/****************************************************************************
 * arch/arm64/include/rk3588/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_RK3588_IRQ_H
#define __ARCH_ARM64_INCLUDE_RK3588_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size the IRQ table to cover the highest SPI currently described by the
 * upstream RK3588 device tree.  SPI 400 has GIC interrupt ID 432.
 */

#define NR_IRQS 512

/* GIC interrupt IDs are the device-tree SPI number plus 32. */

#define RK3588_IRQ_UART2 365  /* GIC_SPI 333 */

#endif /* __ARCH_ARM64_INCLUDE_RK3588_IRQ_H */
