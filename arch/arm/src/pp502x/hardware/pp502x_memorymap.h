/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_MEMORYMAP_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Address space.  SDRAM is physically at 0x10000000; window 0 of the remap
 * unit makes it appear at logical 0 as well, which is where the image is
 * linked.
 */

#define PP502X_SDRAM_LOGICAL    0x00000000  /* SDRAM through window 0 */
#define PP502X_SDRAM_PHYS       0x10000000  /* SDRAM physical address */
#define PP502X_SDRAM_SIZE_32M   0x02000000  /* Smaller SDRAM part */
#define PP502X_SDRAM_SIZE_64M   0x04000000  /* Larger SDRAM part */
#define PP502X_IRAM_BASE        0x40000000  /* Internal SRAM */
#define PP502X_IRAM_SIZE        0x00020000  /* 128 KiB */

/* On-chip peripheral blocks */

#define PP502X_PROCID_BASE      0x60000000  /* Processor id */
#define PP502X_INTC_BASE        0x60004000  /* Interrupt controller */
#define PP502X_TIMER_BASE       0x60005000  /* Timers and USEC_TIMER */
#define PP502X_DEVCTL_BASE      0x60006000  /* Device reset/enable, clocks */
#define PP502X_PROCCTL_BASE     0x60007000  /* Core sleep/wake control */
#define PP502X_CACHECTL_BASE    0x6000c000  /* Cache control */
#define PP502X_GPIO_BASE        0x6000d000  /* GPIO ports A-L */
#define PP502X_SYSCFG_BASE      0x70000000  /* System configuration, GPO32 */
#define PP502X_UART0_BASE       0x70006000  /* UART 0 (dock connector) */
#define PP502X_UART1_BASE       0x70006040  /* UART 1 */
#define PP502X_REMAP_BASE       0xf000f000  /* Remap windows, cache mask */

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_MEMORYMAP_H */
