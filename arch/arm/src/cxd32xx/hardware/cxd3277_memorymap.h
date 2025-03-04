/****************************************************************************
 * arch/arm/src/cxd32xx/hardware/cxd3277_memorymap.h
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

#ifndef __ARCH_ARM_SRC_CXD32XX_HARDWARE_CXD3277_MEMORYMAP_H
#define __ARCH_ARM_SRC_CXD32XX_HARDWARE_CXD3277_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CXD32_UART0_BASE    (0x42100000+0x4000)
#define CXD32_UART1_BASE    (0x42100000+0x5000)
#define CXD32_UART2_BASE    (0x42100000+0x6000)

#define CXD32_TIMER0_BASE   (0x42200000+0x2000)
#define CXD32_TIMER1_BASE   (0x42200000+0x3000)
#define CXD32_TIMER2_BASE   (0x42200000+0x4000)
#define CXD32_TIMER3_BASE   (0x42200000+0x5000)
#define CXD32_TIMER4_BASE   (0x42200000+0x6000)

#define CXD32_INTC_BASE     0xe0045000

#endif /* __ARCH_ARM_SRC_CXD32XX_HARDWARE_CXD3277_MEMORYMAP_H */
