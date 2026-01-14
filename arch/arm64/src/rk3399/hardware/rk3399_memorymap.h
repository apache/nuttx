/****************************************************************************
 * arch/arm64/src/rk3399/hardware/rk3399_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_RK3399_HARDWARE_RK3399_MEMORYMAP_H
#define __ARCH_ARM64_SRC_RK3399_HARDWARE_RK3399_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Base Addresses */
#define RK3399_GPIO0_ADDR        0xff720000
#define RK3399_GPIO1_ADDR        0xff730000
#define RK3399_GPIO2_ADDR        0xff780000
#define RK3399_GPIO3_ADDR        0xff788000
#define RK3399_GPIO4_ADDR        0xff790000

#define RK3399_PIO_ADDR        RK3399_GPIO0_ADDR
#define RK3399_PWM_ADDR        0xff430000
#define RK3399_UART0_ADDR      0xff180000
#define RK3399_UART1_ADDR      0xff190000
#define RK3399_UART2_ADDR      0xff1a0000
#define RK3399_UART3_ADDR      0xff1b0000
#define RK3399_UART4_ADDR      0xff370000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_RK3399_HARDWARE_RK3399_MEMORYMAP_H */
