/****************************************************************************
 * arch/arm64/src/rk3399/rk3399_serial.h
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

#ifndef __ARCH_ARM64_SRC_RK3399_RK3399_SERIAL_H
#define __ARCH_ARM64_SRC_RK3399_RK3399_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_internal.h"
#include "arm64_gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_RK3399

/* IRQ for RK3399 UART Rockchip_RK3399TRM_V1.4_Part1-20170408 page 17 */

#define RK3399_UART2_ADDR      0xff1a0000
#define RK3399_UART0_IRQ       131         /* RK3399 UART0 IRQ */
#define RK3399_UART1_IRQ       130         /* RK3399 UART1 IRQ */
#define RK3399_UART2_IRQ       132         /* RK3399 UART2 IRQ */
#define RK3399_UART3_IRQ       133         /* RK3399 UART3 IRQ */
#define RK3399_UART4_IRQ       134         /* RK3399 UART4 IRQ */

#endif /* CONFIG_ARCH_CHIP_RK3399 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3399_RK3399_SERIAL_H */
