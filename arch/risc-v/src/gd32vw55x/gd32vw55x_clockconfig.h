/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CLOCKCONFIG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock tree after gd32vw55x_clockconfig():
 * HXTAL 40 MHz -> PLLDIG VCO 480 MHz -> /3 -> SYSCLK 160 MHz
 * AHB = 160 MHz, APB2 = 160 MHz, APB1 = 80 MHz
 * SysTimer (mtime) = SYSCLK (CLKSRC = 1) = 160 MHz
 */

#define GD32VW55X_HXTAL_FREQ     40000000
#define GD32VW55X_SYSCLK_FREQ    160000000
#define GD32VW55X_AHB_FREQ       GD32VW55X_SYSCLK_FREQ
#define GD32VW55X_PCLK2_FREQ     GD32VW55X_SYSCLK_FREQ
#define GD32VW55X_PCLK1_FREQ     (GD32VW55X_SYSCLK_FREQ / 2)
#define GD32VW55X_MTIME_FREQ     GD32VW55X_SYSCLK_FREQ

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

void gd32vw55x_clockconfig(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_GD32VW55X_GD32VW55X_CLOCKCONFIG_H */
