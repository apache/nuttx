/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_sysctl.h
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_SYSCTL_PLL0  (K210_SYSCTL_BASE + 0x08)

#define PLL_CLK_R(n)  (n & 0x00000f)
#define PLL_CLK_F(n)  ((n & 0x0003f0) >> 4)
#define PLL_CLK_OD(n) ((n & 0x003c00) >> 10)

#define K210_SYSCTL_CLKSEL0  (K210_SYSCTL_BASE + 0x20)

#define CLKSEL0_ACLK_SEL(n) (n & 0x00000001)

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_SYSCTL_H */
