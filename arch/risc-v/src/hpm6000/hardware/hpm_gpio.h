/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm_gpio.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_GPIO_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/hpm_memorymap.h"

#if defined(CONFIG_ARCH_CHIP_HPM6360IPA)
#include "hpm6300/hpm6300_ioc.h"
#include "hpm6300/hpm6300_pinmux.h"
#else
#error The selected HPM variant is not impelemented
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOD 3
#define GPIOE 4
#define GPIOF 5
#define GPIOX 6
#define GPIOY 7
#define GPIOZ 8
#define HPM_GPIO_NPINS 32

/* Most registers are laid out simply with one bit per pin */

#define GPIO_PIN(n)              (1 << (n)) /* Bit n: Pin n, n=0-31 */

/* Register offsets *********************************************************/

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_GPIO_H */
