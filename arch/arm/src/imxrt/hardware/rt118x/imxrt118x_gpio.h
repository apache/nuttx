/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_gpio.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_GPIO_DR_OFFSET       0x00
#define IMXRT_GPIO_GDIR_OFFSET     0x04
#define IMXRT_GPIO_PSR_OFFSET      0x08
#define IMXRT_GPIO_ICR1_OFFSET     0x0c
#define IMXRT_GPIO_ICR2_OFFSET     0x10
#define IMXRT_GPIO_IMR_OFFSET      0x14
#define IMXRT_GPIO_ISR_OFFSET      0x18
#define IMXRT_GPIO_EDGE_OFFSET     0x1c

#define IMXRT_GPIO_SET_OFFSET      0x84
#define IMXRT_GPIO_CLEAR_OFFSET    0x88
#define IMXRT_GPIO_TOGGLE_OFFSET   0x8c

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPIO_H */
