/****************************************************************************
 * arch/arm/include/imx9/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_INCLUDE_IMX9_CHIP_H
#define __ARCH_ARM_INCLUDE_IMX9_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NVIC priority levels *****************************************************/

/* Each priority field holds an 8-bit priority value, 0-15. The lower the
 * value, the greater the priority of the corresponding interrupt.  The i.MX
 * RT processor implements only bits[7:4] of each field, bits[3:0] read as
 * zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is min pri */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x40 /* Two bits of interrupt pri used */

#define IMX9_GPIO_NPORTS          4

#endif /* __ARCH_ARM_INCLUDE_IMX9_CHIP_H */
