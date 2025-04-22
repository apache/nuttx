/****************************************************************************
 * arch/arm/include/rp23xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RP23XX_CHIP_H
#define __ARCH_ARM_INCLUDE_RP23XX_CHIP_H

/* NVIC priority levels *****************************************************/

/* Each priority field holds an 8-bit value, but only the upper 4 bits [7:4]
 * are implemented by the processor (NVIC_PRIO_BITS = 4). The lower
 * 4 bits [3:0] are read as zero and ignore writes. A lower numeric value
 * indicates a higher interrupt priority.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits [7:4] set: lowest priority (15) */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint: priority level 8 */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* All bits [7:4] cleared: highest priority (0) */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* One step per priority level */

#define ARMV8M_PERIPHERAL_INTERRUPTS 52

#endif /* __ARCH_ARM_INCLUDE_RP23XX_CHIP_H */
