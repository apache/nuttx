/****************************************************************************
 * arch/arm/include/rp2040/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RP2040_CHIP_H
#define __ARCH_ARM_INCLUDE_RP2040_CHIP_H

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-3. The lower the value,
 * the greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:6] of each field, bits[5:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Five bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_RP2040_CHIP_H */
