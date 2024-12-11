/****************************************************************************
 * arch/arm/include/armv8-m/barriers.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV8_M_BARRIERS_H
#define __ARCH_ARM_INCLUDE_ARMV8_M_BARRIERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARMv8-M memory barriers */

#define arm_isb()  __asm__ __volatile__ ("isb " : : : "memory")
#define arm_dmb()  __asm__ __volatile__ ("dmb " : : : "memory")
#define arm_dsb(n) __asm__ __volatile__ ("dsb " #n : : : "memory")

#define UP_ISB()  arm_isb()
#define UP_DMB()  arm_dmb()
#define UP_DSB()  arm_dsb(15)

#endif /* __ARCH_ARM_INCLUDE_ARMV8_M_BARRIERS_H */
