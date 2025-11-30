/****************************************************************************
 * arch/arm/include/arm_m/barriers.h
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

#ifndef __ARCH_ARM_INCLUDE_ARM_M_BARRIERS_H
#define __ARCH_ARM_INCLUDE_ARM_M_BARRIERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ARM-M memory barriers */

#define arm_isb()  __asm__ __volatile__ ("isb" : : : "memory")
#define arm_dmb()  __asm__ __volatile__ ("dmb" : : : "memory")

#if defined(CONFIG_ARCH_ARMV7M) || defined(CONFIG_ARCH_ARMV8M)
#  define arm_dmb_ish()   __asm__ __volatile__ ("dmb ish"   : : : "memory")
#  define arm_dmb_ishst() __asm__ __volatile__ ("dmb ishst" : : : "memory")
#  define arm_dsb_sy()    __asm__ __volatile__ ("dsb sy"    : : : "memory")
#  define arm_rmb()       arm_dmb_ish()
#  define arm_wmb()       arm_dmb_ishst()
#  define arm_dsb()       arm_dsb_sy()
#else
#  define arm_rmb()       arm_dmb()
#  define arm_wmb()       arm_dmb()
#  define arm_dsb()       __asm__ __volatile__ ("dsb" : : : "memory")
#endif

#define UP_ISB()  arm_isb()
#define UP_DMB()  arm_dmb()
#define UP_RMB()  arm_rmb()
#define UP_WMB()  arm_wmb()
#define UP_DSB()  arm_dsb()

#endif /* __ARCH_ARM_INCLUDE_ARM_M_BARRIERS_H */
