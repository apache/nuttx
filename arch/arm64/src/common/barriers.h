/****************************************************************************
 * arch/arm64/src/common/barriers.h
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

#ifndef ___ARCH_ARM64_SRC_COMMON_BARRIERS_H
#define ___ARCH_ARM64_SRC_COMMON_BARRIERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* See Arm® Architecture Reference Manual
 * ARM DDI 0487E.a C6.2.81
 */

#define __DSB(arg) __asm__ volatile ("dsb " #arg : : : "memory");

/* See Arm® Architecture Reference Manual
 * ARM DDI 0487E.a C6.2.79
 */

#define __DMB(arg) __asm__ volatile ("dmb " #arg : : : "memory");

/* See Arm® Architecture Reference Manual
 * ARM DDI 0487E.a C6.2.96
 */

#define __ISB()    __asm__ volatile ("isb" : : : "memory");

/* THe most common barriers */

#define ARM64_DSB()  __DSB(sy)
#define ARM64_DMB()  __DMB(sy)
#define ARM64_ISB()  __ISB()

#endif /* __ASSEMBLY__ */

#endif /* ___ARCH_ARM64_SRC_COMMON_BARRIERS_H */
