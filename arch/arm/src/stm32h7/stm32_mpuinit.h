/****************************************************************************
 * arch/arm/src/stm32h7/stm32_mpuinit.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_MPUINIT_H
#define __ARCH_ARM_SRC_STM32H7_STM32_MPUINIT_H

/****************************************************************************
 * Name: stm32_mpuinitialize
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Name: stm32_mpuinitialize
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpuinitialize
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpuinitialize
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_mpuinitialize
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only unrestricted
 *   STM32H7 resources.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void stm32_mpuinitialize(void);
#else
#  define stm32_mpuinitialize()
#endif

/****************************************************************************
 * Name: stm32_mpu_uheap
 *
 * Description:
 *  Map the user heap region.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void stm32_mpu_uheap(uintptr_t start, size_t size);
#else
#  define stm32_mpu_uheap(start,size)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_MPUINIT_H */
