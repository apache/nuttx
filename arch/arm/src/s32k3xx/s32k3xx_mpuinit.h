/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_mpuinit.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_MPUINIT_H
#define __ARCH_ARM_SRC_S32K3XX_MPUINIT_H

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
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
 * Name: s32k3xx_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only unrestricted
 *   S32K3XX resources.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_MPU
void s32k3xx_mpuinitialize(void);
#endif

/****************************************************************************
 * Name: s32k3xx_mpu_uheap
 *
 * Description:
 *  Map the user heap region.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void s32k3xx_mpu_uheap(uintptr_t start, size_t size);
#else
#  define s32k3xx_mpu_uheap(start,size)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_MPUINIT_H */
