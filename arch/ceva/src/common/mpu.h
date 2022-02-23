/****************************************************************************
 * arch/ceva/src/common/mpu.h
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

#ifndef __ARCH_CEVA_SRC_COMMON_MPU_H
#define __ARCH_CEVA_SRC_COMMON_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ARCH_MPU

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 ****************************************************************************/

void mpu_control(bool enable);

/****************************************************************************
 * Name: mpu_user_code
 *
 * Description:
 *   Configure a region for user code
 *
 ****************************************************************************/

void mpu_user_code(const void *base, size_t size);

/****************************************************************************
 * Name: mpu_priv_code
 *
 * Description:
 *   Configure a region for privileged code
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_code(const void *base, size_t size);

/****************************************************************************
 * Name: mpu_user_data
 *
 * Description:
 *   Configure a region as user data
 *
 ****************************************************************************/

void mpu_user_data(void *base, size_t size);

/****************************************************************************
 * Name: mpu_priv_data
 *
 * Description:
 *   Configure a region as privileged data
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_data(void *base, size_t size);

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged peripheral address space
 *
 ****************************************************************************/

void mpu_peripheral(void *base, size_t size);

/****************************************************************************
 * Name: mpu_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_stronglyordered(void *base, size_t size);

#else

#define mpu_control(enable)
#define mpu_user_code(base, size)
#define mpu_priv_code(base, size)
#define mpu_user_data(base, size)
#define mpu_priv_data(base, size)
#define mpu_peripheral(base, size)
#define mpu_stronglyordered(base, size)

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif  /* __ASSEMBLY__ */
#endif  /* __ARCH_CEVA_SRC_COMMON_MPU_H */
