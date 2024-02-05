/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_mpu.h
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

#ifndef __ARCH_RISC_V_SRC_MPFS_MPFS_MPU_H
#define __ARCH_RISC_V_SRC_MPFS_MPFS_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_mpu_set
 *
 * Description:
 *   Set value to MPFS MPUCFG register.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to write.
 *   perm - The region permissions.
 *   base - The base address of the region.
 *   size - The length of the region.
 *
 * Note:
 *   Only NAPOT encoded regions are supported, thus the base address and
 *   size must align with each other.
 *
 * Returned Value:
 *   0 on success; negated error on failure.
 *
 ****************************************************************************/

int mpfs_mpu_set(uintptr_t reg, uintptr_t perm, uintptr_t base,
                 uintptr_t size);

/****************************************************************************
 * Name: mpfs_mpu_access_ok
 *
 * Description:
 *   Check if MPFS MPUCFG access is OK for register.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to check.
 *   perm - The region permissions.
 *   base - The base address of the region.
 *   size - The length of the region.
 *
 * Returned Value:
 *   true if access OK; false if not.
 *
 ****************************************************************************/

bool mpfs_mpu_access_ok(uintptr_t reg, uintptr_t perm, uintptr_t base,
                        uintptr_t size);

/****************************************************************************
 * Name: mpfs_mpu_lock
 *
 * Description:
 *   Lock an MPUCFG register from further modifications.
 *
 * Input Parameters:
 *   reg  - The MPUCFG register to lock.
 *
 * Returned Value:
 *   0 on success; negated error on failure.
 *
 ****************************************************************************/

int mpfs_mpu_lock(uintptr_t reg);

#endif /* __ARCH_RISC_V_SRC_MPFS_MPFS_MPU_H */
