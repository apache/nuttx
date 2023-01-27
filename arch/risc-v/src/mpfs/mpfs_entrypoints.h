/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_entrypoints.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_ENTRYPOINTS_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_ENTRYPOINTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mpfs_get_entrypt
 *
 * Description:
 *   Obtain Hart entrypoint
 *
 * Input Parameters:
 *   hartid - Hart ID to read
 *
 * Returned value:
 *   Entrypoint on success; 0 on failure
 *
 ****************************************************************************/

uintptr_t mpfs_get_entrypt(uint64_t hartid);

/****************************************************************************
 * Name: mpfs_set_entrypt
 *
 * Description:
 *   Modify Hart entrypoint
 *
 * Input Parameters:
 *   hartid - Hart ID to modify
 *   entry - Entrypoint to set
 *
 * Returned value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int mpfs_set_entrypt(uint64_t hartid, uintptr_t entry);

/****************************************************************************
 * Name: mpfs_set_use_sbi
 *
 * Description:
 *   Set booting via SBI.
 *
 * Input Parameters:
 *   use_sbi - set to true if sbi is needed, false otherwise
 *
 * Returned value:
 *   OK on success, ERROR on failure
 *
 ****************************************************************************/

int mpfs_set_use_sbi(uint64_t hartid, bool use_sbi);

#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_ENTRYPOINTS_H */
