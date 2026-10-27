/****************************************************************************
 * include/nuttx/ivshmem_flat.h
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

#ifndef __INCLUDE_NUTTX_IVSHMEM_FLAT_H
#define __INCLUDE_NUTTX_IVSHMEM_FLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IVSHMEM_FLAT

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ivshmem_flat_config_s
{
  uintptr_t reg;        /* Register block base */
  uintptr_t shmem;      /* Shared memory base */
  size_t    shmem_size; /* Shared memory size */
  int       irq;        /* Doorbell irq, negative if none */
  int       id;         /* ivshmem bus id, negative to let the bus assign */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ivshmem_flat_register
 *
 * Description:
 *   Register a QEMU ivshmem-flat device on the ivshmem bus.
 *
 * Input Parameters:
 *   config - Device resources; copied, may be on the stack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ivshmem_flat_register(FAR const struct ivshmem_flat_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_IVSHMEM_FLAT */
#endif /* __INCLUDE_NUTTX_IVSHMEM_FLAT_H */
