/****************************************************************************
 * include/nuttx/rptun/rptun_ivshmem.h
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

#ifndef __INCLUDE_NUTTX_RPTUN_RPTUN_IVSHMEM_H
#define __INCLUDE_NUTTX_RPTUN_RPTUN_IVSHMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_RPTUN_IVSHMEM

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pci_register_rptun_ivshmem_driver
 *
 * Description:
 *   Initializes the rptun ivshmem driver.
 *
 * Returned Value:
 *   OK on success, negated errno on failure
 *
 ****************************************************************************/

int pci_register_rptun_ivshmem_driver(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RPTUN_IVSHMEM */
#endif /* __INCLUDE_NUTTX_RPTUN_RPTUN_IVSHMEM_H */
