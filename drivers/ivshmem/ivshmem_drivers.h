/****************************************************************************
 * drivers/ivshmem/ivshmem_drivers.h
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

#ifndef __DRIVERS_IVSHMEM_IVSHMEM_DRIVERS_H
#define __DRIVERS_IVSHMEM_IVSHMEM_DRIVERS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ivshmem_uio_register
 *
 * Description:
 *   Register uio ivshmem device driver
 *
 ****************************************************************************/

#ifdef CONFIG_UIO_IVSHMEM
int ivshmem_uio_register(void);
#endif

/****************************************************************************
 * Name: ivshmem_ramdisk_register
 *
 * Description:
 *   Register ramdisk ivshmem device driver
 *
 ****************************************************************************/

#ifdef CONFIG_RAMDISK_IVSHMEM
int ivshmem_ramdisk_register(void);
#endif

#endif /* __DRIVERS_IVSHMEM_IVSHMEM_DRIVERS_H */
