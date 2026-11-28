/****************************************************************************
 * arch/arm/src/n32h7/n32_flash.h
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

#ifndef __ARCH_ARM_SRC_N32H7_N32_FLASH_H
#define __ARCH_ARM_SRC_N32H7_N32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "hardware/n32h7_memorymap.h"

/* Flash geometry */
#define N32_FLASH_SECTOR_SIZE      (4 * 1024)       /* 4KB sector */
#define N32_FLASH_PAGE_SIZE        (4 * 1024)       /* 4KB page size */

#ifdef CONFIG_N32H7_FLASH_CONFIG_I
#define N32_MTD_FLASH_BASE         (0x000E0000)
#define N32_MTD_FLASH_SIZE         (0x00100000)
#endif /* CONFIG_N32H7_FLASH_CONFIG_I */
#ifdef CONFIG_N32H7_FLASH_CONFIG_K
#define N32_MTD_FLASH_BASE         (0x001E0000)
#define N32_MTD_FLASH_SIZE         (0x00200000)
#endif /* CONFIG_N32H7_FLASH_CONFIG_K */

#define N32_FLASH_NBLOCKS          (N32_MTD_FLASH_SIZE / N32_FLASH_SECTOR_SIZE)
#define N32_FLASH_NPAGES           (N32_MTD_FLASH_SIZE / N32_FLASH_PAGE_SIZE)

#endif /* __ARCH_ARM_SRC_N32H7_N32_FLASH_H */
