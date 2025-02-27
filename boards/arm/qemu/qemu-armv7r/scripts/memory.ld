/****************************************************************************
 * boards/arm/qemu/qemu-armv7r/scripts/memory.ld
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

#include <nuttx/config.h>

#ifdef CONFIG_BUILD_PROTECTED
/* Split FLASH and SRAM between kernel and user spaces */

#  define KFLASH_SIZE (CONFIG_NUTTX_USERSPACE)
#  define KSRAM_SIZE  (CONFIG_RAM_SIZE / 2)
#else
/* Give FLASH and RAM to kernel */

#  define KFLASH_SIZE (CONFIG_FLASH_SIZE)
#  define KSRAM_SIZE  (CONFIG_RAM_SIZE)
#endif

#define KFLASH_START  (CONFIG_FLASH_START)
#define KSRAM_START   (CONFIG_RAM_START)

#define UFLASH_START  (KFLASH_START + KFLASH_SIZE)
#define USRAM_START   (KSRAM_START + KSRAM_SIZE)

#define UFLASH_SIZE   (CONFIG_FLASH_SIZE - KFLASH_SIZE)
#define USRAM_SIZE    (CONFIG_RAM_SIZE - KSRAM_SIZE)

#if defined(CONFIG_BUILD_PROTECTED) && (KFLASH_SIZE != UFLASH_SIZE)
/* For simpler MPU settings, use 1:1 split for now */

#  error "CONFIG_NUTTX_USERSPACE != CONFIG_FLASH_SIZE / 2"
#endif

MEMORY
{
  kflash (rx) : ORIGIN = KFLASH_START, LENGTH = KFLASH_SIZE
  uflash (rx) : ORIGIN = UFLASH_START, LENGTH = UFLASH_SIZE
  ksram (rwx) : ORIGIN = KSRAM_START,  LENGTH = KSRAM_SIZE
  usram (rwx) : ORIGIN = USRAM_START,  LENGTH = USRAM_SIZE
}
