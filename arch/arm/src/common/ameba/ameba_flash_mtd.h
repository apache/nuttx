/****************************************************************************
 * arch/arm/src/common/ameba/ameba_flash_mtd.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_FLASH_MTD_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_FLASH_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

struct mtd_dev_s;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* On-chip SPI NOR flash geometry (Puya/Winbond class).  NOR has three
 * distinct granularities; only the smallest two are programmable units:
 *   - erase sector  = 4 KiB : smallest erasable unit (FLASH_EraseXIP).
 *                             Exposed as the MTD erase block
 *                             (geo->erasesize).
 *   - program page  = 256 B : the NOR page-program boundary -- a single
 *                             page-program command cannot cross it (the
 *                             address wraps within the 256-byte page).
 *                             Used as the MTD block unit (geo->blocksize)
 *                             for bread/bwrite.
 *   - read          = any   : reads are byte-granular (FLASH_ReadStream).
 * PAGE_SIZE is the NOR program page, NOT a NAND page -- it applies to NOR
 * too.
 */

#define AMEBA_FLASH_SECTOR_SIZE   4096
#define AMEBA_FLASH_PAGE_SIZE     256

/* Data partition used for the NuttX filesystem -- the SDK's "VFS1" region,
 * which the vendor reserves for the littlefs/KV store and which does NOT
 * overlap the NP/AP image2 or OTA partitions.
 *
 * The geometry is NOT hardcoded here: it is taken from the SDK flash layout
 * (CONFIG_FLASH_VFS1_OFFSET/SIZE), which the board Make.defs extracts from
 * the regenerated platform_autoconf.h and passes in as
 * AMEBA_FLASH_VFS1_OFFSET_XIP / AMEBA_FLASH_VFS1_SIZE_CFG.  The SDK
 * menuconfig is thus the single source of truth -- change the partition
 * table there and this follows automatically.
 * The fallbacks below are the vendor default (used only if the board did not
 * provide the -D, e.g. a clean-clone first parse before PREBUILD ran).
 *
 * CONFIG_FLASH_VFS1_OFFSET is an XIP address (flash region base 0x08000000);
 * the FLASH_xxx primitives want the flash byte offset, so strip the base.
 */

#ifndef AMEBA_FLASH_VFS1_OFFSET_XIP
#  define AMEBA_FLASH_VFS1_OFFSET_XIP 0x083e0000
#endif
#ifndef AMEBA_FLASH_VFS1_SIZE_CFG
#  define AMEBA_FLASH_VFS1_SIZE_CFG   0x00020000
#endif

#define AMEBA_FLASH_XIP_BASE      0x08000000
#define AMEBA_FLASH_VFS1_OFFSET   (AMEBA_FLASH_VFS1_OFFSET_XIP - AMEBA_FLASH_XIP_BASE)
#define AMEBA_FLASH_VFS1_SIZE     AMEBA_FLASH_VFS1_SIZE_CFG

/* Where the data partition is mounted, and the WiFi key-value sub-directory
 * under it (rt_kv_set/get store one file per key here).  Kept in this shared
 * header so the board mount code and ameba_kv.c agree.
 */

#define AMEBA_FLASH_FS_DEVPATH    "/dev/ameba-nor"
#define AMEBA_FLASH_FS_MOUNTPT    "/data"
#define AMEBA_KV_DIR              "/data/kv"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_flash_mtd_initialize
 *
 * Description:
 *   Create an MTD device over a region of the on-chip SPI NOR flash, backed
 *   by the SDK's interrupt-safe, dual-core-locked FLASH_xxx primitives.
 *
 * Input Parameters:
 *   offset - Flash byte offset of the partition start (sector aligned).
 *   nbytes - Partition size in bytes (sector-size multiple).
 *
 * Returned Value:
 *   An initialized MTD device pointer on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *ameba_flash_mtd_initialize(uint32_t offset,
                                                 uint32_t nbytes);

/****************************************************************************
 * Name: ameba_flash_fs_initialize
 *
 * Description:
 *   Create the VFS1-partition MTD device, register it at
 *   AMEBA_FLASH_FS_DEVPATH and mount a littlefs filesystem on it at
 *   AMEBA_FLASH_FS_MOUNTPT (formatting it on first use).  Invoked from the
 *   board bring-up.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_flash_fs_initialize(void);

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_FLASH_MTD_H */
