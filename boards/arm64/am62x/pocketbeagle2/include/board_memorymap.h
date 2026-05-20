/****************************************************************************
 * boards/arm64/am62x/pocketbeagle2/include/board_memorymap.h
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

/* Board memory map */

#ifndef __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_MEMORYMAP_H
#define __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I/O flat-map region — covers all AM62x peripheral space.
 * Must be page-aligned.  Used by CONFIG_DEVICEIO_BASEADDR / SIZE in
 * defconfig and by g_mmu_regions in am62x_boot.c.
 */

#define POCKETBEAGLE2_DEVICEIO_BASE  0x00000000ul
#define POCKETBEAGLE2_DEVICEIO_SIZE  0x80000000ul  /* 2 GB peripheral space */

/* DDR region — 512 MB on PocketBeagle 2.
 * RAM_START and RAM_SIZE go into defconfig as CONFIG_RAMBANK1_ADDR/SIZE
 * and are also used by the linker script to place the heap.
 */

#define POCKETBEAGLE2_DDR_BASE       0x80000000ul
#define POCKETBEAGLE2_DDR_SIZE       0x20000000ul  /* 512 MB */

/* NuttX load address — must match U-Boot's kernel_addr_r.
 * The linker script uses this as the origin of the .text section.
 */

#define POCKETBEAGLE2_LOAD_ADDR      0x82000000ul

#endif /* __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_MEMORYMAP_H */
