/****************************************************************************
 * boards/risc-v/litex/arty_a7/include/board_memorymap.h
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

#ifndef __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_BOARD_MEMORYMAP_H
#define __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_BOARD_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Kernel code memory (RX) */

#define KFLASH_START    (uintptr_t)__kflash_start
#define KFLASH_SIZE     (uintptr_t)__kflash_size
#define KSRAM_START     (uintptr_t)__ksram_start
#define KSRAM_SIZE      (uintptr_t)__ksram_size
#define KSRAM_END       (uintptr_t)__ksram_end

/* Kernel RAM (RW) */

#define PGPOOL_START    (uintptr_t)__pgheap_start
#define PGPOOL_SIZE     (uintptr_t)__pgheap_size

/* Page pool (RWX) */

#define PGPOOL_START    (uintptr_t)__pgheap_start
#define PGPOOL_SIZE     (uintptr_t)__pgheap_size
#define PGPOOL_END      (PGPOOL_START + PGPOOL_SIZE)

/* User flash */

#define UFLASH_START    (uintptr_t)__uflash_start
#define UFLASH_SIZE     (uintptr_t)__uflash_size

/* RAMDisk */

#define RAMDISK_START     (uintptr_t)__ramdisk_start
#define RAMDISK_SIZE      (uintptr_t)__ramdisk_size

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Kernel code memory (RX)  */

extern uint8_t          __kflash_start[];
extern uint8_t          __kflash_size[];

/* Kernel RAM (RW) */

extern uint8_t          __ksram_start[];
extern uint8_t          __ksram_size[];
extern uint8_t          __ksram_end[];

/* Page pool (RWX) */

extern uint8_t          __pgheap_start[];
extern uint8_t          __pgheap_size[];

/* User code memory (RX) */

extern uint8_t          __uflash_start[];
extern uint8_t          __uflash_size[];

/* ramdisk (RW) */

extern uint8_t          __ramdisk_start[];
extern uint8_t          __ramdisk_size[];

#endif /* __BOARDS_RISC_V_LITEX_ARTY_A7_INCLUDE_BOARD_MEMORYMAP_H */
