/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_rom.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ROM FUNCTIONS */

/* RP2040 & RP2350 */
#define ROM_DATA_SOFTWARE_GIT_REVISION          ROM_TABLE_CODE('G', 'R')
#define ROM_FUNC_FLASH_ENTER_CMD_XIP            ROM_TABLE_CODE('C', 'X')
#define ROM_FUNC_FLASH_EXIT_XIP                 ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_FLUSH_CACHE              ROM_TABLE_CODE('F', 'C')
#define ROM_FUNC_CONNECT_INTERNAL_FLASH         ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_RANGE_ERASE              ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM            ROM_TABLE_CODE('R', 'P')

/* RP2350 only */
#define ROM_FUNC_PICK_AB_PARTITION              ROM_TABLE_CODE('A', 'B')
#define ROM_FUNC_CHAIN_IMAGE                    ROM_TABLE_CODE('C', 'I')
#define ROM_FUNC_EXPLICIT_BUY                   ROM_TABLE_CODE('E', 'B')
#define ROM_FUNC_FLASH_RUNTIME_TO_STORAGE_ADDR  ROM_TABLE_CODE('F', 'A')
#define ROM_DATA_FLASH_DEVINFO16_PTR            ROM_TABLE_CODE('F', 'D')
#define ROM_FUNC_FLASH_OP                       ROM_TABLE_CODE('F', 'O')
#define ROM_FUNC_GET_B_PARTITION                ROM_TABLE_CODE('G', 'B')
#define ROM_FUNC_GET_PARTITION_TABLE_INFO       ROM_TABLE_CODE('G', 'P')
#define ROM_FUNC_GET_SYS_INFO                   ROM_TABLE_CODE('G', 'S')
#define ROM_FUNC_GET_UF2_TARGET_PARTITION       ROM_TABLE_CODE('G', 'U')
#define ROM_FUNC_LOAD_PARTITION_TABLE           ROM_TABLE_CODE('L', 'P')
#define ROM_FUNC_OTP_ACCESS                     ROM_TABLE_CODE('O', 'A')
#define ROM_DATA_PARTITION_TABLE_PTR            ROM_TABLE_CODE('P', 'T')
#define ROM_FUNC_FLASH_RESET_ADDRESS_TRANS      ROM_TABLE_CODE('R', 'A')
#define ROM_FUNC_REBOOT                         ROM_TABLE_CODE('R', 'B')
#define ROM_FUNC_SET_ROM_CALLBACK               ROM_TABLE_CODE('R', 'C')
#define ROM_FUNC_SECURE_CALL                    ROM_TABLE_CODE('S', 'C')
#define ROM_FUNC_SET_NS_API_PERMISSION          ROM_TABLE_CODE('S', 'P')
#define ROM_FUNC_BOOTROM_STATE_RESET            ROM_TABLE_CODE('S', 'R')
#define ROM_FUNC_SET_BOOTROM_STACK              ROM_TABLE_CODE('S', 'S')
#define ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR       ROM_TABLE_CODE('X', 'F')
#define ROM_FUNC_FLASH_SELECT_XIP_READ_MODE     ROM_TABLE_CODE('X', 'M')
#define ROM_FUNC_VALIDATE_NS_BUFFER             ROM_TABLE_CODE('V', 'B')

/* these form a bit set */
#define BOOTROM_STATE_RESET_CURRENT_CORE 0x01
#define BOOTROM_STATE_RESET_OTHER_CORE   0x02
#define BOOTROM_STATE_RESET_GLOBAL_STATE 0x04 /* reset any global state (e.g. permissions) */

#define RT_FLAG_FUNC_RISCV      0x0001
#define RT_FLAG_FUNC_RISCV_FAR  0x0003
#define RT_FLAG_FUNC_ARM_SEC    0x0004
/* reserved for 32-bit pointer: 0x0008 */
#define RT_FLAG_FUNC_ARM_NONSEC 0x0010

#define BOOTROM_FUNC_TABLE_OFFSET 0x14

#define BOOTROM_IS_A2() ((*(volatile uint8_t *)0x13) == 2)
#define BOOTROM_WELL_KNOWN_PTR_SIZE (BOOTROM_IS_A2() ? 2 : 4)

#if defined(__riscv)
#define BOOTROM_ENTRY_OFFSET            0x7dfc
#define BOOTROM_TABLE_LOOKUP_ENTRY_OFFSET (BOOTROM_ENTRY_OFFSET - BOOTROM_WELL_KNOWN_PTR_SIZE)
#define BOOTROM_TABLE_LOOKUP_OFFSET     (BOOTROM_ENTRY_OFFSET - BOOTROM_WELL_KNOWN_PTR_SIZE*2)
#else
#define BOOTROM_VTABLE_OFFSET 0x00
#define BOOTROM_TABLE_LOOKUP_OFFSET     (BOOTROM_FUNC_TABLE_OFFSET + BOOTROM_WELL_KNOWN_PTR_SIZE)
#endif

#define ROM_TABLE_CODE(c1, c2) ((c1) | ((c2) << 8))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef void *(*rom_table_lookup_fn)(uint32_t code, uint32_t mask);

static __inline void *rom_func_lookup(uint32_t code)
{
#ifdef __riscv
  uint32_t rom_offset_adjust = rom_size_is_64k() ? 32 * 1024 : 0;

  /* on RISC-V the code (a jmp) is actually embedded in the table */

  rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) (uintptr_t)
    *(uint16_t *)(BOOTROM_TABLE_LOOKUP_ENTRY_OFFSET + rom_offset_adjust);

  return rom_table_lookup(code, RT_FLAG_FUNC_RISCV);
#else
  /* on ARM the function pointer is stored in the table, so we dereference
   * it via lookup() rather than lookup_entry()
   */

  rom_table_lookup_fn rom_table_lookup = (rom_table_lookup_fn) (uintptr_t)
    *(uint16_t *)(BOOTROM_TABLE_LOOKUP_OFFSET);
  if (pico_processor_state_is_nonsecure())
    {
      return rom_table_lookup(code, RT_FLAG_FUNC_ARM_NONSEC);
    }
  else
    {
      return rom_table_lookup(code, RT_FLAG_FUNC_ARM_SEC);
    }
#endif
}
