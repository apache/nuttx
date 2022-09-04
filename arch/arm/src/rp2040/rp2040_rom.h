/****************************************************************************
 * arch/arm/src/rp2040/rp2040_rom.h
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

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROM_HWORD_AS_PTR(a) ((void *)(uintptr_t) (*(uint16_t *)(uintptr_t)a))

#define ROM_CODE(a,b) ((b << 8) | a)

#define ROM_POPCOUNT32             ROM_CODE('P', '3')
#define ROM_REVERSE32              ROM_CODE('R', '3')
#define ROM_CLZ32                  ROM_CODE('L', '3')
#define ROM_CTZ32                  ROM_CODE('T', '3')
#define ROM_MEMSET                 ROM_CODE('M', 'S')
#define ROM_MEMSET4                ROM_CODE('S', '4')
#define ROM_MEMCPY                 ROM_CODE('M', 'C')
#define ROM_MEMCPY44               ROM_CODE('C', '4')
#define ROM_RESET_USB_BOOT         ROM_CODE('U', 'B')
#define ROM_FLASH_CONNECT          ROM_CODE('I', 'F')
#define ROM_FLASH_EXIT_XIP         ROM_CODE('E', 'X')
#define ROM_FLASH_ERASE            ROM_CODE('R', 'E')
#define ROM_FLASH_PROGRAM          ROM_CODE('R', 'P')
#define ROM_FLASH_FLUSH_CACHE      ROM_CODE('F', 'C')
#define ROM_FLASH_ENABLE_XIP       ROM_CODE('C', 'X')

#define ROM_LOOKUP(x) \
          ((rom_table_lookup_fn)ROM_HWORD_AS_PTR(0x18)) \
          (ROM_HWORD_AS_PTR(0x14),x)

#define STR(s)           #s
#define RAM_CODE_ATTR(f) __attribute__((noinline, section(".ram_code." f)))
#define RAM_CODE(f)      RAM_CODE_ATTR(STR(f)) f

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef void *(*rom_table_lookup_fn)(uint16_t *table, uint32_t code);
