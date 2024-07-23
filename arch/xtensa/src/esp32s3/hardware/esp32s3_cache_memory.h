/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_cache_memory.h
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_CACHE_MEMORY_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_CACHE_MEMORY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "esp32s3_soc.h"
#include "soc/ext_mem_defs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_MMU_TABLE                 ((volatile uint32_t *)DR_REG_MMU_TABLE)
#define FLASH_MMU_TABLE_SIZE            (ICACHE_MMU_SIZE / sizeof(uint32_t))

#define MMU_TABLE_INVALID_VAL           0x4000
#define FLASH_MMU_TABLE_INVALID_VAL     DPORT_MMU_TABLE_INVALID_VAL
#define MMU_ADDRESS_MASK                0x3fff
#define MMU_PAGE_SIZE                   0x10000
#define INVALID_PHY_PAGE                0xffff

#define BUS_ADDR_SIZE                   0x200000
#define BUS_ADDR_MASK                   (BUS_ADDR_SIZE - 1)

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_CACHE_MEMORY_H */
