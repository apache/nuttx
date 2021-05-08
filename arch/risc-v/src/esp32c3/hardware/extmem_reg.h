/****************************************************************************
 * arch/risc-v/src/esp32c3/hardware/extmem_reg.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_HARDWARE_EXTMEM_REG_H_
#define __ARCH_RISCV_SRC_ESP32C3_HARDWARE_EXTMEM_REG_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32c3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EXTMEM_CACHE_MMU_POWER_CTRL_REG     (DR_REG_EXTMEM_BASE + 0x0AC)
#define EXTMEM_ICACHE_TAG_POWER_CTRL_REG    (DR_REG_EXTMEM_BASE + 0x008)

/* EXTMEM_CACHE_MMU_MEM_FORCE_ON : R/W ;bitpos:[0] ;default: 1'b1.
 * The bit is used to enable clock gating to save
 * power when access mmu memory  0: enable  1: disable
 */

#define EXTMEM_CACHE_MMU_MEM_FORCE_ON    (BIT(0))
#define EXTMEM_CACHE_MMU_MEM_FORCE_ON_M  (BIT(0))
#define EXTMEM_CACHE_MMU_MEM_FORCE_ON_V  0x1
#define EXTMEM_CACHE_MMU_MEM_FORCE_ON_S  0

/* EXTMEM_ICACHE_TAG_MEM_FORCE_ON : R/W ;bitpos:[0] ;default: 1'b1.
 * description: The bit is used to close clock gating of  icache tag memory.
 * 1: close gating  0: open clock gating.
 */

#define EXTMEM_ICACHE_TAG_MEM_FORCE_ON    (BIT(0))
#define EXTMEM_ICACHE_TAG_MEM_FORCE_ON_M  (BIT(0))
#define EXTMEM_ICACHE_TAG_MEM_FORCE_ON_V  0x1
#define EXTMEM_ICACHE_TAG_MEM_FORCE_ON_S  0

#endif /* __ARCH_RISCV_SRC_ESP32C3_HARDWARE_EXTMEM_REG_H_ */
