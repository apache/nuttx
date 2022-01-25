/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_cache.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CACHE_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CACHE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/mpfs_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Base Address ****************************************************/

#define MPFS_CACHE_CTRL_BASE                         0x02010000

/* Register offsets *********************************************************/

#define MPFS_CACHE_CONFIG_OFFSET                     0x000
#define MPFS_CACHE_WAY_ENABLE_OFFSET                 0x008
#define MPFS_CACHE_ECC_INJECT_ERROR_OFFSET           0x040
#define MPFS_CACHE_ECC_DIR_FIX_ADDR_OFFSET           0x100
#define MPFS_CACHE_ECC_DIR_FIX_COUNT_OFFSET          0x108
#define MPFS_CACHE_ECC_DATA_FIX_ADDR_OFFSET          0x140
#define MPFS_CACHE_ECC_DATA_FIX_COUNT_OFFSET         0x148
#define MPFS_CACHE_ECC_DATA_FAIL_ADDR_OFFSET         0x160
#define MPFS_CACHE_ECC_DATA_FAIL_COUNT_OFFSET        0x168
#define MPFS_CACHE_FLUSH64_OFFSET                    0x200
#define MPFS_CACHE_FLUSH32_OFFSET                    0x240
#define MPFS_CACHE_WAY_MASK_DMA_OFFSET               0x800
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_0_OFFSET 0x808
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_1_OFFSET 0x810
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_2_OFFSET 0x818
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_3_OFFSET 0x820
#define MPFS_CACHE_WAY_MASK_E51_DCACHE_OFFSET        0x828
#define MPFS_CACHE_WAY_MASK_E51_ICACHE_OFFSET        0x830
#define MPFS_CACHE_WAY_MASK_U54_1_DCACHE_OFFSET      0x838
#define MPFS_CACHE_WAY_MASK_U54_1_ICACHE_OFFSET      0x840
#define MPFS_CACHE_WAY_MASK_U54_2_DCACHE_OFFSET      0x848
#define MPFS_CACHE_WAY_MASK_U54_2_ICACHE_OFFSET      0x850
#define MPFS_CACHE_WAY_MASK_U54_3_DCACHE_OFFSET      0x858
#define MPFS_CACHE_WAY_MASK_U54_3_ICACHE_OFFSET      0x860
#define MPFS_CACHE_WAY_MASK_U54_4_DCACHE_OFFSET      0x868
#define MPFS_CACHE_WAY_MASK_U54_4_ICACHE_OFFSET      0x870

/* Registers ****************************************************************/

#define MPFS_CACHE_CONFIG                     (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_CONFIG_OFFSET)
#define MPFS_CACHE_WAY_ENABLE                 (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_ENABLE_OFFSET)
#define MPFS_CACHE_ECC_INJECT_ERROR           (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_ECC_INJECT_ERROR_OFFSET)
#define MPFS_CACHE_ECC_DIR_FIX_ADDR           (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_ECC_DIR_FIX_ADDR_OFFSET)
#define MPFS_CACHE_ECC_DIR_FIX_COUNT          (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_ECC_DIR_FIX_COUNT_OFFSET)
#define MPFS_CACHE_ECC_DATA_FAIL_ADDR         (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_ECC_DATA_FAIL_ADDR_OFFSET)
#define MPFS_CACHE_ECC_DATA_FAIL_COUNT        (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_ECC_DATA_FAIL_COUNT_OFFSET)
#define MPFS_CACHE_FLUSH64                    (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_FLUSH64_OFFSET)
#define MPFS_CACHE_FLUSH32                    (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_FLUSH32_OFFSET)
#define MPFS_CACHE_WAY_MASK_DMA               (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_DMA_OFFSET)
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_0 (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_0_OFFSET)
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_1 (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_1_OFFSET)
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_2 (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_2_OFFSET)
#define MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_3 (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_AXI4_SLAVE_PORT_3_OFFSET)
#define MPFS_CACHE_WAY_MASK_E51_DCACHE        (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_E51_DCACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_E51_ICACHE        (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_E51_ICACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_1_DCACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_1_DCACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_1_ICACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_1_ICACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_2_DCACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_2_DCACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_2_ICACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_2_ICACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_3_DCACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_3_DCACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_3_ICACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_3_ICACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_4_DCACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_4_DCACHE_OFFSET)
#define MPFS_CACHE_WAY_MASK_U54_4_ICACHE      (MPFS_CACHE_CTRL_BASE + MPFS_CACHE_WAY_MASK_U54_4_ICACHE_OFFSET)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CACHE_H */
