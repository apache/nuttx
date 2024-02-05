/****************************************************************************
 * arch/xtensa/include/esp32s3/memory_layout.h
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

/* The heap overview:
 *
 * HEAP_REGION0_START                        g_allocable_vaddr_start
 *     :
 *     : g_mmheap (CONFIG_ESP32_SPIRAM)
 *     :
 * HEAP_REGION0_END                          g_allocable_vaddr_end
 *     :
 *---------------------------------------------------------------------
 * if CONFIG_MM_KERNEL_HEAP && MM_USER_HEAP_IRAM
 *
 * HEAP_REGION1_START                        _sheap
 *     :
 *     : g_iheap (CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
 *     :
 * HEAP_REGION1_START + CONFIG_XTENSA_IMEM_REGION_SIZE
 *     :
 *     : g_mmheap region1
 *     :
 * HEAP_REGION1_END                          3fcc fff0
 *     :
 * HEAP_REGION2_START                        3fcd 0000
 *     :
 *     : g_mmheap region2
 *     :
 * HEAP_REGION2_END                          dram0_rtos_reserved_start
 *     :
 *---------------------------------------------------------------------
 * if !CONFIG_MM_KERNEL_HEAP || !MM_USER_HEAP_IRAM
 *
 * HEAP_REGION1_START                            _sheap
 *     :
 *     : g_iheap (CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
 *     :
 * HEAP_REGION1_START + CONFIG_XTENSA_IMEM_REGION_SIZE
 *     :
 *     : g_mmheap region1
 *     :
 * HEAP_REGION1_END                          dram0_rtos_reserved_start
 *     :
 *---------------------------------------------------------------------
 */

#ifdef CONFIG_MM_KERNEL_HEAP
#  define HEAP_REGION1_END      0x3fccfff0
#  define HEAP_REGION2_START    0x3fcd0000
#endif

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
#  define XTENSA_IMEM_REGION_SIZE  CONFIG_XTENSA_IMEM_REGION_SIZE
#else
#  define XTENSA_IMEM_REGION_SIZE  0
#endif
