/****************************************************************************
 * arch/xtensa/include/esp32/memory_layout.h
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
 * CONFIG_HEAP2_BASE                         eg. 3f80 0000
 *     :
 *     : g_mmheap region3 (CONFIG_ESP32_SPIRAM)
 *     :
 * CONFIG_HEAP2_BASE + CONFIG_HEAP2_SIZE     eg. 3fc0 0000
 *
 * _sheap                                    eg. 3ffc 8c6c
 *     :
 *     : g_mmheap region1
 *     :
 * HEAP_REGION1_END                              3ffd fff0
 *     :
 *     : ROM data
 *     :
 * HEAP_REGION2_START                            3ffe 1330 or 3ffe 7e40
 *     :
 *     : g_iheap (CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
 *     :
 * HEAP_REGION2_START + CONFIG_XTENSA_IMEM_REGION_SIZE
 *     :
 *     : g_mmheap region2
 *     :
 *     : about 123KB
 *     :
 * _eheap                                        4000 0000
 */

/* Region 1 of the heap is the area from the end of the .data section to the
 * beginning of the ROM data.  The start address is defined from the linker
 * script as "_sheap".  Then end is defined here, as follows:
 */

#ifndef HEAP_REGION1_END
#  define HEAP_REGION1_END    0x3ffdfff0
#endif

/* Region 2 of the heap is the area from the end of the ROM data to the end
 * of DRAM.  The linker script has already set "_eheap" as the end of DRAM,
 * the following defines the start of region2.
 * N.B: That ROM data consists of 2 regions, one per CPU.  If SMP is not
 * enabled include APP's region with the heap.
 *
 * When an internal heap is enabled this region starts at an offset equal to
 * the size of the internal heap. (see esp32_imem.c)
 */

#ifndef CONFIG_SMP
#  define HEAP_REGION2_START  0x3ffe1330
#else
#  define HEAP_REGION2_START  0x3ffe7e40
#endif

#ifdef CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP
#  define	XTENSA_IMEM_REGION_SIZE	CONFIG_XTENSA_IMEM_REGION_SIZE
#else
#  define	XTENSA_IMEM_REGION_SIZE	0
#endif

/* Internal heap starts at the end of the ROM data. */

#ifndef CONFIG_SMP
#  define ESP32_IMEM_START  0x3ffe1330
#else
#  define ESP32_IMEM_START  0x3ffe7e40
#endif

