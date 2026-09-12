/****************************************************************************
 * boards/risc-v/esp32p4/common/scripts/kernel-space.ld
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
 * Kernel image layout for the protected build.
 *
 * The kernel's own output sections are identical to the flat build's, so
 * rather than maintaining a second copy of a 700-line script this exports
 * the region boundaries and then includes esp32p4_sections.ld verbatim.
 * The split between kernel and user is expressed entirely in
 * esp32p4_protected_memory.ld, whose REGION_ALIAS definitions point the
 * flat script's region names at the kernel's half of the map.
 *
 * The symbols below are what the kernel needs at run time.  The __u* ones
 * are consumed by esp_userspace.c to program the flash MMU and the PMP; the
 * __k* ones are exported for debugging and for board_memorymap.h.
 ****************************************************************************/

#include <nuttx/config.h>

/* User regions.  esp_userspace.c uses these to describe what user mode may
 * reach; everything not covered is denied to user mode by default.
 */

__uirom_start = ORIGIN(UIROM);
__uirom_size  = LENGTH(UIROM);
__uirom_end   = ORIGIN(UIROM) + LENGTH(UIROM);
__udrom_start = ORIGIN(UDROM);
__udrom_size  = LENGTH(UDROM);
__udrom_end   = ORIGIN(UDROM) + LENGTH(UDROM);
__udram_start = ORIGIN(UDRAM);
__udram_size  = LENGTH(UDRAM);
__udram_end   = ORIGIN(UDRAM) + LENGTH(UDRAM);

/* There is no UIRAM region: user code executes in place from flash.  The
 * symbols are still defined, pointing at an empty span at the base of
 * UDRAM, so that board_memorymap.h can declare them unconditionally.
 */

__uiram_start = ORIGIN(UDRAM);
__uiram_size  = 0;
__uiram_end   = ORIGIN(UDRAM);

/* Kernel regions */

__kirom_start = ORIGIN(KIROM);
__kirom_size  = LENGTH(KIROM);
__kdrom_start = ORIGIN(KDROM);
__kdrom_size  = LENGTH(KDROM);
__kiram_start = ORIGIN(KRAM);
__kiram_size  = LENGTH(KRAM);
__kiram_end   = ORIGIN(KRAM) + LENGTH(KRAM);
__kdram_start = ORIGIN(KRAM);
__kdram_size  = LENGTH(KRAM);
__kdram_end   = ORIGIN(KRAM) + LENGTH(KRAM);

#include "esp32p4_sections.ld"
