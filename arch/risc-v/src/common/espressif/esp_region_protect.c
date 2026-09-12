/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_region_protect.c
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
 * Unlocked build of the HAL's esp_cpu_configure_region_protection().
 *
 * The HAL implementation programs every PMP entry with the lock bit set.  It
 * runs very early, from esp_start() -> bootloader_init() ->
 * bootloader_init_mem(), and that path is not gated by
 * CONFIG_ESPRESSIF_REGION_PROTECTION -- that option guards only a second,
 * redundant call later in esp_start().
 *
 * PMP lock bits cannot be cleared without the Smepmp extension, which the
 * ESP32-P4 does not implement (reading mseccfg raises an illegal
 * instruction).  Once the HAL has run, 14 of the 16 entries are dead for the
 * remainder of the boot, and one of the survivors grants U-mode read/write
 * across the whole kernel data region.  A protected build therefore has to
 * stop the entries being locked in the first place.
 *
 * The same function also programs the 16 PMA entries that mark the invalid
 * address ranges and make external flash/RAM, ROM and L2MEM cacheable.  That
 * work is required for the SoC to run at all: replacing this function with
 * an empty stub boot-loops inside bootloader_init().
 *
 * So rather than reimplement any of it, the HAL translation unit is dropped
 * from the build (see hal_<chip>.mk / hal_<chip>.cmake) and its source is
 * included below with PMP_L defined to zero.  Including "riscv/csr.h" first
 * means its include guard is already set when the HAL source includes it
 * again, so the redefinition survives.  Everything else -- the PMA setup,
 * the chip-revision variants, the PSRAM handling -- stays byte for byte
 * identical to the vendored source and tracks it when the HAL is updated.
 *
 * The result is the HAL's own region layout with every PMP entry left
 * unlocked, so the kernel can re-describe them for a kernel/user split.
 *
 * NOTE: unlocked PMP entries do not constrain machine mode, so this reduces
 * the protection a flat build gets.  It is meant to be paired with a
 * protected build, whose userspace initialisation re-establishes the
 * boundaries.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Pulled in first so that its include guard is already set when the HAL
 * source below includes it again, keeping the redefinition that follows.
 */

#include "riscv/csr.h"

#undef PMP_L
#define PMP_L 0

/* The HAL implementation itself, compiled with the lock bit cleared.  The
 * chip directory is spelled out rather than derived from
 * CONFIG_ESPRESSIF_CHIP_SERIES because an #include directive cannot
 * concatenate string literals; CONFIG_ESPRESSIF_KERNEL_OWNS_PMP depends on
 * ARCH_CHIP_ESP32P4, so this file is only ever built for that chip.
 */

#include "../../chip/esp-hal-3rdparty/components/esp_hw_support/port/esp32p4/cpu_region_protect.c"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The whole point of this file is that the HAL body above was compiled with
 * the lock bit cleared.  If the HAL ever reorganises its includes so that
 * "riscv/csr.h" is no longer guarded by the time it is reached, the
 * redefinition would be silently undone and every PMP entry would be locked
 * again -- which would not fail the build, and would not show up until the
 * kernel tried to reprogram an entry at run time.  Fail loudly instead.
 */

#if PMP_L != 0
#  error "PMP lock bit was not neutralised"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* esp_cpu_configure_region_protection() is defined by the HAL source
 * included above, and is the only symbol this translation unit exports.
 */
