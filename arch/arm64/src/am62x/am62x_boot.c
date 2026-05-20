/****************************************************************************
 * arch/arm64/src/am62x/am62x_boot.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>

#include <arch/chip/chip.h>

#ifdef CONFIG_SMP
#  include "arm64_smp.h"
#endif

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"
#include "am62x_boot.h"
#include "am62x_serial.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MMU region table.
 *
 * Two flat-mapped regions are required:
 *
 *  1. DEVICE_REGION  — peripheral space 0x0000_0000 – 0x7FFF_FFFF.
 *     Mapped as device-nGnRnE, read-write, secure.  This covers UART, GIC,
 *     GPIO, I2C, SPI, USB, MMC, etc. in one shot without needing to list
 *     each peripheral separately.
 *
 *  2. DRAM0_S0 — the main DDR window starting at 0x8000_0000.
 *     Mapped as normal cacheable memory, read-write, secure.
 *     Size comes from CONFIG_RAMBANK1_SIZE (512 MB for PocketBeagle 2).
 *
 * Both addresses and sizes must be page-aligned (4 KB minimum).
 */

static const struct arm_mmu_region g_mmu_regions[] =
{
  MMU_REGION_FLAT_ENTRY("DEVICE_REGION",
                        CONFIG_DEVICEIO_BASEADDR,
                        CONFIG_DEVICEIO_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("DRAM0_S0",
                        CONFIG_RAMBANK1_ADDR,
                        CONFIG_RAMBANK1_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),
};

const struct arm_mmu_config g_mmu_config =
{
  .num_regions = nitems(g_mmu_regions),
  .mmu_regions = g_mmu_regions,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_el_init
 *
 * Description:
 *   Called from arm64_head.S at EL2 or EL3 (whichever is the entry EL)
 *   before dropping to EL1.  Use this hook for:
 *     - Early platform-level hardware that must be touched at high EL.
 *     - Hypervisor / secure monitor initialisation.
 *
 *   On AM62x the TI R5 SYSFW (TIFS) runs in the MCU domain and has already
 *   completed secure world setup before releasing the Cortex-A53 cluster.
 *   U-Boot then sets up DDR, clocks, and UART before loading NuttX.
 *   There is nothing left for us to do at EL2/EL3.
 *
 ****************************************************************************/

void arm64_el_init(void)
{
  /* Nothing to do — SYSFW and U-Boot have completed all high-EL init. */
}

/****************************************************************************
 * Name: arm64_chip_boot
 *
 * Description:
 *   Called from arm64_boot.c (common layer) after the C runtime is ready.
 *   This is the SoC-level continuation of the boot sequence:
 *     1. Initialise the MMU with the region table above.
 *     2. Optionally initialise PSCI for SMP bring-up.
 *     3. Call the board-specific initialisation hook.
 *     4. Initialise the early serial console so boot messages work.
 *
 ****************************************************************************/

void arm64_chip_boot(void)
{
#ifdef CONFIG_ARCH_USE_MMU
  arm64_mmu_init(true);
#endif

#if defined(CONFIG_ARM64_PSCI) && defined(CONFIG_SMP)
  /* Keep PSCI out of the initial UP bring-up path until the basic
   * boot-to-NSH flow is stable on AM62x.  PSCI is still available for
   * future SMP enablement.
   */

  arm64_psci_init("smc");
#endif

  am62x_board_initialize();

#ifdef USE_EARLYSERIALINIT
  arm64_earlyserialinit();
#endif
}

/****************************************************************************
 * Name: arm64_netinitialize  (stub)
 *
 * Description:
 *   Network device initialisation hook called by the common layer when
 *   CONFIG_NET is enabled and CONFIG_NETDEV_LATEINIT is not set.
 *   Ethernet support is not part of the initial port.
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void arm64_netinitialize(void)
{
  /* TODO: add CPSW / MDIO Ethernet support in a later phase. */
}
#endif
