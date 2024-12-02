/****************************************************************************
 * arch/arm64/src/imx9/imx9_boot.c
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
#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/chip/chip.h>
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_mmu.h"

#include "imx9_boot.h"
#include "imx9_clockconfig.h"
#include "imx9_ccm.h"
#include "imx9_trdc.h"
#include "imx9_serial.h"
#include "imx9_gpio.h"
#include "imx9_lowputc.h"
#include "imx9_system_ctl.h"
#ifdef CONFIG_IMX9_DDR_TRAINING
#include "ddr/imx9_ddr_training.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct arm_mmu_region g_mmu_regions[] =
{
  MMU_REGION_FLAT_ENTRY("DEVICE_REGION",
                        CONFIG_DEVICEIO_BASEADDR, CONFIG_DEVICEIO_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),

  MMU_REGION_FLAT_ENTRY("DRAM0_S0",
                        CONFIG_RAMBANK1_ADDR, CONFIG_RAMBANK1_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),

#ifndef CONFIG_IMX9_DDR_TRAINING /* OCRAM set at arm64_mmu.c */
  MMU_REGION_FLAT_ENTRY("OCRAM",
                        CONFIG_OCRAM_BASE_ADDR, CONFIG_OCRAM_SIZE,
                        MT_NORMAL | MT_RW | MT_SECURE),
#endif

  MMU_REGION_FLAT_ENTRY("FSPI_PERIPHERAL",
                        CONFIG_FSPI_PER_BASEADDR, CONFIG_FSPI_PER_SIZE,
                        MT_DEVICE_NGNRNE | MT_RW | MT_SECURE),
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
 *   The function called from arm64_head.S at very early stage for these
 * platform, it's use to:
 *   - Handling special hardware initialize routine which is need to
 *     run at high ELs
 *   - Initialize system software such as hypervisor or security firmware
 *     which is need to run at high ELs
 *
 ****************************************************************************/

void arm64_el_init(void)
{
#if (CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3)
  /* At EL3, cntfrq_el0 is uninitialized. It must be set. */

  uint32_t freq;

  /* Read Frequency ID0 (24MHz) and write it to arm core */

  freq = getreg32(IMX9_SYS_CTR_CONTROL_BASE + SYS_CTR_CNTFID0);
  write_sysreg(freq, cntfrq_el0);
  modifyreg32(IMX9_SYS_CTR_CONTROL_BASE + SYS_CTR_CNTCR, SC_CNTCR_FREQ1,
              SC_CNTCR_FREQ0 | SC_CNTCR_ENABLE | SC_CNTCR_HDBG);
#endif
}

/****************************************************************************
 * Name: arm64_chip_boot
 *
 * Description:
 *   Complete boot operations started in arm64_head.S
 *
 ****************************************************************************/

void arm64_chip_boot(void)
{
#ifdef CONFIG_IMX9_BOOTLOADER
  imx9_mix_powerup();

  /* Before DDR init we need to initialize clocks and trdc */

  imx9_ccm_clock_init();

  imx9_trdc_init();

  imx9_clockconfig();

#ifdef CONFIG_IMX9_DDR_TRAINING
  imx9_dram_init();
#endif
#endif

  /* MAP IO and DRAM, enable MMU. */

  arm64_mmu_init(true);

  /* Do UART early initialization & pin muxing */

#ifdef CONFIG_IMX9_LPUART
  imx9_lowsetup();
#endif

#if defined(CONFIG_ARM64_PSCI)
  arm64_psci_init("smc");
#endif

  /* Initialize pin interrupt support */

#ifdef CONFIG_IMX9_GPIO_IRQ
  imx9_gpioirq_initialize();
#endif

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

  imx9_board_initialize();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm64_earlyserialinit();
#endif
}
