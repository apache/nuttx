/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_userspace.c
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

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/compiler.h>
#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "riscv_internal.h"

#include "esp_userspace.h"

#include "esp_rom_sys.h"
#include "soc/soc.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "hal/mmu_hal.h"
#include "hal/mmu_types.h"
#include "spi_flash_mmap.h"
#include "bootloader_flash_priv.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USER_IMAGE_OFFSET   CONFIG_ESPRESSIF_USER_IMAGE_OFFSET

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Emitted at the start of the user image by user-space.ld.  It tells the
 * kernel where the user image expects its flash-mapped regions to live, so
 * that the flash MMU can be programmed before any user code is reachable.
 */

struct user_image_load_header_s
{
  uintptr_t drom_vma;      /* Destination address (VMA) for DROM region */
  uintptr_t drom_lma;      /* Flash offset (LMA) for start of DROM region */
  uintptr_t drom_size;     /* Size of DROM region */
  uintptr_t irom_vma;      /* Destination address (VMA) for IROM region */
  uintptr_t irom_lma;      /* Flash offset (LMA) for start of IROM region */
  uintptr_t irom_size;     /* Size of IROM region */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct user_image_load_header_s g_header;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: load_header
 *
 * Description:
 *   Read the user image load header out of flash.
 *
 *   Unlike the ESP32-C3 protected port, which temporarily mapped a scratch
 *   flash window to reach the header, this reads it directly.  The SPI flash
 *   driver is already initialised by the time esp_userspace() runs, and
 *   esp_start() uses the same call to parse the kernel image header, so no
 *   scratch mapping is needed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void load_header(void)
{
  int ret = bootloader_flash_read(USER_IMAGE_OFFSET, &g_header,
                                  sizeof(g_header), true);

  if (ret != 0)
    {
      esp_rom_printf("ERROR: user image header read failed: rc=%d at 0x%x\n",
                     ret, (unsigned int)USER_IMAGE_OFFSET);
      PANIC();
    }
}

/****************************************************************************
 * Name: configure_mmu
 *
 * Description:
 *   Map the user image's flash-resident code and read-only data into the
 *   virtual address space.
 *
 *   This is deliberately additive.  esp_start()'s map_rom_segments() calls
 *   mmu_hal_unmap_all() before installing the kernel's own mappings; doing
 *   that here would unmap the kernel while it is executing.  Only the two
 *   user regions are added.
 *
 *   The cache is disabled across the update, so this function -- and
 *   everything it calls -- must be resident in IRAM.  The linker script
 *   places this object there by name.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void noinline_function configure_mmu(void)
{
  uint32_t actual_mapped_len = 0;
  uint32_t drom_lma = USER_IMAGE_OFFSET + g_header.drom_lma;
  uint32_t irom_lma = USER_IMAGE_OFFSET + g_header.irom_lma;
  uint32_t drom_lma_aligned = drom_lma & MMU_FLASH_MASK;
  uint32_t drom_vma_aligned = g_header.drom_vma & MMU_FLASH_MASK;
  uint32_t irom_lma_aligned = irom_lma & MMU_FLASH_MASK;
  uint32_t irom_vma_aligned = g_header.irom_vma & MMU_FLASH_MASK;
  cache_bus_mask_t bus_mask;

  cache_hal_disable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);

  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     drom_vma_aligned, drom_lma_aligned,
                     g_header.drom_size, &actual_mapped_len);

  mmu_hal_map_region(0, MMU_TARGET_FLASH0,
                     irom_vma_aligned, irom_lma_aligned,
                     g_header.irom_size, &actual_mapped_len);

  bus_mask = cache_ll_l1_get_bus(0, drom_vma_aligned, g_header.drom_size);
  cache_ll_l1_enable_bus(0, bus_mask);
  bus_mask = cache_ll_l1_get_bus(0, irom_vma_aligned, g_header.irom_size);
  cache_ll_l1_enable_bus(0, bus_mask);

#if CONFIG_ESPRESSIF_NUM_CPUS > 1
  bus_mask = cache_ll_l1_get_bus(1, drom_vma_aligned, g_header.drom_size);
  cache_ll_l1_enable_bus(1, bus_mask);
  bus_mask = cache_ll_l1_get_bus(1, irom_vma_aligned, g_header.irom_size);
  cache_ll_l1_enable_bus(1, bus_mask);
#endif

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  cache_ll_invalidate_addr(CACHE_LL_LEVEL_ALL, CACHE_TYPE_ALL,
                           CACHE_LL_ID_ALL, irom_vma_aligned,
                           actual_mapped_len);
#endif

  cache_hal_enable(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
}

/****************************************************************************
 * Name: initialize_data
 *
 * Description:
 *   Copy the user image's initialised data from flash into user RAM.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void initialize_data(void)
{
  size_t length = USERSPACE->us_dataend - USERSPACE->us_datastart;
  int ret;

  uintptr_t src = USER_IMAGE_OFFSET + USERSPACE->us_datasource;

  ret = bootloader_flash_read(src, (void *)USERSPACE->us_datastart,
                              length, true);
  if (ret != 0)
    {
      /* The usual cause is alignment: this read wants its flash offset and
       * length aligned, and .data's load address is whatever the end of
       * .rodata left it at.  user-space.ld pads both to 16 for that reason.
       */

      esp_rom_printf("ERROR: user .data read failed: rc=%d src=0x%x "
                     "dst=0x%x len=0x%x\n",
                     ret, (unsigned int)src,
                     (unsigned int)USERSPACE->us_datastart,
                     (unsigned int)length);
      PANIC();
    }
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   Establish the PMP regions that separate kernel from user.
 *
 *   Every entry is reset first.  By the time this runs the HAL has already
 *   programmed eleven regions from bootloader_init(), several of which would
 *   grant user mode access to kernel memory -- entry 5 in particular covers
 *   the whole of SRAM as read/write.  They are only reprogrammable because
 *   CONFIG_ESPRESSIF_KERNEL_OWNS_PMP built them without the lock bit.
 *
 *   Only user-accessible regions are then described.  Per the RISC-V
 *   privileged specification an access matching no PMP entry is permitted in
 *   machine mode and denied in user mode, so the kernel needs no entries of
 *   its own and everything not listed below is automatically inaccessible to
 *   user code.
 *
 *   Regions are expressed as TOR pairs because the linker-defined bounds are
 *   not naturally aligned powers of two.  Every boundary must respect the
 *   128-byte PMP granularity of this SoC; user-space.ld is responsible for
 *   that alignment.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
  const uintptr_t r  = PMPCFG_R;
  const uintptr_t rw = PMPCFG_R | PMPCFG_W;
  const uintptr_t rx = PMPCFG_R | PMPCFG_X;

  /* Drop everything the HAL left behind.  Machine mode is unaffected: an
   * unlocked entry never constrains it, and an address matching no entry is
   * permitted in machine mode.
   */

  riscv_config_pmp_region(0, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(1, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(2, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(3, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(4, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(5, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(6, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(7, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(8, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(9, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(10, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(11, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(12, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(13, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(14, PMPCFG_A_OFF, 0, 0);
  riscv_config_pmp_region(15, PMPCFG_A_OFF, 0, 0);

  /* TOR entries take their lower bound from the preceding entry's address,
   * so the regions must be programmed in ascending address order.  Each pair
   * is an unmatched gap followed by the region proper:
   *
   *   UIROM  0x40200000 - 0x40300000   user code, execute in place
   *   UDROM  0x40300080 - 0x40380000   user rodata (0x80 metadata gap below)
   *   ROM    SOC_IROM_MASK_*           ROM routines the user image calls
   *   UDRAM  0x4ff40000 - 0x4ff80000   user data, bss and heap
   */

  /* User code, executed in place from flash */

  riscv_config_pmp_region(0, PMPCFG_A_TOR, UIROM_START, 0);
  riscv_config_pmp_region(1, PMPCFG_A_TOR | rx, UIROM_END, 0);

  /* User read-only data, mapped from flash */

  riscv_config_pmp_region(2, PMPCFG_A_TOR, UDROM_START, 0);
  riscv_config_pmp_region(3, PMPCFG_A_TOR | r, UDROM_END, 0);

  /* Internal ROM.  CONFIG_LIBC_ARCH_* is selected for this chip, so libc
   * omits its generic memcpy(), strcmp() and friends and the user image is
   * linked against the ROM implementations instead (see the ROM linker
   * scripts in common/kernel/Makefile).  Without this grant the first such
   * call from user mode takes an instruction access fault.
   */

  riscv_config_pmp_region(4, PMPCFG_A_TOR, SOC_IROM_MASK_LOW, 0);
  riscv_config_pmp_region(5, PMPCFG_A_TOR | rx, SOC_IROM_MASK_HIGH, 0);

  /* User data, bss and heap in internal SRAM */

  riscv_config_pmp_region(6, PMPCFG_A_TOR, UDRAM_START, 0);
  riscv_config_pmp_region(7, PMPCFG_A_TOR | rw, UDRAM_END, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_userspace
 *
 * Description:
 *   See esp_userspace.h.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_userspace(void)
{
  uint8_t *dest;
  uint8_t *end;

  /* Where does the user image expect to be mapped? */

  load_header();

  /* Make its code and rodata reachable before anything touches them.  The
   * header values are echoed first: if the metadata block and
   * struct user_image_load_header_s ever disagree, this is where it shows.
   */

  esp_rom_printf("userspace: drom vma=0x%x lma=0x%x size=0x%x\n",
                 (unsigned int)g_header.drom_vma,
                 (unsigned int)g_header.drom_lma,
                 (unsigned int)g_header.drom_size);
  esp_rom_printf("userspace: irom vma=0x%x lma=0x%x size=0x%x\n",
                 (unsigned int)g_header.irom_vma,
                 (unsigned int)g_header.irom_lma,
                 (unsigned int)g_header.irom_size);

  configure_mmu();

  /* Clear all of userspace .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of userspace .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  initialize_data();

  /* Finally, take ownership of the PMP and fence the two worlds apart */

  configure_mpu();

  /* Not showprogress(): that expands to nothing unless
   * CONFIG_DEBUG_FEATURES is set, which would make a hang here
   * indistinguishable from one inside nx_start().
   */

  /* Echo what the kernel will actually act on.  us_entrypoint should be an
   * address inside UIROM and the heap bounds should lie inside UDRAM; if the
   * user image's .userspace structure were mislinked or not loaded, this is
   * where it becomes visible rather than showing up later as a silent
   * failure to start the initial task.
   */

  esp_rom_printf("userspace: entry=0x%x heap=0x%x..0x%x bss=0x%x..0x%x\n",
                 (unsigned int)USERSPACE->us_entrypoint,
                 (unsigned int)USERSPACE->us_bssend,
                 (unsigned int)USERSPACE->us_heapend,
                 (unsigned int)USERSPACE->us_bssstart,
                 (unsigned int)USERSPACE->us_bssend);
  esp_rom_printf("userspace: ready\n");
}

#endif /* CONFIG_BUILD_PROTECTED */
