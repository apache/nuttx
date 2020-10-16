/****************************************************************************
 * arch/xtensa/src/esp32/esp32_spiflash.c
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

#ifdef CONFIG_ESP32_SPIRAM

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/config.h>

#include "esp32_spiram.h"
#include "esp32_psram.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef CONFIG_SMP
#  define PSRAM_MODE PSRAM_VADDR_MODE_NORMAL
#else
#if CONFIG_ESP32_MEMMAP_SPIRAM_CACHE_EVENODD
#  define PSRAM_MODE PSRAM_VADDR_MODE_EVENODD
#else
#  define PSRAM_MODE PSRAM_VADDR_MODE_LOWHIGH
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Let's to assume SPIFLASH SPEED == SPIRAM SPEED for now */

#if CONFIG_ESP32_SPIRAM_SPEED_40M
#  define PSRAM_SPEED PSRAM_CACHE_F40M_S40M
#elif CONFIG_ESP32_SPIRAM_SPEED_80M
#  define PSRAM_SPEED PSRAM_CACHE_F80M_S80M
#else
#  error "FLASH speed can only be equal to or higher than SRAM speed while SRAM is enabled!"
#endif

#if defined(CONFIG_BOOT_SDRAM_DATA)
extern uint8_t _ext_ram_bss_start;
extern uint8_t _ext_ram_bss_end;
#endif
static bool spiram_inited = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* If no function in esp_himem.c is used, this function will be linked into
 * the binary instead of the one in esp_himem.c, automatically making sure
 * no memory is reserved if no himem function is used.
 */

size_t __attribute__((weak)) esp_himem_reserved_area_size(void)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void IRAM_ATTR esp_spiram_init_cache(void)
{
  /* Enable external RAM in MMU */

  cache_sram_mmu_set(0, 0, SOC_EXTRAM_DATA_LOW, 0, 32, 128);

  /* Flush and enable icache for APP CPU */

#ifdef CONFIG_SMP
  DPORT_CLEAR_PERI_REG_MASK(DPORT_APP_CACHE_CTRL1_REG,
                            DPORT_APP_CACHE_MASK_DRAM1);
  cache_sram_mmu_set(1, 0, SOC_EXTRAM_DATA_LOW, 0, 32, 128);
#endif
}

int esp_spiram_get_chip_size(void)
{
  int psram_size;

  if (!spiram_inited)
    {
      merr("SPI RAM not initialized");
      return ESP_SPIRAM_SIZE_INVALID;
    }

  psram_size = psram_get_size();
  switch (psram_size)
  {
    case PSRAM_SIZE_16MBITS:
      return ESP_SPIRAM_SIZE_16MBITS;

    case PSRAM_SIZE_32MBITS:
      return ESP_SPIRAM_SIZE_32MBITS;

    case PSRAM_SIZE_64MBITS:
      return ESP_SPIRAM_SIZE_64MBITS;

    default:
      return ESP_SPIRAM_SIZE_INVALID;
  }
}

int esp_spiram_init(void)
{
  int ret;
  ret = psram_enable(PSRAM_SPEED, PSRAM_MODE);
  if (ret != OK)
    {
#ifdef CONFIG_ESP32_SPIRAM_IGNORE_NOTFOUND
      merr("SPI RAM enabled but initialization failed.\
           Bailing out.");
#endif
      return ret;
    }

  /* note: this needs to be set before esp_spiram_get_chip_* /
   * esp_spiram_get_size calls.
   */

  spiram_inited = true;

#if (CONFIG_ESP32_SPIRAM_SIZE != -1)
  if (esp_spiram_get_size() != CONFIG_ESP32_SPIRAM_SIZE)
    {
      merr("Expected %dKiB chip but found %dKiB chip.\
           Bailing out..\n", CONFIG_ESP32_SPIRAM_SIZE / 1024,
           esp_spiram_get_size() / 1024);
      return -EINVAL;
    }
#endif

  minfo("Found %dMBit SPI RAM device\n",
        (esp_spiram_get_size() * 8) / (1024 * 1024));

  minfo("SPI RAM mode: %s\n",
        PSRAM_SPEED == PSRAM_CACHE_F40M_S40M ? "flash 40m sram 40m" : \
        PSRAM_SPEED == PSRAM_CACHE_F80M_S40M ? "flash 80m sram 40m" : \
        PSRAM_SPEED == PSRAM_CACHE_F80M_S80M ? "flash 80m sram 80m" : \
        "ERROR");

  minfo("PSRAM initialized, cache is in %s mode.\n", \
        (PSRAM_MODE == PSRAM_VADDR_MODE_EVENODD) ? "even/odd (2-core)": \
        (PSRAM_MODE == PSRAM_VADDR_MODE_LOWHIGH) ? "low/high (2-core)": \
        (PSRAM_MODE == PSRAM_VADDR_MODE_NORMAL) ? "normal (1-core)":"ERROR");

  return OK;
}

#if 0
/* DMA is not supported yet */

static uint8_t *dma_heap;

int esp_spiram_reserve_dma_pool(size_t size)
{
  minfo("Reserving pool of %dK of internal memory for DMA/internal\
        allocations", size / 1024);

  /* Pool may be allocated in multiple non-contiguous chunks, depending on
   * available RAM
   */

  while (size > 0)
    {
      size_t next_size = heap_caps_get_largest_free_block(MALLOC_CAP_DMA |
                         MALLOC_CAP_INTERNAL);

      next_size = MIN(next_size, size);

      minfo("Allocating block of size %d bytes", next_size);

      dma_heap = heap_caps_malloc(next_size, MALLOC_CAP_DMA |
                                  MALLOC_CAP_INTERNAL);

      if (!dma_heap || next_size == 0)
        {
          return ESP_ERR_NO_MEM;
        }

      uint32_t caps[] =
                        {
                          0, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL,
                          MALLOC_CAP_8BIT | MALLOC_CAP_32BIT
                        };

      int e = heap_caps_add_region_with_caps(caps, (intptr_t) dma_heap,
                                   (intptr_t) dma_heap + next_size - 1);

      if (e != ESP_OK)
        {
          return e;
        }

      size -= next_size;
    }

  return OK;
}
#endif

size_t esp_spiram_get_size(void)
{
  int size = esp_spiram_get_chip_size();

  if (size == PSRAM_SIZE_16MBITS)
    {
      return 2 * 1024 * 1024;
    }

  if (size == PSRAM_SIZE_32MBITS)
    {
      return 4 * 1024 * 1024;
    }

  if (size == PSRAM_SIZE_64MBITS)
    {
      return 8 * 1024 * 1024;
    }

  return CONFIG_ESP32_SPIRAM_SIZE;
}

/* Before flushing the cache, if psram is enabled as a memory-mapped thing,
 * we need to write back the data in the cache to the psram first, otherwise
 * it will get lost. For now, we just read 64/128K of random PSRAM memory to
 * do this. Note that this routine assumes some unique mapping for the first
 * 2 banks of the PSRAM memory range, as well as the 2 banks after the 2 MiB
 * mark.
 */

void IRAM_ATTR esp_spiram_writeback_cache(void)
{
  int x;
  uint32_t regval;
  volatile int i = 0;
  volatile uint8_t *psram = (volatile uint8_t *)SOC_EXTRAM_DATA_LOW;
  int cache_was_disabled = 0;

  if (!spiram_inited)
    {
      return;
    }

  /* We need cache enabled for this to work. Re-enable it if needed; make
   * sure we disable it again on exit as well.
   */

  regval = getreg32(DPORT_PRO_CACHE_CTRL_REG);

  if ((regval & DPORT_PRO_CACHE_ENABLE) == 0)
    {
      cache_was_disabled |= (1 << 0);
      regval  = getreg32(DPORT_PRO_CACHE_CTRL_REG);
      regval |= (1 << DPORT_PRO_CACHE_ENABLE_S);
      putreg32(regval, DPORT_PRO_CACHE_CTRL_REG);
    }

#ifdef CONFIG_SMP
  regval = getreg32(DPORT_APP_CACHE_CTRL_REG);

  if ((regval & DPORT_APP_CACHE_ENABLE) == 0)
    {
      cache_was_disabled |= (1 << 1);
      regval  = getreg32(DPORT_APP_CACHE_CTRL_REG);
      regval |= 1 << DPORT_APP_CACHE_ENABLE_S;
      putreg32(regval, DPORT_APP_CACHE_CTRL_REG);
    }
#endif

#if (PSRAM_MODE != PSRAM_VADDR_MODE_LOWHIGH)
  /* Single-core and even/odd mode only have 32K of cache evenly distributed
   * over the address lines. We can clear the cache by just reading 64K
   * worth of cache lines.
   */

  for (x = 0; x < 1024 * 64; x += 32)
    {
      i += psram[x];
    }
#else
  /* Low/high psram cache mode uses one 32K cache for the lowest 2MiB of SPI
   * flash and another 32K for the highest 2MiB. Clear this by reading from
   * both regions. Note: this assumes the amount of external RAM is >2M.
   * If it is 2M or less, what this code does is undefined. If we ever
   * support external RAM chips of 2M or smaller, this may need adjusting.
   */

  for (x = 0; x < 1024 * 64; x += 32)
    {
      i += psram[x];
      i += psram[x + (1024 * 1024 * 2)];
    }
#endif

  if (cache_was_disabled & (1 << 0))
    {
      while (((getreg32(DPORT_PRO_DCACHE_DBUG0_REG) >>
              (DPORT_PRO_CACHE_STATE_S)) &
              (DPORT_PRO_CACHE_STATE)) != 1)
        {
        };

      regval  = getreg32(DPORT_PRO_CACHE_CTRL_REG);
      regval &= ~(1 << DPORT_PRO_CACHE_ENABLE_S);
      putreg32(regval, DPORT_PRO_CACHE_CTRL_REG);
    }

#ifdef CONFIG_SMP
  if (cache_was_disabled & (1 << 1))
    {
      while (((getreg32(DPORT_APP_DCACHE_DBUG0_REG) >>
              (DPORT_APP_CACHE_STATE_S)) &
              (DPORT_APP_CACHE_STATE)) != 1)
        {
        };

      regval  = getreg32(DPORT_APP_CACHE_CTRL_REG);
      regval &= ~(1 << DPORT_APP_CACHE_ENABLE_S);
      putreg32(regval, DPORT_APP_CACHE_CTRL_REG);
    }
#endif
}

/* If SPI RAM(PSRAM) has been initialized
 *
 * Return:
 *   - true SPI RAM has been initialized successfully
 *   - false SPI RAM hasn't been initialized or initialized failed
 */

bool esp_spiram_is_initialized(void)
{
  return spiram_inited;
}

#endif
