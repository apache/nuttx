/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash.c
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
#include <string.h>
#include <sys/types.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>

#include "hardware/esp32s3_soc.h"

#include "esp32s3_irq.h"

#include "esp32s3_spiflash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RO data page in MMU index */

#define DROM0_PAGES_START           (2)
#define DROM0_PAGES_END             (128)

/* MMU invalid value */

#define INVALID_MMU_VAL             (0x100)

/* MMU page size */

#define SPI_FLASH_MMU_PAGE_SIZE     (0x10000)

/* MMU base virtual mapped address */

#define VADDR0_START_ADDR           (0x3c020000)

/* Flash MMU table for CPU */

#define MMU_TABLE                   ((volatile uint32_t *)DR_REG_MMU_TABLE)

#define MMU_ADDR2PAGE(_addr)        ((_addr) / SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_ADDR2OFF(_addr)         ((_addr) % SPI_FLASH_MMU_PAGE_SIZE)
#define MMU_BYTES2PAGES(_n)         (((_n) + SPI_FLASH_MMU_PAGE_SIZE - 1) / \
                                     SPI_FLASH_MMU_PAGE_SIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Flash map request data */

struct spiflash_map_req_s
{
  /* Request mapping SPI Flash base address */

  uint32_t  src_addr;

  /* Request mapping SPI Flash size */

  uint32_t  size;

  /* Mapped memory pointer */

  void      *ptr;

  /* Mapped started MMU page index */

  uint32_t  start_page;

  /* Mapped MMU page count */

  uint32_t  page_cnt;
};

/* Structure holding SPI flash access critical sections management
 * functions.
 */

struct spiflash_guard_funcs_s
{
  void (*start)(void);      /* critical section start function */
  void (*end)(void);        /* critical section end function */
  void (*op_lock)(void);    /* flash access API lock function */
  void (*op_unlock)(void);  /* flash access API unlock function */

  /* checks flash write addresses */

  bool (*address_is_safe)(size_t addr, size_t size);

  void (*yield)(void);      /* yield to the OS during flash erase */
};

struct spiflash_cachestate_s
{
  uint32_t value;
  irqstate_t flags;
  int cpu;
#ifdef CONFIG_SMP
  int other;
#endif
};

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

static void spiflash_start(void);
static void spiflash_end(void);

/****************************************************************************
 * Public Functions Declaration
 ****************************************************************************/

extern void spi_flash_guard_set(const struct spiflash_guard_funcs_s *funcs);
extern uint32_t cache_suspend_icache(void);
extern uint32_t cache_suspend_dcache(void);
extern void cache_resume_icache(uint32_t val);
extern void cache_resume_dcache(uint32_t val);
extern int cache_invalidate_addr(uint32_t addr, uint32_t size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spiflash_guard_funcs_s g_spi_flash_guard_funcs =
{
  .start           = spiflash_start,
  .end             = spiflash_end,
  .op_lock         = NULL,
  .op_unlock       = NULL,
  .address_is_safe = NULL,
  .yield           = NULL,
};

static struct spiflash_cachestate_s g_state;

/****************************************************************************
 * Name: spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operation.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_start(void)
{
  g_state.flags = enter_critical_section();
  g_state.cpu = up_cpu_index();
#ifdef CONFIG_SMP
  g_state.other = g_state.cpu ? 0 : 1;
#endif

  DEBUGASSERT(g_state.cpu == 0 || g_state.cpu == 1);
#ifdef CONFIG_SMP
  DEBUGASSERT(g_state.other == 0 || g_state.other == 1);
  DEBUGASSERT(g_state.other != g_state.cpu);
  up_cpu_pause(g_state.other);
#endif

  g_state.value = cache_suspend_icache() << 16;
  g_state.value |= cache_suspend_dcache();
}

/****************************************************************************
 * Name: spiflash_opdone
 *
 * Description:
 *   Undo all the steps of opstart.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_end(void)
{
  DEBUGASSERT(g_state.cpu == 0 || g_state.cpu == 1);

#ifdef CONFIG_SMP
  DEBUGASSERT(g_state.other == 0 || g_state.other == 1);
  DEBUGASSERT(g_state.other != g_state.cpu);
#endif

  cache_resume_icache(g_state.value >> 16);
  cache_resume_dcache(g_state.value & 0xffff);

#ifdef CONFIG_SMP
  up_cpu_resume(g_state.other);
#endif

  leave_critical_section(g_state.flags);
}

/****************************************************************************
 * Name: esp32s3_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32-S3's address bus, so that software
 *   can read SPI Flash data by reading data from memory access.
 *
 *   If SPI Flash hardware encryption is enable, the read from mapped
 *   address is decrypted.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

static IRAM_ATTR int esp32s3_mmap(struct spiflash_map_req_s *req)
{
  int ret;
  int i;
  int start_page;
  int flash_page;
  int page_cnt;
  uint32_t mapped_addr;

  spiflash_start();

  for (start_page = DROM0_PAGES_START;
       start_page < DROM0_PAGES_END;
       ++start_page)
    {
      if (MMU_TABLE[start_page] == INVALID_MMU_VAL)
        {
          break;
        }
    }

  flash_page = MMU_ADDR2PAGE(req->src_addr);
  page_cnt   = MMU_BYTES2PAGES(MMU_ADDR2OFF(req->src_addr) + req->size);

  if (start_page + page_cnt < DROM0_PAGES_END)
    {
      mapped_addr = (start_page - DROM0_PAGES_START) *
                    SPI_FLASH_MMU_PAGE_SIZE +
                    VADDR0_START_ADDR;

      for (i = 0; i < page_cnt; i++)
        {
          MMU_TABLE[start_page + i] = flash_page + i;
          cache_invalidate_addr(mapped_addr + i * SPI_FLASH_MMU_PAGE_SIZE,
                                SPI_FLASH_MMU_PAGE_SIZE);
        }

      req->start_page = start_page;
      req->page_cnt = page_cnt;
      req->ptr = (void *)(mapped_addr + MMU_ADDR2OFF(req->src_addr));
      ret = OK;
    }
  else
    {
      ret = -ENOBUFS;
    }

  spiflash_end();

  return ret;
}

/****************************************************************************
 * Name: esp32s3_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32-S3's address bus, and free resource.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void esp32s3_ummap(const struct spiflash_map_req_s *req)
{
  int i;

  spiflash_start();

  for (i = req->start_page; i < req->start_page + req->page_cnt; ++i)
    {
      MMU_TABLE[i] = INVALID_MMU_VAL;
    }

  spiflash_end();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_flash_read_encrypted
 *
 * Description:
 *   Read decrypted data from SPI Flash at designated address when
 *   enable SPI Flash hardware encryption.
 *
 * Input Parameters:
 *   addr   - target address
 *   buffer - data buffer pointer
 *   size   - data number
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int spi_flash_read_encrypted(uint32_t addr, void *buffer, uint32_t size)
{
  int ret;
  struct spiflash_map_req_s req =
    {
      .src_addr = addr,
      .size = size
    };

  ret = esp32s3_mmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, req.ptr, size);

  esp32s3_ummap(&req);

  return OK;
}

/****************************************************************************
 * Name: esp32s3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_spiflash_init(void)
{
  spi_flash_guard_set(&g_spi_flash_guard_funcs);

  return OK;
}
