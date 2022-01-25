/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_spiflash.c
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

#include "esp32c3.h"
#include "esp32c3_spiflash.h"
#include "esp32c3_irq.h"
#include "rom/esp32c3_spiflash.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_interrupt.h"
#include "hardware/extmem_reg.h"

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

/* Ibus virtual address */

#define IBUS_VADDR_START            (0x42000000)
#define IBUS_VADDR_END              (0x44000000)

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

/****************************************************************************
 * Private Functions Declaration
 ****************************************************************************/

static void spiflash_start(void);
static void spiflash_end(void);

/****************************************************************************
 * Public Functions Declaration
 ****************************************************************************/

extern int cache_invalidate_addr(uint32_t addr, uint32_t size);
extern uint32_t cache_suspend_icache(void);
extern void cache_resume_icache(uint32_t val);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spiflash_guard_funcs g_spi_flash_guard_funcs =
{
  .start           = spiflash_start,
  .end             = spiflash_end,
  .op_lock         = NULL,
  .op_unlock       = NULL,
  .address_is_safe = NULL,
  .yield           = NULL,
};

static uint32_t g_icache_value;
static uint32_t g_int_regval;
static uint32_t g_int_unmask;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static IRAM_ATTR void disable_mask_int(void)
{
  uint32_t regval;

  g_int_regval = getreg32(INTERRUPT_CPU_INT_ENABLE_REG);
  regval = g_int_regval & g_int_unmask;

  putreg32(regval, INTERRUPT_CPU_INT_ENABLE_REG);
}

static IRAM_ATTR void enable_mask_int(void)
{
  putreg32(g_int_regval, INTERRUPT_CPU_INT_ENABLE_REG);
}

/****************************************************************************
 * Name: spiflash_opstart
 *
 * Description:
 *   Prepare for an SPIFLASH operation.
 *
 ****************************************************************************/

static IRAM_ATTR void spiflash_start(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  disable_mask_int();

  g_icache_value = cache_suspend_icache() << 16;

  leave_critical_section(flags);
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
  irqstate_t flags;

  flags = enter_critical_section();

  cache_resume_icache(g_icache_value >> 16);

  enable_mask_int();

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32c3_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32-C3's address bus, so that software
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

static IRAM_ATTR int esp32c3_mmap(struct spiflash_map_req_s *req)
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
 * Name: esp32c3_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32-C3's address bus, and free resource.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static IRAM_ATTR void esp32c3_ummap(const struct spiflash_map_req_s *req)
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

  ret = esp32c3_mmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buffer, req.ptr, size);

  esp32c3_ummap(&req);

  return OK;
}

/****************************************************************************
 * Name: esp32c3_icache2phys
 *
 * Description:
 *   Get Absolute address in SPI Flash by input function pointer.
 *
 * Input Parameters:
 *   func - Function pointer
 *
 * Returned Value:
 *   Absolute address if success or negtive value if failed.
 *
 ****************************************************************************/

int32_t esp32c3_icache2phys(const void *func)
{
  intptr_t pages;
  intptr_t off;
  intptr_t c = (intptr_t)func;

  off   = (c - IBUS_VADDR_START) % SPI_FLASH_MMU_PAGE_SIZE;
  pages = (c - IBUS_VADDR_START) / SPI_FLASH_MMU_PAGE_SIZE;

  pages += MMU_TABLE[pages];

  return pages * SPI_FLASH_MMU_PAGE_SIZE + off;
}

/****************************************************************************
 * Name: esp32c3_spiflash_unmask_cpuint
 *
 * Description:
 *   Unmask CPU interrupt and keep this interrupt work when read, write,
 *   erase SPI Flash.
 *
 *   By default, all CPU interrupts are masked.
 *
 * Input Parameters:
 *   cpuint - CPU interrupt ID
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_spiflash_unmask_cpuint(int cpuint)
{
  g_int_unmask |= 1 << cpuint;
}

/****************************************************************************
 * Name: esp32c3_spiflash_unmask_cpuint
 *
 * Description:
 *   Mask CPU interrupt and disable this interrupt when read, write,
 *   erase SPI Flash.
 *
 *   By default, all CPU interrupts are masked.
 *
 * Input Parameters:
 *   cpuint - CPU interrupt ID
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_spiflash_mask_cpuint(int cpuint)
{
  g_int_unmask &= ~(1 << cpuint);
}

/****************************************************************************
 * Name: esp32c3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-C3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32c3_spiflash_init(void)
{
  spi_flash_guard_set(&g_spi_flash_guard_funcs);

  return OK;
}
