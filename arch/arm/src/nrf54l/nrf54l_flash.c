/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_flash.c
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

#include <errno.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/progmem.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>

#include "arm_internal.h"

#include "hardware/nrf54l_ficr.h"
#include "hardware/nrf54l_rramc.h"
#include "hardware/nrf54l_cache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sizes and masks */

#define NRF54L_FLASH_ERASEDVAL  (0xff)
#define NRF54L_FLASH_PAGESIZE   (4096)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_flash_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_get_flash_size
 *
 * Description:
 *   Return the size of the RRAM array reported by the factory registers.
 *
 ****************************************************************************/

static inline uint32_t nrf54l_get_flash_size(void)
{
  return getreg32(NRF54L_FICR_INFO_RRAM) * 1024;
}

/****************************************************************************
 * Name: nrf54l_get_page_size
 *
 * Description:
 *   RRAM has no page erase. Expose uniform 4 KiB blocks to progmem users.
 *
 ****************************************************************************/

static inline uint32_t nrf54l_get_page_size(void)
{
  return NRF54L_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: nrf54l_get_pages_num
 *
 * Description:
 *   Return the number of emulated erase blocks.
 *
 ****************************************************************************/

static inline uint32_t nrf54l_get_pages_num(void)
{
  return nrf54l_get_flash_size() / NRF54L_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: nrf54l_flash_cache_bypass
 *
 * Description:
 *   Disable the instruction cache while modifying executable memory.
 *
 ****************************************************************************/

static uint32_t nrf54l_flash_cache_bypass(void)
{
  uint32_t enable = getreg32(NRF54L_ICACHE_ENABLE);

  putreg32(0, NRF54L_ICACHE_ENABLE);
  UP_MB();
  return enable;
}

/****************************************************************************
 * Name: nrf54l_flash_cache_restore
 *
 * Description:
 *   Invalidate modified instructions and restore the previous cache state.
 *
 ****************************************************************************/

static void nrf54l_flash_cache_restore(uint32_t enable)
{
  putreg32(1, NRF54L_ICACHE_TASKS_INVALIDATECACHE);
  while (getreg32(NRF54L_ICACHE_STATUS) & CACHE_STATUS_BUSY)
    {
    }

  putreg32(enable, NRF54L_ICACHE_ENABLE);
  UP_MB();
}

/****************************************************************************
 * Name: nrf54l_flash_wait
 *
 * Description:
 *   Wait for pending writes to reach nonvolatile memory.
 *
 ****************************************************************************/

static void nrf54l_flash_wait(void)
{
  while (!(getreg32(NRF54L_RRAMC_READY) & RRAMC_READY_READY))
    {
    }
}

/****************************************************************************
 * Name: nrf54l_flash_program
 *
 * Description:
 *   Write consecutive words, or fill with the erased value if buf is NULL.
 *   The caller holds g_flash_lock and has validated the destination range.
 *
 ****************************************************************************/

static int nrf54l_flash_program(size_t addr, const uint8_t *buf,
                               size_t count)
{
  uint32_t cache;
  uint32_t config;
  uint32_t word;
  size_t i;
  int ret = OK;

  cache = nrf54l_flash_cache_bypass();
  nrf54l_flash_wait();
  config = getreg32(NRF54L_RRAMC_CONFIG);
  putreg32(0, NRF54L_RRAMC_EVENTS_ACCESSERROR);
  putreg32((config & ~RRAMC_CONFIG_WRITEBUFSIZE_MASK) |
           RRAMC_CONFIG_WEN | RRAMC_CONFIG_WRITEBUFSIZE_32,
           NRF54L_RRAMC_CONFIG);
  nrf54l_flash_wait();
  UP_MB();

  for (i = 0; i < count; i += sizeof(word))
    {
      word = UINT32_MAX;
      if (buf != NULL)
        {
          memcpy(&word, buf + i, sizeof(word));
        }

      putreg32(word, addr + i);
    }

  putreg32(1, NRF54L_RRAMC_TASKS_COMMITWRITEBUF);
  UP_MB();
  nrf54l_flash_wait();
  UP_MB();
  putreg32(config & ~RRAMC_CONFIG_WEN, NRF54L_RRAMC_CONFIG);

  if (getreg32(NRF54L_RRAMC_EVENTS_ACCESSERROR) != 0)
    {
      ret = -EACCES;
    }
  else
    {
      for (i = 0; i < count; i += sizeof(word))
        {
          word = UINT32_MAX;
          if (buf != NULL)
            {
              memcpy(&word, buf + i, sizeof(word));
            }

          if (getreg32(addr + i) != word)
            {
              ret = -EIO;
              break;
            }
        }
    }

  nrf54l_flash_cache_restore(cache);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  if (page >= up_progmem_neraseblocks())
    {
      return 0;
    }
  else
    {
      return nrf54l_get_page_size();
    }
}

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return erase block size
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t block)
{
  return up_progmem_pagesize(block);
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to page conversion
 *
 * Input Parameters:
 *   addr - Address to be converted
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= nrf54l_get_flash_size())
    {
      return -EFAULT;
    }

  return addr / nrf54l_get_page_size();
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Page to address conversion
 *
 * Input Parameters:
 *   page - Page to be converted
 *
 * Returned Value:
 *   Base address of given page, maximum size if page is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  if (page >= up_progmem_neraseblocks())
    {
      return SIZE_MAX;
    }

  return page * nrf54l_get_page_size();
}

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return number of erase blocks in the available FLASH memory.
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return nrf54l_get_flash_size() / nrf54l_get_page_size();
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   The emulated erase blocks have a uniform size.
 *
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return true;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase selected block.
 *
 * Input Parameters:
 *   block - Block to be erased
 *
 * Returned Value:
 *   Page size or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid page
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to write protected area
 *     -EACCES: Insufficient permissions (read/write protected)
 *     -EPERM:  If operation is not permitted due to some other constraints
 *              (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t block)
{
  size_t page_address;
  int ret;

  if (block >= up_progmem_neraseblocks())
    {
      return -EFAULT;
    }

  page_address = up_progmem_getaddress(block);

  ret = nxmutex_lock(&g_flash_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* RRAM can overwrite both zeroes and ones; erase is an ordinary write. */

  ret = nrf54l_flash_program(page_address, NULL, NRF54L_FLASH_PAGESIZE);
  nxmutex_unlock(&g_flash_lock);
  return ret < 0 ? ret : NRF54L_FLASH_PAGESIZE;
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether a page is erased
 *
 * Input Parameters:
 *    page - Page to be checked
 *
 * Returned Value:
 *   Returns number of bytes not erased or negative value on error. If it
 *   returns zero then complete page is empty (erased).
 *
 *   The following errors are reported (errno is not set!)
 *     -EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= nrf54l_get_pages_num())
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != NRF54L_FLASH_ERASEDVAL)
        {
          bwritten++;
        }
    }

  return bwritten;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *   buf   - Pointer to buffer
 *   count - Number of bytes to write
 *
 * Returned Value:
 *   Bytes written or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If buflen is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful write
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  int ret;

  /* Use word writes, allowing an unaligned source buffer. */

  if ((addr & 0x3) != 0 || (count & 0x3) != 0 ||
      (buf == NULL && count != 0))
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr > nrf54l_get_flash_size() ||
      count > nrf54l_get_flash_size() - addr)
    {
      return -EFAULT;
    }

  if (count == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&g_flash_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = nrf54l_flash_program(addr, buf, count);
  nxmutex_unlock(&g_flash_lock);

  if (ret < 0)
    {
      return ret;
    }

  return count;
}

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return value of erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return NRF54L_FLASH_ERASEDVAL;
}
