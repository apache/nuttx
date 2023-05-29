/****************************************************************************
 * arch/arm/src/nrf53/nrf53_flash.c
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

#include <nuttx/arch.h>
#include <nuttx/progmem.h>

#include "arm_internal.h"
#include "barriers.h"

#include "hardware/nrf53_ficr.h"
#include "hardware/nrf53_nvmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MEMORY_SYNC() ARM_ISB(); ARM_DSB()

/* Sizes and masks */

#define NRF53_FLASH_ERASEDVAL  (0xff)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t nrf53_get_flash_size(void)
{
  return getreg32(NRF53_FICR_INFO_FLASH) * 1024;
}

static inline uint32_t nrf53_get_page_size(void)
{
  return getreg32(NRF53_FICR_INFO_CODEPAGESIZE);
}

static inline uint32_t nrf53_get_pages_num(void)
{
  return getreg32(NRF53_FICR_INFO_CODESIZE);
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
      return nrf53_get_page_size();
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
  if (addr >= nrf53_get_flash_size())
    {
      return -EFAULT;
    }

  return addr / nrf53_get_page_size();
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

  return page * nrf53_get_page_size();
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
  return nrf53_get_flash_size() / nrf53_get_page_size();
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Page size is uniform?  Say 'yes' even though that is not strictly
 *   true.
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

  if (block >= up_progmem_neraseblocks())
    {
      return -EFAULT;
    }

  page_address = up_progmem_getaddress(block);

  /* Enable erase mode */

  putreg32(NVMC_CONFIG_EEN, NRF53_NVMC_CONFIG);

  /* Memory sync */

  MEMORY_SYNC();

  /* Erase the page by writting 0xffffffff into the first 32-bit word of
   * the flash page
   */

  putreg32(0xffffffff, page_address);

  /* Wait for flash */

  while (!(getreg32(NRF53_NVMC_READY) & NVMC_READY_READY))
    {
    }

  /* Read only access */

  putreg32(NVMC_CONFIG_REN, NRF53_NVMC_CONFIG);

  /* Memory sync */

  MEMORY_SYNC();

  /* Verify */

  if (up_progmem_ispageerased(block) == 0)
    {
      return up_progmem_erasesize(block);
    }
  else
    {
      return -EIO;
    }
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
 *   Returns number of bytes erased or negative value on error. If it
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

  if (page >= nrf53_get_pages_num())
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != NRF53_FLASH_ERASEDVAL)
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
  uint32_t *pword = (uint32_t *)buf;
  size_t written = count;

  /* NRF53 requires word access */

  if (count & 0x3)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if ((addr + count) > nrf53_get_flash_size())
    {
      return -EFAULT;
    }

  /* Flash offset */

  addr += NRF53_FLASH_BASE;

  /* Begin flashing */

  for (; count; count -= 4, pword++, addr += 4)
    {
      /* Enable write */

      putreg32(NVMC_CONFIG_WEN, NRF53_NVMC_CONFIG);

      /* Memory sync */

      MEMORY_SYNC();

      /* Write the word */

      *(uint32_t *)addr = *pword;

      /* Wait for flash */

      while (!(getreg32(NRF53_NVMC_READY) & NVMC_READY_READY))
        {
        }

      /* Read only access */

      putreg32(NVMC_CONFIG_REN, NRF53_NVMC_CONFIG);

      /* Memory sync */

      MEMORY_SYNC();

      /* Verify */

      if (getreg32(addr) != *pword)
        {
          return -EIO;
        }
    }

  return written;
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
  return NRF53_FLASH_ERASEDVAL;
}
