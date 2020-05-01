/****************************************************************************
 * arch/arm/src/nrf52/nrf52_flash.c
 * Standard Flash access functions needed by the flash mtd driver.
 *
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Ported from the Nordic SDK, this is the original license:
 *
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include "chip.h"

#include "arm_arch.h"

#include "hardware/nrf52_ficr.h"
#include "hardware/nrf52_nvmc.h"
#include "nrf52_nvmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_FLASH_PAGE_SIZE  (4*1024)

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
  size_t npage = up_progmem_neraseblocks();

  if (page >= npage)
    {
      _err("Error For Wrong Page Index[%d], Total Page %d.\n", page, npage);
      return 0;
    }
  else
    {
      return NRF52_FLASH_PAGE_SIZE;
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
  size_t page_end = 0;

  if (addr >= nrf_nvmc_get_flash_size())
    {
      _err("Address is out of Total Size.\n");
      return -EFAULT;
    }

  page_end = addr / NRF52_FLASH_PAGE_SIZE;
  return page_end;
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

  return page * NRF52_FLASH_PAGE_SIZE;
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
  return nrf_nvmc_get_flash_size() / NRF52_FLASH_PAGE_SIZE;
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
      _err("Wrong Page number %d.\n", page);
      return -EFAULT;
    }

  page_address = up_progmem_getaddress(block);

  /* Get flash ready and begin erasing single page */

  nrf_nvmc_page_erase(page_address);

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

  if (page >= up_progmem_neraseblocks())
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != 0xff)
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

  /* NRF52 requires word access */

  if (count & 0x3)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if ((addr + count) > nrf_nvmc_get_flash_size())
    {
      return -EFAULT;
    }

  /* Get flash ready and begin flashing */

  for (addr += NRF52_FLASH_BASE; count; count -= 4, pword++, addr += 4)
    {
      /* Write word and wait to complete */

      nrf_nvmc_write_word(addr, *pword);

      /* Verify */

      if (getreg32(addr) != *pword)
        {
          _err("Write Internal Flash Error.\n");
          return -EIO;
        }
    }

  return written;
}
