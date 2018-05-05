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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/progmem.h>

#include "chip.h"

#include "up_arch.h"

#include "chip/nrf52_ficr.h"
#include "chip/nrf52_nvmc.h"
#include "nrf52_nvmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define NRF52_FLASH_PAGE_SIZE  (4*1024)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  size_t npage = up_progmem_npages();

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

size_t up_progmem_getaddress(size_t page)
{

  if (page >= up_progmem_npages())
    {
      return SIZE_MAX;
    }

  return page * NRF52_FLASH_PAGE_SIZE;
}

size_t up_progmem_npages(void)
{
  return nrf_nvmc_get_flash_size() / NRF52_FLASH_PAGE_SIZE;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_erasepage(size_t page)
{
  size_t page_address;

  if (page >= up_progmem_npages())
    {
      _err("Wrong Page number %d.\n", page);
      return -EFAULT;
    }

  page_address = up_progmem_getaddress(page);

  /* Get flash ready and begin erasing single page */

  nrf_nvmc_page_erase(page_address);

  /* Verify */

  if (up_progmem_ispageerased(page) == 0)
    {
      return up_progmem_pagesize(page);
    }
  else
    {
      return -EIO;
    }
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= up_progmem_npages())
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
