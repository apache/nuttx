/****************************************************************************
 * arch/arm/src/at32/at32f43xx_flash.c
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

/* Provides standard flash access functions, to be used by the  flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "at32_flash.h"
#include "at32_rcc.h"
#include "at32_waste.h"
#include "arm_internal.h"

/* Only for the AT32F43xx family. */

#if defined (CONFIG_AT32_AT32F43XX)

#if defined(CONFIG_AT32_FLASH_CONFIG_DEFAULT)
#  warning "Default Flash Configuration Used - See Override Flash Size Designator"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xcdef89ab
#define FLASH_OPTKEY1      0x08192a3b
#define FLASH_OPTKEY2      0x4c5d6e7f
#define FLASH_ERASEDVALUE  0xff

/****************************************************************************
 * Private Data
 ****************************************************************************/

  static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void flash_unlock(void)
{
  while (getreg32(AT32_FLASH_STS) & FLASH_STS_OBF)
    {
      at32_waste();
    }

  if (getreg32(AT32_FLASH_CTRL) & FLASH_CTRL_OPLK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, AT32_FLASH_UNLOCK);
      putreg32(FLASH_KEY2, AT32_FLASH_UNLOCK);
    }

#ifdef AT32_FLASH_BANK2_START
  if (AT32_FLASH_BANK2_START < AT32_FLASH_NPAGES)
    {
      while (getreg32(AT32_FLASH_STS2) & FLASH_STS_OBF)
        {
          at32_waste();
        }

      if (getreg32(AT32_FLASH_CTRL2) & FLASH_CTRL_OPLK)
        {
          /* Unlock sequence */

          putreg32(FLASH_KEY1, AT32_FLASH_UNLOCK2);
          putreg32(FLASH_KEY2, AT32_FLASH_UNLOCK2);
        }
    }
#endif
}

static void flash_lock(void)
{
  modifyreg32(AT32_FLASH_CTRL, 0, FLASH_CTRL_OPLK);

#ifdef AT32_FLASH_BANK2_START
  if (AT32_FLASH_BANK2_START < AT32_FLASH_NPAGES)
    modifyreg32(AT32_FLASH_CTRL2, 0, FLASH_CTRL_OPLK);
#endif
}

#if defined(CONFIG_AT32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
static void data_cache_disable(void)
{
  modifyreg32(AT32_FLASH_PSR, FLASH_PSR_NZW_BST, 0);
}

static void data_cache_enable(void)
{
  /* Enable data cache */

  modifyreg32(AT32_FLASH_PSR, 0, FLASH_PSR_NZW_BST);
}
#endif /* defined(CONFIG_AT32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW) */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int at32_flash_unlock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_unlock();
  nxmutex_unlock(&g_lock);

  return ret;
}

int at32_flash_lock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_lock();
  nxmutex_unlock(&g_lock);

  return ret;
}

size_t up_progmem_pagesize(size_t page)
{
  return AT32_FLASH_PAGESIZE;
}

size_t up_progmem_erasesize(size_t block)
{
  return up_progmem_pagesize(block);
}

ssize_t up_progmem_getpage(size_t addr)
{
  size_t page_end = 0;
  size_t i;

  if (addr >= AT32_FLASH_BASE)
    {
      addr -= AT32_FLASH_BASE;
    }

  if (addr >= AT32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  for (i = 0; i < AT32_FLASH_NPAGES; ++i)
    {
      page_end += up_progmem_pagesize(i);
      if (page_end > addr)
        {
          return i;
        }
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = AT32_FLASH_BASE;
  size_t i;

  if (page >= AT32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      base_address += up_progmem_pagesize(i);
    }

  return base_address;
}

size_t up_progmem_neraseblocks(void)
{
  return AT32_FLASH_NPAGES;
}

bool up_progmem_isuniform(void)
{
#ifdef AT32_FLASH_PAGESIZE
  return true;
#else
  return false;
#endif
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= AT32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
    count; count--, addr++)
    {
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  unsigned int flash_ctrl;
  unsigned int flash_addr;
  unsigned int flash_sts;

  if (block >= AT32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

#ifdef AT32_FLASH_BANK2_START
  flash_ctrl = (block < AT32_FLASH_BANK2_START) ? \
              AT32_FLASH_CTRL : AT32_FLASH_CTRL2;
  flash_addr = (block < AT32_FLASH_BANK2_START) ? \
              AT32_FLASH_ADDR : AT32_FLASH_ADDR2;
  flash_sts = (block < AT32_FLASH_BANK2_START) ? \
              AT32_FLASH_STS : AT32_FLASH_STS2;
#else
  flash_ctrl = AT32_FLASH_CTRL;
  flash_addr = AT32_FLASH_ADDR;
  flash_sts = AT32_FLASH_STS;
#endif

  nxmutex_lock(&g_lock);

  /* Get flash ready and begin erasing single block */

  flash_unlock();

  modifyreg32(flash_ctrl, 0, FLASH_CTRL_SECERS);
  modifyreg32(flash_addr, 0, block * AT32_FLASH_PAGESIZE + AT32_FLASH_BASE);
  modifyreg32(flash_ctrl, 0, FLASH_CTRL_ERSTR);

  while (getreg32(flash_sts) & FLASH_STS_OBF)
    {
      at32_waste();
    }

  modifyreg32(flash_ctrl, FLASH_CTRL_SECERS, 0);

  nxmutex_unlock(&g_lock);

  /* Verify */

  if (up_progmem_ispageerased(block) == 0)
    {
      return up_progmem_pagesize(block); /* success */
    }
  else
    {
      return -EIO; /* failure */
    }
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint16_t *hword = (uint16_t *)buf;
  size_t written = count;
  unsigned int flash_sts;

  /* AT32 requires half-word access */

  if (count & 1)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= AT32_FLASH_BASE)
    {
      addr -= AT32_FLASH_BASE;
    }

  if ((addr + count) > AT32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  nxmutex_lock(&g_lock);

  /* Get flash ready and begin flashing */

  flash_unlock();

#if defined(CONFIG_AT32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_disable();
#endif

  modifyreg32(AT32_FLASH_CTRL, 0, FLASH_CTRL_FPRGM);
  modifyreg32(AT32_FLASH_CTRL2, 0, FLASH_CTRL_FPRGM);

  for (addr += AT32_FLASH_BASE; count; count -= 2, hword++, addr += 2)
    {
      /* Write half-word and wait to complete */

      putreg16(*hword, addr);

  #ifdef AT32_FLASH_BANK2_START
      flash_sts = (addr < AT32_FLASH_BANK2_START *AT32_FLASH_PAGESIZE) ? \
                  AT32_FLASH_STS : AT32_FLASH_STS2;
  #else
      flash_sts = AT32_FLASH_STS;
  #endif

      while (getreg32(flash_sts) & FLASH_STS_OBF)
        {
          at32_waste();
        }

      /* Verify */

      if (getreg32(flash_sts) & FLASH_STS_PRGMERR)
        {
          modifyreg32(AT32_FLASH_CTRL, FLASH_CTRL_FPRGM, 0);
          modifyreg32(AT32_FLASH_CTRL2, FLASH_CTRL_FPRGM, 0);
          nxmutex_unlock(&g_lock);

          return -EROFS;
        }

      if (getreg16(addr) != *hword)
        {
          modifyreg32(AT32_FLASH_CTRL, FLASH_CTRL_FPRGM, 0);
          modifyreg32(AT32_FLASH_CTRL2, FLASH_CTRL_FPRGM, 0);
          nxmutex_unlock(&g_lock);

          return -EIO;
        }
    }

  modifyreg32(AT32_FLASH_CTRL, FLASH_CTRL_FPRGM, 0);
  modifyreg32(AT32_FLASH_CTRL2, FLASH_CTRL_FPRGM, 0);

#if defined(CONFIG_AT32_FLASH_WORKAROUND_DATA_CACHE_CORRUPTION_ON_RWW)
  data_cache_enable();
#endif

  nxmutex_unlock(&g_lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}

#endif /* defined(CONFIG_AT32_AT32F43XX) */
