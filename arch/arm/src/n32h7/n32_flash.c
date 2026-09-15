/****************************************************************************
 * arch/arm/src/n32h7/n32_flash.c
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
#include <nuttx/progmem.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "n32_flash.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/* ROM function addresses (Thumb mode entry points) */
#define ROM_WRITE_FUNC   ((uint32_t (*)(uint32_t, uint8_t*, uint32_t))0x1fff7b81)
#define ROM_ERASE_FUNC   ((uint32_t (*)(uint32_t))0x1fff7c81)

/* Return codes from ROM (as defined in SMU, but duplicated here) */
#define FLASH_SUCCESS           0
#define FLASH_BUS_ADDR_ERROR    1
#define FLASH_LOGIC_ADDR_ERROR  2
#define FLASH_RESTRICTED        3
#define FLASH_RDP_PROTECTED     4
#define FLASH_SECURE_AREA       5
#define FLASH_PFOER_AREA        6
#define FLASH_WRP_PROTECTED     7
#define FLASH_FAILED            8

/* Helper to check success */
#define FLASH_OP_OK(code)   ((code) == FLASH_SUCCESS)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_flash_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool n32_flash_valid_addr(uintptr_t addr, size_t len)
{
  return (addr >= (N32_FLASH_BASE | N32_MTD_FLASH_BASE) &&
          addr + len <= (N32_FLASH_BASE | N32_MTD_FLASH_BASE) +
          N32_MTD_FLASH_SIZE);
}

static ssize_t n32_flash_verify_erased(uintptr_t addr, size_t len)
{
  const uint8_t *p = (const uint8_t *)addr;
  size_t count = 0;

  for (size_t i = 0; i < len; i++)
    {
      if (p[i] != 0xff)
        {
          count++;
        }
    }

  return (ssize_t)count;
}

/****************************************************************************
 * Public Functions – progmem API
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return N32_FLASH_PAGE_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return N32_FLASH_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  return true;
}

size_t up_progmem_erasesize(size_t block)
{
  if (block >= N32_FLASH_NBLOCKS)
    {
      return 0;
    }

  return N32_FLASH_SECTOR_SIZE;
}

size_t up_progmem_getaddress(size_t page)
{
  if (page >= N32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  return (N32_FLASH_BASE | N32_MTD_FLASH_BASE) + page * N32_FLASH_PAGE_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr < (N32_FLASH_BASE | N32_MTD_FLASH_BASE) || addr >= (N32_FLASH_BASE
      | N32_MTD_FLASH_BASE) + N32_MTD_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return (addr - (N32_FLASH_BASE | N32_MTD_FLASH_BASE)) /
         N32_FLASH_PAGE_SIZE;
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr = up_progmem_getaddress(page);

  if (addr == SIZE_MAX)
    {
      return -EFAULT;
    }

  return n32_flash_verify_erased(addr, N32_FLASH_PAGE_SIZE);
}

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret;
  uint32_t status;
  uintptr_t addr;

  if (block >= N32_FLASH_NBLOCKS)
    {
      return -EFAULT;
    }

  addr = (N32_FLASH_BASE | N32_MTD_FLASH_BASE) + block *
         N32_FLASH_SECTOR_SIZE;

  ret = nxmutex_lock(&g_flash_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Invalidate cache before erase */

  up_invalidate_dcache(addr, addr + N32_FLASH_SECTOR_SIZE);

  status = ROM_ERASE_FUNC(addr);
  if (!FLASH_OP_OK(status))
    {
      finfo("Erase failed, status=%" PRIu32 "\n", status);
      ret = -EIO;
      goto unlock;
    }

  /* Verify erased */

  if (n32_flash_verify_erased(addr, N32_FLASH_SECTOR_SIZE) != 0)
    {
      ret = -EIO;
      goto unlock;
    }

  ret = (ssize_t)N32_FLASH_SECTOR_SIZE;

unlock:
  nxmutex_unlock(&g_flash_lock);
  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  int ret;
  uint32_t status;

  if (!n32_flash_valid_addr(addr, count))
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_flash_lock);
  if (ret < 0)
    {
      return ret;
    }

  up_invalidate_dcache(addr, addr + count);

  status = ROM_WRITE_FUNC(addr, (uint8_t *)buf, count);
  if (!FLASH_OP_OK(status))
    {
      finfo("Write failed, status=%" PRIu32 "\n", status);
      ret = -EIO;
      goto unlock;
    }

  if (memcmp((const void *)addr, buf, count) != 0)
    {
      ret = -EIO;
      goto unlock;
    }

  ret = (ssize_t)count;

unlock:
  nxmutex_unlock(&g_flash_lock);
  return ret;
}

uint8_t up_progmem_erasestate(void)
{
  return 0xff;
}
