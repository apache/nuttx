/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_progmem.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <sys/param.h>

#include "hardware/s32k3xx_pflash.h"
#include "hardware/s32k3xx_xrdc.h"

#include "s32k3xx_config.h"
#include "s32k3xx_progmem.h"
#include "arm_internal.h"

#include <arch/board/board.h> /* Include last:  has dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_S32K3XX_PROGMEM_SIZE > DFLASH_SIZE
# error Progmem size too big
#endif

#if CONFIG_S32K3XX_PROGMEM_SIZE % 8 != 0
# error Progmem must be a multiple of 8
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint32_t get_sector(uint32_t address)
{
  DEBUGASSERT(((address >= S32K3XX_PROGMEM_START_ADDR) &&
      (address <= S32K3XX_PROGMEM_END_ADDR)));

  /* The address is from the data sectors */

  return ((address - (uint32_t)S32K3XX_PROGMEM_START_ADDR) /
              S32K3XX_PROGMEM_SECTOR_SIZE);
}

void data_sector_lock(uint32_t sector, uint32_t lock)
{
  uint32_t regval;
  regval = getreg32(S32K3XX_PFLASH_PFCBLK4_SPELOCK);

  if (lock)
    {
      regval |= (1 << sector);
    }
  else
    {
      regval &= ~(1 << sector);
    }

  putreg32(regval, S32K3XX_PFLASH_PFCBLK4_SPELOCK);
}

uint32_t execute_flash_sequence(uint32_t mask)
{
  uint32_t regval;
  uint32_t status;

  /* Set Mask bit */

  regval = getreg32(S32K3XX_FMU_MCR);
  regval |= mask;
  putreg32(regval, S32K3XX_FMU_MCR);

  /* Set EHV bit to start program operation */

  regval |= FMU_MCR_EHV_MASK;
  putreg32(regval, S32K3XX_FMU_MCR);

  /* Wait for MCRS Done */

  do
    {
      regval = getreg32(S32K3XX_FMU_MCRS);
    }
  while ((regval & FMU_MCRS_DONE_MASK) != FMU_MCRS_DONE_MASK);

  regval = getreg32(S32K3XX_FMU_MCR);
  regval &= ~FMU_MCR_EHV_MASK;
  putreg32(regval, S32K3XX_FMU_MCR);

  status = getreg32(S32K3XX_FMU_MCRS);

  regval &= ~mask;
  putreg32(regval, S32K3XX_FMU_MCR);

  return status;
}

void execute_init_sequence(uint32_t addr)
{
  uint8_t domain_id = getreg8(S32K3XX_XRDC_HWCFG1);

  do
    {
      putreg32(addr, S32K3XX_PFLASH_PFCPGM_PEADR_L);
    }
  while (FMU_MCR_PEID(getreg32(S32K3XX_FMU_MCR)) != domain_id);

  putreg32((FMU_MCRS_PES_SHIFT | FMU_MCRS_PEP_SHIFT), S32K3XX_FMU_MCRS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return number of erase blocks
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return S32K3XX_PROGMEM_SECTOR_COUNT;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Is program memory uniform or page size differs?
 *
 ****************************************************************************/

bool up_progmem_isuniform(void)
{
  return true;
}

/****************************************************************************
 * Name: up_progmem_pagesize
 *
 * Description:
 *   Return read/write page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return (size_t)S32K3XX_PROGMEM_PAGE_SIZE;
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
  return (size_t)S32K3XX_PROGMEM_BLOCK_SECTOR_SIZE;
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to read/write page conversion
 *
 * Input Parameters:
 *   addr - Address with or without flash offset (absolute or aligned to
 *          page0)
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
  if (addr >= S32K3XX_PROGMEM_START_ADDR)
    {
      addr -= S32K3XX_PROGMEM_START_ADDR;
    }

  return (size_t)(addr / S32K3XX_PROGMEM_PAGE_SIZE);
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Read/write page to address conversion
 *
 * Input Parameters:
 *   page - page index
 *
 * Returned Value:
 *   Base address of given page, SIZE_MAX if page index is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  return (size_t)(S32K3XX_PROGMEM_START_ADDR
           + (page * S32K3XX_PROGMEM_PAGE_SIZE));
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase selected block.
 *
 * Input Parameters:
 *   block - The erase block index to be erased.
 *
 * Returned Value:
 *   block size or negative value on error.  The following errors are
 *   reported (errno is not set!):
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
  uint32_t dest;

  dest = (block * S32K3XX_PROGMEM_BLOCK_SECTOR_SIZE +
          S32K3XX_PROGMEM_START_ADDR);

  data_sector_lock(get_sector(dest), 0);

  execute_init_sequence(dest);

  putreg32(0x0, S32K3XX_FMU_PD(0));

  execute_flash_sequence(FMU_MCR_ERS_MASK);

  putreg32(0xffffffff, S32K3XX_FMU_PD(0));

  return (ssize_t)S32K3XX_PROGMEM_BLOCK_SECTOR_SIZE;
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether page is erased
 *
 * Input Parameters:
 *   page - The erase page index to be checked.
 *
 * Returned Value:
 *   Returns number of bytes NOT erased or negative value on error. If it
 *   returns zero then complete page is erased.
 *
 *   The following errors are reported:
 *     -EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  const uint8_t *p;
  int i;

  if (page >= S32K3XX_PROGMEM_PAGE_COUNT)
    {
      return -EFAULT;
    }

  p = (const uint8_t *)up_progmem_getaddress(page);

  for (i = 0; i < S32K3XX_PROGMEM_PAGE_SIZE; i++)
    {
      if (p[i] != S32K3XX_PROGMEM_ERASEDVAL)
        {
          break;
        }
    }

  return (ssize_t)(S32K3XX_PROGMEM_PAGE_SIZE - i);
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
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
 *     EINVAL: If count is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful write, do note when this occurs the complete
 *             flash sector is deemed to be unreadable and a read will most
 *             likely result in a hard fault.
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint32_t i;
  uint32_t dest;
  uint32_t offset;
  uint32_t *p_offset;
  uint32_t *p_src;
  size_t words_to_write;

  if (count % S32K3XX_PROGMEM_DFLASH_WRITE_UNIT_SIZE != 0)
    {
      return -EINVAL;
    }

  /* Address with or without flash offset */

  addr = ((addr % S32K3XX_PROGMEM_START_ADDR) + S32K3XX_PROGMEM_START_ADDR);

  offset = addr % S32K3XX_PROGMEM_WRITE_SIZE;
  dest = addr - offset;
  words_to_write = ((count + offset) / 4);

  if (offset > 0)
    {
      p_offset = (uint32_t *)(dest);
    }

  p_src = (uint32_t *)(buf);

  data_sector_lock(get_sector(dest), 0);

  while (words_to_write > 0)
    {
      /* Destination address */

      execute_init_sequence(dest);

      for (i = 0; i < MIN(32, words_to_write); i++)
        {
          if (offset > 0)
            {
              putreg32(*p_offset++, S32K3XX_FMU_PD(i));
              offset = offset - 4;
            }
          else
            {
              putreg32(*p_src++, S32K3XX_FMU_PD(i));
            }

          dest += 4;
        }

      words_to_write = words_to_write - i;

      execute_flash_sequence(FMU_MCR_PGM_MASK);
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
  return S32K3XX_PROGMEM_ERASEDVAL;
}

void s32k3xx_progmem_init(void)
{
  /* Disable D-Flash ECC for back-to-back flash writes */

  putreg32(PFLASH_PFCR4_DERR_SUP, S32K3XX_PFLASH_PFCR4);
}
