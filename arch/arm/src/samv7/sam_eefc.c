/****************************************************************************
 * arch/arm/src/samv7/sam_eefc.c
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

#include <string.h>
#include <semaphore.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/samv7/chip.h>

#include "barriers.h"

#include "hardware/sam_memorymap.h"

#include "sam_eefc.h"

#if defined(CONFIG_ARCH_HAVE_RAMFUNCS) && defined(CONFIG_ARCH_RAMFUNCS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_SAMV71) || defined(CONFIG_ARCH_CHIP_SAME70)
/* All sectors are 128KB and are uniform in size.
 * The only exception is sector 0 which is subdivided into two small sectors
 * of 8KB and one larger sector of 112KB.
 * The page size is 512 bytes.  However, the smallest thing that can be
 * erased is four pages.  We will refer to this as a "cluster".
 */

#  define SAMV7_LOCK_REGION_SHIFT (13)   /* 2**13 = 8*KB = 16 pages */
#else
#  error FLASH geometry for this SAMV7 chip not known
#endif

/* Lock region */

#define SAMV7_LOCK_REGION_SIZE   (1 << SAMV7_LOCK_REGION_SHIFT)
#define SAMV7_LOCK_REGION_MASK   (SAMV7_LOCK_REGION_SIZE - 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_eefc_writefmr
 *
 * Description:
 *   Write flash mode register
 *
 * Input Parameters:
 *   regval - The FLASH mode register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

__ramfunc__ void sam_eefc_writefmr(uint32_t regval)
{
  putreg32(regval, SAM_EEFC_FMR);
}

/****************************************************************************
 * Name: sam_eefc_command
 *
 * Description:
 *   Send a FLASH command
 *
 * Input Parameters:
 *   cmd - The FLASH command to be sent
 *   arg - The argument to accompany the command
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

__ramfunc__ int sam_eefc_command(uint32_t cmd, uint32_t arg)
{
  volatile uint32_t regval;
  irqstate_t flags;

  flags = up_irq_save();

  /* Write the command to the flash command register */

  regval = EEFC_FCR_FCMD(cmd) |  EEFC_FCR_FARG(arg) | EEFC_FCR_FKEY_PASSWD;
  putreg32(regval, SAM_EEFC_FCR);

  /* Wait for the FLASH to become ready again */

  do
    {
      regval = getreg32(SAM_EEFC_FSR);
    }
  while ((regval & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);

  up_irq_restore(flags);

  /* Check for errors */

  if ((regval & (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE | EEFC_FSR_FLERR)) != 0)
    {
      return -EIO;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: sam_eefc_readsequence
 *
 * Description:
 *   Flash sequence read
 *
 * Input Parameters:
 *   start_cmd - Start command to perform.
 *   stop_cmd - Stop command to perform.
 *   buffer - Pointer to a data buffer.
 *
 * Returned Value:
 *   ENOMEM: Input buffer error
 *   OK: Sequence read success
 *
 ****************************************************************************/

__ramfunc__ int sam_eefc_readsequence(uint32_t start_cmd, uint32_t stop_cmd,
                                      uint32_t *buffer, size_t bufsize)
{
  volatile uint32_t regval;
  uint32_t *flash_data;
  size_t read_count;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  flash_data = (uint32_t *)SAM_INTFLASH_BASE;

  /* Enabled Sequential Code Optimization of flash controller */

  regval = getreg32(SAM_EEFC_FMR);
  regval |= EEFC_FMR_SCOD;
  sam_eefc_writefmr(regval);

  /* Send the Start Read command */

  regval = EEFC_FCR_FCMD(start_cmd) |  EEFC_FCR_FARG(FCMD_GETD) |
           EEFC_FCR_FKEY_PASSWD;
  putreg32(regval, SAM_EEFC_FCR);

  /* Wait for the FRDY bit in the Flash Programming Status Register
   * (EEFC_FSR) falls.
   */

  do
    {
      regval = getreg32(SAM_EEFC_FSR);
    }
  while ((regval & EEFC_FSR_FRDY) == EEFC_FSR_FRDY);

  /* The data is located in the first address of the Flash
   * memory mapping.
   */

  for (read_count = 0; read_count < bufsize; read_count++)
    {
      buffer[read_count] = flash_data[read_count];
    }

  /* Send the Stop Read command */

  regval = EEFC_FCR_FCMD(stop_cmd) |  EEFC_FCR_FARG(FCMD_GETD) |
           EEFC_FCR_FKEY_PASSWD;
  putreg32(regval, SAM_EEFC_FCR);

  /* Wait for the FRDY bit in the Flash Programming Status Register
   * rises.
   */

  do
    {
      regval = getreg32(SAM_EEFC_FSR);
    }
  while ((regval & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);

  regval = getreg32(SAM_EEFC_FMR);
  regval &= ~EEFC_FMR_SCOD;
  sam_eefc_writefmr(regval);

  return OK;
}

/****************************************************************************
 * Name: sam_eefc_initaccess
 *
 * Description:
 *   Initial flash access mode and wait status
 *
 * Input Parameters:
 *   access_mode - 0 for 128-bit, EEFC_FMR_FAM for 64-bit.
 *   wait_status - The number of wait states in cycle (no shift).
 *
 * Returned Value:
 *   NONE
 ****************************************************************************/

void sam_eefc_initaccess(uint32_t access_mode, uint32_t wait_status)
{
  sam_eefc_writefmr(access_mode | EEFC_FMR_FWS(wait_status));
}

/****************************************************************************
 * Name: sam_eefc_lock
 *
 * Description:
 *   Lock a region of FLASH
 *
 * Input Parameters:
 *   page  - The first page to unlock
 *   npages - The number of consecutive pages to unlock
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if 0 /* Not used */
int sam_eefc_lock(size_t page, size_t npages)
{
  size_t start_page;
  size_t end_page;
  size_t lockpage;
  int ret;

  /* Align the page to the lock region */

  end_page   = page + npages;
  start_page = page & SAMV7_LOCK_REGION_MASK;

  for (lockpage = start_page;
       lockpage < end_page;
       lockpage += SAMV7_LOCK_REGION_SIZE)
    {
      ret = sam_eefccommand(FCMD_SLB, lockpage);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sam_eefc_unlock
 *
 * Description:
 *   Make sure that the FLASH is unlocked
 *
 * Input Parameters:
 *   page  - The first page to unlock
 *   npages - The number of consecutive pages to unlock
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sam_eefc_unlock(size_t page, size_t npages)
{
  size_t start_page;
  size_t end_page;
  size_t lockpage;
  int ret;

  /* Align the page to the lock region */

  end_page   = page + npages;
  start_page = page & SAMV7_LOCK_REGION_MASK;

  for (lockpage = start_page;
       lockpage < end_page;
       lockpage += SAMV7_LOCK_REGION_SIZE)
    {
      ret = sam_eefc_command(FCMD_CLB, lockpage);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

#endif /* defined(CONFIG_ARCH_HAVE_RAMFUNCS) && defined(CONFIG_ARCH_RAMFUNCS) */
