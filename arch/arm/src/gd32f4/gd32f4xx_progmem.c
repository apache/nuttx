/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_progmem.c
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

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include <arch/chip/chip.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/progmem.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "gd32f4xx_progmem.h"
#include "gd32f4xx_fmc.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Progmem helpers used around FMC program/erase must not fetch from the
 * same flash bank while FMC is busy. Match gd32f4xx_fmc.c RAMFUNC style.
 */

#ifdef CONFIG_ARCH_RAMFUNCS
#  ifdef __clang__
#    define PROGMEM_RAMFUNC __ramfunc__
#  else
#    define PROGMEM_RAMFUNC locate_code(".ramfunc") farcall_function noinline_function
#  endif
#else
#  define PROGMEM_RAMFUNC
#endif

#if defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT)

#  define FMC_PROGMEM_SECTOR_SIZES       {_K(128), _K(128)}
#  define FMC_PROGMEM_SECTOR_NUM         (2)
#  define FMC_PROGMEM_PAGESIZE         4096
#  define FMC_PROGMEM_SECTOR_SADDR       (0x08040000)
#  define FMC_PROGMEM_SECTOR_EADDR       (0x0807FFFF)

#elif !defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT)

#  if defined(CONFIG_GD32F4_FLASH_CONFIG_E)

#    define FMC_PROGMEM_SECTOR_SIZES     {_K(128), _K(128)}
#    define FMC_PROGMEM_SECTOR_NUM       (2)
#    define FMC_PROGMEM_SECTOR_SADDR     (0x08040000)
#    define FMC_PROGMEM_SECTOR_EADDR     (0x0807FFFF)
#    define FMC_PROGMEM_PAGESIZE         4096

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_G)

#    define FMC_PROGMEM_SECTOR_SIZES     {_K(128), _K(128)}
#    define FMC_PROGMEM_SECTOR_NUM       (2)
#    define FMC_PROGMEM_SECTOR_SADDR     (0x080C0000)
#    define FMC_PROGMEM_SECTOR_EADDR     (0x080FFFFF)

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_I)
#    if defined(CONFIG_GD32F4_GD32F470)
/* GD32F470xI: 2048 KiB @ 0x08000000. Expose uniform 4 KiB blocks to MTD
 * (partition math needs uniform pagesize). Hardware erase uses sector erase
 * inside up_progmem_eraseblock() — page-erase BUSY can stick forever when the
 * loader overwrites primary in the same flash bank.
 */

#      define FMC_PROGMEM_SECTOR_SIZES   {_K(256), _K(256), _K(256), _K(256)}
#      define FMC_PROGMEM_SECTOR_NUM     (512)
#      define FMC_PROGMEM_SECTOR_SADDR   (0x08000000)
#      define FMC_PROGMEM_SECTOR_EADDR   (0x081FFFFF)
#      define FMC_PROGMEM_PAGESIZE       4096
#      define FMC_PROGMEM_USE_HW_SECTOR_ERASE 1
#    else
#      define FMC_PROGMEM_SECTOR_SIZES   {_K(16), _K(16), _K(16), _K(16)}
#      define FMC_PROGMEM_SECTOR_NUM     (4)
#      define FMC_PROGMEM_SECTOR_SADDR   (0x08100000)
#      define FMC_PROGMEM_SECTOR_EADDR   (0x0810FFFF)
#    endif

#  elif defined(CONFIG_GD32F4_FLASH_CONFIG_K)
#if defined(CONFIG_GD32F4_GD32F450)
#    define FMC_PROGMEM_SECTOR_SIZES     {_K(16), _K(16), _K(16), _K(16)}
#    define FMC_PROGMEM_SECTOR_NUM       (4)
#    define FMC_PROGMEM_SECTOR_SADDR     (0x08100000)
#    define FMC_PROGMEM_SECTOR_EADDR     (0x0810FFFF)
#elif defined(CONFIG_GD32F4_GD32F470)
#    define FMC_PROGMEM_SECTOR_SIZES     {_K(256), _K(256), _K(256), _K(256)}
#    define FMC_PROGMEM_SECTOR_NUM       (256)
#    define FMC_PROGMEM_SECTOR_SADDR     (0x08200000)
#    define FMC_PROGMEM_SECTOR_EADDR     (0x082FFFFF)
#    define FMC_PROGMEM_PAGESIZE         4096
#endif

#  endif

#endif /* !defined(CONFIG_GD32F4_FLASH_CONFIG_DEFAULT) */

/* Sector size */

#define SIZE_16KB                  (0x00004000)                    /* size of 16KB*/
#define SIZE_64KB                  (0x00010000)                    /* size of 64KB*/
#define SIZE_128KB                 (0x00020000)                    /* size of 128KB*/
#define SIZE_256KB                 (0x00040000)                    /* size of 256KB*/

/* FMC BANK address */

#define FMC_START_ADDRESS          (0x08000000)                    /* FMC start address */
#define FMC_BANK0_START_ADDRESS    FMC_START_ADDRESS               /* FMC BANK0 start address */
#define FMC_BANK1_START_ADDRESS    (0x08100000)                    /* FMC BANK1 start address */
#define FMC_SIZE                   (*(uint16_t *)0x1FFF7A22)       /* FMC SIZE */
#define FMC_END_ADDRESS            (FMC_START_ADDRESS + (FMC_SIZE * 1024) - 1)
                                                                   /* FMC end address */
#define FMC_MAX_END_ADDRESS        (0x08300000)                    /* FMC maximum end address */

/* FMC error message */

#define FMC_WRONG_SECTOR_NAME      (0xFFFFFFFF)                    /* wrong sector name*/
#define FMC_WRONG_SECTOR_NUM       (0xFFFFFFFF)                    /* wrong sector number*/
#define FMC_INVALID_SIZE           (0xFFFFFFFF)                    /* invalid sector size*/
#define FMC_INVALID_ADDR           (0xFFFFFFFF)                    /* invalid sector address*/

#define FMC_ERASE_STATE_VAL        (0xFF)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* FMC sector information */

typedef struct
{
    uint32_t sector_name;                /* the name of the sector */
    uint32_t sector_num;                 /* the number of the sector */
    uint32_t sector_size;                /* the size of the sector */
    uint32_t sector_start_addr;          /* the start address of the sector */
    uint32_t sector_end_addr;            /* the end address of the sector */
} fmc_sector_info_struct;

static const size_t sector_sizes[FMC_PROGMEM_SECTOR_NUM] =
                                               FMC_PROGMEM_SECTOR_SIZES;

static mutex_t g_gd32_progmem_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_fmc_sector_info_get
 *
 * Description:
 *   Get the sector number, size and range of the given address
 *
 * Parameters:
 *   address - The flash address
 *
 * Return Value:
 *    fmc_sector_info_struct: The information of a sector
 *
 ****************************************************************************/

static fmc_sector_info_struct gd32_fmc_sector_info_get(uint32_t addr)
{
  fmc_sector_info_struct sector_info;
  uint32_t temp = 0x00000000;

  if ((FMC_START_ADDRESS <= addr) && (FMC_END_ADDRESS >= addr))
    {
      if ((FMC_BANK1_START_ADDRESS > addr))
        {
          /* Bank0 area */

          temp = (addr - FMC_BANK0_START_ADDRESS) / SIZE_16KB;
          if (4u > temp)
            {
              sector_info.sector_name = (uint32_t)temp;
              sector_info.sector_num = FMC_CTL_SN(temp);
              sector_info.sector_size = SIZE_16KB;
              sector_info.sector_start_addr = FMC_BANK0_START_ADDRESS +
                                              (SIZE_16KB * temp);
              sector_info.sector_end_addr = sector_info.sector_start_addr +
                                            SIZE_16KB - 1;
            }
          else if (8u > temp)
            {
              sector_info.sector_name = 0x00000004u;
              sector_info.sector_num = FMC_CTL_SN(4);
              sector_info.sector_size = SIZE_64KB;
              sector_info.sector_start_addr = 0x08010000u;
              sector_info.sector_end_addr = 0x0801ffffu;
            }
          else
            {
              temp = (addr - FMC_BANK0_START_ADDRESS) / SIZE_128KB;
              sector_info.sector_name = (uint32_t)(temp + 4);
              sector_info.sector_num = FMC_CTL_SN(temp + 4);
              sector_info.sector_size = SIZE_128KB;
              sector_info.sector_start_addr = FMC_BANK0_START_ADDRESS +
                                              (SIZE_128KB * temp);
              sector_info.sector_end_addr = sector_info.sector_start_addr +
                                            SIZE_128KB - 1;
            }
        }
      else
        {
          /* bank1 area */

          temp = (addr - FMC_BANK1_START_ADDRESS) / SIZE_16KB;
          if (4u > temp)
            {
              sector_info.sector_name = (uint32_t)(temp + 12);
              sector_info.sector_num = FMC_CTL_SN(temp + 16);
              sector_info.sector_size = SIZE_16KB;
              sector_info.sector_start_addr = FMC_BANK0_START_ADDRESS +
                                              (SIZE_16KB * temp);
              sector_info.sector_end_addr = sector_info.sector_start_addr +
                                            SIZE_16KB - 1;
            }
          else if (8u > temp)
            {
              sector_info.sector_name = 0x00000010u;
              sector_info.sector_num = FMC_CTL_SN(20);
              sector_info.sector_size = SIZE_64KB;
              sector_info.sector_start_addr = 0x08110000u;
              sector_info.sector_end_addr = 0x0811ffffu;
            }
          else if (64u > temp)
            {
              temp = (addr - FMC_BANK1_START_ADDRESS) / SIZE_128KB;
              sector_info.sector_name = (uint32_t)(temp + 16);
              sector_info.sector_num = FMC_CTL_SN(temp + 20);
              sector_info.sector_size = SIZE_128KB;
              sector_info.sector_start_addr = FMC_BANK1_START_ADDRESS +
                                              (SIZE_128KB * temp);
              sector_info.sector_end_addr = sector_info.sector_start_addr +
                                            SIZE_128KB - 1;
            }
          else
            {
              temp = (addr - FMC_BANK1_START_ADDRESS) / SIZE_256KB;
              sector_info.sector_name = (uint32_t)(temp + 20);
              sector_info.sector_num = FMC_CTL_SN(temp + 8);
              sector_info.sector_size = SIZE_256KB;
              sector_info.sector_start_addr = FMC_BANK1_START_ADDRESS +
                                              (SIZE_256KB * temp);
              sector_info.sector_end_addr = sector_info.sector_start_addr +
                                            SIZE_256KB - 1;
            }
        }
    }
  else
    {
      /* invalid address */

      sector_info.sector_name = FMC_WRONG_SECTOR_NAME;
      sector_info.sector_num = FMC_WRONG_SECTOR_NUM;
      sector_info.sector_size = FMC_INVALID_SIZE;
      sector_info.sector_start_addr = FMC_INVALID_ADDR;
      sector_info.sector_end_addr = FMC_INVALID_ADDR;
    }
  return sector_info;
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
  return FMC_PROGMEM_SECTOR_NUM;
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
  return false;
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
  if (block >= FMC_PROGMEM_SECTOR_NUM)
    {
      return 0;
    }

#ifdef FMC_PROGMEM_PAGESIZE
  UNUSED(sector_sizes);
  return FMC_PROGMEM_PAGESIZE;
#else
  return sector_sizes[block];
#endif
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
  return up_progmem_erasesize(page);
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to read/write page conversion
 *
 * Input Parameters:
 *   addr - Address with or without flash offset
 *          (absolute or aligned to page0)
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
  size_t page = 0;
  uint32_t i;

  if ((addr < FMC_PROGMEM_SECTOR_SADDR) || (addr > FMC_PROGMEM_SECTOR_EADDR))
    {
      return -EFAULT;
    }

  addr -= FMC_PROGMEM_SECTOR_SADDR;

  for (i = 0; i < FMC_PROGMEM_SECTOR_NUM; ++i)
    {
      page += up_progmem_pagesize(i);
      if (page > addr)
        {
          return i;
        }
    }

  return -EFAULT;
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
  size_t addr = FMC_PROGMEM_SECTOR_SADDR;
  size_t i;

  if (page >= FMC_PROGMEM_SECTOR_NUM)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      addr += up_progmem_pagesize(i);
    }

  return addr;
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

/* Spot-check first / mid / last word of a 4 KiB page (RAM-safe). */

static PROGMEM_RAMFUNC bool progmem_page_erased_spot(size_t addr)
{
  if (*(volatile uint32_t *)(addr) != 0xffffffffu)
    {
      return false;
    }

  if (*(volatile uint32_t *)(addr + 2048u) != 0xffffffffu)
    {
      return false;
    }

  if (*(volatile uint32_t *)(addr + 4092u) != 0xffffffffu)
    {
      return false;
    }

  return true;
}

ssize_t PROGMEM_RAMFUNC up_progmem_eraseblock(size_t block)
{
  size_t addr;
#ifndef FMC_PROGMEM_USE_HW_SECTOR_ERASE
  fmc_sector_info_struct sector_info;
#endif

  if (block >= FMC_PROGMEM_SECTOR_NUM)
    {
      return -EFAULT;
    }

  /* Inline address math — do not call Flash-resident up_progmem_getaddress
   * or gd32_fmc_sector_info_get from this .ramfunc path.
   */

#ifdef FMC_PROGMEM_PAGESIZE
  addr = FMC_PROGMEM_SECTOR_SADDR + block * FMC_PROGMEM_PAGESIZE;
#else
  addr = up_progmem_getaddress(block);
#endif

#if defined(CONFIG_GD32F4_GD32F470) || defined(FMC_PROGMEM_USE_HW_SECTOR_ERASE)
  /* GD32F470: 4 KiB page erase. Sector erase from on-chip loader has been
   * observed to leave FMC BUSY stuck. Erasing an already-blank page can also
   * stick — skip FMC when the logical page is already 0xFF (word scan).
   */

  /* FMC must be idle before touching flash. If a prior operation left BUSY
   * set, any flash read stalls the AHB bus forever (observed hang in
   * up_progmem_ispageerased). Fail fast instead of deadlocking so the caller
   * can abort the upgrade cleanly.
   */

  if ((getreg32(GD32_FMC_STAT) & FMC_STAT_BUSY) != 0)
    {
      return -EBUSY;
    }

  if (up_progmem_ispageerased(block) == 0)
    {
      return (ssize_t)FMC_PROGMEM_PAGESIZE;
    }

  gd32_fmc_unlock_ram();
  gd32_fmc_flag_clear_ram(FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR |
                          FMC_STAT_PGMERR | FMC_STAT_PGSERR | FMC_STAT_RDDERR);

  if (gd32_fmc_page_erase((uint32_t)addr) != FMC_READY)
    {
      gd32_fmc_lock_ram();
      return -EIO;
    }

  /* FMC could have been left busy by a failed erase; never read flash then. */

  if ((getreg32(GD32_FMC_STAT) & FMC_STAT_BUSY) != 0)
    {
      gd32_fmc_lock_ram();
      return -EBUSY;
    }

  /* Spot-check only (full word rescan is unnecessary for OVERWRITE). */

  if (!progmem_page_erased_spot(addr))
    {
      gd32_fmc_lock_ram();
      return -EIO;
    }

  gd32_fmc_lock_ram();
  return (ssize_t)FMC_PROGMEM_PAGESIZE;
#else
  sector_info = gd32_fmc_sector_info_get(addr);
  if (sector_info.sector_name == FMC_WRONG_SECTOR_NAME)
    {
      return -EFAULT;
    }

  if (up_progmem_ispageerased(block) == 0)
    {
      return (ssize_t)up_progmem_pagesize(block);
    }

  gd32_fmc_unlock_ram();
  gd32_fmc_flag_clear_ram(FMC_STAT_END | FMC_STAT_OPERR | FMC_STAT_WPERR |
                          FMC_STAT_PGMERR | FMC_STAT_PGSERR | FMC_STAT_RDDERR);

  if (gd32_fmc_sector_erase(sector_info.sector_num))
    {
      gd32_fmc_lock_ram();
      return -EFAULT;
    }

  if (up_progmem_ispageerased(block) == 0)
    {
      gd32_fmc_lock_ram();
      return up_progmem_pagesize(block);
    }

  gd32_fmc_lock_ram();
  return -EIO;
#endif
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

ssize_t PROGMEM_RAMFUNC up_progmem_ispageerased(size_t page)
{
  size_t page_size;
  size_t addr;
  size_t i;

  if (page >= FMC_PROGMEM_SECTOR_NUM)
    {
      return -EFAULT;
    }

#ifdef FMC_PROGMEM_PAGESIZE
  page_size = FMC_PROGMEM_PAGESIZE;
  addr = FMC_PROGMEM_SECTOR_SADDR + page * FMC_PROGMEM_PAGESIZE;
#else
  page_size = up_progmem_pagesize(page);
  addr = up_progmem_getaddress(page);
#endif

  for (i = 0; i < page_size; i += 4)
    {
      if (*(volatile uint32_t *)(addr + i) != 0xffffffffu)
        {
          return (ssize_t)(page_size - i);
        }
    }

  return 0;
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
 *           (absolute or aligned to page0)
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
 *     EIO:    On unsuccessful write
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

/****************************************************************************
 * Name: progmem_write_ram
 *
 * Description:
 *   Program loop that must execute from RAM on GD32F470 (same flash bank as
 *   the loader/app). Flash-resident loops hang if they fetch while FMC BUSY.
 *
 ****************************************************************************/

static PROGMEM_RAMFUNC int progmem_write_ram(size_t addr, FAR const uint8_t *byte,
                                              size_t remaining)
{
  while (remaining > 0)
    {
      if ((addr & 3) == 0 && remaining >= 4)
        {
          uint32_t word =
            ((uint32_t)byte[0]) |
            ((uint32_t)byte[1] << 8) |
            ((uint32_t)byte[2] << 16) |
            ((uint32_t)byte[3] << 24);

          if (word != 0xffffffffu)
            {
              if (gd32_fmc_word_program(addr, word) != FMC_READY)
                {
                  return -EIO;
                }
            }

          addr += 4;
          byte += 4;
          remaining -= 4;
          continue;
        }

      if (*byte != 0xff)
        {
          if (gd32_fmc_byte_program(addr, *byte) != FMC_READY)
            {
              return -EIO;
            }
        }

      addr++;
      byte++;
      remaining--;
    }

  return OK;
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
 *           (absolute or aligned to page0)
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

  /* Check for valid address range */

  if (addr < FMC_PROGMEM_SECTOR_SADDR)
    {
      return -EFAULT;
    }

  if ((addr + count) > (FMC_PROGMEM_SECTOR_EADDR + 1))
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_gd32_progmem_lock);
  if (ret < 0)
    {
      return -EFAULT;
    }

  /* Unlock from SRAM before entering the FMC-busy program loop. */

  gd32_fmc_unlock_ram();

  ret = progmem_write_ram(addr, (FAR const uint8_t *)buf, count);

  /* Fail closed: programmed bytes must be visible on the flash bus. */

  if (ret >= 0)
    {
      FAR const uint8_t *src = (FAR const uint8_t *)buf;
      size_t i;
      size_t vlen = count < 16 ? count : 16;

      for (i = 0; i < vlen; i++)
        {
          if (*(volatile uint8_t *)(addr + i) != src[i])
            {
              ret = -EIO;
              break;
            }
        }
    }

  gd32_fmc_lock_ram();
  nxmutex_unlock(&g_gd32_progmem_lock);

  return (ret < 0) ? (ssize_t)ret : (ssize_t)count;
}

/****************************************************************************
 * Name: up_progmem_read
 *
 * Description:
 *   Read data at given address
 *
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset
 *           (absolute or aligned to page0)
 *   buf   - Pointer to buffer
 *   count - Number of bytes to read
 *
 * Returned Value:
 *   Bytes read or negative value on error.  The following errors are
 *   reported (errno is not set!)
 *
 *     EINVAL: If count is not aligned with the flash boundaries (i.e.
 *             some MCU's require per half-word or even word access)
 *     EFAULT: On invalid address
 *     EIO:    On unsuccessful read
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
ssize_t up_progmem_read(size_t addr, void *buf, size_t count)
{
  uint8_t *byte = (uint8_t *)buf;
  uint32_t i;
  int ret;

  /* Check for valid address range */

  if (addr < FMC_PROGMEM_SECTOR_SADDR)
    {
      return -EFAULT;
    }

  if ((addr + count) > FMC_PROGMEM_SECTOR_EADDR)
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&g_gd32_progmem_lock);
  if (ret < 0)
    {
      return -EFAULT;
    }

  /* Read data */

  for (i = 0; i < count; i++)
    {
      byte[i] = *(uint8_t *)addr;
      addr++;
    }

  nxmutex_unlock(&g_gd32_progmem_lock);
}
#endif

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return value of erase state.
 *
 ****************************************************************************/

uint8_t up_progmem_erasestate(void)
{
  return FMC_ERASE_STATE_VAL;
}
