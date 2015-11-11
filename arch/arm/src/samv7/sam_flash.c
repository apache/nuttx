/****************************************************************************
 * arch/arm/src/samv7/sam_flash.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>
#include <arch/samv7/chip.h>  /* For SAMV7_ALIGNED_FLASH_SIZE */

#include "up_arch.h"
#include "cache.h"

#include "sam_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#if defined(CONFIG_ARCH_CHIP_SAMV71)
#  define SAMV7_SECTOR_SHIFT     (17)  /* 2**17 = 128KB */
#  define SAMV7_PAGE_SHIFT       (9)   /* 2**9  = 512B  */
#else
#  error FLASH geometry for this SAMV7 chip not known
#endif

#define SAMV7_SECTOR_SIZE        (1 << SAMV7_SECTOR_SHIFT)
#define SAMV7_SECTOR_MASK        (SAMV7_SECTOR_SIZE - 1)
#define SAMV7_PAGE_SIZE          (1 << SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE_MASK          (SAMV7_PAGE_SIZE - 1)
#define SAMV7_SEC2PAGE_SHIFT     (SAMV7_SECTOR_SHIFT - SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE_PER_SEC       (1 << SAMV7_SEC2PAGE_SHIFT)

/* Free FLASH begins at the first sector after the FLASH image */
#warning Missing logic
#define SAMV7_FREE_FLASH_BASE    To be provided

#define ALIGN_UP(v,m)            (((v) + (m)) & ~(m))
#define ALIGN_DOWN(v,m)          ((v) & ~(m))
#define SAMV7_ALIGNED_FLASH_BASE ALIGN_UP(SAMV7_FREE_FLASH_BASE)
#define SAMV7_FREE_FLASH_SIZE    (SAMV7_ALIGNED_FLASH_SIZE - SAMV7_ALIGNED_FLASH_BASE)
#define SAMV7_ALIGNED_FLASH_SIZE ALIGN_DOWN(SAMV7_FREE_FLASH_SIZE)
#define SAMV7_NSECTORS           (SAMV7_ALIGNED_FLASH_SIZE >> SAMV7_SECTOR_SHIFT)
#define SAMV7_NPAGES             (SAMV7_ALIGNED_FLASH_SIZE >> SAMV7_PAGE_SHIFT)

#define SAMV7_WRITE_ALIGN        (16)
#define SAMV7_WRITE_ALIGN_MASK   (15)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sam_flash_unlock(void)
{
#warning Missing logic
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_progmem_npages
 *
 * Description:
 *   Return number of pages
 *
 ****************************************************************************/

size_t up_progmem_npages(void)
{
  return SAMV7_NPAGES;
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
 *   Return page size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t page)
{
  return SAMV7_PAGE_SIZE;
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to page conversion
 *
 * Input Parameters:
 *   addr - Address with or without flash offset (absolute or aligned to page0)
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t addr)
{
  if (addr >= SAMV7_ALIGNED_FLASH_BASE)
    {
      addr -= SAMV7_ALIGNED_FLASH_BASE;
    }

  if (addr >= SAMV7_ALIGNED_FLASH_SIZE)
    {
      return -EFAULT;
    }

  return addr >> SAMV7_PAGE_SHIFT;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Page to address conversion
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
  if (page >= SAMV7_NPAGES)
    {
      return SAMV7_ALIGNED_FLASH_SIZE;
    }

  return (page << SAMV7_PAGE_SHIFT) + SAMV7_ALIGNED_FLASH_BASE;
}

/****************************************************************************
 * Name: up_progmem_erasepage
 *
 * Description:
 *   Erase selected page.
 *
 * Input Parameters:
 *   page -
 *
 * Returned Value:
 *   Page size or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     EFAULT: On invalid page
 *     EIO:    On unsuccessful erase
 *     EROFS:  On access to write protected area
 *     EACCES: Insufficient permissions (read/write protected)
 *     EPERM:  If operation is not permitted due to some other constraints
 *             (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_erasepage(size_t page)
{
  size_t page_address;

  if (page >= SAMV7_NPAGES)
    {
      return -EFAULT;
    }

  /* Erase a single page */

  sam_flash_unlock();
#warning Missing logic

  /* Invalidate I- and D-Cache in this address range */
#warning Mising logic

  /* Verify */

  if (up_progmem_ispageerased(page) == 0)
    {
      return SAMV7_PAGE_SIZE; /* Success */
    }
  else
    {
      return -EIO; /* Failure */
    }
}

/****************************************************************************
 * Name: up_progmem_ispageerased
 *
 * Description:
 *   Checks whether page is erased
 *
 * Input Parameters:
 *    page -
 *
 * Returned Value:
 *   Returns number of bytes written or negative value on error. If it
 *   returns zero then complete page is empty (erased).
 *
 *   The following errors are reported (errno is not set!)
 *     EFAULT: On invalid page
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t bwritten;
  int count;

  if (page >= SAMV7_NPAGES)
    {
      return -EFAULT;
    }

  /* Invalidate D-Cache for this address range */

  addr = up_progmem_getaddress(page);
  arch_invalidate_dcache(addr, addr + SAMV7_PAGE_SIZE);

  /* Verify that the page is erased (i.e., all 0xff) */

  for (count = SAMV7_PAGE_SIZE, bwritten = 0;
       count > 0;
       count--, addr++)
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
 *   Note: this function is not limited to single page and nor it requires
 *   the address be aligned inside the page boundaries.
 *
 * Input Parameters:
 *   addr  - Address with or without flash offset (absolute or aligned to page0)
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
  uint8_t *src = (uint8_t *)buf;
  size_t written = count;

  /* SAMV7 requires 128-bit/16-byte aligned access */

  if ((addr  & SAMV7_WRITE_ALIGN_MASK) != 0 ||
      (count & SAMV7_WRITE_ALIGN_MASK) != 0)
    {
      return -EINVAL;
    }

  /* Check for valid address range */

  if (addr >= SAMV7_ALIGNED_FLASH_BASE)
    {
      /* Convert address to an offset relative to be beginning of the
       * writable FLASH region.
       */

      addr -= SAMV7_ALIGNED_FLASH_BASE;
    }

  if ((addr + count) >= SAMV7_ALIGNED_FLASH_SIZE)
    {
      return -EFAULT;
    }

  /* Write the data to FLASH */

  sam_flash_unlock();
#warning Missing logic

  for (addr += SAMV7_ALIGNED_FLASH_BASE;
       count;
       count -= SAMV7_WRITE_ALIGN, src += SAMV7_WRITE_ALIGN, addr += SAMV7_WRITE_ALIGN)
    {
      /* Write 128-bit/16-bytes block to FLASH and wait to complete */
#warning Mising logic

      /* Invalidate I- and D-Caches for this address range */
#warning Mising logic

      /* Verify */
#warning Mising logic

    }

  return written;
}
