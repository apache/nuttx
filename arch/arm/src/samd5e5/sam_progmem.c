/****************************************************************************
 * arch/arm/src/samd5e5/sam_progmem.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/samd5e5/chip.h>

#include "arm_internal.h"
#include "hardware/sam_memorymap.h"
#include "hardware/sam_nvmctrl.h"

#include "sam_progmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMD5E5_PROGMEM_NSECTORS
#  error CONFIG_SAMD5E5_PROGMEM_NSECTORS is not defined
#endif

#define USE_WRITE_WQW
#define USE_UNLOCK
#define USE_LOCK

/* Chip dependencies */

#if defined(CONFIG_ARCH_CHIP_SAMD5X)
/* The page size is 512 bytes.  However, the smallest thing that can be
 * erased is 16 pages.  We will refer to this as a "cluster".
 */

#  define SAMD5E5_SECTOR_SHIFT      (15)   /* 2**15 = 32KB SAMD51J20 */
#  define SAMD5E5_PAGE_SHIFT        (9)    /* 2**9  = 512B  */
#  define SAMD5E5_CLUSTER_SHIFT     (13)   /* 2**13 = 8*KB = 16 pages */
#  define SAMD5E5_LOCK_REGION_SHIFT (15)   /* 2**15 = 32KB SAMD51J20 */
#else
#  error FLASH geometry for this SAMD5E5 chip not known
#endif

/* Sizes and masks */

#define SAMD5E5_SECTOR_SIZE        (1 << SAMD5E5_SECTOR_SHIFT)
#define SAMD5E5_SECTOR_MASK        (SAMD5E5_SECTOR_SIZE - 1)

#define SAMD5E5_PAGE_SIZE          (1 << SAMD5E5_PAGE_SHIFT)
#define SAMD5E5_PAGE_WORDS         (1 << (SAMD5E5_PAGE_SHIFT - 2))
#define SAMD5E5_PAGE_MASK          (SAMD5E5_PAGE_SIZE - 1)

#define SAMD5E5_CLUSTER_SIZE       (1 << SAMD5E5_CLUSTER_SHIFT)
#define SAMD5E5_CLUSTER_MASK       (SAMD5E5_CLUSTER_SHIFT - 1)

/* Relationships */

#define SAMD5E5_PAGE2SEC_SHIFT     (SAMD5E5_SECTOR_SHIFT - SAMD5E5_PAGE_SHIFT)
#define SAMD5E5_PAGE_PER_SEC       (1 << SAMD5E5_PAGE2SEC_SHIFT)

#define SAMD5E5_PAGE2CLUST_SHIFT   (SAMD5E5_CLUSTER_SHIFT - SAMD5E5_PAGE_SHIFT)
#define SAMD5E5_PAGE_PER_CLUSTER   (1 << SAMD5E5_PAGE2CLUST_SHIFT)

#define SAMD5E5_CLUST2SECT_SHIFT   (SAMD5E5_SECTOR_SHIFT - SAMD5E5_CLUSTER_SHIFT)
#define SAMD5E5_CLUSTER_PER_SEC    (1 << SAMD5E5_CLUST2SECT_SHIFT)

/* Conversions */

#define SAMD5E5_BYTE2PAGE(o)       ((o) >> SAMD5E5_PAGE_SHIFT)
#define SAMD5E5_BYTE2CLUST(o)      ((o) >> SAMD5E5_CLUSTER_SHIFT)
#define SAMD5E5_BYTE2SECT(o)       ((o) >> SAMD5E5_SECTOR_SHIFT)

#define SAMD5E5_PAGE2BYTE(p)       ((p) << SAMD5E5_PAGE_SHIFT)
#define SAMD5E5_PAGE2CLUST(p)      ((p) >> SAMD5E5_PAGE2CLUST_SHIFT)
#define SAMD5E5_PAGE2SEC(p)        ((p) >> SAMD5E5_PAGE2SEC_SHIFT)

#define SAMD5E5_CLUST2BYTE(c)      ((c) << SAMD5E5_CLUSTER_SHIFT)
#define SAMD5E5_CLUST2PAGE(c)      ((c) << SAMD5E5_PAGE2CLUST_SHIFT)
#define SAMD5E5_CLUST2SEC(c)       ((c) >> SAMD5E5_CLUST2SECT_SHIFT)

#define SAMD5E5_SEC2BYTE(s)        ((s) << SAMD5E5_SECTOR_SHIFT)
#define SAMD5E5_SEC2PAGE(s)        ((s) << SAMD5E5_PAGE2SEC_SHIFT)
#define SAMD5E5_SEC2CLUST(s)       ((s) << SAMD5E5_CLUST2SECT_SHIFT)

/* Lock region */

#define SAMD5E5_LOCK_REGION_SIZE   (1 << SAMD5E5_LOCK_REGION_SHIFT)
#define SAMD5E5_LOCK_REGION_MASK   (SAMD5E5_LOCK_REGION_SIZE - 1)

/* Total FLASH sizes */

#define SAMD5E5_TOTAL_NSECTORS     (SAMD5E5_FLASH_SIZE >> SAMD5E5_SECTOR_SHIFT)
#define SAMD5E5_TOTAL_NPAGES       SAMD5E5_SEC2PAGE(SAMD5E5_TOTAL_NSECTORS)
#define SAMD5E5_TOTAL_NCLUSTERS    SAMD5E5_SEC2CLUST(SAMD5E5_TOTAL_NSECTORS)

/* Start and size of the programmable region  */

#define SAMD5E5_PROGMEM_NBYTES     (CONFIG_SAMD5E5_PROGMEM_NSECTORS << SAMD5E5_SECTOR_SHIFT)
#define SAMD5E5_PROGMEM_END        (SAM_FLASH_BASE + SAMD5E5_FLASH_SIZE)
#define SAMD5E5_PROGMEM_START      (SAMD5E5_PROGMEM_END - SAMD5E5_PROGMEM_NBYTES)

#define SAMD5E5_PROGMEM_NPAGES     SAMD5E5_SEC2PAGE(CONFIG_SAMD5E5_PROGMEM_NSECTORS)
#define SAMD5E5_PROGMEM_ENDPAGE    (SAMD5E5_TOTAL_NPAGES)
#define SAMD5E5_PROGMEM_STARTPAGE  (SAMD5E5_PROGMEM_ENDPAGE - SAMD5E5_PROGMEM_NPAGES)

#define SAMD5E5_PROGMEM_NCLUSTERS  SAMD5E5_SEC2CLUST(CONFIG_SAMD5E5_PROGMEM_NSECTORS)
#define SAMD5E5_PROGMEM_ENDCLUST   (SAMD5E5_TOTAL_NCLUSTERS)
#define SAMD5E5_PROGMEM_STARTCLUST (SAMD5E5_PROGMEM_ENDCLUST - SAMD5E5_PROGMEM_NCLUSTERS)

#define SAMD5E5_PROGMEM_NSECTORS   (CONFIG_SAMD5E5_PROGMEM_NSECTORS)
#define SAMD5E5_PROGMEM_ENDSEC     (SAMD5E5_TOTAL_NSECTORS)
#define SAMD5E5_PROGMEM_STARTSEC   (SAMD5E5_PROGMEM_ENDSEC - CONFIG_SAMD5E5_PROGMEM_NSECTORS)

#define SAMD5E5_PROGMEM_ERASEDVAL  (0xffu)

/* Misc stuff */

#ifndef MIN
#  define MIN(a, b)              ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a, b)              ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t g_page_buffer[SAMD5E5_PAGE_WORDS];
static mutex_t g_page_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nvm_command
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

static int nvm_command(uint16_t cmd, uint32_t arg)
{
  uint16_t regval;

  while (!(getreg16(SAM_NVMCTRL_STATUS) & NVMCTRL_STATUS_READY))
    {
      /* Wait until this module isn't busy */
    }

  /* Check for errors */

  regval = getreg16(SAM_NVMCTRL_INTFLAG);
  if ((regval & (NVMCTRL_INT_LOCKE |
                 NVMCTRL_INT_PROGE |
                 NVMCTRL_INT_ADDRE)) != 0)
    {
      ferr("ERROR: cmd=0x%x regval=0x%x\n", cmd, regval);
      return -EIO;
    }

  putreg16(NVMCTRL_INT_DONE, SAM_NVMCTRL_INTFLAG);

  /* Set address */

  if (arg)
    putreg32(arg, SAM_NVMCTRL_ADDR);

  /* Write the command to the flash command register */

  putreg16(cmd | NVMCTRL_CTRLB_CMDEX_KEY, SAM_NVMCTRL_CTRLB);

  while (!(getreg16(SAM_NVMCTRL_STATUS) & NVMCTRL_STATUS_READY))
    {
      /* Wait until this module isn't busy */
    }

  return OK;
}

#ifdef USE_UNLOCK
/****************************************************************************
 * Name: nvm_unlock
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

static int nvm_unlock(size_t page, size_t npages)
{
  size_t start_region;
  size_t end_region;
  size_t unlockregion;
  int ret;

  /* Align the page to the unlock region */

  end_region   = SAMD5E5_PAGE2BYTE(page + npages) &
                ~SAMD5E5_LOCK_REGION_MASK;
  start_region = SAMD5E5_PAGE2BYTE(page) &
                ~SAMD5E5_LOCK_REGION_MASK;
  unlockregion = start_region;

  do
    {
      finfo("INFO: unlock region=%d address=0x%x\n",
        unlockregion >> SAMD5E5_LOCK_REGION_SHIFT, unlockregion);
      ret = nvm_command(NVMCTRL_CTRLB_CMD_UR, unlockregion);
      if (ret < 0)
        {
          return ret;
        }

      unlockregion += SAMD5E5_LOCK_REGION_SIZE;
    }
  while (unlockregion < end_region);

  return OK;
}
#endif

#ifdef USE_LOCK

/****************************************************************************
 * Name: nvm_lock
 *
 * Description:
 *   Make sure that the FLASH is locked
 *
 * Input Parameters:
 *   page  - The first page to lock
 *   npages - The number of consecutive pages to lock
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nvm_lock(size_t page, size_t npages)
{
  size_t start_region;
  size_t end_region;
  size_t lockregion;
  int ret;

  /* Align the page to the unlock region */

  end_region   = SAMD5E5_PAGE2BYTE(page + npages) &
                ~SAMD5E5_LOCK_REGION_MASK;
  start_region = SAMD5E5_PAGE2BYTE(page) &
                ~SAMD5E5_LOCK_REGION_MASK;
  lockregion = start_region;

  do
    {
      finfo("INFO: lock region=%d address=0x%x\n",
        lockregion >> SAMD5E5_LOCK_REGION_SHIFT, lockregion);
      ret = nvm_command(NVMCTRL_CTRLB_CMD_LR, lockregion);
      if (ret < 0)
        {
          return ret;
        }

      lockregion += SAMD5E5_LOCK_REGION_SIZE;
    }
  while (lockregion < end_region);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_progmem_initialize
 *
 * Description:
 *   Call to initialize FLASH programming memory access
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_progmem_initialize(void)
{
  uint16_t ctrla;

  ctrla = getreg16(SAM_NVMCTRL_CTRLA);
  ctrla &= ~(NVMCTRL_CTRLA_AHBNS1    |
            NVMCTRL_CTRLA_AHBNS0     |
            NVMCTRL_CTRLA_PRM_MASK   |
            NVMCTRL_CTRLA_WMODE_MASK |
            NVMCTRL_CTRLA_SUSPEN);

  ctrla |= NVMCTRL_CTRLA_CACHEDIS1  |
           NVMCTRL_CTRLA_CACHEDIS0  |
           NVMCTRL_CTRLA_PRM_MANUAL |
           NVMCTRL_CTRLA_WMODE_MAN  |
           NVMCTRL_CTRLA_AUTOWS;
  putreg16(ctrla, SAM_NVMCTRL_CTRLA);

  /* Initialize the mutex that manages exclusive access to the global
   * page buffer.
   */

  nxmutex_init(&g_page_lock);
}

/****************************************************************************
 * Name: up_progmem_neraseblocks
 *
 * Description:
 *   Return number of clusters in the available FLASH memory.
 *
 ****************************************************************************/

size_t up_progmem_neraseblocks(void)
{
  return SAMD5E5_PROGMEM_NCLUSTERS;
}

/****************************************************************************
 * Name: up_progmem_isuniform
 *
 * Description:
 *   Cluster size is uniform?  Say 'yes' even though that is not strictly
 *   true do to the odd organization of sector 0.
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

size_t up_progmem_pagesize(size_t cluster)
{
  return SAMD5E5_PAGE_SIZE;
}

/****************************************************************************
 * Name: up_progmem_erasesize
 *
 * Description:
 *   Return cluster size
 *
 ****************************************************************************/

size_t up_progmem_erasesize(size_t cluster)
{
  return SAMD5E5_CLUSTER_SIZE;
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to page conversion
 *
 * Input Parameters:
 *   address - Address with or without flash offset
 *
 * Returned Value:
 *   Page or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid address
 *
 ****************************************************************************/

ssize_t up_progmem_getpage(size_t address)
{
  if (address >= SAMD5E5_PROGMEM_START)
    {
      address -= SAMD5E5_PROGMEM_START;
    }

  if (address >= SAMD5E5_PROGMEM_NBYTES)
    {
      return -EFAULT;
    }

  return address >> SAMD5E5_PAGE_SHIFT;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Cluster to address conversion
 *
 * Input Parameters:
 *   page - page index
 *
 * Returned Value:
 *   Base address of given page, maximum size if page index is not valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t page)
{
  if (page >= SAMD5E5_PROGMEM_NPAGES)
    {
      return SAMD5E5_PROGMEM_NBYTES;
    }

  return (page << SAMD5E5_PAGE_SHIFT) + SAMD5E5_PROGMEM_START;
}

/****************************************************************************
 * Name: up_progmem_eraseblock
 *
 * Description:
 *   Erase selected cluster.
 *
 * Input Parameters:
 *   cluster - cluster index to be erased
 *
 * Returned Value:
 *   Page size or negative value on error.  The following errors are reported
 *   (errno is not set!):
 *
 *     -EFAULT: On invalid cluster
 *     -EIO:    On unsuccessful erase
 *     -EROFS:  On access to write protected area
 *     -EACCES: Insufficient permissions (read/write protected)
 *     -EPERM:  If operation is not permitted due to some other constraints
 *              (i.e. some internal block is not running etc.)
 *
 ****************************************************************************/

ssize_t up_progmem_eraseblock(size_t cluster)
{
  uint32_t page;
  int ret;

  finfo("INFO: cluster=%d\n", cluster);
  if (cluster >= SAMD5E5_PROGMEM_NCLUSTERS)
    {
      return -EFAULT;
    }

  /* Get the page number of the start of the cluster */

  page = SAMD5E5_CLUST2PAGE((uint32_t)cluster) + SAMD5E5_PROGMEM_STARTPAGE;

  /* Erase all pages in the cluster */

#ifdef USE_UNLOCK
  nvm_unlock(page, SAMD5E5_PAGE_PER_CLUSTER);
#endif

  finfo("INFO: erase block=%d address=0x%x\n",
    page, SAMD5E5_PAGE2BYTE(page));
  ret = nvm_command(NVMCTRL_CTRLB_CMD_EB, SAMD5E5_PAGE2BYTE(page));

#ifdef USE_LOCK
  nvm_lock(page, SAMD5E5_PAGE_PER_CLUSTER);
#endif

  if (ret < 0)
    {
      return ret;
    }

  /* Verify that the cluster of pages is really erased */

  if (up_progmem_ispageerased(cluster) == 0)
    {
      return SAMD5E5_CLUSTER_SIZE; /* Success */
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
 *   Checks whether cluster is erased
 *
 * Input Parameters:
 *    cluster - cluster to be checked
 *
 * Returned Value:
 *   Returns number of bytes erased or negative value on error. If it
 *   returns zero then complete cluster is empty (erased).
 *
 *   The following errors are reported (errno is not set!)
 *     -EFAULT: On invalid cluster
 *
 ****************************************************************************/

ssize_t up_progmem_ispageerased(size_t cluster)
{
  size_t address;
  size_t nwritten;
  int nleft;

  finfo("INFO: cluster=%d\n", cluster);
  if (cluster >= SAMD5E5_PROGMEM_NCLUSTERS)
    {
      return -EFAULT;
    }

  /* Flush and invalidate D-Cache for this address range */

  address = (cluster << SAMD5E5_CLUSTER_SHIFT) + SAMD5E5_PROGMEM_START;
  up_flush_dcache(address, address + SAMD5E5_CLUSTER_SIZE);

  /* Verify that the cluster is erased (i.e., all 0xff) */

  for (nleft = SAMD5E5_CLUSTER_SIZE, nwritten = 0;
       nleft > 0;
       nleft--, address++)
    {
      if (getreg8(address) != SAMD5E5_PROGMEM_ERASEDVAL)
        {
          nwritten++;
        }
    }

  if (nwritten)
    fwarn("WARN: non written=%d\n", nwritten);
  return nwritten;
}

/****************************************************************************
 * Name: up_progmem_write
 *
 * Description:
 *   Program data at given address
 *
 * Input Parameters:
 *   address - Address with or without flash offset
 *   buffer  - Pointer to buffer
 *   buflen   - Number of bytes to write
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

ssize_t up_progmem_write(size_t address, const void *buffer, size_t buflen)
{
  irqstate_t flags;
  uint32_t *dest;
  const uint32_t *src;
  size_t written;
  size_t xfrsize;
  size_t offset;
  size_t page;
  size_t i;
  int ret;
#ifdef USE_UNLOCK
  size_t lock;
  size_t locksize;
#endif

  finfo("INFO: address=0x%x buflen=%d\n", address, buflen);

  /* Convert the address into a FLASH byte offset, if necessary */

  offset = address;
  if (address >= SAMD5E5_PROGMEM_START)
    {
      /* Convert address to an offset relative to be beginning of the
       * writable FLASH region.
       */

      offset -= SAMD5E5_PROGMEM_START;
    }

  /* Check for valid address range */

  if ((offset + buflen) > SAMD5E5_PROGMEM_NBYTES)
    {
      return -EFAULT;
    }

  /* Get exclusive access to the global page buffer */

  nxmutex_lock(&g_page_lock);

  /* Get the page number corresponding to the flash offset and the byte
   * offset into the page.
   */

  page = SAMD5E5_BYTE2PAGE((uint32_t)offset) + SAMD5E5_PROGMEM_STARTPAGE;
  offset &= SAMD5E5_PAGE_MASK;

#ifdef USE_UNLOCK /* Make sure that the FLASH is unlocked */
  lock = page;
  locksize = SAMD5E5_BYTE2PAGE(buflen);
  nvm_unlock(lock, locksize);
#endif

  flags = enter_critical_section();

  /* Loop until all of the data has been written */

  dest = (uint32_t *)(address & ~SAMD5E5_PAGE_MASK);
  written = 0;
  while (buflen > 0)
    {
      /* How much can we write into this page? */

      xfrsize = MIN((size_t)SAMD5E5_PAGE_SIZE - offset, buflen);

      /* Do we need to use the intermediate buffer? */

      if (offset == 0 && xfrsize == SAMD5E5_PAGE_SIZE)
        {
          /* No, we can take the data directly from the user buffer */

          src = (const uint32_t *)buffer;
        }
      else
        {
          /* Yes, copy data into global page buffer */

          if (offset > 0)
            {
              memcpy((uint8_t *)g_page_buffer, (uint8_t *)dest, offset);
            }

          memcpy((uint8_t *)g_page_buffer + offset,
                 (uint8_t *)buffer, xfrsize);

          if (offset + xfrsize < SAMD5E5_PAGE_SIZE)
            {
              memcpy((uint8_t *)g_page_buffer + offset + xfrsize,
                     (const uint8_t *)dest + offset + xfrsize,
                     SAMD5E5_PAGE_SIZE - offset - xfrsize);
            }

          src = g_page_buffer;
        }

#ifdef USE_WRITE_WQW
      if (xfrsize <= (0x10 - (offset & 0xf)))
        {
          /* Write the page buffer */

          dest += (offset & ~0xf) >> 2;
          src += (offset & ~0xf) >> 2;

          /* Dump flash data */

          for (i = 0; i < 4; i++)
            {
              finfo("INFO: dest=%p write 0x%x over 0x%x\n",
                dest + i, *(src + i), *(dest + i));
            }

          nvm_command(NVMCTRL_CTRLB_CMD_PBC, 0);

          /* Write the page buffer */

          for (i = 0; i < 4; i++)
            {
              *dest++ = *src++;
            }

          /* Flush the data cache to memory */

          up_clean_dcache(address & ~SAMD5E5_PAGE_MASK,
            (address & ~SAMD5E5_PAGE_MASK) + SAMD5E5_PAGE_SIZE);

          /* Send the 4 words write command */

          ret = nvm_command(NVMCTRL_CTRLB_CMD_WQW, 0);
          if (ret >= 0)
            {
              written += xfrsize;
            }

          dest -= i;
          src -= i;

          /* Compare page data */

          for (i = 0; i < 4; i++)
            {
              if (*dest != *src)
                {
                  fwarn("WQW dest=%p (dest 0x%x != src 0x%x) address=0x%x",
                    dest, *dest, *src, address);
                  fwarn("offset=0x%x xfrsize=%d buflen=%d ECCERR=0x%x\n",
                    offset, xfrsize, buflen, getreg32(SAM_NVMCTRL_ECCERR));
                }

              dest++;
              src++;
            }
        }
      else
        {
#endif
          nvm_command(NVMCTRL_CTRLB_CMD_PBC, 0);

          /* Write the page buffer */

          for (i = 0; i < SAMD5E5_PAGE_WORDS; i++)
            {
              *dest++ = *src++;
            }

          /* Flush the data cache to memory */

          up_clean_dcache(address & ~SAMD5E5_PAGE_MASK,
            (address & ~SAMD5E5_PAGE_MASK) + SAMD5E5_PAGE_SIZE);

          /* Send the write command */

          finfo("INFO: WP address=0x%x\n", address & ~SAMD5E5_PAGE_MASK);
          ret = nvm_command(NVMCTRL_CTRLB_CMD_WP, 0);
          if (ret >= 0)
            {
              written += xfrsize;
            }

          dest -= i;
          src -= i;

          /* Compare page data */

          for (i = 0; i < SAMD5E5_PAGE_WORDS; i++)
            {
              if (*dest != *src)
                {
                  fwarn("WQW dest=%p (dest 0x%x != src 0x%x) address=0x%x",
                      dest, *dest, *src, address);
                  fwarn("offset=0x%x xfrsize=%d buflen=%d ECCERR=0x%x\n",
                      offset, xfrsize, buflen, getreg32(SAM_NVMCTRL_ECCERR));
                }

              dest++;
              src++;
            }

#ifdef USE_WRITE_WQW
        }
#endif

      /* Adjust pointers and counts for the next time through the loop */

      address += xfrsize;
      dest     = (uint32_t *)address;
      buffer   = (void *)((uintptr_t)buffer + xfrsize);
      buflen  -= xfrsize;
      offset   = 0;
      page++;
    }

#ifdef USE_LOCK
  nvm_lock(lock, locksize);
#endif

  leave_critical_section(flags);
  nxmutex_unlock(&g_page_lock);
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
  return SAMD5E5_PROGMEM_ERASEDVAL;
}

/****************************************************************************
 *  The NVM User Row contains calibration data that are
 *  automatically read at device power on.
 *  The NVM User Row can be read at address 0x804000.
 ****************************************************************************/

#ifndef _NVM_USER_ROW_BASE
#define _NVM_USER_ROW_BASE 0x804000
#endif
#define _NVM_USER_ROW_N_BITS 96
#define _NVM_USER_ROW_N_BYTES (_NVM_USER_ROW_N_BITS / 8)
#define _NVM_USER_ROW_END (((uint8_t *)_NVM_USER_ROW_BASE) + _NVM_USER_ROW_N_BYTES - 1)
#define _IS_NVM_USER_ROW(b)                                                                                            \
  (((uint8_t *)(b) >= (uint8_t *)(_NVM_USER_ROW_BASE)) && ((uint8_t *)(b) <= (uint8_t *)(_NVM_USER_ROW_END)))
#define _IN_NVM_USER_ROW(b, o) (((uint8_t *)(b) + (o)) <= (uint8_t *)(_NVM_USER_ROW_END))

#define _NVM_USER_PAGE_SIZE 512
#define _NVM_USER_PAGE_OFFSET 32
#define _IS_NVM_USER_PAGE(b)                                                                                            \
  (((uint8_t *)(b) >= (uint8_t *)(_NVM_USER_ROW_BASE)) && ((uint8_t *)(b) <= (((uint8_t *)_NVM_USER_ROW_BASE) + _NVM_USER_PAGE_SIZE - 1)))

/****************************************************************************
 *  The NVM Software Calibration Area can be read at address 0x00800080.
 *  The NVM Software Calibration Area can not be written.
 ****************************************************************************/

#ifndef _NVM_SW_CALIB_AREA_BASE
#define _NVM_SW_CALIB_AREA_BASE 0x00800080
#endif
#define _NVM_SW_CALIB_AREA_N_BITS 45
#define _NVM_SW_CALIB_AREA_N_BYTES (_NVM_SW_CALIB_AREA_N_BITS / 8)
#define _NVM_SW_CALIB_AREA_END (((uint8_t *)_NVM_SW_CALIB_AREA_BASE) + _NVM_SW_CALIB_AREA_N_BYTES - 1)
#define _IS_NVM_SW_CALIB_AREA(b)                                                                                       \
  (((uint8_t *)(b) >= (uint8_t *)_NVM_SW_CALIB_AREA_BASE) && ((uint8_t *)(b) <= (uint8_t *)_NVM_SW_CALIB_AREA_END))
#define _IN_NVM_SW_CALIB_AREA(b, o) (((uint8_t *)(b) + (o)) <= (uint8_t *)(_NVM_SW_CALIB_AREA_END))

ssize_t up_progmem_writeuserpage(const uint32_t offset,
                                 const uint8_t *buffer,
                                 uint16_t count)
{
  size_t i;
  size_t written;
  uint32_t *dest;
  const uint32_t *src;
  uint32_t userpage[128]; /* Copy of user page */

  ASSERT(buffer);

  /* Parameter check. */

  if (!_IS_NVM_USER_PAGE(_NVM_USER_ROW_BASE + offset))
    {
      return -EFAULT;
    }

  /* Cut off if request too many bytes */

  if (!_IS_NVM_USER_PAGE(_NVM_USER_ROW_BASE + offset + count - 1))
    {
      return -EFAULT;
    }

  /* Store previous data. */

  memcpy((uint8_t *)userpage,
        ((uint8_t *)_NVM_USER_ROW_BASE),
        _NVM_USER_PAGE_SIZE);

  /* Modify with buffer data. */

  memcpy((uint8_t *)userpage + offset, buffer, count);

  /* Erase AUX page. */

  nvm_command(NVMCTRL_CTRLB_CMD_EP, _NVM_USER_ROW_BASE);

  dest = (uint32_t *)(_NVM_USER_ROW_BASE);
  src = (const uint32_t *)userpage;
  for (written = 0; written <
    _NVM_USER_PAGE_SIZE; written += 4*sizeof(uint32_t))
    {
      /* Page buffer clear & write. */

      nvm_command(NVMCTRL_CTRLB_CMD_PBC, 0);

      /* Write the page buffer */

      for (i = 0; i < 4; i++)
        {
          *dest++ = *src++;
        }

      /* Send the 4 words write command */

      nvm_command(NVMCTRL_CTRLB_CMD_WQW, 0);

      dest -= i;
      src -= i;

      /* Compare page data */

      for (i = 0; i < 4; i++)
        {
          if (*dest != *src)
            {
              fwarn("WQW dest=%p (dest 0x%x != src 0x%x) ECCERR=0x%x\n",
                dest, *dest, *src, getreg32(SAM_NVMCTRL_ECCERR));
            }

          dest++;
          src++;
        }
    }

  return OK;
}

ssize_t up_progmem_readuserpage(const uint32_t offset,
                                uint8_t *buffer,
                                uint16_t count)
{
  ASSERT(buffer);

  /* Parameter check. */

  if (!_IS_NVM_USER_PAGE(_NVM_USER_ROW_BASE + offset))
    {
      return -EFAULT;
    }

  /* Cut off if request too many bytes */

  if (!_IS_NVM_USER_PAGE(_NVM_USER_ROW_BASE + offset + count - 1))
    {
      return -EFAULT;
    }

  /* Copy data */

  memcpy(buffer, ((uint8_t *)_NVM_USER_ROW_BASE) + offset, count);

  return OK;
}
