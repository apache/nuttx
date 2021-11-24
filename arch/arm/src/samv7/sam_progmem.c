/****************************************************************************
 * arch/arm/src/samv7/sam_progmem.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/samv7/chip.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "barriers.h"

#include "hardware/sam_memorymap.h"

#include "sam_eefc.h"
#include "sam_progmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMV7_PROGMEM_NSECTORS
#  error CONFIG_SAMV7_PROGMEM_NSECTORS is not defined
#endif

#ifndef CONFIG_ARCH_RAMFUNCS
#   error "Flashing function should executed in ram"
#endif

/* Chip dependencies */

#if defined(CONFIG_ARCH_CHIP_SAMV71) || defined(CONFIG_ARCH_CHIP_SAME70)
/* All sectors are 128KB and are uniform in size.
 * The only exception is sector 0 which is subdivided into two small sectors
 * of 8KB and one larger sector of 112KB.
 * The page size is 512 bytes.  However, the smallest thing that can be
 * erased is four pages.  We will refer to this as a "cluster".
 */

#  define SAMV7_SECTOR_SHIFT      (17)   /* 2**17 = 128KB */
#  define SAMV7_PAGE_SHIFT        (9)    /* 2**9  = 512B  */
#  define SAMV7_CLUSTER_SHIFT     (13)   /* 2**13 = 8*KB = 16 pages */
#  define SAMV7_LOCK_REGION_SHIFT (13)   /* 2**13 = 8*KB = 16 pages */
#else
#  error FLASH geometry for this SAMV7 chip not known
#endif

/* Sizes and masks */

#define SAMV7_SECTOR_SIZE        (1 << SAMV7_SECTOR_SHIFT)
#define SAMV7_SECTOR_MASK        (SAMV7_SECTOR_SIZE - 1)

#define SAMV7_PAGE_SIZE          (1 << SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE_WORDS         (1 << (SAMV7_PAGE_SHIFT - 2))
#define SAMV7_PAGE_MASK          (SAMV7_PAGE_SIZE - 1)

#define SAMV7_CLUSTER_SIZE       (1 << SAMV7_CLUSTER_SHIFT)
#define SAMV7_CLUSTER_MASK       (SAMV7_CLUSTER_SHIFT - 1)

/* Relationships */

#define SAMV7_PAGE2SEC_SHIFT     (SAMV7_SECTOR_SHIFT - SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE_PER_SEC       (1 << SAMV7_PAGE2SEC_SHIFT)

#define SAMV7_PAGE2CLUST_SHIFT   (SAMV7_CLUSTER_SHIFT - SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE_PER_CLUSTER   (1 << SAMV7_PAGE2CLUST_SHIFT)

#define SAMV7_CLUST2SECT_SHIFT   (SAMV7_SECTOR_SHIFT - SAMV7_CLUSTER_SHIFT)
#define SAMV7_CLUSTER_PER_SEC    (1 << SAMV7_CLUST2SECT_SHIFT)

/* Conversions */

#define SAMV7_BYTE2PAGE(o)       ((o) >> SAMV7_PAGE_SHIFT)
#define SAMV7_BYTE2CLUST(o)      ((o) >> SAMV7_CLUSTER_SHIFT)
#define SAMV7_BYTE2SECT(o)       ((o) >> SAMV7_SECTOR_SHIFT)

#define SAMV7_PAGE2BYTE(p)       ((p) << SAMV7_PAGE_SHIFT)
#define SAMV7_PAGE2CLUST(p)      ((p) >> SAMV7_PAGE2CLUST_SHIFT)
#define SAMV7_PAGE2SEC(p)        ((p) >> SAMV7_PAGE2SEC_SHIFT)

#define SAMV7_CLUST2BYTE(c)      ((c) << SAMV7_CLUSTER_SHIFT)
#define SAMV7_CLUST2PAGE(c)      ((c) << SAMV7_PAGE2CLUST_SHIFT)
#define SAMV7_CLUST2SEC(c)       ((c) >> SAMV7_CLUST2SECT_SHIFT)

#define SAMV7_SEC2BYTE(s)        ((s) << SAMV7_SECTOR_SHIFT)
#define SAMV7_SEC2PAGE(s)        ((s) << SAMV7_PAGE2SEC_SHIFT)
#define SAMV7_SEC2CLUST(s)       ((s) << SAMV7_CLUST2SECT_SHIFT)

/* Total FLASH sizes */

#define SAMV7_TOTAL_NSECTORS     (SAMV7_FLASH_SIZE >> SAMV7_SECTOR_SHIFT)
#define SAMV7_TOTAL_NPAGES       SAMV7_SEC2PAGE(SAMV7_TOTAL_NSECTORS)
#define SAMV7_TOTAL_NCLUSTERS    SAMV7_SEC2CLUST(SAMV7_TOTAL_NSECTORS)

/* Start and size of the programmable region  */

#define SAMV7_PROGMEM_NBYTES     (CONFIG_SAMV7_PROGMEM_NSECTORS << SAMV7_SECTOR_SHIFT)
#define SAMV7_PROGMEM_END        (SAM_INTFLASH_BASE + SAMV7_FLASH_SIZE)
#define SAMV7_PROGMEM_START      (SAMV7_PROGMEM_END - SAMV7_PROGMEM_NBYTES)

#define SAMV7_PROGMEM_NPAGES     SAMV7_SEC2PAGE(CONFIG_SAMV7_PROGMEM_NSECTORS)
#define SAMV7_PROGMEM_ENDPAGE    (SAMV7_TOTAL_NPAGES)
#define SAMV7_PROGMEM_STARTPAGE  (SAMV7_PROGMEM_ENDPAGE - SAMV7_PROGMEM_NPAGES)

#define SAMV7_PROGMEM_NCLUSTERS  SAMV7_SEC2CLUST(CONFIG_SAMV7_PROGMEM_NSECTORS)
#define SAMV7_PROGMEM_ENDCLUST   (SAMV7_TOTAL_NCLUSTERS)
#define SAMV7_PROGMEM_STARTCLUST (SAMV7_PROGMEM_ENDCLUST - SAMV7_PROGMEM_NCLUSTERS)

#define SAMV7_PROGMEM_NSECTORS   (CONFIG_SAMV7_PROGMEM_NSECTORS)
#define SAMV7_PROGMEM_ENDSEC     (SAMV7_TOTAL_NSECTORS)
#define SAMV7_PROGMEM_STARTSEC   (SAMV7_PROGMEM_ENDSEC - CONFIG_SAMV7_PROGMEM_NSECTORS)

#define SAMV7_PROGMEM_ERASEDVAL  (0xff)

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

static uint32_t g_page_buffer[SAMV7_PAGE_WORDS];
static sem_t g_page_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: page_buffer_lock
 *
 * Description:
 *   Get exclusive access to the global page buffer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int page_buffer_lock(void)
{
  return nxsem_wait_uninterruptible(&g_page_sem);
}

#define page_buffer_unlock() nxsem_post(&g_page_sem)

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
  uint32_t regval;

  /* Set flash access mode to 128bit and wait status to 4 */

  sam_eefc_initaccess(SAM_EFC_ACCESS_MODE_128, 4);

  /* Make sure that the read interrupt is disabled */

  regval  = getreg32(SAM_EEFC_FMR);
  regval &= ~EEFC_FMR_FRDY;
  sam_eefc_writefmr(regval);

  /* Initialize the semaphore that manages exclusive access to the global
   * page buffer.
   */

  nxsem_init(&g_page_sem, 0, 1);
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
  return SAMV7_PROGMEM_NCLUSTERS;
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
 *   Return cluster size
 *
 ****************************************************************************/

size_t up_progmem_pagesize(size_t cluster)
{
  return SAMV7_CLUSTER_SIZE;
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
  return SAMV7_CLUSTER_SIZE;
}

/****************************************************************************
 * Name: up_progmem_getpage
 *
 * Description:
 *   Address to cluster conversion
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
  if (address >= SAMV7_PROGMEM_START)
    {
      address -= SAMV7_PROGMEM_START;
    }

  if (address >= SAMV7_PROGMEM_NBYTES)
    {
      return -EFAULT;
    }

  return address >> SAMV7_CLUSTER_SHIFT;
}

/****************************************************************************
 * Name: up_progmem_getaddress
 *
 * Description:
 *   Cluster to address conversion
 *
 * Input Parameters:
 *   cluster - cluster index
 *
 * Returned Value:
 *   Base address of given cluster, maximum size if cluster index is not
 *   valid.
 *
 ****************************************************************************/

size_t up_progmem_getaddress(size_t cluster)
{
  if (cluster >= SAMV7_PROGMEM_NCLUSTERS)
    {
      return SAMV7_PROGMEM_NBYTES;
    }

  return (cluster << SAMV7_CLUSTER_SHIFT) + SAMV7_PROGMEM_START;
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
  uint32_t arg;
  int ret;

  if (cluster >= SAMV7_PROGMEM_NCLUSTERS)
    {
      return -EFAULT;
    }

  /* Get the page number of the start of the cluster */

  page = SAMV7_CLUST2PAGE((uint32_t)cluster) + SAMV7_PROGMEM_STARTPAGE;

  /* Erase all pages in the cluster */

  sam_eefc_unlock(page, SAMV7_PAGE_PER_CLUSTER);

  /* Get FARG field for EPA command:
   *
   * The first page to be erased is specified in the FARG[15:2] field of
   * the EEFC_FCR register. The first page number must be modulo 4, 8,16 or
   * 32 according to the number of pages to erase at the same time.
   *
   * The 2 lowest bits of the FARG field define the number of pages to
   * be erased (FARG[1:0]).
   */

#if SAMV7_PAGE_PER_CLUSTER == 32
    arg   = page | 3;   /* Not valid for small 8 KB sectors */
#elif SAMV7_PAGE_PER_CLUSTER == 16
    arg   = page | 2;
#elif SAMV7_PAGE_PER_CLUSTER == 8
#  error Cluster size of 8 not supported
    arg   = page | 1;   /* Only valid for small 8 KB sectors */
#elif SAMV7_PAGE_PER_CLUSTER == 4
#  error Cluster size of 4 not supported
    arg   = page | 0;   /* Only valid for small 8 KB sectors */
#else
#  error Unsupported/undefined pages-per-cluster size
#endif

  ret = sam_eefc_command(FCMD_EPA, arg);
  if (ret < 0)
    {
      return ret;
    }

  /* Verify that the cluster of pages is really erased */

  if (up_progmem_ispageerased(cluster) == 0)
    {
      return SAMV7_CLUSTER_SIZE; /* Success */
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

  if (cluster >= SAMV7_PROGMEM_NCLUSTERS)
    {
      return -EFAULT;
    }

  /* Flush and invalidate D-Cache for this address range */

  address = (cluster << SAMV7_CLUSTER_SHIFT) + SAMV7_PROGMEM_START;
  up_flush_dcache(address, address + SAMV7_CLUSTER_SIZE);

  /* Verify that the cluster is erased (i.e., all SAMV7_PROGMEM_ERASEDVAL) */

  for (nleft = SAMV7_CLUSTER_SIZE, nwritten = 0;
       nleft > 0;
       nleft--, address++)
    {
      if (getreg8(address) != SAMV7_PROGMEM_ERASEDVAL)
        {
          nwritten++;
        }
    }

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
 *   buflen  - Number of bytes to write
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
  FAR uint32_t *dest;
  FAR const uint32_t *src;
  size_t written;
  size_t xfrsize;
  size_t offset;
  size_t page;
  size_t i;
  int ret;

  /* Convert the address into a FLASH byte offset, if necessary */

  offset = address;
  if (address >= SAMV7_PROGMEM_START)
    {
      /* Convert address to an offset relative to be beginning of the
       * writable FLASH region.
       */

      offset -= SAMV7_PROGMEM_START;
    }

  /* Check for valid address range */

  if ((offset + buflen) > SAMV7_PROGMEM_NBYTES)
    {
      return -EFAULT;
    }

  /* Get the page number corresponding to the flash offset and the byte
   * offset into the page.
   */

  page = SAMV7_BYTE2PAGE((uint32_t)offset) + SAMV7_PROGMEM_STARTPAGE;
  offset &= SAMV7_PAGE_MASK;

  /* Get exclusive access to the global page buffer */

  ret = page_buffer_lock();
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Make sure that the FLASH is unlocked */

  sam_eefc_unlock(page, SAMV7_BYTE2PAGE(buflen + SAMV7_PAGE_MASK));

  /* Loop until all of the data has been written */

  dest    = (FAR uint32_t *)address;
  written = 0;

  while (buflen > 0)
    {
      /* How much can we write into this page? */

      xfrsize = MIN((size_t)SAMV7_PAGE_SIZE - offset, buflen) ;

      /* Do we need to use the intermediate buffer? */

      if (offset == 0 && xfrsize == SAMV7_PAGE_SIZE)
        {
          /* No, we can take the data directly from the user buffer */

          src = (FAR const uint32_t *)buffer;
        }
      else
        {
          /* Yes, copy data into global page buffer */

          if (offset > 0)
            {
              memcpy(g_page_buffer, dest, offset);
            }

          memcpy((uint8_t *)g_page_buffer + offset, buffer, xfrsize);

          if (offset + xfrsize < SAMV7_PAGE_SIZE)
            {
              memcpy((uint8_t *)g_page_buffer + offset + xfrsize,
                     (const uint8_t *)dest + offset + xfrsize,
                     SAMV7_PAGE_SIZE - offset - xfrsize);
            }

          src = g_page_buffer;
        }

      /* Write the page */

      for (i = 0; i < (SAMV7_PAGE_SIZE / sizeof(uint32_t)); i++)
        {
          *dest++ = *src++;
           ARM_DMB();
        }

      /* Flush the data cache to memory */

      up_clean_dcache(address, address + SAMV7_PAGE_SIZE);

      /* Send the write command */

      ret = sam_eefc_command(FCMD_WP, page);
      if (ret >= 0)
        {
          written += xfrsize;
        }

      /* Adjust pointers and counts for the next time through the loop */

      address += xfrsize;
      dest     = (FAR uint32_t *)address;
      buffer   = (FAR void *)((uintptr_t)buffer + xfrsize);
      buflen  -= xfrsize;
      offset   = 0;
      page++;
    }

  page_buffer_unlock();
  return written;
}

/****************************************************************************
 * Name: up_progmem_erasestate
 *
 * Description:
 *   Return a byte that represents flash erased value state
 *
 ****************************************************************************/

ssize_t up_progmem_erasestate(void)
{
  return SAMV7_PROGMEM_ERASEDVAL;
}
