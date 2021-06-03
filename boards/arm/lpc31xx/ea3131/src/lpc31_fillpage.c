/****************************************************************************
 * boards/arm/lpc31xx/ea3131/src/lpc31_fillpage.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/page.h>

#ifdef CONFIG_PAGING
#ifdef CONFIG_PAGING_BINPATH
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <stdbool.h>
#  include <unistd.h>
#  include <fcntl.h>
#  include <nuttx/fs/fs.h>
#  ifdef CONFIG_EA3131_PAGING_SDSLOT
#    include <stdio.h>
#    include <sys/mount.h>
#    include <nuttx/sdio.h>
#    include <nuttx/mmcsd.h>
#    include "lpc31.h"
#  endif
#endif

#if defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
#  include <sys/ioctl.h>
#  include <nuttx/fs/ioctl.h>
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SD SLOT number might depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_EA3131
#  define HAVE_SD     1
#  define HAVE_SPINOR 1
#  if defined(CONFIG_EA3131_PAGING_SDSLOT) && CONFIG_EA3131_PAGING_SDSLOT != 0
#    error "Only one SD slot"
#    undef CONFIG_EA3131_PAGING_SDSLOT
#  endif
#else
  /* Add configuration for new LPC31XX boards here */

#  error "Unrecognized LPC31XX board"
#  undef CONFIG_EA3131_PAGING_SDSLOT
#  undef HAVE_SD
#  undef HAVE_SPINOR
#endif

/* Sanity check:  We can only perform paging using a single source device */

#if defined(CONFIG_PAGING_M25PX) && defined(CONFIG_PAGING_AT45DB)
#  error "Both CONFIG_PAGING_M25PX and CONFIG_PAGING_AT45DB are defined"
#  undef CONFIG_PAGING_M25PX
#endif
#if defined(CONFIG_PAGING_BINPATH) && defined(CONFIG_PAGING_M25PX)
#  error "Both CONFIG_PAGING_BINPATH and CONFIG_PAGING_M25PX are defined"
#  undef CONFIG_PAGING_BINPATH
#endif
#if defined(CONFIG_PAGING_BINPATH) && defined(CONFIG_PAGING_AT45DB)
#  error "Both CONFIG_PAGING_BINPATH and CONFIG_PAGING_AT45DB are defined"
#  undef CONFIG_PAGING_BINPATH
#endif

/* Are we accessing the page source data through a file path? */

#ifdef CONFIG_PAGING_BINPATH

  /* Can't support SD if the board does not support SD (duh) */

#  if defined(CONFIG_EA3131_PAGING_SDSLOT) && !defined(HAVE_SD)
#    error "This board does not support SD"
#    undef CONFIG_EA3131_PAGING_SDSLOT
#  endif

  /* Can't support SD if mountpoints are disabled or if SDIO support
   * is not enabled.
   */

#  if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_LPC31_MCI)
#    ifdef CONFIG_EA3131_PAGING_SDSLOT
#      error "Mountpoints and/or MCI disabled"
#    endif
#    undef CONFIG_EA3131_PAGING_SDSLOT
#    undef HAVE_SD
#  endif

  /* A mountpoint for the FAT file system must be provided */

#  if !defined(CONFIG_EA3131_PAGING_MOUNTPT) && defined(CONFIG_EA3131_PAGING_SDSLOT)
#    error "No CONFIG_EA3131_PAGING_MOUNTPT provided"
#    undef CONFIG_EA3131_PAGING_SDSLOT
#    undef HAVE_SD
#  endif

  /* If no minor number is provided, default to zero */

#  ifndef CONFIG_EA3131_PAGING_MINOR
#    define CONFIG_EA3131_PAGING_MINOR 0
#  endif

#endif /* CONFIG_PAGING_BINPATH */

/* Are we accessing the page source data through the M25P* MTD device? */

#if defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)

  /* Verify that SPI support is enabld */

#ifndef CONFIG_LPC31_SPI
#  error "SPI support is not enabled"
#endif

  /* Make sure that some value is defined for the offset into the FLASH
   * of the NuttX binary image.
   */

#  ifndef CONFIG_EA3131_PAGING_BINOFFSET
#    define CONFIG_EA3131_PAGING_BINOFFSET 0
#  endif

  /* Make sure that some value is defined for the SPI port number */

#  ifndef CONFIG_EA3131_PAGING_SPIPORT
#    define CONFIG_EA3131_PAGING_SPIPORT 0
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State structure needed to support paging from a file */

#ifdef CONFIG_PAGING_BINPATH
struct pg_source_s
{
  bool initialized;  /* TRUE: we are initialized */
  struct file file;  /* File descriptor of the nuttx.bin file */
};
#endif

/* State structured support paging through the M25P* MTD interface. */

#if defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
struct pg_source_s
{
  /* If interrupts/DMA are used, then we will have to defer initialization */

  bool initialized;  /* TRUE: we are initialized */

  /* This is the M25P* device state structure */

  FAR struct mtd_dev_s *mtd;

  /* This the device geometry */

#ifdef CONFIG_DEBUG_FEATURES
  FAR struct mtd_geometry_s geo;
#endif
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_PAGING_BINPATH) || defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
static struct pg_source_s g_pgsrc;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_initsrc()
 *
 * Description:
 *  Initialize the source device that will support paging.
 *  If BINPATH is defined, then it is the full path to a file on a mounted
 *  file system.  In this case initialization will be deferred until the
 *  first time that up_fillpage() is called.
 *
 ****************************************************************************/

#if defined(CONFIG_PAGING_BINPATH)
static inline void lpc31_initsrc(void)
{
#ifdef CONFIG_EA3131_PAGING_SDSLOT
  FAR struct sdio_dev_s *sdio;
  int ret;
#endif

  /* Are we already initialized? */

  if (!g_pgsrc.initialized)
    {
#ifdef CONFIG_EA3131_PAGING_SDSLOT
      char devname[16];
#endif

      pginfo("Initializing %s\n", CONFIG_PAGING_BINPATH);

      /* No, do we need to mount an SD device? */

#ifdef CONFIG_EA3131_PAGING_SDSLOT

      /* Yes.. First, get an instance of the SDIO interface */

      sdio = sdio_initialize(CONFIG_EA3131_PAGING_SDSLOT);
      DEBUGASSERT(sdio != NULL);

      /* Then bind the SDIO interface to the SD driver */

      ret = mmcsd_slotinitialize(CONFIG_EA3131_PAGING_MINOR, sdio);
      DEBUGASSERT(ret == OK);

      /* Then let's guess and say that there is a card in the slot.
       * (We are basically jodido anyway if there is no card in the slot).
       */

      sdio_mediachange(sdio, true);

      /* Now mount the file system */

      snprintf(devname, 16, "/dev/mmcsd%d", CONFIG_EA3131_PAGING_MINOR);
      ret = nx_mount(devname, CONFIG_EA3131_PAGING_MOUNTPT, "vfat",
                     MS_RDONLY, NULL);
      DEBUGASSERT(ret == OK);

#endif /* CONFIG_EA3131_PAGING_SDSLOT */

      /* Open the selected path for read-only access */

      file_open(&g_pgsrc.file, CONFIG_PAGING_BINPATH, O_RDONLY);
      DEBUGASSERT(g_pgsrc.file.f_inode != NULL);

      /* Then we are initialized */

      g_pgsrc.initialized = true;
    }
}

#elif defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
static inline void lpc31_initsrc(void)
{
  FAR struct spi_dev_s *spi;
#ifdef CONFIG_DEBUG_FEATURES
  uint32_t capacity;
  int ret;
#endif

  /* Are we already initialized? */

  if (!g_pgsrc.initialized)
    {
      /* No... the initialize now */

      pginfo("Initializing\n");

      /* First get an instance of the SPI device interface */

      spi = lpc31_spibus_initialize(CONFIG_EA3131_PAGING_SPIPORT);
      DEBUGASSERT(spi != NULL);

      /* Then bind the SPI interface to the MTD driver */

#ifdef CONFIG_PAGING_M25PX
      g_pgsrc.mtd = m25p_initialize(spi);
#else
      g_pgsrc.mtd = at45db_initialize(spi);
#endif
      DEBUGASSERT(g_pgsrc.mtd != NULL);

      /* Verify that we can use the device */

#ifdef CONFIG_DEBUG_FEATURES
      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(g_pgsrc.mtd, MTDIOC_GEOMETRY,
                     (unsigned long)&g_pgsrc.geo);
      DEBUGASSERT(ret >= 0);
      capacity = g_pgsrc.geo.erasesize*g_pgsrc.geo.neraseblocks;
      pginfo("capacity: %d\n", capacity);
      DEBUGASSERT(capacity >=
                  CONFIG_EA3131_PAGING_BINOFFSET + PG_TEXT_VSIZE);
#endif

      /* We are now initialized */

      g_pgsrc.initialized = true;
    }
}

#else
#  define lpc31_initsrc()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fillpage()
 *
 * Description:
 *  After a page is allocated and mapped by up_allocpage(), the actual
 *  filling of the page with data from the non-volatile, must be performed
 *  by a separate call to the architecture-specific function, up_fillpage().
 *  This function is non-blocking, it will start an asynchronous page fill.
 *  The common paging logic will provide a callback function, pg_callback,
 *  that will be called when the page fill is finished (or an error occurs).
 *  This callback is assumed to occur from an interrupt level when the
 *  device driver completes the fill operation.
 *
 *  NOTE 1:
 *  Allocating and filling a page is a two step process.  up_allocpage()
 *  allocates the page, and up_fillpage() fills it with data from some non-
 *  volatile storage device.  This distinction is made because up_allocpage()
 *  can probably be implemented in board-independent logic whereas
 *  up_fillpage() probably must be implemented as board-specific logic.
 *
 *  NOTE 2: The initial mapping of vpage will be read-able, write-able,
 *  but non-cacheable.  No special actions will be required of
 *  up_fillpage() in order to write into this allocated page.  If the
 *  virtual address maps to a text region, however, this function should
 *  remap the region so that is is read/execute only.  It should be made
 *  cache-able in any case.

 * Input Parameters:
 *   tcb - A reference to the task control block of the task that needs to
 *         have a page fill.  Architecture-specific logic can retrieve page
 *         fault information from the architecture-specific context
 *         information in this TCB to perform the fill.
 *   pg_callbck - The function to be called when the page fill is complete.
 *
 * Returned Value:
 *   This function will return zero (OK) if the page fill was successfully
 *   started (the result of the page fill is passed to the callback function
 *   as the result argument).  A negated errno value may be returned if an
 *   error occurs.  All errors, however, are fatal.
 *
 *   NOTE: -EBUSY has a special meaning. It is used internally to mean that
 *   the callback function has not executed.  Therefore, -EBUSY should
 *   never be provided in the result argument of pg_callback.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but
 *     interrupts siabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *   - Upon return, the caller will sleep waiting for the page fill callback
 *     to occur.  The callback function will perform the wakeup.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING_BLOCKINGFILL

/* Version 1:  Supports blocking fill operations */

int up_fillpage(FAR struct tcb_s *tcb, FAR void *vpage)
{
#if defined(CONFIG_PAGING_BINPATH)
  ssize_t nbytes;
  off_t   offset;
  off_t   pos;
#elif defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
  ssize_t nbytes;
  off_t   offset;
#endif

  pginfo("TCB: %p vpage: %p far: %08x\n", tcb, vpage, tcb->xcp.far);
  DEBUGASSERT(tcb->xcp.far >= PG_PAGED_VBASE &&
              tcb->xcp.far < PG_PAGED_VEND);

  /* If BINPATH is defined, then it is the full path to a file on a mounted
   * file system.  In this case initialization will be deferred until the
   * first time that up_fillpage() is called.  Are we initialized?
   */

#if defined(CONFIG_PAGING_BINPATH)

  /* Perform initialization of the paging source device (if necessary) */

  lpc31_initsrc();

  /* Create an offset into the binary image that corresponds to the
   * virtual address.   File offset 0 corresponds to PG_LOCKED_VBASE.
   */

  offset = (off_t)tcb->xcp.far - PG_LOCKED_VBASE;

  /* Seek to that position */

  pos = file_seek(&g_pgsrc.file, offset, SEEK_SET);
  DEBUGASSERT(pos != (off_t)-1);

  /* And read the page data from that offset */

  nbytes = file_read(&g_pgsrc.file, vpage, PAGESIZE);
  DEBUGASSERT(nbytes == PAGESIZE);
  return OK;

#elif defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB) /* !CONFIG_PAGING_BINPATH */

  /* Perform initialization of the paging source device (if necessary) */

  lpc31_initsrc();

  /* Create an offset into the binary image that corresponds to the
   * virtual address.   File offset 0 corresponds to PG_LOCKED_VBASE.
   */

  offset = tcb->xcp.far - PG_LOCKED_VBASE + CONFIG_EA3131_PAGING_BINOFFSET;

  /* Read the page at the correct offset into the SPI FLASH device */

  nbytes = MTD_READ(g_pgsrc.mtd, offset, PAGESIZE, (FAR uint8_t *)vpage);
  DEBUGASSERT(nbytes == PAGESIZE);
  return OK;

#else /* !CONFIG_PAGING_BINPATH && !CONFIG_PAGING_M25PX && !CONFIG_PAGING_AT45DB */

# warning "Not implemented"
  return -ENOSYS;

#endif /* !CONFIG_PAGING_BINPATH && !CONFIG_PAGING_M25PX && !CONFIG_PAGING_AT45DB */
}

#else /* CONFIG_PAGING_BLOCKINGFILL */

/* Version 2:  Supports non-blocking, asynchronous fill operations */

int up_fillpage(FAR struct tcb_s *tcb, FAR void *vpage,
                up_pgcallback_t pg_callback)
{
  pginfo("TCB: %p vpage: %d far: %08x\n", tcb, vpage, tcb->xcp.far);
  DEBUGASSERT(tcb->xcp.far >= PG_PAGED_VBASE &&
              tcb->xcp.far < PG_PAGED_VEND);

#if defined(CONFIG_PAGING_BINPATH)
#  error "File system-based paging must always be implemented with blocking calls"
#elif defined(CONFIG_PAGING_M25PX) || defined(CONFIG_PAGING_AT45DB)
#  error "SPI FLASH paging must always be implemented with blocking calls"
#else
#  warning "Not implemented"
#endif

  return -ENOSYS;
}

#endif /* CONFIG_PAGING_BLOCKINGFILL */

/****************************************************************************
 * Name: lpc31_pginitialize
 *
 * Description:
 *   Set up mass storage device to support on demand paging.
 *
 ****************************************************************************/

void weak_function lpc31_pginitialize(void)
{
  /* This initialization does nothing in this example setup.
   * But this function is where you might, for example:
   *
   * - Initialize and configure a mass storage device to support on-demand
   *   paging. This might be, perhaps an SD card or NAND memory.
   *   An SPI FLASH would probably already have been configured by
   *   lpc31_spidev_initialize(void);
   * - Set up resources to support up_fillpage() operation.  For example,
   *   perhaps the text image is stored in a named binary file.
   *   In this case, the virtual text addresses might map to offsets into
   *   that file.
   * - Do whatever else is necessary to make up_fillpage() ready for the
   *   first time that it is called.
   *
   * In reality, however, this function is not very useful:
   * This function is called from a low level (before nx_start() is even
   * called), it may not be possible to perform file system operations or
   * even to get debug output yet.  Therefore, to keep life simple,
   * initialization will be deferred in all cases until the first time that
   * up_fillpage() is called.
   */
}

#endif /* CONFIG_PAGING */
