/****************************************************************************
 * arch/arm/src/lm/lm_flash.c
 *
 *   Copyright (c) 2013 Max Holtzberg. All rights reserved.
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *
 *   Authors: Max Holtzberg <mh@uvc.de>
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * This code is derived from drivers/mtd/skeleton.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>

#include "up_arch.h"
#include "chip.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LM_VIRTUAL_NPAGES (LM_FLASH_NPAGES - CONFIG_LM_FLASH_STARTPAGE)
#define LM_VIRTUAL_BASE   (LM_FLASH_BASE \
                           + CONFIG_LM_FLASH_STARTPAGE * LM_FLASH_PAGESIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct lm_dev_s.
 */

struct lm_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int lm_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t lm_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                        FAR uint8_t *buf);
static ssize_t lm_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                         FAR const uint8_t *buf);
static ssize_t lm_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                       FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t lm_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                        FAR const uint8_t *buf);
#endif
static int lm_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This structure holds the state of the MTD driver */

static struct lm_dev_s g_lmdev =
{
  { lm_erase,
    lm_bread,
    lm_bwrite,
    lm_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    lm_write,
#endif
    lm_ioctl
  },
  /* Initialization of any other implementation specific data goes here */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int lm_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                    size_t nblocks)
{
  int curpage;
  uint32_t pageaddr;

  DEBUGASSERT(nblocks <= LM_VIRTUAL_NPAGES);

  for (curpage = startblock; curpage < nblocks; curpage++)
    {
      pageaddr = LM_VIRTUAL_BASE + curpage * LM_FLASH_PAGESIZE;

      fvdbg("Erase page at %08x\n", pageaddr);

      /* set page address */

      putreg32((pageaddr << FLASH_FMA_OFFSET_SHIFT) & FLASH_FMA_OFFSET_MASK,
               LM_FLASH_FMA);

      /* set flash write key and erase bit */

      putreg32(FLASH_FMC_WRKEY | FLASH_FMC_ERASE, LM_FLASH_FMC);

      /* wait until erase has finished */

      while (getreg32(LM_FLASH_FMC) & FLASH_FMC_ERASE);
    }

  return OK;
}

/****************************************************************************
 * Name: lm_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t lm_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                        FAR uint8_t *buf)
{
  DEBUGASSERT(startblock + nblocks <= LM_VIRTUAL_NPAGES);

  memcpy(buf, (void*)(LM_VIRTUAL_BASE + startblock * LM_FLASH_PAGESIZE),
         nblocks * LM_FLASH_PAGESIZE);

  return nblocks;
}

/****************************************************************************
 * Name: lm_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t lm_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                         FAR const uint8_t *buf)
{
  FAR uint32_t *src = (uint32_t*)buf;
  FAR uint32_t *dst = (uint32_t*)(LM_VIRTUAL_BASE + startblock * LM_FLASH_PAGESIZE);
  int i;

  DEBUGASSERT(nblocks <= LM_VIRTUAL_NPAGES);

  for (i = 0; i < (nblocks * LM_FLASH_PAGESIZE) >> 2; i++)
    {
      /* set data to write  */

      putreg32(*src++, LM_FLASH_FMD);

      /* set destination address */

      putreg32((uint32_t)dst++, LM_FLASH_FMA);

      /* start write */

      putreg32(FLASH_FMC_WRKEY | FLASH_FMC_WRITE, LM_FLASH_FMC);

      /* wait until write has finished */

      while(getreg32(LM_FLASH_FMC) & FLASH_FMC_WRITE);
    }

  return nblocks;
}

/****************************************************************************
 * Name: lm_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t lm_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                       FAR uint8_t *buf)
{
  DEBUGASSERT(offset + nbytes < LM_VIRTUAL_NPAGES * LM_FLASH_PAGESIZE);

  memcpy(buf, (void*)(LM_VIRTUAL_BASE + offset), nbytes);

  return nbytes;
}

/****************************************************************************
 * Name: lm_write
 *
 * Description:
 *   Some FLASH parts have the ability to write an arbitrary number of
 *   bytes to an arbitrary offset on the device.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t lm_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                        FAR const uint8_t *buf)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: lm_ioctl
 ****************************************************************************/

static int lm_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = LM_FLASH_PAGESIZE;  /* Size of one read/write block */
              geo->erasesize    = LM_FLASH_PAGESIZE;  /* Size of one erase block */
              geo->neraseblocks = LM_VIRTUAL_NPAGES;
              ret               = OK;
          }
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;

          if (ppv)
            {
              /* If media is directly acccesible, return (void*) base address
               * of device memory.  NULL otherwise.  It is acceptable to omit
               * this case altogether and simply return -ENOTTY.
               */

              *ppv = (void*)LM_VIRTUAL_BASE;
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          lm_erase(dev, 0, LM_VIRTUAL_NPAGES);

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *up_flashinitialize(void)
{
  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_lmdev;
}
