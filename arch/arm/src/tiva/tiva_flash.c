/****************************************************************************
 * arch/arm/src/tiva/tiva_flash.c
 *
 *   Copyright (c) 2013 Max Holtzberg. All rights reserved.
 *   Copyright (C) 2013, 2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/mtd/mtd.h>

#include "up_arch.h"
#include "chip.h"

#include "tiva_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIVA_VIRTUAL_NPAGES (TIVA_FLASH_NPAGES - CONFIG_TIVA_FLASH_STARTPAGE)
#define TIVA_VIRTUAL_OFFSET (CONFIG_TIVA_FLASH_STARTPAGE * TIVA_FLASH_PAGESIZE)
#define TIVA_VIRTUAL_BASE   (TIVA_FLASH_BASE + TIVA_VIRTUAL_OFFSET)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct tiva_dev_s.
 */

struct tiva_dev_s
{
  struct mtd_dev_s mtd;

  /* Other implementation specific data may follow here */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int tiva_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks);
static ssize_t tiva_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t tiva_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t tiva_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_write(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR const uint8_t *buf);
#endif
static int tiva_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This structure holds the state of the MTD driver */

static struct tiva_dev_s g_lmdev =
{
  {
    tiva_erase,
    tiva_bread,
    tiva_bwrite,
    tiva_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    tiva_write,
#endif
    tiva_ioctl
  },

  /* Initialization of any other implementation specific data goes here */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int tiva_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  off_t endblock;
  uint32_t pageaddr;
  uint32_t regval;
  int curpage;

  endblock = startblock + nblocks;
  DEBUGASSERT(endblock <= TIVA_VIRTUAL_NPAGES);

  for (curpage = startblock; curpage < endblock; curpage++)
    {
      pageaddr = TIVA_VIRTUAL_BASE + curpage * TIVA_FLASH_PAGESIZE;

      finfo("Erase page at %08x\n", pageaddr);

      /* set page address */

      putreg32((pageaddr << FLASH_FMA_OFFSET_SHIFT) & FLASH_FMA_OFFSET_MASK,
               TIVA_FLASH_FMA);

      /* set flash write key and erase bit */

      putreg32(FLASH_FMC_WRKEY | FLASH_FMC_ERASE, TIVA_FLASH_FMC);

      /* wait until erase has finished */

      while (getreg32(TIVA_FLASH_FMC) & FLASH_FMC_ERASE)
        {
        }

      /* Return an error if an access violation or erase error occurred. */

      regval  = getreg32(TIVA_FLASH_FCRIS);
      regval &= (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS | FLASH_FCRIS_ERRIS);

      if (regval != 0)
        {
           return -EACCES;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tiva_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t tiva_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf)
{
  DEBUGASSERT(startblock + nblocks <= TIVA_VIRTUAL_NPAGES);

  memcpy(buf, (void *)(TIVA_VIRTUAL_BASE + startblock * TIVA_FLASH_PAGESIZE),
         nblocks * TIVA_FLASH_PAGESIZE);

  return nblocks;
}

/****************************************************************************
 * Name: tiva_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t tiva_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buf)
{
  FAR uint32_t *src = (uint32_t *)buf;
  FAR uint32_t *dst = (uint32_t *)(TIVA_VIRTUAL_BASE + startblock * TIVA_FLASH_PAGESIZE);
  int i;

  DEBUGASSERT(nblocks <= TIVA_VIRTUAL_NPAGES);

  for (i = 0; i < (nblocks * TIVA_FLASH_PAGESIZE) >> 2; i++)
    {
      /* set data to write  */

      putreg32(*src++, TIVA_FLASH_FMD);

      /* set destination address */

      putreg32((uint32_t)dst++, TIVA_FLASH_FMA);

      /* start write */

      putreg32(FLASH_FMC_WRKEY | FLASH_FMC_WRITE, TIVA_FLASH_FMC);

      /* wait until write has finished */

      while (getreg32(TIVA_FLASH_FMC) & FLASH_FMC_WRITE);
    }

  return nblocks;
}

/****************************************************************************
 * Name: tiva_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t tiva_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buf)
{
  DEBUGASSERT(offset + nbytes < TIVA_VIRTUAL_NPAGES * TIVA_FLASH_PAGESIZE);

  memcpy(buf, (void *)(TIVA_VIRTUAL_BASE + offset), nbytes);

  return nbytes;
}

/****************************************************************************
 * Name: tiva_write
 *
 * Description:
 *   Some FLASH parts have the ability to write an arbitrary number of
 *   bytes to an arbitrary offset on the device.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buf)
{
  FAR const uint32_t *src = (uint32_t *)((uintptr_t)buf & ~3);
  ssize_t remaining;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && buf != NULL);
  DEBUGASSERT(((uintptr_t)buf & 3) == 0 && (offset & 3) == 0 && (nbytes && 3) == 0);

  /* Clear the flash access and error interrupts. */

   putreg32(FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC | FLASH_FCMISC_ERMISC,
            TIVA_FLASH_FCMISC);

  /* Adjust the offset to the start of the partition.
   * REVISIT:  If we really wanted to gracefully handle unaligned addresses,
   * offsets, and sizes we would have to do a little more than this.
   */

  offset   &= ~3;
  offset   += TIVA_VIRTUAL_OFFSET;
  nbytes   &= ~3;
  remaining = nbytes;

#if defined(CONFIG_ARCH_CHIP_TM4C1294NC)
  while (remaining > 0)
    {
      /* Set the address of this block of words. */

      putreg32((offset & ~(0x7f)), TIVA_FLASH_FMA);

      /* Loop over the words in this 32-word block. */

      while(((offset & 0x7c) || (getreg32(TIVA_FLASH_FWBN) == 0)) &&
            (remaining > 0))
        {
          /* Write this word into the write buffer. */

          putreg32(*src++, (TIVA_FLASH_FWBN + (offset & 0x7c)));
          offset    += 4;
          remaining -= 4;
        }

      /* Program the contents of the write buffer into flash. */

       putreg32(FLASH_FMC_WRKEY | FLASH_FMC2_WRBUF, TIVA_FLASH_FMC2);

       /* Wait until the write buffer has been programmed. */

       while(getreg32(TIVA_FLASH_FMC2) & FLASH_FMC2_WRBUF)
         {
         }
     }
#else
  while (remaining > 0)
    {
      /* Program the next word. */

       putreg32(offset, TIVA_FLASH_FMA);
       putreg32(*src, TIVA_FLASH_FMD);
       putreg32(FLASH_FMC_WRKEY | FLASH_FMC_WRITE, TIVA_FLASH_FMC);

       /* Wait until the word has been programmed. */

       while(getreg32(TIVA_FLASH_FMC) & FLASH_FMC_WRITE);

       /* Increment to the next word. */

       src++;
       offset    += 4;
       remaining -= 4;
    }
#endif

  /* Return an error if an access violation or erase error occurred. */

  regval  = getreg32(TIVA_FLASH_FCRIS);
  regval &= (FLASH_FCRIS_ARIS | FLASH_FCRIS_VOLTRIS | FLASH_FCRIS_INVDRIS |
             FLASH_FCRIS_PROGRIS);

  if (regval != 0)
    {
       return -EACCES;
    }

  return nbytes;
}
#endif

/****************************************************************************
 * Name: tiva_ioctl
 ****************************************************************************/

static int tiva_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
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

              geo->blocksize    = TIVA_FLASH_PAGESIZE;  /* Size of one read/write block */
              geo->erasesize    = TIVA_FLASH_PAGESIZE;  /* Size of one erase block */
              geo->neraseblocks = TIVA_VIRTUAL_NPAGES;
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

              *ppv = (void *)TIVA_VIRTUAL_BASE;
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          tiva_erase(dev, 0, TIVA_VIRTUAL_NPAGES);

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
 * Name: tiva_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *tiva_flash_initialize(void)
{
  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_lmdev;
}
