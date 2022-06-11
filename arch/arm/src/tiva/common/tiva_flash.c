/****************************************************************************
 * arch/arm/src/tiva/common/tiva_flash.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "arm_internal.h"
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

static int tiva_erase(struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks);
static ssize_t tiva_bread(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, uint8_t *buf);
static ssize_t tiva_bwrite(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, const uint8_t *buf);
static ssize_t tiva_read(struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_write(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, const uint8_t *buf);
#endif
static int tiva_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg);

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

static int tiva_erase(struct mtd_dev_s *dev, off_t startblock,
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

static ssize_t tiva_bread(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, uint8_t *buf)
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

static ssize_t tiva_bwrite(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, const uint8_t *buf)
{
  uint32_t *src = (uint32_t *)buf;
  uint32_t *dst = (uint32_t *)(TIVA_VIRTUAL_BASE +
                                   startblock * TIVA_FLASH_PAGESIZE);
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

static ssize_t tiva_read(struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, uint8_t *buf)
{
  DEBUGASSERT(offset + nbytes < TIVA_VIRTUAL_NPAGES * TIVA_FLASH_PAGESIZE);

  memcpy(buf, (void *)(TIVA_VIRTUAL_BASE + offset), nbytes);

  return nbytes;
}

/****************************************************************************
 * Name: tiva_write
 *
 * Description:
 *   Write a block of data to FLASH memory.
 *
 * Input Parameters:
 *   dev: Device representing Tiva FLASH.
 *   offset: Destination offset in FLASH memory.
 *   nbytes: Number of bytes to write.
 *   buf: Source buffer containing data to be written.
 *
 * Returned Value:
 *   Number of bytes written to FLASH.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t tiva_write(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, const uint8_t *buf)
{
  /* WARNING and REVISIT:
   * Because this function exports a byte write interface and
   * is conditioned upon CONFIG_MTD_BYTE_WRITE, it is supposed
   * to support unaligned writes and writes of arbitrary byte
   * counts. But it doesn't. This needs to be fixed!
   */

  const uint32_t *src = (uint32_t *)((uintptr_t)buf & ~3);
  ssize_t remaining;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && buf != NULL);
  DEBUGASSERT(((uintptr_t)buf & 3) == 0 && (offset & 3) == 0 &&
              (nbytes & 3) == 0);

  /* Clear the flash access and error interrupts. */

  putreg32(FLASH_FCMISC_AMISC | FLASH_FCMISC_VOLTMISC | FLASH_FCMISC_ERMISC,
           TIVA_FLASH_FCMISC);

  /* Adjust the offset to the start of the partition. */

  offset   &= ~3;
  offset   += TIVA_VIRTUAL_OFFSET;
  nbytes   &= ~3;
  remaining = nbytes;

#if defined (CONFIG_ARCH_CHIP_TM4C123) || defined (CONFIG_ARCH_CHIP_TM4C129)
  /* TM4C123x and TM4C129x parts have a 32-word FLASH memory write buffer,
   * allowing faster writes by writing 32 words in the time it would take
   * to write 16.
   */

  while (remaining > 0)
    {
      /* Set the address of this block of words. */

      putreg32((offset & ~(0x7f)), TIVA_FLASH_FMA);

      /* Loop over the words in this 32-word block. */

      while (((offset & 0x7c) || (getreg32(TIVA_FLASH_FWBN) == 0)) &&
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

      while (getreg32(TIVA_FLASH_FMC2) & FLASH_FMC2_WRBUF)
        {
        }
    }
#else
  /* These parts do not have the 32-word FLASH memory write buffer, so
   * we must do the slower 1-word-at-a-time write.
   */

  while (remaining > 0)
    {
      /* Program the next word. */

      putreg32(offset, TIVA_FLASH_FMA);
      putreg32(*src, TIVA_FLASH_FMD);
      putreg32(FLASH_FMC_WRKEY | FLASH_FMC_WRITE, TIVA_FLASH_FMC);

      /* Wait until the word has been programmed. */

      while (getreg32(TIVA_FLASH_FMC) & FLASH_FMC_WRITE);

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

static int tiva_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to
               * know the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an
               * array of fixed size blocks.  That is most likely not true,
               * but the client will expect the device logic to do whatever
               * is necessary to make it appear so.
               */

              geo->blocksize    = TIVA_FLASH_PAGESIZE;  /* Size of one read/write block */
              geo->erasesize    = TIVA_FLASH_PAGESIZE;  /* Size of one erase block */
              geo->neraseblocks = TIVA_VIRTUAL_NPAGES;
              ret               = OK;
          }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info =
            (struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = TIVA_VIRTUAL_NPAGES;
              info->sectorsize  = TIVA_FLASH_PAGESIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case BIOC_XIPBASE:
        {
          void **ppv = (void**)arg;

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

struct mtd_dev_s *tiva_flash_initialize(void)
{
  /* Return the implementation-specific state structure as the MTD device */

  return (struct mtd_dev_s *)&g_lmdev;
}
