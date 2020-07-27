/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sfc.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <nuttx/arch.h>
#include <nuttx/mtd/mtd.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

/* Prototypes for Remote API */

int fw_fm_rawwrite(uint32_t offset, const void *buf, uint32_t size);
int fw_fm_rawverifywrite(uint32_t offset, const void *buf, uint32_t size);
int fw_fm_rawread(uint32_t offset, void *buf, uint32_t size);
int fw_fm_rawerasesector(uint32_t sector);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_SPIFLASHSIZE
#  define CONFIG_CXD56_SPIFLASHSIZE (16 * 1024 * 1024)
#endif

#define SECTOR_SHIFT 12
#define SECTOR_SIZE (1 << SECTOR_SHIFT)

#ifdef CONFIG_CXD56_SFC_PAGE_SHIFT_SIZE
#  define PAGE_SHIFT CONFIG_CXD56_SFC_PAGE_SHIFT_SIZE
#else
#  define PAGE_SHIFT 12
#endif
#define PAGE_SIZE (1 << PAGE_SHIFT)

/* Flash device information */

struct flash_controller_s
{
  struct mtd_dev_s mtd; /* MTD interface */
  uint32_t density;
};

static struct flash_controller_s g_sfc;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_erase
 ****************************************************************************/

static int cxd56_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks)
{
  int ret;
  size_t i;

  finfo("erase: %08lx (%u blocks)\n", startblock << PAGE_SHIFT, nblocks);

  for (i = 0; i < nblocks; i++)
    {
      ret = fw_fm_rawerasesector(startblock + i);
      if (ret < 0)
        {
          set_errno(-ret);
          return ERROR;
        }
    }

  return OK;
}

static ssize_t cxd56_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer)
{
  int ret;

  finfo("bread: %08lx (%u blocks)\n", startblock << PAGE_SHIFT, nblocks);

  ret = fw_fm_rawread(startblock << PAGE_SHIFT, buffer,
                      nblocks << PAGE_SHIFT);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return nblocks;
}

static ssize_t cxd56_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buffer)
{
  int ret;

  finfo("bwrite: %08lx (%u blocks)\n", startblock << PAGE_SHIFT, nblocks);

#ifdef CONFIG_CXD56_SFC_VERIFY_WRITE
  ret = fw_fm_rawverifywrite(startblock << PAGE_SHIFT, buffer,
                          nblocks << PAGE_SHIFT);
#else
  ret = fw_fm_rawwrite(startblock << PAGE_SHIFT, buffer,
                    nblocks << PAGE_SHIFT);
#endif
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return nblocks;
}

static ssize_t cxd56_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buffer)
{
  int ret;

  finfo("read: %08lx (%u bytes)\n", offset, nbytes);

  ret = fw_fm_rawread(offset, buffer, nbytes);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return nbytes;
}

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t cxd56_write(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR const uint8_t *buffer)
{
  int ret;

  finfo("write: %08lx (%u bytes)\n", offset, nbytes);

#ifdef CONFIG_CXD56_SFC_VERIFY_WRITE
  ret = fw_fm_rawverifywrite(offset, buffer, nbytes);
#else
  ret = fw_fm_rawwrite(offset, buffer, nbytes);
#endif
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return nbytes;
}
#endif

static int cxd56_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct flash_controller_s *priv = (struct flash_controller_s *)dev;
  int ret                         = OK;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          finfo("cmd: GEOM\n");
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just
               * an array of fixed size blocks.
               * That is most likely not true, but the client will expect
               * the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = PAGE_SIZE;
              geo->erasesize    = SECTOR_SIZE;
              geo->neraseblocks = priv->density >> SECTOR_SHIFT;
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          uint32_t sec  = 0;
          uint32_t last = priv->density >> SECTOR_SHIFT;

          finfo("cmd: ERASE\n");

          while (sec < last)
            {
              fw_fm_rawerasesector(sec);
              sec++;
            }
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

FAR struct mtd_dev_s *cxd56_sfc_initialize(void)
{
  struct flash_controller_s *priv = &g_sfc;

  priv->mtd.erase  = cxd56_erase;
  priv->mtd.bread  = cxd56_bread;
  priv->mtd.bwrite = cxd56_bwrite;
  priv->mtd.read   = cxd56_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = cxd56_write;
#endif
  priv->mtd.ioctl  = cxd56_ioctl;

  /* TODO: Flash reserved area should be configurable dynamically. */

  priv->density = CONFIG_CXD56_SPIFLASHSIZE;

#ifdef CONFIG_SFC_SECTOR512

  /* Allocate a buffer for the erase block cache */

  priv->cache = (FAR uint8_t *)kmm_malloc(SPIFI_BLKSIZE);
  if (!priv->cache)
    {
      /* Allocation failed! */

      /* Discard all of that work we just did and return NULL */

      ferr("ERROR: Allocation failed\n");
      return NULL;
    }
#endif

  return &priv->mtd;
}
