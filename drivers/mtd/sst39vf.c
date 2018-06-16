/****************************************************************************
 * drivers/mtd/sst39vf.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration */

#ifndef CONFIG_SST39VF_BASE_ADDRESS
#  error "The FLASH base address was not provided (CONFIG_SST39VF_BASE_ADDRESS)"
#endif

/* MAP SST39VF address to a 16-bit bus address */

#define SST39VF_ADDR(addr) \
  (volatile FAR uint16_t *)(CONFIG_SST39VF_BASE_ADDRESS | (addr << 1))

/* Timing */

#define SST39VF_TBP_USEC    10  /* Word-Program Time (max); 7uS typical */
#define SST39VF_TIDA_NSEC   150 /* Software ID Access and Exit Time (max) */
#define SST39VF_TSE_MSEC    25  /* Sector-Erase 25 ms (max); 18 ms typical */
#define SST39VF_TBE_MSEC    25  /* Block-Erase 25 ms (max); 18 ms typical */
#define SST39VF_TSCE_MSEC   50  /* Chip-Erase 50 ms (max);  */

#define WORDWRITE_TIMEOUT   0x080000000

/* IDs */

#define SST_MANUFACTURER_ID 0xbf

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This describes one chip in the SST39VF family */

struct sst39vf_chip_s
{
#if 0 /* Not used */
  bool     top;        /* Top protect SST39VF1602/3202 */
#endif
  uint16_t chipid;     /* ID of the chip */
#if 0 /* Not used */
  uint16_t nblocks;    /* Number of erase blocks */
#endif
  uint16_t nsectors;   /* Number of erase-ablesectors */
#if 0 /* Not used */
  uint32_t blocksize;  /* Size of one erase block */
#endif
  uint32_t sectorsize; /* Size of one sector */
};

/* This type holds one FLASH address and one 16-bit FLASH data value */

struct sst39vf_wrinfo_s
{
  uintptr_t address;
  uint16_t data;
};

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct sst39vf_dev_s.
 */

struct sst39vf_dev_s
{
  struct mtd_dev_s mtd;
  FAR const struct sst39vf_chip_s *chip;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Low Level Helpers */

static inline void sst39vf_flashwrite(FAR const struct sst39vf_wrinfo_s *wrinfo);
static inline uint16_t sst39vf_flashread(uintptr_t address);
static void sst39vf_writeseq(FAR const struct sst39vf_wrinfo_s *wrinfo, int nseq);
static int sst39vf_chiperase(FAR struct sst39vf_dev_s *priv);
static int sst39vf_sectorerase(FAR struct sst39vf_dev_s *priv,
                               uintptr_t sectaddr);
static int sst39vf_writeword(FAR const struct sst39vf_wrinfo_s *wrinfo);

/* MTD driver methods */

static int sst39vf_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks);
static ssize_t sst39vf_bread(FAR struct mtd_dev_s *dev,
                              off_t startblock, size_t nblocks,
                            FAR uint8_t *buf);
static ssize_t sst39vf_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf);
static ssize_t sst39vf_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer);
static int sst39vf_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sst39vf_chip_s g_sst39vf1601 =
{
  /* false,                    top - Bottom hardware block protection */
  0x234b,                   /* chipid */
  /* 32,                       nblocks */
  512,                      /* nsectors */
  /* 64*1024,                  blocksize */
  4*1024,                   /* sectorsize */
};

static const struct sst39vf_chip_s g_sst39vf1602 =
{
  /* true,                     top - Top hardware block protection */
  0x234a,                   /* chipid */
  /* 32,                       nblocks */
  512,                      /* nsectors */
  /* 64*1024,                  blocksize */
  4*1024,                   /* sectorsize */
};

static const struct sst39vf_chip_s g_sst39vf3201 =
{
  /* false,                    top - Bottom hardware block protection */
  0x235b,                   /* chipid */
  /* 64,                       nblocks */
  1024,                     /* nsectors */
  /* 64*1024,                  blocksize */
  4*1024,                   /* sectorsize */
};

static const struct sst39vf_chip_s g_sst39vf3202 =
{
  /* true,                     top - Top hardware block protection */
  0x235a,                   /* chipid */
  /* 64,                       nblocks */
  1024,                     /* nsectors */
  /* 64*1024,                  blocksize */
  4*1024,                   /* sectorsize */
};

/* This structure holds the state of the MTD driver */

static struct sst39vf_dev_s g_sst39vf =
{
  {
    sst39vf_erase,          /* erase method */
    sst39vf_bread,          /* bread method */
    sst39vf_bwrite,         /* bwrte method */
    sst39vf_read,           /* read method */
#ifdef CONFIG_MTD_BYTE_WRITE
    NULL,                   /* write method */
#endif
    sst39vf_ioctl           /* ioctl method */
  },
  NULL                      /* Chip */
};

/* Command sequences */

static const struct sst39vf_wrinfo_s g_wordprogram[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055}, {0x5555, 0x00a0} /* , {address, data} */
};

static const struct sst39vf_wrinfo_s g_sectorerase[5] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055}, {0x5555, 0x0080},
  {0x5555, 0x00aa}, {0x2aaa, 0x0055} /* , {sector, 0x0030} */
};

#if 0 /* Not used */
static const struct sst39vf_wrinfo_s g_blockerase[5] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055}, {0x5555, 0x80},
  {0x5555, 0x00aa}, {0x2aaa, 0x0055} /* , {block, 0x0050} */
};
#endif

static const struct sst39vf_wrinfo_s g_chiperase[6] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055}, {0x5555, 0x0080},
  {0x5555, 0x00aa}, {0x2aaa, 0x0055}, {0x5555, 0x0010}
};

#if 0 /* Not used */
static const struct sst39vf_wrinfo_s g_erasesuspend[1] =
{
  {0x5555, 0x00aa}
};

static const struct sst39vf_wrinfo_s g_eraseresume[1] =
{
  {0x5555, 0x00aa}
};

static const struct sst39vf_wrinfo_s g_querysecid[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x0088}
};

static const struct sst39vf_wrinfo_s g_securityid_wordprogram[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x00a5}, /* {address, data} */
};

static const struct sst39vf_wrinfo_s g_securityid_lockout[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x0085} /* {0xXX, 0x0000} */
};
#endif

static const struct sst39vf_wrinfo_s g_swid_entry[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x0090}
};

#if 0 /* Not used */
static const struct sst39vf_wrinfo_s g_cfiquery[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x0080},
};
#endif

static const struct sst39vf_wrinfo_s g_swid_exit[3] =
{
  {0x5555, 0x00aa}, {0x2aaa, 0x0055},  {0x5555, 0x00f0}
};

#if 0 /* Not used */
static const struct sst39vf_wrinfo_s g_swid_exit2[1] =
{
  {0x0000, 0x00f0},
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sst39vf_flashwrite
 *
 * Description:
 *   Write one value to FLASH
 *
 ****************************************************************************/

static inline void sst39vf_flashwrite(FAR const struct sst39vf_wrinfo_s *wrinfo)
{
  volatile uint16_t *addr = SST39VF_ADDR(wrinfo->address);
  *addr = wrinfo->data;
}

/****************************************************************************
 * Name: sst39vf_flashread
 *
 * Description:
 *   Read one value from FLASH
 *
 ****************************************************************************/

static inline uint16_t sst39vf_flashread(uintptr_t address)
{
  return *SST39VF_ADDR(address);
}

/****************************************************************************
 * Name: sst39vf_writeseq
 *
 * Description:
 *   Write a sequence of values to FLASH
 *
 ****************************************************************************/

static void sst39vf_writeseq(FAR const struct sst39vf_wrinfo_s *wrinfo, int nseq)
{
  while (nseq--)
    {
      sst39vf_flashwrite(wrinfo);
      wrinfo++;
    }
}

/****************************************************************************
 * Name: sst39vf_checktoggle
 *
 * Description:
 *   Check for bit toggle
 *
 *   "Toggle Bits (DQ6 and DQ2). During the internal Program or Erase
 *    operation, any consecutive attempts to read DQ6 will produce
 *    alternating “1”s and “0”s, i.e., toggling between 1 and 0. When
 *    the internal Program or Erase operation is completed, the DQ6 bit
 *    will stop toggling. The device is then ready for the next operation.
 *    For Sector-, Block-, or Chip-Erase, the toggle bit (DQ6) is valid
 *    after the rising edge of sixth WE# (or CE#) pulse.  DQ6 will be set to
 *    “1” if a Read operation is attempted on an Erase-Suspended
 *    Sector/Block. If Program operation is initiated in a sector/block not
 *    selected in Erase-Suspend mode, DQ6 will toggle.
 *
 *   "An additional Toggle Bit is available on DQ2, which can be used in
 *    conjunction with DQ6 to check whether a particular sector is being
 *    actively erased or erase-suspended. ... The Toggle Bit (DQ2) is valid
 *    after the rising edge of the last WE# (or CE#) pulse of Write operation.
 *    ..."
 *
 ****************************************************************************/

static bool sst39vf_checktoggle(FAR const struct sst39vf_wrinfo_s *wrinfo)
{
  uint16_t value1;
  uint16_t value2;

  value1 = sst39vf_flashread(wrinfo->address);
  value2 = sst39vf_flashread(wrinfo->address);

  return (value1 == value2);
}

/****************************************************************************
 * Name: sst39vf_waittoggle
 *
 * Description:
 *   Wait until the data is no longer toggling.
 *
 ****************************************************************************/

static int sst39vf_waittoggle(FAR const struct sst39vf_wrinfo_s *wrinfo,
                              uint32_t retries)
{
  while (retries-- > 0)
    {
      if (sst39vf_checktoggle(wrinfo))
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: sst39vf_chiperase
 *
 * Description:
 *   Erase the entire chip
 *
 *   "The SST39VF160x/320x provide a Chip-Erase operation, which allows the
 *    user to erase the entire memory array to the “1” state. This is useful
 *    when the entire device must be quickly erased.  The Chip-Erase operation
 *    is initiated by executing a six-byte command sequence with Chip-Erase
 *    command (10H) at address 5555H in the last byte sequence. The Erase
 *    operation begins with the rising edge of the sixth WE# or CE#,
 *    whichever occurs first. During the Erase operation, the only valid
 *    read is Toggle Bit or Data# Polling... Any commands issued during the
 *    Chip-Erase operation are ignored. When WP# is low, any attempt to
 *    Chip-Erase will be ignored. During the command sequence, WP# should
 *    be statically held high or low."
 *
 ****************************************************************************/

static int sst39vf_chiperase(FAR struct sst39vf_dev_s *priv)
{
#if 0
  struct sst39vf_wrinfo_s wrinfo;
  clock_t start;
  clock_t elapsed;
#endif

  /* Send the sequence to erase the chip */

  sst39vf_writeseq(g_chiperase, 6);

  /* Use the data toggle delay method.  The typical delay is 40 MSec. The
   * maximum is 50 MSec.  So using the data toggle delay method should give
   * better chip erase performance by about 10MS.
   */

#if 0
  wrinfo.address = CONFIG_SST39VF_BASE_ADDRESS;
  wrinfo.data    = 0xffff;

  start = clock_systimer();
  while (delay < MSEC2TICK(SST39VF_TSCE_MSEC))
    {
      /* Check if the erase is complete */

      if (sst39vf_checktoggle(&wrinfo))
        {
          return OK;
        }

      /* No, check if the timeout has elapsed */

      elapsed = clock_systimer() - start;
      if (elapsed > MSEC2TICK(SST39VF_TSCE_MSEC))
        {
          return -ETIMEDOUT;
        }

      /* No, wait one system clock tick */

      nxsig_usleep(USEC_PER_TICK);
    }
#else
  /* Delay the maximum amount of time for the chip erase to complete. */

  nxsig_usleep(SST39VF_TSCE_MSEC * USEC_PER_MSEC);
#endif

  return OK;
}

/****************************************************************************
 * Name: sst39vf_sectorerase
 *
 * Description:
 *   Erase the entire chip
 *
 *  "... The Sector-Erase operation is initiated by executing a six-byte
 *   command sequence with Sector-Erase command (30H) and sector address
 *   (SA) in the last bus cycle.
 *
 *   The sector ... address is latched on the falling edge of the sixth
 *   WE# pulse, while the command (30H or 50H) is latched on the rising edge
 *   of the sixth WE# pulse. The internal Erase operation begins after the
 *   sixth WE# pulse. The End-of-Erase operation can be determined using
 *   either Data# Polling or Toggle Bit methods."
 *
 ****************************************************************************/

static int sst39vf_sectorerase(FAR struct sst39vf_dev_s *priv,
                               uintptr_t sectaddr)
{
  struct sst39vf_wrinfo_s wrinfo;
#if 0
  clock_t start;
  clock_t elapsed;
#endif

  /* Set up the sector address */

  wrinfo.address = sectaddr;
  wrinfo.data    = 0x0030;

  /* Send the sequence to erase the chip */

  sst39vf_writeseq(g_sectorerase, 5);
  sst39vf_flashwrite(&wrinfo);

  /* Use the data toggle delay method.  The typical delay is 18 MSec. The
   * maximum is 25 MSec.  With a 10 MS system timer resolution, this is
   * the difference of of waiting 20MS vs. 20MS.  So using the data toggle
   * delay method should give better write performance by about 10MS per
   * block.
   */

#if 0
  start = clock_systimer();
  while (delay < MSEC2TICK(SST39VF_TSE_MSEC))
    {
      /* Check if the erase is complete */

      if (sst39vf_checktoggle(&wrinfo))
        {
          return OK;
        }

      /* No, check if the timeout has elapsed */

      elapsed = clock_systimer() - start;
      if (elapsed > MSEC2TICK(SST39VF_TSE_MSEC))
        {
          return -ETIMEDOUT;
        }

      /* No, wait one system clock tick */

      nxsig_usleep(USEC_PER_TICK);
    }
#else
  /* Delay the maximum amount of time for the sector erase to complete. */

  nxsig_usleep(SST39VF_TSE_MSEC * USEC_PER_MSEC);
#endif

  return OK;
}

/****************************************************************************
 * Name: sst39vf_writeword
 *
 * Description:
 *   Write one 16-bit word to FLASH
 *
 *  "The SST39VF160x/320x are programmed on a word-by-word basis. Before
 *   programming, the sector where the word exists must be fully erased. The
 *   rogram operation is accomplished in three steps. The first step is the
 *   three-byte load sequence for Software Data Protection. The second step
 *   is to load word address and word data. During the Word-Program operation,
 *   the addresses are latched on the falling edge of either CE# or WE#,
 *   whichever occurs last. The data is latched on the rising edge of either
 *   CE# or WE#, whichever occurs first. The third step is the internal
 *   Program operation which is initiated after the rising edge of the
 *   fourth WE# or CE#, whichever occurs first. The Program operation, once
 *   initiated, will be completed within 10 µs. .... During the Program
 *   operation, the only valid reads are Data# Polling and Toggle Bit.
 *   During the internal Program operation, the host is free to perform
 *   additional tasks. Any commands issued during the internal Program
 *   operation are ignored.  During the command sequence, WP# should be
 *   statically held high or low."
 *
 ****************************************************************************/

static int sst39vf_writeword(FAR const struct sst39vf_wrinfo_s *wrinfo)
{
  /* Send the sequence to write the word to the chip */

  sst39vf_writeseq(g_wordprogram, 3);
  sst39vf_flashwrite(wrinfo);

  /* Use the data toggle delay method.  The typical delay is 7 usec; the
   * maximum is 10 usec.
   */

  return sst39vf_waittoggle(wrinfo, WORDWRITE_TIMEOUT);
}

/****************************************************************************
 * Name: sst39vf_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported (i.e., one
 *   SST39VF sector).
 *
 ****************************************************************************/

static int sst39vf_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks)
{
  FAR struct sst39vf_dev_s *priv = (FAR struct sst39vf_dev_s *)dev;
  uintptr_t address;
  int ret;

  DEBUGASSERT(priv && priv->chip && startblock < priv->chip->nsectors);

  for (address = startblock * priv->chip->sectorsize;
       nblocks > 0;
       nblocks--, address += priv->chip->sectorsize)
    {
      /* Clear the sector */

      ret = sst39vf_sectorerase(priv, address);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sst39vf_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t sst39vf_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf)
{
  FAR struct sst39vf_dev_s *priv = (FAR struct sst39vf_dev_s *)dev;
  FAR const uint8_t *source;
  size_t nbytes;

  DEBUGASSERT(priv && priv->chip && startblock < priv->chip->nsectors);

  /* Get the source address and the size of the transfer */

  source = (FAR const uint8_t *)
           SST39VF_ADDR(startblock * priv->chip->sectorsize);
  nbytes = nblocks * priv->chip->sectorsize;

  /* Copy the data to the user buffer */

  memcpy(buf, source, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: sst39vf_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t sst39vf_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct sst39vf_dev_s *priv = (FAR struct sst39vf_dev_s *)dev;
  struct sst39vf_wrinfo_s wrinfo;
  FAR const uint16_t *source = (FAR const uint16_t *)buf;
  size_t nwords;
  int ret;

  DEBUGASSERT(priv && priv->chip && ((uintptr_t)buf & 1) == 0 &&
              startblock < priv->chip->nsectors);

  /* Get the destination address and the size of the transfer */

  wrinfo.address =
    (uintptr_t)SST39VF_ADDR((startblock * priv->chip->sectorsize));
  nwords = nblocks * (priv->chip->sectorsize >> 1);

  /* Copy the data to the user buffer */

  while (nwords-- > 0)
    {
      wrinfo.data = *source++;
      ret = sst39vf_writeword(&wrinfo);
      if (ret < 0)
        {
          return ret;
        }

      wrinfo.address += sizeof(uint16_t);
    }

  return nblocks;
}

/****************************************************************************
 * Name: sst39vf_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t sst39vf_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer)
{
#ifdef CONFIG_DEBUG_FEATURES
  FAR struct sst39vf_dev_s *priv = (FAR struct sst39vf_dev_s *)dev;
#endif
  FAR const uint8_t *source;

  DEBUGASSERT(priv && priv->chip &&
              offset < priv->chip->nsectors * priv->chip->sectorssize);

  /* Get the source address and the size of the transfer */

  source = (FAR const uint8_t *)SST39VF_ADDR(offset);

  /* Copy the data to the user buffer */

  memcpy(buffer, source, nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: sst39vf_ioctl
 ****************************************************************************/

static int sst39vf_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct sst39vf_dev_s *priv = (FAR struct sst39vf_dev_s *)dev;
  int ret = -ENOTTY;

  DEBUGASSERT(priv && priv->chip);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               */

              geo->blocksize    = priv->chip->sectorsize;
              geo->erasesize    = priv->chip->sectorsize;
              geo->neraseblocks = priv->chip->nsectors;
              ret               = OK;
          }
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void **)arg;
          if (ppv)
            {
              /* Return the base address of FLASH memory */

              *ppv = (FAR void *)CONFIG_SST39VF_BASE_ADDRESS;
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire chip */

            return sst39vf_chiperase(priv);
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
 * Name: sst39vf_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance assuming an SST39VF NOR
 *   FLASH device at the configured address in memory.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst39vf_initialize(void)
{
  uint16_t manufacturer;
  uint16_t chipid;

  DEBUGASSERT(g_sst39vf.chip == NULL);

  /* Issue the software entry command sequence */

  sst39vf_writeseq(g_swid_entry, 3);
  up_udelay(10);

  /* Read the manufacturer and chip ID */

  manufacturer = sst39vf_flashread(0x0000);
  chipid = sst39vf_flashread(0x0001);

  /* Issue the software exit sequence */

  sst39vf_writeseq(g_swid_exit, 3);
  up_udelay(10);

  /* Now see if we can suport the part */

  finfo("Manufacturer: %02x\n", manufacturer);
  finfo("Chip ID:      %04x\n", chipid);

  if (manufacturer != SST_MANUFACTURER_ID)
    {
      ferr("ERROR: Unrecognized manufacturer: %02x\n", manufacturer);
      return NULL;
    }
  else if (chipid == g_sst39vf1601.chipid)
    {
      g_sst39vf.chip = &g_sst39vf1601;
    }
  else if (chipid == g_sst39vf1602.chipid)
    {
      g_sst39vf.chip = &g_sst39vf1602;
    }
  else if (chipid == g_sst39vf3201.chipid)
    {
      g_sst39vf.chip = &g_sst39vf3201;
    }
  else if (chipid == g_sst39vf3202.chipid)
    {
      g_sst39vf.chip = &g_sst39vf3202;
    }
  else
    {
      ferr("ERROR: Unrecognized chip ID: %04x\n", chipid);
      return NULL;
    }

  /* Register the MTD with the procfs system if enabled */

#ifdef CONFIG_MTD_REGISTRATION
  mtd_register(&priv->mtd, "sst39vf");
#endif

  /* Return the state structure as the MTD device */

  return (FAR struct mtd_dev_s *)&g_sst39vf;
}
