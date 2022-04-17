/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_spifi.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc43_cgu.h"
#include "lpc43_spifi.h"
#include "lpc43_pinconfig.h"
#include "spifi/inc/spifilib_api.h"

#ifdef CONFIG_LPC43_SPIFI_FIXME

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPIFI Configuration ******************************************************/

/* This logic supports some special options that can be used to create an
 * mtd device on the SPIFI FLASH.  NOTE:  CONFIG_LPC43_SPIFI=y must also
 * be defined to enable SPIFI setup support:
 *
 * CONFIG_SPIFI_READONLY - Create a read only device on SPIFI.
 * CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
 *   bytes into the device address space.  This offset must be an exact
 *   multiple of the erase block size. Default 0.
 * CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not defined
 *   then the driver will try to determine the correct erase block size by
 *   examining that data returned from spifi_initialize (which sometimes
 *   seems bad).
 * CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
 *   FAT friendly 512 byte sector size and will manage the read-modify-write
 *   operations on the larger erase block.
 * CONFIG_SPIFI_READONLY - Define to support only read-only operations.
 * CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
 *   use an external library implementation of the SPIFI interface.
 * CONFIG_SPIFI_VERIFY - Verify all spi_program() operations by reading
 *   from the SPI address space after each write.
 * CONFIG_DEBUG_SPIFI_DUMP - Debug option to dump read/write buffers.  You
 *   probably do not want to enable this unless you want to dig through a
 *   *lot* of debug output!  Also required CONFIG_DEBUG_FEATURES,
 *   CONFIG_DEBUG_INFO, and CONFIG_DEBUG_FS,
 */

/* This is where the LPC43xx address where random-access reads begin */

#define SPIFI_BASE \
       (uint8_t *)(LPC43_SPIFI_DATA_BASE + CONFIG_SPIFI_OFFSET)

/* Check if we are using a hard-coded block size */

#ifdef CONFIG_SPIFI_BLKSIZE
#  if CONFIG_SPIFI_BLKSIZE < 512
#    error "CONFIG_SPIFI_BLKSIZE is too small"
#  elif CONFIG_SPIFI_BLKSIZE == 512
#    define SPIFI_BLKSHIFT 9
#  elif CONFIG_SPIFI_BLKSIZE == 1024
#    define SPIFI_BLKSHIFT 10
#  elif CONFIG_SPIFI_BLKSIZE == (2*1024)
#    define SPIFI_BLKSHIFT 11
#  elif CONFIG_SPIFI_BLKSIZE == (4*1024)
#    define SPIFI_BLKSHIFT 12
#  elif CONFIG_SPIFI_BLKSIZE == (8*1024)
#    define SPIFI_BLKSHIFT 13
#  elif CONFIG_SPIFI_BLKSIZE == (16*1024)
#    define SPIFI_BLKSHIFT 14
#  elif CONFIG_SPIFI_BLKSIZE == (32*1024)
#    define SPIFI_BLKSHIFT 15
#  elif CONFIG_SPIFI_BLKSIZE == (64*1024)
#    define SPIFI_BLKSHIFT 16
#  elif CONFIG_SPIFI_BLKSIZE == (128*1024)
#    define SPIFI_BLKSHIFT 17
#  elif CONFIG_SPIFI_BLKSIZE == (256*1024)
#    define SPIFI_BLKSHIFT 18
#  else
#    error "Unsupported value of CONFIG_SPIFI_BLKSIZE"
#  endif
#  define SPIFI_BLKSIZE  CONFIG_SPIFI_BLKSIZE
#else
#  define SPIFI_BLKSIZE  priv->blksize
#  define SPIFI_BLKSHIFT priv->blkshift
#endif

/* Can use ROM driver or an external driver library */

#ifndef CONFIG_SPIFI_LIBRARY
#  define SPIFI_INIT(priv, rom, cshigh, options, mhz) \
   priv->spifi->spifi_init(rom, cshigh, options, mhz)
#  define SPIFI_PROGRAM(priv, rom, src, operands) \
   priv->spifi->spifi_program(rom, src, operands)
#  define SPIFI_ERASE(priv, rom, operands) \
   priv->spifi->spifi_erase(rom, operands)
#else
#  define SPIFI_INIT(priv, rom, cshigh, options, mhz) \
   spifiInit(rom, cshigh, options, mhz)
#  define SPIFI_PROGRAM(priv, rom, src, operands) \
   spifiProgram(rom, src, operands)
#  define SPIFI_ERASE(priv, rom, operands) \
   spifiErase(rom, operands)
#endif

/* 512 byte sector simulation */

#ifdef CONFIG_SPIFI_SECTOR512                /* Emulate a 512 byte sector */
#  define SPIFI_512SHIFT         9           /* Sector size 1 << 9 = 512 bytes */
#  define SPIFI_512SIZE          512         /* Sector size = 512 bytes */
#endif

#define SPIFI_ERASED_STATE       0xff        /* State of FLASH when erased */

/* Cache flags */

#define SST25_CACHE_VALID        (1 << 0)    /* 1=Cache has valid data */
#define SST25_CACHE_DIRTY        (1 << 1)    /* 1=Cache is dirty */
#define SST25_CACHE_ERASED       (1 << 2)    /* 1=Backing FLASH is erased */

#define IS_VALID(p)              ((((p)->flags) & SST25_CACHE_VALID) != 0)
#define IS_DIRTY(p)              ((((p)->flags) & SST25_CACHE_DIRTY) != 0)
#define IS_ERASED(p)             ((((p)->flags) & SST25_CACHE_ERASED) != 0)

#define SET_VALID(p)             do { (p)->flags |= SST25_CACHE_VALID; } while (0)
#define SET_DIRTY(p)             do { (p)->flags |= SST25_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)            do { (p)->flags |= SST25_CACHE_ERASED; } while (0)

#define CLR_VALID(p)             do { (p)->flags &= ~SST25_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)             do { (p)->flags &= ~SST25_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)            do { (p)->flags &= ~SST25_CACHE_ERASED; } while (0)

/* Select the divider to use as SPIFI input based on definitions in the
 * board.h header file.
 */

#if defined(BOARD_SPIFI_PLL1)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_PLL1
#elif defined(BOARD_SPIFI_DIVA)
#  define LPC43_IDIV_CTRL        LPC43_IDIVA_CTRL
#  define IDIV_CTRL_PD           IDIVA_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVA_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVA_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     4
#  define IDIV_CTRL_CLKSEL_MASK  IDIVA_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVA_CLKSEL_PLL1 | IDIVA_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVA
#elif defined(BOARD_SPIFI_DIVB)
#  define LPC43_IDIV_CTRL        LPC43_IDIVB_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVB
#elif defined(BOARD_SPIFI_DIVC)
#  define LPC43_IDIV_CTRL        LPC43_IDIVC_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVC
#elif defined(BOARD_SPIFI_DIVD)
#  define LPC43_IDIV_CTRL        LPC43_IDIVD_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVD
#elif defined(BOARD_SPIFI_DIVE)
#  define LPC43_IDIV_CTRL        LPC43_IDIVE_CTRL
#  define IDIV_CTRL_PD           IDIVE_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVE_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVE_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     256
#  define IDIV_CTRL_CLKSEL_MASK  IDIVE_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVE_CLKSEL_PLL1 | IDIVE_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVE
#endif

#if BOARD_SPIFI_DIVIDER < 1 || BOARD_SPIFI_DIVIDER > IDIV_CTRL_IDIV_MAX
#  error "Invalid value for  BOARD_SPIFI_DIVIDER"
#endif

/* SPIFI_CSHIGH should be one less than the minimum number of clock cycles
 * with the CS pin high, that the SPIFI should maintain between commands.
 * Compute this from the SPIFI clock period and the minimum high time of CS
 * from the serial flash data sheet:
 *
 *   csHigh = ceiling(min CS high / SPIFI clock period) - 1
 *
 * where ceiling means round up to the next higher integer if the argument
 * isn't an integer.
 */

#define SPIFI_CSHIGH 9

/* The final parameter of the spifi_init() ROM driver call should be the
 * serial clock rate divided by 1000000, rounded to an integer.  The SPIFI
 * supports transfer rates of up to SPIFI_CLK/2 bytes per second.
 * The SPIF_CLK is the output of the LPC43_BASE_SPIFI_CLK configured above;
 * The frequency should be given by BOARD_SPIFI_FREQUENCY as provided by the
 * board.h header file.
 */

#define SCLK_MHZ (BOARD_SPIFI_FREQUENCY + (1000000 / 2)) / 1000000

/* DEBUG options to dump read/write buffers.  You probably do not want to
 * enable this unless you want to dig through a *lot* of debug output!
 */

#if !defined(CONFIG_DEBUG_FEATURES) || !defined(CONFIG_DEBUG_INFO) || !defined(CONFIG_DEBUG_FS)
#  undef CONFIG_DEBUG_SPIFI_DUMP
#endif

#ifdef CONFIG_DEBUG_SPIFI_DUMP
#  define lpc43_dumpbuffer(m,b,n) lib_dumpbuffer(m,b,n);
#else
#  define lpc43_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct lpc43_dev_s.
 */

struct lpc43_dev_s
{
  struct mtd_dev_s mtd;             /* MTD interface */
#ifndef CONFIG_SPIFI_LIBRARY
  struct spifi_driver_s *spifi;     /* Pointer to ROM driver table */
#endif
  struct spifi_dev_s rom;           /* Needed for communication with ROM driver */
  struct spifi_operands_s operands; /* Needed for program and erase ROM calls */
  uint16_t nblocks;                 /* Number of blocks of size blksize */
#ifndef CONFIG_SPIFI_BLKSIZE
  uint8_t blkshift;                 /* Log2 of erase block size */
  uint32_t blksize;                 /* Size of one erase block (up to 256K) */
#endif

#if defined(CONFIG_SPIFI_SECTOR512) && !defined(CONFIG_SPIFI_READONLY)
  uint8_t flags;                    /* Buffered sector flags */
  uint16_t blkno;                   /* Erase block number in the cache */
  uint8_t *cache;                   /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void lpc43_blockerase(struct lpc43_dev_s *priv, off_t offset);
static inline int lpc43_chiperase(struct lpc43_dev_s *priv);
static inline void lpc43_pageread(struct lpc43_dev_s *priv,
                                  uint8_t *dest, const uint8_t *src,
                                  size_t nbytes);
#ifndef CONFIG_SPIFI_READONLY
#ifdef CONFIG_SPIFI_VERIFY
static int lpc43_verify(struct lpc43_dev_s *priv, uint8_t *dest,
                        const uint8_t *src, size_t nbytes);
#endif
static int lpc43_pagewrite(struct lpc43_dev_s *priv, uint8_t *dest,
                           const uint8_t *src, size_t nbytes);
#ifdef CONFIG_SPIFI_SECTOR512
static void lpc43_cacheflush(struct lpc43_dev_s *priv);
static uint8_t *lpc43_cacheread(struct lpc43_dev_s *priv, off_t sector);
static void lpc43_cacheerase(struct lpc43_dev_s *priv, off_t sector);
static void lpc43_cachewrite(struct lpc43_dev_s *priv,
                             const uint8_t *buffer,
                             off_t sector, size_t nsectors);
#endif
#endif

/* MTD driver methods */

static int lpc43_erase(struct mtd_dev_s *dev,
                       off_t startblock, size_t nblocks);
static ssize_t lpc43_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buf);
static ssize_t lpc43_bwrite(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, const uint8_t *buf);
static ssize_t lpc43_read(struct mtd_dev_s *dev,
                          off_t offset, size_t nbytes,
                          uint8_t *buffer);
static int lpc43_ioctl(struct mtd_dev_s *dev,
                       int cmd, unsigned long arg);

/* Initialization */

#ifndef BOARD_SPIFI_PLL1
static inline void lpc43_idiv_clkconfig(void);
#endif

static inline void lpc43_spifi_clkconfig(void);
static inline void lpc43_spifi_pinconfig(void);
static inline int lpc43_rominit(struct lpc43_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only a single SPIFI driver instance is supported */

static struct lpc43_dev_s g_spifi;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  lpc43_blockerase
 ****************************************************************************/

static void lpc43_blockerase(struct lpc43_dev_s *priv, off_t sector)
{
  int result;

  /* Erase one block on the chip:
   *
   * dest   - Specifies the first address to be programmed or erased, either
   *          in the SPIFI memory area or as a zero-based device address.
   *          It must be at an offset that is an exact multiple of the erase
   *          block size.
   * length - The number of bytes to be programmed or erased
   */

  priv->operands.dest   = SPIFI_BASE + (sector << SPIFI_BLKSHIFT);
  priv->operands.length = SPIFI_BLKSIZE;

  finfo("SPIFI_ERASE: dest=%p length=%d\n",
        priv->operands.dest, priv->operands.length);

  result = SPIFI_ERASE(priv, &priv->rom, &priv->operands);
  if (result != 0)
    {
      ferr("ERROR: SPIFI_ERASE failed: %05x\n", result);
    }
}

/****************************************************************************
 * Name:  lpc43_chiperase
 ****************************************************************************/

static inline int lpc43_chiperase(struct lpc43_dev_s *priv)
{
  int result;

  /* Erase the entire chip:
   *
   * dest   - Specifies the first address to be programmed or erased,
   *          either in the SPIFI memory area or as a zero-based device
   *          address.  It must be at an offset that is an exact multiple
   *          of the erase block size.
   * length - The number of bytes to be programmed or erased
   */

  priv->operands.dest   = SPIFI_BASE;
  priv->operands.length = SPIFI_BLKSIZE * priv->nblocks;

  finfo("SPIFI_ERASE: dest=%p length=%d\n",
        priv->operands.dest, priv->operands.length);

  result = SPIFI_ERASE(priv, &priv->rom, &priv->operands);
  if (result != 0)
    {
      ferr("ERROR: SPIFI_ERASE failed: %05x\n", result);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: lpc43_pagewrite
 ****************************************************************************/

#if !defined(CONFIG_SPIFI_READONLY) && defined(CONFIG_SPIFI_VERIFY)
static int lpc43_verify(struct lpc43_dev_s *priv, uint8_t *dest,
                        const uint8_t *src, size_t nbytes)
{
  return memcmp(src, dest, nbytes) != 0 ? -EIO : OK;
}
#endif

/****************************************************************************
 * Name: lpc43_pagewrite
 ****************************************************************************/

#ifndef CONFIG_SPIFI_READONLY
static int lpc43_pagewrite(struct lpc43_dev_s *priv, uint8_t *dest,
                           const uint8_t *src, size_t nbytes)
{
  int result;

  /* Write FLASH pages:
   *
   * dest   - Specifies the first address to be programmed or erased, either
   *          in the SPIFI memory area or as a zero-based device address.
   *          It must be at an offset that is an exact multiple of the
   *          erase block size.
   * length - The number of bytes to be programmed or erased
   */

  priv->operands.dest   = dest;
  priv->operands.length = nbytes;

  finfo("SPIFI_PROGRAM: src=%p dest=%p length=%d\n",
        src, priv->operands.dest, priv->operands.length);

  result = SPIFI_PROGRAM(priv, &priv->rom, src, &priv->operands);
  if (result != 0)
    {
      ferr("ERROR: SPIFI_PROGRAM failed: %05x\n", result);
      return -EIO;
    }

  /* Verify the data that was written by comparing to the data visible in
   * the SPIFI address space.
   */

#ifdef CONFIG_SPIFI_VERIFY
  result = lpc43_verify(priv, dest, src, nbytes);
  if (result != 0)
    {
      ferr("ERROR: lpc43_verify failed: %05x\n", result);
      return -EIO;
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_pageread
 ****************************************************************************/

static inline void lpc43_pageread(struct lpc43_dev_s *priv,
                                  uint8_t *dest, const uint8_t *src,
                                  size_t nbytes)
{
  finfo("src=%p dest=%p length=%d\n", src, dest, nbytes);
  memcpy(dest, src, nbytes);
}

/****************************************************************************
 * Name: lpc43_cacheflush
 ****************************************************************************/

#if defined(CONFIG_SPIFI_SECTOR512) && !defined(CONFIG_SPIFI_READONLY)
static void lpc43_cacheflush(struct lpc43_dev_s *priv)
{
  uint8_t *dest;
  int ret;

  /* If the cached is dirty (meaning that it no longer matches the old
   * FLASH contents) or was erased (with the cache containing the correct
   * FLASH contents), then write the cached erase block to FLASH.
   */

  finfo("flags: %02x blkno: %d\n", priv->flags, priv->blkno);
  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Get the SPIFI address corresponding to the cached erase block */

      dest = SPIFI_BASE + ((off_t)priv->blkno << SPIFI_BLKSHIFT);

      /* Write entire erase block to FLASH */

      ret = lpc43_pagewrite(priv, dest, priv->cache, SPIFI_BLKSIZE);
      if (ret < 0)
        {
          ferr("ERROR: lpc43_pagewrite failed: %d\n", ret);
        }

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/****************************************************************************
 * Name: lpc43_cacheread
 ****************************************************************************/

#if defined(CONFIG_SPIFI_SECTOR512) && !defined(CONFIG_SPIFI_READONLY)
static uint8_t *lpc43_cacheread(struct lpc43_dev_s *priv, off_t sector)
{
  const uint8_t *src;
  off_t blkno;
  int   index;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For exmample, if the actual erase sector size if 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  blkno = sector >> (SPIFI_BLKSHIFT - SPIFI_512SHIFT);
  finfo("sector: %ld blkno: %d\n", sector, blkno);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || blkno != (off_t)priv->blkno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      lpc43_cacheflush(priv);

      /* Read the new erase block into the cache */

      /* Get the SPIFI address corresponding to the new erase block */

      src = SPIFI_BASE + (blkno << SPIFI_BLKSHIFT);

      /* Read the entire erase block from FLASH */

      lpc43_pageread(priv, priv->cache, src, SPIFI_BLKSIZE);

      /* Mark the sector as cached */

      priv->blkno = (uint16_t)blkno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << (SPIFI_BLKSHIFT - SPIFI_512SHIFT)) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->cache[index << SPIFI_512SHIFT];
}
#endif

/****************************************************************************
 * Name: lpc43_cacheerase
 ****************************************************************************/

#if defined(CONFIG_SPIFI_SECTOR512) && !defined(CONFIG_SPIFI_READONLY)
static void lpc43_cacheerase(struct lpc43_dev_s *priv, off_t sector)
{
  uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = lpc43_cacheread(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase
   * sector is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t blkno  = sector >> (SPIFI_BLKSHIFT - SPIFI_512SHIFT);
      finfo("sector: %ld blkno: %d\n", sector, blkno);

      lpc43_blockerase(priv, blkno);
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as
   * dirty (but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, SPIFI_ERASED_STATE, SPIFI_512SIZE);
  SET_DIRTY(priv);
}
#endif

/****************************************************************************
 * Name: lpc43_cachewrite
 ****************************************************************************/

#if defined(CONFIG_SPIFI_SECTOR512) && !defined(CONFIG_SPIFI_READONLY)
static void lpc43_cachewrite(struct lpc43_dev_s *priv,
                             const uint8_t *buffer,
                             off_t sector, size_t nsectors)
{
  uint8_t *dest;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector
       * is in memory.
       */

      dest = lpc43_cacheread(priv, sector);

      finfo("dest=%p src=%p sector: %ld flags: %02x\n",
            dest, buffer, sector, priv->flags);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t blkno  = sector >> (SPIFI_BLKSHIFT - SPIFI_512SHIFT);
          finfo("sector: %ld blkno: %d\n", sector, blkno);

          lpc43_blockerase(priv, blkno);
          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, SPIFI_512SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += SPIFI_512SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  lpc43_cacheflush(priv);
}
#endif

/****************************************************************************
 * Name: lpc43_erase
 ****************************************************************************/

static int lpc43_erase(struct mtd_dev_s *dev,
                       off_t startblock, size_t nblocks)
{
#ifdef CONFIG_SPIFI_READONLY
  return -EACESS
#else
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_SPIFI_SECTOR512
      lpc43_cacheerase(priv, startblock);
#else
      lpc43_blockerase(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_SPIFI_SECTOR512
  /* Flush the last erase block left in the cache */

  lpc43_cacheflush(priv);
#endif

  return (int)nblocks;
#endif
}

/****************************************************************************
 * Name: lpc43_bread
 ****************************************************************************/

static ssize_t lpc43_bread(struct mtd_dev_s *dev,
                           off_t startblock, size_t nblocks,
                           uint8_t *buffer)
{
#ifdef CONFIG_SPIFI_SECTOR512
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = lpc43_read(dev, startblock << SPIFI_512SHIFT,
                      nblocks << SPIFI_512SHIFT, buffer);
  if (nbytes > 0)
    {
      lpc43_dumpbuffer(__func__, buffer, nbytes)
      return nbytes >> SPIFI_512SHIFT;
    }

  return (int)nbytes;
#else
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  nbytes = lpc43_read(dev, startblock << SPIFI_BLKSHIFT,
                      nblocks << SPIFI_BLKSHIFT, buffer);
  if (nbytes > 0)
    {
      lpc43_dumpbuffer(__func__, buffer, nbytes)
      return nbytes >> SPIFI_BLKSHIFT;
    }

  return (int)nbytes;
#endif
}

/****************************************************************************
 * Name: lpc43_bwrite
 ****************************************************************************/

static ssize_t lpc43_bwrite(struct mtd_dev_s *dev,
                            off_t startblock, size_t nblocks,
                            const uint8_t *buffer)
{
#if defined(CONFIG_SPIFI_READONLY)

  return -EACCESS;

#elif defined(CONFIG_SPIFI_SECTOR512)

  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  lpc43_cachewrite(priv, buffer, startblock, nblocks);

  lpc43_dumpbuffer(__func__, buffer, nblocks << SPIFI_512SHIFT)
  return (ssize_t)nblocks;

#else

  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint8_t *dest;
  int ret;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Get the SPIFI address corresponding to the erase block */

  dest = SPIFI_BASE + (startblock << SPIFI_BLKSHIFT);

  /* Write all of the erase blocks to FLASH */

  ret = lpc43_pagewrite(priv, dest, buffer, nblocks << SPIFI_BLKSHIFT);
  if (ret < 0)
    {
      ferr("ERROR: lpc43_pagewrite failed: %d\n", ret);
      return ret;
    }

  lpc43_dumpbuffer(__func__, buffer, nblocks << SPIFI_BLKSHIFT);
  return (ssize_t)nblocks;

#endif
}

/****************************************************************************
 * Name: lpc43_read
 ****************************************************************************/

static ssize_t lpc43_read(struct mtd_dev_s *dev,
                          off_t offset, size_t nbytes,
                          uint8_t *buffer)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  const uint8_t *src;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Get the SPIFI address corresponding sector */

  src = SPIFI_BASE + offset;

  /* Read FLASH contents into  the user buffer */

  lpc43_pageread(priv, buffer, src, nbytes);

  finfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: lpc43_ioctl
 ****************************************************************************/

static int lpc43_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo =
                             (struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the
               * client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

#ifdef CONFIG_SPIFI_SECTOR512
              geo->blocksize    = 512;
              geo->erasesize    = 512;
              geo->neraseblocks = priv->nblocks <<
                                  (SPIFI_BLKSHIFT - SPIFI_512SHIFT);
#else
              geo->blocksize    = SPIFI_BLKSIZE;
              geo->erasesize    = SPIFI_BLKSIZE;
              geo->neraseblocks = priv->nblocks;
#endif
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info =
            (struct partition_info_s *)arg;
          if (info != NULL)
            {
#ifdef CONFIG_SPIFI_SECTOR512
              info->numsectors  = priv->nblocks <<
                                  (SPIFI_BLKSHIFT - SPIFI_512SHIFT);
              info->sectorsize  = 512;
#else
              info->numsectors  = priv->nblocks;
              info->sectorsize  = SPIFI_BLKSIZE;
#endif
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            ret = lpc43_chiperase(priv);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;
          *result = SPIFI_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Name: lpc43_idiv_clkconfig
 *
 * Description:
 *   Configure PLL1 as the input to the selected divider and enable the
 *   divider.
 *
 ****************************************************************************/

#ifndef BOARD_SPIFI_PLL1
static inline void lpc43_idiv_clkconfig(void)
{
  uint32_t regval;

  /* Configure PLL1 as the input to the selected divider */

  regval  = getreg32(LPC43_IDIV_CTRL);
  regval &= ~IDIV_CTRL_CLKSEL_MASK;
  regval |= IDIV_CTRL_CLKSEL_PLL1;
  putreg32(regval, LPC43_IDIV_CTRL);

  /* Enable the divider (by making sure that the power down bit is clear) */

  regval &= ~IDIV_CTRL_PD;
  putreg32(regval, LPC43_IDIV_CTRL);

  /* Set the divider value */

  regval &= ~IDIVA_CTRL_IDIV_MASK;
  regval |= IDIV_CTRL_IDIV;
  putreg32(regval, LPC43_IDIV_CTRL);
}
#else
#  define lpc43_idiv_clkconfig()
#endif

/****************************************************************************
 * Name: lpc43_spifi_clkconfig
 *
 * Description:
 *   Configure the selected divider (or PLL1) as the input to the SPIFI
 *   and enable the SPIFI clock.
 *
 ****************************************************************************/

static inline void lpc43_spifi_clkconfig(void)
{
  uint32_t regval;

  /* Configure the selected divider (or PLL1) as the input to the SPIFI */

  regval  = getreg32(LPC43_BASE_SPIFI_CLK);
  regval &= ~BASE_SPIFI_CLK_CLKSEL_MASK;
  regval |= BASE_SPIFI_CLKSEL;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);

  /* Enable the SPIFI clocking (by making sure that the power down bit is
   * clear)
   */

  regval &= ~IDIVA_CTRL_PD;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);
}

/****************************************************************************
 * Name: lpc43_spifi_pinconfig
 *
 * Description:
 *   Configure SPIFI pins
 *
 ****************************************************************************/

static inline void lpc43_spifi_pinconfig(void)
{
  /* Configure SPIFI pins */

  lpc43_pin_config(PINCONF_SPIFI_CS);   /* Input buffering not needed */
  lpc43_pin_config(PINCONF_SPIFI_MISO); /* High drive for SCLK */
  lpc43_pin_config(PINCONF_SPIFI_MOSI);
  lpc43_pin_config(PINCONF_SPIFI_SCK);
  lpc43_pin_config(PINCONF_SPIFI_SIO2);
  lpc43_pin_config(PINCONF_SPIFI_SIO3);
}

/****************************************************************************
 * Name: lpc43_rominit
 *
 * Description:
 *   Initialize the SPIFI ROM driver
 *
 ****************************************************************************/

static inline int lpc43_rominit(struct lpc43_dev_s *priv)
{
#ifndef CONFIG_SPIFI_BLKSIZE
  struct spfi_desc_s *desc;
  uint16_t sectors;
  uint8_t log2;
#endif
  int32_t result;

  /* Get the pointer to the SPIFI ROM driver table. */

#ifndef CONFIG_SPIFI_LIBRARY
  priv->spifi = *((struct spifi_driver_s **)SPIFI_ROM_PTR);
#endif

  /* The final parameter of the spifi_init() ROM driver call should be the
   * serial clock rate divided by 1000000, rounded to an integer.  The
   * SPIFI supports transfer rates of up to SPIFI_CLK/2 bytes per second.
   * The SPIF_CLK is the output of the LPC43_BASE_SPIFI_CLK configured above;
   * The frequency should be given by BOARD_SPIFI_FREQUENCY as provided by
   * the board.h header file.
   *
   * A return value of zero frp spifi_init() indicates success.  Non-zero
   * error codes include:
   *
   *   0x2000A  No operative serial flash (JEDEC ID all zeroes or all ones)
   *   0x20009  Unknown manufacturer code
   *   0x20008  Unknown device type code
   *   0x20007  Unknown device ID code
   *   0x20006  Unknown extended device ID value (only for Spansion 25FL12x
   *            in the initial API)
   *   0x20005  Device status error
   *   0x20004  Operand error: S_MODE3+S_FULLCLK+S_RCVCLK in options
   */

  result = SPIFI_INIT(priv, &priv->rom, SPIFI_CSHIGH,
                      S_RCVCLK | S_FULLCLK, SCLK_MHZ);
  if (result != 0)
    {
      ferr("ERROR: SPIFI_INIT failed: %05x\n", result);

      /* Try again */

      result = SPIFI_INIT(priv, &priv->rom, SPIFI_CSHIGH,
                          S_RCVCLK | S_FULLCLK, SCLK_MHZ);
      if (result != 0)
        {
          ferr("ERROR: SPIFI_INIT failed: %05x\n", result);
          return -ENODEV;
        }
    }

  finfo("SPFI:\n");
  finfo("      base: %08x\n", priv->rom.base);
  finfo("   regbase: %08x\n", priv->rom.regbase);
  finfo("   devsize: %08x\n", priv->rom.devsize);
  finfo("   memsize: %08x\n", priv->rom.memsize);
  finfo("     mfger: %02x\n", priv->rom.mfger);
  finfo("   devtype: %02x\n", priv->rom.devtype);
  finfo("     devid: %02x\n", priv->rom.devid);
  finfo("      busy: %02x\n", priv->rom.busy);
  finfo("      stat: %04x\n", priv->rom.stat.h);
  finfo("   setprot: %04x\n", priv->rom.setprot);
  finfo(" writeprot: %04x\n", priv->rom.writeprot);
  finfo("    memcmd: %08x\n", priv->rom.memcmd);
  finfo("   progcmd: %08x\n", priv->rom.progcmd);
  finfo("   sectors: %04x\n", priv->rom.sectors);
  finfo(" protbytes: %04x\n", priv->rom.protbytes);
  finfo("      opts: %08x\n", priv->rom.opts);
  finfo("  errcheck: %08x\n", priv->rom.errcheck);

  /* Get the largest erase block size */

#ifndef CONFIG_SPIFI_BLKSIZE

  desc    = priv->rom.protents;
  sectors = priv->rom.sectors;
  log2    = 0;

  finfo("FLASH Geometry:\n");

  while (sectors > 0)
    {
      finfo("  log2: %d rept: %d\n", desc->log2, desc->rept);

      /* Check if this is the largest erase block size seen */

      if (desc->log2 > log2)
        {
          log2 = desc->log2;
        }

      /* Decrement the count of sectors we have checked */

      sectors -= desc->rept;
    }

  DEBUGASSERT(log2 > 0);

  /* Save the digested FLASH geometry info */

  priv->blkshift = log2;
  priv->blksize  = (1 << log2);
  priv->nblocks  = (priv->rom.memsize - CONFIG_SPIFI_OFFSET) / priv->blksize;

  finfo("Driver FLASH Geometry:\n");
  finfo("  blkshift: %d\n", priv->blkshift);
  finfo("   blksize: %08x\n", priv->blksize);
  finfo("   nblocks: %d\n", priv->nblocks);

#ifdef CONFIG_SPIFI_SECTOR512
  DEBUGASSERT(log2 > 9);
#endif

#else

  /* Save the digested FLASH geometry info */

  priv->nblocks  = ((priv->rom.memsize  - CONFIG_SPIFI_OFFSET) >>
                     SPIFI_BLKSHIFT);

  finfo("Driver FLASH Geometry:\n");
  finfo("  blkshift: %d\n", SPIFI_BLKSHIFT);
  finfo("   blksize: %08x\n", SPIFI_BLKSIZE);
  finfo("   nblocks: %d\n", priv->nblocks);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_spifi_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for the SPIFI device.  MTD
 *   devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 *   SPIFI interface clocking is configured per settings in the board.h file.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a reference to the initialized MTD device instance is
 *   returned;  NULL is returned on any failure.
 *
 ****************************************************************************/

struct mtd_dev_s *lpc43_spifi_initialize(void)
{
  /* At present, only a single instance of the SPIFI driver is supported */

  struct lpc43_dev_s *priv = &g_spifi;
  irqstate_t flags;
  int ret;

  /* Initialize the SPIFI driver structure.  Since the driver instance lies
   * in .bss, it should have been already cleared to zero.
   */

  priv->mtd.erase  = lpc43_erase;
  priv->mtd.bread  = lpc43_bread;
  priv->mtd.bwrite = lpc43_bwrite;
  priv->mtd.read   = lpc43_read;
  priv->mtd.ioctl  = lpc43_ioctl;

  priv->operands.protect = -1;              /* Save and restore protection */
  priv->operands.options = S_CALLER_ERASE;  /* This driver will do erasure */

  /* Initialize the SPIFI.  Interrupts must be disabled here because shared
   * CGU registers will be modified.
   */

  flags = enter_critical_section();

  /* The SPIFI will receive clocking from a divider per the settings
   * provided in the board.h file.  Configure PLL1 as the input clock
   * for the selected divider
   */

  lpc43_idiv_clkconfig();

  /* Configure SPIFI to received clocking from the selected divider */

  lpc43_spifi_clkconfig();

  /* Configure SPIFI pins */

  lpc43_spifi_pinconfig();
  leave_critical_section(flags);

  /* Initialize the SPIFI ROM driver */

  ret = lpc43_rominit(priv);
  if (ret != OK)
    {
      return NULL;
    }

  /* Check if we need to emulator a 512 byte sector */

#ifdef CONFIG_SPIFI_SECTOR512

  /* Allocate a buffer for the erase block cache */

  priv->cache = (uint8_t *)kmm_malloc(SPIFI_BLKSIZE);
  if (!priv->cache)
    {
      /* Allocation failed!
       * Discard all of that work we just did and return NULL
       */

      ferr("ERROR: Allocation failed\n");
      return NULL;
    }
#endif

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (struct mtd_dev_s *)priv;
}

/****************************************************************************
 * Name: pull_miso
 *
 * Description:
 *   hardware-control routine used by spifi_rom_api.c
 *
 * Input Parameters:
 *   high
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SPIFI_LIBRARY
void pull_miso(int high)
{
  uint32_t pinconfig;

  /* Control MISO pull-up/down state  Assume pull down by clearing:
   *
   *  EPD = Enable pull-down connect (bit
   */

  pinconfig = PINCONF_SPIFI_MISO & ~(PINCONF_PULLUP | PINCONF_PULLDOWN);
  switch (high)
    {
      case 0:
        {
          /* Pull down */

          pinconfig |= PINCONF_PULLDOWN;
        }
        break;

      case 1:
        {
          /* Pull up */

          pinconfig |= PINCONF_PULLUP;
        }
        break;

      default:
        {
          /* Neither */
        }
        break;
    }

  /* Reconfigure MISO */

  lpc43_pin_config(pinconfig);
}
#endif

#endif /* CONFIG_LPC43_SPIFI */
