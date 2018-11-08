/************************************************************************************
 * drivers/mtd/at24xx.c
 * Driver for I2C-based at24cxx EEPROM(at24c32,at24c64,at24c128,at24c256,at24c512)
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from drivers/mtd/m25px.c
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_MTD_AT24XX

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* As a minimum, the size of the AT24 part and its 7-bit I2C address are required. */

#ifndef CONFIG_AT24XX_SIZE
#  warning "Assuming AT24 size 64"
#  define CONFIG_AT24XX_SIZE 64
#endif
#if !defined(CONFIG_AT24XX_ADDR) && !defined(CONFIG_AT24XX_MULTI)
#  warning "Assuming AT24 I2C address of 0x50"
#  define CONFIG_AT24XX_ADDR 0x50
#endif
#ifndef CONFIG_AT24XX_FREQUENCY
#  warning "Assuming AT24 I2C frequency of 100KHz"
#  define CONFIG_AT24XX_FREQUENCY 100000
#endif

/* Get the part configuration based on the size configuration */

#if CONFIG_AT24XX_SIZE == 2       /* AT24C02: 2Kbits = 256; 32 * 8 =  256 */
#  define AT24XX_NPAGES     32
#  define AT24XX_PAGESIZE   8
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 4     /* AT24C04: 4Kbits = 512B; 32 * 16 = 512 */
#  define AT24XX_NPAGES     32
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 8     /* AT24C08: 8Kbits = 1KiB; 64 * 16 = 1024 */
#  define AT24XX_NPAGES     64
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 16    /* AT24C16: 16Kbits = 2KiB; 128 * 16 = 2048 */
#  define AT24XX_NPAGES     128
#  define AT24XX_PAGESIZE   16
#  define AT24XX_ADDRSIZE   1
#elif CONFIG_AT24XX_SIZE == 32    /* AT24C32: 32Kbits = 4KiB; 128 * 32 =  4096 */
#  define AT24XX_NPAGES     128
#  define AT24XX_PAGESIZE   32
#  define AT24XX_ADDRSIZE   2
#elif CONFIG_AT24XX_SIZE == 48    /* AT24C48: 48Kbits = 6KiB; 192 * 32 =  6144 */
#  define AT24XX_NPAGES     192
#  define AT24XX_PAGESIZE   32
#  define AT24XX_ADDRSIZE   2
#elif CONFIG_AT24XX_SIZE == 64    /* AT24C64: 64Kbits = 8KiB; 256 * 32 = 8192 */
#  define AT24XX_NPAGES     256
#  define AT24XX_PAGESIZE   32
#  define AT24XX_ADDRSIZE   2
#elif CONFIG_AT24XX_SIZE == 128   /* AT24C128: 128Kbits = 16KiB; 256 * 64 = 16384 */
#  define AT24XX_NPAGES     256
#  define AT24XX_PAGESIZE   64
#  define AT24XX_ADDRSIZE   2
#elif CONFIG_AT24XX_SIZE == 256   /* AT24C256: 256Kbits = 32KiB; 512 * 64 = 32768 */
#  define AT24XX_NPAGES     512
#  define AT24XX_PAGESIZE   64
#  define AT24XX_ADDRSIZE   2
#elif CONFIG_AT24XX_SIZE == 512   /* AT24C512: 512Kbits = 64KiB; 512 * 128 = 65536 */
#  define AT24XX_NPAGES     512
#  define AT24XX_PAGESIZE   128
#  define AT24XX_ADDRSIZE   2
#endif

/* For applications where a file system is used on the AT24, the tiny page sizes
 * will result in very inefficient FLASH usage.  In such cases, it is better if
 * blocks are comprised of "clusters" of pages so that the file system block
 * size is, say, 256 or 512 bytes.  In any event, the block size *must* be an
 * even multiple of the pages.
 */

#ifndef CONFIG_AT24XX_MTD_BLOCKSIZE
#  warning "Assuming MTD driver block size is the same as the FLASH page size"
#  define CONFIG_AT24XX_MTD_BLOCKSIZE AT24XX_PAGESIZE
#endif

#ifndef CONFIG_AT24XX_TIMEOUT_MS
#  define CONFIG_AT24XX_TIMEOUT_MS 10
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at24c_dev_s.
 */

struct at24c_dev_s
{
  struct mtd_dev_s mtd;         /* MTD interface */
  FAR struct i2c_master_s *dev; /* Saved I2C interface instance */
  bool initd;                   /* True: The device has been initialize */
#ifdef CONFIG_AT24XX_EXTENDED
  bool extended;                /* True: use extended memory region */
#endif
  uint8_t addr;                 /* I2C address */
  uint16_t pagesize;            /* 32, 63 */
  uint16_t npages;              /* 128, 256, 512, 1024 */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* MTD driver methods */

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buf);
static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buf);
static ssize_t at24c_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR uint8_t *buffer);
static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifndef CONFIG_AT24XX_MULTI
/* If only a signal AT24 part is supported then a statically allocated state
 * structure may be used.
 */

static struct at24c_dev_s g_at24c;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Name: at24c_i2c_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static int at24c_i2c_write(FAR struct at24c_dev_s *priv, uint16_t at24addr,
                           FAR const uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;

  /* Setup for the transfer */

  msg.frequency = CONFIG_AT24XX_FREQUENCY,
  msg.addr      = at24addr;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)buffer;  /* Override const */
  msg.length    = buflen;

  /* Then perform the transfer. */

  return I2C_TRANSFER(priv->dev, &msg, 1);
}

/****************************************************************************
 * Name: at24c_i2c_read
 *
 * Description:
 *   Read from the I2C device.
 *
 ****************************************************************************/

static int at24c_i2c_read(FAR struct at24c_dev_s *priv, uint16_t at24addr,
                          FAR uint8_t *buffer, int buflen)
{
  struct i2c_msg_s msg;

  /* Setup for the transfer */

  msg.frequency = CONFIG_AT24XX_FREQUENCY,
  msg.addr      = at24addr,
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Then perform the transfer. */

  return I2C_TRANSFER(priv->dev, &msg, 1);
}

/************************************************************************************
 * Name: at24c_eraseall
 ************************************************************************************/

static int at24c_eraseall(FAR struct at24c_dev_s *priv)
{
  uint8_t buf[AT24XX_PAGESIZE + AT24XX_ADDRSIZE];
  int startblock = 0;
  uint16_t wait;

  memset(&buf[AT24XX_ADDRSIZE], 0xff, priv->pagesize);

  for (startblock = 0; startblock < priv->npages; startblock++)
    {
      uint16_t offset = startblock * priv->pagesize;
      uint16_t at24addr;

#if AT24XX_ADDRSIZE == 2
      buf[1]   = offset & 0xff;
      buf[0]   = (offset >> 8) & 0xff;
      at24addr = priv->addr;
#else
      buf[0]   = offset & 0xff;
      at24addr = (priv->addr | ((offset >> 8) & 0x07));
#endif

      wait = CONFIG_AT24XX_TIMEOUT_MS;
      while (at24c_i2c_write(priv, at24addr, buf, AT24XX_ADDRSIZE) < 0)
        {
          finfo("wait\n");
          if (!wait--)
            {
              return -ETIMEDOUT;
            }

          nxsig_usleep(1000);
        }

      at24c_i2c_write(priv, at24addr, buf, AT24XX_PAGESIZE + AT24XX_ADDRSIZE);
    }

  return OK;
}

/************************************************************************************
 * Name: at24c_erase
 ************************************************************************************/

static int at24c_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  /* EEprom need not erase */

  return (int)nblocks;
}

/************************************************************************************
 * Name: at24c_read_internal
 ************************************************************************************/

static ssize_t at24c_read_internal(FAR struct at24c_dev_s *priv, off_t offset,
                                   size_t nbytes, FAR uint8_t *buffer,
                                   uint8_t address)
{
  uint8_t buf[AT24XX_ADDRSIZE];
  uint16_t at24addr;
  uint16_t wait;

  finfo("offset: %lu nbytes: %lu address: %02x\n",
        (unsigned long)offset, (unsigned long)nbytes, address);

  /* "Random Read: A Random Read requires a dummy byte write sequence to load in the
   *  data word address. Once the device address word and data word address are clocked
   *  in and acknowledged by the EEPROM, the microcontroller must generate another
   *  Start condition. The microcontroller now initiates a current address read
   *  by sending a device address with the read/write select bit high. The EEPROM
   *  acknowledges the device address and serially clocks out the data word. To end the
   *  random read sequence, the microcontroller does not respond with a zero but does
   *  generate a Stop condition in the subsequent clock cycle."
   *
   * A repeated START after the address is suggested, however, this simple driver
   * just performs the write as a sepate transfer with an additional, unnecessary STOP.
   */

#if AT24XX_ADDRSIZE == 2
  buf[1]   = offset & 0xff;
  buf[0]   = (offset >> 8) & 0xff;
  at24addr = address;
#else
  buf[0]   = offset & 0xff;
  at24addr = (address | ((offset >> 8) & 0x07));
#endif

  wait = CONFIG_AT24XX_TIMEOUT_MS;
  while (at24c_i2c_write(priv, at24addr, buf, AT24XX_ADDRSIZE) < 0)
    {
      finfo("wait\n");
      if (!wait--)
        {
          return -ETIMEDOUT;
        }

      nxsig_usleep(1000);
    }

  /* Then transfer the following bytes */

  at24c_i2c_read(priv, at24addr, buffer, nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: at24c_bread
 ************************************************************************************/

static ssize_t at24c_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
  off_t offset;
  ssize_t nread;
  size_t i;

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
  startblock *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
  nblocks    *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#endif

  finfo("startblock: %08lx nblocks: %lu\n",
        (unsigned long)startblock, (unsigned long)nblocks);

  if (startblock >= priv->npages)
    {
      return 0;
    }

  if (startblock + nblocks > priv->npages)
    {
      nblocks = priv->npages - startblock;
    }

  /* Convert the access from startblock and number of blocks to a byte
   * offset and number of bytes.
   */

  offset = startblock * priv->pagesize;

  /* Then perform the byte-oriented read for each block separately */

  for (i = 0; i < nblocks; i++)
    {
      nread = at24c_read_internal(priv, offset, priv->pagesize, buffer,
                                  priv->addr);
      if (nread < 0)
        {
          return nread;
        }

      offset += priv->pagesize;
      buffer += priv->pagesize;
    }

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
   return nblocks / (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#else
   return nblocks;
#endif
}

/************************************************************************************
 * Name: at24c_bwrite
 *
 * Operates on MTD block's and translates to FLASH pages
 *
 ************************************************************************************/

static ssize_t at24c_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                            FAR const uint8_t *buffer)
{
  FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
  size_t blocksleft;
  uint8_t buf[AT24XX_PAGESIZE + AT24XX_ADDRSIZE];
  uint16_t wait;

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
  startblock *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
  nblocks    *= (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#endif
  blocksleft  = nblocks;

  if (startblock >= priv->npages)
    {
      return 0;
    }

  if (startblock + nblocks > priv->npages)
    {
      nblocks = priv->npages - startblock;
    }

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      uint16_t offset = startblock * priv->pagesize;
      uint16_t at24addr;

#if AT24XX_ADDRSIZE == 2
      buf[1]   = offset & 0xff;
      buf[0]   = (offset >> 8) & 0xff;
      at24addr = priv->addr;
#else
      buf[0]   = offset & 0xff;
      at24addr = (priv->addr | ((offset >> 8) & 0x07));
#endif

      wait = CONFIG_AT24XX_TIMEOUT_MS;
      while (at24c_i2c_write(priv, at24addr, buf, AT24XX_ADDRSIZE) < 0)
        {
          finfo("wait\n");
          if (!wait--)
            {
              return -ETIMEDOUT;
            }

          nxsig_usleep(1000);
        }

      memcpy(&buf[AT24XX_ADDRSIZE], buffer, priv->pagesize);

      at24c_i2c_write(priv, at24addr, buf, priv->pagesize + AT24XX_ADDRSIZE);
      startblock++;
      buffer += priv->pagesize;
    }

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
  return nblocks / (CONFIG_AT24XX_MTD_BLOCKSIZE / AT24XX_PAGESIZE);
#else
  return nblocks;
#endif
}

/************************************************************************************
 * Name: at24c_read
 ************************************************************************************/

static ssize_t at24c_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
  size_t  memsize;
  uint8_t addr;

  finfo("offset: %lu nbytes: %lu\n", (unsigned long)offset, (unsigned long)nbytes);

  /* Don't permit reads beyond the end of the memory region */

#ifdef CONFIG_AT24XX_EXTENDED
  if (priv->extended)
    {
      memsize = CONFIG_AT24XX_EXTSIZE;
    }
  else
#endif
    {
      memsize = AT24XX_NPAGES * AT24XX_PAGESIZE;
    }

  if (offset + nbytes > memsize)
    {
      /* Return 0 meaning end-of-file */

      return 0;
    }

  /* Get the I2C address, converting it to the extended I2C if needed */

  addr = priv->addr;
#ifdef CONFIG_AT24XX_EXTENDED
  if (priv->extended)
    {
      addr |= 0x08;
    }
#endif

  /* Then perform the byte-oriented read using common logic */

  return at24c_read_internal(priv, offset, nbytes, buffer, addr);
}

/************************************************************************************
 * Name: at24c_ioctl
 ************************************************************************************/

static int at24c_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
         FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               *
               * blocksize:
               *   May be user defined. The block size for the at24XX devices may be
               *   larger than the page size in order to better support file systems.
               *   The read and write functions translate BLOCKS to pages for the
               *   small flash devices
               * erasesize:
               *   It has to be at least as big as the blocksize, bigger serves no
               *   purpose.
               * neraseblocks
               *   Note that the device size is in kilobits and must be scaled by
               *   1024 / 8
               */

#if CONFIG_AT24XX_MTD_BLOCKSIZE > AT24XX_PAGESIZE
              geo->blocksize    = CONFIG_AT24XX_MTD_BLOCKSIZE;
              geo->erasesize    = CONFIG_AT24XX_MTD_BLOCKSIZE;
              geo->neraseblocks = (CONFIG_AT24XX_SIZE * 1024 / 8) / CONFIG_AT24XX_MTD_BLOCKSIZE;
#else
              geo->blocksize    = priv->pagesize;
              geo->erasesize    = priv->pagesize;
              geo->neraseblocks = priv->npages;
#endif
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = at24c_eraseall(priv);
        break;

#ifdef CONFIG_AT24XX_EXTENDED
      case MTDIOC_EXTENDED:
        priv->extended = (arg != 0);
        ret = OK;
        break;
#endif

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at24c_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

#ifdef CONFIG_AT24XX_MULTI
FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_master_s *dev, uint8_t address)
#else
FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_master_s *dev)
#endif
{
  FAR struct at24c_dev_s *priv;

#ifdef CONFIG_AT24XX_MULTI
  finfo("dev: %p address: %02x\n", dev, address);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per I2C
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same I2C bus.
   */

  priv = (FAR struct at24c_dev_s *)kmm_zalloc(sizeof(struct at24c_dev_s));
  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate device structure\n");
      return NULL;
    }

#else
  finfo("dev: %p\n", dev);

  /* If only a signal AT24 part is supported then a statically allocated state
   * structure is used.
   */

  priv = &g_at24c;
#endif

  if (!priv->initd)
    {
      /* Initialize the allocated structure */

#ifdef CONFIG_AT24XX_MULTI
      priv->addr       = address;
#else
      priv->addr       = CONFIG_AT24XX_ADDR;
#endif
      priv->pagesize   = AT24XX_PAGESIZE;
      priv->npages     = AT24XX_NPAGES;

      /* NOTE since the pre-allocated device structure lies in .bss, any
       * unsupported methods should have been nullified.
       */

      priv->mtd.erase  = at24c_erase;
      priv->mtd.bread  = at24c_bread;
      priv->mtd.bwrite = at24c_bwrite;
      priv->mtd.read   = at24c_read;
      priv->mtd.ioctl  = at24c_ioctl;
      priv->mtd.name   = "at24xx";
      priv->dev        = dev;
      priv->initd      = true;
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}

/************************************************************************************
 * Name: at24c_uninitialize
 *
 * Description:
 *   Release resources held by an allocated MTD device instance.  Resources are only
 *   allocated for the case where multiple AT24xx devices are support.
 *
 ************************************************************************************/

#ifdef CONFIG_AT24XX_MULTI
void at24c_uninitialize(FAR struct mtd_dev_s *mtd)
{
  FAR struct at24c_dev_s *priv = (FAR struct at24c_dev_s *)mtd;
  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_MTD_REGISTRATION
  /* Unregister the MTD with the procfs system if enabled */

  mtd_unregister(&priv->mtd);
#endif

  /* Free the MTD driver instance */

  kmm_free(priv);
}
#endif /* CONFIG_AT24XX_MULTI */
#endif /* CONFIG_MTD_AT24XX */
