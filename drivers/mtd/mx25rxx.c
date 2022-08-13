/****************************************************************************
 * drivers/mtd/mx25rxx.c
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
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_MX25RXX_SECTOR512
#  include <stdlib.h>
#  include <string.h>
#endif

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MX25RXX Commands */

#define MX25R_READ        0x03  /* Read data bytes          */
#define MX25R_FAST_READ   0x0b  /* Higher speed read        */
#define MX25R_2READ       0xbb  /* 2 x I/O read command     */
#define MX25R_DREAD       0x3b  /* 1I / 2O read command     */
#define MX25R_4READ       0xeb  /* 4 x I/O read command     */
#define MX25R_QREAD       0x6b  /* 1I / 4O read command     */
#define MX25R_PP          0x02  /* Page program             */
#define MX25R_4PP         0x38  /* Quad page program        */

#define MX25R_SE          0x20  /* 4Kb Sector erase         */
#define MX25R_BE32        0x52  /* 32Kbit block Erase       */
#define MX25R_BE64        0xd8  /* 64Kbit block Erase       */
#define MX25R_CE          0xc7  /* Chip erase               */
#define MX25R_CE_ALT      0x60  /* Chip erase (alternate)   */

#define MX25R_WREN        0x06  /* Write Enable             */
#define MX25R_WRDI        0x04  /* Write Disable            */
#define MX25R_RDSR        0x05  /* Read status register     */
#define MX25R_RDCR        0x15  /* Read config register     */
#define MX25R_WRSR        0x01  /* Write stat/conf register */

#define MX25R_RDID        0x9f  /* Read identification      */
#define MX25R_RES         0xab  /* Read electronic ID       */
#define MX25R_REMS        0x90  /* Read manufacture and ID  */

#define MX25R_DP          0xb9  /* Deep power down          */
#define MX25R_RDP         0xab  /* Release deep power down  */
#define MX25R_PGM_SUSPEND 0x75  /* Suspends program         */
#define MX25R_ERS_SUSPEND 0xb0  /* Suspends erase           */
#define MX25R_PGM_RESUME  0x7A  /* Resume program           */
#define MX25R_ERS_RESUME  0x30  /* Resume erase             */
#define MX25R_ENSO        0xb1  /* Enter secured OTP        */
#define MX25R_EXSO        0xc1  /* Exit secured OTP         */
#define MX25R_RDSCUR      0x2b  /* Read security register   */
#define MX25R_WRSCUR      0x2f  /* Write security register  */
#define MX25R_RSTEN       0x66  /* Reset Enable             */
#define MX25R_RST         0x99  /* Reset Memory             */
#define MX25R_RDSFDP      0x5a  /* read out until CS# high  */
#define MX25R_SBL         0xc0  /* Set Burst Length         */
#define MX25R_SBL_ALT     0x77  /* Set Burst Length         */
#define MX25R_NOP         0x00  /* No Operation             */

/* MX25Rxx Registers */

/* Read ID (RDID) register values */

#define MX25R_MANUFACTURER          0xc2  /* Macronix manufacturer ID */
#define MX25R6435F_DEVID            0x17  /* MX25R6435F device ID */

/* JEDEC Read ID register values */

#define MX25R_JEDEC_MANUFACTURER         0xc2  /* Macronix manufacturer ID */
#ifdef CONFIG_MX25RXX_LXX
#  define MX25R_JEDEC_MEMORY_TYPE          0x20  /* MX25Lx memory type */
#else
#  define MX25R_JEDEC_MEMORY_TYPE          0x28  /* MX25Rx memory type */
#endif
#define MX25R_JEDEC_MX25R6435F_CAPACITY  0x17  /* MX25R6435F memory capacity */
#define MX25R_JEDEC_MX25R8035F_CAPACITY  0x14  /* MX25R8035F memory capacity */

/* Supported chips parameters */

/* MX25R6435F (64 MB) memory capacity */

#define MX25R6435F_SECTOR_SIZE      (4*1024)
#define MX25R6435F_SECTOR_SHIFT     (12)
#define MX25R6435F_SECTOR_COUNT     (2048)
#define MX25R6435F_PAGE_SIZE        (256)

#ifdef CONFIG_MX25RXX_PAGE128
#  define MX25R6435F_PAGE_SHIFT       (7)
#else
#  define MX25R6435F_PAGE_SHIFT       (8)
#endif

/* Status register bit definitions */

#define MX25R_SR_WIP                (1 << 0)  /* Bit 0: Write in progress */
#define MX25R_SR_WEL                (1 << 1)  /* Bit 1: Write enable latch */
#define MX25R_SR_BP_SHIFT           (2)       /* Bits 2-5: Block protect bits */
#define MX25R_SR_BP_MASK            (15 << MX25R_SR_BP_SHIFT)
#define MX25R_SR_QE                 (1 << 6)  /* Bit 6: Quad enable */
#define MX25R_SR_SRWD               (1 << 7)  /* Bit 7: Status register write protect */

/* Configuration registerregister bit definitions */

#define MX25R_CR_LH                 (1 << 9)  /* Bit 9: Power mode */
#define MX25R_CR_TB                 (1 << 3)  /* Bit 3: Top/bottom selected */
#define MX25R_CR_DC                 (1 << 6)  /* Bit 6: Dummy cycle */

/* Cache flags **************************************************************/

#define MX25RXX_CACHE_VALID         (1 << 0)  /* 1=Cache has valid data */
#define MX25RXX_CACHE_DIRTY         (1 << 1)  /* 1=Cache is dirty */
#define MX25RXX_CACHE_ERASED        (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                 ((((p)->flags) & MX25RXX_CACHE_VALID) != 0)
#define IS_DIRTY(p)                 ((((p)->flags) & MX25RXX_CACHE_DIRTY) != 0)
#define IS_ERASED(p)                ((((p)->flags) & MX25RXX_CACHE_ERASED) != 0)

#define SET_VALID(p)                do { (p)->flags |= MX25RXX_CACHE_VALID; } while (0)
#define SET_DIRTY(p)                do { (p)->flags |= MX25RXX_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)               do { (p)->flags |= MX25RXX_CACHE_ERASED; } while (0)

#define CLR_VALID(p)                do { (p)->flags &= ~MX25RXX_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)                do { (p)->flags &= ~MX25RXX_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)               do { (p)->flags &= ~MX25RXX_CACHE_ERASED; } while (0)

/* 512 byte sector support **************************************************/

#define MX25RXX_SECTOR512_SHIFT     9
#define MX25RXX_SECTOR512_SIZE      (1 << 9)
#define MX25RXX_ERASED_STATE        0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Internal state of the MTD device */

struct mx25rxx_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* QuadSPI interface */

  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */

  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  uint16_t               nsectors;    /* Number of erase sectors */

#ifdef CONFIG_MX25RXX_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int mx25rxx_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks);
static ssize_t mx25rxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
static ssize_t mx25rxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf);
static ssize_t mx25rxx_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer);
static int mx25rxx_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg);

/* Internal driver methods */

static void mx25rxx_lock(FAR struct qspi_dev_s *qspi, bool read);
static void mx25rxx_unlock(FAR struct qspi_dev_s *qspi);
static int mx25rxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                FAR void *buffer, size_t buflen);
static int mx25rxx_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                 FAR const void *buffer, size_t buflen);
static int mx25rxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int mx25rxx_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                  off_t addr, uint8_t addrlen);

static int mx25rxx_readid(struct mx25rxx_dev_s *dev);
static int mx25rxx_read_byte(FAR struct mx25rxx_dev_s *dev,
                             FAR uint8_t *buffer, off_t address,
                             size_t buflen);
static int mx25rxx_read_status(FAR struct mx25rxx_dev_s *dev);
static int mx25rxx_read_configuration(FAR struct mx25rxx_dev_s *dev);
static void mx25rxx_write_status_config(FAR struct mx25rxx_dev_s *dev,
                                        uint8_t status, uint16_t config);
static void mx25rxx_write_enable(FAR struct mx25rxx_dev_s *dev, bool enable);

static int mx25rxx_write_page(struct mx25rxx_dev_s *priv,
                              FAR const uint8_t *buffer,
                              off_t address,
                              size_t buflen);
static int mx25rxx_erase_sector(struct mx25rxx_dev_s *priv, off_t sector);
#if 0 /* FIXME:  Not used */
static int mx25rxx_erase_block(struct mx25rxx_dev_s *priv, off_t block);
#endif
static int mx25rxx_erase_chip(struct mx25rxx_dev_s *priv);

#ifdef CONFIG_MX25RXX_SECTOR512
static int  mx25rxx_flush_cache(struct mx25rxx_dev_s *priv);
static FAR uint8_t *mx25rxx_read_cache(struct mx25rxx_dev_s *priv,
                                       off_t sector);
static void mx25rxx_erase_cache(struct mx25rxx_dev_s *priv, off_t sector);
static int  mx25rxx_write_cache(FAR struct mx25rxx_dev_s *priv,
              FAR const uint8_t *buffer,  off_t sector, size_t nsectors);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void mx25rxx_lock(FAR struct qspi_dev_s *qspi, bool read)
{
  /* On SPI buses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive
   * access to the SPI bus. We will retain that exclusive access until the
   * bus is unlocked.
   */

  QSPI_LOCK(qspi, true);

  /* After locking the SPI bus, the we also need call the setfrequency,
   * setbits and setmode methods to make sure that the SPI is properly
   * configured for the device.  If the SPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_MX25RXX_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  QSPI_SETFREQUENCY(qspi,
     read ? CONFIG_MX25RXX_QSPI_READ_FREQUENCY :
            CONFIG_MX25RXX_QSPI_FREQUENCY);
}

void mx25rxx_unlock(FAR struct qspi_dev_s *qspi)
{
  QSPI_LOCK(qspi, false);
}

int mx25rxx_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                         FAR void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_READDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

int mx25rxx_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                          FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu 0x%" PRIx32 "\n",
        cmd, (unsigned long)buflen, *(FAR uint32_t *)buffer);

  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

int mx25rxx_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

int mx25rxx_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                            off_t addr, uint8_t addrlen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x Address: %04lx addrlen=%d\n",
        cmd, (unsigned long)addr, addrlen);

  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = addr;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

int mx25rxx_read_byte(FAR struct mx25rxx_dev_s *dev, FAR uint8_t *buffer,
                      off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = 3;

  /* Ignore performance enhanced mode => 2+4 dummies */

  meminfo.dummies = 6;
  meminfo.buflen  = buflen;
  meminfo.cmd     = MX25R_4READ;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(dev->qspi, &meminfo);
}

int mx25rxx_write_page(struct mx25rxx_dev_s *priv, FAR const uint8_t *buffer,
                       off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  int i;

  finfo("address: %08lx buflen: %u\n",
        (unsigned long)address, (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADIO;
  meminfo.cmd     = MX25R_4PP;
  meminfo.addrlen = 3;
  meminfo.buflen  = pagesize;
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      mx25rxx_write_enable(priv, true);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      mx25rxx_write_enable(priv, false);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06jx\n",
               (intmax_t)address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
    }

  /* Wait for write operation to finish */

  do
    {
      mx25rxx_read_status(priv);
      ret = priv->cmdbuf[0];
    }
  while ((ret & MX25R_SR_WIP) != 0);

  return OK;
}

int mx25rxx_erase_sector(struct mx25rxx_dev_s *priv, off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  /* Send the sector erase command */

  mx25rxx_write_enable(priv, true);
  mx25rxx_command_address(priv->qspi, MX25R_SE, address, 3);

  /* Wait for erasure to finish */

  do
    {
      nxsig_usleep(50 * 1000);
      mx25rxx_read_status(priv);
      status = priv->cmdbuf[0];
    }
  while ((status & MX25R_SR_WIP) != 0);

  return OK;
}

#if 0 /* FIXME:  Not used */
int mx25rxx_erase_block(struct mx25rxx_dev_s *priv, off_t block)
{
  uint8_t status;

  finfo("block: %08lx\n", (unsigned long)block);

  /* Send the 64k block erase command */

  mx25rxx_write_enable(priv, true);
  mx25rxx_command_address(priv->qspi, MX25R_BE64, block << 16, 3);

  /* Wait for erasure to finish */

  do
    {
      nxsig_usleep(300 * 1000);
      mx25rxx_read_status(priv);
      status = priv->cmdbuf[0];
    }
  while ((status & MX25R_SR_WIP) != 0);

  return OK;
}
#endif

int mx25rxx_erase_chip(struct mx25rxx_dev_s *priv)
{
  uint8_t status;

  /* Erase the whole chip */

  mx25rxx_write_enable(priv, true);
  mx25rxx_command(priv->qspi, MX25R_CE);

  /* Wait for the erasure to complete */

  mx25rxx_read_status(priv);
  status = priv->cmdbuf[0];

  while ((status & MX25R_SR_WIP) != 0)
    {
      nxsig_sleep(2);
      mx25rxx_read_status(priv);
      status = priv->cmdbuf[0];
    }

  return OK;
}

void mx25rxx_write_enable(FAR struct mx25rxx_dev_s *dev, bool enable)
{
  uint8_t status;

  do
    {
      mx25rxx_command(dev->qspi, enable ? MX25R_WREN : MX25R_WRDI);
      mx25rxx_read_status(dev);
      status = dev->cmdbuf[0];
    }
  while ((status & MX25R_SR_WEL) ^ (enable ? MX25R_SR_WEL : 0));
}

int mx25rxx_read_status(FAR struct mx25rxx_dev_s *dev)
{
  return mx25rxx_command_read(dev->qspi, MX25R_RDSR, dev->cmdbuf, 1);
}

int mx25rxx_read_configuration(FAR struct mx25rxx_dev_s *dev)
{
  return mx25rxx_command_read(dev->qspi, MX25R_RDCR, dev->cmdbuf, 4);
}

void mx25rxx_write_status_config(FAR struct mx25rxx_dev_s *dev,
                                 uint8_t status,
                                 uint16_t config)
{
  mx25rxx_write_enable(dev, true);

  /* take care to mask of the SRP bit; it is one-time-programmable */

  config &= ~MX25R_CR_TB;

  dev->cmdbuf[0] = status | 2;
  dev->cmdbuf[1] = config & 0xff;
  dev->cmdbuf[2] = config >> 8;

#ifdef CONFIG_MX25RXX_LXX
  mx25rxx_command_write(dev->qspi, MX25R_WRSR, dev->cmdbuf, 2);
#else
  mx25rxx_command_write(dev->qspi, MX25R_WRSR, dev->cmdbuf, 3);
#endif
  mx25rxx_write_enable(dev, false);
}

int mx25rxx_erase(FAR struct mtd_dev_s *dev,
                  off_t startblock,
                  size_t nblocks)
{
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_MX25RXX_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mx25rxx_lock(priv->qspi, false);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_MX25RXX_SECTOR512
      mx25rxx_erase_cache(priv, startblock);
#else
      mx25rxx_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_MX25RXX_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = mx25rxx_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

#if 0
  /* FIXME: use mx25rxx_erase_block in case CONFIG_MX25RXX_SECTOR512 is
   * not configured to speed up block erase.
   */

  unsigned int sectorsperblock = (64 * 1024) >> priv->sectorshift;
  while (blocksleft > 0)
    {
      /* Check if current block is aligned on 64k block to speed up erase */

      if (((startblock & (sectorsperblock - 1)) == 0) &&
          (blocksleft >= sectorsperblock))
        {
          /* Erase 64k block */

          mx25rxx_erase_block(priv, startblock >> (16 - priv->sectorshift));
          startblock += sectorsperblock;
          blocksleft -= sectorsperblock;
        }
      else
        {
          /* Erase each sector */

          mx25rxx_erase_sector(priv, startblock);
          startblock++;
          blocksleft--;
        }
    }
#endif

  mx25rxx_unlock(priv->qspi);

  return (int)nblocks;
}

ssize_t mx25rxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks, FAR uint8_t *buf)
{
#ifndef CONFIG_MX25RXX_SECTOR512
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the
   * byte-oriented read
   */

#ifdef CONFIG_MX25RXX_SECTOR512
  nbytes = mx25rxx_read(dev, startblock << MX25RXX_SECTOR512_SHIFT,
                       nblocks << MX25RXX_SECTOR512_SHIFT, buf);
  if (nbytes > 0)
    {
      nbytes >>= MX25RXX_SECTOR512_SHIFT;
    }
#else
  nbytes = mx25rxx_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buf);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

ssize_t mx25rxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
  int ret;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  mx25rxx_lock(priv->qspi, false);

#if defined(CONFIG_MX25RXX_SECTOR512)
  ret = mx25rxx_write_cache(priv, buf, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: mx25rxx_write_cache failed: %d\n", ret);
    }

#else
  ret = mx25rxx_write_page(priv, buf, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: mx25rxx_write_page failed: %d\n", ret);
    }
#endif

  mx25rxx_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

ssize_t mx25rxx_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                     FAR uint8_t *buffer)
{
  int ret;
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  mx25rxx_lock(priv->qspi, true);
  ret = mx25rxx_read_byte(priv, buffer, offset, nbytes);
  mx25rxx_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: mx25rxx_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

int mx25rxx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an
               * array of fixed size blocks.  That is most likely not true,
               * but the client will expect the device logic to do whatever
               * is necessary to make it appear so.
               */

#ifdef CONFIG_MX25RXX_SECTOR512
              geo->blocksize    = (1 << MX25RXX_SECTOR512_SHIFT);
              geo->erasesize    = (1 << MX25RXX_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                  (priv->sectorshift -
                                  MX25RXX_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              finfo("blocksize: %" PRId32
                    " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
#ifdef CONFIG_MX25RXX_SECTOR512
              info->numsectors  = priv->nsectors <<
                               (priv->sectorshift - MX25RXX_SECTOR512_SHIFT);
              info->sectorsize  = 1 << MX25RXX_SECTOR512_SHIFT;
#else
              info->numsectors  = priv->nsectors <<
                                  (priv->sectorshift - priv->pageshift);
              info->sectorsize  = 1 << priv->pageshift;
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

          mx25rxx_lock(priv->qspi, false);
          ret = mx25rxx_erase_chip(priv);
          mx25rxx_unlock(priv->qspi);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = MX25RXX_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

int mx25rxx_readid(struct mx25rxx_dev_s *dev)
{
  /* Lock the QuadSPI bus and configure the bus. */

  mx25rxx_lock(dev->qspi, false);

  /* Read the JEDEC ID */

  mx25rxx_command_read(dev->qspi, MX25R_RDID, dev->cmdbuf, 3);

  /* Unlock the bus */

  mx25rxx_unlock(dev->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        dev->cmdbuf[0], dev->cmdbuf[1], dev->cmdbuf[2]);

  /* Check for Macronix MX25Rxx chip */

  if (dev->cmdbuf[0] != MX25R_JEDEC_MANUFACTURER ||
      dev->cmdbuf[1] != MX25R_JEDEC_MEMORY_TYPE)
    {
      ferr("ERROR: Unrecognized device type: 0x%02x 0x%02x\n",
           dev->cmdbuf[0], dev->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (dev->cmdbuf[2])
    {
      case MX25R_JEDEC_MX25R6435F_CAPACITY:
        dev->sectorshift = MX25R6435F_SECTOR_SHIFT;
        dev->pageshift   = MX25R6435F_PAGE_SHIFT;
        dev->nsectors    = MX25R6435F_SECTOR_COUNT;
        break;

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", dev->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: mx25rxx_flush_cache
 ****************************************************************************/

#ifdef CONFIG_MX25RXX_SECTOR512
static int mx25rxx_flush_cache(struct mx25rxx_dev_s *priv)
{
  int ret = OK;

  /* If the cache is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = mx25rxx_write_page(priv,
                               priv->sector,
                               address,
                               1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: mx25rxx_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif /* CONFIG_MX25RXX_SECTOR512 */

/****************************************************************************
 * Name: mx25rxx_read_cache
 ****************************************************************************/

#ifdef CONFIG_MX25RXX_SECTOR512
static FAR uint8_t *mx25rxx_read_cache(struct mx25rxx_dev_s *priv,
                                       off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.
   * For example, if the actual erase sector size is 4Kb (1 << 12), then we
   * first shift to the right by 3 to get the sector number in 4096
   * increments.
   */

  shift    = priv->sectorshift - MX25RXX_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %jd esectno: %jd (%d) shift=%d\n",
        (intmax_t)sector, (intmax_t)esectno, priv->esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = mx25rxx_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: mx25rxx_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = mx25rxx_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: mx25rxx_read_byte failed: %d\n", ret);
          return NULL;
        }

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << MX25RXX_SECTOR512_SHIFT];
}
#endif /* CONFIG_MX25RXX_SECTOR512 */

/****************************************************************************
 * Name: mx25rxx_erase_cache
 ****************************************************************************/

#ifdef CONFIG_MX25RXX_SECTOR512
static void mx25rxx_erase_cache(struct mx25rxx_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = mx25rxx_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >>
                      (priv->sectorshift - MX25RXX_SECTOR512_SHIFT);
      finfo("sector: %jd esectno: %jd\n",
            (intmax_t)sector, (intmax_t)esectno);

      DEBUGVERIFY(mx25rxx_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as
   * dirty (but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, MX25RXX_ERASED_STATE, MX25RXX_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif /* CONFIG_MX25RXX_SECTOR512 */

/****************************************************************************
 * Name: mx25rxx_write_cache
 ****************************************************************************/

#ifdef CONFIG_MX25RXX_SECTOR512
static int mx25rxx_write_cache(FAR struct mx25rxx_dev_s *priv,
                              FAR const uint8_t *buffer, off_t sector,
                              size_t nsectors)
{
  FAR uint8_t *dest;
  int ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = mx25rxx_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                           (priv->sectorshift - MX25RXX_SECTOR512_SHIFT);
          finfo("sector: %jd esectno: %jd\n",
                (intmax_t)sector, (intmax_t)esectno);

          ret = mx25rxx_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: mx25rxx_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, MX25RXX_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      finfo("address: %08jx nbytes: %d 0x%04" PRIx32 "\n",
            (intmax_t)(sector << MX25RXX_SECTOR512_SHIFT),
            MX25RXX_SECTOR512_SIZE,
            *(FAR uint32_t *)buffer);
      buffer += MX25RXX_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return mx25rxx_flush_cache(priv);
}
#endif /* CONFIG_MX25RXX_SECTOR512 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx25rxx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mx25rxx_initialize(FAR struct qspi_dev_s *qspi,
                                         bool unprotect)
{
  FAR struct mx25rxx_dev_s *dev;
  int ret;
  uint8_t status;
  uint16_t config;

  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  dev = (FAR struct mx25rxx_dev_s *)kmm_zalloc(sizeof(*dev));

  if (dev == NULL)
    {
      ferr("Failed to allocate mtd device\n");
      return NULL;
    }

  dev->mtd.erase  = mx25rxx_erase;
  dev->mtd.bread  = mx25rxx_bread;
  dev->mtd.bwrite = mx25rxx_bwrite;
  dev->mtd.read   = mx25rxx_read;
  dev->mtd.ioctl  = mx25rxx_ioctl;
  dev->mtd.name   = "mx25rxx";
  dev->qspi       = qspi;

  /* Allocate a 4-byte buffer to support DMA-able command data */

  dev->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
  if (dev->cmdbuf == NULL)
    {
      ferr("Failed to allocate command buffer\n");
      goto exit_free_dev;
    }

  /* Identify the FLASH chip and get its capacity */

  ret = mx25rxx_readid(dev);
  if (ret != OK)
    {
      /* Unrecognized! Discard all of that work we just did and return NULL */

      ferr("Unrecognized QSPI device\n");
      goto exit_free_cmdbuf;
    }

#ifdef CONFIG_MX25RXX_SECTOR512  /* Simulate a 512 byte sector */
  /* Allocate a buffer for the erase block cache */

  dev->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, 1 << dev->sectorshift);
  if (dev->sector == NULL)
    {
      /* Allocation failed! Discard all of that work we just did and
       * return NULL
       */

      ferr("ERROR: Sector allocation failed\n");
      goto exit_free_cmdbuf;
    }
#endif

  mx25rxx_lock(dev->qspi, false);

  /* Set MTD device in low power mode, with minimum dummy cycles */

  mx25rxx_write_status_config(dev, MX25R_SR_QE, 0x0000);

  mx25rxx_read_status(dev);
  status = dev->cmdbuf[0];
  mx25rxx_read_configuration(dev);
  config = *(FAR uint16_t *)(dev->cmdbuf);

  /* Avoid compiler warnings in case info logs are disabled */

  UNUSED(status);
  UNUSED(config);

  finfo("device ready 0x%02x 0x%04x\n", status, config);

  mx25rxx_unlock(dev->qspi);

  /* Return the implementation-specific state structure as the MTD device */

  return &dev->mtd;

exit_free_cmdbuf:
  QSPI_FREE(qspi, dev->cmdbuf);
exit_free_dev:
  kmm_free(dev);
  return NULL;
}
