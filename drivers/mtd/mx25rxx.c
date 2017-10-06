/************************************************************************************
 * drivers/mtd/mx25rxx.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>

 *   Derived from QuadSPI-based N25QxxxA driver (drivers/mtd/n25qxxx.c)
 *   Author: dev@ziggurat29.com
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
#include <errno.h>
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

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
#define MX25R_JEDEC_MEMORY_TYPE          0x28  /* MX25Rx memory type */
#define MX25R_JEDEC_MX25R6435F_CAPACITY  0x17  /* MX25R6435F memory capacity */
#define MX25R_JEDEC_MX25R8035F_CAPACITY  0x14  /* MX25R8035F memory capacity */

/* Supported chips parameters */

/* MX25R6435F (64 MB) memory capacity */

#define MX25R6435F_SECTOR_SIZE      (4*1024)
#define MX25R6435F_SECTOR_SHIFT     (12)
#define MX25R6435F_SECTOR_COUNT     (16384)
#define MX25R6435F_PAGE_SIZE        (256)
#define MX25R6435F_PAGE_SHIFT       (8)

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

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Internal state of the MTD device */

struct mx25rxx_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* QuadSPI interface */

  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */

  uint8_t                sectorshift; /* 16 or 18 */
  uint8_t                pageshift;   /* 8 */
  uint16_t               nsectors;    /* 128 or 64 */
};

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/* MTD driver methods */

static int mx25rxx_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks);
static ssize_t mx25rxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
static ssize_t mx25rxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf);
static ssize_t mx25rxx_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer);
static int mx25rxx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

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
                             FAR uint8_t *buffer, off_t address, size_t buflen);
static int mx25rxx_read_status(FAR struct mx25rxx_dev_s *dev);
static int mx25rxx_read_configuration(FAR struct mx25rxx_dev_s *dev);
static void mx25rxx_write_status_config(FAR struct mx25rxx_dev_s *dev,
                                        uint8_t status, uint16_t config);
static void mx25rxx_write_enable(FAR struct mx25rxx_dev_s *dev, bool enable);

static int mx25rxx_write_page(struct mx25rxx_dev_s *priv,
                       FAR const uint8_t *buffer, off_t address, size_t buflen);
static int mx25rxx_erase_sector(struct mx25rxx_dev_s *priv, off_t sector);
static int mx25rxx_erase_block(struct mx25rxx_dev_s *priv, off_t block);
static int mx25rxx_erase_chip(struct mx25rxx_dev_s *priv);

/******************************************************************************
 * Private Functions
 ******************************************************************************/

void mx25rxx_lock(FAR struct qspi_dev_s *qspi, bool read)
{
  /* On SPI busses where there are multiple devices, it will be necessary to
   * lock SPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access
   * to the SPI buss.  We will retain that exclusive access until the bus is
   * unlocked.
   */

  (void)QSPI_LOCK(qspi, true);

  /* After locking the SPI bus, the we also need call the setfrequency, setbits
   * and setmode methods to make sure that the SPI is properly configured for
   * the device.  If the SPI buss is being shared, then it may have been left
   * in an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_MX25RXX_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  (void)QSPI_SETFREQUENCY(qspi,
     read ? CONFIG_MX25RXX_QSPI_READ_FREQUENCY : CONFIG_MX25RXX_QSPI_FREQUENCY);
}

void mx25rxx_unlock(FAR struct qspi_dev_s *qspi)
{
  (void)QSPI_LOCK(qspi, false);
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

  finfo("CMD: %02x buflen: %lu 0x%x\n",
        cmd, (unsigned long)buflen, *(uint32_t*)buffer);

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

  /* Ignore performace enhanced mode => 2+4 dummies */

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
          ferr("ERROR: QSPI_MEMORY failed writing address=%06x\n",
               address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
    }

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
      nxsig_usleep(50*1000);
      mx25rxx_read_status(priv);
      status = priv->cmdbuf[0];
    }
  while ((status & MX25R_SR_WIP) != 0);

  return OK;
}

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
      nxsig_usleep(300*1000);
      mx25rxx_read_status(priv);
      status = priv->cmdbuf[0];
    }
  while ((status & MX25R_SR_WIP) != 0);

  return OK;
}

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

void mx25rxx_write_status_config(FAR struct mx25rxx_dev_s *dev, uint8_t status,
                                 uint16_t config)
{
  mx25rxx_write_enable(dev, true);

  /* take care to mask of the SRP bit; it is one-time-programmable */

  config &= ~MX25R_CR_TB;

  dev->cmdbuf[0] = status | 2;
  dev->cmdbuf[1] = config & 0xff;
  dev->cmdbuf[2] = config >> 8;

  mx25rxx_command_write(dev->qspi, MX25R_WRSR, dev->cmdbuf, 3);
  mx25rxx_write_enable(dev, false);
}

int mx25rxx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
  size_t blocksleft = nblocks;
  unsigned int sectorsPerBlock = (64*1024)>>priv->sectorshift;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  mx25rxx_lock(priv->qspi, false);

  while (blocksleft > 0)
    {
      /* Check if current block is aligned on 64k block to speed up erase */

      if (((startblock & (sectorsPerBlock-1)) == 0) &&
          (blocksleft >= sectorsPerBlock))
        {
          /* Erase 64k block */

          mx25rxx_erase_block(priv, startblock>>(16-priv->sectorshift));
          startblock += sectorsPerBlock;
          blocksleft -= sectorsPerBlock;
        }
      else
        {
          /* Erase each sector */

          mx25rxx_erase_sector(priv, startblock);
          startblock ++;
          blocksleft --;
        }
    }

  mx25rxx_unlock(priv->qspi);

  return (int)nblocks;
}

ssize_t mx25rxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks, FAR uint8_t *buf)
{
  FAR struct mx25rxx_dev_s *mx_dev = (FAR struct mx25rxx_dev_s*)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  nbytes = mx25rxx_read(dev, startblock << mx_dev->pageshift,
                        nblocks << mx_dev->pageshift, buf);
  if (nbytes > 0)
    {
      nbytes >>= mx_dev->pageshift;
    }

  return nbytes;
}

ssize_t mx25rxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct mx25rxx_dev_s *priv = (FAR struct mx25rxx_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  mx25rxx_lock(priv->qspi, false);

  ret = mx25rxx_write_page(priv, buf, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: mx25rxx_write_page failed: %d\n", ret);
    }

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

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
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

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: mx25rxx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ******************************************************************************/

FAR struct mtd_dev_s *mx25rxx_initialize(FAR struct qspi_dev_s *qspi, bool unprotect)
{
  FAR struct mx25rxx_dev_s *dev;
  int ret;
  uint8_t status;
  uint16_t config;

  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same QuadSPI bus.
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

  mx25rxx_lock(dev->qspi, false);

  /* Set MTD device in low power mode, with minimum dummy cycles */

  mx25rxx_write_status_config(dev, MX25R_SR_QE, 0x0000);

  mx25rxx_read_status(dev);
  status = dev->cmdbuf[0];
  mx25rxx_read_configuration(dev);
  config = *(uint16_t*)(dev->cmdbuf);

  /* FIXME avoid compiler warnings in case info logs are disabled */
  (void)status;
  (void)config;

  finfo("device ready 0x%02x 0x%04x\n", status, config);

  mx25rxx_unlock(dev->qspi);

#ifdef CONFIG_MTD_REGISTRATION
  /* Register the MTD with the procfs system if enabled */

  mtd_register(&dev->mtd, "mx25rxx");
#endif

  /* Return the implementation-specific state structure as the MTD device */

  return &dev->mtd;

exit_free_cmdbuf:
  QSPI_FREE(qspi, dev->cmdbuf);
exit_free_dev:
  kmm_free(dev);
  return NULL;
}
