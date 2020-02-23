/************************************************************************************
 * drivers/mtd/w25qxxxjv.c
 * Driver for QuadSPI-based W25QxxxJV NOR FLASH
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: kyChu <kyChu@qq.com>
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* QuadSPI Mode.  Per data sheet, either Mode 0 or Mode 3 may be used. */

#ifndef CONFIG_W25QXXXJV_QSPIMODE
#  define CONFIG_W25QXXXJV_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency per data sheet:
 *
 * In this implementation, only "Quad" reads are performed.
 */

#ifndef CONFIG_W25QXXXJV_QSPI_FREQUENCY
/* If you haven't specified frequency, default to 100 MHz which will work with all
 * commands. up to 133MHz.
 */

#  define CONFIG_W25QXXXJV_QSPI_FREQUENCY 100000000
#endif

#ifndef CONFIG_W25QXXXJV_DUMMIES
/* If you haven't specified the number of dummy cycles for quad reads, provide a
 * reasonable default.  The actual number of dummies needed is clock and IO command
 * dependent. (four to six times according to data sheet)
 */

#  define CONFIG_W25QXXXJV_DUMMIES 6
#endif

/* W25QXXXJV Commands *****************************************************************/

/* Configuration, Status, Erase, Program Commands *************************************/
/*      Command                    Value    Description:                              */
/*                                            Data sequence                           */
                                           /* Read status registers-1/2/3:            */
#define W25QXXXJV_READ_STATUS_1      0x05  /* SRP|SEC|TB |BP2|BP1|BP0|WEL|BUSY        */
#define W25QXXXJV_READ_STATUS_2      0x35  /* SUS|CMP|LB3|LB2|LB1|(R)|QE |SRL         */
#define W25QXXXJV_READ_STATUS_3      0x15  /* HOLD/RST|DRV1|DRV0|(R)|(R)|WPS|(R)|(R)  */
                                           /* Write status registers-1/2/3:           */
#define W25QXXXJV_WRITE_STATUS_1     0x01  /* SRP|SEC|TB |BP2|BP1|BP0|WEL|BUSY        */
#define W25QXXXJV_WRITE_STATUS_2     0x31  /* SUS|CMP|LB3|LB2|LB1|(R)|QE |SRL         */
#define W25QXXXJV_WRITE_STATUS_3     0x11  /* HOLD/RST|DRV1|DRV0|(R)|(R)|WPS|(R)|(R)  */
#define W25QXXXJV_WRITE_ENABLE       0x06  /* Write enable                            */
#define W25QXXXJV_WRITE_DISABLE      0x04  /* Write disable                           */
#define W25QXXXJV_PAGE_PROGRAM       0x02  /* Page Program:                           *
                                            *  0x02 | A23-A16 | A15-A8 | A7-A0 | data */
#define W25QXXXJV_SECTOR_ERASE       0x20  /* Sector Erase (4 kB)                     *
                                            *  0x20 | A23-A16 | A15-A8 | A7-A0        */
#define W25QXXXJV_BLOCK_ERASE_32K    0x52  /* Block Erase (32 KB)                     *
                                            *  0x52 | A23-A16 | A15-A8 | A7-A0        */
#define W25QXXXJV_BLOCK_ERASE_64K    0xd8  /* Block Erase (64 KB)                     *
                                            *  0xd8 | A23-A16 | A15-A8 | A7-A0        */
#define W25QXXXJV_CHIP_ERASE         0x60  /* Chip Erase:                             *
                                            *  0xc7 or 0x60                           */

/* Read Commands **********************************************************************/
/*      Command                    Value    Description:                              */
/*                                            Data sequence                           */
#define W25QXXXJV_FAST_READ_QUADIO   0xeb  /* Fast Read Quad I/O:                     *
                                            *   0xeb | ADDR | data...                 */

/* Reset Commands *********************************************************************/
/*      Command                    Value    Description:                              */
/*                                            Data sequence                           */
#define W25QXXXJV_RESET_ENABLE       0x66  /* Enable Reset                            */
#define W25QXXXJV_DEVICE_RESET       0x99  /* Reset Device                            */

/* ID/Security Commands ***************************************************************/
/*      Command                    Value    Description:                              */
/*                                            Data sequence                           */
#define W25QXXXJV_JEDEC_ID           0x9f  /* JEDEC ID:                               *
                                            * 0x9f | Manufacturer | MemoryType |      *
                                            * Capacity                                */

/* Flash Manufacturer JEDEC IDs */

#define W25QXXXJV_JEDEC_ID_WINBOND   0xef  /* Winbond Serial Flash */

/* W25QXXXJV JEDIC IDs */

#define W25QXXXJVQ_JEDEC_DEVICE_TYPE 0x40  /* with QE = 1 (fixed) in Status register-2.
                                            * Backward compatible to FV family. */

#define W25QXXXJVM_JEDEC_DEVICE_TYPE 0x70  /* with QE = 0 (programmable) in Status register-2.
                                            * New device ID is used to identify JV family */

#define W25Q016_JEDEC_CAPACITY       0x15  /* W25Q016 (2 MB) memory capacity */
#define W25Q032_JEDEC_CAPACITY       0x16  /* W25Q032 (4 MB) memory capacity */
#define W25Q064_JEDEC_CAPACITY       0x17  /* W25Q064 (8 MB) memory capacity */
#define W25Q128_JEDEC_CAPACITY       0x18  /* W25Q128 (16 MB) memory capacity */
#define W25Q256_JEDEC_CAPACITY       0x19  /* W25Q256 (32 MB) memory capacity */
#define W25Q512_JEDEC_CAPACITY       0x20  /* W25Q512 (64 MB) memory capacity */
#define W25Q01_JEDEC_CAPACITY        0x21  /* W25Q01 (128 MB) memory capacity */

/* W25QXXXJV Registers ****************************************************************/
/* Status register 1 bit definitions                                                  */

#define STATUS_BUSY_MASK           (1 << 0) /* Bit 0: Device ready/busy status      */
#  define STATUS_READY             (0 << 0) /*   0 = Not Busy                       */
#  define STATUS_BUSY              (1 << 0) /*   1 = Busy                           */
#define STATUS_WEL_MASK            (1 << 1) /* Bit 1: Write enable latch status     */
#  define STATUS_WEL_DISABLED      (0 << 1) /*   0 = Not Write Enabled              */
#  define STATUS_WEL_ENABLED       (1 << 1) /*   1 = Write Enabled                  */
#define STATUS_BP_SHIFT            (2)      /* Bits 2-4: Block protect bits         */
#define STATUS_BP_MASK             (7 << STATUS_BP_SHIFT)
#  define STATUS_BP_NONE           (0 << STATUS_BP_SHIFT)
#  define STATUS_BP_ALL            (7 << STATUS_BP_SHIFT)
#define STATUS_TB_MASK             (1 << 5) /* Bit 5: Top / Bottom Protect          */
#  define STATUS_TB_TOP            (0 << 5) /*   0 = BP2-BP0 protect Top down       */
#  define STATUS_TB_BOTTOM         (1 << 5) /*   1 = BP2-BP0 protect Bottom up      */
#define STATUS_SEC_MASK            (1 << 6) /* Bit 6: SEC                           */
#  define STATUS_SEC_64KB          (0 << 6) /*   0 = Protect 64KB Blocks            */
#  define STATUS_SEC_4KB           (1 << 6) /*   1 = Protect 4KB Sectors            */
#define STATUS_SRP_MASK            (1 << 7) /* Bit 7: Status register protect 0     */
#  define STATUS_SRP_UNLOCKED      (0 << 7) /*   see blow for details               */
#  define STATUS_SRP_LOCKED        (1 << 7) /*   see blow for details               */

/* Status Register Protect (SRP, SRL)
 * SRL  SRP  /WP Status Register         Description
 *  0    0    X  Software Protection     [Factory Default] /WP pin has no
 *                                       control.
 *                                       The Status register can be written
 *                                       to after a Write Enable instruction,
 *                                       WEL=1.
 *  0    1    0  Hardware Protected      When /WP pin is low the Status
 *                                       Register locked and cannot be
 *                                       written to.
 *  0    1    1  Hardware Unprotected    When /WP pin is high the Status
 *                                       register is unlocked and can be
 *                                       written to after a Write Enable
 *                                       instruction, WEL=1.
 *  1    X    X  Power Supply Lock-Down  Status Register is protected and
 *                                       cannot be written to again until
 *                                       the next power-down, power-up cycle.
 *                                       (When SRL =1 , a power-down, power-
 *                                       up cycle will change SRL =0 state.)
 *  1    X    X  One Time Program        Status Register is permanently
 *                                       protected and cannot be written to.
 *                                       (enabled by adding prefix command
 *                                       aah, 55h)
 */

/* Chip Geometries ******************************************************************/

/* All members of the family support uniform 4K-byte 'sub sectors'; they also support
 * 64k (and sometimes 32k) 'sectors' proper, but we won't be using those here.
 */

/* W25Q016 (2 MB) memory capacity */

#define W25Q016_SECTOR_SIZE         (4 * 1024)
#define W25Q016_SECTOR_SHIFT        (12)
#define W25Q016_SECTOR_COUNT        (512)
#define W25Q016_PAGE_SIZE           (256)
#define W25Q016_PAGE_SHIFT          (8)

/* W25Q032 (4 MB) memory capacity */

#define W25Q032_SECTOR_SIZE         (4 * 1024)
#define W25Q032_SECTOR_SHIFT        (12)
#define W25Q032_SECTOR_COUNT        (1024)
#define W25Q032_PAGE_SIZE           (256)
#define W25Q032_PAGE_SHIFT          (8)

/* W25Q064 (8 MB) memory capacity */

#define W25Q064_SECTOR_SIZE         (4 * 1024)
#define W25Q064_SECTOR_SHIFT        (12)
#define W25Q064_SECTOR_COUNT        (2048)
#define W25Q064_PAGE_SIZE           (256)
#define W25Q064_PAGE_SHIFT          (8)

/* W25Q128 (16 MB) memory capacity */

#define W25Q128_SECTOR_SIZE         (4 * 1024)
#define W25Q128_SECTOR_SHIFT        (12)
#define W25Q128_SECTOR_COUNT        (4096)
#define W25Q128_PAGE_SIZE           (256)
#define W25Q128_PAGE_SHIFT          (8)

/* W25Q256 (32 MB) memory capacity */

#define W25Q256_SECTOR_SIZE         (4 * 1024)
#define W25Q256_SECTOR_SHIFT        (12)
#define W25Q256_SECTOR_COUNT        (8196)
#define W25Q256_PAGE_SIZE           (256)
#define W25Q256_PAGE_SHIFT          (8)

/* W25Q512 (64 MB) memory capacity */

#define W25Q512_SECTOR_SIZE         (4 * 1024)
#define W25Q512_SECTOR_SHIFT        (12)
#define W25Q512_SECTOR_COUNT        (16384)
#define W25Q512_PAGE_SIZE           (256)
#define W25Q512_PAGE_SHIFT          (8)

/* W25Q01 (128 MB) memory capacity */

#define W25Q01_SECTOR_SIZE          (4 * 1024)
#define W25Q01_SECTOR_SHIFT         (12)
#define W25Q01_SECTOR_COUNT         (32768)
#define W25Q01_PAGE_SIZE            (256)
#define W25Q01_PAGE_SHIFT           (8)

/* Cache flags **********************************************************************/

#define W25QXXXJV_CACHE_VALID       (1 << 0)  /* 1=Cache has valid data */
#define W25QXXXJV_CACHE_DIRTY       (1 << 1)  /* 1=Cache is dirty */
#define W25QXXXJV_CACHE_ERASED      (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                 ((((p)->flags) & W25QXXXJV_CACHE_VALID) != 0)
#define IS_DIRTY(p)                 ((((p)->flags) & W25QXXXJV_CACHE_DIRTY) != 0)
#define IS_ERASED(p)                ((((p)->flags) & W25QXXXJV_CACHE_ERASED) != 0)

#define SET_VALID(p)                do { (p)->flags |= W25QXXXJV_CACHE_VALID; } while (0)
#define SET_DIRTY(p)                do { (p)->flags |= W25QXXXJV_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)               do { (p)->flags |= W25QXXXJV_CACHE_ERASED; } while (0)

#define CLR_VALID(p)                do { (p)->flags &= ~W25QXXXJV_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)                do { (p)->flags &= ~W25QXXXJV_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)               do { (p)->flags &= ~W25QXXXJV_CACHE_ERASED; } while (0)

/* 512 byte sector support **********************************************************/

#define W25QXXXJV_SECTOR512_SHIFT     9
#define W25QXXXJV_SECTOR512_SIZE      (1 << 9)
#define W25QXXXJV_ERASED_STATE        0xff

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct w25qxxxjv_dev_s.
 */

struct w25qxxxjv_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */

#ifdef CONFIG_W25QXXXJV_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Locking */

static void w25qxxxjv_lock(FAR struct qspi_dev_s *qspi);
static inline void w25qxxxjv_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

static int  w25qxxxjv_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  w25qxxxjv_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              off_t addr, uint8_t addrlen);
static int  w25qxxxjv_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              FAR void *buffer, size_t buflen);
static int  w25qxxxjv_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              FAR const void *buffer, size_t buflen);
static uint8_t w25qxxxjv_read_status(FAR struct w25qxxxjv_dev_s *priv);
static void w25qxxxjv_write_status(FAR struct w25qxxxjv_dev_s *priv);
#if 0
static uint8_t w25qxxxjv_read_volcfg(FAR struct w25qxxxjv_dev_s *priv);
static void w25qxxxjv_write_volcfg(FAR struct w25qxxxjv_dev_s *priv);
#endif
static void w25qxxxjv_write_enable(FAR struct w25qxxxjv_dev_s *priv);
static void w25qxxxjv_write_disable(FAR struct w25qxxxjv_dev_s *priv);

static int  w25qxxxjv_readid(FAR struct w25qxxxjv_dev_s *priv);
static int  w25qxxxjv_protect(FAR struct w25qxxxjv_dev_s *priv,
              off_t startblock, size_t nblocks);
static int  w25qxxxjv_unprotect(FAR struct w25qxxxjv_dev_s *priv,
              off_t startblock, size_t nblocks);
static bool w25qxxxjv_isprotected(FAR struct w25qxxxjv_dev_s *priv,
              uint8_t status, off_t address);
static int  w25qxxxjv_erase_sector(FAR struct w25qxxxjv_dev_s *priv, off_t offset);
static int  w25qxxxjv_erase_chip(FAR struct w25qxxxjv_dev_s *priv);
static int  w25qxxxjv_read_byte(FAR struct w25qxxxjv_dev_s *priv, FAR uint8_t *buffer,
              off_t address, size_t nbytes);
static int  w25qxxxjv_write_page(FAR struct w25qxxxjv_dev_s *priv,
              FAR const uint8_t *buffer, off_t address, size_t nbytes);
#ifdef CONFIG_W25QXXXJV_SECTOR512
static int  w25qxxxjv_flush_cache(struct w25qxxxjv_dev_s *priv);
static FAR uint8_t *w25qxxxjv_read_cache(struct w25qxxxjv_dev_s *priv, off_t sector);
static void w25qxxxjv_erase_cache(struct w25qxxxjv_dev_s *priv, off_t sector);
static int  w25qxxxjv_write_cache(FAR struct w25qxxxjv_dev_s *priv,
              FAR const uint8_t *buffer,  off_t sector, size_t nsectors);
#endif

/* MTD driver methods */

static int  w25qxxxjv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks);
static ssize_t w25qxxxjv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks, FAR uint8_t *buf);
static ssize_t w25qxxxjv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks, FAR const uint8_t *buf);
static ssize_t w25qxxxjv_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
              FAR uint8_t *buffer);
static int  w25qxxxjv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: w25qxxxjv_lock
 ************************************************************************************/

static void w25qxxxjv_lock(FAR struct qspi_dev_s *qspi)
{
  /* On QuadSPI buses where there are multiple devices, it will be necessary to
   * lock QuadSPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive access to
   * the QuadSPI bus.  We will retain that exclusive access until the bus is
   * unlocked.
   */

  (void)QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency, setbits,
   * and setmode methods to make sure that the QuadSPI is properly configured for
   * the device. If the QuadSPI bus is being shared, then it may have been left in
   * an incompatible state.
   */

  QSPI_SETMODE(qspi, CONFIG_W25QXXXJV_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  (void)QSPI_SETFREQUENCY(qspi, CONFIG_W25QXXXJV_QSPI_FREQUENCY);
}

/************************************************************************************
 * Name: w25qxxxjv_unlock
 ************************************************************************************/

static inline void w25qxxxjv_unlock(FAR struct qspi_dev_s *qspi)
{
  (void)QSPI_LOCK(qspi, false);
}

/************************************************************************************
 * Name: w25qxxxjv_command
 ************************************************************************************/

static int w25qxxxjv_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
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

/************************************************************************************
 * Name: w25qxxxjv_command_address
 ************************************************************************************/

static int w25qxxxjv_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                     off_t addr, uint8_t addrlen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x Address: %04lx addrlen=%d\n", cmd, (unsigned long)addr, addrlen);

  cmdinfo.flags   = QSPICMD_ADDRESS;
  cmdinfo.addrlen = addrlen;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = addr;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/************************************************************************************
 * Name: w25qxxxjv_command_read
 ************************************************************************************/

static int w25qxxxjv_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
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

/************************************************************************************
 * Name: w25qxxxjv_command_write
 ************************************************************************************/

static int w25qxxxjv_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
                                   FAR const void *buffer, size_t buflen)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x buflen: %lu\n", cmd, (unsigned long)buflen);

  cmdinfo.flags   = QSPICMD_WRITEDATA;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = buflen;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = (FAR void *)buffer;

  return QSPI_COMMAND(qspi, &cmdinfo);
}

/************************************************************************************
 * Name: w25qxxxjv_read_status
 ************************************************************************************/

static uint8_t w25qxxxjv_read_status(FAR struct w25qxxxjv_dev_s *priv)
{
  DEBUGVERIFY(w25qxxxjv_command_read(priv->qspi, W25QXXXJV_READ_STATUS_1,
                                     (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/************************************************************************************
 * Name:  w25qxxxjv_write_status
 ************************************************************************************/

static void w25qxxxjv_write_status(FAR struct w25qxxxjv_dev_s *priv)
{
  w25qxxxjv_write_enable(priv);

  /* Keep in Software Protection */

  priv->cmdbuf[0] &= ~STATUS_SRP_MASK;

  w25qxxxjv_command_write(priv->qspi, W25QXXXJV_WRITE_STATUS_1,
                       (FAR const void *)priv->cmdbuf, 1);
  w25qxxxjv_write_disable(priv);
}

/************************************************************************************
 * Name:  w25qxxxjv_write_enable
 ************************************************************************************/

static void w25qxxxjv_write_enable(FAR struct w25qxxxjv_dev_s *priv)
{
  uint8_t status;

  do
    {
      w25qxxxjv_command(priv->qspi, W25QXXXJV_WRITE_ENABLE);
      status = w25qxxxjv_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_ENABLED);
}

/************************************************************************************
 * Name:  w25qxxxjv_write_disable
 ************************************************************************************/

static void w25qxxxjv_write_disable(FAR struct w25qxxxjv_dev_s *priv)
{
  uint8_t status;

  do
    {
      w25qxxxjv_command(priv->qspi, W25QXXXJV_WRITE_DISABLE);
      status = w25qxxxjv_read_status(priv);
    }
  while ((status & STATUS_WEL_MASK) != STATUS_WEL_DISABLED);
}

/************************************************************************************
 * Name: w25qxxxjv_readid
 ************************************************************************************/

static inline int w25qxxxjv_readid(struct w25qxxxjv_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */

  w25qxxxjv_lock(priv->qspi);

  /* Read the JEDEC ID */

  w25qxxxjv_command_read(priv->qspi, W25QXXXJV_JEDEC_ID, priv->cmdbuf, 3);

  /* Unlock the bus */

  w25qxxxjv_unlock(priv->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

  /* Check for a recognized memory device type */

  if (priv->cmdbuf[1] != W25QXXXJVQ_JEDEC_DEVICE_TYPE &&
      priv->cmdbuf[1] != W25QXXXJVM_JEDEC_DEVICE_TYPE)
    {
      ferr("ERROR: Unrecognized device type: 0x%02x\n", priv->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (priv->cmdbuf[2])
    {
      case W25Q016_JEDEC_CAPACITY:
        priv->sectorshift = W25Q016_SECTOR_SHIFT;
        priv->pageshift   = W25Q016_PAGE_SHIFT;
        priv->nsectors    = W25Q016_SECTOR_COUNT;
        break;

      case W25Q032_JEDEC_CAPACITY:
        priv->sectorshift = W25Q032_SECTOR_SHIFT;
        priv->pageshift   = W25Q032_PAGE_SHIFT;
        priv->nsectors    = W25Q032_SECTOR_COUNT;
        break;

      case W25Q064_JEDEC_CAPACITY:
        priv->sectorshift = W25Q064_SECTOR_SHIFT;
        priv->pageshift   = W25Q064_PAGE_SHIFT;
        priv->nsectors    = W25Q064_SECTOR_COUNT;
        break;

      case W25Q128_JEDEC_CAPACITY:
        priv->sectorshift = W25Q128_SECTOR_SHIFT;
        priv->pageshift   = W25Q128_PAGE_SHIFT;
        priv->nsectors    = W25Q128_SECTOR_COUNT;
        break;

      case W25Q256_JEDEC_CAPACITY:
        priv->sectorshift = W25Q256_SECTOR_SHIFT;
        priv->pageshift   = W25Q256_PAGE_SHIFT;
        priv->nsectors    = W25Q256_SECTOR_COUNT;
        break;

      case W25Q512_JEDEC_CAPACITY:
        priv->sectorshift = W25Q512_SECTOR_SHIFT;
        priv->pageshift   = W25Q512_PAGE_SHIFT;
        priv->nsectors    = W25Q512_SECTOR_COUNT;
        break;

      case W25Q01_JEDEC_CAPACITY:
        priv->sectorshift = W25Q01_SECTOR_SHIFT;
        priv->pageshift   = W25Q01_PAGE_SHIFT;
        priv->nsectors    = W25Q01_SECTOR_COUNT;
        break;

      /* Support for this part is not implemented yet */

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", priv->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/************************************************************************************
 * Name: w25qxxxjv_protect
 ************************************************************************************/

static int w25qxxxjv_protect(FAR struct w25qxxxjv_dev_s *priv,
                             off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = w25qxxxjv_read_status(priv);

  if ((priv->cmdbuf[0] & STATUS_BP_MASK) == STATUS_BP_ALL)
    {
      /* Protection already enabled */

      return 0;
    }

  /* set the BP bits as necessary to protect the range of sectors. */

  priv->cmdbuf[0] |= STATUS_BP_ALL;
  w25qxxxjv_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = w25qxxxjv_read_status(priv);
  if ((priv->cmdbuf[0] & STATUS_BP_MASK) != STATUS_BP_ALL)
    {
      return -EACCES;
    }

  return OK;
}

/************************************************************************************
 * Name: w25qxxxjv_unprotect
 ************************************************************************************/

static int w25qxxxjv_unprotect(FAR struct w25qxxxjv_dev_s *priv,
                               off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = w25qxxxjv_read_status(priv);

  if ((priv->cmdbuf[0] & STATUS_BP_MASK) == STATUS_BP_NONE)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Set the protection mask to zero (and not complemented).
   * REVISIT:  This logic should really just re-write the BP bits as
   * necessary to unprotect the range of sectors.
   */

  priv->cmdbuf[0] &= ~STATUS_BP_MASK;
  w25qxxxjv_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = w25qxxxjv_read_status(priv);
  if ((priv->cmdbuf[0] & (STATUS_SRP_MASK | STATUS_BP_MASK)) != 0)
    {
      return -EACCES;
    }

  return OK;
}

/************************************************************************************
 * Name: w25qxxxjv_isprotected
 ************************************************************************************/

static bool w25qxxxjv_isprotected(FAR struct w25qxxxjv_dev_s *priv, uint8_t status,
                                  off_t address)
{
  off_t protstart;
  off_t protend;
  off_t protsize;
  unsigned int bp;

  /* The BP field is spread across non-contiguous bits */

  bp = (status & STATUS_BP_MASK) >> STATUS_BP_SHIFT;

  /* the BP field is essentially the power-of-two of the number of 64k sectors,
   * saturated to the device size.
   */

  if (0 == bp)
    {
      return false;
    }

  protsize = 0x00010000;
  protsize <<= (protsize << (bp - 1));
  protend = (1 << priv->sectorshift) * priv->nsectors;
  if (protsize > protend)
    {
      protsize = protend;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up  (assuming CMP=0).
   */

  if ((status & STATUS_TB_MASK) != 0)
    {
      protstart = 0x00000000;
      protend   = protstart + protsize;
    }
  else
    {
      protstart = protend - protsize;

      /* protend already computed above */
    }

  return (address >= protstart && address < protend);
}

/************************************************************************************
 * Name:  w25qxxxjv_erase_sector
 ************************************************************************************/

static int w25qxxxjv_erase_sector(FAR struct w25qxxxjv_dev_s *priv, off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = w25qxxxjv_read_status(priv);
  if ((status & STATUS_BUSY_MASK) != STATUS_READY)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  if ((status & STATUS_BP_MASK) != 0 && w25qxxxjv_isprotected(priv, status, address))
    {
      ferr("ERROR: Flash protected: %02x", status);
      return -EACCES;
    }

  /* Send the sector erase command */

  w25qxxxjv_write_enable(priv);
  w25qxxxjv_command_address(priv->qspi, W25QXXXJV_SECTOR_ERASE, address, 3);

  /* Wait for erasure to finish */

  while ((w25qxxxjv_read_status(priv) & STATUS_BUSY_MASK) != 0);

  return OK;
}

/************************************************************************************
 * Name:  w25qxxxjv_erase_chip
 ************************************************************************************/

static int w25qxxxjv_erase_chip(FAR struct w25qxxxjv_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = w25qxxxjv_read_status(priv);
  if ((status & STATUS_BP_MASK) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  w25qxxxjv_write_enable(priv);
  w25qxxxjv_command(priv->qspi, W25QXXXJV_CHIP_ERASE);

  /* Wait for the erasure to complete */

  status = w25qxxxjv_read_status(priv);
  while ((status & STATUS_BUSY_MASK) != 0)
    {
      nxsig_usleep(200  *1000);
      status = w25qxxxjv_read_status(priv);
    }

  return OK;
}

/************************************************************************************
 * Name: w25qxxxjv_read_byte
 ************************************************************************************/

static int w25qxxxjv_read_byte(FAR struct w25qxxxjv_dev_s *priv, FAR uint8_t *buffer,
                               off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.addrlen = 3;
  meminfo.dummies = CONFIG_W25QXXXJV_DUMMIES;
  meminfo.buflen  = buflen;
  meminfo.cmd     = W25QXXXJV_FAST_READ_QUADIO;
  meminfo.addr    = address;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/************************************************************************************
 * Name:  w25qxxxjv_write_page
 ************************************************************************************/

static int w25qxxxjv_write_page(struct w25qxxxjv_dev_s *priv,
                                FAR const uint8_t *buffer,
                                off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  unsigned int pagesize;
  unsigned int npages;
  int ret;
  int i;

  finfo("address: %08lx buflen: %u\n", (unsigned long)address, (unsigned)buflen);

  npages   = (buflen >> priv->pageshift);
  pagesize = (1 << priv->pageshift);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.cmd     = W25QXXXJV_PAGE_PROGRAM;
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

      w25qxxxjv_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      w25qxxxjv_write_disable(priv);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06x\n",
               address);
          return ret;
        }

      /* Update for the next time through the loop */

      buffer  += pagesize;
      address += pagesize;
      buflen  -= pagesize;
    }

  /* The transfer should always be an even number of sectors and hence also
   * pages.  There should be no remainder.
   */

  DEBUGASSERT(buflen == 0);

  return OK;
}

/************************************************************************************
 * Name: w25qxxxjv_flush_cache
 ************************************************************************************/

#ifdef CONFIG_W25QXXXJV_SECTOR512
static int w25qxxxjv_flush_cache(struct w25qxxxjv_dev_s *priv)
{
  int ret = OK;

  /* If the cache is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = w25qxxxjv_write_page(priv, priv->sector, address, 1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: w25qxxxjv_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: w25qxxxjv_read_cache
 ************************************************************************************/

#ifdef CONFIG_W25QXXXJV_SECTOR512
static FAR uint8_t *w25qxxxjv_read_cache(struct w25qxxxjv_dev_s *priv, off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.  For
   * example, if the actual erase sector size is 4Kb (1 << 12), then we first
   * shift to the right by 3 to get the sector number in 4096 increments.
   */

  shift    = priv->sectorshift - W25QXXXJV_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = w25qxxxjv_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: w25qxxxjv_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = w25qxxxjv_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: w25qxxxjv_read_byte failed: %d\n", ret);
          return NULL;
        }

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the argument */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << W25QXXXJV_SECTOR512_SHIFT];
}
#endif

/************************************************************************************
 * Name: w25qxxxjv_erase_cache
 ************************************************************************************/

#ifdef CONFIG_W25QXXXJV_SECTOR512
static void w25qxxxjv_erase_cache(struct w25qxxxjv_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is in
   * the cache.
   */

  dest = w25qxxxjv_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (priv->sectorshift - W25QXXXJV_SECTOR512_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      DEBUGVERIFY(w25qxxxjv_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as dirty
   * (but don't update the FLASH yet.  The caller will do that at a more optimal
   * time).
   */

  memset(dest, W25QXXXJV_ERASED_STATE, W25QXXXJV_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/************************************************************************************
 * Name: w25qxxxjv_write_cache
 ************************************************************************************/

#ifdef CONFIG_W25QXXXJV_SECTOR512
static int w25qxxxjv_write_cache(FAR struct w25qxxxjv_dev_s *priv,
                                 FAR const uint8_t *buffer, off_t sector,
                                 size_t nsectors)
{
  FAR uint8_t *dest;
  int ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is in
       * memory.
       */

      dest = w25qxxxjv_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase sector
       * is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >> (priv->sectorshift - W25QXXXJV_SECTOR512_SHIFT);
          finfo("sector: %ld esectno: %d\n", sector, esectno);

          ret = w25qxxxjv_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: w25qxxxjv_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, W25QXXXJV_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += W25QXXXJV_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return w25qxxxjv_flush_cache(priv);
}
#endif

/************************************************************************************
 * Name: w25qxxxjv_erase
 ************************************************************************************/

static int w25qxxxjv_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks)
{
  FAR struct w25qxxxjv_dev_s *priv = (FAR struct w25qxxxjv_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_W25QXXXJV_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  w25qxxxjv_lock(priv->qspi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_W25QXXXJV_SECTOR512
      w25qxxxjv_erase_cache(priv, startblock);
#else
      w25qxxxjv_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_W25QXXXJV_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = w25qxxxjv_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

  w25qxxxjv_unlock(priv->qspi);

  return (int)nblocks;
}

/************************************************************************************
 * Name: w25qxxxjv_bread
 ************************************************************************************/

static ssize_t w25qxxxjv_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer)
{
#ifndef CONFIG_W25QXXXJV_SECTOR512
  FAR struct w25qxxxjv_dev_s *priv = (FAR struct w25qxxxjv_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

#ifdef CONFIG_W25QXXXJV_SECTOR512
  nbytes = w25qxxxjv_read(dev, startblock << W25QXXXJV_SECTOR512_SHIFT,
                       nblocks << W25QXXXJV_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= W25QXXXJV_SECTOR512_SHIFT;
    }
#else
  nbytes = w25qxxxjv_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

/************************************************************************************
 * Name: w25qxxxjv_bwrite
 ************************************************************************************/

static ssize_t w25qxxxjv_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct w25qxxxjv_dev_s *priv = (FAR struct w25qxxxjv_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  w25qxxxjv_lock(priv->qspi);

#if defined(CONFIG_W25QXXXJV_SECTOR512)
  ret = w25qxxxjv_write_cache(priv, buffer, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: w25qxxxjv_write_cache failed: %d\n", ret);
    }

#else
  ret = w25qxxxjv_write_page(priv, buffer, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: w25qxxxjv_write_page failed: %d\n", ret);
    }
#endif

  w25qxxxjv_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

/************************************************************************************
 * Name: w25qxxxjv_read
 ************************************************************************************/

static ssize_t w25qxxxjv_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                              FAR uint8_t *buffer)
{
  FAR struct w25qxxxjv_dev_s *priv = (FAR struct w25qxxxjv_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  w25qxxxjv_lock(priv->qspi);
  ret = w25qxxxjv_read_byte(priv, buffer, offset, nbytes);
  w25qxxxjv_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: w25qxxxjv_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/************************************************************************************
 * Name: w25qxxxjv_ioctl
 ************************************************************************************/

static int w25qxxxjv_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct w25qxxxjv_dev_s *priv = (FAR struct w25qxxxjv_dev_s *)dev;
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

#ifdef CONFIG_W25QXXXJV_SECTOR512
              geo->blocksize    = (1 << W25QXXXJV_SECTOR512_SHIFT);
              geo->erasesize    = (1 << W25QXXXJV_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors <<
                                  (priv->sectorshift - W25QXXXJV_SECTOR512_SHIFT);
#else
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          w25qxxxjv_lock(priv->qspi);
          ret = w25qxxxjv_erase_chip(priv);
          w25qxxxjv_unlock(priv->qspi);
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = w25qxxxjv_protect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = w25qxxxjv_unprotect(priv, prot->startblock, prot->nblocks);
        }
        break;

      default:
        ret = -ENOTTY; /* Bad/unsupported command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: w25qxxxjv_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the QuadSPI-based W25QxxxJV
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as instances
 *   that can be bound to other functions (such as a block or character driver front
 *   end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *w25qxxxjv_initialize(FAR struct qspi_dev_s *qspi,
                                           bool unprotect)
{
  FAR struct w25qxxxjv_dev_s *priv;
  int ret;

  finfo("qspi: %p\n", qspi);
  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same QuadSPI bus.
   */

  priv = (FAR struct w25qxxxjv_dev_s *)kmm_zalloc(sizeof(struct w25qxxxjv_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = w25qxxxjv_erase;
      priv->mtd.bread  = w25qxxxjv_bread;
      priv->mtd.bwrite = w25qxxxjv_bwrite;
      priv->mtd.read   = w25qxxxjv_read;
      priv->mtd.ioctl  = w25qxxxjv_ioctl;
      priv->mtd.name   = "w25qxxxjv";
      priv->qspi       = qspi;

      /* Allocate a 4-byte buffer to support DMA-able command data */

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
      if (priv->cmdbuf == NULL)
        {
          ferr("ERROR Failed to allocate command buffer\n");
          goto errout_with_priv;
        }

      /* Allocate a one-byte buffer to support DMA-able status read data */

      priv->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 1);
      if (priv->readbuf == NULL)
        {
          ferr("ERROR Failed to allocate read buffer\n");
          goto errout_with_cmdbuf;
        }

      /* Identify the FLASH chip and get its capacity */

      ret = w25qxxxjv_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          ferr("ERROR Unrecognized QSPI device\n");
          goto errout_with_readbuf;
        }

      /* Unprotect FLASH sectors if so requested. */

      if (unprotect)
        {
          ret = w25qxxxjv_unprotect(priv, 0, priv->nsectors - 1);
          if (ret < 0)
            {
              ferr("ERROR: Sector unprotect failed\n");
            }
        }

#ifdef CONFIG_W25QXXXJV_SECTOR512  /* Simulate a 512 byte sector */
      /* Allocate a buffer for the erase block cache */

      priv->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, 1 << priv->sectorshift);
      if (priv->sector == NULL)
        {
          /* Allocation failed! Discard all of that work we just did and return NULL */

          ferr("ERROR: Sector allocation failed\n");
          goto errout_with_readbuf;
        }
#endif
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

errout_with_readbuf:
  QSPI_FREE(qspi, priv->readbuf);

errout_with_cmdbuf:
  QSPI_FREE(qspi, priv->cmdbuf);

errout_with_priv:
  kmm_free(priv);
  return NULL;
}
