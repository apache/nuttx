/****************************************************************************
 * drivers/mtd/gd55.c
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

#ifdef CONFIG_MTD_GD55_SECTOR512
#  include <stdlib.h>
#  include <string.h>
#endif

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4 byte addressing is needed for addresses needing more than a 3 byte
 * address, i.e. 16Mbyte
 */

#define MODE_3BYTE_LIMIT        ((16 * 1024 * 1024))

/* GD55 Commands                                                            */

#define GD55_QREAD              0x6b  /* Quad output fast read              */
#define GD55_QREAD_DUMMIES      8
#define GD55_QC_READ            0xeb  /* Quad output continuous fast read   */
#define GD55_QC_READ_DUMMIES    6
#define GD55_EQPP               0xc2  /* Extended quad page program         */
#define GD55_EQPP_DUMMIES       0     /* No dummy clocks                    */
#define GD55_SE                 0x20  /* 4Kb Sector erase                   */
#define GD55_BE32               0x52  /* 32Kbit block Erase                 */
#define GD55_BE64               0xd8  /* 64Kbit block Erase                 */
#define GD55_CE                 0x60  /* Chip erase (alternate)             */
#define GD55_WREN               0x06  /* Write Enable                       */
#define GD55_WRDI               0x04  /* Write Disable                      */
#define GD55_RDSR1              0x05  /* Read status register 1             */
#define GD55_EN4B               0xb7  /* Enable 4 byte Addressing Mode      */
#define GD55_DIS4B              0xe9  /* Disable 4 byte Addressing Mode     */
#define GD55_IBSL               0x36  /* Individual block/sector lock       */
#define GD55_IBSUL              0x39  /* Individual block/sector unlock     */
#define GD55_RIBSL              0x3d  /* Read individual block/sector lock  */
#define GD55_RDNVCR             0xb5  /* Read Non-Volatile config register  */
#define GD55_RD_NVCR_DUMMIES    8
#define GD55_RDSR2              0x35  /* Read status register 2             */
#define GD55_WRSR1              0x01  /* Write status register 1            */
#define GD55_SE_ALT             0x21  /* Alternate 4Kb Sector erase         */
#define GD55_QC_READ_ALT        0xec  /* Quad output continuous fast read   */
#define GD55_4B_QDTR_READ       0xed  /* Quad I/O DTR read                  */
#define GD55_4B_QDTR_READ_ALT   0xee  /* Alternate quad I/O DTR read        */
#define GD55_PP                 0x02  /* Page program (SPI, not used)       */
#define GD55_PP_ALT             0x12  /* Aternate page program (SPI)        */
#define GD55_BE32_ALT           0x5c  /* Alternate 32Kbit block Erase       */
#define GD55_BE64_ALT           0xd8  /* ALternate 64Kbit block Erase       */
#define GD55_CE_ALT             0xc7  /* Alternate chip erase               */
#define GD55_QPP                0x32  /* Quad page program                  */
#define GD55_QPP_ALT            0x34  /* ALternate quad page program        */
#define GD55_QPP_DUMMIES        0     /* No dummy clocks                    */
#define GD55_QPIEN              0x38  /* Enable QPI Operation               */
#define GD55_QPIDIS             0xff  /* Disable QPI Operation              */
#define GD55_DP                 0xb9  /* Deep power down                    */
#define GD55_RDP                0xab  /* Release deep power down            */
#define GD55_RUID               0x4b  /* Read Unique ID                     */
#define GD55_RDID               0x9e  /* Read identification                */
#define GD55_RDID_ALT           0x9f  /* Read identification (alternate)    */
#define GD55_PE_SUSPEND         0x75  /* Suspends program/erase             */
#define GD55_PE_RESUME          0x7a  /* Resume program                     */
#define GD55_RDVCR              0x85  /* Read Volatile config register      */
#define GD55_RD_VCR_DUMMIES     1
#define GD55_WRSR2              0x31  /* Write status register 2            */
#define GD55_WRNVCR             0xb1  /* Write Non-Volatile config register */
#define GD55_WRENVSC            0x50  /* Write en. Volatile config register */
#define GD55_WRVCR              0x91  /* Write Volatile config register     */
#define GD55_WREAR              0xc5  /* Write Extended address register    */
#define GD55_EARR               0xc8  /* Read extended address register     */
#define GD55_RSFDP              0x5a  /* Read SFDP                          */
#define GD55_RDSCUR             0x48  /* Read security register             */
#define GD55_WRSCUR             0x42  /* Write security register            */
#define GD55_ERSCUR             0x44  /* Erase security register            */
#define GD55_RSTEN              0x66  /* Reset Enable                       */
#define GD55_RST                0x99  /* Reset Memory                       */
#define GD55_GBSL               0x7e  /* Global block/sector lock           */
#define GD55_GBSUL              0x98  /* Global block/sector unlock         */

/* Read ID (RDID) register values                                           */

#define GD55_MANUFACTURER       0xc8  /* GigaSevice manufacturer ID         */

/* JEDEC Read ID register values                                            */

#define GD55_JEDEC_MANUFACTURER 0xc8 /* GigaDevice manufacturer ID          */

#define GD55B_JEDEC_MEMORY_TYPE 0x47 /* GD55B memory type, 3V               */
#define GD55L_JEDEC_MEMORY_TYPE 0x67 /* GD55L memory type, 1.8V             */
#define GD55_JEDEC_1G_CAPACITY  0x1b /* 1Gbit memory capacity               */
#define GD55_JEDEC_2G_CAPACITY  0x1c /* 2Gbit memory capacity               */

/* GD55 devices all have identical sector sizes:
 * block protection size: 64KiB
 * sector size:           4KiB
 * page size:             256B
 */

#define GD55_SECTOR_SHIFT       (12)
#define GD55_SECTOR_SIZE        (1 << GD55_SECTOR_SHIFT)           /* 4KiB  */
#define GD55_PAGE_SHIFT         (8)                                /* 256B  */
#define GD55_PAGE_SIZE          (1 << GD55_PAGE_SHIFT)
#define GD55_BP_SHIFT           (16)
#define GD55_BP_SIZE            (1 << GD55_BP_SHIFT)               /* 64KiB */
#define GD55_MIN_BP_BLKS        (GD55_BP_SIZE  >> GD55_PAGE_SHIFT)
#define GD55_SECTORS_PER_BP_BLK (GD55_BP_SIZE / GD55_SECTOR_SIZE)

/* GD55B01xx (128 MiB) memory capacity                                      */

#define GD55_NSECTORS_1GBIT     (32768)

/* GD55B02xx (256 MiB) memory capacity                                      */

#define GD55_NSECTORS_2GBIT     (65536)

/* 512 byte sector support **************************************************/

#define GD55_SECTOR512_SHIFT    (9)
#define GD55_SECTOR512_SIZE     (1 << GD55_SECTOR512_SHIFT)

/* Status register 1 bit definitions                                        */

#define GD55_SR_WIP             (1 << 0)  /* Bit 0: Write in progress       */
#define GD55_SR_WEL             (1 << 1)  /* Bit 1: Write enable latch      */
#define GD55_SR_BP_SHIFT        (2)       /* Bits 2-6: Block protect bits   */
#define GD55_SR_BP_MASK         (31 << GD55_SR_BP_SHIFT)
#define GD55_STATUS_BP_NONE     (0 << GD55_SR_BP_SHIFT)
#define GD55_STATUS_BP_ALL      (7 << GD55_SR_BP_SHIFT)
#define GD55_STATUS_TB_MASK     (1 << 6)  /* BP4 Top/Bottom Protect         */
#define GD55_STATUS_TB_TOP      (0 << 6)  /*   = 0, BP3..0 protect Top down */
#define GD55_STATUS_TB_BOTTOM   (1 << 6)  /*   = 1, BP3..0   "   Bottom up  */
#define GD55_SR_BP_TOP(b)       (((b + 1) << GD55_SR_BP_SHIFT) | \
                                 GD55_STATUS_TB_TOP)
#define GD55_SR_BP_BOTTOM(b)    (((b + 1) << GD55_SR_BP_SHIFT) | \
                                 GD55_STATUS_TB_BOTTOM)
#define GD55_BP_ALL             (14 << GD55_SR_BP_SHIFT)
                                          /* GD55B01 needs BP bits = 0xx11xx
                                           * GD55B02 needs BP bits = 0xx111x
                                           */
#define GD55_SR_SRP0            (1 << 7)  /* Bit 7: SR protect bit 0        */

/* Status register 2 bit definitions                                        */

#define GD55_SR_ADS             (1 << 0)  /* Bit 0: Current Address Mode    */
                                          /* Bit 1 - reserved               */
#define GD55_SR_SUS2            (1 << 2)  /* Bit 2: Program suspend bit 2   */
#define GD55_SR_LB              (1 << 3)  /* Bit 3: Security Register Lock  */
#define GD55_SR_PE              (1 << 4)  /* Bit 4: Program Error Bit       */
#define GD55_SR_EE              (1 << 5)  /* Bit 5: Erase Error Bit         */
#define GD55_SR_SRP1            (1 << 6)  /* Bit 6: SR protection bit 1     */
#define GD55_SR_SUS1            (1 << 7)  /* Bit 7: Program suspend bit 1   */

/* Non-volatile and volatile config register addresses and bits             */

#define GD55_DUMMY_CYCLES_REG   1         /* Dummy Cycle Configuration      */
#define GD55_ODT_DS_REG         3         /* On-die termination and driver
                                           * strength configuration
                                           */
#define GD55_DLP_PROT_REG       4         /* Data Learning and protect mode */
#define GD55_PROT_MODE_MASK     (1 << 2)  /* Bit 2, BP or WPS mode          */
#define GD55_PROT_MODE_WPS      (0 << 2)  /* 0 = Sector Protect mode        */
#define GD55_PROT_MODE_BP       (1 << 2)  /* 1 = Block Protect mode (def.)  */
#define GD55_4BYTE_MODE_REG     5         /* 3 pr 4-byte address mode       */
#define GD55_XIP_MODE_REG       6         /* XIP (continuous read) mode     */
#define GD55_WRAP_CONFIG_REG    7         /* Wrap mode (none/64/32/16 byte) */

/* Block protection bit */

#define GD55_BLK_PROTECTED      (1 << 0)  /* lsb set means block is locked  */

/* Cache flags **************************************************************/

#define GD55_CACHE_VALID        (1 << 0)  /* 1=Cache has valid data         */
#define GD55_CACHE_DIRTY        (1 << 1)  /* 1=Cache is dirty               */
#define GD55_CACHE_ERASED       (1 << 2)  /* 1=Backing FLASH is erased      */

#define IS_VALID(p)             ((((p)->flags) & GD55_CACHE_VALID)  != 0)
#define IS_DIRTY(p)             ((((p)->flags) & GD55_CACHE_DIRTY)  != 0)
#define IS_ERASED(p)            ((((p)->flags) & GD55_CACHE_ERASED) != 0)

#define SET_VALID(p)  do { (p)->flags |= GD55_CACHE_VALID;   } while (0)
#define SET_DIRTY(p)  do { (p)->flags |= GD55_CACHE_DIRTY;   } while (0)
#define SET_ERASED(p) do { (p)->flags |= GD55_CACHE_ERASED;  } while (0)

#define CLR_VALID(p)  do { (p)->flags &= ~GD55_CACHE_VALID;  } while (0)
#define CLR_DIRTY(p)  do { (p)->flags &= ~GD55_CACHE_DIRTY;  } while (0)
#define CLR_ERASED(p) do { (p)->flags &= ~GD55_CACHE_ERASED; } while (0)

#define GD55_ERASED_STATE       0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Internal state of the MTD device */

struct gd55_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface                      */
  FAR struct qspi_dev_s *qspi;        /* QuadSPI interface                  */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer           */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer       */
  uint32_t               nsectors;    /* Number of erase sectors            */
#ifdef CONFIG_MTD_GD55_SECTOR512
  uint8_t                flags;       /* Buffered sector flags              */
  uint16_t               esectno;     /* Erase sector number in the cache   */
  FAR uint8_t           *sector;      /* Allocated sector data              */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int     gd55_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks);
static ssize_t gd55_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf);
static ssize_t gd55_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf);
static ssize_t gd55_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buffer);
static int     gd55_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                          unsigned long arg);

/* Internal driver methods */

static void     gd55_lock(FAR struct gd55_dev_s *priv);
static void     gd55_unlock(FAR struct gd55_dev_s *priv);
static int      gd55_command_read(FAR struct gd55_dev_s *priv, uint8_t cmd,
                                  FAR void *buffer, size_t buflen);
static int      gd55_command(FAR struct gd55_dev_s *priv, uint8_t cmd);
static int      gd55_command_address(FAR struct gd55_dev_s *priv,
                                     uint8_t cmd, off_t addr,
                                     uint8_t addrlen);
static int      gd55_readid(FAR struct gd55_dev_s *priv);
static int      gd55_protect(FAR struct gd55_dev_s *priv, off_t startblock,
                             size_t nblocks);
static int      gd55_unprotect(FAR struct gd55_dev_s *priv, off_t startblock,
                               size_t nblocks);
static bool     gd55_isprotected(FAR struct gd55_dev_s *priv, off_t addr,
                                 uint8_t status);
static int      gd55_read_bytes(FAR struct gd55_dev_s *priv,
                                FAR uint8_t *buffer, off_t address,
                                size_t buflen);
static uint8_t  gd55_read_status1(FAR struct gd55_dev_s *priv);
static uint8_t  gd55_read_status2(FAR struct gd55_dev_s *priv);
static void     gd55_write_status1(FAR struct gd55_dev_s *priv);
static int      gd55_command_write(FAR struct gd55_dev_s *priv, uint8_t cmd,
                                   FAR const void *buffer, size_t buflen);
static void     gd55_write_enable(FAR struct gd55_dev_s *priv);
static int      gd55_write_page(FAR struct gd55_dev_s *priv,
                                FAR const uint8_t *buffer, off_t address,
                                size_t buflen);
static int      gd55_erase_sector(FAR struct gd55_dev_s *priv, off_t sector);

static int      gd55_erase_chip(FAR struct gd55_dev_s *priv);
#ifdef CONFIG_MTD_GD55_SECTOR512
static int          gd55_flush_cache(FAR struct gd55_dev_s *priv);
static FAR uint8_t *gd55_read_cache(FAR struct gd55_dev_s *priv,
                                    off_t sector);
static void         gd55_erase_cache(FAR struct gd55_dev_s *priv,
                                     off_t sector);
static int          gd55_write_cache(FAR struct gd55_dev_s *priv,
                                     FAR const uint8_t *buffer, off_t sector,
                                     size_t nsectors);
#else
static int          gd55_erase_64kblock(FAR struct gd55_dev_s *priv,
                                        off_t sector);
static int          gd55_erase_32kblock(FAR struct gd55_dev_s *priv,
                                        off_t sector);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd55_lock
 *
 * Description:
 * On QSPI buses where there are multiple devices, it will be necessary to
 * lock QSPI to have exclusive access to the bus for a sequence of
 * transfers.  The bus should be locked before the chip is selected.
 *
 * This is a blocking call and will not return until we have exclusive
 * access to the SPI bus. We will retain that exclusive access until the
 * bus is unlocked.
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void gd55_lock(FAR struct gd55_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, true);

  /* After locking the QSPI bus, the we also need call the setfrequency,
   * setbits and setmode methods to make sure that the QSPI is properly
   * configured for the device.  If the QSPI bus is being shared, then it
   * may have been left in an incompatible state.
   */

  QSPI_SETMODE(priv->qspi, CONFIG_MTD_GD55_QSPIMODE);
  QSPI_SETBITS(priv->qspi, 8);
  QSPI_SETFREQUENCY(priv->qspi, CONFIG_MTD_GD55_FREQUENCY);
}

/****************************************************************************
 * Name: gd55_unlock
 *
 * Description:
 * On QSPI buses where there are multiple devices, it will have been
 * necessary to lock QSSPI to have exclusive access to the bus for a sequence
 * of transfers.  The bus must be unlocked after the transfers to relinquish
 * the exclusive access from the call to LOCK the bus.
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void gd55_unlock(FAR struct gd55_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: gd55_command_read
 *
 * Description:
 *   Read data from the device.
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   cmd          - the read command to be used
 *   buffer       - pointer to variable to store the read data
 *   buflen       - the number of bytes to be read into the buffer
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_command_read(FAR struct gd55_dev_s *priv, uint8_t cmd,
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

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: gd55_command
 *
 * Description:
 *   Send a command to the device.
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   cmd          - the command to be sent
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_command(FAR struct gd55_dev_s *priv, uint8_t cmd)
{
  struct qspi_cmdinfo_s cmdinfo;

  finfo("CMD: %02x\n", cmd);

  cmdinfo.flags   = 0;
  cmdinfo.addrlen = 0;
  cmdinfo.cmd     = cmd;
  cmdinfo.buflen  = 0;
  cmdinfo.addr    = 0;
  cmdinfo.buffer  = NULL;

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: gd55_command_write
 *
 * Description:
 *   Send a command to the device with data
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   cmd          - the command to be sent
 *   buffer       - pointer to variable with the data to write
 *   buflen       - the number of data bytes to be written
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_command_write(FAR struct gd55_dev_s *priv, uint8_t cmd,
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

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: gd55_command_address
 *
 * Description:
 *   Send a command with an associated address to the device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   cmd          - the command to be sent
 *   addr         - address to send
 *   addrlen      - address length
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_command_address(FAR struct gd55_dev_s *priv, uint8_t cmd,
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

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: gd55_read_bytes
 *
 * Description:
 *   Read data from the device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   buffer       - pointer to buffer to read the data to
 *   address      - address to read from
 *   buflen       - number of bytes to read (buffer length)
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_read_bytes(FAR struct gd55_dev_s *priv, FAR uint8_t *buffer,
                           off_t address, size_t buflen)
{
  bool                  mode_4byte_addr;
  int                   ret;
  struct qspi_meminfo_s meminfo;

  /* Check if any address exceeds range of 3 byte addressing */

  if ((address + buflen) >= MODE_3BYTE_LIMIT)
    {
      gd55_command(priv, GD55_EN4B);
      mode_4byte_addr = true;
    }

  finfo("4byte mode: %s\taddress: %08lx\tnbytes: %d\n",
         mode_4byte_addr ? "true" : "false", (long)address, (int)buflen);

  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
  meminfo.dummies = GD55_QC_READ_DUMMIES;
  meminfo.buflen  = buflen;
  meminfo.cmd     = GD55_QC_READ;
  meminfo.addr    = address;
  meminfo.addrlen = mode_4byte_addr ? 4 : 3;
  meminfo.buffer  = buffer;

  ret = QSPI_MEMORY(priv->qspi, &meminfo);
  if (mode_4byte_addr)
    {
      gd55_command(priv, GD55_DIS4B);
    }

  return ret;
}

/****************************************************************************
 * Name: gd55_write_page
 *
 * Description:
 *   Write a page of data to the device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   buffer       - pointer to the buffer with the data to write
 *   address      - address to write to
 *   buflen       - number of bytes to write (buffer length)
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_write_page(FAR struct gd55_dev_s *priv,
                           FAR const uint8_t *buffer,
                           off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;
  uint8_t               status;
  unsigned int          npages;
  int                   ret;
  int                   i;

  npages = (buflen >> GD55_PAGE_SHIFT);

  /* Check if address exceeds range of 3 byte addressing */

  if ((address + buflen) >= MODE_3BYTE_LIMIT)
    {
      gd55_command(priv, GD55_EN4B);
      meminfo.addrlen = 4;
    }
  else
    {
      gd55_command(priv, GD55_DIS4B);
      meminfo.addrlen = 3;
    }

  finfo("4byte mode: %s\taddress: %08lx\tbuflen: %u\n",
        (meminfo.addrlen == 4) ? "true" : "false", (unsigned long)address,
        (unsigned)buflen);

  /* Set up non-varying parts of transfer description */

  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_QUADIO;
  meminfo.cmd     = GD55_EQPP;
  meminfo.buflen  = GD55_PAGE_SIZE;
  meminfo.dummies = GD55_EQPP_DUMMIES;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (FAR void *)buffer;

      /* Write one page */

      gd55_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);

      if (ret < 0)
        {
          ferr("ERROR: QSPI_MEMORY failed writing address=%06jx\n",
               (intmax_t)address);
          goto exit;
        }

      /* Update for the next time through the loop */

      buffer  += GD55_PAGE_SIZE;
      address += GD55_PAGE_SIZE;

      /* Wait for write operation to finish */

      do
        {
          status = gd55_read_status1(priv);
        }
      while ((status & GD55_SR_WIP) != 0);
    }

exit:
  if (meminfo.addrlen == 4)
    {
      gd55_command(priv, GD55_DIS4B);
    }

  return ret;
}

/****************************************************************************
 * Name: gd55_erase_sector
 *
 * Description:
 *   Erase a single sector of th device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   sector       - the sector to erase
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_erase_sector(FAR struct gd55_dev_s *priv, off_t sector)
{
  uint8_t status;
  bool    mode_4byte_addr = false;
  off_t   addr = sector << GD55_SECTOR_SHIFT;

  finfo("4byte mode: %s\tsector: %08lx\n", mode_4byte_addr ?
                                           "true" : "false",
                                           (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = gd55_read_status1(priv);
  if ((status & GD55_SR_WIP) == GD55_SR_WIP)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  if (gd55_isprotected(priv, addr, status))
    {
      ferr("ERROR: Flash protected at addr: %02" PRIx32, addr);
      return -EACCES;
    }

  /* Check if address exceeds range of 3 byte addressing */

  if (addr >= MODE_3BYTE_LIMIT)
    {
      gd55_command(priv, GD55_EN4B);
      mode_4byte_addr = true;
    }

  /* Send the sector erase command */

  gd55_write_enable(priv);
  gd55_command_address(priv, GD55_SE, addr, mode_4byte_addr ? 4 : 3);

  /* Wait for erasure to finish */

  do
    {
      nxsig_usleep(10 * 1000); /* Typical sector erase time is 30ms */
      status = gd55_read_status1(priv);
    }
  while ((status & GD55_SR_WIP) != 0);

  if (mode_4byte_addr)
    {
      gd55_command(priv, GD55_DIS4B);
    }

  return OK;
}

#ifndef CONFIG_MTD_GD55_SECTOR512
/****************************************************************************
 * Name: gd55_erase_64kblock
 *
 * Description:
 *   Erase a 64k block of the device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   sector       - an address of a sector within the 64k block to erase
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_erase_64kblock(FAR struct gd55_dev_s *priv, off_t sector)
{
  off_t   addr = sector << GD55_SECTOR_SHIFT;
  uint8_t status;
  bool    mode_4byte_addr = false;

  finfo("4byte mode: %s\tsector: %08lx\n", mode_4byte_addr ?
                                           "true" : "false",
                                           (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = gd55_read_status1(priv);
  if ((status & GD55_SR_WIP) == GD55_SR_WIP)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  if (gd55_isprotected(priv, addr, status))
    {
      ferr("ERROR: Flash protected at addr: %02" PRIx32, addr);
      return -EACCES;
    }

  if (addr >= MODE_3BYTE_LIMIT)
    {
      gd55_command(priv, GD55_EN4B);
      mode_4byte_addr = true;
    }

  /* Send the 64k block erase command */

  gd55_write_enable(priv);
  gd55_command_address(priv, GD55_BE64, addr, mode_4byte_addr ?
                                                       4 : 3);

  /* Wait for erasure to finish */

  do
    {
      nxsig_usleep(50 * 1000); /* typical 64k erase time is 220ms */
      status = gd55_read_status1(priv);
    }
  while ((status & GD55_SR_WIP) != 0);

  if (mode_4byte_addr)
    {
      gd55_command(priv, GD55_DIS4B);
    }

  return OK;
}

/****************************************************************************
 * Name: gd55_erase_32kblock
 *
 * Description:
 *   Erase a 32k block of the device
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *   sector       - an address of a sector within the 32k block to erase
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_erase_32kblock(FAR struct gd55_dev_s *priv, off_t sector)
{
  off_t   addr = sector << GD55_SECTOR_SHIFT;
  uint8_t status;
  bool    mode_4byte_addr = false;

  finfo("4byte mode: %s\tsector: %08lx\n", mode_4byte_addr ?
                                           "true" : "false",
                                           (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = gd55_read_status1(priv);
  if ((status & GD55_SR_WIP) == GD55_SR_WIP)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  if (gd55_isprotected(priv, addr, status))
    {
      ferr("ERROR: Flash protected at addr: %02" PRIx32, addr);
      return -EACCES;
    }

  if (addr >= MODE_3BYTE_LIMIT)
    {
      gd55_command(priv, GD55_EN4B);
      mode_4byte_addr = true;
    }

  /* Send the 32k block erase command */

  gd55_write_enable(priv);
  gd55_command_address(priv, GD55_BE32, addr, mode_4byte_addr ? 4 : 3);

  /* Wait for erasure to finish */

  do
    {
      nxsig_usleep(50 * 1000); /* typical 32k erase time is 150ms */
      status = gd55_read_status1(priv);
    }
  while ((status & GD55_SR_WIP) != 0);

  if (mode_4byte_addr)
    {
      gd55_command(priv, GD55_DIS4B);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: gd55_erase_chip
 *
 * Description:
 *   Erase entire chip
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   Zero (OK) on SUCCESS, a negated errno on value of failure
 *
 ****************************************************************************/

static int gd55_erase_chip(FAR struct gd55_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = gd55_read_status1(priv);
  if ((status & GD55_SR_BP_MASK) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  gd55_write_enable(priv);
  gd55_command(priv, GD55_CE);

  /* Wait for the erasure to complete */

  status = gd55_read_status1(priv);

  while ((status & GD55_SR_WIP) != 0)
    {
      nxsig_sleep(2);
      status = gd55_read_status1(priv);
    }

  return OK;
}

/****************************************************************************
 * Name: gd55_write_enable
 *
 * Description:
 *   Enable the device for writing by setting the wriet enable latch bit
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd55_write_enable(FAR struct gd55_dev_s *priv)
{
  uint8_t status;

  gd55_command(priv, GD55_WREN);
  do
    {
      status = gd55_read_status1(priv);
    }
  while ((status & GD55_SR_WEL) != GD55_SR_WEL);
}

/****************************************************************************
 * Name: gd55_read_status1
 *
 * Description:
 *   Read status register 1
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   The status register data
 *
 ****************************************************************************/

static uint8_t gd55_read_status1(FAR struct gd55_dev_s *priv)
{
  uint8_t status;

  gd55_command_read(priv, GD55_RDSR1, &status, 1);
  return status;
}

/****************************************************************************
 * Name: gd55_read_status2
 *
 * Description:
 *   Read status register 2
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   The status register data
 *
 ****************************************************************************/

static uint8_t gd55_read_status2(FAR struct gd55_dev_s *priv)
{
  uint8_t status;

  gd55_command_read(priv, GD55_RDSR2, &status, 1);
  return status;
}

/****************************************************************************
 * Name: gd55_write_status1
 *
 * Description:
 *   Write data to status register 1
 *   The data to be written must have been written to the device structures
 *   command buffer (cmdbuf)
 *
 * Input Parameters:
 *   priv         - a reference to the device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd55_write_status1(FAR struct gd55_dev_s *priv)
{
  uint8_t status;

  gd55_write_enable(priv);

  /* take care to mask of the SRP bit; it is one-time-programmable */

  priv->cmdbuf[0] &= ~GD55_SR_SRP0;

  gd55_command_write(priv, GD55_WRSR1,
                     (FAR const void *)priv->cmdbuf, 1);

  /* Wait for write operation to finish */

  do
    {
      status = gd55_read_status1(priv);
    }
  while ((status & GD55_SR_WIP) != 0);
}

/****************************************************************************
 * Name: gd55_erase
 *
 * Description:
 *   Erase a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the erase
 *   nblocks    - nblocks to erase
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  FAR struct gd55_dev_s *priv = (FAR struct gd55_dev_s *)dev;
  size_t                blocksleft = nblocks;
  int                   ret;
#ifndef CONFIG_MTD_GD55_SECTOR512
  const size_t         sectorsper64kblock = (64 * 1024) >> GD55_SECTOR_SHIFT;
  const size_t         sectorsper32kblock = (32 * 1024) >> GD55_SECTOR_SHIFT;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  gd55_lock(priv);

#ifdef CONFIG_MTD_GD55_SECTOR512
  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      gd55_erase_cache(priv, startblock);
      startblock++;
    }

  /* Flush the last erase block left in the cache */

  ret = gd55_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#else
  while (blocksleft > 0)
    {
      /* Check if block is aligned on 64k or 32k block for faster erase */

      if (((startblock & (sectorsper64kblock - 1)) == 0) &&
          (blocksleft >= sectorsper64kblock))
        {
          /* Erase 64k block */

          ret = gd55_erase_64kblock(priv, startblock);
          if (ret < 0)
            {
              nblocks = ret;
            }

          startblock += sectorsper64kblock;
          blocksleft -= sectorsper64kblock;
          finfo("Erased 64kbytes at address 0x%08" PRIx32 "\n",
                  startblock << GD55_SECTOR_SHIFT);
        }
      else if (((startblock & (sectorsper32kblock - 1)) == 0) &&
          (blocksleft >= sectorsper32kblock))
        {
          /* Erase 32k block */

          ret = gd55_erase_32kblock(priv, startblock);
          if (ret < 0)
            {
              nblocks = ret;
            }

          startblock += sectorsper32kblock;
          blocksleft -= sectorsper32kblock;
          finfo("Erased 32kbytes at address 0x%08" PRIx32 "\n",
                  startblock << GD55_SECTOR_SHIFT);
        }
      else
        {
          /* Erase each sector */

          ret = gd55_erase_sector(priv, startblock);
          if (ret < 0)
            {
             nblocks = ret;
            }

          startblock++;
          blocksleft--;
          finfo("Erased 4kbytes at address 0x%08" PRIx32 "\n",
                  startblock << GD55_SECTOR_SHIFT);
        }
    }
#endif

  ret = (int)nblocks;
  gd55_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: gd55_bread
 *
 * Description:
 *   Read a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the memory to read
 *   nblocks    - nblocks to read
 *   buf        - pointer to the buffer to store the read data
 *
 * Returned Value:
 *   Size of the data read
 *
 ****************************************************************************/

static ssize_t gd55_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf)
{
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the
   * byte-oriented read
   */

#ifdef CONFIG_MTD_GD55_SECTOR512
  nbytes = gd55_read(dev, startblock << GD55_SECTOR512_SHIFT,
                     nblocks << GD55_SECTOR512_SHIFT, buf);
  if (nbytes > 0)
    {
      nbytes >>= GD55_SECTOR512_SHIFT;
    }
#else
  nbytes = gd55_read(dev, startblock << GD55_PAGE_SHIFT,
                       nblocks << GD55_PAGE_SHIFT, buf);
  if (nbytes > 0)
    {
      nbytes >>= GD55_PAGE_SHIFT;
    }
#endif

  return nbytes;
}

/****************************************************************************
 * Name: gd55_bwrite
 *
 * Description:
 *   Write a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the memory to write
 *   nblocks    - nblocks to write
 *   buf        - pointer to the buffer with the data to write
 *
 * Returned Value:
 *   Size of the data written
 *
 ****************************************************************************/

static ssize_t gd55_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct gd55_dev_s *priv = (FAR struct gd55_dev_s *)dev;
  int                   ret;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  gd55_lock(priv);

#if defined(CONFIG_MTD_GD55_SECTOR512)
  ret = gd55_write_cache(priv, buf, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: gd55_write_cache failed: %d\n", ret);
    }
#else
  ret = gd55_write_page(priv, buf, startblock << GD55_PAGE_SHIFT,
                        nblocks << GD55_PAGE_SHIFT);
  if (ret < 0)
    {
      ferr("ERROR: gd55_write_page failed: %d\n", ret);
    }
#endif

  gd55_unlock(priv);

  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: gd55_read
 *
 * Description:
 *   Read a number of bytes of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   offset     - starting address of the memory to read
 *   nbytes     - nbytes to read
 *   buf        - pointer to the buffer to store the read data
 *
 * Returned Value:
 *   Size of the data read
 *
 ****************************************************************************/

static ssize_t gd55_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buffer)
{
  int                   ret;
  FAR struct gd55_dev_s *priv = (FAR struct gd55_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  gd55_lock(priv);
  ret = gd55_read_bytes(priv, buffer, offset, nbytes);
  gd55_unlock(priv);

  if (ret < 0)
    {
      ferr("ERROR: gd55_read_bytes returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: gd55_ioctl
 *
 * Description:
 *   IOCTLS relating to the GD55 mtd device
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   cmd        - ioctl command
 *   arg        - ioctl argument
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 ****************************************************************************/

static int gd55_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct gd55_dev_s *priv = (FAR struct gd55_dev_s *)dev;
  int                    ret  = -EINVAL;

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an
               * array of fixed size blocks.  That is most likely not true,
               * but the client will expect the device logic to do whatever
               * is necessary to make it appear so.
               */

#ifdef CONFIG_MTD_GD55_SECTOR512
              geo->blocksize    = GD55_SECTOR512_SIZE;
              geo->erasesize    = GD55_SECTOR512_SIZE;
              geo->neraseblocks = priv->nsectors <<
                                  (GD55_SECTOR_SHIFT -
                                   GD55_SECTOR512_SHIFT);
#else
              geo->blocksize    = GD55_PAGE_SIZE;
              geo->erasesize    = GD55_SECTOR_SIZE;
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
#ifdef CONFIG_MTD_GD55_SECTOR512
              info->numsectors  = priv->nsectors <<
                               (GD55_SECTOR_SHIFT - GD55_SECTOR512_SHIFT);
              info->sectorsize  = GD55_SECTOR512_SIZE;
#else
              info->numsectors  = priv->nsectors <<
                                  (GD55_SECTOR_SHIFT - GD55_PAGE_SHIFT);
              info->sectorsize  = GD55_PAGE_SIZE;
#endif
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          gd55_lock(priv);
          ret = gd55_protect(priv, prot->startblock, prot->nblocks);
          gd55_unlock(priv);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          gd55_lock(priv);
          ret = gd55_unprotect(priv, prot->startblock, prot->nblocks);
          gd55_unlock(priv);
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          gd55_lock(priv);
          ret = gd55_erase_chip(priv);
          gd55_unlock(priv);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = GD55_ERASED_STATE;

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

/****************************************************************************
 * Name: gd55_readid
 *
 * Description:
 *   Read the device ID.
 *   - the read ID is stored in the cmdbuf variable of the device structure
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_readid(FAR struct gd55_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */

  gd55_lock(priv);

  /* Read the JEDEC ID */

  gd55_command_read(priv, GD55_RDID, priv->cmdbuf, 4);

  /* Unlock the bus */

  gd55_unlock(priv);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x\n",
        priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

  /* Check for GigaDevices GD55 chip */

  if (priv->cmdbuf[0] != GD55_JEDEC_MANUFACTURER &&
      (priv->cmdbuf[1] != GD55L_JEDEC_MEMORY_TYPE ||
       priv->cmdbuf[1] != GD55B_JEDEC_MEMORY_TYPE))
    {
      ferr("ERROR: Unrecognized device type: 0x%02x 0x%02x\n",
           priv->cmdbuf[0], priv->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (priv->cmdbuf[2])
    {
      case GD55_JEDEC_1G_CAPACITY:
        priv->nsectors    = GD55_NSECTORS_1GBIT;
        break;

      case GD55_JEDEC_2G_CAPACITY:
        priv->nsectors    = GD55_NSECTORS_2GBIT;
        break;

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", priv->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: gd55_protect
 *
 * Description:
 *   The GD55 flash supports sector protection either by individual 64KiB
 *   blocks, or in a (64KiB * n^2) block from the bottom of the device memory
 *   OR from the top of the device memory.
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   startblock - first block to protect
 *   nblocks    - nblocks to protect
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_protect(FAR struct gd55_dev_s *priv, off_t startblock,
                        size_t nblocks)
{
  uint8_t status[2];
  int     blkmask;

  if (nblocks < GD55_MIN_BP_BLKS)
    {
      return -EINVAL; /* Too few blocks to protect */
    }

  /* Check if sector protection registers are locked */

  status[0] = gd55_read_status1(priv);
  status[1] = gd55_read_status2(priv);
  if (status[1] & GD55_SR_SRP1)
    {
      /* Status register cannot be written to as device is in
       * power supply lockdown or is set for OTP.
       * If the external HW WP# pin is asserted we won't know until we
       * attempt to unlock sectors though, regardless of state of SRP0 bit
       * in status register 0.
       */

      return -EACCES;
    }

  if (nblocks == (priv->nsectors * GD55_SECTORS_PER_BP_BLK))
    {
      if (startblock == 0)
        {
          blkmask = GD55_BP_ALL; /* protect every block */
        }
      else
        {
          return -EINVAL;        /* Invalid size and startblock */
        }
    }
  else
    {
      /* We can only protect in certain increments of size */

      blkmask = 0;
      while (nblocks > (GD55_MIN_BP_BLKS << blkmask))
        {
          if ((startblock % (GD55_MIN_BP_BLKS << blkmask)) ||
              (nblocks % (GD55_MIN_BP_BLKS << blkmask)))
            {
              return -EINVAL; /* Not a size we can protect */
            }

          blkmask++;
        }

        blkmask = (startblock == 0) ? GD55_SR_BP_BOTTOM(blkmask) :
                                      GD55_SR_BP_TOP(blkmask);
    }

  /* startblock must be first block, or (memory top - nblocks) */

  if ((startblock != 0) &&
      (startblock != (((priv->nsectors << GD55_SECTOR_SHIFT) /
                        GD55_MIN_BP_BLKS) - nblocks)))
    {
      return -EINVAL;
    }

  /* Clear the relevant status register bits for the new mask */

  priv->cmdbuf[0] = status[0] & ~GD55_SR_BP_MASK;

  /* Now set them */

  priv->cmdbuf[0] |= blkmask;

  if ((priv->cmdbuf[0] & GD55_SR_BP_MASK) == (status[0] & GD55_SR_BP_MASK))
    {
      return OK; /* this protection is already set */
    }

  gd55_write_status1(priv);
  status[0] = gd55_read_status1(priv);
  if ((status[0] & GD55_SR_BP_MASK) != (priv->cmdbuf[0] & GD55_SR_BP_MASK))
    {
      return -EACCES; /* Likely that the external HW WP# pin is asserted */
    }

  return OK;
}

/****************************************************************************
 * Name: gd55_unprotect
 *
 * Description:
 *   The GD55 flash supports sector protection either by individual 64KiB
 *   blocks, or in a (64KiB * n^2) block from the bottom of the device memory
 *   OR from the top of the device memory.
 *
 *   This function removes protection from all blocks
 *
 *   REVISIT - there may be benefit from trying to only unprotect a range of
 *   sectors but this means complex checking of the request range against the
 *   current range of blocks that are currently protected so is non-trivial
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   startblock - first block to unprotect (ignored for now)
 *   nblocks    - nblocks to unprotect (ignored for now)
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_unprotect(FAR struct gd55_dev_s *priv, off_t startblock,
                          size_t nblocks)
{
  uint8_t status[2];

  /* Check if sector protection registers are locked */

  status[0] = gd55_read_status1(priv);
  status[1] = gd55_read_status2(priv);
  if (status[1] & GD55_SR_SRP1)
    {
      /* Status register cannot be written to as device is in
       * power supply lockdown or is set for OTP.
       * If the external HW WP# pin is asserted we won't know until we
       * attempt to unlock sectors though, regardless of state of SRP0 bit
       * in status register 0.
       */

      return -EACCES;
    }

  if (!(status[0] & GD55_SR_BP_MASK))
    {
      return OK; /* all blocks are already unprotected */
    }

  /* Clear all the status register BP bits */

  priv->cmdbuf[0] = status[0] & ~GD55_SR_BP_MASK;

  gd55_write_status1(priv);
  status[0] = gd55_read_status1(priv);
  if ((status[0] & GD55_SR_BP_MASK) != (priv->cmdbuf[0] & GD55_SR_BP_MASK))
    {
      return -EACCES; /* Likely that the external HW WP# pin is asserted */
    }

  return OK;
}

/****************************************************************************
 * Name: gd55_isprotected
 *
 * Description:
 *   Check if an address has been write protected
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   addr       - address to check
 *   status     - the previously read status register value
 *
 * Returned Value:
 *   Protected (true) or unprotected (false)
 *
 ****************************************************************************/

static bool gd55_isprotected(FAR struct gd55_dev_s *priv, off_t addr,
                             uint8_t status)
{
  off_t        protstart;
  off_t        protend;
  off_t        protsize;
  unsigned int bp;

  /* the BP field is essentially the power-of-two of the number of 64k
   * sectors that are protected, saturated to the device size.
   * The msb determines if protection is:
   *   - top down  (msb not set)
   *   - bottom up (msb set)
   */

  bp = (status & GD55_SR_BP_MASK);
  bp &= ~GD55_STATUS_TB_MASK; /* Ignore top/bottom for now */
  bp >>= GD55_SR_BP_SHIFT;

  if (bp == 0)
    {
      return false;
    }

  protsize = GD55_BP_SIZE;
  protsize <<= (bp - 1);
  protend = GD55_SECTOR_SIZE * priv->nsectors;
  if (protsize > protend)
    {
      protsize = protend;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up.
   */

  if ((status & GD55_STATUS_TB_BOTTOM))
    {
      protstart = 0;
      protend   = protstart + protsize;
    }
  else
    {
      protstart = protend - protsize;

      /* protend already computed above */
    }

  return (addr >= protstart && addr < protend);
}

#ifdef CONFIG_MTD_GD55_SECTOR512
/****************************************************************************
 * Name: gd55_flush_cache
 *
 * Description:
 *   If the cache is dirty (meaning that it no longer matches the old FLASH
 *   contents) or was erased (with the cache containing the correct FLASH
 *   contents), then write the cached erase block to FLASH.
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_flush_cache(FAR struct gd55_dev_s *priv)
{
  int ret = OK;

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector number into a FLASH address */

      address = (off_t)priv->esectno << GD55_SECTOR_SHIFT;

      /* Write entire erase block to FLASH */

      ret = gd55_write_page(priv, priv->sector, address, GD55_SECTOR_SIZE);
      if (ret < 0)
        {
          ferr("ERROR: gd55_write_page failed: %d\n", ret);
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: gd55_read_cache
 *
 * Description:
 *   Read cached data
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   sector     = sector to read
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static FAR uint8_t *gd55_read_cache(FAR struct gd55_dev_s *priv,
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

  shift    = GD55_SECTOR_SHIFT - GD55_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %jd esectno: %jd (%d) shift=%d\n",
        (intmax_t)sector, (intmax_t)esectno, priv->esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = gd55_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: gd55_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = gd55_read_bytes(priv, priv->sector,
                             (esectno << GD55_SECTOR_SHIFT),
                              GD55_SECTOR_SIZE);
      if (ret < 0)
        {
          ferr("ERROR: gd55_read_bytes failed: %d\n", ret);
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

  return &priv->sector[index << GD55_SECTOR512_SHIFT];
}

/****************************************************************************
 * Name: gd55_erase_cache
 *
 * Description:
 *   erase cached data
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   sector     = sector to read
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static void gd55_erase_cache(FAR struct gd55_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is
   * in the cache.
   */

  dest = gd55_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >>
                      (GD55_SECTOR_SHIFT - GD55_SECTOR512_SHIFT);
      finfo("sector: %jd esectno: %jd\n",
            (intmax_t)sector, (intmax_t)esectno);

      DEBUGVERIFY(gd55_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mark the cache as
   * dirty (but don't update the FLASH yet.  The caller will do that at a
   * more optimal time).
   */

  memset(dest, GD55_ERASED_STATE, GD55_SECTOR512_SIZE);
  SET_DIRTY(priv);
}

/****************************************************************************
 * Name: gd55_write_cache
 *
 * Description:
 *   write cached data
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   sector     = sector to read
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int gd55_write_cache(FAR struct gd55_dev_s *priv,
                            FAR const uint8_t *buffer, off_t sector,
                            size_t nsectors)
{
  FAR uint8_t *dest;
  int         ret;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = gd55_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >>
                           (GD55_SECTOR_SHIFT - GD55_SECTOR512_SHIFT);
          finfo("sector: %jd esectno: %jd\n",
                (intmax_t)sector, (intmax_t)esectno);

          ret = gd55_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: gd55_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, GD55_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      finfo("address: %08jx nbytes: %d 0x%04" PRIx32 "\n",
            (intmax_t)(sector << GD55_SECTOR512_SHIFT),
            GD55_SECTOR512_SIZE,
            *(FAR uint32_t *)buffer);
      buffer += GD55_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return gd55_flush_cache(priv);
}
#endif /* CONFIG_MTD_GD55_SECTOR512 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd55_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 * Input Parameters:
 *   qspi       - a reference to the qspi device to initialize
 *   unprotect  - if true, unprotect the device
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 ****************************************************************************/

FAR struct mtd_dev_s *gd55_initialize(FAR struct qspi_dev_s *qspi,
                                      bool unprotect)
{
  FAR struct gd55_dev_s *dev;
  int                   ret;
  uint8_t               status;

  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      ferr("Failed to allocate mtd device\n");
      return NULL;
    }

  dev->mtd.erase  = gd55_erase;
  dev->mtd.bread  = gd55_bread;
  dev->mtd.bwrite = gd55_bwrite;
  dev->mtd.read   = gd55_read;
  dev->mtd.ioctl  = gd55_ioctl;
  dev->mtd.name   = "gd55";
  dev->qspi       = qspi;

  /* Allocate a 4-byte buffer to support DMA-able command data */

  dev->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
  if (dev->cmdbuf == NULL)
    {
      ferr("Failed to allocate command buffer\n");
      goto exit_free_dev;
    }

  dev->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 2);
  if (dev->readbuf == NULL)
    {
      ferr("ERROR Failed to allocate read buffer\n");
      goto exit_free_cmdbuf;
    }

  /* Identify the FLASH chip and get its capacity */

  ret = gd55_readid(dev);
  if (ret != OK)
    {
      /* Unrecognized! Discard all of that work we just did and return NULL */

      ferr("Unrecognized QSPI device\n");
      goto exit_free_readbuf;
    }

  /* Unprotect all FLASH sectors if so requested. */

  if (unprotect)
    {
      ret = gd55_unprotect(dev, 0, dev->nsectors - 1);
      if (ret < 0)
        {
          ferr("ERROR: Sector unprotect failed\n");
        }
    }

#ifdef CONFIG_MTD_GD55_SECTOR512  /* Simulate a 512 byte sector */
  /* Allocate a buffer for the erase block cache */

  dev->sector = (FAR uint8_t *)QSPI_ALLOC(qspi, GD55_SECTOR_SIZE);
  if (dev->sector == NULL)
    {
      /* Allocation failed! Discard all of that work we just did and
       * return NULL
       */

      ferr("ERROR: Sector allocation failed\n");
      goto exit_free_readbuf;
    }
#endif

  status = gd55_read_status1(dev);

  /* Avoid compiler warnings in case info logs are disabled */

  UNUSED(status);

  finfo("device ready Status  = 0x%02x\n", status);

  /* Return the implementation-specific state structure as the MTD device */

  return &dev->mtd;

exit_free_readbuf:
  QSPI_FREE(qspi, dev->readbuf);
exit_free_cmdbuf:
  QSPI_FREE(qspi, dev->cmdbuf);
exit_free_dev:
  kmm_free(dev);
  return NULL;
}
