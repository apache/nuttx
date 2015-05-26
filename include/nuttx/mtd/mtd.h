/****************************************************************************
 * include/nuttx/mtd/mtd.h
 * Memory Technology Device (MTD) interface
 *
 *   Copyright (C) 2009-2013, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_MTD_MTD_H
#define __INCLUDE_NUTTX_MTD_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to hide implementation */

#define MTD_ERASE(d,s,n)   ((d)->erase   ? (d)->erase(d,s,n)    : (-ENOSYS))
#define MTD_BREAD(d,s,n,b) ((d)->bread   ? (d)->bread(d,s,n,b)  : (-ENOSYS))
#define MTD_BWRITE(d,s,n,b)((d)->bwrite  ? (d)->bwrite(d,s,n,b) : (-ENOSYS))
#define MTD_READ(d,s,n,b)  ((d)->read    ? (d)->read(d,s,n,b)   : (-ENOSYS))
#define MTD_WRITE(d,s,n,b) ((d)->write   ? (d)->write(d,s,n,b)  : (-ENOSYS))
#define MTD_IOCTL(d,c,a)   ((d)->ioctl   ? (d)->ioctl(d,c,a)    : (-ENOSYS))

/* If any of the low-level device drivers declare they want sub-sector erase
 * support, then define MTD_SUBSECTOR_ERASE.
 */

#if defined(CONFIG_M25P_SUBSECTOR_ERASE)
#  define CONFIG_MTD_SUBSECTOR_ERASE 1
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MTD)
#define CONFIG_MTD_REGISTRATION   1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The following defines the geometry for the device.  It treats the device
 * as though it where just an array of fixed size blocks.  That is most likely
 * not true, but the client will expect the device logic to do whatever is
 * necessary to make it appear so.
 */

struct mtd_geometry_s
{
  uint16_t blocksize;     /* Size of one read/write block. */
                          /* Probably 16-bits wasted here for alignment */
  uint32_t erasesize;     /* Size of one erase blocks -- must be a multiple
                           * of blocksize.*/
  uint32_t neraseblocks;  /* Number of erase blocks */
};

/* The following defines the information for writing bytes to a sector
 * that are not a full page write (bytewrite).
 */

struct mtd_byte_write_s
{
  uint32_t offset;        /* Offset within the device to write to */
  uint16_t count;         /* Number of bytes to write */
  const uint8_t *buffer;  /* Pointer to the data to write */
};

/* This structure defines the interface to a simple memory technology device.
 * It will likely need to be extended in the future to support more complex
 * devices.
 */

struct mtd_dev_s
{
  /* The following methods operate on the MTD: */

  /* Erase the specified erase blocks (units are erase blocks).  Semantic
   * Clarification:  Here, we are not referring to the erase block according
   * to the FLASH data sheet.  Rather, we are referring to the *smallest*
   * erasable part of the FLASH which may have a name like a page or sector
   * or subsector.
   */

  int (*erase)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);

  /* Read/write from the specified read/write blocks */

  ssize_t (*bread)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                   FAR uint8_t *buffer);
  ssize_t (*bwrite)(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                    FAR const uint8_t *buffer);

  /* Some devices may support byte oriented reads (optional).  Most MTD devices
   * are inherently block oriented so byte-oriented writing is not supported. It
   * is recommended that low-level drivers not support read() if it requires
   * buffering.
   */

  ssize_t (*read)(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                  FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
  ssize_t (*write)(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                   FAR const uint8_t *buffer);
#endif

  /* Support other, less frequently used commands:
   *  - MTDIOC_GEOMETRY:  Get MTD geometry
   *  - MTDIOC_XIPBASE:   Convert block to physical address for eXecute-In-Place
   *  - MTDIOC_BULKERASE: Erase the entire device
   * (see include/nuttx/fs/ioctl.h)
   */

  int (*ioctl)(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

#ifdef CONFIG_MTD_REGISTRATION
  /* An assigned MTD number for procfs reporting */

  uint8_t mtdno;

  /* Pointer to the next registered MTD device */

  FAR struct mtd_dev_s  *pnext;

  /* Name of this MTD device */

  FAR const char *name;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* MTD Support **************************************************************/

/****************************************************************************
 * Name: mtd_partition
 *
 * Description:
 *   Given an instance of an MTD driver, create a flash partition, ie.,
 *   another MTD driver instance that only operates with a sub-region of
 *   FLASH media.  That sub-region is defined by a sector offset and a
 *   sector count (where the size of a sector is provided the by parent MTD
 *   driver).
 *
 *   NOTE: Since there may be a number of MTD partition drivers operating on
 *   the same, underlying FLASH driver, that FLASH driver must be capable
 *   of enforcing mutually exclusive access to the FLASH device.  Without
 *   partitions, that mutual exclusion would be provided by the file system
 *   above the FLASH driver.
 *
 * Input parameters:
 *   mtd        - The MTD device to be partitioned
 *   firstblock - The offset in bytes to the first block
 *   nblocks    - The number of blocks in the partition
 *
 * Returned Value:
 *   On success, another MTD device representing the partition is returned.
 *   A NULL value is returned on a failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
                                    off_t firstblock, off_t nblocks);

/****************************************************************************
 * Name: mtd_setpartitionname
 *
 * Description:
 *   Sets the name of the specified partition.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_PARTITION_NAMES
int mtd_setpartitionname(FAR struct mtd_dev_s *mtd, FAR const char *name);
#endif

/************************************************************************************
 * Name: mtd_rwb_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  This MTD driver contains another
 *   MTD driver and converts a larger sector size to a standard 512 byte sector
 *   size.
 *
 *   MTD devices are not registered in the file system, but are created as instances
 *   that can be bound to other functions (such as a block or character driver front
 *   end).
 *
 ************************************************************************************/

#if defined(CONFIG_MTD_WRBUFFER) || defined(CONFIG_MTD_READAHEAD)
FAR struct mtd_dev_s *mtd_rwb_initialize(FAR struct mtd_dev_s *mtd);
#endif

/****************************************************************************
 * Name: ftl_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/mtdblockN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int ftl_initialize(int minor, FAR struct mtd_dev_s *mtd);

/****************************************************************************
 * Name: flash_eraseall
 *
 * Description:
 *   Call a block driver with the MTDIOC_BULKERASE ioctl command.  This will
 *   cause the MTD driver to erase all of the flash.
 *
 ****************************************************************************/

int flash_eraseall(FAR const char *driver);

/****************************************************************************
 * Name: smart_initialize
 *
 * Description:
 *   Initialize to provide a Sector Mapped Allocation for Really Tiny (SMART)
 *   Flash block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/mtdsmartN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *   partname - Optional partition name to append to dev entry, NULL if
 *              not supplied.
 *
 ****************************************************************************/

int smart_initialize(int minor, FAR struct mtd_dev_s *mtd,
                     FAR const char *partname);

/* MTD Driver Initialization ************************************************/
/* Create an initialized MTD device instance for a particular memory device.
 * MTD devices are not registered in the file system as are other device
 * driver but, but are created as instances that can be bound to other
 * functions (such as a block or character driver front end).
 */

/************************************************************************************
 * Name: s512_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  This MTD driver contains another
 *   MTD driver and converts a larger sector size to a standard 512 byte sector
 *   size.
 *
 *   MTD devices are not registered in the file system, but are created as instances
 *   that can be bound to other functions (such as a block or character driver front
 *   end).
 *
 ************************************************************************************/

#ifdef CONFIG_MTD_SECT512
FAR struct mtd_dev_s *s512_initialize(FAR struct mtd_dev_s *mtd);
#endif

/****************************************************************************
 * Name: at45db_initialize
 *
 * Description:
 *   Initializes the driver for SPI-based AT45DB161D (16Mbit).
 *
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */
FAR struct mtd_dev_s *at45db_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: at24c_initialize
 *
 * Description:
 *   Initializes the driver for I2C-based at24cxx EEPROM(AT24C32, AT24C64,
 *   AT24C128, AT24C256)
 *
 ****************************************************************************/

struct i2c_dev_s; /* Forward reference */
FAR struct mtd_dev_s *at24c_initialize(FAR struct i2c_dev_s *dev);

/****************************************************************************
 * Name: at25_initialize
 *
 * Description:
 *   Initializes the driver for SPI-based AT25DF321 (32Mbit) flash.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *at25_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: m25p_initialize
 *
 * Description:
 *   Initializes the for SPI-based M25P1 (128Kbit),  M25P64 (32Mbit), M25P64
 *   (64Mbit), and M25P128 (128Mbit) FLASH (and compatible).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *m25p_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: rammtd_initialize
 *
 * Description:
 *   Create and initialize a RAM MTD device instance.
 *
 * Input Parameters:
 *   start - Address of the beginning of the allocated RAM regions.
 *   size  - The size in bytes of the allocated RAM region.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *rammtd_initialize(FAR uint8_t *start, size_t size);

/****************************************************************************
 * Name: ramtron_initialize
 *
 * Description:
 *   Create and initialize a Ramtron MTD device instance.
 *
 * Input Parameters:
 *   start - Address of the beginning of the allocated RAM regions.
 *   size  - The size in bytes of the allocated RAM region.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: sst25_initialize
 *
 * Description:
 *   Initializes the driver for SPI-based SST25 FLASH
 *
 *   Supports SST25VF512, SST25VF010, SST25VF520, SST25VF540, SST25VF080,
 *   and SST25VF016
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst25_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: sst25xx_initialize
 *
 * Description:
 *   Initializes the driver for SPI-based SST25XX FLASH
 *
 *   Supports SST25VF064
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst25xx_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: sst39vf_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance assuming an SST39VF NOR
 *   FLASH device at the configured address in memory.
 *
 *   Supports SST39VF1601, SST39VF1602, SST39VF3201, SST39VF3202.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *sst39vf_initialize(void);

/****************************************************************************
 * Name: w25_initialize
 *
 * Description:
 *   Initializes the driver for SPI-based W25x16, x32, and x64 and W25q16,
 *   q32, q64, and q128 FLASH
 *
 ****************************************************************************/

FAR struct mtd_dev_s *w25_initialize(FAR struct spi_dev_s *dev);

/****************************************************************************
 * Name: up_flashinitialize
 *
 * Description:
 *   Create an initialized MTD device instance for internal flash.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *up_flashinitialize(void);

/****************************************************************************
 * Name: mtd_register
 *
 * Description:
 *   Registers MTD device with the procfs file system.  This assigns a unique
 *   MTD number and associates the given device name, then  add adds it to
 *   the list of registered devices.
 *
 * In an embedded system, this all is really unnecessary, but is provided
 * in the procfs system simply for information purposes (if desired).
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_REGISTRATION
int mtd_register(FAR struct mtd_dev_s *mtd, FAR const char *name);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_MTD_MTD_H */
