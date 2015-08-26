/************************************************************************************
 * drivers/mtd/st25fl1.c
 * Driver for QuadSPI-based S25FL116K, S25FL132K, and S25L164K
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* QuadSPI Mode */

#ifndef CONFIG_ST25FL1_QSPIMODE
#  define CONFIG_ST25FL1_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency.  May be up to 25MHz. */

#ifndef CONFIG_ST25FL1_QSPIFREQUENCY
#  define CONFIG_ST25FL1_QSPIFREQUENCY 20000000
#endif

/* ST25FL1 Commands *****************************************************************/
/* Configuration, Status, Erase, Program Commands ***********************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define ST25FL1_READ_STATUS1       0x05  /* Read status register 1:                 *
                                          *   0x05 | SR1                            */
#define ST25FL1_READ_STATUS2       0x35  /* Read status register 2:                 *
                                          *   0x35 | SR2                            */
#define ST25FL1_READ_STATUS3       0x33  /* Read status register 3:                 *
                                          *   0x33 | SR3                            */
#define ST25FL1_WRITE_ENABLE       0x06  /* Write enable:                           *
                                          *   0x06                                  */
#define ST25FL1_VWRITE_ENABLE      0x50  /* Write enable for volatile status:       *
                                          *   0x50                                  */
#define ST25FL1_WRITE_DISABLE      0x04  /* Write disable command code:             *
                                          *   0x04                                  */
#define ST25FL1_WRITE_STATUS       0x01  /* Write status register:                  *
                                          *   0x01 | SR1 | SR2 | SR3                */
#define ST25FL1_WRAP_ENABLE        0x77  /* Set Burst with Wrap:                    *
                                          *   0x77 | xx | xx | xx | SR3             */
#define ST25FL1_UNPROTECT_SECTOR   0x39  /* Set Block / Pointer Protection:         *
                                          *   0x39 | ADDR(MS) | ADDR(MID) | xx      */
#define ST25FL1_PAGE_PROGRAM       0x02  /* Page Program:                           *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data                       */
#define ST25FL1_SECTOR_ERASE       0x20  /* Sector Erase (4 kB)                     *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define ST25FL1_BLOCK_ERASE        0xd8  /* Block Erase (64 kB):                    *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define ST25FL1_CHIP_ERASE_1       0x60  /* Chip Erase 1:                           *
                                          *   0x60                                  */
#define ST25FL1_CHIP_ERASE_2       0xc7  /* Chip Erase 2:                           *
                                          *   0xc7                                  */
#define ST25FL1_ERASE_PROG_SUSPEND 0x75  /* Erase / Program Suspend:                *
                                          *   0x75                                  */
#define ST25FL1_ERASE_PROG_RESUME  0x7a  /* Erase / Program Resume:                 *
                                          *   0x7a                                  */

/* Read Commands ********************************************************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define ST25FL1_READ_DATA          0x03  /* Read Data:                              *
                                          *   0x03 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data...                    */
#define ST25FL1_FAST_READ          0x0b  /* Fast Read:                              *
                                          *   0x0b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define ST25FL1_FAST_READ_DUAL     0x3b  /* Fast Read Dual Output:                  *
                                          *   0x3b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define ST25FL1_FAST_READ_QUAD     0x6b  /* Fast Read Dual Output:                  *
                                          *   0x6b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define ST25FL1_FAST_READ_DUALIO   0xbb  /* Fast Read Dual I/O:                     *
                                          *   0xbb | ADDR(MS) | ADDR(LS) | data...  */
#define ST25FL1_FAST_READ_QUADIO   0xeb  /* Fast Read Quad I/O:                     *
                                          *   0xeb | ADDR | data...                 */
#define ST25FL1_CONT_READ_RESET    0xff  /* Continuous Read Mode Reset:             *
                                          *   0xff | 0xff                           */

/* Reset Commands *******************************************************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define ST25FL1_SOFT_RESET_ENABLE  0x66  /* Software Reset Enable:                  *
                                          *   0x66                                  */
#define ST25FL1_SOFT_RESET         0x99  /* Software Reset:                         *
                                          *   0x99                                  */
                                         /* Continuous Read Mode Reset:             *
                                          *   0xff | 0xff                           */

/* ID/Security Commands *************************&***********************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define ST25FL1_DEEP_PWRDOWN       0xb9  /* Deep Power-down:                        *
                                          *   0xb9                                  */
#define ST25FL1_RELEASE_PWRDOWN    0xab  /* Release Power down / Device ID:         *
                                          *   0xab | dummy | dummy | dummy |        *
                                          *   DeviceID  */
#define ST25FL1_MANUFACTURER       0x90  /* Manufacturer / Device ID:               *
                                          *   0x90 | dummy | dummy | 0x00 |         *
                                          *   Manufacturer | DeviceID               */
#define ST25FL1_JEDEC_ID           0x9f  /* JEDEC ID:                               *
                                          *   0x9f | Manufacturer | MemoryType |    *
                                          *   Capacity                              */
#define ST25FL1_READ_SFDP          0x5a  /* Read SFDP Register / Read Unique ID     *
                                          * Number:                                 *
                                          *   0x5a | 0x00 | 0x00 | ADDR | dummy |   *
                                          *   data...                               */
#define ST25FL1_READ_SECURITY      0x48  /* Read Security Registers:                *
                                          *   0x48 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define ST25FL1_ERASE_SECURITY     0x44  /* Erase Security Registers:               *
                                          *   0x48 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define ST25FL1_PROgRAM_SECURITY   0x42  /* Program Security Registers:             *
                                          *   0x42 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data...                    */

/* Flash Manufacturer JEDEC IDs */

#define ST25FL1_JEDEC_ID_ATMEL     0x1f
#define ST25FL1_JEDEC_ID_ST        0x20
#define ST25FL1_JEDEC_ID_SST       0xbf
#define ST25FL1_JEDEC_ID_MACRONIX  0xc2
#define ST25FL1_JEDEC_ID_WINBOND   0xef

/* ST25FL1 Registers ****************************************************************/
/* Status register bit definitions                                                  */

#define STATUS1_BUSY_MASK          (1 << 0) /* Bit 0: Device ready/busy status      */
#  define STATUS1_READY            (0 << 0) /*   0 = Not Busy                       */
#  define STATUS1_BUSY             (1 << 0) /*   1 = Busy                           */
#define STATUS1_WEL_MASK           (1 << 1) /* Bit 1: Write enable latch status     */
#  define STATUS1_WEL_DISABLED     (0 << 1) /*   0 = Not Write Enabled              */
#  define STATUS1_WEL_ENABLED      (1 << 1) /*   1 = Write Enabled                  */
#define STATUS1_BP_SHIFT           (2)      /* Bits 2-4: Block protect bits         */
#define STATUS1_BP_MASK            (7 << STATUS1_BP_SHIFT)
#  define STATUS1_BP_NONE          (0 << STATUS1_BP_SHIFT)
#  define STATUS1_BP_ALL           (7 << STATUS1_BP_SHIFT)
#define STATUS1_TB_MASK            (1 << 5) /* Bit 5: Top / Bottom Protect          */
#  define STATUS1_TB_TOP           (0 << 5) /*   0 = BP2-BP0 protect Top down       */
#  define STATUS1_TB_BOTTOM        (1 << 5) /*   1 = BP2-BP0 protect Bottom up      */
#define STATUS1_SEC_MASK           (1 << 6) /* Bit 6: Sector / Block Protect        */
#  define STATUS1_SEC_BLOCK        (0 << 6) /*   0 = BP2-BP0 protect 64-kB blocks   */
#  define STATUS1_SEC_BLOCK        (1 << 6) /*   1 = BP2-BP0 protect 4-kB sectors   */
#define STATUS1_SRP0_MASK          (1 << 7) /* Bit 7: Status register protect 0     */
#  define STATUS1_SRP0_UNLOCKED    (0 << 7) /*   0 = WP# no effect / PS Lock Down   */
#  define STATUS1_SRP0_LOCKED      (1 << 7) /*   1 = WP# protect / OTP Lock Down    */

#define STATUS2_SRP1_MASK          (1 << 0) /* Bit 0: Status register protect 1     */
#  define STATUS2_SRP1_UNLOCKED    (0 << 0) /*   0 = WP# no effect / PS Lock Down   */
#  define STATUS2_SRP1_LOCKED      (1 << 0) /*   1 = WP# protect / OTP Lock Down    */
#define STATUS2_QUAD_ENABLE_MASK   (1 << 1) /* Bit 1: Quad Enable                   */
#  define STATUS2_QUAD_DISABLE     (0 << 1) /*   0 = Quad Mode Not Enabled          */
#  define STATUS2_QUAD_ENABLE      (1 << 1) /*   1 = Quad Mode Enabled              */
#define STATUS2_LB_SHIFT           (2)      /* Bits 2-5: Security Register Lock     */
#define STATUS2_LB_MASK            (15 << STATUS2_LB_SHIFT)
#  define STATUS2_LB_NONE          (0 << STATUS2_LB_SHIFT)
#  define STATUS2_LB_ALL           (15 << STATUS2_LB_SHIFT)
#define STATUS2_CMP_MASK           (1 << 6) /* Bit 6: Complement Protect            */
#  define STATUS2_CMP_NORMAL       (0 << 6) /*   0 = Normal Protection Map          */
#  define STATUS2_CMP_INVERTED     (1 << 6) /*   1 = Inverted Protection Map        */
#define STATUS2_SUS_MASK           (1 << 7) /* Bit 7: Suspend Status                */
#  define STATUS2_SUS_NONE         (0 << 7) /*   0 = Erase / Program not suspended  */
#  define STATUS2_SUS_SUSPENDED    (1 << 7) /*   1 = Erase / Program suspended      */

#define STATUS3_LC_SHIFT           (0)      /* Bits 0-3: Latency control            */
#define STATUS3_LC_MASK            (15 << STATUS3_LC_SHIFT)
#define STATUS3_W4_MASK            (1 << 4) /* Bit 4: Burst Wrap Enable             */
#  define STATUS3_W4_DISABLED      (0 << 4) /*   0 = Wrap Enabled                   */
#  define STATUS3_W4_ENABLED       (1 << 4) /*   1 = Wrap Disabled                  */
#define STATUS3_W56_SHIFT           (5)      /* Bits 5-6: Burst Wrap Length          */
#define STATUS3_W56_MASK            (3 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_8BYTE         (0 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_16BYTE        (1 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_32BYTE        (2 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_63BYTE        (3 << STATUS3_W56_SHIFT)
                                            /* Bit 7: Reserved                      */


/* Chip Geometries ******************************************************************/
/* All members of the family support uniform 4K-byte sectors  */

#define S25FL116K_SECTOR_SIZE      (4*1024)
#define S25FL116K_SECTOR_COUNT     (512)

#define S25FL132K_SECTOR_SIZE      (4*1024)
#define S25FL132K_SECTOR_COUNT     (1024)

#define S25FL164K_SECTOR_SIZE      (4*1024)
#define S25FL164K_SECTOR_COUNT     (2048)

/* Cache flags **********************************************************************/

#define ST25FL1_CACHE_VALID        (1 << 0)  /* 1=Cache has valid data */
#define ST25FL1_CACHE_DIRTY        (1 << 1)  /* 1=Cache is dirty */
#define ST25FL1_CACHE_ERASED       (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & ST25FL1_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & ST25FL1_CACHE_DIRTY) != 0)
#define IS_ERASED(p)               ((((p)->flags) & ST25FL1_CACHE_DIRTY) != 0)

#define SET_VALID(p)               do { (p)->flags |= ST25FL1_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= ST25FL1_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= ST25FL1_CACHE_DIRTY; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~ST25FL1_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~ST25FL1_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~ST25FL1_CACHE_DIRTY; } while (0)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct st25fl1_dev_s.
 */

struct st25fl1_dev_s
{
  struct mtd_dev_s      mtd;         /* MTD interface */
  FAR struct spi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t              nsectors;    /* Number of erase sectors */

#ifdef CONFIG_ST25FL1_SECTOR512
  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache*/
  FAR uint8_t          *sector;      /* Allocated sector data */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static void st25fl1_lock(FAR struct spi_dev_s *qspi);
static inline void st25fl1_unlock(FAR struct spi_dev_s *qspi);
static inline int st25fl1_readid(FAR struct st25fl1_dev_s *priv);
static void st25fl1_unprotect(FAR struct st25fl1_dev_s *priv);
static uint8_t st25fl1_waitwritecomplete(FAR struct st25fl1_dev_s *priv);
static inline void st25fl1_wren(FAR struct st25fl1_dev_s *priv);
static inline void st25fl1_wrdi(FAR struct st25fl1_dev_s *priv);
static void st25fl1_sectorerase(FAR struct st25fl1_dev_s *priv, off_t offset);
static inline int st25fl1_chiperase(FAR struct st25fl1_dev_s *priv);
static void st25fl1_byteread(FAR struct st25fl1_dev_s *priv, FAR uint8_t *buffer,
                             off_t address, size_t nbytes);
static void st25fl1_pagewrite(FAR struct st25fl1_dev_s *priv, FAR const uint8_t *buffer,
                              off_t address, size_t nbytes);
#ifdef CONFIG_ST25FL1_SECTOR512
static void st25fl1_cacheflush(struct st25fl1_dev_s *priv);
static FAR uint8_t *st25fl1_cacheread(struct st25fl1_dev_s *priv, off_t sector);
static void st25fl1_cacheerase(struct st25fl1_dev_s *priv, off_t sector);
static void st25fl1_cachewrite(FAR struct st25fl1_dev_s *priv, FAR const uint8_t *buffer,
                               off_t sector, size_t nsectors);
#endif

/* MTD driver methods */

static int st25fl1_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t st25fl1_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
static ssize_t st25fl1_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf);
static ssize_t st25fl1_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                            FAR uint8_t *buffer);
static int st25fl1_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: st25fl1_lock
 ************************************************************************************/

static void st25fl1_lock(FAR struct spi_dev_s *qspi)
{
  /* On QuadSPI busses where there are multiple devices, it will be necessary to
   * lock QuadSPI to have exclusive access to the busses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusiv access to
   * the QuadSPI buss.  We will retain that exclusive access until the bus is unlocked.
   */

  (void)QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the QuadSPI is properly configured for the device.
   * If the QuadSPI buss is being shared, then it may have been left in an incompatible
   * state.
   */

  QSPI_SETMODE(qspi, CONFIG_ST25FL1_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  (void)QSPI_SETFREQUENCY(qspi, CONFIG_ST25FL1_QSPIFREQUENCY);
}

/************************************************************************************
 * Name: st25fl1_unlock
 ************************************************************************************/

static inline void st25fl1_unlock(FAR struct spi_dev_s *qspi)
{
  (void)QSPI_LOCK(qspi, false);
}

/************************************************************************************
 * Name: st25fl1_readid
 ************************************************************************************/

static inline int st25fl1_readid(struct st25fl1_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t memory;
  uint16_t capacity;

  fvdbg("priv: %p\n", priv);

  /* Lock the QuadSPI bus and configure the bus. */

  st25fl1_lock(priv->qspi);

#warning Missing Logic

  /* Unlock the bus */

  st25fl1_unlock(priv->qspi);

  fvdbg("manufacturer: %02x memory: %02x capacity: %02x\n",
        manufacturer, memory, capacity);

  /* Check for a valid manufacturer and memory type */
#warning Missing Logic

  /* We don't understand the manufacturer or the memory type */

  return -ENODEV;
}

/************************************************************************************
 * Name: st25fl1_unprotect
 ************************************************************************************/

static void st25fl1_unprotect(FAR struct st25fl1_dev_s *priv)
{
#warning Missing Logic
}

/************************************************************************************
 * Name: st25fl1_waitwritecomplete
 ************************************************************************************/

static uint8_t st25fl1_waitwritecomplete(struct st25fl1_dev_s *priv)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
#warning Missing Logic
    }
  while ((status & ST25FL1_SR_BUSY) != 0);

  return status;
}

/************************************************************************************
 * Name:  st25fl1_wren
 ************************************************************************************/

static inline void st25fl1_wren(struct st25fl1_dev_s *priv)
{
#warning Missing Logic
}

/************************************************************************************
 * Name:  st25fl1_wrdi
 ************************************************************************************/

static inline void st25fl1_wrdi(struct st25fl1_dev_s *priv)
{
#warning Missing Logic
}

/************************************************************************************
 * Name:  st25fl1_sectorerase
 ************************************************************************************/

static void st25fl1_sectorerase(struct st25fl1_dev_s *priv, off_t sector)
{
  fvdbg("sector: %08lx\n", (long)sector);
#warning Missing Logic
}

/************************************************************************************
 * Name:  st25fl1_chiperase
 ************************************************************************************/

static inline int st25fl1_chiperase(struct st25fl1_dev_s *priv)
{
  fvdbg("priv: %p\n", priv);
#warning Missing Logic
}

/************************************************************************************
 * Name: st25fl1_byteread
 ************************************************************************************/

static void st25fl1_byteread(FAR struct st25fl1_dev_s *priv, FAR uint8_t *buffer,
                           off_t address, size_t nbytes)
{
  fvdbg("address: %08lx nbytes: %d\n", (long)address, (int)nbytes);
#warning Missing Logic
}

/************************************************************************************
 * Name:  st25fl1_pagewrite
 ************************************************************************************/

static void st25fl1_pagewrite(struct st25fl1_dev_s *priv, FAR const uint8_t *buffer,
                          off_t address, size_t nbytes)
{
  fvdbg("address: %08lx nwords: %d\n", (long)address, (int)nbytes);
#warning Missing Logic
}

/************************************************************************************
 * Name: st25fl1_cacheflush
 ************************************************************************************/

#ifdef CONFIG_ST25FL1_SECTOR512
static void st25fl1_cacheflush(struct st25fl1_dev_s *priv)
{
  /* If the cached is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */
#warning Missing Logic

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/************************************************************************************
 * Name: st25fl1_cacheread
 ************************************************************************************/

#ifdef CONFIG_ST25FL1_SECTOR512
static FAR uint8_t *st25fl1_cacheread(struct st25fl1_dev_s *priv, off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;

  /* Convert from the 512 byte sector to the erase sector size of the device.  For
   * exmample, if the actual erase sector size if 4Kb (1 << 12), then we first
   * shift to the right by 3 to get the sector number in 4096 increments.
   */

  shift    = ST25FL1_SECTOR_SHIFT - ST25FL1_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  fvdbg("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      st25fl1_cacheflush(priv);

      /* Read the erase block into the cache */

      st25fl1_byteread(priv, priv->sector, (esectno << ST25FL1_SECTOR_SHIFT), ST25FL1_SECTOR_SIZE);

      /* Mark the sector as cached */

      priv->esectno = esectno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the argument */

  index = sector & ((1 << shift) - 1);

  /* Return the address in the cache that holds this sector */

  return &priv->sector[index << ST25FL1_SECTOR512_SHIFT];
}
#endif

/************************************************************************************
 * Name: st25fl1_cacheerase
 ************************************************************************************/

#ifdef CONFIG_ST25FL1_SECTOR512
static void st25fl1_cacheerase(struct st25fl1_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is in
   * the cache.
   */

  dest = st25fl1_cacheread(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (ST25FL1_SECTOR_SHIFT - ST25FL1_SECTOR512_SHIFT);
      fvdbg("sector: %ld esectno: %d\n", sector, esectno);

      st25fl1_sectorerase(priv, esectno);
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as dirty
   * (but don't update the FLASH yet.  The caller will do that at a more optimal
   * time).
   */

  memset(dest, ST25FL1_ERASED_STATE, ST25FL1_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/************************************************************************************
 * Name: st25fl1_cachewrite
 ************************************************************************************/

#ifdef CONFIG_ST25FL1_SECTOR512
static void st25fl1_cachewrite(FAR struct st25fl1_dev_s *priv, FAR const uint8_t *buffer,
                            off_t sector, size_t nsectors)
{
  FAR uint8_t *dest;

  for (; nsectors > 0; nsectors--)
    {
      /* First, make sure that the erase block containing 512 byte sector is in
       * memory.
       */

      dest = st25fl1_cacheread(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase sector
       * is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >> (ST25FL1_SECTOR_SHIFT - ST25FL1_SECTOR512_SHIFT);
          fvdbg("sector: %ld esectno: %d\n", sector, esectno);

          st25fl1_sectorerase(priv, esectno);
          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, ST25FL1_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += ST25FL1_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  st25fl1_cacheflush(priv);
}
#endif

/************************************************************************************
 * Name: st25fl1_erase
 ************************************************************************************/

static int st25fl1_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct st25fl1_dev_s *priv = (FAR struct st25fl1_dev_s *)dev;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
#warning Missing Logic

}

/************************************************************************************
 * Name: st25fl1_bread
 ************************************************************************************/

static ssize_t st25fl1_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR uint8_t *buffer)
{
  ssize_t nbytes;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

#ifdef CONFIG_ST25FL1_SECTOR512
  nbytes = st25fl1_read(dev, startblock << ST25FL1_SECTOR512_SHIFT, nblocks << ST25FL1_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= ST25FL1_SECTOR512_SHIFT;
    }
#else
  nbytes = st25fl1_read(dev, startblock << ST25FL1_SECTOR_SHIFT, nblocks << ST25FL1_SECTOR_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= ST25FL1_SECTOR_SHIFT;
    }
#endif

  return nbytes;
}

/************************************************************************************
 * Name: st25fl1_bwrite
 ************************************************************************************/

static ssize_t st25fl1_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                            FAR const uint8_t *buffer)
{
  FAR struct st25fl1_dev_s *priv = (FAR struct st25fl1_dev_s *)dev;

  fvdbg("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  st25fl1_lock(priv->qspi);

#if defined(CONFIG_ST25FL1_SECTOR512)
  st25fl1_cachewrite(priv, buffer, startblock, nblocks);
#else
  st25fl1_pagewrite(priv, buffer, startblock << ST25FL1_SECTOR_SHIFT,
                  nblocks << ST25FL1_SECTOR_SHIFT);
#endif
  st25fl1_unlock(priv->qspi);

  return nblocks;
}

/************************************************************************************
 * Name: st25fl1_read
 ************************************************************************************/

static ssize_t st25fl1_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR uint8_t *buffer)
{
  FAR struct st25fl1_dev_s *priv = (FAR struct st25fl1_dev_s *)dev;

  fvdbg("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  st25fl1_lock(priv->qspi);
  st25fl1_byteread(priv, buffer, offset, nbytes);
  st25fl1_unlock(priv->qspi);

  fvdbg("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/************************************************************************************
 * Name: st25fl1_ioctl
 ************************************************************************************/

static int st25fl1_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct st25fl1_dev_s *priv = (FAR struct st25fl1_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  fvdbg("cmd: %d \n", cmd);

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
               */

#ifdef CONFIG_ST25FL1_SECTOR512
              geo->blocksize    = (1 << ST25FL1_SECTOR512_SHIFT);
              geo->erasesize    = (1 << ST25FL1_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors << (ST25FL1_SECTOR_SHIFT - ST25FL1_SECTOR512_SHIFT);
#else
              geo->blocksize    = ST25FL1_SECTOR_SIZE;
              geo->erasesize    = ST25FL1_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
#endif
              ret               = OK;

              fvdbg("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
            /* Erase the entire device */

            st25fl1_lock(priv->qspi);
            ret = st25fl1_chiperase(priv);
            st25fl1_unlock(priv->qspi);
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  fvdbg("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: st25fl1_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *st25fl1_initialize(FAR struct spi_dev_s *qspi)
{
  FAR struct st25fl1_dev_s *priv;
  int ret;

  fvdbg("qspi: %p\n", qspi);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH definition) and so would have
   * to be extended to handle multiple FLASH parts on the same QuadSPI bus.
   */

  priv = (FAR struct st25fl1_dev_s *)kmm_zalloc(sizeof(struct st25fl1_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = st25fl1_erase;
      priv->mtd.bread  = st25fl1_bread;
      priv->mtd.bwrite = st25fl1_bwrite;
      priv->mtd.read   = st25fl1_read;
      priv->mtd.ioctl  = st25fl1_ioctl;
      priv->qspi       = qspi;

      /* Identify the FLASH chip and get its capacity */

      ret = st25fl1_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          fdbg("Unrecognized\n");
          kmm_free(priv);
          priv = NULL;
        }
      else
        {
          /* Make sure that the FLASH is unprotected so that we can write into it */

          st25fl1_unprotect(priv);

#ifdef CONFIG_ST25FL1_SECTOR512        /* Simulate a 512 byte sector */
          /* Allocate a buffer for the erase block cache */

          priv->sector = (FAR uint8_t *)kmm_malloc(ST25FL1_SECTOR_SIZE);
          if (!priv->sector)
            {
              /* Allocation failed! Discard all of that work we just did and return NULL */

              fdbg("Allocation failed\n");
              kmm_free(priv);
              priv = NULL;
            }
#endif
        }
    }

#ifdef CONFIG_MTD_REGISTRATION
  /* Register the MTD with the procfs system if enabled */

  mtd_register(&priv->mtd, "st25fl1");
#endif

  /* Return the implementation-specific state structure as the MTD device */

  fvdbg("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
