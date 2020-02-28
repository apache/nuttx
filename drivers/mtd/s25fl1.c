/************************************************************************************
 * drivers/mtd/s25fl1.c
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
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* QuadSPI Mode.  Per data sheet, either Mode 0 or Mode 3 may be used. */

#ifndef CONFIG_S25FL1_QSPIMODE
#  define CONFIG_S25FL1_QSPIMODE QSPIDEV_MODE0
#endif

/* QuadSPI Frequency per data sheet::
 *
 * – Normal Read (Serial):
 *   50 MHz clock rate (-40°C to +85°C/105°C)
 *   45 MHz clock rate (-40°C to +125°C)
 * – Fast Read (Serial):
 *   108 MHz clock rate (-40°C to +85°C/105°C)
 *   97 MHz clock rate (-40°C to +125°C)
 * – Dual Read:
 *   108 MHz clock rate (-40°C to +85°C/105°C)
 *   97 MHz clock rate (-40°C to +125°C)
 * – Quad Read:
 *   108 MHz clock rate (-40°C to +85°C/105°C)
 *   97 MHz clock rate for S25FL164K (-40°C to +125°C)
 *
 * Table 5.8:
 * - Clock frequency for all SPI commands except for Read Data
 *   command (0x03) and Fast Read command (0x0b): 108 MHz
 * - Clock frequency for Read Data command (0x03): 50 MHz
 * - Clock frequency for all Fast Read commands SIO and MIO: 108 MHz
 *
 * In this implementation, only "Quad" reads are performed.
 */

#ifndef CONFIG_S25FL1_QSPI_FREQUENCY
#  define CONFIG_S25FL1_QSPI_FREQUENCY 108000000
#endif

/* S25FL1 Commands ******************************************************************/
/* Configuration, Status, Erase, Program Commands ***********************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define S25FL1_READ_STATUS1        0x05  /* Read status register 1:                 *
                                          *   0x05 | SR1                            */
#define S25FL1_READ_STATUS2        0x35  /* Read status register 2:                 *
                                          *   0x35 | SR2                            */
#define S25FL1_READ_STATUS3        0x33  /* Read status register 3:                 *
                                          *   0x33 | SR3                            */
#define S25FL1_WRITE_ENABLE        0x06  /* Write enable:                           *
                                          *   0x06                                  */
#define S25FL1_VWRITE_ENABLE       0x50  /* Write enable for volatile status:       *
                                          *   0x50                                  */
#define S25FL1_WRITE_DISABLE       0x04  /* Write disable command code:             *
                                          *   0x04                                  */
#define S25FL1_WRITE_STATUS        0x01  /* Write status register:                  *
                                          *   0x01 | SR1 | SR2 | SR3                */
#define S25FL1_WRAP_ENABLE         0x77  /* Set Burst with Wrap:                    *
                                          *   0x77 | xx | xx | xx | SR3             */
#define S25FL1_UNPROTECT_SECTOR    0x39  /* Set Block / Pointer Protection:         *
                                          *   0x39 | ADDR(MS) | ADDR(MID) | xx      */
#define S25FL1_PAGE_PROGRAM        0x02  /* Page Program:                           *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data                       */
#define S25FL1_SECTOR_ERASE        0x20  /* Sector Erase (4 kB)                     *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define S25FL1_BLOCK_ERASE         0xd8  /* Block Erase (64 kB):                    *
                                          *   0x02 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define S25FL1_CHIP_ERASE_1        0x60  /* Chip Erase 1:                           *
                                          *   0x60                                  */
#define S25FL1_CHIP_ERASE_2        0xc7  /* Chip Erase 2:                           *
                                          *   0xc7                                  */
#define S25FL1_ERASE_PROG_SUSPEND  0x75  /* Erase / Program Suspend:                *
                                          *   0x75                                  */
#define S25FL1_ERASE_PROG_RESUME   0x7a  /* Erase / Program Resume:                 *
                                          *   0x7a                                  */

/* Read Commands ********************************************************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define S25FL1_READ_DATA           0x03  /* Read Data:                              *
                                          *   0x03 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data...                    */
#define S25FL1_FAST_READ           0x0b  /* Fast Read:                              *
                                          *   0x0b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define S25FL1_FAST_READ_DUAL      0x3b  /* Fast Read Dual Output:                  *
                                          *   0x3b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define S25FL1_FAST_READ_QUAD      0x6b  /* Fast Read Dual Output:                  *
                                          *   0x6b | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define S25FL1_FAST_READ_DUALIO    0xbb  /* Fast Read Dual I/O:                     *
                                          *   0xbb | ADDR(MS) | ADDR(LS) | data...  */
#define S25FL1_FAST_READ_QUADIO    0xeb  /* Fast Read Quad I/O:                     *
                                          *   0xeb | ADDR | data...                 */
#define S25FL1_CONT_READ_RESET     0xff  /* Continuous Read Mode Reset:             *
                                          *   0xff | 0xff                           */

/* Reset Commands *******************************************************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define S25FL1_SOFT_RESET_ENABLE   0x66  /* Software Reset Enable:                  *
                                          *   0x66                                  */
#define S25FL1_SOFT_RESET          0x99  /* Software Reset:                         *
                                          *   0x99                                  */
                                         /* Continuous Read Mode Reset:             *
                                          *   0xff | 0xff                           */

/* ID/Security Commands *************************&***********************************/
/*      Command                    Value    Description:                            */
/*                                            Data sequence                         */
#define S25FL1_DEEP_PWRDOWN        0xb9  /* Deep Power-down:                        *
                                          *   0xb9                                  */
#define S25FL1_RELEASE_PWRDOWN     0xab  /* Release Power down / Device ID:         *
                                          *   0xab | dummy | dummy | dummy |        *
                                          *   DeviceID  */
#define S25FL1_MANUFACTURER        0x90  /* Manufacturer / Device ID:               *
                                          *   0x90 | dummy | dummy | 0x00 |         *
                                          *   Manufacturer | DeviceID               */
#define S25FL1_JEDEC_ID            0x9f  /* JEDEC ID:                               *
                                          *   0x9f | Manufacturer | MemoryType |    *
                                          *   Capacity                              */
#define S25FL1_READ_SFDP           0x5a  /* Read SFDP Register / Read Unique ID     *
                                          * Number:                                 *
                                          *   0x5a | 0x00 | 0x00 | ADDR | dummy |   *
                                          *   data...                               */
#define S25FL1_READ_SECURITY       0x48  /* Read Security Registers:                *
                                          *   0x48 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | dummy | data...            */
#define S25FL1_ERASE_SECURITY      0x44  /* Erase Security Registers:               *
                                          *   0x48 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS)                              */
#define S25FL1_PROgRAM_SECURITY    0x42  /* Program Security Registers:             *
                                          *   0x42 | ADDR(MS) | ADDR(MID) |         *
                                          *   ADDR(LS) | data...                    */

/* Flash Manufacturer JEDEC IDs */

#define S25FL1_JEDEC_ID_SPANSION   0x01
#define S25FL1_JEDEC_ID_ATMEL      0x1f
#define S25FL1_JEDEC_ID_ST         0x20
#define S25FL1_JEDEC_ID_SST        0xbf
#define S25FL1_JEDEC_ID_MACRONIX   0xc2
#define S25FL1_JEDEC_ID_WINBOND    0xef

/* S25FL1 JEDIC IDs */

#define S25FL1_JEDEC_DEVICE_TYPE   0x40  /* S25FL1 memory device type */
#define S25FL116K_JEDEC_CAPACITY   0x15  /* S25FL116K memory capacity */
#define S25FL132K_JEDEC_CAPACITY   0x16  /* S25FL132K memory capacity */
#define S25FL164K_JEDEC_CAPACITY   0x17  /* S25FL164K memory capacity */

/* S25FL1 Registers ****************************************************************/
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
#  define STATUS1_SEC_SECTOR       (1 << 6) /*   1 = BP2-BP0 protect 4-kB sectors   */
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
#define STATUS3_W56_SHIFT          (5)      /* Bits 5-6: Burst Wrap Length          */
#define STATUS3_W56_MASK           (3 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_8BYTE        (0 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_16BYTE       (1 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_32BYTE       (2 << STATUS3_W56_SHIFT)
#  define STATUS3_W56_63BYTE       (3 << STATUS3_W56_SHIFT)
                                            /* Bit 7: Reserved                      */

/* Chip Geometries ******************************************************************/
/* All members of the family support uniform 4K-byte sectors  */

#define S25FL116K_SECTOR_SIZE      (4*1024)
#define S25FL116K_SECTOR_SHIFT     (12)
#define S25FL116K_SECTOR_COUNT     (512)
#define S25FL116K_PAGE_SIZE        (256)
#define S25FL116K_PAGE_SHIFT       (8)

#define S25FL132K_SECTOR_SIZE      (4*1024)
#define S25FL132K_SECTOR_SHIFT     (12)
#define S25FL132K_SECTOR_COUNT     (1024)
#define S25FL132K_PAGE_SIZE        (256)
#define S25FL132K_PAGE_SHIFT       (8)

#define S25FL164K_SECTOR_SIZE      (4*1024)
#define S25FL164K_SECTOR_SHIFT     (12)
#define S25FL164K_SECTOR_COUNT     (2048)
#define S25FL164K_PAGE_SIZE        (256)
#define S25FL164K_PAGE_SHIFT       (8)

/* Cache flags **********************************************************************/

#define S25FL1_CACHE_VALID         (1 << 0)  /* 1=Cache has valid data */
#define S25FL1_CACHE_DIRTY         (1 << 1)  /* 1=Cache is dirty */
#define S25FL1_CACHE_ERASED        (1 << 2)  /* 1=Backing FLASH is erased */

#define IS_VALID(p)                ((((p)->flags) & S25FL1_CACHE_VALID) != 0)
#define IS_DIRTY(p)                ((((p)->flags) & S25FL1_CACHE_DIRTY) != 0)
#define IS_ERASED(p)               ((((p)->flags) & S25FL1_CACHE_ERASED) != 0)

#define SET_VALID(p)               do { (p)->flags |= S25FL1_CACHE_VALID; } while (0)
#define SET_DIRTY(p)               do { (p)->flags |= S25FL1_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)              do { (p)->flags |= S25FL1_CACHE_ERASED; } while (0)

#define CLR_VALID(p)               do { (p)->flags &= ~S25FL1_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)               do { (p)->flags &= ~S25FL1_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)              do { (p)->flags &= ~S25FL1_CACHE_ERASED; } while (0)

/* 512 byte sector support **********************************************************/

#define S25FL1_SECTOR512_SHIFT     9
#define S25FL1_SECTOR512_SIZE      (1 << 9)
#define S25FL1_ERASED_STATE        0xff

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct s25fl1_dev_s.
 */

struct s25fl1_dev_s
{
  struct mtd_dev_s       mtd;         /* MTD interface */
  FAR struct qspi_dev_s *qspi;        /* Saved QuadSPI interface instance */
  uint16_t               nsectors;    /* Number of erase sectors */
  uint8_t                sectorshift; /* Log2 of sector size */
  uint8_t                pageshift;   /* Log2 of page size */
  FAR uint8_t           *cmdbuf;      /* Allocated command buffer */
  FAR uint8_t           *readbuf;     /* Allocated status read buffer */

#ifdef CONFIG_S25FL1_SECTOR512
  uint8_t                flags;       /* Buffered sector flags */
  uint16_t               esectno;     /* Erase sector number in the cache */
  FAR uint8_t           *sector;      /* Allocated sector data */
#endif
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Locking */

static void s25fl1_lock(FAR struct qspi_dev_s *qspi);
static inline void s25fl1_unlock(FAR struct qspi_dev_s *qspi);

/* Low-level message helpers */

static int  s25fl1_command(FAR struct qspi_dev_s *qspi, uint8_t cmd);
static int  s25fl1_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              off_t addr, uint8_t addrlen);
static int  s25fl1_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              FAR void *buffer, size_t buflen);
static int  s25fl1_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
              FAR const void *buffer, size_t buflen);
static uint8_t sf25fl1_read_status1(FAR struct s25fl1_dev_s *priv);
static uint8_t sf25fl1_read_status2(FAR struct s25fl1_dev_s *priv);
static uint8_t sf25fl1_read_status3(FAR struct s25fl1_dev_s *priv);
static void s25fl1_write_enable(FAR struct s25fl1_dev_s *priv);
static void s25fl1_write_disable(FAR struct s25fl1_dev_s *priv);

static int  s25fl1_readid(FAR struct s25fl1_dev_s *priv);
static int  s25fl1_protect(FAR struct s25fl1_dev_s *priv,
              off_t startblock, size_t nblocks);
static int  s25fl1_unprotect(FAR struct s25fl1_dev_s *priv,
              off_t startblock, size_t nblocks);
static bool s25fl1_isprotected(FAR struct s25fl1_dev_s *priv,
              uint8_t status, off_t address);
static int  s25fl1_erase_sector(FAR struct s25fl1_dev_s *priv, off_t offset);
static int  s25fl1_erase_chip(FAR struct s25fl1_dev_s *priv);
static int  s25fl1_read_byte(FAR struct s25fl1_dev_s *priv, FAR uint8_t *buffer,
              off_t address, size_t nbytes);
static int  s25fl1_write_page(FAR struct s25fl1_dev_s *priv,
              FAR const uint8_t *buffer, off_t address, size_t nbytes);
#ifdef CONFIG_S25FL1_SECTOR512
static int  s25fl1_flush_cache(struct s25fl1_dev_s *priv);
static FAR uint8_t *s25fl1_read_cache(struct s25fl1_dev_s *priv, off_t sector);
static void s25fl1_erase_cache(struct s25fl1_dev_s *priv, off_t sector);
static int  s25fl1_write_cache(FAR struct s25fl1_dev_s *priv,
              FAR const uint8_t *buffer,  off_t sector, size_t nsectors);
#endif

/* MTD driver methods */

static int  s25fl1_erase(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks);
static ssize_t s25fl1_bread(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks, FAR uint8_t *buf);
static ssize_t s25fl1_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
              size_t nblocks, FAR const uint8_t *buf);
static ssize_t s25fl1_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
              FAR uint8_t *buffer);
static int  s25fl1_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: s25fl1_lock
 ************************************************************************************/

static void s25fl1_lock(FAR struct qspi_dev_s *qspi)
{
  /* On QuadSPI buses where there are multiple devices, it will be necessary to
   * lock QuadSPI to have exclusive access to the buses for a sequence of
   * transfers.  The bus should be locked before the chip is selected.
   *
   * This is a blocking call and will not return until we have exclusive access to
   * the QuadSPI bus.  We will retain that exclusive access until the bus is unlocked.
   */

  QSPI_LOCK(qspi, true);

  /* After locking the QuadSPI bus, the we also need call the setfrequency, setbits, and
   * setmode methods to make sure that the QuadSPI is properly configured for the device.
   * If the QuadSPI bus is being shared, then it may have been left in an incompatible
   * state.
   */

  QSPI_SETMODE(qspi, CONFIG_S25FL1_QSPIMODE);
  QSPI_SETBITS(qspi, 8);
  QSPI_SETFREQUENCY(qspi, CONFIG_S25FL1_QSPI_FREQUENCY);
}

/************************************************************************************
 * Name: s25fl1_unlock
 ************************************************************************************/

static inline void s25fl1_unlock(FAR struct qspi_dev_s *qspi)
{
  QSPI_LOCK(qspi, false);
}

/************************************************************************************
 * Name: s25fl1_command
 ************************************************************************************/

static int s25fl1_command(FAR struct qspi_dev_s *qspi, uint8_t cmd)
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
 * Name: s25fl1_command_address
 ************************************************************************************/

static int s25fl1_command_address(FAR struct qspi_dev_s *qspi, uint8_t cmd,
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
 * Name: s25fl1_command_read
 ************************************************************************************/

static int s25fl1_command_read(FAR struct qspi_dev_s *qspi, uint8_t cmd,
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
 * Name: s25fl1_command_write
 ************************************************************************************/

static int s25fl1_command_write(FAR struct qspi_dev_s *qspi, uint8_t cmd,
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
 * Name: sf25fl1_read_status1
 ************************************************************************************/

static uint8_t sf25fl1_read_status1(FAR struct s25fl1_dev_s *priv)
{
  DEBUGVERIFY(s25fl1_command_read(priv->qspi, S25FL1_READ_STATUS1,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/************************************************************************************
 * Name: sf25fl1_read_status2
 ************************************************************************************/

static uint8_t sf25fl1_read_status2(FAR struct s25fl1_dev_s *priv)
{
  DEBUGVERIFY(s25fl1_command_read(priv->qspi, S25FL1_READ_STATUS2,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/************************************************************************************
 * Name: sf25fl1_read_status3
 ************************************************************************************/

static uint8_t sf25fl1_read_status3(FAR struct s25fl1_dev_s *priv)
{
  DEBUGVERIFY(s25fl1_command_read(priv->qspi, S25FL1_READ_STATUS3,
                                  (FAR void *)&priv->readbuf[0], 1));
  return priv->readbuf[0];
}

/************************************************************************************
 * Name:  s25fl1_write_enable
 ************************************************************************************/

static void s25fl1_write_enable(FAR struct s25fl1_dev_s *priv)
{
  uint8_t status;

  do
    {
      s25fl1_command(priv->qspi, S25FL1_WRITE_ENABLE);
      status = sf25fl1_read_status1(priv);
    }
  while ((status & STATUS1_WEL_MASK) != STATUS1_WEL_ENABLED);
}

/************************************************************************************
 * Name:  s25fl1_write_disable
 ************************************************************************************/

static void s25fl1_write_disable(FAR struct s25fl1_dev_s *priv)
{
  uint8_t status;

  do
    {
      s25fl1_command(priv->qspi, S25FL1_WRITE_DISABLE);
      status = sf25fl1_read_status1(priv);
    }
  while ((status & STATUS1_WEL_MASK) != STATUS1_WEL_DISABLED);
}

/************************************************************************************
 * Name:  s25fl1_write_status
 ************************************************************************************/

static void s25fl1_write_status(FAR struct s25fl1_dev_s *priv)
{
  s25fl1_write_enable(priv);
  s25fl1_command_write(priv->qspi, S25FL1_WRITE_STATUS,
                       (FAR const void *)priv->cmdbuf, 3);
  s25fl1_write_disable(priv);
}

/************************************************************************************
 * Name: s25fl1_readid
 ************************************************************************************/

static inline int s25fl1_readid(struct s25fl1_dev_s *priv)
{
  /* Lock the QuadSPI bus and configure the bus. */

  s25fl1_lock(priv->qspi);

  /* Read the JEDEC ID */

  s25fl1_command_read(priv->qspi, S25FL1_JEDEC_ID, priv->cmdbuf, 3);

  /* Unlock the bus */

  s25fl1_unlock(priv->qspi);

  finfo("Manufacturer: %02x Device Type %02x, Capacity: %02x",
        priv->cmdbuf[0], priv->cmdbuf[1], priv->cmdbuf[2]);

  /* Check for a recognized memory device type */

  if (priv->cmdbuf[1] != S25FL1_JEDEC_DEVICE_TYPE)
    {
      ferr("ERROR: Unrecognized device type: %02x\n", priv->cmdbuf[1]);
      return -ENODEV;
    }

  /* Check for a supported capacity */

  switch (priv->cmdbuf[2])
    {
      case S25FL116K_JEDEC_CAPACITY:
        priv->sectorshift = S25FL116K_SECTOR_SHIFT;
        priv->pageshift   = S25FL116K_PAGE_SHIFT;
        priv->nsectors    = S25FL116K_SECTOR_COUNT;
        break;

      case S25FL132K_JEDEC_CAPACITY:
        priv->sectorshift = S25FL132K_SECTOR_SHIFT;
        priv->pageshift   = S25FL116K_PAGE_SHIFT;
        priv->nsectors    = S25FL132K_SECTOR_COUNT;
        break;

      case S25FL164K_JEDEC_CAPACITY:
        priv->sectorshift = S25FL164K_SECTOR_SHIFT;
        priv->pageshift   = S25FL116K_PAGE_SHIFT;
        priv->nsectors    = S25FL164K_SECTOR_COUNT;
        break;

      /* Support for this part is not implemented yet */

      default:
        ferr("ERROR: Unsupported memory capacity: %02x\n", priv->cmdbuf[2]);
        return -ENODEV;
    }

  return OK;
}

/************************************************************************************
 * Name: s25fl1_protect
 ************************************************************************************/

static int s25fl1_protect(FAR struct s25fl1_dev_s *priv,
                          off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = sf25fl1_read_status1(priv);
  priv->cmdbuf[1] = sf25fl1_read_status2(priv);
  priv->cmdbuf[2] = sf25fl1_read_status3(priv);

  if ((priv->cmdbuf[0] & STATUS1_BP_MASK) == STATUS1_BP_NONE)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Check if sector protection registers are locked */

  if ((priv->cmdbuf[0] & STATUS1_SRP0_MASK) == STATUS1_SRP0_LOCKED)
    {
      /* Yes.. unprotect section protection registers */

      priv->cmdbuf[0] &= ~STATUS1_SRP0_MASK;
      s25fl1_write_status(priv);
    }

  /* Set the protection mask to zero.
   * REVISIT:  This logic should really just set the BP bits as
   * necessary to protect the range of sectors.
   */

  priv->cmdbuf[0] |= STATUS1_BP_MASK;
  s25fl1_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = sf25fl1_read_status1(priv);
  if ((priv->cmdbuf[0] & STATUS1_BP_MASK) != STATUS1_BP_MASK)
    {
      return -EACCES;
    }

  return OK;
}

/************************************************************************************
 * Name: s25fl1_unprotect
 ************************************************************************************/

static int s25fl1_unprotect(FAR struct s25fl1_dev_s *priv,
                            off_t startblock, size_t nblocks)
{
  /* Get the status register value to check the current protection */

  priv->cmdbuf[0] = sf25fl1_read_status1(priv);
  priv->cmdbuf[1] = sf25fl1_read_status2(priv);
  priv->cmdbuf[2] = sf25fl1_read_status3(priv);

  if ((priv->cmdbuf[0] & STATUS1_BP_MASK) == STATUS1_BP_NONE &&
      (priv->cmdbuf[1] & STATUS2_CMP_MASK) == 0)
    {
      /* Protection already disabled */

      return 0;
    }

  /* Check if sector protection registers are locked */

  if ((priv->cmdbuf[0] & STATUS1_SRP0_MASK) == STATUS1_SRP0_LOCKED)
    {
      /* Yes.. unprotect section protection registers */

      priv->cmdbuf[0] &= ~STATUS1_SRP0_MASK;
      s25fl1_write_status(priv);
    }

  /* Set the protection mask to zero (and not complemented).
   * REVISIT:  This logic should really just re-write the BP bits as
   * necessary to unprotect the range of sectors.
   */

  priv->cmdbuf[0] &= ~STATUS1_BP_MASK;
  priv->cmdbuf[1] &= ~STATUS2_CMP_MASK;
  s25fl1_write_status(priv);

  /* Check the new status */

  priv->cmdbuf[0] = sf25fl1_read_status1(priv);
  if ((priv->cmdbuf[0] & (STATUS1_SRP0_MASK | STATUS1_BP_MASK)) != 0)
    {
      return -EACCES;
    }

  return OK;
}

/************************************************************************************
 * Name: s25fl1_isprotected
 ************************************************************************************/

static bool s25fl1_isprotected(FAR struct s25fl1_dev_s *priv, uint8_t status,
                               off_t address)
{
  off_t protstart;
  off_t protend;
  off_t protsize;
  unsigned int bp;

  /* What is protected? 64 Kb blocks?  Or 4Kb sectors? */

  if ((status & STATUS1_SEC_MASK) == STATUS1_SEC_BLOCK)
    {
      /* 64 Kb block */

      protsize = 0x00010000;
    }
  else
    {
      /* 4 Kb sector */

      protsize = 0x00001000;
    }

  /* The BP field is the essentially a multiplier on this protection size */

  bp = (status & STATUS1_BP_MASK) >> STATUS1_BP_SHIFT;
  switch (bp)
    {
      case 0:
        return false;

      case 1:
        break;

      case 6:
      case 7:
        return true;

       default:
        protsize <<= (protsize << (bp - 1));
        break;
    }

  /* The final protection range then depends on if the protection region is
   * configured top-down or bottom up  (assuming CMP=0).
   */

  if ((status & STATUS1_TB_MASK) != 0)
    {
      protstart = 0x00000000;
      protend   = protstart + protsize;
    }
  else
    {
      protend   = 0x00200000;
      protstart = protend - protsize;
    }

  return (address >= protstart && address < protend);
}

/************************************************************************************
 * Name:  s25fl1_erase_sector
 ************************************************************************************/

static int s25fl1_erase_sector(struct s25fl1_dev_s *priv, off_t sector)
{
  off_t address;
  uint8_t status;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Check that the flash is ready and unprotected */

  status = sf25fl1_read_status1(priv);
  if ((status & STATUS1_BUSY_MASK) != STATUS1_READY)
    {
      ferr("ERROR: Flash busy: %02x", status);
      return -EBUSY;
    }

  /* Get the address associated with the sector */

  address = (off_t)sector << priv->sectorshift;

  if ((status & STATUS1_BP_MASK) != 0 &&
      s25fl1_isprotected(priv, status, address))
    {
      ferr("ERROR: Flash protected: %02x", status);
      return -EACCES;
    }

  /* Send the sector erase command */

  s25fl1_write_enable(priv);
  s25fl1_command_address(priv->qspi, S25FL1_SECTOR_ERASE, address, 3);

  /* Wait for erasure to finish */

  while ((sf25fl1_read_status1(priv) & STATUS1_BUSY_MASK) != 0);
  return OK;
}

/************************************************************************************
 * Name:  s25fl1_erase_chip
 ************************************************************************************/

static int s25fl1_erase_chip(struct s25fl1_dev_s *priv)
{
  uint8_t status;

  /* Check if the FLASH is protected */

  status = sf25fl1_read_status1(priv);
  if ((status & STATUS1_BP_MASK) != 0)
    {
      ferr("ERROR: FLASH is Protected: %02x", status);
      return -EACCES;
    }

  /* Erase the whole chip */

  s25fl1_write_enable(priv);
  s25fl1_command(priv->qspi, S25FL1_CHIP_ERASE_2);

  /* Wait for the erasure to complete */

  status = sf25fl1_read_status1(priv);
  while ((status & STATUS1_BUSY_MASK) != 0)
    {
      nxsig_usleep(200*1000);
      status = sf25fl1_read_status1(priv);
    }

  return OK;
}

/************************************************************************************
 * Name: s25fl1_read_byte
 ************************************************************************************/

static int s25fl1_read_byte(FAR struct s25fl1_dev_s *priv, FAR uint8_t *buffer,
                            off_t address, size_t buflen)
{
  struct qspi_meminfo_s meminfo;

  finfo("address: %08lx nbytes: %d\n", (long)address, (int)buflen);

#ifdef CONFIG_S25FL1_SCRAMBLE
  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO | QSPIMEM_SCRAMBLE;
#else
  meminfo.flags   = QSPIMEM_READ | QSPIMEM_QUADIO;
#endif
  meminfo.addrlen = 3;
  meminfo.dummies = 6;
  meminfo.buflen  = buflen;
  meminfo.cmd     = S25FL1_FAST_READ_QUADIO;
  meminfo.addr    = address;
#ifdef CONFIG_S25FL1_SCRAMBLE
  meminfo.key     = CONFIG_S25FL1_SCRAMBLE_KEY;
#endif
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/************************************************************************************
 * Name:  s25fl1_write_page
 ************************************************************************************/

static int s25fl1_write_page(struct s25fl1_dev_s *priv, FAR const uint8_t *buffer,
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

#ifdef CONFIG_S25FL1_SCRAMBLE
  meminfo.flags   = QSPIMEM_WRITE | QSPIMEM_SCRAMBLE;
#else
  meminfo.flags   = QSPIMEM_WRITE;
#endif
  meminfo.cmd     = S25FL1_PAGE_PROGRAM;
  meminfo.addrlen = 3;
  meminfo.buflen  = pagesize;
#ifdef CONFIG_S25FL1_SCRAMBLE
  meminfo.key     = CONFIG_S25FL1_SCRAMBLE_KEY;
#endif
  meminfo.dummies = 0;

  /* Then write each page */

  for (i = 0; i < npages; i++)
    {
      /* Set up varying parts of the transfer description */

      meminfo.addr   = address;
      meminfo.buffer = (void *)buffer;

      /* Write one page */

      s25fl1_write_enable(priv);
      ret = QSPI_MEMORY(priv->qspi, &meminfo);
      s25fl1_write_disable(priv);

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
 * Name: s25fl1_flush_cache
 ************************************************************************************/

#ifdef CONFIG_S25FL1_SECTOR512
static int s25fl1_flush_cache(struct s25fl1_dev_s *priv)
{
  int ret = OK;

  /* If the cached is dirty (meaning that it no longer matches the old FLASH contents)
   * or was erased (with the cache containing the correct FLASH contents), then write
   * the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      off_t address;

      /* Convert the erase sector numuber into a FLASH address */

      address = (off_t)priv->esectno << priv->sectorshift;

      /* Write entire erase block to FLASH */

      ret = s25fl1_write_page(priv, priv->sector, address, 1 << priv->sectorshift);
      if (ret < 0)
        {
          ferr("ERROR: s25fl1_write_page failed: %d\n", ret);
        }

      /* The case is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }

  return ret;
}
#endif

/************************************************************************************
 * Name: s25fl1_read_cache
 ************************************************************************************/

#ifdef CONFIG_S25FL1_SECTOR512
static FAR uint8_t *s25fl1_read_cache(struct s25fl1_dev_s *priv, off_t sector)
{
  off_t esectno;
  int   shift;
  int   index;
  int   ret;

  /* Convert from the 512 byte sector to the erase sector size of the device.  For
   * exmample, if the actual erase sector size if 4Kb (1 << 12), then we first
   * shift to the right by 3 to get the sector number in 4096 increments.
   */

  shift    = priv->sectorshift - S25FL1_SECTOR512_SHIFT;
  esectno  = sector >> shift;
  finfo("sector: %ld esectno: %d shift=%d\n", sector, esectno, shift);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || esectno != priv->esectno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      ret = s25fl1_flush_cache(priv);
      if (ret < 0)
        {
          ferr("ERROR: s25fl1_flush_cache failed: %d\n", ret);
          return NULL;
        }

      /* Read the erase block into the cache */

      ret = s25fl1_read_byte(priv, priv->sector,
                             (esectno << priv->sectorshift),
                             (1 << priv->sectorshift));
      if (ret < 0)
        {
          ferr("ERROR: s25fl1_read_byte failed: %d\n", ret);
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

  return &priv->sector[index << S25FL1_SECTOR512_SHIFT];
}
#endif

/************************************************************************************
 * Name: s25fl1_erase_cache
 ************************************************************************************/

#ifdef CONFIG_S25FL1_SECTOR512
static void s25fl1_erase_cache(struct s25fl1_dev_s *priv, off_t sector)
{
  FAR uint8_t *dest;

  /* First, make sure that the erase block containing the 512 byte sector is in
   * the cache.
   */

  dest = s25fl1_read_cache(priv, sector);

  /* Erase the block containing this sector if it is not already erased.
   * The erased indicated will be cleared when the data from the erase sector
   * is read into the cache and set here when we erase the block.
   */

  if (!IS_ERASED(priv))
    {
      off_t esectno  = sector >> (priv->sectorshift - S25FL1_SECTOR512_SHIFT);
      finfo("sector: %ld esectno: %d\n", sector, esectno);

      DEBUGVERIFY(s25fl1_erase_sector(priv, esectno));
      SET_ERASED(priv);
    }

  /* Put the cached sector data into the erase state and mart the cache as dirty
   * (but don't update the FLASH yet.  The caller will do that at a more optimal
   * time).
   */

  memset(dest, S25FL1_ERASED_STATE, S25FL1_SECTOR512_SIZE);
  SET_DIRTY(priv);
}
#endif

/************************************************************************************
 * Name: s25fl1_write_cache
 ************************************************************************************/

#ifdef CONFIG_S25FL1_SECTOR512
static int s25fl1_write_cache(FAR struct s25fl1_dev_s *priv,
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

      dest = s25fl1_read_cache(priv, sector);

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase sector
       * is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          off_t esectno  = sector >> (priv->sectorshift - S25FL1_SECTOR512_SHIFT);
          finfo("sector: %ld esectno: %d\n", sector, esectno);

          ret = s25fl1_erase_sector(priv, esectno);
          if (ret < 0)
            {
              ferr("ERROR: s25fl1_erase_sector failed: %d\n", ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, S25FL1_SECTOR512_SIZE);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += S25FL1_SECTOR512_SIZE;
      sector++;
    }

  /* Flush the last erase block left in the cache */

  return s25fl1_flush_cache(priv);
}
#endif

/************************************************************************************
 * Name: s25fl1_erase
 ************************************************************************************/

static int s25fl1_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct s25fl1_dev_s *priv = (FAR struct s25fl1_dev_s *)dev;
  size_t blocksleft = nblocks;
#ifdef CONFIG_S25FL1_SECTOR512
  int ret;
#endif

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  s25fl1_lock(priv->qspi);

  while (blocksleft-- > 0)
    {
      /* Erase each sector */

#ifdef CONFIG_S25FL1_SECTOR512
      s25fl1_erase_cache(priv, startblock);
#else
      s25fl1_erase_sector(priv, startblock);
#endif
      startblock++;
    }

#ifdef CONFIG_S25FL1_SECTOR512
  /* Flush the last erase block left in the cache */

  ret = s25fl1_flush_cache(priv);
  if (ret < 0)
    {
      nblocks = ret;
    }
#endif

  s25fl1_unlock(priv->qspi);
  return (int)nblocks;
}

/************************************************************************************
 * Name: s25fl1_bread
 ************************************************************************************/

static ssize_t s25fl1_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buffer)
{
#ifndef CONFIG_S25FL1_SECTOR512
  FAR struct s25fl1_dev_s *priv = (FAR struct s25fl1_dev_s *)dev;
#endif
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented read */

#ifdef CONFIG_S25FL1_SECTOR512
  nbytes = s25fl1_read(dev, startblock << S25FL1_SECTOR512_SHIFT,
                       nblocks << S25FL1_SECTOR512_SHIFT, buffer);
  if (nbytes > 0)
    {
      nbytes >>= S25FL1_SECTOR512_SHIFT;
    }
#else
  nbytes = s25fl1_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }
#endif

  return nbytes;
}

/************************************************************************************
 * Name: s25fl1_bwrite
 ************************************************************************************/

static ssize_t s25fl1_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct s25fl1_dev_s *priv = (FAR struct s25fl1_dev_s *)dev;
  int ret = (int)nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock the QuadSPI bus and write all of the pages to FLASH */

  s25fl1_lock(priv->qspi);

#if defined(CONFIG_S25FL1_SECTOR512)
  ret = s25fl1_write_cache(priv, buffer, startblock, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: s25fl1_write_cache failed: %d\n", ret);
    }

#else
  ret = s25fl1_write_page(priv, buffer, startblock << priv->pageshift,
                          nblocks << priv->pageshift);
  if (ret < 0)
    {
      ferr("ERROR: s25fl1_write_page failed: %d\n", ret);
    }
#endif

  s25fl1_unlock(priv->qspi);

  return ret < 0 ? ret : nblocks;
}

/************************************************************************************
 * Name: s25fl1_read
 ************************************************************************************/

static ssize_t s25fl1_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                           FAR uint8_t *buffer)
{
  FAR struct s25fl1_dev_s *priv = (FAR struct s25fl1_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the QuadSPI bus and select this FLASH part */

  s25fl1_lock(priv->qspi);
  ret = s25fl1_read_byte(priv, buffer, offset, nbytes);
  s25fl1_unlock(priv->qspi);

  if (ret < 0)
    {
      ferr("ERROR: s25fl1_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/************************************************************************************
 * Name: s25fl1_ioctl
 ************************************************************************************/

static int s25fl1_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct s25fl1_dev_s *priv = (FAR struct s25fl1_dev_s *)dev;
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

#ifdef CONFIG_S25FL1_SECTOR512
              geo->blocksize    = (1 << S25FL1_SECTOR512_SHIFT);
              geo->erasesize    = (1 << S25FL1_SECTOR512_SHIFT);
              geo->neraseblocks = priv->nsectors << (priv->sectorshift - S25FL1_SECTOR512_SHIFT);
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

          s25fl1_lock(priv->qspi);
          ret = s25fl1_erase_chip(priv);
          s25fl1_unlock(priv->qspi);
        }
        break;

      case MTDIOC_PROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = s25fl1_protect(priv, prot->startblock, prot->nblocks);
        }
        break;

      case MTDIOC_UNPROTECT:
        {
          FAR const struct mtd_protect_s *prot =
            (FAR const struct mtd_protect_s *)((uintptr_t)arg);

          DEBUGASSERT(prot);
          ret = s25fl1_unprotect(priv, prot->startblock, prot->nblocks);
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
 * Name: s25fl1_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the QuadSPI-based ST24FL1
 *   FLASH part.
 *
 *   MTD devices are not registered in the file system, but are created as instances
 *   that can be bound to other functions (such as a block or character driver front
 *   end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *s25fl1_initialize(FAR struct qspi_dev_s *qspi, bool unprotect)
{
  FAR struct s25fl1_dev_s *priv;
  int ret;

  finfo("qspi: %p\n", qspi);
  DEBUGASSERT(qspi != NULL);

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same QuadSPI bus.
   */

  priv = (FAR struct s25fl1_dev_s *)kmm_zalloc(sizeof(struct s25fl1_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = s25fl1_erase;
      priv->mtd.bread  = s25fl1_bread;
      priv->mtd.bwrite = s25fl1_bwrite;
      priv->mtd.read   = s25fl1_read;
      priv->mtd.ioctl  = s25fl1_ioctl;
      priv->mtd.name   = "s25fl1";
      priv->qspi       = qspi;

      /* Allocate a 4-byte buffer to support DMA command data */

      priv->cmdbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 4);
      if (priv->cmdbuf == NULL)
        {
          ferr("ERROR Failed to allocate command buffer\n");
          goto errout_with_priv;
        }

      /* Allocate a one-byte buffer to support DMA status read data */

      priv->readbuf = (FAR uint8_t *)QSPI_ALLOC(qspi, 1);
      if (priv->readbuf == NULL)
        {
          ferr("ERROR Failed to allocate read buffer\n");
          goto errout_with_cmdbuf;
        }

      /* Identify the FLASH chip and get its capacity */

      ret = s25fl1_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          ferr("ERROR Unrecognized QSPI device\n");
          goto errout_with_readbuf;
        }

      /* Enable quad mode */

      priv->cmdbuf[0] = sf25fl1_read_status1(priv);
      priv->cmdbuf[1] = sf25fl1_read_status2(priv);
      priv->cmdbuf[2] = sf25fl1_read_status3(priv);

      while ((priv->cmdbuf[1] & STATUS2_QUAD_ENABLE_MASK) == 0)
        {
          priv->cmdbuf[1] |= STATUS2_QUAD_ENABLE;
          s25fl1_write_status(priv);
          priv->cmdbuf[1] = sf25fl1_read_status2(priv);
          nxsig_usleep(50*1000);
        }

      /* Unprotect FLASH sectors if so requested. */

      if (unprotect)
        {
          ret = s25fl1_unprotect(priv, 0, priv->nsectors - 1);
          if (ret < 0)
            {
              ferr("ERROR: Sector unprotect failed\n");
            }
        }

#ifdef CONFIG_S25FL1_SECTOR512  /* Simulate a 512 byte sector */
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
