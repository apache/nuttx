/****************************************************************************
 * drivers/mmcsd/mmcsd_spi.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/fs.h>
#include <nuttx/mmcsd.h>

#include "mmcsd_spi.h"
#include "mmcsd_csd.h"
#include "mmcsd_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_MMCSD_NSLOTS
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "CONFIG_MMCSD_NSLOTS not defined"
#  endif
#  define CONFIG_MMCSD_NSLOTS 1
#endif

#define MMCSD_IDMODE_CLOCK           (400000)

#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
#  define MMCSD_MODE 0666
#else
#  define MMCSD_MODE 0444
#endif

/* Slot struct info *********************************************************/
/* Slot status definitions */

#define MMCSD_SLOTSTATUS_NOTREADY    0x01 /* Card not initialized */
#define MMCSD_SLOTSTATUS_NODISK      0x02 /* No card in the slot */
#define MMCSD_SLOTSTATUS_WRPROTECT   0x04 /* Card is write protected */
#define MMCSD_SLOTSTATUS_MEDIACHGD   0x08 /* Media changed in slot */

/* Values in the MMC/SD command table ***************************************/
/* These define the expected arguments of the MMC/SD command */

#define MMCSD_CMDARG_NONE            0
#define MMCSD_CMDARG_BLKLEN          1
#define MMCSD_CMDARG_ADDRESS         2
#define MMCSD_CMDARG_DUMMY           3

/* These define the value returned by the MMC/SD command */

#define MMCSD_CMDRESP_R1             0
#define MMCSD_CMDRESP_R1B            1
#define MMCSD_CMDRESP_R2             2
#define MMCSD_CMDRESP_R3             3


/* Fudge factor for SD read timeout: ~100msec, Write Time out ~250ms.  Units
 * of Hz.
 */

#define SD_READACCESSHZ              7
#define SD_WRITEACCESSHZ             3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one card slot */

struct mmcsd_slot_s
{
  FAR struct spi_dev_s *spi; /* SPI port bound to this slot */
  sem_t  sem;            /* Assures mutually exclusive accesss to card and SPI */
  ubyte  state;          /* State of the slot (see MMCSD_SLOTSTATUS_* definitions) */
  ubyte  type;           /* Disk type */
  ubyte  csd[16];        /* Copy of card CSD */
  uint16 sectorsize;     /* Media block size (in bytes) */
  uint32 nsectors;       /* Number of blocks on the media */
  uint32 taccess;        /* Card access time */
  uint32 twrite;         /* Card write time */
};

struct mmcsd_cmdinfo_s
{
  ubyte  cmd;
  ubyte  arg;
  ubyte  resp;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc *********************************************************************/

static void   mmcsd_semtake(sem_t *sem);

/* Card SPI interface *******************************************************/

static int    mmcsd_waitready(FAR struct mmcsd_slot_s *slot);
static uint32 mmcsd_sendcmd(FAR struct mmcsd_slot_s *slot,
                const struct mmcsd_cmdinfo_s *cmd, uint32 arg);
static void   mmcsd_setblklen(FAR struct mmcsd_slot_s *slot, uint32 length);
static void   mmcsd_decodecsd(FAR struct mmcsd_slot_s *slot, ubyte *csd);
static void   mmcsd_checkwrprotect(FAR struct mmcsd_slot_s *slot, ubyte *csd);
static int    mmcsd_getcardinfo(FAR struct mmcsd_slot_s *slot, ubyte *buffer,
                const struct mmcsd_cmdinfo_s *cmd);

#define mmcsd_getcsd(slot, csd) mmcsd_getcardinfo(slot, csd, &g_cmd9);
#define mmcsd_getcid(slot, cid) mmcsd_getcardinfo(slot, cid, &g_cmd10);

/* Block driver interfaces **************************************************/

static int     mmcsd_open(FAR struct inode *inode);
static int     mmcsd_close(FAR struct inode *inode);
static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#endif
static int     mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry);

/* Initialization ***********************************************************/

static int    mmcsd_mediainitialize(FAR struct mmcsd_slot_s *slot);
static void   mmcsd_mediachanged(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver state *************************************************************/

/* These are the lock driver methods supported by this file */

static const struct block_operations g_bops =
{
  mmcsd_open,     /* open     */
  mmcsd_close,    /* close    */
  mmcsd_read,     /* read     */
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
  mmcsd_write,    /* write    */
#else
  NULL,        /* write    */
#endif
  mmcsd_geometry, /* geometry */
  NULL         /* ioctl    */
};

/* A slot structure allocated for each configured slot */

static struct mmcsd_slot_s g_mmcsdslot[CONFIG_MMCSD_NSLOTS];

/* Timing *******************************************************************/

/* We will use the TRAN_SPEED from the CSD to determine the maximum SPI
 * clocking (TRAN_SPEED defines the maximum transfer rate per bit per data
 * line).
 *
 * The CSD TRAN_SPEED is provided as a 3 bit rate unit (RU) and a 4 bit time
 * value (TU). We need the transfer frequency which is:  RU*TU bits/sec
 *
 * g_transpeedru holds RU/10 and g_transpeedtu holds TU*10 so that the
 * correct value is returned in the product
 */

static const uint32 g_transpeedru[8] =
{
     10000,   /*  0:  10 Kbit/sec / 10 */
    100000,   /*  1:  1 Mbit/sec / 10 */
   1000000,   /*  2:  10 Mbit/sec / 10 */
  10000000,   /*  3:  100 Mbit/sec / 10*/

  0, 0, 0, 0  /* 4-7: Reserved values */
};

static const uint32 g_transpeedtu[16] =
{
   0, 10, 12, 13, /*  0-3:  Reserved, 1.0, 1.1, 1.2, 1.3 */
  15, 20, 25, 30, /*  4-7:  1.5, 2.0, 2.5, 3.0 */
  35, 40, 45, 50, /*  8-11: 3.5, 4.0, 4.5, 5.0 */
  55, 60, 70, 80, /* 12-15: 5.5, 6.0, 7.0, 8.0 */
};

/* The TAAC defines the asynchronous part of the data access time.  The
 * read access time the sum of the TAAC and the NSAC.  These define the
 * time from the end bit of the read command to start bit of the data block.
 *
 * The TAAC consists of a 3-bit time unit (TU) and a 4-bit time value (TV).
 * The access we need time is then given by:
 *
 *   taccess = spifrequency / (TU*TV) + NAC
 *
 * g_taactu holds the (1 / TU / 100 ) and g_taactv holds (100 / TV) so
 * that taccess can be computed without division.
 */

static const uint32 g_taactu[8] =
{
  10000000, /* 0:   1 ns -> 1,000,000,000 Hz / 100 = 10,000,000 */
   1000000, /* 1:  10 ns ->   100,000,000 Hz / 100 =  1,000,000 */
    100000, /* 2: 100 ns ->    10,000,000 Hz / 100 =    100,000 */
     10000, /* 3:   1 us ->     1,000,000 Hz / 100 =     10,000 */
      1000, /* 4:  10 us ->       100,000 Hz / 100 =      1,000 */
       100, /* 5: 100 us ->        10,000 Hz / 100 =        100 */
        10, /* 6:   1 ms ->         1,000 Hz / 100 =         10 */
         1, /* 7:  10 ms ->           100 Hz / 100 =          1 */
};

static const uint32 g_taactv[] =
{
   0, 100,  83,  77, /*  0-3:  Reserved, 100/1.0, 100/1.2, 100/1.3 */
  67,  50,  40,  33, /*  4-7:   100/1.5, 100/2.0, 100/2.5, 100/3.0 */
  29,  25,  22,  20, /*  8-11:  100/3.5, 100/4.0, 100/4.5, 100/5.0 */
  18,  17,  14,  13  /* 12-15:  100/5.5, 100/6.0, 100/7.0, 100/8.0 */
};

/* Commands *****************************************************************/

static const struct mmcsd_cmdinfo_s g_cmd0   = {0x40, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd1   = {0x41, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd9   = {0x49, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd10  = {0x4a, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd12  = {0x4c, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd13  = {0x4d, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R2};
static const struct mmcsd_cmdinfo_s g_cmd16  = {0x50, MMCSD_CMDARG_BLKLEN,  MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd17  = {0x51, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd18  = {0x52, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd24  = {0x58, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd25  = {0x59, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd27  = {0x5b, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd28  = {0x5c, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1B};
static const struct mmcsd_cmdinfo_s g_cmd29  = {0x5d, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1B};
static const struct mmcsd_cmdinfo_s g_cmd30  = {0x5e, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd32  = {0x60, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd33  = {0x61, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd34  = {0x62, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd35  = {0x63, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd36  = {0x64, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd37  = {0x65, MMCSD_CMDARG_ADDRESS, MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd38  = {0x66, MMCSD_CMDARG_DUMMY,   MMCSD_CMDRESP_R1B};
static const struct mmcsd_cmdinfo_s g_cmd42  = {0x6a, MMCSD_CMDARG_DUMMY,   MMCSD_CMDRESP_R1B};
static const struct mmcsd_cmdinfo_s g_cmd55  = {0x77, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd56  = {0x78, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_cmd58  = {0x7a, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R3};
static const struct mmcsd_cmdinfo_s g_cmd59  = {0x7b, MMCSD_CMDARG_DUMMY,   MMCSD_CMDRESP_R1};
static const struct mmcsd_cmdinfo_s g_acmd41 = {0x69, MMCSD_CMDARG_NONE,    MMCSD_CMDRESP_R1};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_semtake
 ****************************************************************************/

static void mmcsd_semtake(sem_t *sem)
{
  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define mmcsd_semgive(sem) sem_post(sem)

/****************************************************************************
 * Name: mmcsd_waitready
 *
 * Description:
 *   Wait until the the card is no longer busy
 *
 ****************************************************************************/

static int mmcsd_waitready(FAR struct mmcsd_slot_s *slot)
{
  FAR struct spi_dev_s *spi = slot->spi;
  ubyte response;
  int i;

  /* Wait until the card is no longer busy */

  for (i = 0; i < slot->twrite; i++)
    {
      response = SPI_SNDBYTE(spi, 0xff);
      if (response == 0xff)
        {
          return OK;
        }
    }

  fdbg("Card still busy, last response: %02x\n", response);
  return -EBUSY;
}

/****************************************************************************
 * Name: mmcsd_sendcmd
 *
 * Description:
 *   Send a command to MMC
 *
 ****************************************************************************/

static uint32 mmcsd_sendcmd(FAR struct mmcsd_slot_s *slot,
                            const struct mmcsd_cmdinfo_s *cmd, uint32 arg)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32 result;
  ubyte response = 0xff;
  int i;

  /* Select SPI */

  SPI_SELECT(spi, SPIDEV_MMCSD, TRUE);

  /* Send command code */

  SPI_SNDBYTE(spi, cmd->cmd);

  /* Send command's arguments */

  if (cmd->arg == MMCSD_CMDARG_NONE)
    {
      SPI_SNDBYTE(spi, 0x00);
      SPI_SNDBYTE(spi, 0x00);
      SPI_SNDBYTE(spi, 0x00);
      SPI_SNDBYTE(spi, 0x00);
    }
  else
    {
      SPI_SNDBYTE(spi, arg >> 24);
      SPI_SNDBYTE(spi, arg >> 16);
      SPI_SNDBYTE(spi, arg >> 8);
      SPI_SNDBYTE(spi, arg);
    }

  /* Send CRC if needed.  The SPI interface is initialized in non-protected
   * mode.  However, the reset command (CMD0) is received by the card while it
   * is still in SD mode and, therefore, must have a valid CRC field.
   */

  if (cmd->cmd == 0x40)
    {
      SPI_SNDBYTE(spi, 0x95);
    }
  else
    {
      SPI_SNDBYTE(spi, 0xff);
    }

  /* Get the response to the command */

  for (i = 0; i < 9 && response == 0xff; i++)
    {
      response = SPI_SNDBYTE(spi, 0xff);
    }

  if (i == 0)
    {
      fdbg("Failed: i=%d response=%02x\n", i, response);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
      return (uint32)-1;
    }

  /* Interpret the response according to the command */

  switch (cmd->resp)
    {
    case MMCSD_CMDRESP_R1B:
      {
        uint32 busy = 0;
        for (i = 0; i < slot->twrite && busy != 0xff; i++)
          {
            busy = SPI_SNDBYTE(spi, 0xff);
          }
        fvdbg("Return R1B=%02x\n", response);
      }
      return (uint32)response;

    case MMCSD_CMDRESP_R1:
      {
        fvdbg("Return R1=%02x\n", response);
      }
      return (uint32)response;

    case MMCSD_CMDRESP_R2:
      {
        result  = ((uint32) response << 8) & 0x0000ff00;
        result |= SPI_SNDBYTE(spi, 0xff) & 0xff;
        fvdbg("Return R2=%04x\n", result);
      }
      return result;

    case MMCSD_CMDRESP_R3:
    default:
      {
        result  = ((uint32) response << 24) & 0xff000000;
        result |= ((uint32) SPI_SNDBYTE(spi, 0xff) << 16) & 0x00ff0000;
        result |= ((uint32) SPI_SNDBYTE(spi, 0xff) << 8) & 0x0000ff00;
        result |= SPI_SNDBYTE(spi, 0xff) & 0xff;
        fvdbg("Return R3=%08x\n", result);
      }
      return result;
    }
}

/****************************************************************************
 * Name: mmcsd_setblklen
 *
 * Description:
 *   Set block length
 *
 ****************************************************************************/

static void mmcsd_setblklen(FAR struct mmcsd_slot_s *slot, uint32 length)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32 result;

  result = mmcsd_sendcmd(slot, &g_cmd16, length);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
}

/****************************************************************************
 * Name: mmcsd_decodecsd
 *
 * Description: 
 *
 ****************************************************************************/

static void mmcsd_decodecsd(FAR struct mmcsd_slot_s *slot, ubyte *csd)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32 frequency;

  /* Calculate SPI max clock */

  frequency =
    g_transpeedru[MMCSD_CSD_TRANSPEED_TIMEVALUE(csd)] *
    g_transpeedtu[MMCSD_CSD_TRANSPEED_TRANSFERRATEUNIT(csd)];

  if (frequency > 20000000)
    {
      frequency = 20000000;
    }

  /* Set the SPI frequency to that value */

  frequency = SPI_SETFREQUENCY(spi, frequency);

  /* Now determine the delay to */

  if (slot->type == MMCSD_CARDTYPE_MMC)
    {
      slot->taccess =
        g_taactu[MMCSD_CSD_TAAC_TIMEUNIT(csd)] *
        g_taactv[MMCSD_CSD_TAAC_TIMEVALUE(csd)];

      slot->taccess  = frequency / slot->taccess;
      slot->taccess += 1 << (MMCSD_CSD_NSAC(csd) + 4);
      slot->taccess *= 10;
      slot->twrite   = slot->taccess * MMCSD_CSD_R2WFACTOR(csd);
    }
  else
    {
      slot->taccess  = frequency / SD_READACCESSHZ;
      slot->twrite   = frequency / SD_WRITEACCESSHZ;
    }

  fvdbg("Frequency:         %d\n", frequency);
  fvdbg("Read access time:  %d\n", slot->taccess);
  fvdbg("Write access time: %d\n", slot->twrite);

  /* Get the physical geometry of the card: sector size and number of
   * sectors. The card's total capacity is computed from
   *
   *   capacity = BLOCKNR * BLOCK_LEN
   *   BLOCKNR = (C_SIZE+1)*MULT
   *   MULT = 2**(C_SIZE_MULT+2)         (C_SIZE_MULT < 8)
   *   BLOCK_LEN = 2**READD_BL_LEN       (READ_BL_LEN < 12)
   *
   * Or
   *
   *   capacity = ((C_SIZE+1) << (READD_BL_LEN + C_SIZE_MULT + 2))
   *
   * In units of the sector size (1 << READ_BL_LEN), then simplifies to
   *
   *   nsectors = ((C_SIZE+1) << (C_SIZE_MULT + 2))
   */

  if (MMCSD_CSD_CSDSTRUCT(csd) == 1)
    {
      /* SDC ver 2.00 */
      /* Note: On SD card WRITE_BL_LEN is always the same as READ_BL_LEN */

      slot->sectorsize = 1 << SD20_CSD_READBLLEN(csd);
      slot->nsectors   = (SD20_CSD_CSIZE(csd) + 1) << (SD20_CSD_CSIZEMULT(csd) + 2);
    }
  else
    {
      /* MMC or SD ver 1.xx */
      /* Note: On SD card WRITE_BL_LEN is always the same as READ_BL_LEN */

      slot->sectorsize = 1 << MMCSD_CSD_READBLLEN(csd);
      slot->nsectors = (MMCSD_CSD_CSIZE(csd) + 1) << (MMCSD_CSD_CSIZEMULT(csd) + 2);
    }

  fvdbg("Sector size:       %d\n", slot->sectorsize);
  fvdbg("Number of sectors: %d\n", slot->nsectors);
}

/****************************************************************************
 * Name: mmcsd_checkwrprotect
 *
 * Description: 
 *
 ****************************************************************************/

static void mmcsd_checkwrprotect(FAR struct mmcsd_slot_s *slot, ubyte *csd)
{
  FAR struct spi_dev_s *spi = slot->spi;

  /* Check if (1) the slot is reporting that reporting that write protection
   * is set, (2) the card reports permanent write protect, or (2) the card
   * reports temporary write protect.
   */

  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_WRPROTECTED) != 0 ||
      MMCSD_CSD_PERMWRITEPROTECT(csd) ||
      MMCSD_CSD_TMPWRITEPROTECT(csd))
    {
      slot->state |= MMCSD_SLOTSTATUS_WRPROTECT;
    }
  else
    {
      slot->state &= ~MMCSD_SLOTSTATUS_WRPROTECT;
    }
}

/****************************************************************************
 * Name: mmcsd_getcardinfo
 *
 * Description:
 *   Read CSD or CID  registers
 *
 ****************************************************************************/

static int mmcsd_getcardinfo(FAR struct mmcsd_slot_s *slot, ubyte *buffer,
                             const struct mmcsd_cmdinfo_s *cmd)
{
  FAR struct spi_dev_s *spi = slot->spi;
  uint32 result;
  ubyte response;
  int i;

  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  SPI_SNDBYTE(spi, 0xff);

  /* Send the CMD9 or CMD10 */

  result = mmcsd_sendcmd(slot, cmd, 0);
  if (result != MMCSD_SPIR1_OK)
    {
      fdbg("CMD9/10 failed: R1=%02x\n", result);
      goto errout_with_eio;
    }

  /* Try up to 8 times to find the start of block (or until an error occurs) */

  for (i = 0; i < 8; i++)
    {
      response = SPI_SNDBYTE(spi, 0xff);
      fvdbg("%d. SPI sndbyte returned %02x\n", i, response);

      /* If a read operation fails and the card cannot provide the requested
       * data, it will send a data error token instead.  The 4 least
       * significant bits are the same as those in the R2 response.
       */

      if (response != 0 && (response & MMCSD_SPIDET_UPPER) == 0)
        {
          fdbg("%d. Data transfer error: %02x\n", i, response);
          goto errout_with_eio;
        }
      else if (response == MMCSD_SPIDT_STARTBLKSNGL)
        {
          for (i = 0; i < 16; ++i)
            {
              *buffer++ = SPI_SNDBYTE(spi, 0xff);
            }

          /* CRC receive */

          (void)SPI_SNDBYTE(spi, 0xff);
          (void)SPI_SNDBYTE(spi, 0xff);
          SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
          return OK;
        }
    }

errout_with_eio:
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  return -EIO;
}

/****************************************************************************
 * Block Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mmcsd_open(FAR struct inode *inode)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  int ret;

  fvdbg("Entry\n");

#ifdef CONFIG_DEBUG
  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Select the slave */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

  /* Verify that the MMC/SD card is alive and ready for business */

  ret = mmcsd_waitready(slot);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  mmcsd_semgive(&slot->sem);
  return ret;
}

/****************************************************************************
 * Name: mmcsd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int mmcsd_close(FAR struct inode *inode)
{
  fvdbg("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: mmcsd_read
 *
 * Description:  Read the specified numer of sectors
 *
 ****************************************************************************/

static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  size_t nbytes;
  off_t  offset;
  ubyte response;
  int i;

  fvdbg("start_sector=%d nsectors=%d\n", start_sector, nsectors);

#ifdef CONFIG_DEBUG
  if (!buffer)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Verify that card is availabled */

  if (slot->state & MMCSD_SLOTSTATUS_NOTREADY)
    {
      fdbg("Slot not ready\n");
      return -ENODEV;
    }

  /* Do nothing on zero-length transfer */

  if (nsectors < 1)
    {
      return 0;
    }

  /* Convert sector and nsectors to nbytes and byte offset */

  nbytes = nsectors * slot->sectorsize;
  offset = start_sector * slot->sectorsize;
  fvdbg("nbytes=%d offset=%d\n", nbytes, offset);

  /* Select the slave and synchronize */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  (void)SPI_SNDBYTE(spi, 0xff);

  /* Send CMD17: Reads a block of the size selected by the SET_BLOCKLEN
   * command and verify that good R1 status is returned
   */

  response = mmcsd_sendcmd(slot, &g_cmd17, offset);
  if (response != MMCSD_SPIR1_OK)
    {
      fdbg("CMD17 failed: R1=%02x\n", response);
      goto errout_with_eio;
    }

  /* Loop only for the computed timeout */

  for (i = 0; i < slot->taccess; i++)
    {
      /* Synchronize */

      response = SPI_SNDBYTE(spi, 0xff);
      fvdbg("(%d) SPI sndbyte returned %02x\n", i, response);

      /* If a read operation fails and the card cannot provide the requested
       * data, it will send a data error token instead.  The 4 least
       * significant bits are the same as those in the R2 response.
       */

      if (response != 0 && (response & MMCSD_SPIDET_UPPER) == 0)
        {
          fdbg("(%d) Data transfer error: %02x\n", i, response);
          goto errout_with_eio;
        }
      else if (response == MMCSD_SPIDT_STARTBLKSNGL)
        {
          /* Receive the block of data */

          SPI_RECVBLOCK(spi, buffer, nbytes);

          /* Receive and ignore the two CRC bytes */

          (void)SPI_SNDBYTE(spi, 0xff);
          (void)SPI_SNDBYTE(spi, 0xff);

          /* On success, return the number of sectors transfer */

          SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
          mmcsd_semgive(&slot->sem);

          fvdbg("(%d) Read %d bytes:\n", i, nbytes);
          mmcsd_dumpbuffer(buffer, nbytes);
          return nsectors;
        }
    }

errout_with_eio:
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  mmcsd_semgive(&slot->sem);
  return -EIO;
}

/****************************************************************************
 * Name: mmcsd_write
 *
 * Description:
 *   Write the specified number of sectors
 *
 ****************************************************************************/

#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer,
                        size_t start_sector, unsigned int nsectors)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  size_t nbytes;
  off_t  offset;
  ubyte response;
  int ret;

  fvdbg("start_sector=%d nsectors=%d\n", start_sector, nsectors);

#ifdef CONFIG_DEBUG
  if (!buffer)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Verify that card is availabled */

  if (slot->state & MMCSD_SLOTSTATUS_NOTREADY)
    {
      fdbg("Slot not ready\n");
      return -ENODEV;
    }

  /* Verify that the card is write enabled */

  if (slot->state & MMCSD_SLOTSTATUS_WRPROTECT)
    {
      fdbg("Not write enabled\n");
      return -EACCES;
    }

  /* Do nothing on zero-length transfer */

  if (nsectors < 1)
    {
      return 0;
    }

  /* Convert sector and nsectors to nbytes and byte offset */

  nbytes = nsectors * slot->sectorsize;
  offset = start_sector * slot->sectorsize;
  fvdbg("Writing %d bytes to offset %d:\n", nbytes, offset);
  mmcsd_dumpbuffer(buffer, nbytes);

  /* Select the slave and synchronize */

  mmcsd_semtake(&slot->sem);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  (void)SPI_SNDBYTE(spi, 0xff);

  /* Send CMD24 (WRITE_BLOCK) and verify that good R1 status is returned */

  response = mmcsd_sendcmd(slot, &g_cmd24, offset);
  if (response != MMCSD_SPIR1_OK)
    {
      fdbg("CMD24 failed: R1=%02x\n", response);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
      ret = -EIO;
      goto errout_with_sem;
    }

  /* Start the block transfer:
   * 1. 0xff (sync)
   * 2. 0xfe (start of block)
   * 3. Followed by the block of data
   */

  (void)SPI_SNDBYTE(spi, 0xff);
  (void)SPI_SNDBYTE(spi, MMCSD_SPIDT_STARTBLKSNGL);
  (void)SPI_SNDBLOCK(spi, buffer, nbytes);

  /* Add the bogus CRC.  By default, the SPI interface is initialized in
   * non-protected mode.  However, we still have to send bogus CRC values
   */

  (void)SPI_SNDBYTE(spi, 0xff);
  (void)SPI_SNDBYTE(spi, 0xff);

  /* Now get the data response */

  response = SPI_SNDBYTE(spi, 0xff);
  if ((response & MMCSD_SPIDR_MASK) != MMCSD_SPIDR_ACCEPTED)
    {
      fdbg("Bad data response: %02x\n", response);
      ret = -EIO;
      goto errout_with_sem;
    }

  /* Wait until the card is no longer busy */

  ret = mmcsd_waitready(slot);
  SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
  mmcsd_semgive(&slot->sem);

  /* Verify that the card successfully became non-busy */

  if (ret < 0)
    {
      return ret;
    }

  /* The success return value is the number of sectors written */

  return nsectors;

errout_with_sem:
  mmcsd_semgive(&slot->sem);
  return ret;
}
#endif

/****************************************************************************
 * Name: mmcsd_geometry
 *
 * Description:
 *   Return device geometry
 *
 ****************************************************************************/

static int mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct mmcsd_slot_s *slot;
  FAR struct spi_dev_s *spi;
  ubyte csd[16];
  int ret;

#ifdef CONFIG_DEBUG
  if (!geometry)
    {
      fdbg("Invalid parameters\n");
      return -EINVAL;
    }

  if (!inode || !inode->i_private)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Extract our private data from the inode structure */

  slot = (FAR struct mmcsd_slot_s *)inode->i_private;
  spi  = slot->spi;

#ifdef CONFIG_DEBUG
  if (!spi)
    {
      fdbg("Internal confusion\n");
      return -EIO;
    }
#endif

  /* Re-sample the CSD */

  mmcsd_semtake(&slot->sem);
  ret = mmcsd_getcsd(slot, csd);
  if (ret < 0)
    {
      mmcsd_semgive(&slot->sem);
      fdbg("mmcsd_getcsd returned %d\n", ret);
      return ret;
    }

  /* Check for changes related to write protection */

  mmcsd_checkwrprotect(slot, csd);

  /* Then return the card geometry */

  geometry->geo_available =
    ((slot->state & (MMCSD_SLOTSTATUS_NOTREADY|MMCSD_SLOTSTATUS_NODISK)) == 0);
  geometry->geo_mediachanged =
    ((slot->state & MMCSD_SLOTSTATUS_MEDIACHGD) != 0);
#if defined(CONFIG_FS_WRITABLE) && !defined(CONFIG_MMCSD_READONLY)
  geometry->geo_writeenabled =
    ((slot->state & MMCSD_SLOTSTATUS_WRPROTECT) == 0);
#else
  geometry->geo_writeenabled = FALSE;
#endif
  geometry->geo_nsectors   = slot->nsectors;
  geometry->geo_sectorsize = slot->sectorsize;

  /* After reporting mediachanged, clear the indication so that it is not
   * reported again.
   */

  slot->state &= ~MMCSD_SLOTSTATUS_MEDIACHGD;
  mmcsd_semgive(&slot->sem);

  fvdbg("geo_available:     %d\n", geometry->geo_available);
  fvdbg("geo_mediachanged:  %d\n", geometry->geo_mediachanged);
  fvdbg("geo_writeenabled:  %d\n", geometry->geo_writeenabled);
  fvdbg("geo_nsectors:      %d\n", geometry->geo_nsectors);
  fvdbg("geo_sectorsize:    %d\n", geometry->geo_sectorsize);

  return OK;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_mediainitialize
 *
 * Description:
 *   Detect media and initialize
 *
 ****************************************************************************/

static int mmcsd_mediainitialize(FAR struct mmcsd_slot_s *slot)
{
  FAR struct spi_dev_s *spi = slot->spi;
  ubyte csd[16];
  uint32 result = MMCSD_SPIR1_IDLESTATE;
  int i, j;

  /* Assume that the card is not ready (we'll clear this on successful car
   * initialization.
   */

  slot->state |= MMCSD_SLOTSTATUS_NOTREADY;


  /* Check if there is a card present in the slot.  This is normally a matter is
   * of GPIO sensing and does not really involve SPI, but by putting this
   * functionality in the SPI interface, we encapuslate the SPI MMC/SD
   * interface
   */

  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_PRESENT) == 0)
    {
      fdbg("No card present\n");
      slot->state |= MMCSD_SLOTSTATUS_NODISK;
      return -ENODEV;
    }

  /* Clock Freq. Identification Mode < 400kHz */

  SPI_SETFREQUENCY(spi, MMCSD_IDMODE_CLOCK);

  /* Set the maximum access time out */

  slot->taccess = MMCSD_IDMODE_CLOCK / SD_READACCESSHZ;

  /* The SD card wakes up in SD mode. It will enter SPI mode if the chip select signal is
   * asserted (negative) during the reception of the reset command (CMD0) and the card is in
   * IDLE state.
   */

  /* After power up at least 74 clock cycles are required prior to starting bus communication */

  fvdbg("Send CMD0\n");
  for (i = 0; i < 2; i++)
    {
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

      for (j = 10; j; j--)
        {
          SPI_SNDBYTE(spi, 0xff);
        }

      /* Send CMD0 (GO_TO_IDLE) to put MMC/SD in IDLE/SPI mode */

      result = mmcsd_sendcmd(slot, &g_cmd0, 0);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

      /* Return from CMD0 is R1 which should now show IDLE STATE */

      if (result == MMCSD_SPIR1_IDLESTATE)
        {
          fvdbg("Card is in IDLE state\n");
          break;
        }
    }

  /* Verify that we exit the above loop with the card reporting IDLE state */

  if (result != MMCSD_SPIR1_IDLESTATE)
    {
      fdbg("Send CMD0 failed: R1=%02x\n", result);
      return -EIO;
    }

  /* Determinate Card type SD or MMC */

  slot->type = MMCSD_CARDTYPE_MMC;

  for (i = 100; i; --i)
    {
      fvdbg("%d. Send CMD55\n", i);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
      SPI_SNDBYTE(spi, 0xff);
      result = mmcsd_sendcmd(slot, &g_cmd55, 0);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

      fvdbg("%d. Send ACMD41\n", i);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
      SPI_SNDBYTE(spi, 0xff);
      result = mmcsd_sendcmd(slot, &g_acmd41, 0);
      SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

      /* If this is an MMC card, it will response with ILLEGAL COMMAND */

      if (result & MMCSD_SPIR1_ILLEGALCMD)
        {
          /* MMC card may be CMD1 for MMC Init sequence will be complete within 
           * 500ms */

          for (i = 100; i; --i)
            {
              fvdbg("%d. Send CMD1\n", i);
              SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);
              SPI_SNDBYTE(spi, 0xff);
              result = mmcsd_sendcmd(slot, &g_cmd1, 0);
              SPI_SELECT(spi, SPIDEV_MMCSD, FALSE);

              if (result == MMCSD_SPIR1_OK)
                {
                  fvdbg("%d. Identified MMC card\n", i);
                  slot->type = MMCSD_CARDTYPE_MMC;
                  break;
                }
              up_mdelay(50);
            }
          break;
        }
      else if (result == MMCSD_SPIR1_OK)
        {
          fvdbg("%d. Identified SD card\n", i);
          slot->type = MMCSD_CARDTYPE_SD;
          break;
        }

      up_mdelay(50);
    }

  if (i == 0)
    {
      fdbg("Retry exhausted\n");
      return -EIO;
    }

  /* Read CSD.  CSD must always be valid */

  fvdbg("Get CSD\n");
  result = mmcsd_getcsd(slot, csd);
  if (result != OK)
    {
      fdbg("mmcsd_getcsd(CMD9) failed: %d\n", result);
      return -EIO;
    }

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
  if (slot->type == MMCSD_CARDTYPE_MMC)
    {
      fdbg("Found MMC card\n");
    }
  else if (MMCSD_CSD_CSDSTRUCT(csd) == 1)
    {
      fdbg("Found SDHC card\n");
    }
  else
    {
      fdbg("Found SD card\n");
    }
  mmcsd_dmpcsd(csd, slot->type);
#endif

  /* CSD data and set block size */

  mmcsd_decodecsd(slot, csd);
  mmcsd_checkwrprotect(slot, csd);
  mmcsd_setblklen(slot, slot->sectorsize);
  slot->state &= ~MMCSD_SLOTSTATUS_NOTREADY;
  return OK;
}

/****************************************************************************
 * Name: mmcsd_mediachanged
 *
 * Description:
 *   Handle initialization/media change events
 *
 ****************************************************************************/

static void mmcsd_mediachanged(void *arg)
{
  struct mmcsd_slot_s *slot = (struct mmcsd_slot_s*)arg;
  FAR struct spi_dev_s *spi;
  ubyte oldstate;
  int ret;

#ifdef CONFIG_DEBUG
  if (!slot || !slot->spi)
    {
      fdbg("Internal confusion\n");
      return;
    }
#endif
  spi  = slot->spi;

  /* Save the current slot state and reassess the new state */

  mmcsd_semtake(&slot->sem);
  oldstate = slot->state;

  /* Check if media was removed or inserted */

  slot->state &= ~(MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY|MMCSD_SLOTSTATUS_MEDIACHGD);
  if ((SPI_STATUS(spi, SPIDEV_MMCSD) & SPI_STATUS_PRESENT) == 0)
    {
      /* Media is not present */

      fdbg("No card present\n");
      slot->state |= (MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY);

      /* Was media removed? */

      if ((oldstate & MMCSD_SLOTSTATUS_NODISK) == 0)
        {
          slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
        }
    }

  /* Media is present, was it just inserted? Or, if it was previously not ready,
   * then try re-initializing it
   */

  else if ((oldstate & (MMCSD_SLOTSTATUS_NODISK|MMCSD_SLOTSTATUS_NOTREADY)) != 0)
    {
      /* (Re-)ininitialize for the media in the slot */

      ret = mmcsd_mediainitialize(slot);
      if (ret == 0)
        {
          fvdbg("mmcsd_mediainitialize returned OK\n");
          slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called
 *     up_spiinitialize() with the appropriate port number (see spi.h)
 *
 ****************************************************************************/

int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi)
{
  struct mmcsd_slot_s *slot;
  char devname[16];
  int ret;

#ifdef CONFIG_DEBUG
  if ((unsigned)slotno >= CONFIG_MMCSD_NSLOTS || (unsigned)minor > 255 || !spi)
    {
      fdbg("Invalid arguments\n");
      return -EINVAL;
    }
#endif

  /* Select the slot structure */

  slot = &g_mmcsdslot[slotno];
  memset(slot, 0, sizeof(struct mmcsd_slot_s));
  sem_init(&slot->sem, 0, 1);

#ifdef CONFIG_DEBUG
  if (slot->spi)
    {
      fdbg("Already registered\n");
      return -EBUSY;
    }
#endif

  /* Bind the SPI port to the slot */

  slot->spi = spi;

  /* Ininitialize for the media in the slot (if any) */

  ret = mmcsd_mediainitialize(slot);
  if (ret == 0)
    {
      fvdbg("mmcsd_mediainitialize returned OK\n");
      slot->state |= MMCSD_SLOTSTATUS_MEDIACHGD;
    }

  /* Create a MMC/SD device name */

  snprintf(devname, 16, "/dev/mmcsd%d", minor);

  /* Register the driver, even on a failure condition.  A
   * card may be inserted later, for example.
   */

  ret = register_blockdriver(devname, &g_bops, MMCSD_MODE, slot);
  if (ret < 0)
    {
      fdbg("register_blockdriver failed: %d\n", -ret);
      slot->spi = NULL;
      return ret;
    }

  /* Register a media change callback to handler insertion and
   * removal of cards.
   */

  (void)SPI_REGISTERCALLBACK(spi, mmcsd_mediachanged, (void*)slot);
  return OK;
}
