/****************************************************************************
 * drivers/mmcsd/mmcsd_sdio.c
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

#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SDIO)

#include <nuttx/compiler.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/mutex.h>

#include "mmcsd.h"
#include "mmcsd_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum number of references on the driver (because a uint8_t is used.
 * Use a larger type if more references are needed.
 */

#define MAX_CREFS               0xff

/* Timing (all in units of microseconds) */

#define MMCSD_POWERUP_DELAY     ((useconds_t)250)    /* 74 clock cycles @ 400KHz = 185uS */
#define MMCSD_IDLE_DELAY        ((useconds_t)100000) /* Short delay to allow change to IDLE state */
#define MMCSD_DSR_DELAY         ((useconds_t)100000) /* Time to wait after setting DSR */
#define MMCSD_CLK_DELAY         ((useconds_t)5000)   /* Delay after changing clock speeds */

/* Data delays (all in units of milliseconds).
 *
 *   For MMC & SD V1.x, these should be based on Nac = TAAC + NSAC; The
 *   maximum value of TAAC is 80MS and the maximum value of NSAC is 25.5K
 *   clock cycle.  For SD V2.x, a fixed delay of 100MS is recommend which is
 *   pretty close to the worst case SD V1.x Nac.  Here we just use 100MS
 *   delay for all data transfers.
 */

#define MMCSD_SCR_DATADELAY     (100)      /* Wait up to 100MS to get SCR */
#define MMCSD_BLOCK_RDATADELAY  (100)      /* Wait up to 100MS to get one data block */
#define MMCSD_BLOCK_WDATADELAY  (260)      /* Wait up to 260MS to write one data block */

#define IS_EMPTY(priv) (priv->type == MMCSD_CARDTYPE_UNKNOWN)

#if CONFIG_MMCSD_MULTIBLOCK_LIMIT == 0
# define MMCSD_MULTIBLOCK_LIMIT  SSIZE_MAX
#else
# define MMCSD_MULTIBLOCK_LIMIT  CONFIG_MMCSD_MULTIBLOCK_LIMIT
#endif

#define MMCSD_CAPACITY(b, s)    ((s) >= 10 ? (b) << ((s) - 10) : (b) >> (10 - (s)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is contains the unique state of the MMC/SD block driver */

struct mmcsd_state_s
{
  FAR struct sdio_dev_s *dev;      /* The SDIO device bound to this instance */
  uint8_t  crefs;                  /* Open references on the driver */
  mutex_t  lock;                   /* Assures mutually exclusive access to the slot */

  /* Status flags */

  uint8_t probed:1;                /* true: mmcsd_probe() discovered a card */
  uint8_t widebus:1;               /* true: Wide 4-bit bus selected */
  uint8_t mediachanged:1;          /* true: Media changed since last check */
  uint8_t wrbusy:1;                /* true: Last transfer was a write, card may be busy */
  uint8_t wrprotect:1;             /* true: Card is write protected (from CSD) */
  uint8_t locked:1;                /* true: Media is locked (from R1) */
  uint8_t dsrimp:1;                /* true: card supports CMD4/DSR setting (from CSD) */
#ifdef CONFIG_SDIO_DMA
  uint8_t dma:1;                   /* true: hardware supports DMA */
#endif

  uint8_t mode:2;                  /* (See MMCSDMODE_* definitions) */
  uint8_t type:4;                  /* Card type (See MMCSD_CARDTYPE_* definitions) */
  uint8_t buswidth:4;              /* Bus widths supported (SD only) */
  sdio_capset_t caps;              /* SDIO driver capabilities/limitations */
  uint16_t selblocklen;            /* The currently selected block length */
  uint16_t rca;                    /* Relative Card Address (RCS) register */

  /* Memory card geometry (extracted from the CSD) */

  uint8_t  blockshift;             /* Log2 of blocksize */
  uint16_t blocksize;              /* Read block length (== block size) */
  uint32_t nblocks;                /* Number of blocks */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc Helpers *************************************************************/

static int    mmcsd_lock(FAR struct mmcsd_state_s *priv);
static void   mmcsd_unlock(FAR struct mmcsd_state_s *priv);

/* Command/response helpers *************************************************/

static int     mmcsd_sendcmdpoll(FAR struct mmcsd_state_s *priv,
                 uint32_t cmd, uint32_t arg);
static int     mmcsd_recv_r1(FAR struct mmcsd_state_s *priv, uint32_t cmd);
static int     mmcsd_recv_r6(FAR struct mmcsd_state_s *priv, uint32_t cmd);
static int     mmcsd_get_scr(FAR struct mmcsd_state_s *priv,
                 uint32_t scr[2]);

static void    mmcsd_decode_csd(FAR struct mmcsd_state_s *priv,
                 uint32_t csd[4]);
#ifdef CONFIG_DEBUG_FS_INFO
static void    mmcsd_decode_cid(FAR struct mmcsd_state_s *priv,
                 uint32_t cid[4]);
#else
#  define mmcsd_decode_cid(priv,cid)
#endif
static void    mmcsd_decode_scr(FAR struct mmcsd_state_s *priv,
                 uint32_t scr[2]);

static int     mmcsd_get_r1(FAR struct mmcsd_state_s *priv,
                 FAR uint32_t *r1);
static int     mmcsd_verifystate(FAR struct mmcsd_state_s *priv,
                 uint32_t status);

/* Transfer helpers *********************************************************/

static bool    mmcsd_wrprotected(FAR struct mmcsd_state_s *priv);
static int     mmcsd_eventwait(FAR struct mmcsd_state_s *priv,
                 sdio_eventset_t failevents);
static int     mmcsd_transferready(FAR struct mmcsd_state_s *priv);
#if MMCSD_MULTIBLOCK_LIMIT != 1
static int     mmcsd_stoptransmission(FAR struct mmcsd_state_s *priv);
#endif
static int     mmcsd_setblocklen(FAR struct mmcsd_state_s *priv,
                 uint32_t blocklen);
static ssize_t mmcsd_readsingle(FAR struct mmcsd_state_s *priv,
                 FAR uint8_t *buffer, off_t startblock);
#if MMCSD_MULTIBLOCK_LIMIT != 1
static ssize_t mmcsd_readmultiple(FAR struct mmcsd_state_s *priv,
                 FAR uint8_t *buffer, off_t startblock, size_t nblocks);
#endif
static ssize_t mmcsd_writesingle(FAR struct mmcsd_state_s *priv,
                 FAR const uint8_t *buffer, off_t startblock);
#if MMCSD_MULTIBLOCK_LIMIT != 1
static ssize_t mmcsd_writemultiple(FAR struct mmcsd_state_s *priv,
                 FAR const uint8_t *buffer, off_t startblock,
                 size_t nblocks);
#endif

/* Block driver methods *****************************************************/

static int     mmcsd_open(FAR struct inode *inode);
static int     mmcsd_close(FAR struct inode *inode);
static ssize_t mmcsd_read(FAR struct inode *inode, FAR unsigned char *buffer,
                 blkcnt_t startsector, unsigned int nsectors);
static ssize_t mmcsd_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, blkcnt_t startsector,
                 unsigned int nsectors);
static int     mmcsd_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     mmcsd_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

/* Initialization/uninitialization/reset ************************************/

static void    mmcsd_mediachange(FAR void *arg);
static int     mmcsd_widebus(FAR struct mmcsd_state_s *priv);
#ifdef CONFIG_MMCSD_MMCSUPPORT
static int     mmcsd_mmcinitialize(FAR struct mmcsd_state_s *priv);
static int     mmcsd_read_csd(FAR struct mmcsd_state_s *priv);
#endif
static int     mmcsd_sdinitialize(FAR struct mmcsd_state_s *priv);
static int     mmcsd_cardidentify(FAR struct mmcsd_state_s *priv);
static int     mmcsd_probe(FAR struct mmcsd_state_s *priv);
static int     mmcsd_removed(FAR struct mmcsd_state_s *priv);
static int     mmcsd_hwinitialize(FAR struct mmcsd_state_s *priv);
static void    mmcsd_hwuninitialize(FAR struct mmcsd_state_s *priv);
#ifdef CONFIG_MMCSD_IOCSUPPORT
static int     mmcsd_iocmd(FAR struct mmcsd_state_s *priv,
                           FAR struct mmc_ioc_cmd *ic_ptr);
static int     mmcsd_multi_iocmd(FAR struct mmcsd_state_s *priv,
                                 FAR struct mmc_ioc_multi_cmd *imc_ptr);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  mmcsd_open,     /* open     */
  mmcsd_close,    /* close    */
  mmcsd_read,     /* read     */
  mmcsd_write,    /* write    */
  mmcsd_geometry, /* geometry */
  mmcsd_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Misc Helpers
 ****************************************************************************/

static int mmcsd_lock(FAR struct mmcsd_state_s *priv)
{
  int ret;

  /* Take the lock, giving exclusive access to the driver (perhaps
   * waiting)
   */

  if (!up_interrupt_context() && !sched_idletask())
    {
      ret = nxmutex_lock(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }

      /* Lock the bus if mutually exclusive access to the
       * SDIO bus is required on this platform.
       */

#ifdef CONFIG_SDIO_MUXBUS
      SDIO_LOCK(priv->dev, TRUE);
#endif
    }
  else
    {
      ret = OK;
    }

  return ret;
}

static void mmcsd_unlock(FAR struct mmcsd_state_s *priv)
{
  if (!up_interrupt_context() && !sched_idletask())
    {
      /* Release the SDIO bus lock, then the MMC/SD driver mutex in the
       * opposite order that they were taken to assure that no deadlock
       * conditions will arise.
       */

#ifdef CONFIG_SDIO_MUXBUS
      SDIO_LOCK(priv->dev, FALSE);
#endif
      nxmutex_unlock(&priv->lock);
    }
}

/****************************************************************************
 * Command/Response Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_sendcmdpoll
 *
 * Description:
 *   Send a command and poll-wait for the response.
 *
 ****************************************************************************/

static int mmcsd_sendcmdpoll(FAR struct mmcsd_state_s *priv, uint32_t cmd,
                             uint32_t arg)
{
  int ret;

  /* Send the command */

  ret = SDIO_SENDCMD(priv->dev, cmd, arg);
  if (ret == OK)
    {
      /* Then poll-wait until the response is available */

      ret = SDIO_WAITRESPONSE(priv->dev, cmd);
      if (ret != OK)
        {
          ferr("ERROR: Wait for response to cmd: %08" PRIx32
               " failed: %d\n",
               cmd, ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_sendcmd4
 *
 * Description:
 *   Set the Driver Stage Register (DSR) if (1) a CONFIG_MMCSD_DSR has been
 *   provided and (2) the card supports a DSR register.  If no DSR value
 *   the card default value (0x0404) will be used.
 *
 ****************************************************************************/

static inline int mmcsd_sendcmd4(FAR struct mmcsd_state_s *priv)
{
  int ret = OK;

#ifdef CONFIG_MMCSD_DSR
  /* The dsr_imp bit from the CSD will tell us if the card supports setting
   * the DSR via CMD4 or not.
   */

  if (priv->dsrimp != false)
    {
      /* CMD4 = SET_DSR will set the cards DSR register. The DSR and CMD4
       * support are optional.  However, since this is a broadcast command
       * with no response (like CMD0), we will never know if the DSR was
       * set correctly or not
       */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD4, CONFIG_MMCSD_DSR << 16);
      nxsig_usleep(MMCSD_DSR_DELAY);

      /* Send it again to have more confidence */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD4, CONFIG_MMCSD_DSR << 16);
      nxsig_usleep(MMCSD_DSR_DELAY);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: mmcsd_recv_r1
 *
 * Description:
 *   Receive R1 response and check for errors.
 *
 ****************************************************************************/

static int mmcsd_recv_r1(FAR struct mmcsd_state_s *priv, uint32_t cmd)
{
  uint32_t r1;
  int ret;

  /* Get the R1 response from the hardware */

  ret = SDIO_RECVR1(priv->dev, cmd, &r1);
  if (ret == OK)
    {
      /* Check if R1 reports an error */

      if ((r1 & MMCSD_R1_ERRORMASK) != 0)
        {
          /* Card locked is considered an error. Save the card locked
           * indication for later use.
           */

          ferr("ERROR: R1=%08" PRIx32 "\n", r1);
          priv->locked = ((r1 & MMCSD_R1_CARDISLOCKED) != 0);
          ret = -EIO;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_recv_r6
 *
 * Description:
 *   Receive R6 response and check for errors.  On success, priv->rca is set
 *   to the received RCA
 *
 ****************************************************************************/

static int mmcsd_recv_r6(FAR struct mmcsd_state_s *priv, uint32_t cmd)
{
  uint32_t r6 = 0;
  int ret;

  /* R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * Get the R6 response from the hardware
   */

  ret = SDIO_RECVR6(priv->dev, cmd, &r6);
  if (ret == OK)
    {
      /* Check if R6 reports an error */

      if ((r6 & MMCSD_R6_ERRORMASK) == 0)
        {
          /* No, save the RCA and return success */

          priv->rca = (uint16_t)(r6 >> 16);
          return OK;
        }

      /* Otherwise, return an I/O failure */

      ret = -EIO;
    }

  ferr("ERROR: Failed to get RCA. R6=%08" PRIx32 ": %d\n", r6, ret);
  return ret;
}

/****************************************************************************
 * Name: mmcsd_get_scr
 *
 * Description:
 *   Obtain the SD card's Configuration Register (SCR)
 *
 * Returned Value:
 *   OK on success; a negated ernno on failure.
 *
 ****************************************************************************/

static int mmcsd_get_scr(FAR struct mmcsd_state_s *priv, uint32_t scr[2])
{
  int ret;

  /* Set Block Size To 8 Bytes */

  ret = mmcsd_setblocklen(priv, 8);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* Setup up to receive data with interrupt mode */

  SDIO_BLOCKSETUP(priv->dev, 8, 1);
  SDIO_RECVSETUP(priv->dev, (FAR uint8_t *)scr, 8);

  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_SCR_DATADELAY);

  /* Send CMD55 APP_CMD with argument as card's RCA */

  mmcsd_sendcmdpoll(priv, SD_CMD55, (uint32_t)priv->rca << 16);
  ret = mmcsd_recv_r1(priv, SD_CMD55);
  if (ret != OK)
    {
      ferr("ERROR: RECVR1 for CMD55 failed: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 to start data receipt */

  mmcsd_sendcmdpoll(priv, SD_ACMD51, 0);
  ret = mmcsd_recv_r1(priv, SD_ACMD51);
  if (ret != OK)
    {
      ferr("ERROR: RECVR1 for ACMD51 failed: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Wait for data to be transferred */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_eventwait for READ DATA failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_decode_csd
 *
 * Description:
 *   Decode and extract necessary information from the CSD. If debug is
 *   enabled, then decode and show the full contents of the CSD.
 *
 * Returned Value:
 *   OK on success; a negated ernno on failure.  On success, the following
 *   values will be set in the driver state structure:
 *
 *   priv->dsrimp      true: card supports CMD4/DSR setting (from CSD)
 *   priv->wrprotect   true: card is write protected (from CSD)
 *   priv->blocksize   Read block length (== block size)
 *   priv->nblocks     Number of blocks
 *
 ****************************************************************************/

static void mmcsd_decode_csd(FAR struct mmcsd_state_s *priv, uint32_t csd[4])
{
#ifdef CONFIG_DEBUG_FS_INFO
  struct mmcsd_csd_s decoded;
#endif
  unsigned int readbllen;
  bool permwriteprotect;
  bool tmpwriteprotect;

  /* Word 1: Bits 127-96:
   *
   * CSD_STRUCTURE      127:126 CSD structure
   * SPEC_VERS          125:122 (MMC) Spec version
   * TAAC               119:112 Data read access-time-1
   *   TIME_VALUE         6:3   Time mantissa
   *   TIME_UNIT          2:0   Time exponent
   * NSAC               111:104 Data read access-time-2 in CLK
   *                            cycle(NSAC*100)
   * TRAN_SPEED         103:96  Max. data transfer rate
   *   TIME_VALUE         6:3   Rate exponent
   *   TRANSFER_RATE_UNIT 2:0   Rate mantissa
   */

#ifdef CONFIG_DEBUG_FS_INFO
  memset(&decoded, 0, sizeof(struct mmcsd_csd_s));
  decoded.csdstructure               =  csd[0] >> 30;
  decoded.mmcspecvers                = (csd[0] >> 26) & 0x0f;
  decoded.taac.timevalue             = (csd[0] >> 19) & 0x0f;
  decoded.taac.timeunit              = (csd[0] >> 16) & 7;
  decoded.nsac                       = (csd[0] >> 8)  & 0xff;
  decoded.transpeed.timevalue        = (csd[0] >> 3)  & 0x0f;
  decoded.transpeed.transferrateunit =  csd[0]        & 7;
#endif

  /* Word 2: Bits 64:95
   *   CCC                95:84 Card command classes
   *   READ_BL_LEN        83:80 Max. read data block length
   *   READ_BL_PARTIAL    79:79 Partial blocks for read allowed
   *   WRITE_BLK_MISALIGN 78:78 Write block misalignment
   *   READ_BLK_MISALIGN  77:77 Read block misalignment
   *   DSR_IMP            76:76 DSR implemented
   * Byte addressed SD and MMC:
   *   C_SIZE             73:62 Device size
   * Block addressed SD:
   *                      75:70 (reserved)
   *   C_SIZE             48:69 Device size
   */

  priv->dsrimp             = (csd[1] >> 12) & 1;
  readbllen                = (csd[1] >> 16) & 0x0f;

#ifdef CONFIG_DEBUG_FS_INFO
  decoded.ccc              = (csd[1] >> 20) & 0x0fff;
  decoded.readbllen        = (csd[1] >> 16) & 0x0f;
  decoded.readblpartial    = (csd[1] >> 15) & 1;
  decoded.writeblkmisalign = (csd[1] >> 14) & 1;
  decoded.readblkmisalign  = (csd[1] >> 13) & 1;
  decoded.dsrimp           = priv->dsrimp;
#endif

  /* Word 3: Bits 32-63
   *
   * Byte addressed SD:
   *   C_SIZE             73:62 Device size
   *   VDD_R_CURR_MIN     61:59 Max. read current at Vcc min
   *   VDD_R_CURR_MAX     58:56 Max. read current at Vcc max
   *   VDD_W_CURR_MIN     55:53 Max. write current at Vcc min
   *   VDD_W_CURR_MAX     52:50 Max. write current at Vcc max
   *   C_SIZE_MULT        49:47 Device size multiplier
   *   SD_ER_BLK_EN       46:46 Erase single block enable (SD only)
   *   SD_SECTOR_SIZE     45:39 Erase sector size
   *   SD_WP_GRP_SIZE     38:32 Write protect group size
   * Block addressed SD:
   *                      75:70 (reserved)
   *   C_SIZE             48:69 Device size
   *                      47:47 (reserved)
   *   SD_ER_BLK_EN       46:46 Erase single block enable (SD only)
   *   SD_SECTOR_SIZE     45:39 Erase sector size
   *   SD_WP_GRP_SIZE     38:32 Write protect group size
   * MMC:
   *   C_SIZE             73:62 Device size
   *   VDD_R_CURR_MIN     61:59 Max. read current at Vcc min
   *   VDD_R_CURR_MAX     58:56 Max. read current at Vcc max
   *   VDD_W_CURR_MIN     55:53 Max. write current at Vcc min
   *   VDD_W_CURR_MAX     52:50 Max. write current at Vcc max
   *   C_SIZE_MULT        49:47 Device size multiplier
   *   MMC_SECTOR_SIZE    46:42 Erase sector size
   *   MMC_ER_GRP_SIZE    41:37 Erase group size (MMC)
   *   MMC_WP_GRP_SIZE    36:32 Write protect group size
   */

  if (IS_BLOCK(priv->type))
    {
#ifdef CONFIG_MMCSD_MMCSUPPORT
      if (IS_MMC(priv->type))
        {
          /* Block addressed MMC:
           *
           * C_SIZE: 73:64 from Word 2 and 63:62 from Word 3
           */

          /* If the card is MMC and it has Block addressing
           * then the correct  number of blocks should already be
           * read from extended CSD register.
           */

#ifdef CONFIG_DEBUG_FS_INFO
          uint16_t csize        = ((csd[1] & 0x03ff) << 2) |
                                  ((csd[2] >> 30) & 3);
          uint8_t  csizemult    = (csd[2] >> 15) & 7;
#endif

          priv->blockshift      = readbllen;
          priv->blocksize       = (1 << readbllen);

          if (priv->blocksize > 512)
            {
              priv->blocksize   = 512;
              priv->blockshift  = 9;
            }

#ifdef CONFIG_DEBUG_FS_INFO
          decoded.u.mmc.csize               = csize;
          decoded.u.mmc.vddrcurrmin         = (csd[2] >> 27) & 7;
          decoded.u.mmc.vddrcurrmax         = (csd[2] >> 24) & 7;
          decoded.u.mmc.vddwcurrmin         = (csd[2] >> 21) & 7;
          decoded.u.mmc.vddwcurrmax         = (csd[2] >> 18) & 7;
          decoded.u.mmc.csizemult           = csizemult;
          decoded.u.mmc.er.mmc22.sectorsize = (csd[2] >> 10) & 0x1f;
          decoded.u.mmc.er.mmc22.ergrpsize  = (csd[2] >> 5) & 0x1f;
          decoded.u.mmc.mmcwpgrpsize        =  csd[2] & 0x1f;
#endif
        }
      else
#endif
        {
          /* Block addressed SD:
           *
           * C_SIZE: 69:64 from Word 2 and 63:48 from Word 3
           *
           *   512      = (1 << 9)
           *   1024     = (1 << 10)
           *   512*1024 = (1 << 19)
           */

          uint32_t csize        = ((csd[1] & 0x3f) << 16) | (csd[2] >> 16);

          priv->blockshift      = 9;
          priv->blocksize       = 1 << 9;
          priv->nblocks         = (csize + 1) << (19 - priv->blockshift);

#ifdef CONFIG_DEBUG_FS_INFO
          decoded.u.sdblock.csize        = csize;
          decoded.u.sdblock.sderblen     = (csd[2] >> 14) & 1;
          decoded.u.sdblock.sdsectorsize = (csd[2] >> 7) & 0x7f;
          decoded.u.sdblock.sdwpgrpsize  =  csd[2] & 0x7f;
#endif
        }
    }
  else
    {
      /* Byte addressed SD:
       *
       * C_SIZE: 73:64 from Word 2 and 63:62 from Word 3
       */

      uint16_t csize            = ((csd[1] & 0x03ff) << 2) |
                                  ((csd[2] >> 30) & 3);
      uint8_t  csizemult        = (csd[2] >> 15) & 7;

      priv->nblocks             = ((uint32_t)csize + 1) *
                                  (1 << (csizemult + 2));
      priv->blockshift          = readbllen;
      priv->blocksize           = (1 << readbllen);

      /* Some devices, such as 2Gb devices, report blocksizes larger than
       * 512 bytes but still expect to be accessed with a 512 byte blocksize.
       *
       * NOTE: A minor optimization would be to eliminated priv->blocksize
       * and priv->blockshift:  Those values will be 512 and 9 in all cases
       * anyway.
       */

      if (priv->blocksize > 512)
        {
          priv->nblocks       <<= (priv->blockshift - 9);
          priv->blocksize       = 512;
          priv->blockshift      = 9;
        }

#ifdef CONFIG_DEBUG_FS_INFO
      if (IS_SD(priv->type))
        {
          decoded.u.sdbyte.csize            = csize;
          decoded.u.sdbyte.vddrcurrmin      = (csd[2] >> 27) & 7;
          decoded.u.sdbyte.vddrcurrmax      = (csd[2] >> 24) & 7;
          decoded.u.sdbyte.vddwcurrmin      = (csd[2] >> 21) & 7;
          decoded.u.sdbyte.vddwcurrmax      = (csd[2] >> 18) & 7;
          decoded.u.sdbyte.csizemult        = csizemult;
          decoded.u.sdbyte.sderblen         = (csd[2] >> 14) & 1;
          decoded.u.sdbyte.sdsectorsize     = (csd[2] >> 7) & 0x7f;
          decoded.u.sdbyte.sdwpgrpsize      =  csd[2] & 0x7f;
        }
#ifdef CONFIG_MMCSD_MMCSUPPORT
      else if (IS_MMC(priv->type))
        {
          decoded.u.mmc.csize               = csize;
          decoded.u.mmc.vddrcurrmin         = (csd[2] >> 27) & 7;
          decoded.u.mmc.vddrcurrmax         = (csd[2] >> 24) & 7;
          decoded.u.mmc.vddwcurrmin         = (csd[2] >> 21) & 7;
          decoded.u.mmc.vddwcurrmax         = (csd[2] >> 18) & 7;
          decoded.u.mmc.csizemult           = csizemult;
          decoded.u.mmc.er.mmc22.sectorsize = (csd[2] >> 10) & 0x1f;
          decoded.u.mmc.er.mmc22.ergrpsize  = (csd[2] >> 5) & 0x1f;
          decoded.u.mmc.mmcwpgrpsize        =  csd[2] & 0x1f;
        }
#endif
#endif
    }

  /* Word 4: Bits 0-31
   *   WP_GRP_EN           31:31 Write protect group enable
   *   MMC DFLT_ECC        30:29 Manufacturer default ECC (MMC only)
   *   R2W_FACTOR          28:26 Write speed factor
   *   WRITE_BL_LEN        25:22 Max. write data block length
   *   WRITE_BL_PARTIAL    21:21 Partial blocks for write allowed
   *   FILE_FORMAT_GROUP   15:15 File format group
   *   COPY                14:14 Copy flag (OTP)
   *   PERM_WRITE_PROTECT  13:13 Permanent write protection
   *   TMP_WRITE_PROTECT   12:12 Temporary write protection
   *   FILE_FORMAT         10:11 File format
   *   ECC                  9:8  ECC (MMC only)
   *   CRC                  7:1  CRC
   *   Not used             0:0
   */

  permwriteprotect              = (csd[3] >> 13) & 1;
  tmpwriteprotect               = (csd[3] >> 12) & 1;
  priv->wrprotect               = (permwriteprotect || tmpwriteprotect);

#ifdef CONFIG_DEBUG_FS_INFO
  decoded.wpgrpen               =  csd[3] >> 31;
  decoded.mmcdfltecc            = (csd[3] >> 29) & 3;
  decoded.r2wfactor             = (csd[3] >> 26) & 7;
  decoded.writebllen            = (csd[3] >> 22) & 0x0f;
  decoded.writeblpartial        = (csd[3] >> 21) & 1;
  decoded.fileformatgrp         = (csd[3] >> 15) & 1;
  decoded.copy                  = (csd[3] >> 14) & 1;
  decoded.permwriteprotect      = permwriteprotect;
  decoded.tmpwriteprotect       = tmpwriteprotect;
  decoded.fileformat            = (csd[3] >> 10) & 3;
  decoded.mmcecc                = (csd[3] >> 8)  & 3;
  decoded.crc                   = (csd[3] >> 1)  & 0x7f;

  finfo("CSD:\n");
  finfo("  CSD_STRUCTURE: %d SPEC_VERS: %d (MMC)\n",
        decoded.csdstructure, decoded.mmcspecvers);
  finfo("  TAAC {TIME_UNIT: %d TIME_VALUE: %d} NSAC: %d\n",
        decoded.taac.timeunit, decoded.taac.timevalue, decoded.nsac);
  finfo("  TRAN_SPEED {TRANSFER_RATE_UNIT: %d TIME_VALUE: %d}\n",
        decoded.transpeed.transferrateunit, decoded.transpeed.timevalue);
  finfo("  CCC: %d\n", decoded.ccc);
  finfo("  READ_BL_LEN: %d READ_BL_PARTIAL: %d\n",
        decoded.readbllen, decoded.readblpartial);
  finfo("  WRITE_BLK_MISALIGN: %d READ_BLK_MISALIGN: %d\n",
        decoded.writeblkmisalign, decoded.readblkmisalign);
  finfo("  DSR_IMP: %d\n",
        decoded.dsrimp);

  if (IS_BLOCK(priv->type))
    {
#ifdef CONFIG_MMCSD_MMCSUPPORT
      if (IS_MMC(priv->type))
        {
          finfo("  MMC Block Addressing:\n");
          finfo("    C_SIZE: %d C_SIZE_MULT: %d\n",
                decoded.u.mmc.csize, decoded.u.mmc.csizemult);
          finfo("    VDD_R_CURR_MIN: %d VDD_R_CURR_MAX: %d\n",
                decoded.u.mmc.vddrcurrmin, decoded.u.mmc.vddrcurrmax);
          finfo("    VDD_W_CURR_MIN: %d VDD_W_CURR_MAX: %d\n",
                decoded.u.mmc.vddwcurrmin, decoded.u.mmc.vddwcurrmax);
          finfo("    MMC_SECTOR_SIZE: %d MMC_ER_GRP_SIZE: %d "
                "MMC_WP_GRP_SIZE: %d\n",
                decoded.u.mmc.er.mmc22.sectorsize,
                decoded.u.mmc.er.mmc22.ergrpsize,
                decoded.u.mmc.mmcwpgrpsize);
        }
      else
#endif
        {
          finfo("  SD Block Addressing:\n");
          finfo("    C_SIZE: %d SD_ER_BLK_EN: %d\n",
                decoded.u.sdblock.csize, decoded.u.sdblock.sderblen);
          finfo("    SD_SECTOR_SIZE: %d SD_WP_GRP_SIZE: %d\n",
                decoded.u.sdblock.sdsectorsize,
                decoded.u.sdblock.sdwpgrpsize);
        }
    }
  else if (IS_SD(priv->type))
    {
      finfo("  SD Byte Addressing:\n");
      finfo("    C_SIZE: %d C_SIZE_MULT: %d\n",
            decoded.u.sdbyte.csize, decoded.u.sdbyte.csizemult);
      finfo("    VDD_R_CURR_MIN: %d VDD_R_CURR_MAX: %d\n",
            decoded.u.sdbyte.vddrcurrmin, decoded.u.sdbyte.vddrcurrmax);
      finfo("    VDD_W_CURR_MIN: %d VDD_W_CURR_MAX: %d\n",
            decoded.u.sdbyte.vddwcurrmin, decoded.u.sdbyte.vddwcurrmax);
      finfo("    SD_ER_BLK_EN: %d SD_SECTOR_SIZE: %d (SD) "
            "SD_WP_GRP_SIZE: %d\n",
            decoded.u.sdbyte.sderblen, decoded.u.sdbyte.sdsectorsize,
            decoded.u.sdbyte.sdwpgrpsize);
    }
#ifdef CONFIG_MMCSD_MMCSUPPORT
  else if (IS_MMC(priv->type))
    {
      finfo("  MMC:\n");
      finfo("    C_SIZE: %d C_SIZE_MULT: %d\n",
            decoded.u.mmc.csize, decoded.u.mmc.csizemult);
      finfo("    VDD_R_CURR_MIN: %d VDD_R_CURR_MAX: %d\n",
            decoded.u.mmc.vddrcurrmin, decoded.u.mmc.vddrcurrmax);
      finfo("    VDD_W_CURR_MIN: %d VDD_W_CURR_MAX: %d\n",
            decoded.u.mmc.vddwcurrmin, decoded.u.mmc.vddwcurrmax);
      finfo("    MMC_SECTOR_SIZE: %d MMC_ER_GRP_SIZE: %d "
            "MMC_WP_GRP_SIZE: %d\n",
            decoded.u.mmc.er.mmc22.sectorsize,
            decoded.u.mmc.er.mmc22.ergrpsize,
            decoded.u.mmc.mmcwpgrpsize);
    }
#endif

  finfo("  WP_GRP_EN: %d MMC DFLT_ECC: %d (MMC) R2W_FACTOR: %d\n",
        decoded.wpgrpen, decoded.mmcdfltecc, decoded.r2wfactor);
  finfo("  WRITE_BL_LEN: %d WRITE_BL_PARTIAL: %d\n",
        decoded.writebllen, decoded.writeblpartial);
  finfo("  FILE_FORMAT_GROUP: %d COPY: %d\n",
        decoded.fileformatgrp, decoded.copy);
  finfo("  PERM_WRITE_PROTECT: %d TMP_WRITE_PROTECT: %d\n",
        decoded.permwriteprotect, decoded.tmpwriteprotect);
  finfo("  FILE_FORMAT: %d ECC: %d (MMC) CRC: %d\n",
        decoded.fileformat, decoded.mmcecc, decoded.crc);

  finfo("Capacity: %luKb, Block size: %db, nblocks: %d wrprotect: %d\n",
        (unsigned long)MMCSD_CAPACITY(priv->nblocks, priv->blockshift),
        priv->blocksize, priv->nblocks, priv->wrprotect);
#endif
}

/****************************************************************************
 * Name: mmcsd_decode_cid
 *
 * Description:
 *   Show the contents of the Card Identification Data (CID) (for debug
 *   purposes only)
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FS_INFO
static void mmcsd_decode_cid(FAR struct mmcsd_state_s *priv, uint32_t cid[4])
{
  struct mmcsd_cid_s decoded;

  /* Word 1: Bits 127-96:
   *   mid - 127-120  8-bit Manufacturer ID
   *   oid - 119-104 16-bit OEM/Application ID (ascii)
   *   pnm - 103-64  40-bit Product Name (ascii) + null terminator
   *         pnm[0] 103:96
   */

  decoded.mid    =  cid[0] >> 24;
  decoded.oid    = (cid[0] >> 8) & 0xffff;
  decoded.pnm[0] =  cid[0] & 0xff;

  /* Word 2: Bits 64:95
   *   pnm - 103-64  40-bit Product Name (ascii) + null terminator
   *         pnm[1] 95:88
   *         pnm[2] 87:80
   *         pnm[3] 79:72
   *         pnm[4] 71:64
   */

  decoded.pnm[1] =  cid[1] >> 24;
  decoded.pnm[2] = (cid[1] >> 16) & 0xff;
  decoded.pnm[3] = (cid[1] >> 8) & 0xff;
  decoded.pnm[4] =  cid[1] & 0xff;
  decoded.pnm[5] = '\0';

  /* Word 3: Bits 32-63
   *   prv -  63-56   8-bit Product revision
   *   psn -  55-24  32-bit Product serial number
   */

  decoded.prv    = cid[2] >> 24;
  decoded.psn    = cid[2] << 8;

  /* Word 4: Bits 0-31
   *   psn -  55-24  32-bit Product serial number
   *          23-20   4-bit (reserved)
   *   mdt -  19:8   12-bit Manufacturing date
   *   crc -   7:1    7-bit CRC7
   */

  decoded.psn   |=  cid[3] >> 24;
  decoded.mdt    = (cid[3] >> 8) & 0x0fff;
  decoded.crc    = (cid[3] >> 1) & 0x7f;

  finfo("mid: %02x oid: %04x pnm: %s prv: %d psn: %lu mdt: %02x crc: %02x\n",
      decoded.mid, decoded.oid, decoded.pnm, decoded.prv,
      (unsigned long)decoded.psn, decoded.mdt, decoded.crc);
}
#endif

/****************************************************************************
 * Name: mmcsd_decode_scr
 *
 * Description:
 *   Show the contents of the SD Configuration Register (SCR).  The only
 *   value retained is:  priv->buswidth;
 *
 ****************************************************************************/

static void mmcsd_decode_scr(FAR struct mmcsd_state_s *priv, uint32_t scr[2])
{
#ifdef CONFIG_DEBUG_FS_INFO
  struct mmcsd_scr_s decoded;
#endif

  /* Word 1, bits 63:32
   *   SCR_STRUCTURE          63:60 4-bit SCR structure version
   *   SD_VERSION             59:56 4-bit SD memory spec. version
   *   DATA_STATE_AFTER_ERASE 55:55 1-bit erase status
   *   SD_SECURITY            54:52 3-bit SD security support level
   *   SD_BUS_WIDTHS          51:48 4-bit bus width indicator
   *   Reserved               47:32 16-bit SD reserved space
   */

#ifdef CONFIG_ENDIAN_BIG  /* Card transfers SCR in big-endian order */
  priv->buswidth     = (scr[0] >> 16) & 15;
#else
  priv->buswidth     = (scr[0] >> 8) & 15;
#endif

#ifdef CONFIG_DEBUG_FS_INFO
#ifdef CONFIG_ENDIAN_BIG
  /* Card SCR is big-endian order / CPU also big-endian
   *   60   56   52   48   44   40   36   32
   * VVVV SSSS ESSS BBBB RRRR RRRR RRRR RRRR
   */

  decoded.scrversion =  scr[0] >> 28;
  decoded.sdversion  = (scr[0] >> 24) & 15;
  decoded.erasestate = (scr[0] >> 23) & 1;
  decoded.security   = (scr[0] >> 20) & 7;
#else
  /* Card SCR is big-endian order / CPU is little-endian
   *   36   32   44   40   52   48   60   56
   * RRRR RRRR RRRR RRRR ESSS BBBB VVVV SSSS
   */

  decoded.scrversion = (scr[0] >> 4)  & 15;
  decoded.sdversion  =  scr[0]        & 15;
  decoded.erasestate = (scr[0] >> 15) & 1;
  decoded.security   = (scr[0] >> 12) & 7;
#endif
  decoded.buswidth   = priv->buswidth;
#endif

  /* Word 1, bits 63:32
   *   Reserved               31:0  32-bits reserved for manufacturing usage.
   */

#ifdef CONFIG_DEBUG_FS_INFO
  decoded.mfgdata   = scr[1];  /* Might be byte reversed! */

  finfo("SCR:\n");
  finfo("  SCR_STRUCTURE: %d SD_VERSION: %d\n",
        decoded.scrversion, decoded.sdversion);
  finfo("  DATA_STATE_AFTER_ERASE: %d SD_SECURITY: %d SD_BUS_WIDTHS: %x\n",
        decoded.erasestate, decoded.security, decoded.buswidth);
  finfo("  Manufacturing data: %08x\n",
        decoded.mfgdata);
#endif
}

/****************************************************************************
 * Name: mmcsd_get_r1
 *
 * Description:
 *   Get the R1 status of the card using CMD13
 *
 ****************************************************************************/

static int mmcsd_get_r1(FAR struct mmcsd_state_s *priv, FAR uint32_t *r1)
{
  uint32_t local_r1;
  int ret;

  DEBUGASSERT(priv != NULL && r1 != NULL);

  /* Send CMD13, SEND_STATUS.  The addressed card responds by sending its
   * R1 card status register.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD13, (uint32_t)priv->rca << 16);
  ret = SDIO_RECVR1(priv->dev, MMCSD_CMD13, &local_r1);
  if (ret == OK)
    {
      /* Check if R1 reports an error */

      if ((local_r1 & MMCSD_R1_ERRORMASK) != 0)
        {
          /* Card locked is considered an error. Save the card locked
           * indication for later use.
           */

          priv->locked = ((local_r1 & MMCSD_R1_CARDISLOCKED) != 0);

          /* We must tell someone which error bits were set. */

          fwarn("WARNING: mmcsd_get_r1 returned errors: R1=%08" PRIx32 "\n",
                local_r1);
          ret = -EIO;
        }
      else
        {
          /* No errors, return R1 */

          *r1 = local_r1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_verifystate
 *
 * Description:
 *   Verify that the card is in STANDBY state
 *
 ****************************************************************************/

static int mmcsd_verifystate(FAR struct mmcsd_state_s *priv, uint32_t state)
{
  uint32_t r1;
  int ret;

  /* Get the current R1 status from the card */

  ret = mmcsd_get_r1(priv, &r1);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_get_r1 failed: %d\n", ret);
      return ret;
    }

  /* Now check if the card is in the expected state. */

  if (IS_STATE(r1, state))
    {
      /* Yes.. return Success */

      priv->wrbusy = false;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_wrprotected
 *
 * Description:
 *   Return true if the card is nlocked or write protected.
 *
 ****************************************************************************/

static bool mmcsd_wrprotected(FAR struct mmcsd_state_s *priv)
{
  /* Check if the card is locked (priv->locked) or write protected either (1)
   * via software as reported via the CSD and retained in priv->wrprotect or
   * (2) via the mechanical write protect on the card (which we get from the
   * SDIO driver via SDIO_WRPROTECTED)
   */

  return (priv->wrprotect || priv->locked || SDIO_WRPROTECTED(priv->dev));
}

/****************************************************************************
 * Name: mmcsd_eventwait
 *
 * Description:
 *   Wait for the specified events to occur.  Check for wakeup on error
 *   events.
 *
 ****************************************************************************/

static int mmcsd_eventwait(FAR struct mmcsd_state_s *priv,
                           sdio_eventset_t failevents)
{
  sdio_eventset_t wkupevent;

  /* Wait for the set of events enabled by SDIO_EVENTENABLE. */

  wkupevent = SDIO_EVENTWAIT(priv->dev);

  /* SDIO_EVENTWAIT returns the event set containing the event(s) that ended
   * the wait.  It should always be non-zero, but may contain failure as
   * well as success events.  Check if it contains any failure events.
   */

  if ((wkupevent & failevents) != 0)
    {
      /* Yes.. the failure event is probably SDIOWAIT_TIMEOUT */

      ferr("ERROR: Awakened with %02x\n", wkupevent);
      return wkupevent & SDIOWAIT_TIMEOUT ? -ETIMEDOUT : -EIO;
    }

  /* Since there are no failure events, we must have been awakened by one
   * (or more) success events.
   */

  return OK;
}

/****************************************************************************
 * Name: mmcsd_transferready
 *
 * Description:
 *   Check if the MMC/SD card is ready for the next read or write transfer.
 *   Ready means:  (1) card still in the slot, and (2) if the last transfer
 *   was a write transfer, the card is no longer busy from that transfer.
 *
 ****************************************************************************/

static int mmcsd_transferready(FAR struct mmcsd_state_s *priv)
{
  clock_t starttime;
  clock_t elapsed;
  uint32_t r1;
  int ret;

  /* First, check if the card has been removed. */

  if (IS_EMPTY(priv) || !SDIO_PRESENT(priv->dev))
    {
      ferr("ERROR: Card has been removed\n");
      return -ENODEV;
    }

  /* If the last data transfer was not a write, then we do not have to check
   * the card status.
   */

  else if (!priv->wrbusy)
    {
      return OK;
    }

  /* The card is still present and the last transfer was a write transfer.
   * Loop, querying the card state.  Return when (1) the card is in the
   * TRANSFER state, (2) the card stays in the PROGRAMMING state too long,
   * or (3) the card is in any other state.
   *
   * The PROGRAMMING state occurs normally after a WRITE operation.  During
   * this time, the card may be busy completing the WRITE and is not
   * available for other operations.  The card will transition from the
   * PROGRAMMING state to the TRANSFER state when the card completes the
   * WRITE operation.
   */

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_eventwait for transfer ready failed: %d\n", ret);
    }
#endif

  starttime = clock_systime_ticks();
  do
    {
      /* Get the current R1 status from the card */

      ret = mmcsd_get_r1(priv, &r1);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_get_r1 failed: %d\n", ret);
          goto errorout;
        }

      /* Now check if the card is in the expected transfer state. */

      if (IS_STATE(r1, MMCSD_R1_STATE_TRAN))
        {
          /* Yes.. return Success */

          priv->wrbusy = false;
          return OK;
        }

      /* Check for the programming state. This is not an error.  It means
       * that the card is still busy from the last (write) transfer.  The
       * card can also still be receiving data, for example, if hardware
       * receive FIFOs are not yet empty.
       */

      else if (!IS_STATE(r1, MMCSD_R1_STATE_PRG) &&
               !IS_STATE(r1, MMCSD_R1_STATE_RCV))
        {
          /* Any other state would be an error in this context.  There is
           * a possibility that the card is not selected.  In this case,
           * it could be in STANDBY or DISCONNECTED state and the fix
           * might be to send CMD7 to re-select the card.  Consider this
           * if this error occurs.
           */

          ferr("ERROR: Unexpected R1 state: %08" PRIx32 "\n", r1);
          ret = -EINVAL;
          goto errorout;
        }

      /* Do not hog the CPU */

      nxsig_usleep(1000);

      /* We are still in the programming state. Calculate the elapsed
       * time... we can't stay in this loop forever!
       */

      elapsed = clock_systime_ticks() - starttime;
    }
  while (elapsed < TICK_PER_SEC);

  return -ETIMEDOUT;

errorout:
  mmcsd_removed(priv);
  return ret;
}

/****************************************************************************
 * Name: mmcsd_stoptransmission
 *
 * Description:
 *   Send STOP_TRANSMISSION
 *
 ****************************************************************************/

#if MMCSD_MULTIBLOCK_LIMIT != 1
static int mmcsd_stoptransmission(FAR struct mmcsd_state_s *priv)
{
  int ret;

  /* Send CMD12, STOP_TRANSMISSION, and verify good R1 return status  */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD12, 0);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD12);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD12 failed: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: mmcsd_setblocklen
 *
 * Description:
 *   Set the block length.
 *
 ****************************************************************************/

static int mmcsd_setblocklen(FAR struct mmcsd_state_s *priv,
                             uint32_t blocklen)
{
  int ret = OK;

  /* Is the block length already selected in the card? */

  if (priv->selblocklen != blocklen)
    {
      /* Send CMD16 = SET_BLOCKLEN.  This command sets the block length (in
       * bytes) for all following block commands (read and write). Default
       * block length is specified in the CSD.
       */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD16, blocklen);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD16);
      if (ret == OK)
        {
          priv->selblocklen = blocklen;
        }
      else
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD16 failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_readsingle
 *
 * Description:
 *   Read a single block of data.
 *
 ****************************************************************************/

static ssize_t mmcsd_readsingle(FAR struct mmcsd_state_s *priv,
                                FAR uint8_t *buffer, off_t startblock)
{
  off_t offset;
  int ret;

  finfo("startblock=%jd\n", (intmax_t)startblock);
  DEBUGASSERT(priv != NULL && buffer != NULL);

  /* Check if the card is locked */

  if (priv->locked)
    {
      ferr("ERROR: Card is locked\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, priv->blocksize);

      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* If this is a byte addressed SD card, then convert sector start sector
   * number to a byte offset
   */

  if (IS_BLOCK(priv->type))
    {
      offset = startblock;
    }
  else
    {
      offset = startblock << priv->blockshift;
    }

  finfo("offset=%jd\n", (intmax_t)offset);

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* Configure SDIO controller hardware for the read transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, 1);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_BLOCK_RDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMARECVSETUP(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          finfo("SDIO_DMARECVSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_RECVSETUP(priv->dev, buffer, priv->blocksize);
    }

  /* Send CMD17, READ_SINGLE_BLOCK: Read a block of the size selected
   * by the mmcsd_setblocklen() and verify that good R1 status is
   * returned.  The card state should change from Transfer to Sending-Data
   * state.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD17, offset);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD17);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD17 failed: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Then wait for the data transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD17 transfer failed: %d\n", ret);
      return ret;
    }

  /* Return value:  One sector read */

  return 1;
}

/****************************************************************************
 * Name: mmcsd_readmultiple
 *
 * Description:
 *   Read multiple, contiguous blocks of data from the physical device.
 *
 ****************************************************************************/

#if MMCSD_MULTIBLOCK_LIMIT != 1
static ssize_t mmcsd_readmultiple(FAR struct mmcsd_state_s *priv,
                                  FAR uint8_t *buffer, off_t startblock,
                                  size_t nblocks)
{
  size_t nbytes = nblocks << priv->blockshift;
  off_t  offset;
  int ret;

  finfo("startblock=%jd nblocks=%zu\n", (intmax_t)startblock, nblocks);
  DEBUGASSERT(priv != NULL && buffer != NULL && nblocks > 1);

  /* Check if the card is locked */

  if (priv->locked)
    {
      ferr("ERROR: Card is locked\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, nbytes);

      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* If this is a byte addressed SD card, then convert both the total
   * transfer size to bytes and the sector start sector number to a byte
   * offset
   */

  if (IS_BLOCK(priv->type))
    {
      offset = startblock;
    }
  else
    {
      offset = startblock << priv->blockshift;
    }

  finfo("nbytes=%zu byte offset=%jd\n", nbytes, (intmax_t)offset);

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* Configure SDIO controller hardware for the read transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, nblocks);
  SDIO_WAITENABLE(priv->dev,
                 SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                 nblocks * MMCSD_BLOCK_RDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMARECVSETUP(priv->dev, buffer, nbytes);
      if (ret != OK)
        {
          finfo("SDIO_DMARECVSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_RECVSETUP(priv->dev, buffer, nbytes);
    }

  /* Send CMD18, READ_MULT_BLOCK: Read a block of the size selected by
   * the mmcsd_setblocklen() and verify that good R1 status is returned
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD18, offset);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD18);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD18 failed: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Wait for the transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD18 transfer failed: %d\n", ret);
      return ret;
    }

  /* Send STOP_TRANSMISSION */

  ret = mmcsd_stoptransmission(priv);

  if (ret != OK)
    {
      ferr("ERROR: mmcsd_stoptransmission failed: %d\n", ret);
    }

  /* On success, return the number of blocks read */

  return nblocks;
}
#endif

/****************************************************************************
 * Name: mmcsd_writesingle
 *
 * Description:
 *   Write a single block of data to the physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_writesingle(FAR struct mmcsd_state_s *priv,
                                 FAR const uint8_t *buffer, off_t startblock)
{
  off_t offset;
  int ret;

  finfo("startblock=%jd\n", (intmax_t)startblock);
  DEBUGASSERT(priv != NULL && buffer != NULL);

  /* Check if the card is locked or write protected (either via software or
   * via the mechanical write protect on the card)
   */

  if (mmcsd_wrprotected(priv))
    {
      ferr("ERROR: Card is locked or write protected\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, priv->blocksize);

      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* If this is a byte addressed SD card, then convert sector start sector
   * number to a byte offset
   */

  if (IS_BLOCK(priv->type))
    {
      offset = startblock;
    }
  else
    {
      offset = startblock << priv->blockshift;
    }

  finfo("offset=%jd\n", (intmax_t)offset);

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* If Controller does not need DMA setup before the write then send CMD24
   * now.
   */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) == 0)
    {
      /* Send CMD24, WRITE_BLOCK, and verify good R1 status is returned */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD24, offset);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD24);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD24 failed: %d\n", ret);
          return ret;
        }
    }

  /* Configure SDIO controller hardware for the write transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, 1);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_BLOCK_WDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMASENDSETUP(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          finfo("SDIO_DMASENDSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_SENDSETUP(priv->dev, buffer, priv->blocksize);
    }

  /* If Controller needs DMA setup before write then only send CMD24 now. */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) != 0)
    {
      /* Send CMD24, WRITE_BLOCK, and verify good R1 status is returned */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD24, offset);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD24);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD24 failed: %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }

  /* Wait for the transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD24 transfer failed: %d\n", ret);
      return ret;
    }

  /* Flag that a write transfer is pending that we will have to check for
   * write complete at the beginning of the next transfer.
   */

  priv->wrbusy = true;

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  /* Arm the write complete detection with timeout */

  SDIO_WAITENABLE(priv->dev, SDIOWAIT_WRCOMPLETE | SDIOWAIT_TIMEOUT,
                  MMCSD_BLOCK_WDATADELAY);
#endif

  /* On success, return the number of blocks written */

  return 1;
}

/****************************************************************************
 * Name: mmcsd_writemultiple
 *
 * Description:
 *   Write multiple, contiguous blocks of data to the physical device.
 *   This function expects that the data to be written is contained in
 *   one large buffer that is pointed to by buffer.
 *
 ****************************************************************************/

#if MMCSD_MULTIBLOCK_LIMIT != 1
static ssize_t mmcsd_writemultiple(FAR struct mmcsd_state_s *priv,
                                   FAR const uint8_t *buffer,
                                   off_t startblock, size_t nblocks)
{
  size_t nbytes = nblocks << priv->blockshift;
  off_t  offset;
  int ret;
  int evret = OK;

  finfo("startblock=%jd nblocks=%zu\n", (intmax_t)startblock, nblocks);
  DEBUGASSERT(priv != NULL && buffer != NULL && nblocks > 1);

  /* Check if the card is locked or write protected (either via software or
   * via the mechanical write protect on the card)
   */

  if (mmcsd_wrprotected(priv))
    {
      ferr("ERROR: Card is locked or write protected\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, nbytes);

      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* If this is a byte addressed SD card, then convert both the total
   * transfer size to bytes and the sector start sector number to a byte
   * offset
   */

  if (IS_BLOCK(priv->type))
    {
      offset = startblock;
    }
  else
    {
      offset = startblock << priv->blockshift;
    }

  finfo("nbytes=%zu byte offset=%jd\n", nbytes, (intmax_t)offset);

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* If this is an SD card, then send ACMD23 (SET_WR_BLK_ERASE_COUNT) just
   * before sending CMD25 (WRITE_MULTIPLE_BLOCK).  This sets the number of
   * write blocks to be pre-erased and might make the following multiple
   * block write command faster.
   */

  if (IS_SD(priv->type))
    {
      /* Send CMD55, APP_CMD, a verify that good R1 status is returned */

      mmcsd_sendcmdpoll(priv, SD_CMD55, (uint32_t)priv->rca << 16);
      ret = mmcsd_recv_r1(priv, SD_CMD55);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD55 (ACMD23) failed: %d\n", ret);
          return ret;
        }

      /* Send CMD23, SET_WR_BLK_ERASE_COUNT, and verify that good R1 status
       * is returned.
       */

      mmcsd_sendcmdpoll(priv, SD_ACMD23, nblocks);
      ret = mmcsd_recv_r1(priv, SD_ACMD23);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for ACMD23 failed: %d\n", ret);
          return ret;
        }
    }

  /* If Controller does not need DMA setup before the write then send CMD25
   * now.
   */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) == 0)
    {
      /* Send CMD25, WRITE_MULTIPLE_BLOCK, and verify that good R1 status
       * is returned
       */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD25, offset);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD25);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD25 failed: %d\n", ret);
          return ret;
        }
    }

  /* Configure SDIO controller hardware for the write transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, nblocks);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  nblocks * MMCSD_BLOCK_WDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMASENDSETUP(priv->dev, buffer, nbytes);
      if (ret != OK)
        {
          finfo("SDIO_DMASENDSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_SENDSETUP(priv->dev, buffer, nbytes);
    }

  /* If Controller needs DMA setup before write then only send CMD25 now. */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) != 0)
    {
      /* Send CMD25, WRITE_MULTIPLE_BLOCK, and verify that good R1 status
       * is returned
       */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD25, offset);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD25);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD25 failed: %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }

  /* Wait for the transfer to complete */

  evret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (evret != OK)
    {
      ferr("ERROR: CMD25 transfer failed: %d\n", evret);

      /* If we return from here, we probably leave the sd-card in
       * Receive-data State. Instead, we will remember that
       * an error occurred and try to execute the STOP_TRANSMISSION
       * to put the sd-card back into Transfer State.
       */
    }

  /* Send STOP_TRANSMISSION */

  ret = mmcsd_stoptransmission(priv);
  if (evret != OK)
    {
      return evret;
    }

  if (ret != OK)
    {
      ferr("ERROR: mmcsd_stoptransmission failed: %d\n", ret);
      return ret;
    }

  /* Flag that a write transfer is pending that we will have to check for
   * write complete at the beginning of the next transfer.
   */

  priv->wrbusy = true;

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  /* Arm the write complete detection with timeout */

  SDIO_WAITENABLE(priv->dev, SDIOWAIT_WRCOMPLETE | SDIOWAIT_TIMEOUT,
                  nblocks * MMCSD_BLOCK_WDATADELAY);
#endif

  /* On success, return the number of blocks written */

  return nblocks;
}
#endif

/****************************************************************************
 * Name: mmcsd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mmcsd_open(FAR struct inode *inode)
{
  FAR struct mmcsd_state_s *priv;
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct mmcsd_state_s *)inode->i_private;

  /* Just increment the reference count on the driver */

  DEBUGASSERT(priv->crefs < MAX_CREFS);

  ret = mmcsd_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs++;
  mmcsd_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int mmcsd_close(FAR struct inode *inode)
{
  FAR struct mmcsd_state_s *priv;
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct mmcsd_state_s *)inode->i_private;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 0);
  ret = mmcsd_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->crefs--;
  mmcsd_unlock(priv);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_read
 *
 * Description:
 *   Read the specified number of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                          blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct mmcsd_state_s *priv;
  size_t sector;
  size_t endsector;
  ssize_t nread;
  ssize_t ret = nsectors;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct mmcsd_state_s *)inode->i_private;
  finfo("startsector: %" PRIuOFF " nsectors: %u sectorsize: %d\n",
        startsector, nsectors, priv->blocksize);

  if (nsectors > 0)
    {
      ret = mmcsd_lock(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = nsectors;
      endsector = startsector + nsectors;
      for (sector = startsector; sector < endsector; sector += nread)
        {
          /* Read this sector into the user buffer */

#if MMCSD_MULTIBLOCK_LIMIT == 1
          /* Read each block using only the single block transfer method */

          nread = mmcsd_readsingle(priv, buffer, sector);
#else
          nread = endsector - sector;
          if (nread > MMCSD_MULTIBLOCK_LIMIT)
            {
              nread = MMCSD_MULTIBLOCK_LIMIT;
            }

          if (nread == 1)
            {
              nread = mmcsd_readsingle(priv, buffer, sector);
            }
          else
            {
              nread = mmcsd_readmultiple(priv, buffer, sector, nread);
            }

#endif
          if (nread < 0)
            {
              ret = nread;
              break;
            }

          /* Increment the buffer pointer by the sector size */

          buffer += nread * priv->blocksize;
        }

      mmcsd_unlock(priv);
    }

  /* On success, return the number of blocks read */

  return ret;
}

/****************************************************************************
 * Name: mmcsd_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct mmcsd_state_s *priv;
  size_t sector;
  size_t endsector;
  ssize_t nwrite;
  ssize_t ret = nsectors;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct mmcsd_state_s *)inode->i_private;
  finfo("startsector: %" PRIuOFF " nsectors: %u sectorsize: %d\n",
        startsector, nsectors, priv->blocksize);

  if (nsectors > 0)
    {
      ret = mmcsd_lock(priv);
      if (ret < 0)
        {
          return ret;
        }

      ret = nsectors;
      endsector = startsector + nsectors;
      for (sector = startsector; sector < endsector; sector += nwrite)
        {
          /* Write this sector into the user buffer */

#if MMCSD_MULTIBLOCK_LIMIT == 1
          /* Write each block using only the single block transfer method */

          nwrite = mmcsd_writesingle(priv, buffer, sector);
#else
          nwrite = endsector - sector;
          if (nwrite > MMCSD_MULTIBLOCK_LIMIT)
            {
              nwrite = MMCSD_MULTIBLOCK_LIMIT;
            }

          if (nwrite == 1)
            {
              nwrite = mmcsd_writesingle(priv, buffer, sector);
            }
          else
            {
              nwrite = mmcsd_writemultiple(priv, buffer, sector, nwrite);
            }

#endif
          if (nwrite < 0)
            {
              ret = nwrite;
              break;
            }

          /* Increment the buffer pointer by the sector size */

          buffer += nwrite * priv->blocksize;
        }

      mmcsd_unlock(priv);
    }

  /* On success, return the number of blocks written */

  return ret;
}

/****************************************************************************
 * Name: mmcsd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct mmcsd_state_s *priv;
  int ret = -EINVAL;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  if (geometry)
    {
      memset(geometry, 0, sizeof(*geometry));

      /* Is there a (supported) card inserted in the slot? */

      priv = (FAR struct mmcsd_state_s *)inode->i_private;
      ret = mmcsd_lock(priv);
      if (ret < 0)
        {
          return ret;
        }

      if (IS_EMPTY(priv))
        {
          /* No.. return ENODEV */

          finfo("IS_EMPTY\n");
          ret = -ENODEV;
        }
      else
        {
          /* Yes.. return the geometry of the card */

          geometry->geo_available     = true;
          geometry->geo_mediachanged  = priv->mediachanged;
          geometry->geo_writeenabled  = !mmcsd_wrprotected(priv);
          geometry->geo_nsectors      = priv->nblocks;
          geometry->geo_sectorsize    = priv->blocksize;

          finfo("available: true mediachanged: %s writeenabled: %s\n",
                 geometry->geo_mediachanged ? "true" : "false",
                 geometry->geo_writeenabled ? "true" : "false");
          finfo("nsectors: %lu sectorsize: %" PRIi16 "\n",
                 (unsigned long)geometry->geo_nsectors,
                 geometry->geo_sectorsize);

          priv->mediachanged = false;
          ret = OK;
        }

      mmcsd_unlock(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcsd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct mmcsd_state_s *priv;
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct mmcsd_state_s *)inode->i_private;

  /* Process the IOCTL by command */

  ret = mmcsd_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
    case BIOC_PROBE: /* Check for media in the slot */
      {
        finfo("BIOC_PROBE\n");

        /* Probe the MMC/SD slot for media */

        ret = mmcsd_probe(priv);
        if (ret != OK)
          {
            ferr("ERROR: mmcsd_probe failed: %d\n", ret);
          }
      }
      break;

    case BIOC_EJECT: /* Media has been removed from the slot */
      {
        finfo("BIOC_EJECT\n");

        /* Process the removal of the card */

        ret = mmcsd_removed(priv);
        if (ret != OK)
          {
            ferr("ERROR: mmcsd_removed failed: %d\n", ret);
          }

        /* Enable logic to detect if a card is re-inserted */

        SDIO_CALLBACKENABLE(priv->dev, SDIOMEDIA_INSERTED);
      }
      break;

#ifdef CONFIG_MMCSD_IOCSUPPORT
    case MMC_IOC_CMD: /* MMCSD device ioctl commands */
      {
        finfo("MMC_IOC_CMD\n");
        ret = mmcsd_iocmd(priv, (FAR struct mmc_ioc_cmd *)arg);
        if (ret != OK)
          {
            ferr("ERROR: mmcsd_iocmd failed: %d\n", ret);
          }
      }
      break;

    case MMC_IOC_MULTI_CMD: /* MMCSD device ioctl muti commands */
      {
        finfo("MMC_IOC_MULTI_CMD\n");
        ret = mmcsd_multi_iocmd(priv, (FAR struct mmc_ioc_multi_cmd *)arg);
        if (ret != OK)
          {
            ferr("ERROR: mmcsd_iocmd failed: %d\n", ret);
          }
      }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  mmcsd_unlock(priv);
  return ret;
}

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_mediachange
 *
 * Description:
 *  This is a callback function from the SDIO driver that indicates that
 *  there has been a change in the slot... either a card has been inserted
 *  or a card has been removed.
 *
 * Assumptions:
 *   This callback is NOT supposd to run in the context of an interrupt
 *   handler; it is probably running in the context of work thread.
 *
 ****************************************************************************/

static void mmcsd_mediachange(FAR void *arg)
{
  FAR struct mmcsd_state_s *priv = (FAR struct mmcsd_state_s *)arg;
  int ret;

  finfo("arg: %p\n", arg);
  DEBUGASSERT(priv);

  /* Is there a card present in the slot? */

  ret = mmcsd_lock(priv);
  if (ret < 0)
    {
      return;
    }

  if (SDIO_PRESENT(priv->dev))
    {
      /* Yes... process the card insertion.  This could cause chaos if we
       * think that a card is already present and there are mounted file
       * systems!  NOTE that mmcsd_probe() will always re-enable callbacks
       * appropriately.
       */

      mmcsd_probe(priv);
    }
  else
    {
      /* No... process the card removal.  This could have very bad
       * implications for any mounted file systems!  NOTE that
       * mmcsd_removed() does NOT re-enable callbacks so we will need to
       * do that here.
       */

      mmcsd_removed(priv);

      /* Enable logic to detect if a card is re-inserted */

      SDIO_CALLBACKENABLE(priv->dev, SDIOMEDIA_INSERTED);
    }

  mmcsd_unlock(priv);
}

/****************************************************************************
 * Name: mmcsd_widebus
 *
 * Description:
 *  An SD card has been inserted and its SCR has been obtained.  Select wide
 *  (4-bit) bus operation if the card supports it.
 *
 * Assumptions:
 *   This function is called only once per card insertion as part of the SD
 *   card initialization sequence.  It is not necessary to reselect the card
 *   there is not need to check if wide bus operation has already been
 *   selected.
 *
 ****************************************************************************/

static int mmcsd_widebus(FAR struct mmcsd_state_s *priv)
{
  int ret;

  /* Check if the SD card supports wide bus operation (as reported in the
   * SCR or in the SDIO driver capabililities)
   */

  if ((priv->buswidth & MMCSD_SCR_BUSWIDTH_4BIT) != 0 &&
      (priv->caps & SDIO_CAPS_1BIT_ONLY) == 0)
    {
      /* Disconnect any CD/DAT3 pull up using ACMD42.  ACMD42 is optional and
       * need not be supported by all SD calls.
       *
       * First end CMD55 APP_CMD with argument as card's RCA.
       */

      mmcsd_sendcmdpoll(priv, SD_CMD55, (uint32_t)priv->rca << 16);
      ret = mmcsd_recv_r1(priv, SD_CMD55);
      if (ret != OK)
        {
          ferr("ERROR: RECVR1 for CMD55 of ACMD42: %d\n", ret);
          return ret;
        }

      /* Then send ACMD42 with the argument to disconnect the CD/DAT3
       * pull-up
       *
       * TODO: May want to disable, then re-enable around data transfers
       * to support card detection"
       */

      mmcsd_sendcmdpoll(priv, SD_ACMD42, MMCSD_ACMD42_CD_DISCONNECT);
      ret = mmcsd_recv_r1(priv, SD_ACMD42);
      if (ret != OK)
        {
          fwarn("WARNING: SD card does not support ACMD42: %d\n", ret);
          return ret;
        }

      /* Now send ACMD6 to select wide, 4-bit bus operation, beginning
       * with CMD55, APP_CMD:
       */

      mmcsd_sendcmdpoll(priv, SD_CMD55, (uint32_t)priv->rca << 16);
      ret = mmcsd_recv_r1(priv, SD_CMD55);
      if (ret != OK)
        {
          ferr("ERROR: RECVR1 for CMD55 of ACMD6: %d\n", ret);
          return ret;
        }

      /* Then send ACMD6 */

      mmcsd_sendcmdpoll(priv, SD_ACMD6, MMCSD_ACMD6_BUSWIDTH_4);
      ret = mmcsd_recv_r1(priv, SD_ACMD6);
      if (ret != OK)
        {
          return ret;
        }

      /* Configure the SDIO peripheral */

      finfo("Wide bus operation selected\n");
      SDIO_WIDEBUS(priv->dev, true);
      priv->widebus = true;

      SDIO_CLOCK(priv->dev, CLOCK_SD_TRANSFER_4BIT);
      nxsig_usleep(MMCSD_CLK_DELAY);
      return OK;
    }

  /* Wide bus operation not supported */

  fwarn("WARNING: Card does not support wide-bus operation\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name: mmcsd_mmcinitialize
 *
 * Description:
 *   We believe that there is an MMC card in the slot.  Attempt to initialize
 *   and configure the MMC card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_MMCSUPPORT
static int mmcsd_mmcinitialize(FAR struct mmcsd_state_s *priv)
{
  uint32_t cid[4];
  uint32_t csd[4];
  int ret;

  /* At this point, slow, ID mode clocking has been supplied to the card
   * and CMD0 has been sent successfully. CMD1 succeeded and ACMD41 failed
   * so there is good evidence that we have an MMC card inserted into the
   * slot.
   *
   * Send CMD2, ALL_SEND_CID. This implementation supports only one MMC
   * slot.  If multiple cards were installed, each card would respond to
   * CMD2 by sending its CID (only one card completes the response at a
   * time).  The driver should send CMD2 and assign an RCAs until no
   * response to ALL_SEND_CID is received. CMD2 causes transition to
   * identification state / card-identification mode.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD2, 0);
  ret = SDIO_RECVR2(priv->dev, MMCSD_CMD2, cid);
  if (ret != OK)
    {
      ferr("ERROR: SDIO_RECVR2 for MMC CID failed: %d\n", ret);
      return ret;
    }

  mmcsd_decode_cid(priv, cid);

  /* Send CMD3, SET_RELATIVE_ADDR.  This command is used to assign a logical
   * address to the card.  For MMC, the host assigns the address. CMD3 causes
   * transition to standby state/data-transfer mode
   */

  priv->rca = 1;  /* There is only one card */
  mmcsd_sendcmdpoll(priv, MMC_CMD3, priv->rca << 16);
  ret = mmcsd_recv_r1(priv, MMC_CMD3);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1(CMD3) failed: %d\n", ret);
      return ret;
    }

  /* This should have caused a transition to standby state. However, this
   * will not be reflected in the present R1/6 status.  R1/6 contains the
   * state of the card when the command was received, not when it completed
   * execution.
   *
   * Verify that we are in standby state/data-transfer mode
   */

  ret = mmcsd_verifystate(priv, MMCSD_R1_STATE_STBY);
  if (ret != OK)
    {
      ferr("ERROR: Failed to enter standby state\n");
      return ret;
    }

  /* Send CMD9, SEND_CSD in standby state/data-transfer mode to obtain the
   * Card Specific Data (CSD) register, e.g., block length, card storage
   * capacity, etc. (Stays in standby state/data-transfer mode)
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD9, priv->rca << 16);
  ret = SDIO_RECVR2(priv->dev, MMCSD_CMD9, csd);
  if (ret != OK)
    {
      ferr("ERROR: Could not get SD CSD register: %d\n", ret);
      return ret;
    }

  /* Set the Driver Stage Register (DSR) if (1) a CONFIG_MMCSD_DSR has been
   * provided and (2) the card supports a DSR register.  If no DSR value
   * the card default value (0x0404) will be used.
   */

  mmcsd_sendcmd4(priv);

  /* Send CMD7 with the argument == RCA in order to select the card
   * and send it in data-trasfer mode. Since we are supporting
   * only a single card, we just leave the card selected all of the time.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD7S, (uint32_t)priv->rca << 16);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD7S);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD7 failed: %d\n", ret);
      return ret;
    }

  /* CSD Decoding for MMC should be done after entering in data-transfer mode
   * because if the card has block addressing then extended CSD register
   * must be read in order to get the right number of blocks and capacity,
   * but it has to be done in data-transfer mode.
   */

  if (IS_BLOCK(priv->type))
    {
      ret = mmcsd_read_csd(priv);
      if (ret != OK)
        {
          ferr("ERROR: Failed to determinate number of blocks: %d\n", ret);
          return ret;
        }
    }

  mmcsd_decode_csd(priv, csd);

  /* Select high speed MMC clocking (which may depend on the DSR setting) */

  SDIO_CLOCK(priv->dev, CLOCK_MMC_TRANSFER);
  nxsig_usleep(MMCSD_CLK_DELAY);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_read_csd
 *
 * Description:
 *   MMC card is detected with block addressing and this function will read
 *   the correct number of blocks and capacity. Returns OK if ext CSD is read
 *   correctly or error in not.
 *
 *   Note:  For some MCU architectures, buffer[] must be aligned.
 *
 ****************************************************************************/

static int mmcsd_read_csd(FAR struct mmcsd_state_s *priv)
{
  uint8_t buffer[512] aligned_data(16);
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Check if the card is locked */

  if (priv->locked)
    {
      ferr("ERROR: Card is locked\n");
      return -EPERM;
    }

  memset(buffer, 0, sizeof(buffer));

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, 512);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* Configure SDIO controller hardware for the read transfer */

  SDIO_BLOCKSETUP(priv->dev, 512, 1);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_BLOCK_RDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMARECVSETUP(priv->dev, buffer, 512);
      if (ret != OK)
        {
          finfo("SDIO_DMARECVSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_RECVSETUP(priv->dev, buffer, 512);
    }

  /* Send CMD8 in data-transfer mode to obtain the
   * extended Card Specific Data (CSD) register, e.g., block length, card
   * storage capacity, etc.
   */

  mmcsd_sendcmdpoll(priv, MMC_CMD8, 0);
  ret = mmcsd_recv_r1(priv, MMC_CMD8);
  if (ret != OK)
    {
      ferr("ERROR: Could not get MMC extended CSD register: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Then wait for the data transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD17 transfer failed: %d\n", ret);
      return ret;
    }

  priv->nblocks = (buffer[215] << 24) | (buffer[214] << 16) |
                  (buffer[213] << 8) | buffer[212];

  finfo("MMC ext CSD read succsesfully, number of block %" PRId32 "\n",
        priv->nblocks);

  SDIO_GOTEXTCSD(priv->dev, buffer);

  /* Return value:  One sector read */

  return OK;
}
#endif

/****************************************************************************
 * Name: mmcsd_general_cmd_write
 *
 * Description:
 *   Send cmd56 data, one sector size
 *
 ****************************************************************************/

static int mmcsd_general_cmd_write(FAR struct mmcsd_state_s *priv,
                                   FAR const uint8_t *buffer,
                                   off_t startblock)
{
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL);

  /* Check if the card is locked or write protected (either via software or
   * via the mechanical write protect on the card)
   */

  if (mmcsd_wrprotected(priv))
    {
      ferr("ERROR: Card is locked or write protected\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* If Controller does not need DMA setup before the write then send CMD56
   * now.
   */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) == 0)
    {
      /* Send CMD56, WRITE_BLOCK, and verify good R1 status is returned */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD56WR, startblock);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD56WR);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD56 failed: %d\n", ret);
          return ret;
        }
    }

  /* Configure SDIO controller hardware for the write transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, 1);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_BLOCK_WDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMASENDSETUP(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          finfo("SDIO_DMASENDSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_SENDSETUP(priv->dev, buffer, priv->blocksize);
    }

  /* If Controller needs DMA setup before write then only send CMD24 now. */

  if ((priv->caps & SDIO_CAPS_DMABEFOREWRITE) != 0)
    {
      /* Send CMD56, WRITE_BLOCK, and verify good R1 status is returned */

      mmcsd_sendcmdpoll(priv, MMCSD_CMD56WR, startblock);
      ret = mmcsd_recv_r1(priv, MMCSD_CMD56WR);
      if (ret != OK)
        {
          ferr("ERROR: mmcsd_recv_r1 for CMD56 failed: %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }

  /* Wait for the transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD56 transfer failed: %d\n", ret);
      return ret;
    }

  /* Flag that a write transfer is pending that we will have to check for
   * write complete at the beginning of the next transfer.
   */

  priv->wrbusy = true;

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  /* Arm the write complete detection with timeout */

  SDIO_WAITENABLE(priv->dev, SDIOWAIT_WRCOMPLETE | SDIOWAIT_TIMEOUT,
                  MMCSD_BLOCK_WDATADELAY);
#endif

  /* On success, return OK */

  return OK;
}

/****************************************************************************
 * Name: mmcsd_general_cmd_read
 *
 * Description:
 *   Read cmd56 data, one sector size
 *
 ****************************************************************************/

static int mmcsd_general_cmd_read(FAR struct mmcsd_state_s *priv,
                                  FAR uint8_t *buffer, off_t startblock)
{
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL);

  /* Check if the card is locked */

  if (priv->locked)
    {
      ferr("ERROR: Card is locked\n");
      return -EPERM;
    }

#if defined(CONFIG_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
  /* If we think we are going to perform a DMA transfer, make sure that we
   * will be able to before we commit the card to the operation.
   */

  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMAPREFLIGHT(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          return ret;
        }
    }
#endif

  /* Verify that the card is ready for the transfer.  The card may still be
   * busy from the preceding write transfer.  It would be simpler to check
   * for write busy at the end of each write, rather than at the beginning of
   * each read AND write, but putting the busy-wait at the beginning of the
   * transfer allows for more overlap and, hopefully, better performance
   */

  ret = mmcsd_transferready(priv);
  if (ret != OK)
    {
      ferr("ERROR: Card not ready: %d\n", ret);
      return ret;
    }

  /* Select the block size for the card */

  ret = mmcsd_setblocklen(priv, priv->blocksize);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_setblocklen failed: %d\n", ret);
      return ret;
    }

  /* Configure SDIO controller hardware for the read transfer */

  SDIO_BLOCKSETUP(priv->dev, priv->blocksize, 1);
  SDIO_WAITENABLE(priv->dev,
                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR,
                  MMCSD_BLOCK_RDATADELAY);

#ifdef CONFIG_SDIO_DMA
  if ((priv->caps & SDIO_CAPS_DMASUPPORTED) != 0)
    {
      ret = SDIO_DMARECVSETUP(priv->dev, buffer, priv->blocksize);
      if (ret != OK)
        {
          finfo("SDIO_DMARECVSETUP: error %d\n", ret);
          SDIO_CANCEL(priv->dev);
          return ret;
        }
    }
  else
#endif
    {
      SDIO_RECVSETUP(priv->dev, buffer, priv->blocksize);
    }

  /* Send CMD56: Read a sector size data and verify that good R1
   * status is returned.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD56RD, startblock);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD56RD);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD56 failed: %d\n", ret);
      SDIO_CANCEL(priv->dev);
      return ret;
    }

  /* Then wait for the data transfer to complete */

  ret = mmcsd_eventwait(priv, SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR);
  if (ret != OK)
    {
      ferr("ERROR: CMD56 transfer failed: %d\n", ret);
      return ret;
    }

  /* Return value:  OK */

  return OK;
}

#ifdef CONFIG_MMCSD_IOCSUPPORT
/****************************************************************************
 * Name: mmcsd_iocmd
 *
 * Description:
 *   MMCSD device ioctl commands.
 *
 ****************************************************************************/

static int mmcsd_iocmd(FAR struct mmcsd_state_s *priv,
                       FAR struct mmc_ioc_cmd *ic_ptr)
{
  int ret;

  DEBUGASSERT(priv != NULL && ic_ptr != NULL);

  if (!ic_ptr->is_acmd)
    {
      uint32_t opcode = ic_ptr->opcode & MMCSD_CMDIDX_MASK;
      switch (opcode)
        {
        case MMCSD_CMDIDX56: /* support general commands */
          {
            if (ic_ptr->write_flag)
              {
                ret = mmcsd_general_cmd_write(priv,
                      (FAR uint8_t *)(uintptr_t)(ic_ptr->data_ptr),
                      ic_ptr->arg);
                if (ret != OK)
                  {
                    ferr("mmcsd_iocmd MMCSD_CMDIDX56 write failed.\n");
                    return ret;
                  }
              }
            else
              {
                ret = mmcsd_general_cmd_read(priv,
                      (FAR uint8_t *)(uintptr_t)(ic_ptr->data_ptr),
                      ic_ptr->arg);
                if (ret != OK)
                  {
                    ferr("mmcsd_iocmd MMCSD_CMDIDX56 read failed.\n");
                    return ret;
                  }
              }
          }
          break;

        default:
          ferr("mmcsd_iocmd opcode unsupported.\n");
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mmcsd_multi_iocmd
 *
 * Description:
 *   MMCSD device ioctl multi commands.
 *
 ****************************************************************************/

static int mmcsd_multi_iocmd(FAR struct mmcsd_state_s *priv,
                             FAR struct mmc_ioc_multi_cmd *imc_ptr)
{
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && imc_ptr != NULL);

  if (imc_ptr->num_of_cmds > MMC_IOC_MAX_CMDS)
    {
      ferr("mmcsd_multi_iocmd too many cmds.\n");
      return -EINVAL;
    }

  for (i = 0; i < imc_ptr->num_of_cmds; ++i)
    {
      ret = mmcsd_iocmd(priv, &imc_ptr->cmds[i]);
      if (ret != OK)
        {
          ferr("cmds %d failed.\n", i);
          return ret;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: mmcsd_sdinitialize
 *
 * Description:
 *   We believe that there is an SD card in the slot.  Attempt to initialize
 *   and configure the SD card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

static int mmcsd_sdinitialize(FAR struct mmcsd_state_s *priv)
{
  uint32_t cid[4];
  uint32_t csd[4];
  uint32_t scr[2];
  int ret;

  /* At this point, clocking has been supplied to the card, both CMD0 and
   * ACMD41 (with OCR=0) have been sent successfully, the card is no longer
   * busy and (presumably) in the IDLE state so there is good evidence
   * that we have an SD card inserted into the slot.
   *
   * Send CMD2, ALL_SEND_CID.  The SD CMD2 is similar to the MMC CMD2 except
   * that the buffer type used to transmit to response of the card (SD Memory
   * Card: Push-Pull, MMC: Open-Drain). This implementation supports only a
   * single SD card.  If multiple cards were installed in the slot, each card
   * would respond to CMD2 by sending its CID (only one card completes the
   * response at a time).  The driver should send CMD2 and obtain RCAs until
   * no response to ALL_SEND_CID is received.
   *
   * When an SD card receives the CMD2 command it should transition to the
   * identification state/card-identification mode
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD2, 0);
  ret = SDIO_RECVR2(priv->dev, MMCSD_CMD2, cid);
  if (ret != OK)
    {
      ferr("ERROR: SDIO_RECVR2 for SD CID failed: %d\n", ret);
      return ret;
    }

  mmcsd_decode_cid(priv, cid);

  /* Send CMD3, SEND_RELATIVE_ADDR.  In both protocols, this command is used
   * to assign a logical address to the card.  For MMC, the host assigns the
   * address; for SD, the memory card has this responsibility. CMD3 causes
   * transition to standby state/data-transfer mode
   *
   * Send CMD3 with argument 0, SD card publishes its RCA in the response.
   */

  mmcsd_sendcmdpoll(priv, SD_CMD3, 0);
  ret = mmcsd_recv_r6(priv, SD_CMD3);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r6 for SD RCA failed: %d\n", ret);
      return ret;
    }

  finfo("RCA: %04x\n", priv->rca);

  /* This should have caused a transition to standby state. However, this
   * will not be reflected in the present R1/6 status.  R1/6 contains the
   * state of the card when the command was received, not when it
   * completed execution.
   *
   * Verify that we are in standby state/data-transfer mode
   */

  ret = mmcsd_verifystate(priv, MMCSD_R1_STATE_STBY);
  if (ret != OK)
    {
      ferr("ERROR: Failed to enter standby state\n");
      return ret;
    }

  /* Send CMD9, SEND_CSD, in standby state/data-transfer mode to obtain the
   * Card Specific Data (CSD) register.  The argument is the RCA that we
   * just obtained from CMD3.  The card stays in standby state/data-transfer
   * mode.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD9, (uint32_t)priv->rca << 16);
  ret = SDIO_RECVR2(priv->dev, MMCSD_CMD9, csd);
  if (ret != OK)
    {
      ferr("ERROR: Could not get SD CSD register(%d)\n", ret);
      return ret;
    }

  mmcsd_decode_csd(priv, csd);

  /* Send CMD7 with the argument == RCA in order to select the card.
   * Since we are supporting only a single card, we just leave the
   * card selected all of the time.
   */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD7S, (uint32_t)priv->rca << 16);
  ret = mmcsd_recv_r1(priv, MMCSD_CMD7S);
  if (ret != OK)
    {
      ferr("ERROR: mmcsd_recv_r1 for CMD7 failed: %d\n", ret);
      return ret;
    }

  /* Set the Driver Stage Register (DSR) if (1) a CONFIG_MMCSD_DSR has been
   * provided and (2) the card supports a DSR register.  If no DSR value
   * the card default value (0x0404) will be used.
   */

  mmcsd_sendcmd4(priv);

  /* Select high speed SD clocking (which may depend on the DSR setting) */

  SDIO_CLOCK(priv->dev, CLOCK_SD_TRANSFER_1BIT);
  nxsig_usleep(MMCSD_CLK_DELAY);

  /* If the hardware only supports 4-bit transfer mode then we forced to
   * attempt to setup the card in this mode before checking the SCR register.
   */

  if ((priv->caps & SDIO_CAPS_4BIT_ONLY) != 0)
    {
      /* Select width (4-bit) bus operation */

      priv->buswidth = 4;
      ret = mmcsd_widebus(priv);
      if (ret != OK)
        {
          ferr("ERROR: Failed to set wide bus operation: %d\n", ret);
        }
    }

  /* Get the SD card Configuration Register (SCR).  We need this now because
   * that configuration register contains the indication whether or not
   * this card supports wide bus operation.
   */

  ret = mmcsd_get_scr(priv, scr);
  if (ret != OK)
    {
      ferr("ERROR: Could not get SD SCR register(%d)\n", ret);
      return ret;
    }

  mmcsd_decode_scr(priv, scr);

  if ((priv->caps & SDIO_CAPS_4BIT_ONLY) == 0)
    {
      /* Select width (4-bit) bus operation (if the card supports it) */

      ret = mmcsd_widebus(priv);
      if (ret != OK)
        {
          ferr("ERROR: Failed to set wide bus operation: %d\n", ret);
        }
    }

  /* TODO: If wide-bus selected, then send CMD6 to see if the card supports
   * high speed mode.  A new SDIO method will be needed to set high speed
   * mode.
   */

  return OK;
}

/****************************************************************************
 * Name: mmcsd_cardidentify
 *
 * Description:
 *   We believe that there is media in the slot.  Attempt to initialize and
 *   configure the card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

static int mmcsd_cardidentify(FAR struct mmcsd_state_s *priv)
{
  uint32_t response;
  uint32_t sdcapacity = MMCSD_ACMD41_STDCAPACITY;
#ifdef CONFIG_MMCSD_MMCSUPPORT
  uint32_t mmccapacity = MMCSD_R3_HIGHCAPACITY;
#endif
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Assume failure to identify the card */

  priv->type = MMCSD_CARDTYPE_UNKNOWN;

  /* Check if there is a card present in the slot.  This is normally a
   * matter is of GPIO sensing.
   */

  if (!SDIO_PRESENT(priv->dev))
    {
      finfo("No card present\n");
      return -ENODEV;
    }

  /* Set ID mode clocking (<400KHz) */

  SDIO_CLOCK(priv->dev, CLOCK_IDMODE);

  /* After power up at least 74 clock cycles are required prior to starting
   * bus communication
   */

  up_udelay(MMCSD_POWERUP_DELAY);

  /* Then send CMD0 just once is standard procedure */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD0, 0);
  nxsig_usleep(MMCSD_IDLE_DELAY);

#ifdef CONFIG_MMCSD_MMCSUPPORT
  /* Send CMD1 which is supported only by MMC.  if there is valid response
   * then the card is definitely of MMC type
   */

  mmcsd_sendcmdpoll(priv, MMC_CMD1, MMCSD_VDD_33_34 | mmccapacity);
  ret = SDIO_RECVR3(priv->dev, MMC_CMD1, &response);

  /* Was the operating range set successfully */

  if (ret != OK)
    {
      ferr("ERROR: CMD1 RECVR3: %d\n", ret);

      /* CMD1 did not succeed, card is not MMC. This sleep let
       * the communication to recover before another send.
       */

      nxsig_usleep(MMCSD_IDLE_DELAY);
    }
  else
    {
      /* CMD1 succeeded... this must be an MMC card */

      finfo("MMC card detected\n");
      priv->type = MMCSD_CARDTYPE_MMC;

      /* Now, check if this is a MMC card/chip that supports block
       * addressing
       */

      if ((response & MMCSD_R3_HIGHCAPACITY) != 0)
        {
          finfo("MMC card/chip with block addressing\n");
          mmccapacity = MMCSD_R3_HIGHCAPACITY;
          priv->type |= MMCSD_CARDTYPE_BLOCK;
        }
      else
        {
          mmccapacity = MMCSD_R3_STDCAPACITY;
        }

      /* Check if the card is busy.  Very confusing, BUSY is set LOW
       * if the card has not finished its initialization, so it really
       * means NOT busy.
       */

      if ((response & MMCSD_CARD_BUSY) != 0)
        {
          /* NO.. We really should check the current state to see if the
           * MMC successfully made it to the IDLE state, but at least for
           * now, we will simply assume that that is the case.
           *
           * Then break out of the look with an MMC card identified
           */

          finfo("MMC card/chip ready!\n");
          return OK;
        }
    }

  if (!IS_MMC(priv->type))
#endif
    {
      /* Check for SDHC Version 2.x.  Send CMD8 to verify SD card interface
       * operating condition. CMD 8 is reserved on SD version 1.0 and MMC.
       *
       * CMD8 Argument:
       *    [31:12]: Reserved (shall be set to '0')
       *    [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
       *    [7:0]: Check Pattern (recommended 0xaa)
       * CMD8 Response: R7
       */

      ret = mmcsd_sendcmdpoll(priv, SD_CMD8,
                              MMCSD_CMD8CHECKPATTERN | MMCSD_CMD8VOLTAGE_27);
      if (ret == OK)
        {
          /* CMD8 was sent successfully... Get the R7 response */

          ret = SDIO_RECVR7(priv->dev, SD_CMD8, &response);
        }

      /* Were both the command sent and response received correctly? */

      if (ret == OK)
        {
          /* CMD8 succeeded this is probably a SDHC card. Verify the
           * operating voltage and that the check pattern was correctly
           * echoed
           */

          if (((response & MMCSD_R7VOLTAGE_MASK) == MMCSD_R7VOLTAGE_27) &&
              ((response & MMCSD_R7ECHO_MASK) ==  MMCSD_R7CHECKPATTERN))
            {
              finfo("SD V2.x card\n");
              priv->type = MMCSD_CARDTYPE_SDV2;
              sdcapacity = MMCSD_ACMD41_HIGHCAPACITY;
            }
          else
            {
              ferr("ERROR: R7: %08" PRIx32 "\n", response);
              return -EIO;
            }
        }
    }

  /* At this point, type is either UNKNOWN or SDV2.  Try sending
   * CMD55 and (maybe) ACMD41 for up to 1 second or until the card
   * exits the IDLE state.  CMD55 is supported by SD V1.x and SD V2.x,
   * but not MMC
   */

  start   = clock_systime_ticks();
  elapsed = 0;
  do
    {
      /* We may have already determined that his card is an MMC card from
       * an earlier pass through this loop.  In that case, we should
       * skip the SD-specific commands.
       */
#ifdef CONFIG_MMCSD_MMCSUPPORT
      if (!IS_MMC(priv->type))
#endif
        {
          /* Send CMD55 with argument = 0 */

          mmcsd_sendcmdpoll(priv, SD_CMD55, 0);
          ret = mmcsd_recv_r1(priv, SD_CMD55);
          if (ret != OK)
            {
              /* I am a little confused.. I think both SD and MMC cards
               * support CMD55 (but maybe only SD cards support CMD55).
               * We'll make the the MMC vs. SD decision based on CMD1 and
               * ACMD41.
               */

              ferr("ERROR: mmcsd_recv_r1(CMD55) failed: %d\n", ret);
            }
          else
            {
              /* Send ACMD41 */

              mmcsd_sendcmdpoll(priv, SD_ACMD41,
                                MMCSD_ACMD41_VOLTAGEWINDOW_33_32 |
                                sdcapacity);
              ret = SDIO_RECVR3(priv->dev, SD_ACMD41, &response);
              if (ret != OK)
                {
                  /* If the error is a timeout, then it is probably an MMC
                   * card, but we will make the decision based on CMD1
                   * below.
                   */

                  ferr("ERROR: ACMD41 RECVR3: %d\n", ret);
                }
              else
                {
                  /* ACMD41 succeeded.  ACMD41 is supported by SD V1.x and
                   * SD V2.x, but not MMC.  If we did not previously
                   * determine that this is an SD V2.x (via CMD8), then this
                   * must be SD V1.x
                   */

                  finfo("R3: %08" PRIx32 "\n", response);
                  if (priv->type == MMCSD_CARDTYPE_UNKNOWN)
                    {
                      finfo("SD V1.x card\n");
                      priv->type = MMCSD_CARDTYPE_SDV1;
                    }

                  /* Check if the card is busy.  Very confusing, BUSY is set
                   * LOW if the card has not finished its initialization,
                   * so it really means NOT busy.
                   */

                  if ((response & MMCSD_CARD_BUSY) != 0)
                    {
                      /* No.. We really should check the current state to
                       * see if the SD card successfully made it to the IDLE
                       * state, but at least for now, we will simply assume
                       * that that is the case.
                       *
                       * Now, check if this is a SD V2.x card that supports
                       * block addressing
                       */

                      if ((response & MMCSD_R3_HIGHCAPACITY) != 0)
                        {
                          finfo("SD V2.x card with block addressing\n");
                          DEBUGASSERT(priv->type == MMCSD_CARDTYPE_SDV2);
                          priv->type |= MMCSD_CARDTYPE_BLOCK;
                        }

                      /* And break out of the loop with an card identified */

                      break;
                    }
                }
            }
        }

      /* If we get here then either (1) CMD55 failed, (2) CMD41 failed, or
       * (3) and SD or MMC card has been identified, but it is not yet in
       * the IDLE state.  If SD card has not been identified, then we might
       * be looking at an MMC card.  We can send the CMD1 to find out for
       * sure.  CMD1 is supported by MMC cards, but not by SD cards.
       */

#ifdef CONFIG_MMCSD_MMCSUPPORT
      if (IS_MMC(priv->type))
        {
          /* Send the MMC CMD1 to specify the operating voltage. CMD1 causes
           * transition to ready state/ card-identification mode.  NOTE: If
           * the card does not support this voltage range, it will go the
           * inactive state.
           *
           * NOTE: An MMC card will only respond once to CMD1 (unless it is
           * busy).  This is part of the logic used to determine how  many
           * MMC cards are connected (This implementation supports only a
           * single MMC card).  So we cannot re-send CMD1 without first
           * placing the card back into stand-by state (if the card is busy,
           * it will automatically go back to the standby state).
           */

          mmcsd_sendcmdpoll(priv, MMC_CMD1, MMCSD_VDD_33_34 | mmccapacity);
          ret = SDIO_RECVR3(priv->dev, MMC_CMD1, &response);

          /* Was the operating range set successfully */

          if (ret != OK)
            {
              ferr("ERROR: CMD1 RECVR3: %d\n", ret);
            }
          else
            {
              /* CMD1 succeeded... this must be an MMC card */

              priv->type = MMCSD_CARDTYPE_MMC;

              /* Now, check if this is a MMC card/chip that supports block
               * addressing
               */

              if ((response & MMCSD_R3_HIGHCAPACITY) != 0)
                {
                  mmccapacity = MMCSD_R3_HIGHCAPACITY;
                  priv->type |= MMCSD_CARDTYPE_BLOCK;
                }
              else
                {
                  mmccapacity = MMCSD_R3_STDCAPACITY;
                }

              /* Check if the card is busy.  Very confusing, BUSY is set LOW
               * if the card has not finished its initialization, so it
               * really means NOT busy.
               */

              if ((response & MMCSD_CARD_BUSY) != 0)
                {
                  /* NO.. We really should check the current state to see if
                   * the MMC successfully made it to the IDLE state, but at
                   * least for now we will simply assume that that is the
                   * case.
                   *
                   * Then break out of the look with an MMC card identified
                   */

                  finfo("MMC card/chip ready!\n");
                  break;
                }
            }
        }
#endif

      /* Check the elapsed time.  We won't keep trying this forever! */

      elapsed = clock_systime_ticks() - start;
    }
  while (elapsed < TICK_PER_SEC); /* On successful reception while 'breaks', see above. */

  /* We get here when the above loop completes, either (1) we could not
   * communicate properly with the card due to errors (and the loop times
   * out), or (2) it is an MMC or SD card that has successfully transitioned
   * to the IDLE state (well, at least, it provided its OCR saying that it
   * it is no longer busy).
   */

  if (elapsed >= TICK_PER_SEC || priv->type == MMCSD_CARDTYPE_UNKNOWN)
    {
      priv->type = MMCSD_CARDTYPE_UNKNOWN;
      ferr("ERROR: Failed to identify card\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: mmcsd_probe
 *
 * Description:
 *   Check for media inserted in a slot.  Called (1) during initialization to
 *   see if there was a card in the slot at power up, (2) when/if a media
 *   insertion event occurs, or (3) if the BIOC_PROBE ioctl command is
 *   received.
 *
 ****************************************************************************/

static int mmcsd_probe(FAR struct mmcsd_state_s *priv)
{
  int ret;

  finfo("type: %d probed: %d\n", priv->type, priv->probed);

  /* If we have reliable card detection events and if we have
   * already probed the card, then we don't need to do anything
   * else
   */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if (priv->probed && SDIO_PRESENT(priv->dev))
    {
      return OK;
    }
#endif

  /* Otherwise, we are going to probe the card.  There are lots of
   * possibilities here:  We may think that there is a card in the slot,
   * or not.  There may be a card in the slot, or not.  If there is
   * card in the slot, perhaps it is a different card than we one we
   * think is there?  The safest thing to do is to process the card
   * removal first and start from known place.
   */

  mmcsd_removed(priv);

  /* Now.. is there a card in the slot? */

  if (SDIO_PRESENT(priv->dev))
    {
      /* Yes.. probe it.  First, what kind of card was inserted? */

      ret = mmcsd_cardidentify(priv);
      if (ret != OK)
        {
          ferr("ERROR: Failed to initialize card: %d\n", ret);
        }
      else
        {
          /* Then initialize the driver according to the card type */

          switch (priv->type)
            {
              /* Bit 1: SD version 1.x */

              case MMCSD_CARDTYPE_SDV1:

              /* SD version 2.x with byte addressing */

              case MMCSD_CARDTYPE_SDV2:

              /* SD version 2.x with block addressing */

              case MMCSD_CARDTYPE_SDV2 | MMCSD_CARDTYPE_BLOCK:
                ret = mmcsd_sdinitialize(priv);
                break;

              /* MMC card with byte addressing */

              case MMCSD_CARDTYPE_MMC:

              /* MMC card with block addressing */

              case MMCSD_CARDTYPE_MMC | MMCSD_CARDTYPE_BLOCK:
#ifdef CONFIG_MMCSD_MMCSUPPORT
                ret = mmcsd_mmcinitialize(priv);
                break;
#endif
              /* Unknown card type */

              case MMCSD_CARDTYPE_UNKNOWN:
              default:
                ferr("ERROR: Internal confusion: %d\n", priv->type);
                ret = -EPERM;
                break;
            }

          /* Was the card configured successfully? */

          if (ret == OK)
            {
              /* Yes...  */

              finfo("Capacity: %" PRIu32 " Kbytes\n",
                    MMCSD_CAPACITY(priv->nblocks, priv->blockshift));
              priv->mediachanged = true;
            }

          /* When the card is identified, we have probed this card */

          priv->probed = true;
        }

      /* Regardless of whether or not a card was successfully initialized,
       * there is apparently a card inserted. If it wasn't successfully
       * initialized, there's nothing we can do about it now. Perhaps it's
       * a bad card? The best we can do is wait for the card to be ejected
       * and re-inserted. Then we can try to initialize again.
       */

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      /* Set up to receive asynchronous, media removal events */

      SDIO_CALLBACKENABLE(priv->dev, SDIOMEDIA_EJECTED);
#endif
    }
  else
    {
      /* There is no card in the slot */

      finfo("No card\n");
#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      SDIO_CALLBACKENABLE(priv->dev, SDIOMEDIA_INSERTED);
#endif
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: mmcsd_removed
 *
 * Description:
 *   Disable support for media in the slot.  Called (1) when/if a media
 *   removal event occurs, or (2) if the BIOC_EJECT ioctl command is
 *   received.
 *
 ****************************************************************************/

static int mmcsd_removed(FAR struct mmcsd_state_s *priv)
{
  finfo("type: %d present: %d\n", priv->type, SDIO_PRESENT(priv->dev));

  /* Forget the card geometry, pretend the slot is empty (it might not
   * be), and that the card has never been initialized.
   */

  priv->blocksize    = 0;
  priv->probed       = false;
  priv->mediachanged = false;
  priv->wrbusy       = false;
  priv->type         = MMCSD_CARDTYPE_UNKNOWN;
  priv->rca          = 0;
  priv->selblocklen  = 0;

  /* Go back to the default 1-bit data bus. */

  SDIO_WIDEBUS(priv->dev, false);
  priv->widebus      = false;

  /* Disable clocking to the card */

  SDIO_CLOCK(priv->dev, CLOCK_SDIO_DISABLED);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_hwinitialize
 *
 * Description:
 *   One-time hardware initialization.  Called only from sdio_slotinitialize.
 *
 ****************************************************************************/

static int mmcsd_hwinitialize(FAR struct mmcsd_state_s *priv)
{
  int ret;

  ret = mmcsd_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the capabilities of the SDIO driver */

  priv->caps = SDIO_CAPABILITIES(priv->dev);
  finfo("DMA supported: %d\n", (priv->caps & SDIO_CAPS_DMASUPPORTED) != 0);

  /* Attach and prepare MMC/SD interrupts */

  if (SDIO_ATTACH(priv->dev))
    {
      ferr("ERROR: Unable to attach MMC/SD interrupts\n");
      mmcsd_unlock(priv);
      return -EBUSY;
    }

  finfo("Attached MMC/SD interrupts\n");

  /* Register a callback so that we get informed if media is inserted or
   * removed from the slot (Initially all callbacks are disabled).
   */

  SDIO_REGISTERCALLBACK(priv->dev, mmcsd_mediachange, (FAR void *)priv);

  /* Is there a card in the slot now? For an MMC/SD card, there are three
   * possible card detect mechanisms:
   *
   *  1. Mechanical insertion that can be detected using the WP switch
   *     that is closed when a card is inserted into then SD slot (SD
   *     "hot insertion capable" card connector only)
   *  2. Electrical insertion that can be sensed using the pull-up resistor
   *     on CD/DAT3 (both SD/MMC),
   *  3. Or by periodic attempts to initialize the card from software.
   *
   * The behavior of SDIO_PRESENT() is to use whatever information is
   * available on the particular platform.  If no card insertion information
   * is available (polling only), then SDIO_PRESENT() will always return
   * true and we will try to initialize the card.
   */

  if (SDIO_PRESENT(priv->dev))
    {
      /* Yes... probe for a card in the slot */

      ret = mmcsd_probe(priv);
      if (ret != OK)
        {
          finfo("Slot not empty, but initialization failed: %d\n", ret);

          /* NOTE: The failure to initialize a card does not mean that
           * initialization has failed! A card could be installed in the slot
           * at a later time. ENODEV is return in this case,
           * sdio_slotinitialize will use this return value to set up the
           * card inserted callback event.
           */

          ret = -ENODEV;
        }
    }
  else
    {
      /* ENODEV is returned to indicate that no card is inserted in the slot.
       * sdio_slotinitialize will use this return value to set up the card
       * inserted callback event.
       */

      ret = -ENODEV;
    }

  /* OK is returned only if the slot initialized correctly AND the card in
   * the slot was successfully configured.
   */

  mmcsd_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: mmcsd_hwuninitialize
 *
 * Description:
 *   Restore the MMC/SD slot to the uninitialized state.  Called only from
 *   sdio_slotinitialize on a failure to initialize.
 *
 ****************************************************************************/

static void mmcsd_hwuninitialize(FAR struct mmcsd_state_s *priv)
{
  mmcsd_removed(priv);
  SDIO_RESET(priv->dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

int mmcsd_slotinitialize(int minor, FAR struct sdio_dev_s *dev)
{
  FAR struct mmcsd_state_s *priv;
  char devname[16];
  int ret = -ENOMEM;

  finfo("minor: %d\n", minor);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (minor < 0 || minor > 255 || !dev)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a MMC/SD state structure */

  priv = (FAR struct mmcsd_state_s *)
    kmm_malloc(sizeof(struct mmcsd_state_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the MMC/SD state structure */

  memset(priv, 0, sizeof(struct mmcsd_state_s));
  nxmutex_init(&priv->lock);

  /* Bind the MMCSD driver to the MMCSD state structure */

  priv->dev = dev;

  /* Initialize the hardware associated with the slot */

  ret = mmcsd_hwinitialize(priv);

  /* Was the slot initialized successfully? */

  if (ret != OK)
    {
      /* No... But the error ENODEV is returned if hardware
       * initialization succeeded but no card is inserted in the slot.
       * In this case, the no error occurred, but the driver is still
       * not ready.
       */

      if (ret == -ENODEV)
        {
          /* No card in the slot (or if there is, we could not recognize
           * it).. Setup to receive the media inserted event
           */

          SDIO_CALLBACKENABLE(priv->dev, SDIOMEDIA_INSERTED);

          finfo("MMC/SD slot is empty\n");
        }
      else
        {
          /* Some other non-recoverable bad thing happened */

          ferr("ERROR: Failed to initialize MMC/SD slot: %d\n", ret);
          goto errout_with_alloc;
        }
    }

  /* Create a MMCSD device name */

  snprintf(devname, 16, "/dev/mmcsd%d", minor);

  /* Inode private data is a reference to the MMCSD state structure */

  ret = register_blockdriver(devname, &g_bops, 0, priv);
  if (ret < 0)
    {
      ferr("ERROR: register_blockdriver failed: %d\n", ret);
      goto errout_with_hwinit;
    }

  return OK;

errout_with_hwinit:
  mmcsd_hwuninitialize(priv);
errout_with_alloc:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SDIO) */
