/****************************************************************************
 * drivers/mmcsd/mmcsd_sdio.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs.h>
#include <nuttx/ioctl.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/rwbuffer.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "mmcsd_internal.h"
#include "mmcsd_sdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum number of references on the driver (because a ubyte is used.
 * Use a larger type if more references are needed.
 */

#define MAX_CREFS               0xff

/* Timing */

#define MMCSD_POWERUP_DELAY     250     /* 74 clock cycles @ 400KHz = 185uS */
#define MMCSD_IDLE_DELAY        50      /* Short delay to allow change to IDLE state */

#define IS_EMPTY(priv) (priv->type == MMCSD_CARDTYPE_UNKNOWN)

/* Transfer mode */

#define MMCSDMODE_POLLED        0
#define MMCSDMODE_INTERRUPT     1
#define MMCSDMODE_DMA           2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is contains the unique state of the MMC/SD block driver */

struct mmcsd_state_s
{
  struct sdio_dev_s *dev;          /* The SDIO device bound to this instance */
  ubyte  crefs;                    /* Open references on the driver */

  /* Status flags */

  ubyte probed:1;                  /* TRUE: mmcsd_probe() discovered a card */
  ubyte widebus:1;                 /* TRUE: Wide 4-bit bus selected */
  ubyte mediachanged:1;            /* TRUE: Media changed since last check */
  ubyte wrprotect:1;               /* TRUE: Media is write protected */
  ubyte selected:1;                /* TRUE: card is selected */
#ifdef CONFIG_SDIO_DMA
  ubyte dma:1;                     /* TRUE: hardware supports DMA */
#endif

  ubyte type;                      /* Card type (See MMCSD_CARDTYPE_* definitions) */
  ubyte mode;                      /* (See MMCSDMODE_* definitions) */
  uint16 selblocklen;              /* The currently selected block length */
  uint16 rca;                      /* Relative Card Address (RCS) register */

  /* Memory card geometry (extracted from the CSD) */

  uint16 rdblocklen;               /* Read block length (== block size) */
  uint16 wrblocklen;               /* Write block length */
  size_t nblocks;                  /* Number of blocks */
  size_t capacity;                 /* Total capacity of volume */

  /* Read-ahead and write buffering support */

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)
  struct rwbuffer_s rwbuffer;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Command/response helpers *************************************************/

static int     mmcsd_sendcmdpoll(struct mmcsd_state_s *priv, uint32 cmd, uint32 arg);

/* Transfer helpers *********************************************************/

static ssize_t mmcsd_doread(FAR void *dev, FAR ubyte *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t mmcsd_dowrite(FAR void *dev, FAR const ubyte *buffer,
                 off_t startblock, size_t nblocks);

/* Block driver methods *****************************************************/

static int     mmcsd_open(FAR struct inode *inode);
static int     mmcsd_close(FAR struct inode *inode);
static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer,
                 size_t start_sector, unsigned int nsectors);
#endif
static int     mmcsd_geometry(FAR struct inode *inode,
                 struct geometry *geometry);
static int     mmcsd_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

/* Initialization/uninitialization/reset ************************************/

static inline int
               mmcsd_mmcinitialize(struct mmcsd_state_s *priv);
static inline int
               mmcsd_sdinitialize(struct mmcsd_state_s *priv);
static inline int
               mmcsd_cardidentify(struct mmcsd_state_s *priv);
static int     mmcsd_probe(struct mmcsd_state_s *priv);
static int     mmcsd_removed(struct mmcsd_state_s *priv);
static int     mmcsd_hwinitialize(struct mmcsd_state_s *priv);
static inline void
               mmcsd_hwuninitialize(struct mmcsd_state_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  mmcsd_open,     /* open     */
  mmcsd_close,    /* close    */
  mmcsd_read,     /* read     */
#ifdef CONFIG_FS_WRITABLE
  mmcsd_write,    /* write    */
#else
  NULL,           /* write    */
#endif
  mmcsd_geometry, /* geometry */
  mmcsd_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Command/Response Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_sendcmdpoll
 *
 * Description:
 *   send a command and poll-wait for the response.
 *
 ****************************************************************************/

static int mmcsd_sendcmdpoll(struct mmcsd_state_s *priv, uint32 cmd, uint32 arg)
{
  int ret;

  /* Send the command */

  SDIO_SENDCMD(priv->dev, cmd, arg);

  /* Then poll-wait until the response is available */

  ret = SDIO_WAITRESPONSE(priv->dev, cmd);
  if (ret != OK)
    {
      fdbg("ERROR: Wait for response to cmd=%08x failed: %d\n", cmd, ret);
    }
  return ret;
}

/****************************************************************************
 * Transfer Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: mmcsd_doread
 *
 * Description:
 *   Read the specified numer of sectors from the physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_doread(FAR void *dev, FAR ubyte *buffer,
                            off_t startblock, size_t nblocks)
{
  struct mmcsd_state_s *priv = (struct mmcsd_state_s *)dev;
#ifdef CONFIG_CPP_HAVE_WARNING
#  warning "Not implemented"
#endif
  return -ENOSYS;
}

/****************************************************************************
 * Name: mmcsd_dowrite
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t mmcsd_dowrite(FAR void *dev, FAR const ubyte *buffer,
                             off_t startblock, size_t nblocks)
{
  struct mmcsd_state_s *priv = (struct mmcsd_state_s *)dev;
#ifdef CONFIG_CPP_HAVE_WARNING
#  warning "Not implemented"
#endif
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Block Driver Methods
 ****************************************************************************/
/****************************************************************************
 * Name: mmcsd_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mmcsd_open(FAR struct inode *inode)
{
  struct mmcsd_state_s *priv;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (struct mmcsd_state_s *)inode->i_private;

  /* Just increment the reference count on the driver */

  DEBUGASSERT(priv->crefs < MAX_CREFS);
  priv->crefs++;
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
  struct mmcsd_state_s *priv;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (struct mmcsd_state_s *)inode->i_private;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;
  return OK;
}

/****************************************************************************
 * Name: mmcsd_read
 *
 * Description:
 *   Read the specified numer of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_read(FAR struct inode *inode, unsigned char *buffer,
                          size_t start_sector, unsigned int nsectors)
{
  struct mmcsd_state_s *priv;

  fvdbg("sector: %d nsectors: %d sectorsize: %d\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (struct mmcsd_state_s *)inode->i_private;

#ifdef CONFIG_FS_READAHEAD
  return rwb_read(&priv->rwbuffer, start_sector, nsectors, buffer);
#else
  return mmcsd_doread(priv, buffer, start_sector, nsectors);
#endif
}

/****************************************************************************
 * Name: mmcsd_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer,
                           size_t start_sector, unsigned int nsectors)
{
  struct mmcsd_state_s *priv;

  fvdbg("sector: %d nsectors: %d sectorsize: %d\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (struct mmcsd_state_s *)inode->i_private;

#ifdef CONFIG_FS_WRITEBUFFER
  return rwb_write(&priv->rwbuffer, start_sector, nsectors, buffer);
#else
  return mmcsd_dowrite(priv, buffer, start_sector, nsectors);
#endif
}
#endif

/****************************************************************************
 * Name: mmcsd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  struct mmcsd_state_s *priv;
  int ret = -EINVAL;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  if (geometry)
    {
      /* Is there a (supported) card inserted in the slot? */

      priv = (struct mmcsd_state_s *)inode->i_private;
      if (IS_EMPTY(priv))
        {
          /* No.. return ENODEV */

          fvdbg("IS_EMPTY\n");
          ret = -ENODEV;
        }
      else
        {
          /* Yes.. return the geometry of the card */

          geometry->geo_available     = TRUE;
          geometry->geo_mediachanged  = priv->mediachanged;
#ifdef CONFIG_FS_WRITABLE
          geometry->geo_writeenabled  = !priv->wrprotect;
#else
          geometry->geo_writeenabled  = FALSE;
#endif
          geometry->geo_nsectors      = priv->nblocks;
          geometry->geo_sectorsize    = priv->rdblocklen;

          fvdbg("available: TRUE mediachanged: %s writeenabled: %s\n",
                 geometry->geo_mediachanged ? "TRUE" : "FALSE",
                 geometry->geo_writeenabled ? "TRUE" : "FALSE");
          fvdbg("nsectors: %ld sectorsize: %d\n",
                 (long)geometry->geo_nsectors, geometry->geo_sectorsize);

          priv->mediachanged = FALSE;
          ret = OK;
        }
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
  struct mmcsd_state_s *priv;
  int ret;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct mmcsd_state_s *)inode->i_private;

  /* Process the IOCTL by command */

  switch (cmd)
    {
    case BIOC_PROBE: /* Check for media in the slot */
      {
        fvdbg("BIOC_PROBE\n");

        /* Probe the MMC/SD slot for media */

        ret = mmcsd_probe(priv);
        if (ret != OK)
          {
            fdbg("ERROR: mmcsd_probe failed: %d\n", ret);
          }
      }
      break;

    case BIOC_EJECT: /* Media has been removed from the slot */
      {
        fvdbg("BIOC_EJECT\n");

        /* Process the removal of the card */

        ret = mmcsd_removed(priv);
        if (ret != OK)
          {
            fdbg("ERROR: mmcsd_removed failed: %d\n", ret);
          }
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_mmcinitialize
 *
 * Description:
 *   We believe that there is an MMC card in the slot.  Attempt to initialize
 *   and configure the MMC card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

static inline int mmcsd_mmcinitialize(struct mmcsd_state_s *priv)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mmcsd_sdinitialize
 *
 * Description:
 *   We believe that there is an SD card in the slot.  Attempt to initialize
 *   and configure the SD card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

static inline int mmcsd_sdinitialize(struct mmcsd_state_s *priv)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mmcsd_cardidentify
 *
 * Description:
 *   We believe that there is media in the slot.  Attempt to initialize and
 *   configure the card.  This is called only from mmcsd_probe().
 *
 ****************************************************************************/

static inline int mmcsd_cardidentify(struct mmcsd_state_s *priv)
{
  uint32  response;
  uint32  start;
  uint32  elapsed;
  uint32  state;
  uint32  sdcapacity = MMCD_ACMD41_STDCAPACITY;
  boolean cardbusy;
  int     ret;

  /* Assume failure to identify the card */

  priv->type = MMCSD_CARDTYPE_UNKNOWN;
  priv->mode = MMCSDMODE_POLLED;

  /* Check if there is a card present in the slot.  This is normally a matter is
   * of GPIO sensing.
   */

  if (SDIO_PRESENT(priv->dev))
    {
      fvdbg("No card present\n");
      return -ENODEV;
    }

  /* Initialize device state structure */

  priv->type = MMCSD_CARDTYPE_SDV1;
  priv->mode = MMCSDMODE_POLLED;

  /* Set ID mode clocking (<400KHz) */

  SDIO_CLOCK(priv->dev, CLOCK_IDMODE);

  /* After power up at least 74 clock cycles are required prior to starting bus
   * communication
   */

  up_udelay(MMCSD_POWERUP_DELAY);

  /* Then send CMD0 (twice just to be sure) */

  mmcsd_sendcmdpoll(priv, MMCSD_CMD0, 0);
  mmcsd_sendcmdpoll(priv, MMCSD_CMD0, 0);
  up_udelay(MMCSD_IDLE_DELAY);

  /* Check for SDHC Version 2.x.  Send CMD8 to verify SD card interface
   * operating condition. CMD 8 is reserved on SD version 1.0 and MMC.
   *
   * CMD8 Argument:
   *    [31:12]: Reserved (shall be set to '0')   *    [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
   *    [7:0]: Check Pattern (recommended 0xaa)
   * CMD8 Response: R7
   */

  mmcsd_sendcmdpoll(priv, SD_CMD8, MMCSD_CMD8CHECKPATTERN|MMCSD_CMD8VOLTAGE_27);
  ret = SDIO_RECVR7(priv->dev, SD_CMD8, &response);
  if (ret == OK)
  {
    /* CMD8 succeeded this is probably a SDHC card. Verify the operating
     * voltage and that the check pattern was correctly echoed
     */

    if (((response & MMCSD_R7VOLTAGE_MASK) == MMCSD_R7VOLTAGE_27) &&
        ((response & MMCSD_R7ECHO_MASK) ==  MMCSD_R7CHECKPATTERN))
      {
        fvdbg("SD V2.x card\n");
        priv->type = MMCSD_CARDTYPE_SDV2;
        sdcapacity = MMCD_ACMD41_HIGHCAPACITY;
      }
    else
      {
        fdbg("ERROR: R7: %08x\n", r7);
        return -EIO;
      }
  }

  /* At this point, type is either UNKNOWN or SDV2.  Try sending
   * CMD55 and (maybe) ACMD41 for up to 1 second or until the card
   * exits the IDLE state.  CMD55 is supported by SD V1.x and SD V2.x,
   * but not MMC
   */

  start   = g_system_timer;
  elapsed = 0;
  do
    {
      /* Send CMD55 */

      mmcsd_sendcmdpoll(priv, SD_CMD55, 0);
      ret = SDIO_RECVR1(priv->dev, SD_CMD55, &response);
      if (ret != OK)
        {
          /* If the error is a timeout, then it is an MMC card */

          if (ret == -ETIMEDOUT)
            {
              fvdbg("ERROR: CMD55 timeout, assuming MMC card\n");
              priv->type = MMCSD_CARDTYPE_MMC;
              break;        
            }
          else
            {
              fdbg("ERROR: CMD55 RECVR1: %d\n", ret);
              return -EIO;
            }
        }
      else
        {
          /* CMD55 succeeded.  CMD55 is supported by SD V1.x and SD V2.x,
           * but not MMC.  If we did not previoulsy determine that this is
           * an SD V2.x (via CMD8), then this must be SD V1.x
           */

          if (priv->type == MMCSD_CARDTYPE_UNKNOWN)
            {
              fvdbg("SD V1.x card\n");
              priv->type = MMCSD_CARDTYPE_SDV1;
            }

          /* Save the state from CMD55 R1 response */

          state = response & MMCSD_R1_STATE_MASK;
          fvdbg("CMD55 R1: %08x\n", response);

          /* Send ACMD41 */

          mmcsd_sendcmdpoll(priv, SD_ACMD41, MMCD_ACMD41_VOLTAGEWINDOW|sdcapacity);
          ret = SDIO_RECVR3(priv->dev, SD_CMD55, &response);
          if (ret != OK)
            {
              fdbg("ERROR: ACMD41 RECVR1: %d\n", ret);
              return -EIO;
            }
          else
            {
              /* Check if the card is busy */

              cardbusy = ((response >> 31) == 1);
              if (!cardbusy)
                {
                  /* Check if this is a SD V2.x card that supports block addressing */

                  if ((response & MMCD_R3_HIGHCAPACITY) != 0)
                    {
                      fvdbg("SD V2.x card with block addressing\n");
                      DEBUGASSERT(priv->type == MMCSD_CARDTYPE_SDV2);
                      priv->type |= MMCSD_CARDTYPE_BLOCK;
                    }
                }
            }
        }
      elapsed = g_system_timer - start;
    }
  while (elapsed < TICK_PER_SEC && (ret != OK || state != MMCSD_R1_STATE_IDLE || cardbusy));

  /* We get here when the above loop completes, either (1) because this is 
   * an MMC card, (2) we could not communicate properly with the card due to
   * errors (and the loop times out), or (3) it is an SD card that has successfully
   * transitioned to the IDLE state.
   */

  if (elapsed >= TICK_PER_SEC || priv->type == MMCSD_CARDTYPE_UNKNOWN)
    {
      fdbg("ERROR: Failed to identify card\n");
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

static int mmcsd_probe(struct mmcsd_state_s *priv)
{
  int ret;

  fvdbg("type=%d probed=%d\n", priv->type, priv->probed);

  /* If we have reliable card detection events and if we have
   * already probed the card, then we don't need to do anything
   * else
   */

#ifdef CONFIG_MMCSD_HAVECARDDETECT
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
          fdbg("ERROR: Failed to initialize card: %d\n");
          SDIO_EVENTENABLE(priv->dev, SDIOEVENT_INSERTED);
        }
      else
        {
          /* Then initialize the driver according to the identified card type */

          switch (priv->type)
            {
            case MMCSD_CARDTYPE_MMC:                       /* MMC card */
              ret = mmcsd_mmcinitialize(priv);

            case MMCSD_CARDTYPE_SDV1:                      /* Bit 1: SD version 1.x */
            case MMCSD_CARDTYPE_SDV2:                      /* SD version 2.x with byte addressing */
            case MMCSD_CARDTYPE_SDV2|MMCSD_CARDTYPE_BLOCK: /* SD version 2.x with block addressing */
              ret = mmcsd_sdinitialize(priv);

            case MMCSD_CARDTYPE_UNKNOWN:                   /* Unknown card type */
            default:
              fdbg("ERROR: Internal confusion: %d\n", priv->type);
              ret = -EPERM;
            };

            /* Was the card configured successfully? */

            if (ret == OK)
              {
                /* Yes...  */

                fvdbg("Capacity: %d Kbytes\n", priv->capacity / 1024);
                priv->mediachanged = TRUE;

                /* Set up to receive asynchronous, media removal events */

                SDIO_EVENTENABLE(priv->dev, SDIOEVENT_EJECTED);
              }
        }

      /* In any event, we have probed this card */

      priv->probed = TRUE;
    }
  else
    {
      /* There is no card in the slot */

      fvdbg("No card\n");
      SDIO_EVENTENABLE(priv->dev, SDIOEVENT_INSERTED);
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

static int mmcsd_removed(struct mmcsd_state_s *priv)
{
  fvdbg("type: %d present: %d\n", priv->type, SDIO_PRESENT(priv->dev));

  /* Forget the card geometry, pretend the slot is empty (it might not
   * be), and that the card has never been initialized.
   */

  priv->capacity    = 0; /* Capacity=0 sometimes means no media */
  priv->rdblocklen  = 0;
  priv->wrblocklen  = 0;
  priv->mediachanged = FALSE;
  priv->type        = MMCSD_CARDTYPE_UNKNOWN;
  priv->probed      = FALSE;
  priv->selected    = FALSE;
  priv->rca         = 0;
  priv->selblocklen = 0;

  /* Go back to the default 1-bit data bus. */

  SDIO_WIDEBUS(priv->dev, FALSE);
  priv->widebus     = FALSE;

  /* Disable clocking to the card */

  (void)SDIO_CLOCK(priv->dev, CLOCK_SDIO_DISABLED);

  /* Enable logic to detect if a card is re-inserted */

  SDIO_EVENTENABLE(priv->dev, SDIOEVENT_INSERTED);
  return OK;
}

/****************************************************************************
 * Name: mmcsd_hwinitialize
 *
 * Description:
 *   One-time hardware initialization
 *
 ****************************************************************************/

static int mmcsd_hwinitialize(struct mmcsd_state_s *priv)
{
#ifdef CONFIG_CPP_HAVE_WARNING
#  warning "Not implemented"
#endif
  return -ENODEV;
}

/****************************************************************************
 * Name: mmcsd_hwinitialize
 *
 * Description:
 *   Restore the MMC/SD slot to the uninitialized state
 *
 ****************************************************************************/

static inline void mmcsd_hwuninitialize(struct mmcsd_state_s *priv)
{
  if (priv)
    {
      mmcsd_removed(priv);
      SDIO_RESET(priv->dev);
      free(priv);
    }
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
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

int mmcsd_slotinitialize(int minor, int slotno, FAR struct sdio_dev_s *dev)
{
  struct mmcsd_state_s *priv;
  char devname[16];
  int ret = -ENOMEM;

  fvdbg("minor: %d slotno: %d\n", minor, slotno);

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if ((unsigned)slotno >= CONFIG_MMCSD_NSLOTS || minor < 0 || minor > 255 || !dev)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a MMC/SD state structure */

  priv = (struct mmcsd_state_s *)malloc(sizeof(struct mmcsd_state_s));
  if (priv)
    {
      /* Initialize the MMC/SD state structure */

      memset(priv, 0, sizeof(struct mmcsd_state_s));

      /* Bind the MMCSD driver to the MMCSD state structure */

      priv->dev = dev;

      /* Initialize the hardware associated with the slot */

      ret = mmcsd_hwinitialize(priv);

      /* Was the slot initialized successfully? */

      if (ret != OK)
        {
          /* No... But the error ENODEV is returned if hardware initialization
           * succeeded but no card is inserted in the slot. In this case, the
           * no error occurred, but the driver is still not ready.
           */

          if (ret == -ENODEV)
            {
              fdbg("MMC/SD slot %d is empty\n", slotno);
            }
          else
            {
              fdbg("ERROR: Failed to initialize MMC/SD slot %d: %d\n",
                   slotno, -ret);
              goto errout_with_alloc;
            }
        }

      /* Initialize buffering */

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)

      ret = rwb_initialize(&priv->rwbuffer);
      if (ret < 0)
        {
          fdbg("ERROR: Buffer setup failed: %d\n", -ret);
          goto errout_with_hwinit;
        }
#endif

      /* Create a MMCSD device name */

      snprintf(devname, 16, "/dev/mmcsd%d", minor);

      /* Inode private data is a reference to the MMCSD state structure */

      ret = register_blockdriver(devname, &g_bops, 0, priv);
      if (ret < 0)
        {
          fdbg("ERROR: register_blockdriver failed: %d\n", -ret);
          goto errout_with_buffers;
        }
    }
  return OK;

errout_with_buffers:
#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)
  rwb_uninitialize(&priv->rwbuffer);
#endif
errout_with_hwinit:
  mmcsd_hwuninitialize(priv);
errout_with_alloc:
  free(priv);
  return ret;
}
