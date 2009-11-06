/****************************************************************************
 * drivers/mmcsd/mmcsd.c
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
#include <nuttx/rwbuffer.h>
#include <nuttx/mmcsd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_CREFS 0xff             /* Because a ubyte is used.  Use a larger
                                    * type if necessary */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is contains the unique state of the MMC/SD block driver */

struct mmcsd_state_s
{
  struct mmcsd_dev_s *dev;         /* The MMCSD device bound to this instance */
  ubyte  crefs;                    /* Open references on the driver */

  /* Status flags */

  ubyte widebus:1;                 /* TRUE: Wide 4-bit bus selected */
  ubyte mediachange:1;             /* TRUE: Media changed since last check */
#ifdef CONFIG_MMCSD_DMA
  ubyte dma:1;                     /* TRUE: hardware supports DMA */
#endif

  /* Read-ahead and write buffering support */

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)
  struct rwbuffer_s rwbuffer;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
#  warning "Not implemented"
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
#  warning "Not implemented"
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

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  if (geometry)
    {
      priv = (struct mmcsd_state_s *)inode->i_private;
#warning "Not implemented"
      return -ENOSYS;
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: mmcsd_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcsd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  struct mmcsd_state_s *priv ;

  fvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct mmcsd_state_s *)inode->i_private;

#warning "Not implemented"
  return -ENOTTY;
}

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: mmcsd_hwinitialize
 *
 * Description: One-time hardware initialization
 *
 ****************************************************************************/

static int mmcsd_hwinitialize(struct mmcsd_state_s *priv)
{
#warning "Not implemented"
  return -ENODEV;
}

/****************************************************************************
 * Name: mmcsd_hwinitialize
 *
 * Description: Restore the MMC/SD slot to the uninitialized state
 *
 ****************************************************************************/

static inline void mmcsd_hwuninitialize(struct mmcsd_state_s *priv)
{
  MMCSD_RESET(priv->dev);
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

int mmcsd_slotinitialize(int minor, int slotno, FAR struct mmcsd_dev_s *dev)
{
  struct mmcsd_state_s *priv;
  char devname[16];
  int ret = -ENOMEM;

  fvdbg("minor: %d slotno: %d\n", minor, slotno);

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !dev)
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

      if (ret != 0)
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
