/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_sbram.c
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

/* This will driver create a set of files in the RX65N's Battery backed up
 * SRAM. That can be used to store data retained across power cycles.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/crc32.h>

#include "rx65n_sbram.h"
#include "chip.h"

#ifdef CONFIG_RX65N_SBRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_OPENCNT           (255) /* Limit of uint8_t */

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_SBRAM_DEBUG
#endif

#if defined(CONFIG_SBRAM_DEBUG)
#  define SBRAM_DEBUG_READ() rx65n_sbram_rd()
#  define SBRAM_DUMP(p,s)    rx65n_sbram_dump(p,s)
#else
#  define SBRAM_DEBUG_READ()
#  define SBRAM_DUMP(p,s)
#endif

#define SBRAM_HEADER_SIZE    (sizeof(struct sbramfh_s))
#define SBRAM_CRCED_OFFSET   (sizeof(((struct sbramfh_s *)0)->crc))
#define SBRAM_CRCED_SIZE(l)  (SBRAM_HEADER_SIZE-(SBRAM_CRCED_OFFSET)+(l))
#define SBRAM_ALIGNMENT      (sizeof(((struct sbramfh_s *)0)->crc))
#define SBRAM_ALIGNMENT_MASK (SBRAM_ALIGNMENT-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* File Header */

struct sbramfh_s
{
  uint32_t crc;                /* CRC calculated over data and this struct
                                * starting at fileno */
  uint8_t fileno;              /* The minor number */
  uint8_t dirty;               /* Data has been written to the file */
  uint16_t len;                /* Total Bytes in this file */
  struct timespec lastwrite;   /* Last write time */
  uint8_t  data[];             /* Data in the file */
};

struct rx65n_sbram_s
{
  mutex_t  lock;               /* For atomic accesses to this structure */
  uint8_t  refs;               /* Number of references */
  FAR struct sbramfh_s *bbf;   /* File in bbram */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rx65n_sbram_open(FAR struct file *filep);
static int     rx65n_sbram_close(FAR struct file *filep);
static off_t   rx65n_sbram_seek(FAR struct file *filep, off_t offset,
                 int whence);
static ssize_t rx65n_sbram_read(FAR struct file *filep, FAR char *buffer,
                 size_t len);
static ssize_t rx65n_sbram_write(FAR struct file *filep,
                 FAR const char *buffer, size_t len);
static int rx65n_sbram_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     rx65n_sbram_poll(FAR struct file *filep,
                                FAR struct pollfd *fds,
                                                                 bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     rx65n_sbram_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_SBRAM_DEBUG)
static uint8_t debug[RX65N_SBRAM_SIZE];
#endif

static const struct file_operations rx65n_sbram_fops =
{
  .open   = rx65n_sbram_open,
  .close  = rx65n_sbram_close,
  .read   = rx65n_sbram_read,
  .write  = rx65n_sbram_write,
  .seek   = rx65n_sbram_seek,
  .ioctl  = rx65n_sbram_ioctl,
  .poll   = rx65n_sbram_poll,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = rx65n_sbram_unlink
#endif
};

static struct rx65n_sbram_s g_sbram[CONFIG_RX65N_SBRAM_FILES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_sbram_rd
 ****************************************************************************/

#if defined(CONFIG_SBRAM_DEBUG)
static void rx65n_sbram_rd(void)
{
  memcpy(&debug, (uint8_t *)RX65N_SBRAM_BASE, sizeof debug);
}
#endif

/****************************************************************************
 * Name: rx65n_sbram_rd
 ****************************************************************************/

#if defined(CONFIG_SBRAM_DEBUG)
static void rx65n_sbram_dump(FAR struct sbramfh_s *bbf, char *op)
{
  SBRAM_DEBUG_READ();
  _info("%s:\n", op);
  _info(" File Address:0x%8x\n", bbf);
  _info("  crc:0x%8x\n", bbf->crc);
  _info("  fileno:%d\n", (int) bbf->fileno);
  _info("  dirty:%d\n", (int) bbf->dirty);
  _info("  length:%d\n", (int) bbf->len);
  _info("  time:%ld:%ld\n", bbf->lastwrite.tv_sec, bbf->lastwrite.tv_nsec);
  _info("  data: 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x\n",
  bbf->data[0], bbf->data[1], bbf->data[2], bbf->data[3], bbf->data[4]);
}
#endif

/****************************************************************************
 * Name: rx65n_sbram_crc
 *
 * Description:
 *   Calculates the CRC of the block
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static uint32_t rx65n_sbram_crc(FAR struct sbramfh_s *pf)
{
  return crc32((uint8_t *)pf + SBRAM_CRCED_OFFSET,
    SBRAM_CRCED_SIZE(pf->len));
}

/****************************************************************************
 * Name: rx65n_sbram_open
 *
 * Description: Open the device
 *
 ****************************************************************************/

static int rx65n_sbram_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  /* Increment the reference count */

  nxmutex_lock(&bbr->lock);
  if (bbr->refs == MAX_OPENCNT)
    {
      return -EMFILE;
    }
  else
    {
      bbr->refs++;
    }

  nxmutex_unlock(&bbr->lock);
  return OK;
}

/****************************************************************************
 * Name: rx65n_sbram_internal_close
 *
 * Description:
 *    Close SBRAM entry; Recalculate the time and crc
 *
 ****************************************************************************/

static int rx65n_sbram_internal_close(FAR struct sbramfh_s *bbf)
{
  bbf->dirty = 0;
  clock_gettime(CLOCK_REALTIME, &bbf->lastwrite);
  bbf->crc = rx65n_sbram_crc(bbf);

  SBRAM_DUMP(bbf, "close done");
  return bbf->len;
}

/****************************************************************************
 * Name: rx65n_sbram_close
 *
 * Description: close the device
 *
 ****************************************************************************/

static int rx65n_sbram_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  nxmutex_lock(&bbr->lock);

  SBRAM_DUMP(bbr->bbf, "close");

  if (bbr->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      bbr->refs--;

      if (bbr->refs == 0)
        {
          if (bbr->bbf->dirty)
            {
              /* Recalculate the time and crc */

              rx65n_sbram_internal_close(bbr->bbf);
            }
        }
    }

  nxmutex_unlock(&bbr->lock);
  return ret;
}

/****************************************************************************
 * Name: rx65n_sbram_seek
 ****************************************************************************/

static off_t rx65n_sbram_seek(FAR struct file *filep, off_t offset,
                               int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;
  off_t newpos;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  nxmutex_lock(&bbr->lock);

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = bbr->bbf->len + offset;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      nxmutex_unlock(&bbr->lock);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set
   *   beyond the end of the existing data in the file. If data is
   *   later written at this point,
   *   subsequent reads of data in the gap shall return bytes with the value
   *   0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second.
   * But return -EINVAL if
   *
   *  "...the resulting file offset would be negative for a regular
   *  file, block
   *   special file, or directory."
   */

  if (newpos >= 0)
    {
      filep->f_pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&bbr->lock);
  return ret;
}

/****************************************************************************
 * Name: rx65n_sbram_read
 ****************************************************************************/

static ssize_t rx65n_sbram_read(FAR struct file *filep, FAR char *buffer,
                                 size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  nxmutex_lock(&bbr->lock);

  /* Trim len if read would go beyond end of device */

  if ((filep->f_pos + len) > bbr->bbf->len)
    {
      len = bbr->bbf->len - filep->f_pos;
    }

  memcpy(buffer, &bbr->bbf->data[filep->f_pos], len);
  filep->f_pos += len;
  nxmutex_unlock(&bbr->lock);
  return len;
}

/****************************************************************************
 * Name: rx65n_sbram_internal_write
 ****************************************************************************/

static ssize_t rx65n_sbram_internal_write(FAR struct sbramfh_s *bbf,
                                           FAR const char *buffer,
                                           off_t offset, size_t len)
{
  bbf->dirty = 1;
  memcpy(&bbf->data[offset], buffer, len);
  return len;
}

/****************************************************************************
 * Name: rx65n_sbram_write
 ****************************************************************************/

static ssize_t rx65n_sbram_write(FAR struct file *filep,
                                 FAR const char *buffer,
                                  size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;
  int ret = -EFBIG;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  /* Forbid writes past the end of the device */

  if (filep->f_pos <  bbr->bbf->len)
    {
      /* Clamp len to avoid crossing the end of the memory */

      if ((filep->f_pos + len) > bbr->bbf->len)
        {
          len = bbr->bbf->len - filep->f_pos;
        }

      ret = len; /* save number of bytes written */

      nxmutex_lock(&bbr->lock);
      SBRAM_DUMP(bbr->bbf, "write");
      rx65n_sbram_internal_write(bbr->bbf, buffer, filep->f_pos, len);
      filep->f_pos += len;
      SBRAM_DUMP(bbr->bbf, "write done");
      nxmutex_unlock(&bbr->lock);
    }

  SBRAM_DEBUG_READ();
  return ret;
}

/****************************************************************************
 * Name: rx65n_sbram_poll
 ****************************************************************************/

static int rx65n_sbram_poll(FAR struct file *filep, FAR struct pollfd *fds,
                             bool setup)
{
  if (setup)
    {
      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_sbram_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int rx65n_sbram_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct rx65n_sbram_s *bbr;
  int ret = -ENOTTY;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  if (cmd == RX65N_SBRAM_GETDESC_IOCTL)
    {
      FAR struct sbramd_s *bbrr = (FAR struct sbramd_s *)((uintptr_t)arg);

      nxmutex_lock(&bbr->lock);
      if (!bbrr)
        {
          ret = -EINVAL;
        }
      else
        {
          bbrr->fileno = bbr->bbf->fileno;
          bbrr->lastwrite = bbr->bbf->lastwrite;
          bbrr->len = bbr->bbf->len;
          bbrr->flags = ((bbr->bbf->crc == rx65n_sbram_crc(bbr->bbf))
                          ? SBRAM_CRC_VALID : 0);
          bbrr->flags |= ((bbr->bbf->dirty) ? SBRAM_DIRTY : 0);
          ret = OK;
        }

      nxmutex_unlock(&bbr->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_sbram_unlink
 *
 * Description:
 *  This function will remove the remove the file from the file system
 *  it will zero the contents and time stamp. It will leave the fileno
 *  and pointer to the SBRAM intact.
 *  It should be called called on the file used for the crash dump
 *  to remove it from visibility in the file system after it is created or
 *  read thus arming it.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rx65n_sbram_unlink(FAR struct inode *inode)
{
  FAR struct rx65n_sbram_s *bbr;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (FAR struct rx65n_sbram_s *)inode->i_private;

  nxmutex_lock(&bbr->lock);
  memset(bbr->bbf->data, 0, bbr->bbf->len);
  bbr->bbf->lastwrite.tv_nsec = 0;
  bbr->bbf->lastwrite.tv_sec = 0;
  bbr->bbf->crc = rx65n_sbram_crc(bbr->bbf);
  bbr->refs  = 0;
  nxmutex_unlock(&bbr->lock);
  nxmutex_destroy(&bbr->lock);
  return 0;
}
#endif

/****************************************************************************
 * Name: rx65n_sbram_probe
 *
 * Description: Based on the number of files defined and their sizes
 * Initializes the base pointers to the file entries.
 *
 ****************************************************************************/

static int rx65n_sbram_probe(int *ent, struct rx65n_sbram_s pdev[])
{
  int i;
  int avail = RX65N_SBRAM_SIZE;
  int alloc;
  int size;
  int ret = -EFBIG;
  struct sbramfh_s *pf = (struct sbramfh_s *) RX65N_SBRAM_BASE;

  for (i = 0; (i < CONFIG_RX65N_SBRAM_FILES) && ent[i] && (avail > 0); i++)
    {
      /* Validate the actual allocations against what is in the SBRAM */

      size = ent[i];

      /* Use all that is left */

      if (size == -1)
        {
          size = avail - (SBRAM_HEADER_SIZE + SBRAM_ALIGNMENT_MASK);
        }

      /* Add in header size and keep aligned */

      alloc = size + SBRAM_HEADER_SIZE + SBRAM_ALIGNMENT_MASK;
      alloc &= ~(SBRAM_ALIGNMENT_MASK);

      /* Does it fit? */

      if (alloc <= avail)
        {
          ret = i + 1;
          SBRAM_DUMP(pf, "probe");
          if (pf->len != size ||
              pf->fileno != i ||
              pf->crc != rx65n_sbram_crc(pf))
            {
              /* Not Valid so wipe the file in SBRAM */

              memset((uint8_t *)pf, 0, alloc);
              pf->fileno = i;
              pf->len = size;
              pf->crc = rx65n_sbram_crc(pf);
              SBRAM_DUMP(pf, "probe reset");
            }

          pdev[i].bbf = pf;
          pf = (struct sbramfh_s *)((uint8_t *)pf + alloc);
          nxmutex_init(&g_sbram[i].lock);
        }

      avail -= alloc;
    }

  SBRAM_DEBUG_READ();
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: rx65n_sbraminitialize
 *
 * Description:
 *   Initialize the Battery Backed up SRAM driver.
 *
 * Input Parameters:
 *   devpath - the path to instantiate the files.
 *   sizes   - Pointer to a any array of file sizes to create
 *             the last entry should be 0
 *             A size of -1 will use all the remaining spaces
 *
 * If the length of sizes is greater then CONFIG_RX65N_SBRAM_FILES
 * CONFIG_RX65N_SBRAM_FILES will be returned.
 *
 * Returned Value:
 *   Number of files created on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int rx65n_sbraminitialize(char *devpath, int *sizes)
{
  int i;
  int fcnt;
  char devname[32];

  int ret = OK;

  if (devpath == NULL)
    {
      return -EINVAL;
    }

  i = strlen(devpath);
  if (i == 0 || i > sizeof(devname) - 3)
    {
      return -EINVAL;
    }

  memset(g_sbram, 0, sizeof(g_sbram));

  /* Clocking for the PWR block must be provided.  However, this is done
   * unconditionally in rx65nf40xxx_rcc.c on power up.  This done
   * unconditionally because the PWR block is also needed to set the
   * internal voltage regulator for maximum performance.
   */

  /* Enable backup SRAM clock is done in rcc_enableahb1() when
   * CONFIG_RX65N_SBRAM is defined.
   */

  /* Allow Access */

  fcnt = rx65n_sbram_probe(sizes, g_sbram);

  for (i = 0; i < fcnt && ret >= OK; i++)
    {
      snprintf(devname, sizeof(devname), "%s%d", devpath, i);
      ret = register_driver(devname, &rx65n_sbram_fops, 0666, &g_sbram[i]);
    }

  /* Disallow Access */

  return ret < OK ? ret : fcnt;
}

/****************************************************************************
 * Function: rx65n_sbram_savepanic
 *
 * Description:
 *   Saves the panic context in a previously allocated SBRAM file
 *
 * Input Parameters:
 *   fileno  - the value returned by the ioctl RX65N_SBRAM_GETDESC_IOCTL
 *   context - Pointer to a any array of bytes to save
 *   length  - The length of the data pointed to byt context
 *
 * Returned Value:
 *   Length saved or negated errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_RX65N_SAVE_CRASHDUMP)
int rx65n_sbram_savepanic(int fileno, uint8_t *context, int length)
{
  FAR struct sbramfh_s *bbf;
  int fill;
  int ret = -ENOSPC;

  /* On a bad day we could panic while panicking, (and we debug assert)
   * this is a potential feeble attempt at only writing the first
   * panic's context to the file
   */

  static bool once = false;

  if (!once)
    {
      once = true;

      DEBUGASSERT(fileno > 0 && fileno < CONFIG_RX65N_SBRAM_FILES);

      bbf = g_sbram[fileno].bbf;

      DEBUGASSERT(bbf);

      /* If the g_sbram has been nulled out we return ENXIO.
       *
       * As once ensures we will keep the first dump. Checking the time for
       * 0 protects from over writing a previous crash dump that has not
       * been saved to long term storage and erased.  The dreaded reboot
       * loop.
       */

      if (!bbf)
        {
          ret = -ENXIO;
        }
      else if ((bbf->lastwrite.tv_sec == 0 && bbf->lastwrite.tv_nsec == 0))
        {
          /* Clamp length if too big  */

          if (length > bbf->len)
            {
              length = bbf->len;
            }

          rx65n_sbram_internal_write(bbf, (char *) context, 0, length);

          /* Fill with 0 if data is less then file size */

          fill = (int) bbf->len - length;

          if (fill > 0)
            {
              memset(&bbf->data[length], 0, fill);
            }

          /* Seal the file */

          rx65n_sbram_internal_close(bbf);

          ret = length;
        }
    }

  return ret;
}
#endif

#endif /* CONFIG_SBRAM_DRIVER */
