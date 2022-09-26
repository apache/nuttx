/****************************************************************************
 * arch/arm/src/stm32/stm32_bbsram.c
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

/* This will driver create a set of files in the STM32's Battery backed up
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
#include <nuttx/fs/fs.h>
#include <nuttx/crc32.h>

#include "stm32_bbsram.h"
#include "chip.h"
#include "stm32_pwr.h"
#include "stm32_rtc.h"

#ifdef CONFIG_STM32_BBSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_STM32_BKPSRAM)
#error Driver Requires CONFIG_STM32_BKPSRAM to be enabled
#endif

#define MAX_OPENCNT           (255) /* Limit of uint8_t */

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_BBSRAM_DEBUG
#endif

#if defined(CONFIG_BBSRAM_DEBUG)
#  define BBSRAM_DEBUG_READ() stm32_bbsram_rd()
#  define BBSRAM_DUMP(p,s)    stm32_bbsram_dump(p,s)
#else
#  define BBSRAM_DEBUG_READ()
#  define BBSRAM_DUMP(p,s)
#endif

#define BBSRAM_HEADER_SIZE    (sizeof(struct bbsramfh_s))
#define BBSRAM_CRCED_OFFSET   (sizeof(((struct bbsramfh_s *)0)->crc))
#define BBSRAM_CRCED_SIZE(l)  (BBSRAM_HEADER_SIZE-(BBSRAM_CRCED_OFFSET)+(l))
#define BBSRAM_ALIGNMENT      (sizeof(((struct bbsramfh_s *)0)->crc))
#define BBSRAM_ALIGNMENT_MASK (BBSRAM_ALIGNMENT-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* File Header */

struct bbsramfh_s
{
  uint32_t crc;              /* CRC calculated over data and this struct
                             * starting at fileno */
  uint8_t fileno;            /* The minor number */
  uint8_t dirty;             /* Data has been written to the file */
  uint16_t len;              /* Total Bytes in this file */
  struct timespec lastwrite; /* Last write time */
  uint8_t  data[];           /* Data in the file */
};

struct stm32_bbsram_s
{
  sem_t    exclsem;          /* For atomic accesses to this structure */
  uint8_t  refs;             /* Number of references */
  struct bbsramfh_s *bbf;    /* File in bbram */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     stm32_bbsram_open(struct file *filep);
static int     stm32_bbsram_close(struct file *filep);
static off_t   stm32_bbsram_seek(struct file *filep, off_t offset,
                 int whence);
static ssize_t stm32_bbsram_read(struct file *filep, char *buffer,
                 size_t len);
static ssize_t stm32_bbsram_write(struct file *filep,
                 const char *buffer, size_t len);
static int stm32_bbsram_ioctl(struct file *filep, int cmd,
                 unsigned long arg);
static int     stm32_bbsram_poll(struct file *filep,
                 struct pollfd *fds, bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     stm32_bbsram_unlink(struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_BBSRAM_DEBUG)
static uint8_t debug[STM32_BBSRAM_SIZE];
#endif

static const struct file_operations stm32_bbsram_fops =
{
  .open   = stm32_bbsram_open,
  .close  = stm32_bbsram_close,
  .read   = stm32_bbsram_read,
  .write  = stm32_bbsram_write,
  .seek   = stm32_bbsram_seek,
  .ioctl  = stm32_bbsram_ioctl,
  .poll   = stm32_bbsram_poll,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = stm32_bbsram_unlink
#endif
};

static struct stm32_bbsram_s g_bbsram[CONFIG_STM32_BBSRAM_FILES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bbsram_rd
 ****************************************************************************/

#if defined(CONFIG_BBSRAM_DEBUG)
static void stm32_bbsram_rd(void)
{
  memcpy(&debug, (uint8_t *)STM32_BKPSRAM_BASE, sizeof debug);
}
#endif

/****************************************************************************
 * Name: stm32_bbsram_rd
 ****************************************************************************/

#if defined(CONFIG_BBSRAM_DEBUG)
static void stm32_bbsram_dump(struct bbsramfh_s *bbf, char *op)
{
  BBSRAM_DEBUG_READ();
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
 * Name: stm32_bbsram_semgive
 ****************************************************************************/

static void stm32_bbsram_semgive(struct stm32_bbsram_s *priv)
{
  nxsem_post(&priv->exclsem);
}

/****************************************************************************
 * Name: stm32_bbsram_semtake
 *
 * Description:
 *   Take a semaphore handling any exceptional conditions
 *
 * Input Parameters:
 *   priv - A reference to the CAN peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static int stm32_bbsram_semtake(struct stm32_bbsram_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->exclsem);
}

/****************************************************************************
 * Name: stm32_bbsram_ulock
 *
 * Description:
 *   Unprotects RTC registers, RTC backup data registers and backup SRAM
 *   against parasitic write access
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_bbsram_unlock(void)
{
  stm32_pwr_enablebkp(true);
}

/****************************************************************************
 * Name: stm32_bbsram_lock
 *
 * Description:
 *   Protects RTC registers, RTC backup data registers and backup SRAM
 *   against parasitic write access
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_bbsram_lock(void)
{
  stm32_pwr_enablebkp(false);
}

/****************************************************************************
 * Name: stm32_bbsram_crc
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

static uint32_t stm32_bbsram_crc(struct bbsramfh_s *pf)
{
  return crc32((uint8_t *)pf + BBSRAM_CRCED_OFFSET,
               BBSRAM_CRCED_SIZE(pf->len));
}

/****************************************************************************
 * Name: stm32_bbsram_open
 *
 * Description: Open the device
 *
 ****************************************************************************/

static int stm32_bbsram_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  /* Increment the reference count */

  ret = stm32_bbsram_semtake(bbr);
  if (ret < 0)
    {
      return ret;
    }

  if (bbr->refs == MAX_OPENCNT)
    {
      return -EMFILE;
    }
  else
    {
      bbr->refs++;
    }

  stm32_bbsram_semgive(bbr);
  return OK;
}

/****************************************************************************
 * Name: stm32_bbsram_internal_close
 *
 * Description:
 *    Close BBSRAM entry; Recalculate the time and crc
 *
 ****************************************************************************/

static int stm32_bbsram_internal_close(struct bbsramfh_s *bbf)
{
  bbf->dirty = 0;
  clock_gettime(CLOCK_REALTIME, &bbf->lastwrite);
  bbf->crc = stm32_bbsram_crc(bbf);

  BBSRAM_DUMP(bbf, "close done");
  return bbf->len;
}

/****************************************************************************
 * Name: stm32_bbsram_close
 *
 * Description: close the device
 *
 ****************************************************************************/

static int stm32_bbsram_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  ret = stm32_bbsram_semtake(bbr);
  if (ret < 0)
    {
      return ret;
    }

  BBSRAM_DUMP(bbr->bbf, "close");

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

              stm32_bbsram_unlock();
              stm32_bbsram_internal_close(bbr->bbf);
              stm32_bbsram_lock();
            }
        }
    }

  stm32_bbsram_semgive(bbr);
  return ret;
}

/****************************************************************************
 * Name: stm32_bbsram_seek
 ****************************************************************************/

static off_t stm32_bbsram_seek(struct file *filep, off_t offset,
                               int whence)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  off_t newpos;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  ret = stm32_bbsram_semtake(bbr);
  if (ret < 0)
    {
      return (off_t)ret;
    }

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

      stm32_bbsram_semgive(bbr);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *   end of the existing data in the file. If data is later written at this
   *   point, subsequent reads of data in the gap shall return bytes with the
   *   value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second. But return -EINVAL
   * if "...the resulting file offset would be negative for a regular file,
   *     block special file, or directory."
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

  stm32_bbsram_semgive(bbr);
  return ret;
}

/****************************************************************************
 * Name: stm32_bbsram_read
 ****************************************************************************/

static ssize_t stm32_bbsram_read(struct file *filep, char *buffer,
                                 size_t len)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  ret = stm32_bbsram_semtake(bbr);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Trim len if read would go beyond end of device */

  if ((filep->f_pos + len) > bbr->bbf->len)
    {
      len = bbr->bbf->len - filep->f_pos;
    }

  memcpy(buffer, &bbr->bbf->data[filep->f_pos], len);
  filep->f_pos += len;
  stm32_bbsram_semgive(bbr);
  return len;
}

/****************************************************************************
 * Name: stm32_bbsram_internal_write
 ****************************************************************************/

static ssize_t stm32_bbsram_internal_write(struct bbsramfh_s *bbf,
                                           const char *buffer,
                                           off_t offset, size_t len)
{
  bbf->dirty = 1;
  memcpy(&bbf->data[offset], buffer, len);
  return len;
}

/****************************************************************************
 * Name: stm32_bbsram_write
 ****************************************************************************/

static ssize_t stm32_bbsram_write(struct file *filep,
                                  const char *buffer, size_t len)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  int ret = -EFBIG;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  /* Forbid writes past the end of the device */

  if (filep->f_pos <  bbr->bbf->len)
    {
      /* Clamp len to avoid crossing the end of the memory */

      if ((filep->f_pos + len) > bbr->bbf->len)
        {
          len = bbr->bbf->len - filep->f_pos;
        }

      ret = stm32_bbsram_semtake(bbr);
      if (ret < 0)
        {
          return (ssize_t)ret;
        }

      ret = len; /* save number of bytes written */

      BBSRAM_DUMP(bbr->bbf, "write");
      stm32_bbsram_unlock();
      stm32_bbsram_internal_write(bbr->bbf, buffer, filep->f_pos, len);
      stm32_bbsram_lock();
      filep->f_pos += len;
      BBSRAM_DUMP(bbr->bbf, "write done");
      stm32_bbsram_semgive(bbr);
    }

  BBSRAM_DEBUG_READ();
  return ret;
}

/****************************************************************************
 * Name: stm32_bbsram_poll
 ****************************************************************************/

static int stm32_bbsram_poll(struct file *filep, struct pollfd *fds,
                             bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_bbsram_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int stm32_bbsram_ioctl(struct file *filep, int cmd,
                              unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct stm32_bbsram_s *bbr;
  int ret = -ENOTTY;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  if (cmd == STM32_BBSRAM_GETDESC_IOCTL)
    {
      struct bbsramd_s *bbrr = (struct bbsramd_s *)((uintptr_t)arg);

      ret = stm32_bbsram_semtake(bbr);
      if (ret < 0)
        {
          return ret;
        }

      if (!bbrr)
        {
          ret = -EINVAL;
        }
      else
        {
          bbrr->fileno = bbr->bbf->fileno;
          bbrr->lastwrite = bbr->bbf->lastwrite;
          bbrr->len = bbr->bbf->len;
          bbrr->flags = ((bbr->bbf->crc == stm32_bbsram_crc(bbr->bbf))
                          ? BBSRAM_CRC_VALID : 0);
          bbrr->flags |= ((bbr->bbf->dirty) ? BBSRAM_DIRTY : 0);
          ret = OK;
        }

      stm32_bbsram_semgive(bbr);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_bbsram_unlink
 *
 * Description:
 *  This function will remove the remove the file from the file system
 *  it will zero the contents and time stamp. It will leave the fileno
 *  and pointer to the BBSRAM intact.
 *  It should be called called on the file used for the crash dump
 *  to remove it from visibility in the file system after it is created or
 *  read thus arming it.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int stm32_bbsram_unlink(struct inode *inode)
{
  struct stm32_bbsram_s *bbr;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  bbr = (struct stm32_bbsram_s *)inode->i_private;

  ret = stm32_bbsram_semtake(bbr);
  if (ret < 0)
    {
      return ret;
    }

  stm32_bbsram_unlock();
  memset(bbr->bbf->data, 0, bbr->bbf->len);
  bbr->bbf->lastwrite.tv_nsec = 0;
  bbr->bbf->lastwrite.tv_sec = 0;
  bbr->bbf->crc = stm32_bbsram_crc(bbr->bbf);
  stm32_bbsram_lock();
  bbr->refs  = 0;
  stm32_bbsram_semgive(bbr);
  nxsem_destroy(&bbr->exclsem);

  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_bbsram_probe
 *
 * Description: Based on the number of files defined and their sizes
 * Initializes the base pointers to the file entries.
 *
 ****************************************************************************/

static int stm32_bbsram_probe(int *ent, struct stm32_bbsram_s pdev[])
{
  int i;
  int avail = STM32_BBSRAM_SIZE;
  int alloc;
  int size;
  int ret = -EFBIG;
  struct bbsramfh_s *pf = (struct bbsramfh_s *) STM32_BKPSRAM_BASE;

  for (i = 0; (i < CONFIG_STM32_BBSRAM_FILES) && ent[i] && (avail > 0); i++)
    {
      /* Validate the actual allocations against what is in the BBSRAM */

      size = ent[i];

      /* Use all that is left */

      if (size == -1)
        {
          size = avail - (BBSRAM_HEADER_SIZE + BBSRAM_ALIGNMENT_MASK);
        }

      /* Add in header size and keep aligned */

      alloc = size + BBSRAM_HEADER_SIZE + BBSRAM_ALIGNMENT_MASK;
      alloc &= ~(BBSRAM_ALIGNMENT_MASK);

      /* Does it fit? */

      if (alloc <= avail)
        {
          ret = i + 1;
          BBSRAM_DUMP(pf, "probe");

          if (pf->len != size ||
              pf->fileno != i ||
              pf->crc != stm32_bbsram_crc(pf))
            {
              /* Not Valid so wipe the file in BBSRAM */

              memset((uint8_t *)pf, 0, alloc);
              pf->fileno = i;
              pf->len = size;
              pf->crc = stm32_bbsram_crc(pf);
              BBSRAM_DUMP(pf, "probe reset");
            }

          pdev[i].bbf = pf;
          pf = (struct bbsramfh_s *)((uint8_t *)pf + alloc);
          nxsem_init(&g_bbsram[i].exclsem, 0, 1);
        }

      avail -= alloc;
    }

  BBSRAM_DEBUG_READ();
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: stm32_bbsraminitialize
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
 * If the length of sizes is greater then CONFIG_STM32_BBSRAM_FILES
 * CONFIG_STM32_BBSRAM_FILES will be returned.
 *
 * Returned Value:
 *   Number of files created on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_bbsraminitialize(char *devpath, int *sizes)
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

  memset(g_bbsram, 0, sizeof(g_bbsram));

  /* Clocking for the PWR block must be provided.  However, this is done
   * unconditionally in stm32f40xxx_rcc.c on power up.  This done
   * unconditionally because the PWR block is also needed to set the
   * internal voltage regulator for maximum performance.
   */

  /* Enable backup SRAM clock is done in rcc_enableahb1() when
   * CONFIG_STM32_BKPSRAM is defined.
   */

  /* Allow Access */

  stm32_bbsram_unlock();

  /* Enable backup regulator so that the data is retained in Standby and
   * VBAT modes
   */

  stm32_pwr_enablebreg(true);

  fcnt = stm32_bbsram_probe(sizes, g_bbsram);

  for (i = 0; i < fcnt && ret >= OK; i++)
    {
      snprintf(devname, sizeof(devname), "%s%d", devpath, i);
      ret = register_driver(devname, &stm32_bbsram_fops, 0666, &g_bbsram[i]);
    }

  /* Disallow Access */

  stm32_bbsram_lock();
  return ret < OK ? ret : fcnt;
}

/****************************************************************************
 * Function: stm32_bbsram_savepanic
 *
 * Description:
 *   Saves the panic context in a previously allocated BBSRAM file
 *
 * Input Parameters:
 *   fileno  - the value returned by the ioctl STM32_BBSRAM_GETDESC_IOCTL
 *   context - Pointer to a any array of bytes to save
 *   length  - The length of the data pointed to byt context
 *
 * Returned Value:
 *   Length saved or negated errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_SAVE_CRASHDUMP)
int stm32_bbsram_savepanic(int fileno, uint8_t *context, int length)
{
  struct bbsramfh_s *bbf;
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

      DEBUGASSERT(fileno > 0 && fileno < CONFIG_STM32_BBSRAM_FILES);

      bbf = g_bbsram[fileno].bbf;

      DEBUGASSERT(bbf);

      /* If the g_bbsram has been nulled out we return ENXIO.
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

          stm32_bbsram_unlock();

          stm32_bbsram_internal_write(bbf, (char *) context, 0, length);

          /* Fill with 0 if data is less then file size */

          fill = (int) bbf->len - length;

          if (fill > 0)
            {
              memset(&bbf->data[length], 0, fill);
            }

          /* Seal the file */

          stm32_bbsram_internal_close(bbf);

          stm32_bbsram_lock();
          ret = length;
        }
    }

  return ret;
}
#endif

#endif /* CONFIG_BBSRAM_DRIVER */
