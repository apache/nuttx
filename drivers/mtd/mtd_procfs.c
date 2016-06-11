/****************************************************************************
 * drivers/mtd/mtd_procfs.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mtd/mtd.h>

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_MTD) && defined(CONFIG_FS_PROCFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure describes one open "file" */

struct mtd_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */
  FAR struct mtd_dev_s  *pnextmtd;   /* Pointer to next registered MTD */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* File system methods */

static int     mtd_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     mtd_close(FAR struct file *filep);
static ssize_t mtd_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     mtd_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     mtd_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations mtd_procfsoperations =
{
  mtd_open,       /* open */
  mtd_close,      /* close */
  mtd_read,       /* read */
  NULL,           /* write */

  mtd_dup,        /* dup */

  NULL,           /* opendir */
  NULL,           /* closedir */
  NULL,           /* readdir */
  NULL,           /* rewinddir */

  mtd_stat        /* stat */
};

/* MTD registration variables */

static struct mtd_dev_s *g_pfirstmtd = NULL;
static uint8_t g_nextmtdno = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_open
 ****************************************************************************/

static int mtd_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct mtd_file_s *attr;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate a context structure */

  attr = (FAR struct mtd_file_s *)kmm_zalloc(sizeof(struct mtd_file_s));
  if (!attr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  attr->pnextmtd = g_pfirstmtd;

  /* Save the context as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)attr;
  return OK;
}

/****************************************************************************
 * Name: mtd_close
 ****************************************************************************/

static int mtd_close(FAR struct file *filep)
{
  FAR struct mtd_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = (FAR struct mtd_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: mtd_read
 ****************************************************************************/

static ssize_t mtd_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct mtd_file_s *priv;
  ssize_t total = 0;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct mtd_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* If we are at the end of the list, then return 0 signifying the
   * end-of-file.  This also handles the special case when there are
   * no registered MTD devices.
   */

  if (priv->pnextmtd)
    {
      /* Output a header before the first entry */

      if (priv->pnextmtd == g_pfirstmtd)
        {
          total = snprintf(buffer, buflen, "Num  Device\n");
        }

      /* The provide the requested data */

      do
        {
          ret = snprintf(&buffer[total], buflen - total, "%-5d%s\n",
                         priv->pnextmtd->mtdno, priv->pnextmtd->name);

          if (ret + total < buflen)
            {
              total += ret;
              priv->pnextmtd = priv->pnextmtd->pnext;
            }
          else
            {
              buffer[total] = '\0';
              break;
            }
        }
      while (priv->pnextmtd);
    }

  /* Update the file offset */

  if (total > 0)
    {
      filep->f_pos += total;
    }

  return total;
}

/****************************************************************************
 * Name: mtd_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int mtd_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct mtd_file_s *oldattr;
  FAR struct mtd_file_s *newattr;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct mtd_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = (FAR struct mtd_file_s *)kmm_zalloc(sizeof(struct mtd_file_s));
  if (!newattr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribtes from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct mtd_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: mtd_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int mtd_stat(const char *relpath, struct stat *buf)
{
  /* File/directory size, access block size */

  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int mtd_register(FAR struct mtd_dev_s *mtd, FAR const char *name)
{
  FAR struct mtd_dev_s *plast;

  /* Assign the MTD number and device name */

  mtd->mtdno = g_nextmtdno++;
  mtd->name = name;
  mtd->pnext = NULL;

  /* Add to the list of registered devices */

  if (g_pfirstmtd == NULL)
    {
      g_pfirstmtd = mtd;
    }
  else
    {
      /* Insert at end of list */

      plast = g_pfirstmtd;
      while (plast->pnext)
        {
          /* Skip to next entry as long as there is one */

          plast = plast->pnext;
        }

      /* Now insert at this location */

      plast->pnext = mtd;
    }

  return OK;
}

/****************************************************************************
 * Name: mtd_unregister
 *
 * Description:
 *   Un-Registers an MTD device with the procfs file system.
 *
 * In an embedded system, this all is really unnecessary, but is provided
 * in the procfs system simply for information purposes (if desired).
 *
 ****************************************************************************/

int mtd_unregister(FAR struct mtd_dev_s *mtd)
{
  FAR struct mtd_dev_s *plast;

  /* Remove the MTD from the list of registered devices */

  if (g_pfirstmtd == mtd)
    {
      g_pfirstmtd = mtd->pnext;
    }
  else
    {
      /* Remove from middle of list */

      plast = g_pfirstmtd;
      while (plast->pnext != mtd)
        {
          /* Skip to next entry as long as there is one */

          plast = plast->pnext;
        }

      /* Now remove at this location */

      plast->pnext = mtd->pnext;
    }

  return OK;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
